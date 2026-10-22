"""Offline packet validation and horizontal head-reference calibration."""
import copy
import json
import math
import unittest

import numpy as np
from lerobot.utils.rotation import Rotation

from vr_heading import decode_head, heading_alignment


STAMP = 1_800_000_000_000_000_000
FRAME = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])


def rotation(axis, degrees):
    return Rotation.from_rotvec(np.array(axis)*math.radians(degrees)).as_matrix()


def frame(yaw=0., pitch=0., roll=0., position=(0., 0., 0.)):
    orientation = (rotation([0, 1, 0], yaw) @ rotation([1, 0, 0], pitch)
                   @ rotation([0, 0, 1], roll))
    head = {'pose': [*position, *Rotation.from_matrix(orientation).as_quat().tolist()], 'status': 3}
    return {'stamp': STAMP, 'received_monotonic': 10., 'head': decode_head(head, STAMP),
            'left': {'grip': 0.},
            'right': {'grip': 0., 'joystick_available': True, 'axis_y': 0., 'axis_click': False}}


class DecodeHeadTests(unittest.TestCase):
    def test_optional_timestamp_binds_to_this_packet_and_known_statuses(self):
        for status in (1, 3):
            head = {'pose': [1, 2, 3, 0, 0, 0, 1], 'status': status}
            decoded = decode_head(head, STAMP)
            self.assertEqual(decoded, {'pose': [1., 2., 3., 0., 0., 0., 1.],
                                       'status': status, 'stamp': STAMP})
            self.assertEqual(decode_head(head, STAMP+1_000_000)['stamp'], STAMP+1_000_000)

    def test_json_head_and_comma_pose_are_supported(self):
        value = {'pose': '1,2,3,0,0,0,1', 'status': 3, 'timeStampNs': STAMP-1_000_000}
        self.assertEqual(decode_head(json.dumps(value), STAMP),
                         {'pose': [1., 2., 3., 0., 0., 0., 1.],
                          'status': 3, 'stamp': STAMP-1_000_000})

    def test_missing_bad_or_untracked_head_is_error_not_exception(self):
        good = {'pose': [0, 0, 0, 0, 0, 0, 1], 'status': 3}
        cases = [None, '', '{bad', '[]', [], 42, {}]
        cases += [{**good, 'status': s} for s in (None, 0, 2, 4, 7, -1, True, '3', 3.)]
        cases += [{**good, 'pose': p} for p in (None, [], [0]*7, [0]*6,
                  '0,0,0,0,0,0,bad', [0, 0, 0, 0, 0, 0, float('nan')],
                  [True, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 10])]
        for value in cases:
            with self.subTest(value=value):
                self.assertIn('error', decode_head(value, STAMP))

    def test_independent_head_timestamp_cannot_be_stale_or_inconsistent(self):
        good = {'pose': [0, 0, 0, 0, 0, 0, 1], 'status': 3}
        for offset in (-250_000_000, 0, 250_000_000):
            self.assertNotIn('error', decode_head({**good, 'timeStampNs': STAMP+offset}, STAMP))
        for stamp in (None, 0, -1, True, '123', float('inf'), STAMP-250_000_001, STAMP+250_000_001):
            with self.subTest(stamp=stamp):
                self.assertIn('error', decode_head({**good, 'timeStampNs': stamp}, STAMP))
        for outer in (None, 0, True, float('nan')):
            self.assertIn('error', decode_head(good, outer))


class HeadingAlignmentTests(unittest.TestCase):
    def test_cardinal_headings_map_new_forward_left_and_up_to_old_convention(self):
        directions = (np.array([0., 0., -1.]), np.array([-1., 0., 0.]), np.array([0., 1., 0.]))
        for yaw in (0., 90., -90., 180.):
            with self.subTest(yaw=yaw):
                alignment, reported = heading_alignment(frame(yaw), 10.1)
                expected_rotation = rotation([0, 1, 0], yaw)
                np.testing.assert_allclose(alignment, expected_rotation.T, atol=1e-10)
                self.assertAlmostEqual((reported-yaw+180.) % 360.-180., 0., places=9)
                for axis, direction in enumerate(directions):
                    np.testing.assert_allclose(FRAME @ alignment @ expected_rotation @ direction,
                                               np.eye(3)[:, axis], atol=1e-10)
                np.testing.assert_allclose(alignment.T @ alignment, np.eye(3), atol=1e-10)
                self.assertAlmostEqual(np.linalg.det(alignment), 1.)

    def test_pitch_roll_translation_and_quaternion_sign_do_not_change_yaw(self):
        expected, _ = heading_alignment(frame(63), 10.1)
        for pitch, roll in ((35, 42), (-50, -65), (0, 90)):
            packet = frame(63, pitch, roll, (123, -456, 789))
            for sign in (1, -1):
                packet['head']['pose'][3:] = [sign*q for q in packet['head']['pose'][3:]]
                actual, yaw = heading_alignment(packet, 10.1)
                np.testing.assert_allclose(actual, expected, atol=1e-10)
                self.assertAlmostEqual(yaw, 63., places=9)

    def test_near_vertical_gaze_cannot_define_heading(self):
        for pitch in (87, 90, -87, -90):
            with self.subTest(pitch=pitch), self.assertRaisesRegex(ValueError, '接近竖直'):
                heading_alignment(frame(42, pitch), 10.1)

    def test_missing_stale_or_future_packet_rejected(self):
        for elapsed in (-.001, .251):
            with self.assertRaisesRegex(ValueError, '0.25 秒'):
                heading_alignment(frame(), 10.+elapsed)
        for key in ('received_monotonic', 'stamp', 'head'):
            packet = frame()
            del packet[key]
            with self.subTest(key=key), self.assertRaises(ValueError):
                heading_alignment(packet, 10.1)
        packet = frame()
        packet['head'] = {'error': '测试头显无效'}
        with self.assertRaisesRegex(ValueError, '测试头显无效'):
            heading_alignment(packet, 10.1)

    def test_decoded_head_still_requires_valid_pose_status_and_matching_stamp(self):
        for changes in ({'status': 0}, {'status': 2}, {'pose': [0]*7},
                        {'stamp': STAMP-300_000_000}, {'stamp': None}):
            packet = frame()
            packet['head'].update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                heading_alignment(packet, 10.1)

    def test_both_grips_and_body_lift_must_be_neutral(self):
        for side in ('left', 'right'):
            packet = frame()
            packet[side]['grip'] = .31
            with self.assertRaisesRegex(ValueError, '松开左右 Grip'):
                heading_alignment(packet, 10.1)
        for changes in ({'axis_y': .21}, {'axis_y': -.21}, {'axis_click': True},
                        {'axis_click': None}, {'joystick_available': False}):
            packet = frame()
            packet['right'].update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                heading_alignment(packet, 10.1)
        packet = frame()
        packet['left'].update(grip=.3, axis_click=True)
        packet['right'].update(grip=.3, axis_y=.2)
        heading_alignment(packet, 10.1)  # Left click is the calibration trigger.

    def test_invalid_input_cannot_pass_neutral_gates_and_packet_is_not_changed(self):
        for side, key, value in (('left', 'grip', None), ('right', 'grip', -1),
                                 ('left', 'grip', False), ('right', 'axis_y', float('nan'))):
            packet = frame()
            packet[side][key] = value
            with self.subTest(side=side, key=key), self.assertRaises(ValueError):
                heading_alignment(packet, 10.1)
        packet = frame(15)
        before = copy.deepcopy(packet)
        heading_alignment(packet, 10.1)
        self.assertEqual(packet, before)

    def chassis_frame(self):
        packet = frame(35.)
        packet['left'].update(joystick_available=True, axis_x=0., axis_y=0., axis_click=True)
        packet['right']['axis_x'] = 0.
        return packet

    def test_optional_chassis_gate_uses_translation_radial_deadband_and_keeps_left_press(self):
        packet = self.chassis_frame()
        packet['left']['axis_x'] = .2
        packet['right'].update(axis_x=.2, axis_y=-.2)
        actual, _ = heading_alignment(packet, 10.1, require_chassis_neutral=True)
        expected, _ = heading_alignment(frame(35.), 10.1)
        np.testing.assert_allclose(actual, expected)
        packet['left'].update(axis_x=.15, axis_y=.15)
        with self.assertRaisesRegex(ValueError, '底盘平移'):
            heading_alignment(packet, 10.1, require_chassis_neutral=True)

    def test_chassis_gate_rejects_unavailable_or_invalid_or_active_axes_on_either_controller(self):
        for side in ('left', 'right'):
            for key, value in (('joystick_available', False), ('joystick_available', None),
                               ('axis_x', None), ('axis_y', float('nan')),
                               ('axis_x', True), ('axis_x', .201), ('axis_y', -.201)):
                with self.subTest(side=side, key=key, value=value):
                    packet = self.chassis_frame()
                    packet[side][key] = value
                    with self.assertRaises(ValueError):
                        heading_alignment(packet, 10.1, require_chassis_neutral=True)

    def test_without_chassis_left_axes_and_right_horizontal_axis_do_not_change_calibration(self):
        packet = frame(25.)
        packet['right']['axis_x'] = .8
        packet['left']['axis_x'] = -.8
        expected, _ = heading_alignment(frame(25.), 10.1)
        actual, _ = heading_alignment(packet, 10.1)
        np.testing.assert_allclose(actual, expected)


if __name__ == '__main__':
    unittest.main()
