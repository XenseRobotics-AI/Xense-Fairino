"""Offline end-to-end checks of the physical left-stick heading binding."""
import contextlib
import copy
import io
import json
import math
import queue
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import paths
from lerobot.utils.rotation import Rotation
from mounts import tcp_in_chest
from test_tcp import hardware
from test_probe import connected_wrist
from vendor_kinematics import VendorKinematics
from vr_client import operate
from vr_input import decode_packet
from vr_chassis import ChassisTeleop
from apps.fairino_keyboard.axis_session import AxisSession, ZERO
from apps.fairino_keyboard.control_core import check_axes, input_twist


class EndFrames(Exception):
    pass


class HeadingClientTests(unittest.TestCase):
    @staticmethod
    def frame(*, yaw=90., left_grip=0., right_grip=0., click=False,
              right_click=False, right_y=0., left_x=0., left_y=0., right_x=0., position=(0., 0., 0.),
              head=True, age=0., samples=None, ready=True, velocity=None):
        controllers = {}
        for side, grip in (("left", left_grip), ("right", right_grip)):
            controllers[side] = {
                "pose": [*position, 0., 0., 0., 1.], "grip": grip,
                "axisX": right_x if side == 'right' else left_x,
                "axisY": right_y if side == 'right' else left_y,
                "axisClick": click if side == 'left' else right_click,
                "primaryButton": False, "secondaryButton": False,
            }
        raw = {'timeStampNs': 1_000_000_000, 'Controller': controllers}
        if head is True:
            raw['Head'] = {'pose': [0., 1.6, 0., *Rotation.from_rotvec(
                np.array([0., math.radians(yaw), 0.])).as_quat()], 'status': 3}
        elif head is not False:
            raw['Head'] = head
        frame = decode_packet(json.dumps({'value': raw}))
        frame['_test_age'] = age
        frame['_test_ready'] = ready
        frame['_test_velocity'] = velocity or [0.]*20
        if samples is not None:
            frame['samples'] = samples
        return frame

    def run_frames(self, frames, *, mode='mirror', auto_home=False,
                   body_lift=False, keys=None, home_type=None,
                   chassis_feedback=None, grippers=None):
        """Run real input/IK/planning against fresh, stationary fake feedback."""
        clock, packets, gui = [10.], [], []
        hw = hardware()
        session = AxisSession('unused', 'unused', None) if chassis_feedback is not None else None
        if session:
            session.latest = dict(ready=True, hardware={'enabled': [True]*8, 'velocity': [0.]*8},
                                  command=ZERO, received=clock[0])
        with tempfile.TemporaryDirectory() as directory:
            connection = connected_wrist(NS(events=queue.Queue(), done=threading.Event(),
                last_status=clock[0], latest={'hardware': hw, 'stamp': clock[0],
                'ready': True}, log_path=Path(directory)/'run.jsonl'))
            def send(**values):
                if session:
                    axes = session.command_at(clock[0], session.latest['ready'])
                    command = input_twist(check_axes(axes), .06, .10)
                    session.latest['command'] = command
                    values.update(chassis_command=command, chassis_active=session.active,
                                  chassis_armed=session.armed)
                packets.append(copy.deepcopy(values))
            def read():
                if len(packets) == len(frames):
                    raise EndFrames()
                frame = copy.deepcopy(frames[len(packets)])
                frame['received_monotonic'] = clock[0]-frame.pop('_test_age')
                connection.latest['ready'] = frame.pop('_test_ready')
                hw['velocity'][:] = frame.pop('_test_velocity')
                if session:
                    session.latest['received'] = clock[0]
                    session.latest.update(chassis_feedback[len(packets)])
                if clock[0]-frame['received_monotonic'] > .25:
                    # Real Stream.read rejects stale data before button decoding.
                    raise ValueError('双手柄数据超过 0.25 秒未更新')
                if 'samples' in frame:
                    for index, sample in enumerate(frame['samples']):
                        sample['received_monotonic'] = clock[0]-.002*(len(frame['samples'])-index)
                    # Real Stream.read includes the newest packet in its batch.
                    frame['samples'].append({k: copy.deepcopy(v) for k, v in frame.items()
                                             if k != 'samples'})
                return frame
            def sleep(_):
                clock[0] += .02
                connection.last_status = clock[0]
                connection.latest['stamp'] = clock[0]
            gui_input = NS(key=lambda: (keys or {}).get(len(packets), ''))
            connection.send = send
            args = NS(sensitivity=.25, execute=True, mode=mode, hand_mapping='auto',
                freeze_buttons=True, no_auto_home=not auto_home,
                interpolation_delay_ms=0., body_lift=body_lift,
                lift_speed_mm_s=15., gui_input=gui_input)
            if session:
                args.user = 'unused'
                args.chassis_linear, args.chassis_angular = .06, .10
                args.chassis_control = ChassisTeleop(args, None, lambda *_, **__: session)
            if grippers:
                args.gripper_control = grippers
            with contextlib.ExitStack() as stack:
                stack.enter_context(patch('vr_client.time.monotonic', side_effect=lambda: clock[0]))
                stack.enter_context(patch('vr_client.time.sleep', side_effect=sleep))
                stack.enter_context(patch('vr_client.keyboard', contextlib.nullcontext))
                stack.enter_context(patch('vr_gui_ipc.status', side_effect=lambda **v: gui.append(v)))
                stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                if home_type is not None:
                    stack.enter_context(patch('vr_client.HomeMotion', home_type))
                with self.assertRaises(EndFrames):
                    operate(args, VendorKinematics(), NS(read=read), connection)
            logs = [json.loads(line) for line in connection.log_path.with_suffix(
                '.actions.jsonl').read_text().splitlines()]
        return packets, logs, gui

    @staticmethod
    def events(logs, event):
        return [row for row in logs if row.get('event') == event]

    @staticmethod
    def tcp(packet, arm='right'):
        return np.array(packet['request']['arms'][arm]['tcp']).reshape(4, 4)

    def test_left_press_calibrates_once_and_reports_gui_heading(self):
        frames = [self.frame(), self.frame(click=True), self.frame(click=True, yaw=0.),
                  self.frame(click=True, yaw=-90.), self.frame()]
        packets, logs, gui = self.run_frames(frames)
        events = self.events(logs, 'heading_calibrated')
        self.assertEqual(len(events), 1)
        self.assertAlmostEqual(events[0]['yaw_deg'], 90.)
        self.assertTrue(all(packet['request'] is None for packet in packets))
        calibrated = [state for state in gui if state.get('heading_calibrated')]
        self.assertTrue(calibrated)
        self.assertAlmostEqual(calibrated[-1]['heading_yaw_deg'], 90.)
        self.assertTrue(calibrated[-1]['heading_message'])

    def test_initial_held_button_requires_release_and_new_press(self):
        _, logs, _ = self.run_frames([self.frame(click=True), self.frame(click=True),
                                     self.frame(), self.frame(click=True)])
        self.assertEqual(len(self.events(logs, 'heading_calibrated')), 1)

    def test_grip_rejection_is_not_deferred_until_grip_release(self):
        frames = [self.frame(), self.frame(left_grip=1.),
                  self.frame(click=True, left_grip=1.), self.frame(click=True),
                  self.frame(click=True), self.frame(), self.frame(left_grip=1.),
                  self.frame(left_grip=1., position=(0., 0., -.0004))]
        packets, logs, _ = self.run_frames(frames)
        self.assertEqual(len(self.events(logs, 'heading_calibration_rejected')), 1)
        self.assertFalse(self.events(logs, 'heading_calibrated'))
        for packet in packets[2:6]:
            self.assertIsNone(packet['request'])
        delta = tcp_in_chest('right', self.tcp(packets[-1]))[:3, 3] - tcp_in_chest(
            'right', self.tcp(packets[-2]))[:3, 3]
        np.testing.assert_allclose(delta, [.1, 0., 0.], atol=1e-8)

    def test_calibration_drops_old_samples_and_regrip_has_no_jump(self):
        # Simulate Stream.read batching packets that still contain an earlier
        # engaged Grip and a large motion made before the calibration press.
        old_samples = [self.frame(left_grip=1.),
                       self.frame(left_grip=1., position=(.1, .2, .3))]
        frames = [self.frame(), self.frame(left_grip=1.),
                  self.frame(click=True, samples=old_samples), self.frame(),
                  self.frame(left_grip=1., position=(.8, -.3, .2))]
        packets, logs, _ = self.run_frames(frames)
        self.assertEqual(len(self.events(logs, 'heading_calibrated')), 1)
        self.assertIsNone(packets[2]['request'])
        self.assertIsNone(packets[3]['request'])
        np.testing.assert_allclose(self.tcp(packets[-1]), self.tcp(packets[1]), atol=1e-8)

    def test_heading_is_fixed_when_user_turns_head_after_calibration(self):
        frames = [self.frame(), self.frame(click=True), self.frame(),
                  self.frame(left_grip=1., yaw=0.),
                  self.frame(left_grip=1., yaw=-90., position=(-.0004, 0., 0.))]
        packets, logs, _ = self.run_frames(frames)
        delta = tcp_in_chest('right', self.tcp(packets[-1]))[:3, 3] - tcp_in_chest(
            'right', self.tcp(packets[-2]))[:3, 3]
        np.testing.assert_allclose(delta, [.1, 0., 0.], atol=1e-8)
        self.assertEqual(len(self.events(logs, 'heading_calibrated')), 1)

    def test_regrip_cannot_resume_before_a_post_calibration_release_frame(self):
        frames = [self.frame(), self.frame(click=True), self.frame(left_grip=1.),
                  self.frame(left_grip=1.), self.frame(), self.frame(left_grip=1.)]
        packets, _, _ = self.run_frames(frames)
        for packet in packets[1:5]:
            self.assertIsNone(packet['request'])
        self.assertIsNotNone(packets[-1]['request'])

    def test_calibrated_forward_and_sideways_preserve_modes_and_routing(self):
        modes = {'first_person': ('left', (1., 1.)),
                 'mirror': ('right', (1., -1.)),
                 'mirror_same_direction': ('right', (-1., -1.))}
        for mode, (arm, signs) in modes.items():
            for axis, position in enumerate(((-.0004, 0., 0.), (0., 0., .0004))):
                with self.subTest(mode=mode, axis=axis):
                    frames = [self.frame(), self.frame(click=True), self.frame(),
                              self.frame(left_grip=1.),
                              self.frame(left_grip=1., position=position)]
                    packets, _, _ = self.run_frames(frames, mode=mode)
                    self.assertEqual(set(packets[-1]['request']['arms']), {arm})
                    start, end = (tcp_in_chest(arm, self.tcp(packet, arm)) for packet in packets[-2:])
                    expected = np.zeros(3); expected[axis] = .1*signs[axis]
                    np.testing.assert_allclose(end[:3, 3]-start[:3, 3], expected, atol=1e-8)

    def test_heading_also_rotates_orientation_axes_with_mirror_parity(self):
        for mode, arm, sign in (('first_person', 'left', 1.),
                                ('mirror', 'right', -1.),
                                ('mirror_same_direction', 'right', -1.)):
            with self.subTest(mode=mode):
                rotated = self.frame(left_grip=1.)
                # After a +90-degree heading calibration, tracking -X is the
                # operator's forward axis. Mirror rotation is an axial vector.
                rotated['left']['pose'][3:] = Rotation.from_rotvec(
                    np.array([-.0004, 0., 0.])).as_quat().tolist()
                packets, _, _ = self.run_frames([self.frame(), self.frame(click=True),
                    self.frame(), self.frame(left_grip=1.), rotated], mode=mode)
                start, end = (tcp_in_chest(arm, self.tcp(packet, arm)) for packet in packets[-2:])
                np.testing.assert_allclose(end[:3, :3] @ start[:3, :3].T,
                    Rotation.from_rotvec(np.array([sign*.0001, 0., 0.])).as_matrix(), atol=1e-8)
                np.testing.assert_allclose(end[:3, 3], start[:3, 3], atol=1e-8)

    def test_bad_or_missing_head_does_not_stop_uncalibrated_teleop(self):
        for head in (False, {'pose': [0.]*7, 'status': 3}, {'pose': 'bad', 'status': 0}):
            with self.subTest(head=head):
                packets, logs, _ = self.run_frames([self.frame(head=head),
                    self.frame(head=head, left_grip=1.),
                    self.frame(head=head, left_grip=1., position=(0., 0., -.0004))])
                self.assertIsNotNone(packets[-1]['request'])
                self.assertFalse(self.events(logs, 'heading_calibration_rejected'))

    def test_invalid_input_rejects_calibration_and_sends_hold(self):
        cases = [dict(head=False), dict(head={'pose': [0.]*7, 'status': 3}),
                 dict(right_y=.21), dict(right_click=True),
                 dict(right_grip=.31), dict(ready=False),
                 dict(velocity=[0.]*19+[.005])]
        for case in cases:
            with self.subTest(case=case):
                packets, logs, _ = self.run_frames([self.frame(), self.frame(click=True, **case)])
                self.assertIsNone(packets[-1]['request'])
                self.assertFalse(self.events(logs, 'heading_calibrated'))
                rejected = self.events(logs, 'heading_calibration_rejected')
                self.assertEqual(len(rejected), 1)
                self.assertTrue(rejected[0]['reason'])

    def test_stale_click_stops_input_and_is_not_replayed(self):
        packets, logs, _ = self.run_frames([self.frame(), self.frame(left_grip=1.),
            self.frame(click=True, age=.3), self.frame(click=True), self.frame(),
            self.frame(click=True)])
        for packet in packets[2:]:
            self.assertIsNone(packet['request'])
        self.assertEqual(len(self.events(logs, 'heading_calibrated')), 1)
        self.assertFalse(self.events(logs, 'heading_calibration_rejected'))
        self.assertTrue(any('未更新' in event['reason'] for event in self.events(logs, 'input_paused')))

    def test_heading_does_not_rotate_chassis_axes_in_any_mode(self):
        for mode, expected in [('first_person', (.06, 0., 0.)),
                               ('mirror', (.06, 0., 0.)),
                               ('mirror_same_direction', (-.06, 0., 0.))]:
            with self.subTest(mode=mode):
                packets, logs, _ = self.run_frames([self.frame(), self.frame(click=True),
                    self.frame(), self.frame(left_y=1.)], mode=mode, chassis_feedback=[{}]*4)
                self.assertEqual(len(self.events(logs, 'heading_calibrated')), 1)
                self.assertFalse(packets[1]['chassis_active'])
                self.assertEqual(packets[1]['chassis_command'], ZERO)
                self.assertTrue(packets[2]['chassis_armed'])
                self.assertEqual(packets[3]['chassis_command'], expected)

    def test_calibration_rejects_moving_or_stale_chassis_feedback(self):
        cases = [dict(received=9.), dict(ready=False), dict(command=(.01, 0., 0.)),
                 dict(hardware={'enabled': [True]*8, 'velocity': [.05]+[0.]*7})]
        for state in cases:
            with self.subTest(state=state):
                packets, logs, _ = self.run_frames([self.frame(), self.frame(click=True)],
                                                  chassis_feedback=[{}, state])
                self.assertFalse(self.events(logs, 'heading_calibrated'))
                self.assertIn('底盘', self.events(logs, 'heading_calibration_rejected')[0]['reason'])
                self.assertIsNone(packets[-1]['request'])
                self.assertFalse(packets[-1]['chassis_active'])
                self.assertEqual(packets[-1]['chassis_command'], ZERO)

    def test_rejected_calibration_holds_all_auxiliary_devices_until_neutral(self):
        class Grippers:
            speed = .75
            def __init__(self): self.active = []
            def update(self, frame, hands, active): self.active.append(active)
            def snapshot(self): return {'devices': {}, 'message': 'offline'}
        grippers = Grippers()
        frames = [self.frame(), self.frame(left_y=1., right_y=.5),
                  self.frame(click=True, left_y=1., right_y=.5),
                  self.frame(left_y=1., right_y=.5), self.frame(),
                  self.frame(left_y=1., right_y=.5)]
        packets, logs, _ = self.run_frames(frames, body_lift=True, grippers=grippers,
                                          chassis_feedback=[{}]*len(frames))
        self.assertEqual(packets[1]['request']['lift'], .5)
        self.assertTrue(grippers.active[1])
        self.assertFalse(self.events(logs, 'heading_calibrated'))
        self.assertEqual(len(self.events(logs, 'heading_calibration_rejected')), 1)
        for index in (2, 3):
            self.assertIsNone(packets[index]['request'])
            self.assertEqual(packets[index]['chassis_command'], ZERO)
            self.assertFalse(packets[index]['chassis_active'])
            self.assertFalse(grippers.active[index])
        self.assertEqual(packets[4]['chassis_command'], ZERO)
        self.assertEqual(packets[5]['request']['lift'], .5)
        self.assertEqual(packets[5]['chassis_command'], (.06, 0., 0.))
        self.assertTrue(grippers.active[5])

    def test_failed_recalibration_preserves_previous_alignment(self):
        packets, logs, gui = self.run_frames([self.frame(), self.frame(click=True),
            self.frame(), self.frame(click=True, head=False), self.frame(),
            self.frame(left_grip=1.), self.frame(left_grip=1., position=(-.0004, 0., 0.))])
        self.assertEqual(len(self.events(logs, 'heading_calibrated')), 1)
        self.assertEqual(len(self.events(logs, 'heading_calibration_rejected')), 1)
        self.assertTrue(gui[-1]['heading_calibrated'])
        self.assertAlmostEqual(gui[-1]['heading_yaw_deg'], 90.)
        self.assertIn('未执行', gui[-1]['heading_message'])
        start, end = (tcp_in_chest('right', self.tcp(p)) for p in packets[-2:])
        np.testing.assert_allclose(end[:3, 3]-start[:3, 3], [.1, 0., 0.], atol=1e-8)

    def test_pending_home_is_cancelled_and_calibration_rejected(self):
        packets, logs, _ = self.run_frames([
            self.frame(ready=False), self.frame(click=True)], auto_home=True)
        self.assertIsNone(packets[-1]['request'])
        self.assertFalse(self.events(logs, 'heading_calibrated'))
        self.assertIn('Home', self.events(logs, 'heading_calibration_rejected')[0]['reason'])

    def test_running_home_is_cancelled_and_calibration_rejected(self):
        class Home:
            duration = 5.
            def __init__(self, *_): pass
            def complete(self, *_): return False
            def request(self, *_): return {'id': 1, 'arms': {}}
        packets, logs, _ = self.run_frames([self.frame(), self.frame(click=True)],
            auto_home=True, home_type=Home)
        self.assertIsNotNone(packets[0]['request'])
        self.assertIsNone(packets[-1]['request'])
        self.assertFalse(self.events(logs, 'heading_calibrated'))
        self.assertIn('Home', self.events(logs, 'heading_calibration_rejected')[0]['reason'])

    def test_lift_input_is_stopped_on_rejected_calibration(self):
        frames = [self.frame(), self.frame(), self.frame(right_y=.5),
                  self.frame(right_y=.5, click=True), self.frame(right_y=.5)]
        packets, logs, _ = self.run_frames(frames, body_lift=True)
        self.assertEqual(packets[2]['request']['lift'], .5)
        self.assertIsNone(packets[3]['request'])
        self.assertIsNone(packets[4]['request'])
        self.assertEqual(len(self.events(logs, 'heading_calibration_rejected')), 1)

    def test_space_pause_still_discards_targets_and_requires_release(self):
        frames = [self.frame(), self.frame(left_grip=1.),
                  self.frame(left_grip=1.), self.frame(left_grip=1.),
                  self.frame(), self.frame(left_grip=1.)]
        packets, logs, _ = self.run_frames(frames, keys={2: ' '})
        self.assertIsNotNone(packets[1]['request'])
        for packet in packets[2:5]:
            self.assertIsNone(packet['request'])
        self.assertIsNotNone(packets[-1]['request'])
        self.assertFalse(self.events(logs, 'heading_calibrated'))


if __name__ == '__main__':
    unittest.main()
