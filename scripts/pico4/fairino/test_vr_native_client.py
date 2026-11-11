"""Main native client integration with real IK and fake input/feedback only."""
import contextlib
import copy
import io
import json
import math
from pathlib import Path
import queue
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
import paths
import test_vr_heading_client as heading_fixture
import vr_client
from lerobot.utils.rotation import Rotation
from mounts import tcp_in_chest
from test_tcp import hardware
from test_probe import connected_wrist
from vendor_kinematics import VendorKinematics
from vr_chassis import ChassisTeleop
from vr_native_ik import NativeIK
from apps.fairino_keyboard.axis_session import AxisSession, ZERO
from apps.fairino_keyboard.control_core import check_axes, input_twist


class EndFrames(Exception):
    pass


class GripperRecorder:
    speed = .75

    def __init__(self):
        self.calls = []

    def update(self, frame, hands, active):
        self.calls.append(dict(active=active, hands=dict(hands),
                               triggers={s: frame[s].get('trigger') for s in hands} if frame else {}))

    def snapshot(self):
        return {'devices': {}, 'message': 'offline'}


class NativeClientTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = NativeIK()
        cls.kin = VendorKinematics()

    @staticmethod
    def frame(*, angle=0., primary=False, secondary=False, **values):
        frame = heading_fixture.HeadingClientTests.frame(**values)
        for side in ('left', 'right'):
            frame[side]['pose'][3:] = Rotation.from_rotvec(
                np.array([0., math.radians(angle), 0.])).as_quat().tolist()
            frame[side]['primary'], frame[side]['secondary'] = primary, secondary
            frame[side]['trigger'], frame[side]['trigger_available'] = .4, True
        return frame

    def run_frames(self, frames, *, mode='mirror_same_direction', auto_home=False,
                   auxiliary=False, keys=None, read_delay=0., proof_delay=0., interpolation_delay_ms=0.):
        """No SDK/SSH/ROS/session threads; only the actual input/IK logic runs."""
        clock, packets, gui, goals = [10.], [], [], []
        hw = hardware()
        session = AxisSession('unused', 'unused', None) if auxiliary else None
        grippers = GripperRecorder() if auxiliary else None
        if session:
            session.latest = dict(ready=True, hardware={'enabled': [True]*8, 'velocity': [0.]*8},
                                  command=ZERO, received=clock[0])
        with tempfile.TemporaryDirectory() as directory:
            connection = connected_wrist(NS(events=queue.Queue(), done=threading.Event(),
                last_status=clock[0], latest={'hardware': hw, 'stamp': clock[0], 'ready': True},
                log_path=Path(directory)/'run.jsonl'))

            def send(**values):
                if session:
                    axes = session.command_at(clock[0], session.latest['ready'])
                    session.latest['command'] = input_twist(check_axes(axes), .06, .10)
                    values.update(chassis_command=session.latest['command'], chassis_active=session.active)
                values['guard_enabled'] = connection.wrist.guard().enabled
                packets.append(copy.deepcopy(values))

            def read():
                if len(packets) == len(frames):
                    raise EndFrames()
                clock[0] += read_delay
                frame = copy.deepcopy(frames[len(packets)])
                frame['received_monotonic'] = clock[0]-frame.pop('_test_age')
                connection.latest['ready'] = frame.pop('_test_ready')
                hw['velocity'][:] = frame.pop('_test_velocity')
                if session:
                    session.latest['received'] = clock[0]
                return frame

            def sleep(_):
                clock[0] += .002
                connection.last_status = connection.latest['stamp'] = clock[0]

            original_proof = vr_client.verify_both

            def verify(*values):
                result = original_proof(*values)
                clock[0] += proof_delay
                return result

            def plan(*values):
                goals.append(copy.deepcopy(values[2]))
                return self.native.request(*values)

            connection.send = send
            args = NS(ik_backend=getattr(self, 'backend', 'native'),
                native_ik=NS(request=plan), tron_ik=NS(request=plan), sensitivity=.25,
                execute=True, mode=mode, hand_mapping='auto', freeze_buttons=True,
                no_auto_home=not auto_home, interpolation_delay_ms=interpolation_delay_ms,
                command_hz=500., receiver_hz=500., feedback_hz=100.,
                body_lift=auxiliary, lift_speed_mm_s=15.,
                gui_input=NS(key=lambda: (keys or {}).get(len(packets), '')))
            if auxiliary:
                args.user, args.chassis_linear, args.chassis_angular = 'unused', .06, .10
                args.chassis_control = ChassisTeleop(args, None, lambda *_, **__: session)
                args.gripper_control = grippers
            with contextlib.ExitStack() as stack:
                stack.enter_context(patch('vr_client.time.monotonic', side_effect=lambda: clock[0]))
                stack.enter_context(patch('vr_client.time.sleep', side_effect=sleep))
                stack.enter_context(patch('vr_client.keyboard', contextlib.nullcontext))
                stack.enter_context(patch('vr_client.verify_both', side_effect=verify))
                stack.enter_context(patch('vr_client.request', side_effect=AssertionError('vendor teleop IK used')))
                stack.enter_context(patch('vr_gui_ipc.status', side_effect=lambda **v: gui.append(v)))
                stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                with self.assertRaises(EndFrames):
                    vr_client.operate(args, self.kin, NS(read=read), connection)
            logs = [json.loads(line) for line in connection.log_path.with_suffix(
                '.actions.jsonl').read_text().splitlines()]
        if session:
            self.assertIsNone(session.thread.ident)
        return NS(packets=packets, logs=logs, gui=gui, goals=goals, grippers=grippers)

    @staticmethod
    def tcp(packet, arm='left', requested=False):
        entry = packet['request']['arms'][arm]
        return np.asarray(entry['native_ik']['requested_tcp'] if requested else entry['tcp']).reshape(4, 4)

    @staticmethod
    def events(result, event):
        return [row for row in result.logs if row.get('event') == event]

    def test_native_both_arms_and_single_hand_release_preserve_routing(self):
        for mode, remaining in (('first_person', 'right'), ('mirror_same_direction', 'left')):
            with self.subTest(mode=mode):
                result = self.run_frames([self.frame(), self.frame(left_grip=1., right_grip=1.),
                    self.frame(left_grip=1., right_grip=1., position=(.002, 0., 0.)),
                    self.frame(right_grip=1., position=(.003, 0., 0.)), self.frame()], mode=mode)
                self.assertEqual([set(g) for g in result.goals],
                                 [{'left', 'right'}, {'left', 'right'}, {remaining}])
                for packet in result.packets[1:4]:
                    self.assertEqual(packet['request']['ik_backend'], 'native')
                    for arm in packet['request']['arms'].values():
                        self.assertTrue(arm['native_ik']['success'])
                        self.assertLess(arm['native_ik']['position_error_mm'], .02)
                        self.assertNotIn('wrist_scale', arm)
                self.assertEqual(set(result.packets[3]['request']['arms']), {remaining})
                self.assertIsNone(result.packets[-1]['request'])
                self.assertEqual(result.gui[0]['ik_backend'], 'native')

    def test_500hz_loop_interpolates_tcp_position_and_rotation_before_ik(self):
        frames = [self.frame(), self.frame(right_grip=1.)]
        frames.extend(self.frame(right_grip=1., position=(i*.0002, 0., 0.), angle=i*.2)
                      for i in range(1, 13))
        # An 11 ms test delay puts the sampled target halfway between 2 ms
        # samples, making both interpolation and the actual 500 Hz cadence
        # distinguishable from pass-through/holding the latest controller.
        result = self.run_frames(frames, interpolation_delay_ms=11.)
        timing = self.events(result, 'timing_config')[0]
        self.assertEqual((timing['command_hz'], timing['receiver_hz']), (500., 500.))
        self.assertEqual(timing['horizon_ms'], 0.)
        baseline = result.goals[0]['left']
        self.assertEqual(len(result.goals), 13)
        for index, goals in enumerate(result.goals):
            expected = max(0., index-5.5)*.05
            goal = goals['left']
            self.assertAlmostEqual(np.linalg.norm(goal[:3, 3]-baseline[:3, 3]), expected, places=8)
            angle = math.degrees(np.linalg.norm(Rotation.from_matrix(
                goal[:3, :3] @ baseline[:3, :3].T).as_rotvec()))
            self.assertAlmostEqual(angle, expected, places=8)
        plans = [row for row in result.logs if 'ik_ms' in row]
        for before, after in zip(plans, plans[1:]):
            self.assertAlmostEqual(after['plan_monotonic']-before['plan_monotonic'], .002, places=10)

    def test_native_rotation_freeze_and_release_keep_current_command_without_catchup(self):
        result = self.run_frames([self.frame(), self.frame(right_grip=1.),
            self.frame(right_grip=1., angle=4.), self.frame(right_grip=1., angle=8., primary=True),
            self.frame(right_grip=1., angle=16., primary=True), self.frame(right_grip=1., angle=24.)])
        baseline = self.tcp(result.packets[1], requested=True)
        self.assertEqual(len(result.goals), 5)
        self.assertGreater(np.linalg.norm(result.goals[1]['left'][:3, :3]-baseline[:3, :3]), .01)
        for packet in result.packets[3:]:
            np.testing.assert_allclose(self.tcp(packet, requested=True)[:3, :3], baseline[:3, :3], atol=1e-10)

    def test_native_uses_continuous_clutch_at_half_turn(self):
        result = self.run_frames([self.frame(), self.frame(right_grip=1.),
            self.frame(right_grip=1., angle=170.), self.frame(right_grip=1., angle=179.),
            self.frame(right_grip=1., angle=181.)])
        self.assertEqual(len(result.goals), 4)
        a, b = [g['left'] for g in result.goals[-2:]]
        degrees = math.degrees(np.linalg.norm(Rotation.from_matrix(b[:3, :3] @ a[:3, :3].T).as_rotvec()))
        self.assertAlmostEqual(degrees, .5, places=7)

    def test_heading_calibration_remains_on_main_path_and_regrip_has_no_jump(self):
        result = self.run_frames([self.frame(), self.frame(click=True), self.frame(),
            self.frame(left_grip=1., position=(.3, .1, .2)),
            self.frame(left_grip=1., position=(.2996, .1, .2))])
        self.assertEqual(len(self.events(result, 'heading_calibrated')), 1)
        self.assertTrue(any(g['heading_calibrated'] for g in result.gui))
        start, end = [tcp_in_chest('right', g['right']) for g in result.goals]
        np.testing.assert_allclose(end[:3, 3]-start[:3, 3], [-.1, 0., 0.], atol=1e-8)
        for packet in result.packets[3:]:
            self.assertEqual(packet['request']['ik_backend'], 'native')

    def test_real_home_generates_joint_requests_without_native_ik(self):
        result = self.run_frames([self.frame(), self.frame(), self.frame()], auto_home=True)
        self.assertEqual(result.goals, [])
        self.assertEqual(len(self.events(result, 'home_started')), 1)
        for packet in result.packets:
            self.assertEqual(packet['request']['source'], 'home')
            self.assertEqual(set(packet['request']['arms']), {'left', 'right'})

    def test_lift_only_mixed_motion_and_pause_preserve_auxiliary_gates(self):
        result = self.run_frames([self.frame(), self.frame(left_y=1., right_y=.5),
            self.frame(left_grip=1., right_grip=1., left_y=1., right_y=.6),
            self.frame(left_grip=1., right_grip=1., left_y=1., right_y=.6), self.frame(),
            self.frame(left_y=1., right_y=.5)], auxiliary=True, keys={3: ' '})
        self.assertEqual(result.packets[1]['request']['arms'], {})
        self.assertEqual(result.packets[1]['request']['lift'], .5)
        self.assertEqual(set(result.packets[2]['request']['arms']), {'left', 'right'})
        self.assertEqual(result.packets[2]['request']['lift'], .6)
        self.assertEqual(len(result.goals), 1)
        for index in (1, 2, 5):
            self.assertEqual(result.packets[index]['chassis_command'], (-.06, 0., 0.))
            self.assertTrue(result.grippers.calls[index]['active'])
            self.assertEqual(result.grippers.calls[index]['hands'], {'left': 'right', 'right': 'left'})
            self.assertEqual(result.grippers.calls[index]['triggers'], {'left': .4, 'right': .4})
        self.assertIsNone(result.packets[3]['request'])
        self.assertEqual(result.packets[3]['chassis_command'], ZERO)
        self.assertFalse(result.grippers.calls[3]['active'])
        self.assertEqual(result.packets[4]['chassis_command'], ZERO)

    def test_guard_off_is_rejected_and_regrip_keeps_native_guard_enabled(self):
        result = self.run_frames([self.frame(), self.frame(right_grip=1.),
            self.frame(right_grip=1.), self.frame(right_grip=1.), self.frame(),
            self.frame(right_grip=1.)], keys={2: 'G'})
        self.assertTrue(all(packet['guard_enabled'] for packet in result.packets))
        self.assertIsNotNone(result.packets[1]['request'])
        self.assertTrue(all(result.packets[i]['request'] is None for i in (2, 3, 4)))
        self.assertIsNotNone(result.packets[5]['request'])
        self.assertTrue(any('必须保持开启' in e['reason'] for e in self.events(result, 'input_paused')))

    def test_receive_after_cycle_start_has_nonnegative_input_and_sampling_ages(self):
        result = self.run_frames([self.frame(), self.frame(right_grip=1.),
            self.frame(right_grip=1., position=(.002, 0., 0.))], read_delay=.003)
        plans = [row for row in result.logs if 'ik_ms' in row]
        self.assertEqual(len(plans), 2)
        self.assertFalse(self.events(result, 'input_paused'))
        for row in plans:
            self.assertEqual(row['pico_pc_sample_age_ms'], 0.)
            self.assertGreaterEqual(row['robot_status_age_ms'], 0.)
            for sample in row['cartesian_sampling'].values():
                self.assertGreaterEqual(sample['position_age_ms'], 0.)
                self.assertGreaterEqual(sample['rotation_age_ms'], 0.)

    def test_slow_startup_verification_expires_input_and_requires_release(self):
        result = self.run_frames([self.frame(), self.frame(right_grip=1.), self.frame(),
                                  self.frame(right_grip=1.)], proof_delay=.3)
        self.assertTrue(any('0.25 秒' in e['reason'] for e in self.events(result, 'input_paused')))
        self.assertTrue(all(result.packets[i]['request'] is None for i in (0, 1, 2)))
        self.assertIsNotNone(result.packets[3]['request'])
        self.assertEqual(len(result.goals), 1)


if __name__ == '__main__':
    unittest.main()
