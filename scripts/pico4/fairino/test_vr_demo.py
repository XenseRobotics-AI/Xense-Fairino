"""Model-only PICO acceptance tests: real C++ IK, synthetic PICO packets."""
import io
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from vr_demo import DemoCommands, DemoEngine, ModelKinematics, main, wrist_state
from vr_demo_ik import DemoBoxIK
import numpy as np
from lerobot.utils.rotation import Rotation
from tcp_core import ARMS, BOUNDS_DEG, pose_error
from vr_input import mapped_pose, POSE_FRAMES
from mounts import BASE_FROM_TELEOP
from tool_frames import tool_config
from vr_native_ik import NativeIK
from wrist_guard import WristGuard


def hand(grip=0., position=(0., 0., 0.), angle=0., **values):
    return dict(pose=[*position, *Rotation.from_rotvec([0., math.radians(angle), 0.]).as_quat().tolist()],
                grip=grip, primary=False, secondary=False, axis_x=0., axis_y=0., axis_click=False,
                joystick_available=True, trigger=0., trigger_available=True, **values)


def frame(now, **hands):
    return dict(stamp=now*1e9, received_monotonic=now,
                left=hands.get('left', hand()), right=hands.get('right', hand()),
                head=dict(status=3, stamp=now*1e9, pose=[0., 0., 0., 0., 0., 0., 1.]))


class DemoTests(unittest.TestCase):
    def engine(self, **kw):
        return DemoEngine(interpolation_delay_ms=0., **kw)

    def engage(self, engine, arm='left', now=10.):
        engine.step(frame(now), now)
        controller = engine.hands[arm]
        engine.step(frame(now+.01, **{controller: hand(grip=1.)}), now+.01)
        return controller

    def test_initial_pose_is_selected_home_and_calibrated_body_high(self):
        for preset in ('default', 'right_mirrored', 'left_cross_body'):
            engine = self.engine(home_preset=preset)
            state = engine.snapshot(10.)
            self.assertTrue(state['demo'])
            self.assertEqual(len(state['hardware']['position']), 20)
            self.assertAlmostEqual(state['body']['height'], 100.)
            self.assertFalse(any(state['wrist_collision'].values()))
            for side, joints in ARMS.items():
                self.assertEqual([engine.q[j-1] for j in joints], engine.home['target'][side])
            json.dumps(state, allow_nan=False)

    def test_all_modes_both_arms_axes_and_sensitivities_reach_mapped_tcp(self):
        cases = 0
        for mode in ('first_person', 'mirror', 'mirror_same_direction'):
            for arm in ARMS:
                for axis in range(3):
                    for sensitivity in (.25, .5, 1.):
                        with self.subTest(mode=mode, arm=arm, axis=axis, sensitivity=sensitivity):
                            engine = self.engine(mode=mode, sensitivity=sensitivity)
                            control = self.engage(engine, arm)
                            initial = engine.q[:]
                            goal = np.asarray(engine.kin.forward_tcp([initial[j-1] for j in ARMS[arm]], arm)[0])
                            position = [0., 0., 0.]
                            position[axis] = .01
                            moving = hand(grip=1., position=position)
                            shift = (mapped_pose(moving['pose'], arm, mode)[0]-
                                     mapped_pose(hand()['pose'], arm, mode)[0])*sensitivity
                            goal[:3, 3] += shift
                            state = engine.step(frame(10.02, **{control: moving}), 10.02)
                            self.assertFalse(state['paused'], state['message'])
                            actual = engine.kin.forward_tcp([engine.q[j-1] for j in ARMS[arm]], arm)[0]
                            mm, deg = pose_error(actual, goal)
                            self.assertLessEqual(mm, .02)
                            self.assertLessEqual(deg, .01)
                            other = 'right' if arm == 'left' else 'left'
                            self.assertEqual([engine.q[j-1] for j in ARMS[other]], [initial[j-1] for j in ARMS[other]])
                            cases += 1
        self.assertEqual(cases, 54)

    def test_held_startup_never_moves_until_release_and_regrip(self):
        engine = self.engine()
        q = engine.q[:]
        for i in range(30):
            t = 10.+i*.01
            engine.step(frame(t, right=hand(grip=1., position=(i*.01, 0., 0.))), t)
        self.assertEqual(engine.q, q)
        self.assertTrue(engine.paused)
        engine.step(frame(10.3), 10.3)
        engine.step(frame(10.31, right=hand(grip=1., position=(1., 0., 0.))), 10.31)
        self.assertEqual(engine.q, q)

    def test_stationary_controller_and_release_have_no_motion_tail(self):
        engine = self.engine()
        control = self.engage(engine)
        moving = hand(grip=1., position=(.02, 0., 0.))
        engine.step(frame(10.02, **{control: moving}), 10.02)
        q = engine.q[:]
        for i in range(1, 31):
            t = 10.02+.01*i
            engine.step(frame(t, **{control: moving}), t)
        self.assertEqual(engine.q, q)
        engine.step(frame(10.33), 10.33)
        for i in range(10):
            t = 10.34+i*.01
            engine.step(frame(t, right=hand(position=(1., 0., 0.))), t)
        self.assertEqual(engine.q, q)

    def test_stale_and_disconnected_input_hold_then_require_release(self):
        for failure in ('stale', 'missing', 'loopgap'):
            engine = self.engine()
            control = self.engage(engine)
            q = engine.q[:]
            t = 10.5 if failure == 'loopgap' else 10.02
            broken = None if failure == 'missing' else frame(9. if failure == 'stale' else t)
            state = engine.step(broken, t)
            self.assertTrue(state['paused'])
            engine.step(frame(t+.01, **{control: hand(grip=1., position=(1., 0., 0.))}), t+.01)
            self.assertEqual(engine.q, q)
            self.assertTrue(engine.paused)
            engine.step(frame(t+.02), t+.02)
            engine.step(frame(t+.03, **{control: hand(grip=1., position=(1., 0., 0.))}), t+.03)
            self.assertEqual(engine.q, q)

    def test_pause_sensitivity_stop_reset_anchors(self):
        engine = self.engine()
        self.engage(engine)
        q = engine.q[:]
        for command, sensitivity in (('faster', .5), ('slower', .25), ('pause', .25)):
            state = engine.step(frame(10.02), 10.02, command)
            self.assertTrue(state['paused'])
            self.assertEqual(state['sensitivity'], sensitivity)
            self.assertFalse(any(c.engaged for c in engine.clutches.values()))
        engine.command('stop')
        engine.step(frame(10.03), 10.03)
        self.assertTrue(engine.stopped)
        self.assertEqual(engine.q, q)

    def test_freeze_position_suppresses_translation_and_unfreeze_no_catchup(self):
        engine = self.engine(freeze_buttons=True)
        control = self.engage(engine)
        q = engine.q[:]
        frozen = hand(grip=1., position=(.1, 0., 0.)); frozen['secondary'] = True
        engine.step(frame(10.02, **{control: frozen}), 10.02)
        self.assertEqual(engine.q, q)
        frozen['secondary'] = False
        engine.step(frame(10.03, **{control: frozen}), 10.03)
        self.assertEqual(engine.q, q)

    def test_heading_calibration_reuses_shared_neutral_and_release_gates(self):
        engine = self.engine()
        engine.step(frame(10.), 10.)
        calibrate = frame(10.01)
        calibrate['left']['axis_click'] = True
        calibrate['head']['pose'][3:] = Rotation.from_rotvec([0., math.pi/2, 0.]).as_quat().tolist()
        state = engine.step(calibrate, 10.01)
        self.assertTrue(state['heading_calibrated'])
        self.assertAlmostEqual(state['heading_yaw_deg'], 90.)
        self.assertTrue(engine.paused)
        calibrate['received_monotonic'] = 10.02
        engine.step(calibrate, 10.02)
        self.assertTrue(engine.paused)
        engine.step(frame(10.03), 10.03)
        self.assertFalse(engine.paused)

    def test_home_returns_only_arms_and_grip_cancels(self):
        engine = self.engine()
        control = self.engage(engine)
        engine.step(frame(10.02, **{control: hand(grip=1., position=(.02, 0., 0.))}), 10.02)
        body = engine.q[14:]
        engine.command('home')
        for i in range(500):
            t = 10.03+i*.01
            engine.step(frame(t), t)
        self.assertEqual(engine.q[:7], engine.home['target']['left'])
        self.assertEqual(engine.q[14:], body)
        engine.command('home')
        engine.step(frame(15.03), 15.03)
        state = engine.step(frame(15.04, left=hand(grip=1.)), 15.04)
        self.assertIsNone(engine.home_motion)
        self.assertIn('中止', state['message'])

    def test_body_preset_and_pause_only_change_folding_axes(self):
        engine = self.engine(body_lift=True)
        initial = engine.q[:]
        engine.step(frame(10.), 10.)
        engine.step(frame(10.01), 10.01)
        for i in range(130):
            t = 10.02+i*.01
            value = frame(t); value['right']['axis_y'] = -1.
            state = engine.step(value, t)
            self.assertFalse(state['paused'], state['message'])
        self.assertTrue(engine.lift.moving)
        self.assertLess(state['body']['height'], 100.)
        self.assertEqual(engine.q[:17], initial[:17])
        engine.command('pause')
        q = engine.q[:]
        for i in range(20):
            t = 11.32+i*.01
            value = frame(t); value['left']['grip'] = 1.
            engine.step(value, t)
        self.assertEqual(engine.q, q)

    def test_grippers_use_routed_triggers_caps_and_rearm_after_pause(self):
        engine = self.engine(grippers=True, max_openings={'left': .8, 'right': .6})
        engine.step(frame(10.), 10.)
        engine.step(frame(10.01), 10.01)
        press = frame(10.02); press['right']['trigger'] = .5
        engine.step(press, 10.02)
        self.assertEqual(engine.grippers, {'left': .4, 'right': 1.})
        engine.command('pause')
        engine.step(frame(10.03), 10.03)
        press['received_monotonic'] = 10.04
        engine.step(press, 10.04)
        self.assertEqual(engine.grippers['left'], .4)
        engine.step(frame(10.05), 10.05)
        press['received_monotonic'] = 10.06; press['right']['trigger'] = 1.
        engine.step(press, 10.06)
        self.assertEqual(engine.grippers['left'], 0.)

    def test_demo_guard_off_actually_allows_wrist_envelope_violation(self):
        engine = self.engine()
        engine.q[5:7] = [math.radians(65.), math.radians(30.)]
        hw = engine.hardware()
        desired = engine.q[:7]; desired[-1] = math.radians(65.)
        goal = np.asarray(engine.kin.forward_tcp(desired, 'left')[0])
        protected = engine.native.request(engine.kin, hw, {'left': goal}, 1, 10.)
        engine._apply(protected)
        self.assertFalse(wrist_state(engine.q)['wrist_collision']['left'])
        engine.q = hw['position'][:]
        unprotected = DemoBoxIK(engine.kin.tools).request(engine.kin, hw, {'left': goal}, 1, 10.)
        engine._apply(unprotected)
        self.assertTrue(wrist_state(engine.q)['wrist_collision']['left'])
        self.assertTrue(unprotected['arms']['left']['native_ik']['success'])
        for q, bounds in zip(engine.q[:7], BOUNDS_DEG):
            self.assertGreaterEqual(math.degrees(q), bounds[0]-1e-9)
            self.assertLessEqual(math.degrees(q), bounds[1]+1e-9)
        engine.command('guard_off')
        engine.command('guard_on')
        self.assertFalse(engine.guard.enabled)
        self.assertIn('先回 Home', engine.message)
        engine.command('home')
        for i in range(1800):
            t = 10.+i*.01
            engine.step(frame(t), t)
        self.assertFalse(wrist_state(engine.q)['wrist_collision']['left'])
        engine.command('guard_on')
        self.assertTrue(engine.guard.enabled)
        # The physical NativeIK still rejects opt-out, with no code changes.
        with self.assertRaisesRegex(ValueError, 'remain enabled'):
            NativeIK().request(engine.kin, engine.hardware(), {'left': goal}, 1, 30., guard=WristGuard(False))

    def test_wrist_display_distinguishes_protection_margin_from_raw_envelope(self):
        engine = self.engine()
        engine.q[5:7] = [math.radians(69.), 0.]
        state = wrist_state(engine.q)
        self.assertTrue(state['wrist_guard_violation']['left'])
        self.assertFalse(state['wrist_collision']['left'])
        engine.q[6] = math.radians(50.)
        self.assertTrue(wrist_state(engine.q)['wrist_collision']['left'])

    def test_complete_input_engine_json_path_can_enter_and_leave_wrist_collision(self):
        engine = self.engine(mode='first_person', sensitivity=1., wrist_guard=False)
        engine.q[5:7] = [math.radians(65.), math.radians(30.)]
        self.engage(engine)
        origin = np.asarray(engine.kin.forward_tcp(engine.q[:7], 'left')[0])
        desired = engine.q[:7]; desired[-1] = math.radians(65.)
        goal = np.asarray(engine.kin.forward_tcp(desired, 'left')[0])
        basis = BASE_FROM_TELEOP['left'] @ POSE_FRAMES['first_person']
        position = (basis.T @ (goal[:3, 3]-origin[:3, 3])/1000.).tolist()
        rotation = basis.T @ (goal[:3, :3] @ origin[:3, :3].T) @ basis
        moving = hand(grip=1., position=position)
        moving['pose'][3:] = Rotation.from_matrix(rotation).as_quat().tolist()
        state = json.loads(json.dumps(engine.step(frame(10.02, left=moving), 10.02), allow_nan=False))
        self.assertFalse(state['paused'], state['message'])
        self.assertTrue(state['wrist_collision']['left'])
        self.assertFalse(state['wrist_guard_enabled'])
        state = engine.step(frame(10.03, left=hand(grip=1.)), 10.03)
        self.assertFalse(state['wrist_collision']['left'])

    def test_rotation_both_tcp_tools_and_arms(self):
        for tool in ('flange', 'taccap'):
            for side in ARMS:
                engine = self.engine(mode='first_person', sensitivity=.5, tools=tool_config(tool))
                control = self.engage(engine, side)
                start = np.asarray(engine.kin.forward_tcp([engine.q[j-1] for j in ARMS[side]], side)[0])
                moving = hand(grip=1., angle=5.)
                state = engine.step(frame(10.02, **{control: moving}), 10.02)
                self.assertFalse(state['paused'], state['message'])
                actual = np.asarray(engine.kin.forward_tcp([engine.q[j-1] for j in ARMS[side]], side)[0])
                self.assertLess(np.linalg.norm(actual[:3, 3]-start[:3, 3]), .02)
                self.assertAlmostEqual(pose_error(actual, start)[1], 2.5, delta=.01)

    def test_disabled_gripper_simulation_does_not_claim_gripper_feedback(self):
        self.assertEqual(self.engine().snapshot(10.)['grippers'], {})

    def test_command_pipe_has_no_password_handshake_and_closes_on_bad_input(self):
        for contents in ('', '{"password":"secret"}\n', '{"command":"unknown"}\n', 'invalid\n'):
            commands = DemoCommands(io.StringIO(contents))
            commands.read()
            self.assertTrue(commands.closed.is_set())
            self.assertIsNone(commands.take())
        commands = DemoCommands(io.StringIO('{"command":"home"}\n{"command":"stop"}\n'))
        commands.read()
        self.assertEqual(commands.take(), 'home')
        self.assertEqual(commands.take(), 'stop')

    def test_cli_does_not_open_pico_or_any_robot_when_parent_already_closed(self):
        with patch('vr_demo.sys.stdin', io.StringIO('')), patch('vr_demo.Stream') as stream, \
             patch('vr_demo.emit') as emit, patch('vr_demo.threading.Thread') as thread:
            # Complete a known-EOF reader deterministically, independent of
            # thread scheduling in a loaded test runner.
            thread.return_value.start.side_effect = lambda: thread.call_args.kwargs['target']()
            self.assertEqual(main(['--gui-stdio']), 0)
        stream.assert_not_called()
        self.assertTrue(emit.called)

    def test_cli_uses_only_pico_input_and_closes_it_on_interrupt(self):
        with patch('vr_demo.Stream') as stream, patch('vr_demo.emit') as emit:
            stream.return_value.read.side_effect = KeyboardInterrupt()
            self.assertEqual(main(['--no-wrist-guard', '--grippers', '--body-lift']), 0)
        stream.return_value.open.assert_called_once_with()
        stream.return_value.close.assert_called_once_with()
        self.assertFalse(emit.call_args.args[0]['wrist_guard_enabled'])
        self.assertEqual(emit.call_args.args[0]['phase'], '已停止')

    def test_invalid_configuration_and_nonfinite_time_are_rejected(self):
        for sensitivity in (0., -1., float('nan'), 2.):
            with self.assertRaises(ValueError):
                self.engine(sensitivity=sensitivity)
        engine = self.engine()
        with self.assertRaises(ValueError):
            engine.step(None, float('inf'))


if __name__ == '__main__':
    unittest.main()
