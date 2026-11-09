"""Vendor-library Demo acceptance, synthetic PICO input and no robot transport."""
import io
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from vr_demo import DemoEngine, main
import numpy as np
from lerobot.utils.rotation import Rotation
from mounts import BASE_FROM_TELEOP
from tcp_core import ARMS, pose_error
from test_vr_demo import frame, hand
from tool_frames import tool_config
from vendor_kinematics import VendorKinematics
from vr_input import POSE_FRAMES, mapped_pose


class VendorDemoTests(unittest.TestCase):
    def engine(self, **kwargs):
        return DemoEngine(ik_backend='vendor', interpolation_delay_ms=0., **kwargs)

    def engage(self, engine, arm='left'):
        engine.step(frame(10.), 10.)
        controller = engine.hands[arm]
        engine.step(frame(10.01, **{controller: hand(grip=1.)}), 10.01)
        return controller

    def wrist_target(self, engine):
        engine.q[5:7] = list(map(math.radians, (65., 30.)))
        origin = np.asarray(engine.kin.forward_tcp(engine.q[:7], 'left')[0])
        desired = engine.q[:7]
        desired[6] = math.radians(65.)
        goal = np.asarray(engine.kin.forward_tcp(desired, 'left')[0])
        basis = BASE_FROM_TELEOP['left'] @ POSE_FRAMES['first_person']
        moving = hand(grip=1., position=(basis.T @ (goal[:3, 3]-origin[:3, 3])/1000.).tolist())
        relative = basis.T @ (goal[:3, :3] @ origin[:3, :3].T) @ basis
        moving['pose'][3:] = Rotation.from_matrix(relative).as_quat().tolist()
        return moving, goal

    def test_vendor_loads_only_offline_library_and_checks_every_home(self):
        with patch('vr_demo.NativeIK', side_effect=AssertionError('Native IK constructed')):
            for home in ('default', 'right_mirrored', 'left_cross_body'):
                engine = self.engine(home_preset=home)
                self.assertIsInstance(engine.kin, VendorKinematics)
                self.assertIsNone(engine.native)
                self.assertIsNone(engine.unconstrained)
                state = engine.snapshot(10.)
                self.assertEqual(state['ik_backend'], 'vendor')
                self.assertEqual(state['ik_requested_backend'], 'vendor')
                self.assertFalse(any(state['wrist_collision'].values()))

    def test_same_translation_mapping_for_modes_sides_axes_and_gains(self):
        cases = 0
        for mode in POSE_FRAMES:
            for side in ARMS:
                for axis in range(3):
                    for sensitivity in (.25, .5, 1.):
                        with self.subTest(mode=mode, side=side, axis=axis, sensitivity=sensitivity):
                            engine = self.engine(mode=mode, sensitivity=sensitivity)
                            control = self.engage(engine, side)
                            initial = engine.q[:]
                            goal = np.asarray(engine.kin.forward_tcp([initial[j-1] for j in ARMS[side]], side)[0])
                            position = [0., 0., 0.]
                            position[axis] = .01
                            moving = hand(grip=1., position=position)
                            goal[:3, 3] += sensitivity * (mapped_pose(moving['pose'], side, mode)[0]
                                                         - mapped_pose(hand()['pose'], side, mode)[0])
                            state = engine.step(frame(10.02, **{control: moving}), 10.02)
                            self.assertFalse(state['paused'], state['message'])
                            actual = engine.kin.forward_tcp([engine.q[j-1] for j in ARMS[side]], side)[0]
                            mm, deg = pose_error(actual, goal)
                            self.assertLess(mm, .02)
                            self.assertLess(deg, .01)
                            other = 'left' if side == 'right' else 'right'
                            self.assertEqual([engine.q[j-1] for j in ARMS[other]],
                                             [initial[j-1] for j in ARMS[other]])
                            cases += 1
        self.assertEqual(cases, 54)

    def test_rotation_tools_both_arms_and_both_guard_settings(self):
        for tool in ('flange', 'taccap'):
            for side in ARMS:
                for protected in (True, False):
                    with self.subTest(tool=tool, side=side, protected=protected):
                        engine = self.engine(mode='first_person', sensitivity=.5,
                                             tools=tool_config(tool), wrist_guard=protected)
                        control = self.engage(engine, side)
                        start = np.asarray(engine.kin.forward_tcp([engine.q[j-1] for j in ARMS[side]], side)[0])
                        state = engine.step(frame(10.02, **{control: hand(grip=1., angle=5.)}), 10.02)
                        self.assertFalse(state['paused'], state['message'])
                        actual = engine.kin.forward_tcp([engine.q[j-1] for j in ARMS[side]], side)[0]
                        mm, deg = pose_error(start, actual)
                        self.assertLess(mm, .02)
                        self.assertAlmostEqual(deg, 2.5, delta=.01)
                        self.assertEqual(state['ik_backend'], 'vendor')

    def test_protected_vendor_limits_target_and_discards_excess_without_drift(self):
        engine = self.engine(mode='first_person', sensitivity=1.)
        moving, goal = self.wrist_target(engine)
        self.engage(engine)
        state = engine.step(frame(10.02, left=moving), 10.02)
        self.assertFalse(state['paused'], state['message'])
        self.assertTrue(state['wrist_limited'])
        self.assertFalse(state['ik_partial'])
        self.assertFalse(any(state['wrist_guard_violation'].values()))
        self.assertFalse(state['wrist_collision']['left'])
        limited = engine.q[:]
        for i in range(1, 31):
            now = 10.02+i*.01
            engine.step(frame(now, left=moving), now)
        np.testing.assert_allclose(engine.q, limited, atol=1e-8)
        self.assertGreater(pose_error(goal, engine.kin.forward_tcp(engine.q[:7], 'left')[0])[1], 1.)

    def test_guard_off_stays_vendor_and_can_enter_and_leave_red_zone(self):
        engine = self.engine(mode='first_person', sensitivity=1., wrist_guard=False)
        moving, goal = self.wrist_target(engine)
        self.engage(engine)
        with patch('vr_demo_ik.DemoBoxIK', side_effect=AssertionError('Wrong IK fallback')):
            state = json.loads(json.dumps(engine.step(frame(10.02, left=moving), 10.02), allow_nan=False))
            self.assertFalse(state['paused'], state['message'])
            self.assertEqual(state['ik_backend'], 'vendor')
            self.assertFalse(state['wrist_guard_enabled'])
            self.assertTrue(state['wrist_collision']['left'])
            self.assertFalse(state['wrist_limited'])
            mm, deg = pose_error(goal, engine.kin.forward_tcp(engine.q[:7], 'left')[0])
            self.assertLess(mm, .02)
            self.assertLess(deg, .01)
            state = engine.step(frame(10.03, left=hand(grip=1.)), 10.03)
            self.assertFalse(state['wrist_collision']['left'])

    def test_failure_in_second_arm_never_applies_first_arm_or_falls_back(self):
        engine = self.engine(mode='first_person')
        engine.step(frame(10.), 10.)
        engine.step(frame(10.01, left=hand(grip=1.), right=hand(grip=1.)), 10.01)
        initial = engine.q[:]
        original = engine.kin.inverse_tcp
        def inverse(tcp, angle, ref, side):
            if side == 'right':
                raise ValueError('模拟原厂逆解失败')
            return original(tcp, angle, ref, side)
        with patch.object(engine.kin, 'inverse_tcp', side_effect=inverse) as solve:
            moving = hand(grip=1., position=(.01, 0., 0.))
            state = engine.step(frame(10.02, left=moving, right=moving), 10.02)
        self.assertGreaterEqual(solve.call_count, 2)
        self.assertEqual(initial, engine.q)
        self.assertTrue(state['paused'])
        self.assertIn('模拟原厂逆解失败', state['message'])
        self.assertEqual(state['ik_backend'], 'vendor')

    def test_pause_stale_and_guard_toggle_hold_until_grip_is_released(self):
        engine = self.engine()
        control = self.engage(engine)
        engine.step(frame(10.02, **{control: hand(grip=1., position=(.01, 0., 0.))}), 10.02)
        initial = engine.q[:]
        engine.command('guard_off')
        self.assertTrue(engine.paused)
        self.assertEqual(initial, engine.q)
        state = engine.step(frame(10.03, **{control: hand(grip=1., position=(1., 0., 0.))}), 10.03)
        self.assertTrue(state['paused'])
        self.assertEqual(initial, engine.q)
        engine.step(frame(10.04), 10.04)
        engine.step(frame(10.05, **{control: hand(grip=1.)}), 10.05)
        # Regrip may re-solve the identical TCP with floating-point roundoff.
        np.testing.assert_allclose(initial, engine.q, atol=1e-12, rtol=0.)
        settled = engine.q[:]
        state = engine.step(None, 10.06)
        self.assertTrue(state['paused'])
        self.assertEqual(settled, engine.q)
        engine.command('guard_on')
        self.assertTrue(engine.guard.enabled)
        self.assertEqual(engine.snapshot(10.07)['ik_backend'], 'vendor')

    def test_vendor_cannot_enable_guard_in_red_zone_but_can_return_home(self):
        engine = self.engine(mode='first_person', sensitivity=1., wrist_guard=False)
        moving, _ = self.wrist_target(engine)
        self.engage(engine)
        engine.step(frame(10.02, left=moving), 10.02)
        engine.command('guard_on')
        self.assertFalse(engine.guard.enabled)
        self.assertIn('先回 Home', engine.message)
        engine.command('home')
        for i in range(1, 1800):
            now = 10.02+i*.01
            engine.step(frame(now), now)
        engine.command('guard_on')
        self.assertTrue(engine.guard.enabled)
        np.testing.assert_allclose(engine.q[:7], engine.home['target']['left'], atol=1e-9)

    def test_cli_forwards_selection_and_library_failure_never_opens_input(self):
        with patch('vr_demo.Stream') as stream, patch('vr_demo.emit') as emit:
            stream.return_value.read.side_effect = KeyboardInterrupt()
            self.assertEqual(main(['--ik-backend', 'vendor', '--no-wrist-guard']), 0)
        self.assertEqual(emit.call_args.args[0]['ik_backend'], 'vendor')
        self.assertFalse(emit.call_args.args[0]['wrist_guard_enabled'])
        stream.return_value.close.assert_called_once_with()
        with patch('vendor_kinematics.VendorKinematics', side_effect=OSError('missing offline IK library')), \
             patch('vr_demo.Stream') as stream, patch('sys.stdout', new_callable=io.StringIO) as output:
            self.assertEqual(main(['--ik-backend', 'vendor']), 1)
        stream.assert_not_called()
        self.assertIn('missing offline IK library', output.getvalue())

    def test_unknown_backend_is_rejected_and_native_default_is_unchanged(self):
        with self.assertRaisesRegex(ValueError, 'native 或 vendor'):
            DemoEngine(ik_backend='unknown')
        engine = DemoEngine()
        self.assertEqual(engine.snapshot(10.)['ik_backend'], 'native')
        engine.command('guard_off')
        self.assertEqual(engine.snapshot(10.)['ik_backend'], 'demo_joint_box_only')
        self.assertEqual(engine.snapshot(10.)['ik_requested_backend'], 'native')


if __name__ == '__main__':
    unittest.main()
