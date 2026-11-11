"""TRON2 Demo acceptance on current ART7 geometry, with synthetic PICO input."""
import io
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from vr_demo import DemoEngine, main
from vr_demo_tron_ik import TronDemoIK
from test_vr_demo import frame, hand
from tcp_core import ARMS, pose_error
from tool_frames import tool_config
from vr_core import require_robot_request
from vr_input import POSE_FRAMES, mapped_pose
from wrist_guard import WristGuard


class TronAdapterTests(unittest.TestCase):
    """Boundary checks run even when optional TRON2 native dependencies are absent."""
    def setUp(self):
        self.engine = DemoEngine(interpolation_delay_ms=0.)
        self.adapter = TronDemoIK.__new__(TronDemoIK)
        self.adapter.tools = self.engine.kin.tools
        self.references = {s: [self.engine.q[j-1] for j in js] for s, js in ARMS.items()}
        self.goals = {s: np.asarray(self.engine.kin.forward_tcp(q, s)[0]) for s, q in self.references.items()}

    def answer(self):
        return {s: dict(target=q[:], tcp=self.goals[s].copy(), success=True, iterations=0)
                for s, q in self.references.items()}

    def request(self):
        return self.adapter.request(self.engine.kin, self.engine.hardware(), self.goals,
                                    1, 10., self.references, WristGuard())

    def test_packet_marked_demo_only_and_receiver_rejects_it(self):
        with patch.object(self.adapter, 'solver', create=True) as solver:
            solver.solve.return_value = self.answer()
            packet = self.request()
        self.assertTrue(packet['demo_only'])
        self.assertEqual(packet['ik_backend'], 'tron2')
        with self.assertRaisesRegex(ValueError, 'Demo'):
            require_robot_request(packet)

    def test_bad_second_arm_fk_rejects_entire_packet(self):
        answer = self.answer()
        answer['right']['tcp'][0, 3] += 1.
        with patch.object(self.adapter, 'solver', create=True) as solver:
            solver.solve.return_value = answer
            with self.assertRaisesRegex(ValueError, '正解校验'):
                self.request()

    def test_consistent_fk_but_worsening_solution_is_rejected(self):
        answer = self.answer()
        answer['left']['target'][0] += .05
        answer['left']['tcp'] = self.engine.kin.forward_tcp(answer['left']['target'], 'left')[0]
        with patch.object(self.adapter, 'solver', create=True) as solver:
            solver.solve.return_value = answer
            with self.assertRaisesRegex(ValueError, '增大了完整目标误差'):
                self.request()

    def test_nonfinite_or_invalid_rotation_never_calls_native(self):
        for value in (float('nan'), 2.):
            self.goals['left'][0, 0] = value
            with patch.object(self.adapter, 'solver', create=True) as solver:
                with self.assertRaises(ValueError):
                    self.request()
                solver.solve.assert_not_called()

    def test_startup_failure_never_opens_pico_or_falls_back(self):
        with patch('vr_demo_tron_ik.TronDemoIK', side_effect=RuntimeError('missing TRON2 dependencies')), \
             patch('vr_demo.NativeIK', side_effect=AssertionError('fallback')), \
             patch('vr_demo.Stream') as stream, patch('sys.stdout', new_callable=io.StringIO) as output:
            self.assertEqual(main(['--ik-backend', 'tron2']), 1)
        stream.assert_not_called()
        self.assertIn('missing TRON2 dependencies', output.getvalue())


class TronDemoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from tools.ik_experiments.tron2_art7 import dependencies_available
        if not dependencies_available():
            raise unittest.SkipTest('optional TRON2 native dependencies not installed')

    def engine(self, **kwargs):
        return DemoEngine(ik_backend='tron2', interpolation_delay_ms=0., **kwargs)

    def engage(self, engine, side='left'):
        engine.step(frame(10.), 10.)
        controller = engine.hands[side]
        state = engine.step(frame(10.01, **{controller: hand(grip=1.)}), 10.01)
        self.assertFalse(state['paused'], state['message'])
        return controller

    def test_all_modes_sides_axes_and_gains(self):
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
                            goal[:3, 3] += sensitivity*(mapped_pose(moving['pose'], side, mode)[0]
                                                       - mapped_pose(hand()['pose'], side, mode)[0])
                            state = engine.step(frame(10.02, **{control: moving}), 10.02)
                            self.assertFalse(state['paused'], state['message'])
                            actual = engine.kin.forward_tcp([engine.q[j-1] for j in ARMS[side]], side)[0]
                            mm, deg = pose_error(actual, goal)
                            self.assertLessEqual(mm, .02)
                            self.assertLessEqual(deg, .01)
                            self.assertEqual(state['ik_backend'], 'tron2')
                            other = 'left' if side == 'right' else 'right'
                            self.assertEqual([engine.q[j-1] for j in ARMS[other]],
                                             [initial[j-1] for j in ARMS[other]])
                            cases += 1
        self.assertEqual(cases, 54)

    def test_both_tools_rotations_and_guard_settings(self):
        for tool in ('flange', 'taccap'):
            for side in ARMS:
                for enabled in (True, False):
                    with self.subTest(tool=tool, side=side, guard=enabled):
                        engine = self.engine(mode='first_person', tools=tool_config(tool),
                                             sensitivity=.5, wrist_guard=enabled)
                        control = self.engage(engine, side)
                        original = engine.kin.forward_tcp([engine.q[j-1] for j in ARMS[side]], side)[0]
                        state = engine.step(frame(10.02, **{control: hand(grip=1., angle=5.)}), 10.02)
                        self.assertFalse(state['paused'], state['message'])
                        actual = engine.kin.forward_tcp([engine.q[j-1] for j in ARMS[side]], side)[0]
                        mm, deg = pose_error(original, actual)
                        self.assertLessEqual(mm, .02)
                        self.assertAlmostEqual(deg, 2.5, delta=.01)
                        self.assertIsNone(engine.unconstrained)

    def test_hold_release_and_stale_input_have_no_tail(self):
        engine = self.engine()
        control = self.engage(engine)
        moving = hand(grip=1., position=(.01, 0., 0.))
        state = engine.step(frame(10.02, **{control: moving}), 10.02)
        self.assertFalse(state['paused'], state['message'])
        initial = engine.q[:]
        for i in range(1, 21):
            now = 10.02+i*.01
            state = engine.step(frame(now, **{control: moving}), now)
            self.assertFalse(state['paused'], state['message'])
        np.testing.assert_allclose(engine.q, initial, atol=1e-12, rtol=0.)
        state = engine.step(None, 10.23)
        self.assertTrue(state['paused'])
        self.assertEqual(engine.q, initial)
        state = engine.step(frame(10.24, **{control: hand(grip=1., position=(1., 0., 0.))}), 10.24)
        self.assertTrue(state['paused'])
        self.assertEqual(engine.q, initial)

    def test_guard_switch_keeps_tron_and_home_resets(self):
        engine = self.engine()
        control = self.engage(engine)
        engine.step(frame(10.02, **{control: hand(grip=1., position=(.01, 0., 0.))}), 10.02)
        for command in ('guard_off', 'guard_on'):
            engine.command(command)
            state = engine.snapshot(10.03)
            self.assertEqual(state['ik_backend'], 'tron2')
            self.assertTrue(state['paused'])
        engine.command('home')
        for i in range(1, 900):
            now = 10.03+i*.01
            state = engine.step(frame(now), now)
        self.assertIsNone(engine.home_motion)
        self.assertFalse(engine.home_pending)
        for side, joints in ARMS.items():
            np.testing.assert_allclose([engine.q[j-1] for j in joints], engine.home['target'][side], atol=1e-9)

    def test_solver_failure_keeps_both_arms_without_fallback(self):
        engine = self.engine(mode='first_person')
        engine.step(frame(10.), 10.)
        engine.step(frame(10.01, left=hand(grip=1.), right=hand(grip=1.)), 10.01)
        initial = engine.q[:]
        with patch.object(engine.native.solver, 'solve', side_effect=RuntimeError('TRON2 test failure')):
            moving = hand(grip=1., position=(.01, 0., 0.))
            state = engine.step(frame(10.02, left=moving, right=moving), 10.02)
        self.assertTrue(state['paused'])
        self.assertEqual(engine.q, initial)
        self.assertIn('TRON2 test failure', state['message'])
        self.assertEqual(state['ik_backend'], 'tron2')

    def test_cli_selection_uses_real_tron_core_without_robot_transport(self):
        with patch('vr_demo.Stream') as stream, patch('vr_demo.emit') as emit, \
             patch('vr_demo.NativeIK', side_effect=AssertionError('unexpected fallback')):
            stream.return_value.read.side_effect = KeyboardInterrupt()
            self.assertEqual(main(['--ik-backend', 'tron2', '--no-wrist-guard']), 0)
        state = json.loads(json.dumps(emit.call_args.args[0], allow_nan=False))
        self.assertEqual(state['ik_backend'], 'tron2')
        self.assertFalse(state['wrist_guard_enabled'])
        stream.return_value.close.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
