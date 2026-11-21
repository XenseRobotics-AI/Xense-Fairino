"""Synthetic PICO input, with Fairino native calls isolated from TRON2."""
from functools import wraps
import io
import math
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

import numpy as np

from test_vr_demo import frame, hand
from tools.ik_experiments.setup_fairino_ik import extension_path
from vr_controls import JoystickControls
from vr_core import require_robot_backend, require_robot_request
from vr_demo import DemoEngine, main
from vr_fairino_ik import AXES


def isolated(method):
    @wraps(method)
    def run(self):
        module = Path(sys.argv[0]).stem if method.__module__ == '__main__' else method.__module__
        name = f'{module}.{type(self).__name__}.{method.__name__}'
        if os.environ.get('FAIRINO_DEMO_TEST_CHILD') == name:
            return method(self)
        child = subprocess.run([sys.executable, '-B', '-m', 'unittest', name],
                               cwd=Path(__file__).parent, capture_output=True, text=True,
                               env=dict(os.environ, FAIRINO_DEMO_TEST_CHILD=name))
        self.assertEqual(child.returncode, 0, child.stderr + child.stdout)
        self.assertNotIn('skipped=', child.stderr)
    return run


class InputTests(unittest.TestCase):
    def test_right_a_only_on_armed_physical_rising_edge(self):
        for enabled in (False, True):
            controls = JoystickControls(right_a_home=enabled)
            def sample(t, a, left_a=False):
                f = frame(t)
                f['right']['primary'], f['left']['primary'] = a, left_a
                return f
            for t, a in ((1., True), (1.01, True), (1.02, False)):
                self.assertFalse(controls.update(sample(t, a, True), t)['home'])
            f = sample(1.04, False)
            f['samples'] = [sample(1.03, True), sample(1.04, False)]
            self.assertEqual(controls.update(f, 1.04)['home'], enabled)
            self.assertFalse(controls.update(f, 1.05)['home'])
            self.assertFalse(controls.update(sample(2., True), 2.)['home'])

    def test_lift_conflict_and_robot_gate(self):
        with patch('vr_demo_fairino_ik.FairinoDemoIK') as solver:
            with self.assertRaisesRegex(ValueError, '升降'):
                DemoEngine(ik_backend='fairino_qp', body_lift=True)
            solver.assert_not_called()
        self.assertEqual(require_robot_backend('fairino_qp'), 'fairino_qp')

    @isolated
    @unittest.skipUnless(extension_path().is_file(), 'Fairino QP native extension absent')
    def test_second_home_press_cancels_with_buffered_release(self):
        for split in (False, True):
            with self.subTest(two_ticks=split):
                e = DemoEngine(ik_backend='fairino_qp')
                e.step(frame(1.), 1.)
                samples = [frame(1.01), frame(1.02), frame(1.03), frame(1.04)]
                for index in (0, 2):
                    samples[index]['right']['primary'] = True
                if split:
                    e.step(dict(samples[1], samples=samples[:2]), 1.02)
                    self.assertIsNotNone(e.home_motion)
                batch = samples[2:] if split else samples
                state = e.step(dict(samples[3], samples=batch), 1.04)
                self.assertTrue(state['paused'])
                self.assertIn('已中止', state['message'])
                self.assertFalse(e.home_pending)
                self.assertIsNone(e.home_motion)

    @isolated
    def test_missing_build_prints_install_command_before_opening_pico(self):
        with (patch('tools.ik_experiments.fairino_qp_art7.extension_path',
                    return_value=Path('/no-such-directory/_native.so')),
              patch('vr_demo.NativeIK') as fallback, patch('vr_demo.Stream') as stream,
              patch('sys.stdout', new_callable=io.StringIO) as output):
            self.assertEqual(main(['--ik-backend', 'fairino_qp']), 1)
        self.assertIn('.local/pico4/env/bin/python tools/ik_experiments/setup_fairino_ik.py --install',
                      output.getvalue())
        stream.assert_not_called()
        fallback.assert_not_called()


@unittest.skipUnless(extension_path().is_file(), 'Fairino QP native extension absent')
class DemoFlowTests(unittest.TestCase):
    @isolated
    def test_dxhand_released_grips_keep_whole_body_solver_engaged(self):
        engine = DemoEngine(ik_backend='fairino_qp', dxhand=True, interpolation_delay_ms=0.)
        initial = np.array(engine.q)
        for i in range(11):
            now = 2. + i*.01
            packet = frame(now, left=hand(position=(0., -.001*i, -.001*i)))
            state = engine.step(dict(packet, samples=[packet]), now)
            if i:
                self.assertEqual(state['sides'], 'left/right', state['message'])
        self.assertGreater(np.max(np.abs(np.array(engine.q)-initial)), .001)
        engine.command('pause')
        held = engine.q[:]
        engine.step(frame(2.12, right=hand(grip=1.)), 2.12)
        np.testing.assert_array_equal(engine.q, held)
        self.assertEqual(engine.sides, ())

    @isolated
    def test_follow_release_physical_a_and_two_stage_home(self):
        e = DemoEngine(ik_backend='fairino_qp', sensitivity=1., interpolation_delay_ms=0.,
                       mode='first_person', hand_mapping='crossed')
        e.q[14], e.q[16] = .19, math.radians(.07)
        original = np.array(e.q)
        t = 2.
        def tick(grip=0., shift=0., a=False):
            nonlocal t
            f = frame(t, left=hand(grip=grip, position=(0., -shift, -shift)),
                      right=hand(grip=grip, position=(0., -shift, -shift)))
            f['right']['primary'] = a
            state = e.step(f, t)
            t += .01
            self.assertFalse(state['message'].startswith('模型已暂停'), state['message'])
            np.testing.assert_array_equal(np.array(e.q)[[14,16]], original[[14,16]])
        tick()
        with patch.object(e, '_apply', wraps=e._apply) as apply:
            for i in range(151):
                tick(1., .30*min(1., i/100.))
            self.assertTrue(e.partial)
            self.assertGreater(max(abs(np.array(e.q)[AXES]-original[AXES])), math.radians(.1))
            packet = apply.call_args.args[0]
            self.assertEqual(packet['ik_backend'], 'fairino_qp')
            self.assertTrue(packet['demo_only'])
            with self.assertRaises(ValueError):
                require_robot_request(packet)
            path = e.native.path
            moved = e.q[:]
            tick(1., .30, a=True)
            self.assertTrue(e.home_pending)
            self.assertEqual(e.q, moved)
            tick()
            saw_torso = False
            for _ in range(6000):
                previous = np.array(e.q)
                tick()
                if e.home_motion is None:
                    break
                packet = apply.call_args.args[0]
                self.assertEqual(packet['source'], 'home')
                self.assertTrue(packet['demo_only'])
                if 'torso' in packet:
                    saw_torso = True
                    self.assertEqual(set(packet['arms']), {'left','right'})
                    np.testing.assert_array_equal(e.q[:14], previous[:14])
                    np.testing.assert_allclose(previous[:14], original[:14], atol=1e-14, rtol=0)
                    self.assertLessEqual(max(abs(np.array(e.q)[AXES]-previous[AXES])),
                                         math.radians(.05)+2e-14)
                else:
                    self.assertFalse(saw_torso)
                    np.testing.assert_array_equal(e.q[14:], moved[14:])
            self.assertTrue(saw_torso)
            self.assertIsNone(e.home_motion)
            self.assertTrue(e.paused)
            self.assertIs(e.native.path, path)
            np.testing.assert_array_equal(e.native.velocity, np.zeros(3))
            np.testing.assert_array_equal(np.array(e.q)[AXES], original[AXES])

    @isolated
    def test_freeze_a_and_disabled_guard_with_real_demo_backend(self):
        e = DemoEngine(ik_backend='fairino_qp', freeze_buttons=True, wrist_guard=False)
        e.step(frame(1.), 1.)
        f = frame(1.01, right=hand(grip=1.))
        f['right']['primary'] = True
        with patch.object(e, '_apply', wraps=e._apply) as apply:
            state = e.step(f, 1.01)
            self.assertFalse(state['paused'], state['message'])
            self.assertTrue(apply.call_args.args[0]['demo_only'])
        self.assertTrue(e.clutches['left'].frozen_rotation)
        self.assertFalse(e.home_pending)
        self.assertIsNone(e.home_motion)
        f = frame(1.02)
        f['right']['axis_click'] = True
        e.step(f, 1.02)
        self.assertIsNotNone(e.home_motion)


if __name__ == '__main__':
    unittest.main()
