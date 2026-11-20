"""Constant-pivot Home timing, packet checks and cancellation."""
import math
import unittest
from unittest.mock import patch

import numpy as np

from test_vr_demo import frame, hand
from test_vr_demo_fairino import isolated
from test_vr_fairino_packet import pose
from tools.ik_experiments.setup_fairino_ik import extension_path
from vr_core import validate
from vr_demo import DemoEngine
from vr_fairino_home import TORSO_RETURN_DEG_S, FairinoHomeMotion, return_duration
from vr_fairino_ik import AXES, BodyPath
from body_core import load_config


class TimingTests(unittest.TestCase):
    def test_bound_covers_combined_pitch_and_nonlinear_fold(self):
        body = load_config(live=True)
        for offset in (-5., 0., 3.55, 5.):
            q = pose(body, h=100., forward=-.8, x=offset)
            path = BodyPath(body, q)
            a = path.coord_min.copy()
            b = path.coord_max.copy()
            for start, end in ((a,b), (b,a), (a, [b[0], b[1], a[2]]),
                               (b, [a[0], b[1], b[2]]), (a,a)):
                end = np.asarray(end)
                duration = return_duration(path, start, end)
                self.assertGreaterEqual(duration, 2.)
                u = np.linspace(0., 1., 201)
                coords = start + np.outer(u**3*(10.+u*(-15.+6.*u)), end-start)
                angles = np.array([path.joints(c, q)[AXES] for c in coords])
                self.assertLessEqual(abs(np.degrees(np.diff(angles, axis=0))).max()*200/duration,
                                     TORSO_RETURN_DEG_S+1e-9)


@unittest.skipUnless(extension_path().is_file(), 'Fairino QP native extension absent')
class HomeTests(unittest.TestCase):
    def torso_phase(self):
        e = DemoEngine(ik_backend='fairino_qp')
        q = pose(e.body_model, h=80., forward=4., roll=.08, yaw=1., x=3.55)
        q[:14] = e.q[:14]
        e.q = q.tolist()
        e.step(frame(1.), 1.)
        e.step(frame(1.01, left=hand(grip=1.)), 1.01)
        path = e.native.path
        start = path.coordinates(q)
        e.q = path.joints(start + [3., 2., 3.], q).tolist()
        e.command('home')
        for i in range(1, 300):
            t = 1.01 + i*.1
            state = e.step(frame(t), t)
            self.assertFalse(state['message'].startswith('模型已暂停'), state['message'])
            if e.home_motion is not None and e.home_motion.returning_torso:
                return e, t
        self.fail('躯干回程未启动')

    @isolated
    def test_torso_return_cancels_on_a_stick_pause_grip_or_missing_sample(self):
        for action in ('a', 'stick', 'pause', 'home', 'grip', 'gap'):
            with self.subTest(action=action):
                e, t = self.torso_phase()
                path = e.native.path
                t += .1
                e.step(frame(t), t)
                before = e.q[:]
                t += .26 if action == 'gap' else .01
                f = frame(t)
                if action == 'grip':
                    f['samples'] = [frame(t-.005, right=hand(grip=1.)), frame(t)]
                elif action in ('a', 'stick'):
                    f['right']['primary' if action == 'a' else 'axis_click'] = True
                state = e.step(f, t, action if action in ('pause','home') else None)
                self.assertIsNone(e.home_motion)
                self.assertTrue(state['paused'])
                self.assertEqual(e.q, before)
                self.assertIs(e.native.path, path)
                np.testing.assert_array_equal(e.native.velocity, np.zeros(3))

    @isolated
    def test_packet_validates_nonzero_offset_and_interrupted_samples(self):
        e, t = self.torso_phase()
        motion = e.home_motion
        hw = e.hardware()
        with patch('vr_fairino_ik.validate', wraps=validate) as check:
            packet = motion.request(e.kin, hw, t+.1, 10, t+.1)
        self.assertIs(check.call_args.args[-1], e.native.body)
        self.assertEqual(set(packet['arms']), {'left', 'right'})
        self.assertEqual(packet['torso']['path_offset_mm'], e.native.path.offset)
        q = np.array(hw['command_position'])
        q[AXES] = packet['torso']['target']
        self.assertAlmostEqual(e.body_model.state(q)['forward_offset_mm'], 3.55)
        for stamp in (t+.09, t+.351):
            with self.assertRaisesRegex(ValueError, '更新中断'):
                motion.request(e.kin, hw, stamp, 11, stamp)

    @isolated
    def test_skips_torso_without_path_or_displacement(self):
        e = DemoEngine(ik_backend='fairino_qp')
        for create_path in (False, True):
            if create_path:
                e.native.path = BodyPath(e.body_model, e.q)
            motion = FairinoHomeMotion(e.home, e.hardware(), 1., e.guard, e.native)
            for i in range(100):
                now = 1.+i*.1
                if motion.complete(e.hardware(), now):
                    break
                packet = motion.request(e.kin, e.hardware(), now, 1, now)
                self.assertNotIn('torso', packet)
            else:
                self.fail('仅手臂 Home 未结束')


if __name__ == '__main__':
    unittest.main()
