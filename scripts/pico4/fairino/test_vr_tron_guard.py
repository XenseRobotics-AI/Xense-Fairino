"""Actual TRON2 C++ wrist-boundary tests with synthetic PICO input only."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import paths
import numpy as np
from lerobot.utils.rotation import Rotation
from mounts import BASE_FROM_TELEOP
from tcp_core import ARMS, pose_error
from test_vr_demo import frame, hand
from tool_frames import tool_config
from vr_demo import DemoEngine
from vr_input import POSE_FRAMES
from wrist_guard import WristGuard


class TronWristGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from tools.ik_experiments.tron2_art7 import dependencies_available
        if not dependencies_available():
            raise unittest.SkipTest('optional TRON2 native dependencies not installed')

    def boundary_case(self, side, enabled, tool='flange'):
        engine = DemoEngine(ik_backend='tron2', mode='first_person', sensitivity=1.,
                            wrist_guard=enabled, tools=tool_config(tool), interpolation_delay_ms=0.)
        joints = ARMS[side]
        for index, degrees in zip(joints[-2:], (65., 30.)):
            engine.q[index-1] = math.radians(degrees)
        before = [engine.q[j-1] for j in joints]
        start = np.asarray(engine.kin.forward_tcp(before, side)[0])
        target = before[:]
        target[-1] = math.radians(65.)
        goal = np.asarray(engine.kin.forward_tcp(target, side)[0])
        basis = BASE_FROM_TELEOP[side] @ POSE_FRAMES['first_person']
        moving = hand(grip=1., position=(basis.T @ (goal[:3, 3]-start[:3, 3])/1000.).tolist())
        rotation = basis.T @ (goal[:3, :3] @ start[:3, :3].T) @ basis
        moving['pose'][3:] = Rotation.from_matrix(rotation).as_quat().tolist()
        engine.step(frame(10.), 10.)
        state = engine.step(frame(10.01, **{side: hand(grip=1.)}), 10.01)
        self.assertFalse(state['paused'], state['message'])
        return engine, before, goal, moving

    def test_protected_boundary_rebases_and_stationary_controller_has_no_drift(self):
        for side in ARMS:
            for tool in ('flange', 'taccap'):
                with self.subTest(side=side, tool=tool):
                    engine, before, goal, moving = self.boundary_case(side, True, tool)
                    state = engine.step(frame(10.02, **{side: moving}), 10.02)
                    self.assertFalse(state['paused'], state['message'])
                    self.assertTrue(state['wrist_limited'])
                    self.assertFalse(any(state['wrist_guard_violation'].values()))
                    self.assertFalse(any(state['wrist_collision'].values()))
                    accepted = [engine.q[j-1] for j in ARMS[side]]
                    WristGuard().arm_segment(before, accepted)
                    error = pose_error(goal, engine.kin.forward_tcp(accepted, side)[0])
                    self.assertGreater(max(error), 1.)  # Target was deliberately outside the wrist envelope.
                    settled = engine.q[:]
                    for i in range(1, 51):
                        now = 10.02+i*.01
                        state = engine.step(frame(now, **{side: moving}), now)
                        self.assertFalse(state['paused'], state['message'])
                        self.assertFalse(any(state['wrist_guard_violation'].values()))
                    np.testing.assert_allclose(engine.q, settled, atol=1e-12, rtol=0.)
                    self.assertEqual(state['ik_backend'], 'tron2')

    def test_guard_off_can_enter_red_zone_and_rejects_enabling_guard_there(self):
        for side in ARMS:
            with self.subTest(side=side):
                engine, _, goal, moving = self.boundary_case(side, False)
                state = engine.step(frame(10.02, **{side: moving}), 10.02)
                self.assertFalse(state['paused'], state['message'])
                self.assertFalse(state['wrist_limited'])
                self.assertTrue(state['wrist_collision'][side])
                actual = engine.kin.forward_tcp([engine.q[j-1] for j in ARMS[side]], side)[0]
                mm, deg = pose_error(goal, actual)
                self.assertLessEqual(mm, .02)
                self.assertLessEqual(deg, .01)
                settled = engine.q[:]
                for i in range(1, 31):
                    now = 10.02+i*.01
                    state = engine.step(frame(now, **{side: moving}), now)
                    self.assertFalse(state['paused'], state['message'])
                np.testing.assert_allclose(engine.q, settled, atol=1e-12, rtol=0.)
                engine.command('guard_on')
                self.assertFalse(engine.guard.enabled)
                self.assertIn('先回 Home', engine.message)
                self.assertEqual(engine.q, settled)

    def test_guard_off_retreat_leaves_collision_and_home_restores_guard(self):
        engine, _, _, moving = self.boundary_case('left', False)
        state = engine.step(frame(10.02, left=moving), 10.02)
        self.assertTrue(state['wrist_collision']['left'])
        state = engine.step(frame(10.03, left=hand(grip=1.)), 10.03)
        self.assertFalse(state['paused'], state['message'])
        self.assertFalse(state['wrist_collision']['left'])
        # Same TCP can use different redundant joint angles. Returning the
        # controller does not promise to restore the original joint posture
        # or the eroded 2-degree margin; Home explicitly restores both.
        engine.command('home')
        for i in range(1, 1800):
            now = 10.03+i*.01
            state = engine.step(frame(now), now)
        self.assertFalse(any(state['wrist_guard_violation'].values()))
        engine.command('guard_on')
        self.assertTrue(engine.guard.enabled)
        self.assertTrue(engine.paused)
        self.assertEqual(engine.snapshot(28.)['ik_backend'], 'tron2')


if __name__ == '__main__':
    unittest.main()
