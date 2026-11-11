"""Real TRON2 planner protocol with offline vendor FK and synthetic feedback."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import paths
sys.path.insert(0, str(paths.ROOT))
import numpy as np
from tcp_core import ARMS, multiply, pose_error, target_matrix, tcp_forward_matrix
from tool_frames import for_arm, tool_config
from tools.ik_experiments.tron2_art7 import dependencies_available
from vendor_kinematics import VendorKinematics
from vr_core import direct_positions, require_robot_request, validate
from vr_tron_ik import TronIK
from wrist_guard import PREDICTION_SECONDS, WristGuard
from test_vr_native_ik import hardware


@unittest.skipUnless(dependencies_available(), 'Optional TRON2 native dependencies/build absent')
class TronRealPlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tron = TronIK(tool_config())
        cls.kin = VendorKinematics()

    def setUp(self):
        self.hw = hardware()

    def goal(self, side='left', shift=5.):
        q = [self.hw['command_position'][j-1] for j in ARMS[side]]
        goal = np.asarray(self.kin.forward_tcp(q, side)[0])
        goal[0, 3] += shift
        return goal

    def request(self, goals=None, **kw):
        return self.tron.request(self.kin, self.hw,
            {'left': self.goal()} if goals is None else goals, 1, 100., **kw)

    def test_real_both_tools_both_arms_translations_and_rotations(self):
        for mode in ('flange', 'taccap'):
            tools = tool_config(mode)
            tron, kin = TronIK(tools), VendorKinematics(tools=tools)
            for axis in range(6):
                for sign in (-1, 1):
                    with self.subTest(mode=mode, axis=axis, sign=sign):
                        goals = {}
                        for side, joints in ARMS.items():
                            q = [self.hw['command_position'][j-1] for j in joints]
                            origin = kin.forward_tcp(q, side)[0]
                            if axis < 3:
                                goals[side] = np.asarray(origin).copy()
                                goals[side][axis, 3] += sign*5.
                            else:
                                goals[side] = np.asarray(target_matrix(origin, axis, sign, .5))
                        with patch.object(kin, 'inverse_tcp', side_effect=AssertionError('vendor IK fallback')):
                            packet = tron.request(kin, self.hw, goals, 1, 100.)
                        require_robot_request(packet)
                        self.assertNotIn('demo_only', packet)
                        self.assertEqual(packet['ik_backend'], 'tron2')
                        self.assertEqual(set(packet['arms']), set(ARMS))
                        validate(self.hw, packet, 100., tools)
                        json.dumps(packet, allow_nan=False)
                        for side, entry in packet['arms'].items():
                            self.assertTrue(entry['native_ik']['success'], entry)
                            actual = multiply(tcp_forward_matrix(entry['target']), for_arm(tools, side))
                            mm, deg = pose_error(goals[side], actual)
                            self.assertLessEqual(mm, .02)
                            self.assertLessEqual(deg, .01)

    def test_single_arm_holds_other_and_body_exactly(self):
        packet = self.request()
        self.assertEqual(set(packet['arms']), {'left'})
        held = self.hw['command_position'][:]
        held[7] += .01
        result = direct_positions(held, validate(self.hw, packet, 100.), held)
        self.assertEqual(result[-1][1][7:], held[7:])
        with patch.object(self.tron.solver, 'solve', side_effect=AssertionError('empty solve')):
            self.assertIsNone(self.request({}))

    def test_guard_off_refused_before_solve(self):
        with patch.object(self.tron.solver, 'solve') as solve:
            with self.assertRaisesRegex(ValueError, '必须保持开启'):
                self.request(guard=WristGuard(False))
            solve.assert_not_called()

    def test_planned_and_measured_paths_use_measured_velocity(self):
        self.hw['position'][0] += .001
        self.hw['velocity'][5:7] = np.radians([5., -3.]).tolist()
        guard = WristGuard()
        with patch.object(guard, 'arm_segment', wraps=guard.arm_segment) as check:
            packet = self.request(guard=guard)
        self.assertEqual(check.call_count, 2)
        self.assertEqual(check.call_args_list[0].args[0], self.hw['command_position'][:7])
        self.assertEqual(check.call_args_list[1].args[0], self.hw['position'][:7])
        self.assertEqual(check.call_args_list[1].args[2], self.hw['velocity'][:7])
        np.testing.assert_allclose(packet['arms']['left']['native_ik']['wrist_reserve_deg'],
                                   np.array([5., 3.])*PREDICTION_SECONDS)

    def test_moving_outward_near_boundary_is_not_hidden_by_safe_reference(self):
        # Feedback says the real wrist is already moving into the envelope.
        self.hw['position'][5:7] = np.radians([40., 30.]).tolist()
        self.hw['velocity'][5:7] = np.radians([100., 100.]).tolist()
        with self.assertRaises(ValueError):
            self.request()

    def test_inward_motion_is_checked_from_lagging_measured_wrist(self):
        self.hw['position'][5:7] = np.radians([40., 30.]).tolist()
        self.hw['command_position'][5:7] = np.radians([35., 25.]).tolist()
        self.hw['velocity'][5:7] = np.radians([-30., -30.]).tolist()
        q = self.hw['command_position'][:7]
        q[5:7] = np.radians([30., 20.]).tolist()
        goal = self.tron.solver.forward('left', q)
        packet = self.request({'left': goal})
        target = packet['arms']['left']['target']
        for start in (self.hw['position'][:7], self.hw['command_position'][:7]):
            WristGuard().arm_segment(start, target, self.hw['velocity'][:7])

    def test_bad_second_arm_preserves_input_and_never_returns_partial_packet(self):
        goals = {s: self.goal(s) for s in ARMS}
        seeds = {s: [self.hw['command_position'][j-1] for j in js] for s, js in ARMS.items()}
        results = self.tron.solver.solve(goals, seeds)
        results['right']['tcp'][0, 3] += 1.
        before = copy.deepcopy(self.hw)
        with patch.object(self.tron.solver, 'solve', return_value=results):
            with self.assertRaisesRegex(ValueError, '正解校验'):
                self.request(goals)
        self.assertEqual(self.hw, before)

    def test_client_vendor_fk_is_independently_checked(self):
        goal = self.goal()
        incorrect = goal.copy()
        incorrect[0, 3] += 100.
        with patch.object(self.kin, 'forward_tcp', return_value=(incorrect, 0.)):
            with self.assertRaisesRegex(ValueError, '正解校验'):
                self.request({'left': goal})

    def test_solved_target_is_stationary_and_no_cycle_speed_cap_added(self):
        goal = self.goal(shift=20.)
        packet = self.request({'left': goal})
        entry = packet['arms']['left']
        self.assertTrue(entry['native_ik']['success'])
        self.assertGreater(np.max(np.abs(np.array(entry['target'])-self.hw['command_position'][:7]))/.01,
                           math.radians(10.))
        q = entry['target'][:]
        self.hw['position'][:7] = self.hw['command_position'][:7] = q[:]
        for i in range(20):
            packet = self.tron.request(self.kin, self.hw, {'left': goal}, i+2, 100., references={'left': q})
            self.assertEqual(packet['arms']['left']['target'], q)

    def test_reference_is_explicit_and_never_mutates_on_failure(self):
        first = self.request()
        q = first['arms']['left']['target'][:]
        references = {'left': q[:]}
        packet = self.request(references=references)
        self.assertEqual(packet['arms']['left']['reference'], q)
        self.assertEqual(references, {'left': q})
        with patch.object(self.tron.solver, 'solve', side_effect=ValueError('QP test error')):
            with self.assertRaisesRegex(ValueError, 'QP test error'):
                self.request(references=references)
        self.assertEqual(references, {'left': q})

    def test_tool_mismatch_and_nonfinite_feedback_are_refused(self):
        with self.assertRaisesRegex(ValueError, '配置不一致'):
            self.tron.request(VendorKinematics(tools=tool_config('taccap')), self.hw,
                              {'left': self.goal()}, 1, 100.)
        for name in ('position', 'command_position', 'velocity'):
            hw = copy.deepcopy(self.hw)
            hw[name][0] = float('nan')
            with self.assertRaises(ValueError):
                self.tron.request(self.kin, hw, {'left': self.goal()}, 1, 100.)

    def test_preflight_model_mismatch_is_rejected(self):
        bad = np.eye(4)
        with patch('tools.ik_experiments.tron2_art7.Art7TronIK.forward', return_value=bad):
            with self.assertRaisesRegex(ValueError, '启动模型正解不一致'):
                TronIK(tool_config())

    def test_preflight_nonfinite_model_is_rejected(self):
        with patch('tools.ik_experiments.tron2_art7.Art7TronIK.forward', return_value=np.full((4, 4), np.nan)):
            with self.assertRaisesRegex(ValueError, 'finite'):
                TronIK(tool_config())


if __name__ == '__main__':
    unittest.main()
