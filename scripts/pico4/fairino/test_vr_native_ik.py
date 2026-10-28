"""Offline acceptance tests for native IK in the production PICO protocol.

Uses the real C++ solver and independent/vendor FK; never opens a robot, ROS,
PICO SDK, or network connection.
"""
import copy
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import paths
import numpy as np

from tcp_core import ARMS, multiply, pose_error, target_matrix, tcp_forward_matrix
from tool_frames import for_arm, tool_config
from vendor_kinematics import VendorKinematics
from vr_cartesian import CartesianInput
from vr_core import validate, direct_positions
from vr_native_ik import NativeIK
from wrist_guard import WristGuard


def hardware():
    left = np.radians([45., 75., -100., 85., 85., 0., 0.])
    right = left*np.array([-1., 1., -1., 1., -1., 1., -1.])
    q = np.r_[left, right, np.zeros(6)].tolist()
    return dict(position=q[:], command_position=q[:], velocity=[0.]*20)


class NativeMainChainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = NativeIK()
        cls.kin = VendorKinematics()

    def setUp(self):
        self.hw = hardware()

    def goal(self, side='left', offset=20.):
        q = [self.hw['command_position'][j-1] for j in ARMS[side]]
        goal = np.asarray(self.kin.forward_tcp(q, side)[0])
        goal[0, 3] += offset
        return goal

    def request(self, goals=None, **kwargs):
        return self.native.request(self.kin, self.hw,
            {'left': self.goal()} if goals is None else goals, 1, 100., **kwargs)

    def result_for(self, q, success=False):
        return dict(feasible=True, success=success, target=np.asarray(q),
                    tcp=self.native.solvers['left'].forward(q), status='partial_progress',
                    iterations=1, qp_calls=1, elapsed_ms=1.)

    def test_real_solver_covers_both_tools_arms_and_cartesian_directions(self):
        for mode in ('flange', 'taccap'):
            tools = tool_config(mode)
            native, kin = NativeIK(tools), VendorKinematics(tools=tools)
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
                        with patch.object(kin, 'inverse_tcp', side_effect=AssertionError('vendor IK used')):
                            packet = native.request(kin, self.hw, goals, 1, 100.)
                        self.assertEqual(set(packet['arms']), set(ARMS))
                        self.assertEqual(packet['tool_id'], tools['id'])
                        self.assertEqual(validate(self.hw, packet, 100., tools),
                                         {s: a['target'] for s, a in packet['arms'].items()})
                        json.dumps(packet, allow_nan=False)
                        for side, arm in packet['arms'].items():
                            self.assertTrue(arm['native_ik']['success'], arm['native_ik'])
                            independent = multiply(tcp_forward_matrix(arm['target']), for_arm(tools, side))
                            mm, deg = pose_error(goals[side], independent)
                            self.assertLessEqual(mm, .02); self.assertLessEqual(deg, .01)
                            self.assertNotIn('wrist_scale', arm)

    def test_single_arm_omits_released_arm_and_receiver_holds_it_exactly(self):
        self.hw['velocity'][12:14] = np.radians([150., 150.]).tolist()
        with patch.object(self.native.solvers['right'], 'solve', side_effect=AssertionError('released arm solved')):
            packet = self.request()
        self.assertEqual(set(packet['arms']), {'left'})
        held = self.hw['command_position'][:]
        held[7] += .01  # Receiver history may be newer than PC feedback.
        targets = validate(self.hw, packet, 100.)
        points = direct_positions(held, targets, held)
        self.assertEqual(points[-1][1][7:], held[7:])
        with patch.object(self.native.solvers['left'], 'solve', side_effect=AssertionError('empty input solved')):
            self.assertIsNone(self.request({}))

    def test_partial_answer_retains_full_goal_without_rebasing_input(self):
        goal = self.goal(offset=1000.)
        packet = self.request({'left': goal})
        entry = packet['arms']['left']; result = entry['native_ik']
        self.assertTrue(result['feasible']); self.assertFalse(result['success'])
        achieved = np.asarray(entry['tcp']).reshape(4, 4)
        mm, deg = pose_error(goal, achieved)
        self.assertGreater(mm, 100.)
        self.assertAlmostEqual(result['position_error_mm'], mm)
        self.assertAlmostEqual(result['orientation_error_deg'], deg)
        np.testing.assert_array_equal(np.asarray(result['requested_tcp']).reshape(4, 4), goal)
        clutches = {side: Mock() for side in ARMS}
        cartesian = CartesianInput(clutches)
        cartesian.buffer.push('left', 10., goal)
        cartesian.accepted_plan(packet, frame={'left': {}, 'right': {}},
                                hands={'left': 'left', 'right': 'right'})
        clutches['left'].rebase.assert_not_called()
        np.testing.assert_allclose(cartesian.buffer.sample(10.)['left'], goal, rtol=0., atol=1e-12)

    def test_fixed_solved_target_has_no_integration_tail(self):
        goal = self.goal()
        packet = self.request({'left': goal})
        endpoint = packet['arms']['left']['target'][:]
        self.hw['position'][:7] = self.hw['command_position'][:7] = endpoint[:]
        for index in range(20):
            packet = self.native.request(self.kin, self.hw, {'left': goal}, index+2, 100.,
                                         references={'left': endpoint})
            self.assertTrue(packet['arms']['left']['native_ik']['success'])
            self.assertEqual(packet['arms']['left']['target'], endpoint)

    def test_full_target_is_not_clipped_by_cycle_speed_or_acceleration(self):
        solver = self.native.solvers['left']
        with patch.object(solver, 'solve', wraps=solver.solve) as solve, \
             patch.object(solver, 'step', side_effect=AssertionError('bounded step used')):
            packet = self.request()
        self.assertTrue(packet['arms']['left']['native_ik']['success'])
        difference = np.asarray(packet['arms']['left']['target'])-self.hw['command_position'][:7]
        self.assertGreater(np.max(np.abs(difference))/.01, math.radians(10.))
        self.assertEqual(set(solve.call_args.kwargs),
                         {'measured', 'measured_velocity', 'allow_guard_retreat'})
        baseline = packet['arms']['left']['target']
        self.hw['velocity'][0] = math.radians(90.)
        self.assertEqual(self.request()['arms']['left']['target'], baseline)

    def test_two_arm_failure_returns_no_packet_and_does_not_advance_state(self):
        goal = self.goal(); expected = self.request()['arms']['left']['target']
        before = copy.deepcopy(self.hw)
        with patch.object(self.native.solvers['right'], 'solve',
                          return_value={'feasible': False, 'status': 'no_feasible_candidate'}):
            with self.assertRaisesRegex(ValueError, 'complete request'):
                self.request({'left': goal, 'right': self.goal('right')})
        self.assertEqual(self.hw, before)
        self.assertEqual(self.request()['arms']['left']['target'], expected)

    def test_references_are_explicit_and_not_implicitly_committed(self):
        first = self.request(); target = first['arms']['left']['target']
        repeated = self.request()
        self.assertEqual(repeated['arms']['left']['reference'], self.hw['command_position'][:7])
        self.assertEqual(repeated['arms']['left']['target'], target)
        planned = self.request(references={'left': target})
        self.assertEqual(planned['arms']['left']['reference'], target)
        self.assertEqual(planned['arms']['left']['target'], target)

    def test_both_command_and_measured_paths_are_independently_checked(self):
        reference = self.hw['command_position'][:7]
        self.hw['position'][0] += .001
        guard = WristGuard()
        with patch.object(guard, 'arm_segment', wraps=guard.arm_segment) as check:
            packet = self.request(references={'left': reference}, guard=guard)
        self.assertEqual(check.call_count, 2)
        self.assertEqual(check.call_args_list[0].args[0], reference)
        self.assertEqual(check.call_args_list[1].args[0], self.hw['position'][:7])
        self.assertEqual(check.call_args_list[1].args[1], packet['arms']['left']['target'])

    def test_original_prediction_rejects_outward_motion_and_permits_inward_retreat(self):
        self.hw['position'][5:7] = self.hw['command_position'][5:7] = np.radians([40., 30.]).tolist()
        inward = self.hw['command_position'][:7]
        inward[-2:] = np.radians([30., 20.]).tolist()
        goal = self.native.solvers['left'].forward(inward)
        self.hw['velocity'][5:7] = np.radians([100., 100.]).tolist()
        with self.assertRaises(ValueError):
            self.request({'left': goal})
        self.hw['velocity'][5:7] = np.radians([-100., -100.]).tolist()
        packet = self.request({'left': goal})
        q = packet['arms']['left']['target']
        self.assertTrue(np.all(np.abs(q[-2:]) <= np.radians([40., 30.])+1e-10))

    def test_partial_result_cannot_claim_success_or_move_away_from_goal(self):
        q = self.hw['command_position'][:7]
        with patch.object(self.native.solvers['left'], 'solve', return_value=self.result_for(q, True)):
            with self.assertRaisesRegex(ValueError, 'incorrectly reported'):
                self.request()
        goal = self.goal(offset=0.)
        q[0] += .1
        with patch.object(self.native.solvers['left'], 'solve', return_value=self.result_for(q)):
            with self.assertRaisesRegex(ValueError, 'moved away'):
                self.request({'left': goal})

    def test_independent_fk_detects_corrupted_native_tcp(self):
        result = self.result_for(self.hw['command_position'][:7])
        result['tcp'][0, 3] += 1.
        with patch.object(self.native.solvers['left'], 'solve', return_value=result):
            with self.assertRaisesRegex(ValueError, 'independent TCP FK'):
                self.request()

    def test_independent_fk_detects_wrong_client_tool_transform(self):
        bad = self.goal(offset=100.)
        with patch.object(self.kin, 'forward_tcp', return_value=(bad, 0.)):
            # Supply the original goal explicitly so only validation is forged.
            goal = self.native.solvers['left'].forward(self.hw['command_position'][:7])
            with self.assertRaisesRegex(ValueError, 'independent TCP FK'):
                self.request({'left': goal})

    def test_tool_mismatch_and_disabled_guard_are_explicitly_rejected(self):
        other = VendorKinematics(tools=tool_config('taccap'))
        with self.assertRaisesRegex(ValueError, 'tool configurations'):
            self.native.request(other, self.hw, {'left': self.goal()}, 1, 100.)
        with self.assertRaisesRegex(ValueError, 'remain enabled'):
            self.request(guard=WristGuard(False))

    def test_invalid_goals_hardware_references_and_identifiers_are_rejected(self):
        with self.assertRaises(ValueError):
            self.request({'head': np.eye(4)})
        broken = self.goal(); broken[0, 0] = math.nan
        with self.assertRaises(ValueError):
            self.request({'left': broken})
        broken = self.goal(); broken[3, 0] = 1.
        with self.assertRaises(ValueError):
            self.request({'left': broken})
        with self.assertRaises(ValueError):
            self.request(references={'left': [0.] * 6})
        for sequence, stamp in ((0, 100.), (True, 100.), (1, math.nan)):
            with self.subTest(sequence=sequence, stamp=stamp), self.assertRaises(ValueError):
                self.native.request(self.kin, self.hw, {'left': self.goal()}, sequence, stamp)
        self.hw['velocity'][19] = math.nan
        with self.assertRaises(ValueError):
            self.request()


if __name__ == '__main__':
    unittest.main()
