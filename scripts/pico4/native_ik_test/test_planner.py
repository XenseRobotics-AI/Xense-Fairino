"""Offline full-target IK and transactional position-publication contracts."""
import copy
import json
import math
import unittest

import numpy as np

from scripts.pico4.native_ik_test.planner import NativePlanner
from tcp_core import tcp_forward_matrix, multiply, pose_error
from tool_frames import for_arm


def hardware():
    left = np.radians([45., 75., -100., 85., 85., 0., 0.])
    right = left*np.array([-1., 1., -1., 1., -1., 1., -1.])
    q = np.r_[left, right, np.zeros(6)].tolist()
    return dict(position=q[:], command_position=q[:], velocity=[0.]*20)


def follow(hw, proposal):
    hw = copy.deepcopy(hw)
    for side, offset in (('left', 0), ('right', 7)):
        hw['position'][offset:offset+7] = proposal['arms'][side]['target'][:]
        hw['command_position'][offset:offset+7] = proposal['arms'][side]['target'][:]
        hw['velocity'][offset:offset+7] = proposal['results'][side]['velocity'][:]
    return hw


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.hw = hardware()
        self.planner = NativePlanner()
        self.planner.reset(self.hw)

    def goal(self, side='left', offset=10.):
        goal = self.planner.solvers[side].forward(self.planner.references[side])
        goal[0, 3] += offset
        return goal

    def test_reset_is_required_and_initial_state_must_be_settled(self):
        planner = NativePlanner()
        with self.assertRaises(ValueError):
            planner.propose({}, self.hw, .01)
        moving = copy.deepcopy(self.hw); moving['velocity'][0] = .01
        with self.assertRaises(ValueError):
            planner.reset(moving)
        self.assertFalse(planner.initialized)

    def test_reset_rejects_raw_safe_but_reserved_invalid_wrist(self):
        bad = copy.deepcopy(self.hw)
        bad['position'][5:7] = bad['command_position'][5:7] = np.radians([69., 30.]).tolist()
        before = self.planner.references
        with self.assertRaises(ValueError):
            self.planner.reset(bad)
        for side in before:
            np.testing.assert_array_equal(before[side], self.planner.references[side])

    def test_full_target_is_solved_and_missing_arm_holds(self):
        proposal = self.planner.propose({'left': self.goal(offset=20.)}, self.hw, .01)
        self.assertEqual(set(proposal['arms']), {'left', 'right'})
        self.assertTrue(proposal['results']['left']['success'])
        self.assertTrue(proposal['results']['left']['feasible'])
        self.assertTrue(proposal['results']['right']['command_stopped'])
        self.assertEqual(proposal['results']['right']['solve_mode'], 'hold_last_command')
        np.testing.assert_array_equal(proposal['arms']['right']['target'], self.hw['command_position'][7:14])
        json.dumps(proposal, allow_nan=False)
        for side, arm in proposal['arms'].items():
            independent = multiply(tcp_forward_matrix(arm['target']), for_arm(self.planner.tools, side))
            mm, deg = pose_error(np.asarray(arm['tcp']).reshape(4, 4), independent)
            self.assertLess(mm, 1e-9); self.assertLess(deg, 1e-5)

    def test_uncommitted_proposals_never_advance_history(self):
        goals = {'left': self.goal()}; original = self.planner.references
        first = self.planner.propose(goals, self.hw, .01)
        second = self.planner.propose(goals, self.hw, .01)
        self.assertNotEqual(first['proposal_id'], second['proposal_id'])
        for side in original:
            np.testing.assert_array_equal(self.planner.references[side], original[side])
            np.testing.assert_array_equal(self.planner.previous_velocities[side], np.zeros(7))
            np.testing.assert_allclose(first['arms'][side]['target'], second['arms'][side]['target'], atol=1e-12)
        with self.assertRaises(ValueError):
            self.planner.commit(first)
        self.planner.commit(second)
        for side in original:
            np.testing.assert_allclose(self.planner.references[side], second['arms'][side]['target'], atol=0.)
        with self.assertRaises(ValueError):
            self.planner.commit(second)

    def test_tampering_is_rejected_without_advancing_either_arm(self):
        proposal = self.planner.propose({'left': self.goal()}, self.hw, .01)
        changed = copy.deepcopy(proposal); changed['arms']['right']['target'][0] += .001
        with self.assertRaises(ValueError):
            self.planner.commit(changed)
        for side in self.planner.references:
            np.testing.assert_array_equal(self.planner.previous_velocities[side], np.zeros(7))
        self.planner.commit(proposal)

    def test_second_arm_failure_does_not_advance_first_arm(self):
        bad = copy.deepcopy(self.hw)
        bad['position'][12:14] = np.radians([69., 80.]).tolist()
        original = self.planner.references
        with self.assertRaises(ValueError):
            self.planner.propose({'left': self.goal(), 'right': self.goal('right')}, bad, .01)
        for side in original:
            np.testing.assert_array_equal(self.planner.references[side], original[side])
            np.testing.assert_array_equal(self.planner.previous_velocities[side], np.zeros(7))

    def test_reference_comes_from_commit_not_late_command_feedback(self):
        goal = self.goal()
        first = self.planner.propose({'left': goal}, self.hw, .01)
        self.planner.commit(first)
        second = self.planner.propose({'left': goal}, self.hw, .01)
        np.testing.assert_array_equal(second['arms']['left']['reference'], first['arms']['left']['target'])

    def test_release_republishes_exact_position_even_if_measured_wrist_is_moving(self):
        proposal = self.planner.propose({'left': self.goal(offset=20.)}, self.hw, .01)
        self.planner.commit(proposal)
        endpoint = self.planner.references
        moving = copy.deepcopy(self.hw)
        moving['velocity'][5:7] = np.radians([150., 150.]).tolist()
        for _ in range(5):
            hold = self.planner.propose({}, moving, .01)
            self.assertTrue(hold['command_stopped'])
            for side in endpoint:
                np.testing.assert_array_equal(hold['arms'][side]['target'], endpoint[side])
                np.testing.assert_array_equal(hold['results'][side]['velocity'], np.zeros(7))
                self.assertFalse(hold['results'][side]['requested'])
            self.planner.commit(hold)

    def test_timing_does_not_clip_the_full_target_or_slow_it_over_multiple_ticks(self):
        goal = self.goal(offset=20.)
        quick = self.planner.propose({'left': goal}, self.hw, .001)
        slow = self.planner.propose({'left': goal}, self.hw, .05)
        np.testing.assert_array_equal(quick['arms']['left']['target'], slow['arms']['left']['target'])
        self.assertTrue(quick['results']['left']['success'])
        self.assertGreater(max(abs(v) for v in quick['results']['left']['velocity']), math.radians(10.))
        self.planner.commit(slow)
        # Simulate measured arrival with zero motor velocity. Repeated fixed
        # input must preserve the exact solution, not integrate a velocity tail.
        settled = follow(self.hw, slow)
        settled['velocity'] = [0.]*20
        for _ in range(20):
            repeated = self.planner.propose({'left': goal}, settled, .01)
            self.assertTrue(repeated['results']['left']['success'])
            np.testing.assert_array_equal(repeated['arms']['left']['target'], slow['arms']['left']['target'])
            self.assertTrue(repeated['command_stopped'])
            self.planner.commit(repeated)

    def test_original_wrist_prediction_rejects_outward_motion_but_allows_inward_retreat(self):
        hw = copy.deepcopy(self.hw)
        hw['position'][5:7] = hw['command_position'][5:7] = np.radians([40., 30.]).tolist()
        self.planner.reset(hw)
        inward_q = self.planner.references['left']
        inward_q[-2:] = np.radians([30., 20.])
        goal = self.planner.solvers['left'].forward(inward_q)
        hw['velocity'][5:7] = np.radians([100., 100.]).tolist()
        with self.assertRaises(ValueError):
            self.planner.propose({'left': goal}, hw, .01)

        hw['velocity'][5:7] = np.radians([-100., -100.]).tolist()
        proposal = self.planner.propose({'left': goal}, hw, .01)
        self.assertTrue(proposal['results']['left']['feasible'])
        target = np.asarray(proposal['arms']['left']['target'])
        self.assertTrue(np.all(np.abs(target[-2:]) <= np.radians([40., 30.])+1e-10))
        self.assertGreaterEqual(self.planner.guard.clearance(target[-2:].tolist(), margin=0.), 0.)
        # The same retreat exception never legalizes a raw collision state.
        hw['position'][5:7] = np.radians([69., 80.]).tolist()
        with self.assertRaises(ValueError):
            self.planner.propose({'left': goal}, hw, .01)

    def test_unreachable_goal_keeps_full_residual_and_never_claims_success(self):
        goal = self.goal(offset=1000.)
        proposal = self.planner.propose({'left': goal}, self.hw, .01)
        result = proposal['results']['left']
        self.assertTrue(result['feasible'])
        self.assertFalse(result['success'])
        achieved = np.asarray(proposal['arms']['left']['tcp']).reshape(4, 4)
        mm, deg = pose_error(goal, achieved)
        self.assertGreater(mm, 100.)
        self.assertAlmostEqual(result['position_error_mm'], mm)
        self.assertAlmostEqual(result['orientation_error_deg'], deg)

    def test_stale_commit_cannot_survive_reset(self):
        old = self.planner.propose({}, self.hw, .01)
        self.planner.reset(self.hw)
        new = self.planner.propose({}, self.hw, .01)
        self.assertGreater(new['proposal_id'], old['proposal_id'])
        with self.assertRaises(ValueError):
            self.planner.commit(old)
        self.planner.commit(new)

    def test_invalid_inputs_and_configuration_are_rejected(self):
        for dt in (0., -1., math.inf, math.nan):
            with self.assertRaises(ValueError):
                self.planner.propose({}, self.hw, dt)
        with self.assertRaises(ValueError):
            self.planner.propose({'head': np.eye(4)}, self.hw, .01)
        goal = self.goal(); goal[0, 0] = math.nan
        with self.assertRaises(ValueError):
            self.planner.propose({'left': goal}, self.hw, .01)
        with self.assertRaises(TypeError):
            NativePlanner(speed_deg_s=10.)


if __name__ == '__main__':
    unittest.main()
