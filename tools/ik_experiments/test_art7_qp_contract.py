"""Offline API/dynamic-bound regressions for the experimental ART7 solver.

Run from the repository root:
    PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B tools/ik_experiments/test_art7_qp_contract.py

No robot transport or vendor IK is imported. Independent FK and the existing
wrist guard are used to check returned commands, not the solver's status alone.
"""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(Path(__file__).resolve().parent),
               str(ROOT / 'apps/fairino_tcp_probe'),
               str(ROOT / 'apps/fairino_upper_probe')]

import numpy as np

from art7_qp import Art7ConstrainedIK
from tcp_core import BOUNDS_DEG, multiply, pose_error, tcp_forward_matrix
from tool_frames import tool_config, for_arm
from wrist_guard import WristGuard, WristBlocked


HOME = np.radians([45., 75., -100., 85., 85., 0., 0.])
NEAR_WRIST = np.radians([45., 75., -100., 85., 135., 51., 37.2])
VMAX = math.radians(30.)
AMAX = math.radians(120.)


class SolverContractTests(unittest.TestCase):
    def setUp(self):
        self.tools = tool_config('flange')
        self.solver = Art7ConstrainedIK(tools=self.tools, side='left')
        self.guard = WristGuard()

    def assert_valid_result(self, result, reference, goal, *, measured=None,
                            measured_velocity=None, custom_bounds=None):
        self.assertIsInstance(result, dict)
        self.assertTrue(result['feasible'], result)
        q = np.asarray(result['target'])
        self.assertEqual(q.shape, (7,))
        self.assertTrue(np.isfinite(q).all())
        bounds = np.radians(np.asarray(BOUNDS_DEG if custom_bounds is None else custom_bounds))
        self.assertTrue(np.all(q >= bounds[:, 0] - 1e-9))
        self.assertTrue(np.all(q <= bounds[:, 1] + 1e-9))
        measured = np.asarray(reference if measured is None else measured)
        velocity = np.zeros(7) if measured_velocity is None else np.asarray(measured_velocity)
        self.guard.arm_segment(np.asarray(reference).tolist(), q.tolist(), velocity.tolist())
        self.guard.arm_segment(measured.tolist(), q.tolist(), velocity.tolist())
        independent = multiply(tcp_forward_matrix(q.tolist()), for_arm(self.tools, 'left'))
        mm, deg = pose_error(independent, np.asarray(result['tcp']).tolist())
        self.assertLess(mm, 1e-7)
        self.assertLess(deg, 1e-4)
        mm, deg = pose_error(np.asarray(goal).tolist(), independent)
        self.assertAlmostEqual(mm, result['position_error_mm'], delta=1e-7)
        self.assertAlmostEqual(deg, result['orientation_error_deg'], delta=1e-4)
        if result['success']:
            self.assertLessEqual(mm, .0200001)
            self.assertLessEqual(deg, .0100001)
        return q

    def assert_dynamic_bounds(self, q, reference, previous_velocity, dt):
        # The final displacement is measured from the ORIGINAL frame start.
        # Internal solver iterations must never reset this time budget.
        delta = q - reference
        lower = np.maximum(-VMAX * dt, (previous_velocity - AMAX * dt) * dt)
        upper = np.minimum(VMAX * dt, (previous_velocity + AMAX * dt) * dt)
        self.assertTrue(np.all(delta >= lower - 1e-9), (delta, lower))
        self.assertTrue(np.all(delta <= upper + 1e-9), (delta, upper))

    def test_internal_iterations_share_one_control_period(self):
        reference = HOME.copy()
        goal = self.solver.forward(reference).copy()
        goal[0, 3] += 20.
        moved = False
        for dt in (1 / 450, 1 / 90):
            for iterations in (1, 8, 40):
                with self.subTest(dt=dt, iterations=iterations):
                    result = self.solver.step(goal, reference, dt,
                                              np.zeros(7), max_iterations=iterations)
                    q = self.assert_valid_result(result, reference, goal)
                    self.assert_dynamic_bounds(q, reference, np.zeros(7), dt)
                    self.assertFalse(result['success'])
                    moved |= bool(np.max(np.abs(q - reference)) > 1e-10)
        self.assertTrue(moved, 'A solver that always holds cannot pass the progress contract.')

    def test_nonzero_previous_velocity_cannot_be_reset_by_hold_target(self):
        reference = HOME.copy()
        goal = self.solver.forward(reference)
        dt = 1 / 450
        previous = np.full(7, .2)  # rad/s; cannot stop within this dt at 120 deg/s^2.
        result = self.solver.step(goal, reference, dt, previous, max_iterations=20)
        q = self.assert_valid_result(result, reference, goal)
        self.assert_dynamic_bounds(q, reference, previous, dt)
        self.assertTrue(np.all(q - reference > 0.))

    def test_empty_braking_intersection_is_not_reported_as_hold(self):
        reference = HOME.copy()
        reference[0] = math.radians(174.999)
        previous = np.zeros(7)
        previous[0] = .3
        result = self.solver.step(self.solver.forward(reference), reference,
                                  .01, previous, max_iterations=20)
        self.assertFalse(result['feasible'], result)
        self.assertFalse(result['success'], result)
        self.assertIsNone(result['target'])
        self.assertTrue(result['reason'])

    def test_fixed_tcp_is_compared_to_original_goal_for_every_frame(self):
        # Do not rebase goal to FK(last_command): doing so hides accumulated drift.
        q = NEAR_WRIST.copy()
        goal = self.solver.forward(q).copy()
        original_goal = goal.copy()
        dt = 1 / 450
        previous = np.zeros(7)
        for frame in range(100):
            result = self.solver.step(goal, q, dt, previous, max_iterations=8)
            next_q = self.assert_valid_result(result, q, original_goal)
            self.assert_dynamic_bounds(next_q, q, previous, dt)
            self.assertTrue(result['success'], (frame, result))
            previous = (next_q - q) / dt
            q = next_q
        np.testing.assert_array_equal(goal, original_goal)

    def test_fresh_measured_path_is_checked_even_when_goal_equals_reference(self):
        reference = NEAR_WRIST.copy()
        measured = reference.copy()
        measured[6] = math.radians(37.8)
        velocity = np.zeros(7)
        velocity[5] = math.radians(10.)  # Outward; inward-retreat exception is unavailable.
        with self.assertRaises(WristBlocked):
            self.guard.arm_segment(measured.tolist(), reference.tolist(), velocity.tolist())
        result = self.solver.step(self.solver.forward(reference), reference,
                                  1 / 450, np.zeros(7), measured=measured,
                                  measured_velocity=velocity)
        self.assertFalse(result['feasible'], result)
        self.assertFalse(result['success'], result)
        self.assertIsNone(result['target'])

    def test_initial_reserved_region_violation_never_uses_manual_retreat_exception(self):
        outside = NEAR_WRIST.copy()
        outside[6] = math.radians(37.8)
        self.assertGreaterEqual(self.guard.clearance(outside[-2:].tolist(), margin=0.), 0.)
        self.assertLess(self.guard.clearance(outside[-2:].tolist()), 0.)
        # Existing guard allows a stationary command in the static reserve band;
        # this IK intentionally requires an admissible initial state instead.
        self.guard.arm_segment(outside.tolist(), outside.tolist(), np.zeros(7).tolist())
        for reference, measured in ((outside, outside), (NEAR_WRIST, outside)):
            with self.subTest(reference_outside=bool(np.array_equal(reference, outside))):
                result = self.solver.step(self.solver.forward(reference), reference,
                                          1 / 450, np.zeros(7), measured=measured)
                self.assertFalse(result['feasible'], result)
                self.assertFalse(result['success'], result)
                self.assertIsNone(result['target'])
                self.assertEqual(result['reason'], 'initial_wrist_state_infeasible')

        # Inward measured velocity permits the old manual-retreat exception,
        # but its speed-dependent reserve can still make this initial state
        # inadmissible to the new solver. It must reject rather than auto-escape.
        inward = np.zeros(7)
        inward[5:] = np.radians([-15., -5.])
        reserve = tuple(np.abs(np.degrees(inward[5:])) * .2)
        self.assertLess(self.guard.clearance(NEAR_WRIST[-2:].tolist(), reserve=reserve), 0.)
        self.guard.arm_segment(NEAR_WRIST.tolist(), NEAR_WRIST.tolist(), inward.tolist())
        result = self.solver.step(self.solver.forward(NEAR_WRIST), NEAR_WRIST,
                                  1 / 450, np.zeros(7), measured_velocity=inward)
        self.assertFalse(result['feasible'], result)
        self.assertFalse(result['success'], result)
        self.assertIsNone(result['target'])
        self.assertEqual(result['reason'], 'initial_wrist_state_infeasible')

    def test_exact_staircase_boundary_and_neighbors_never_bypass_guard(self):
        for sixth in (0., 2., 4., 6., 10., 30., 50., 52., 60., 68.):
            for offset in (-1e-7, 0., 1e-7):
                if sixth + offset < 0:
                    continue
                q = np.radians([45., 75., -100., 85., 135., sixth + offset, 0.])
                # Keep y near this EXACT state's step cap, rather than assuming
                # perturbations across a lookup jump retain the same feasible y.
                from wrist_guard import cap
                cap_deg = cap(abs(math.degrees(q[5])) + 2.) - 2.
                if not math.isfinite(cap_deg) or cap_deg < 0.:
                    continue
                q[6] = math.radians(max(0., cap_deg - .05))
                goal = self.solver.forward(q)
                with self.subTest(sixth=sixth, offset=offset):
                    result = self.solver.step(goal, q, 1 / 450, np.zeros(7))
                    target = self.assert_valid_result(result, q, goal)
                    self.assert_dynamic_bounds(target, q, np.zeros(7), 1 / 450)
                    self.assertTrue(result['success'])

    def test_custom_position_limits_are_enforced_and_seed_is_not_clipped(self):
        bounds = np.asarray(BOUNDS_DEG).copy()
        bounds[5:] = [-40., 40.]
        solver = Art7ConstrainedIK(tools=self.tools, bounds_deg=bounds)
        seed = np.radians([45., 75., -100., 85., 135., 39.8, 39.8])
        goal = solver.forward(seed).copy()
        goal[0, 3] -= 2.
        result = solver.solve(goal, seed, max_iterations=80)
        self.assert_valid_result(result, seed, goal, custom_bounds=bounds)
        outside = seed.copy()
        outside[5] = math.radians(40.1)
        snapshot = outside.copy()
        with self.assertRaises(ValueError):
            solver.solve(solver.forward(outside), outside)
        np.testing.assert_array_equal(outside, snapshot)

    def test_invalid_pose_is_rejected_before_any_candidate_is_returned(self):
        valid = self.solver.forward(HOME)
        invalid = []
        for value in (float('nan'), float('inf')):
            pose = valid.copy()
            pose[0, 3] = value
            invalid.append(pose)
        pose = valid.copy(); pose[:3, 0] *= 2.; invalid.append(pose)
        pose = valid.copy(); pose[:3, 0] *= -1.; invalid.append(pose)
        pose = valid.copy(); pose[3, 3] = 2.; invalid.append(pose)
        invalid.append(np.eye(3))
        for index, goal in enumerate(invalid):
            with self.subTest(index=index):
                with self.assertRaises(ValueError):
                    self.solver.solve(goal, HOME)
                with self.assertRaises(ValueError):
                    self.solver.step(goal, HOME, 1 / 450, np.zeros(7))

    def test_invalid_time_state_and_bounds_are_explicitly_rejected(self):
        goal = self.solver.forward(HOME)
        for dt in (0., -1., float('nan'), float('inf')):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                self.solver.step(goal, HOME, dt, np.zeros(7))
        for state in (np.zeros(6), np.full(7, np.nan), np.full(7, np.inf)):
            with self.assertRaises(ValueError):
                self.solver.solve(goal, state)
            with self.assertRaises(ValueError):
                self.solver.step(goal, HOME, 1 / 450, state)
        for invalid_bounds in (np.zeros((6, 2)), np.full((7, 2), np.nan)):
            with self.assertRaises(ValueError):
                Art7ConstrainedIK(tools=self.tools, bounds_deg=invalid_bounds)
        crossed = np.asarray(BOUNDS_DEG).copy(); crossed[0] = [2., 1.]
        with self.assertRaises(ValueError):
            Art7ConstrainedIK(tools=self.tools, bounds_deg=crossed)

    def test_half_turn_target_does_not_look_like_zero_rotation_error(self):
        goal = self.solver.forward(HOME).copy()
        goal[:3, :3] = np.diag([1., -1., -1.]) @ goal[:3, :3]
        result = self.solver.step(goal, HOME, 1 / 450, np.zeros(7), max_iterations=8)
        q = self.assert_valid_result(result, HOME, goal)
        self.assert_dynamic_bounds(q, HOME, np.zeros(7), 1 / 450)
        self.assertFalse(result['success'])
        self.assertGreater(result['orientation_error_deg'], 179.)

    def test_solver_does_not_modify_callers_inputs_or_reuse_rejected_state(self):
        q = HOME.copy()
        goal = self.solver.forward(q).copy()
        previous = np.zeros(7)
        snapshots = [value.copy() for value in (q, goal, previous)]
        result = self.solver.step(goal, q, 1 / 450, previous)
        self.assert_valid_result(result, q, goal)
        for value, expected in zip((q, goal, previous), snapshots):
            np.testing.assert_array_equal(value, expected)
        invalid = goal.copy(); invalid[0, 3] = float('nan')
        with self.assertRaises(ValueError):
            self.solver.solve(invalid, q)
        again = self.solver.step(goal, q, 1 / 450, previous)
        self.assert_valid_result(again, q, goal)
        np.testing.assert_allclose(again['target'], result['target'], atol=1e-12, rtol=0.)


if __name__ == '__main__':
    unittest.main(verbosity=2)
