"""Independent numerical checks for the original offline ART7 experiment."""
import itertools
import math
import unittest

import numpy as np

from tools.ik_experiments.art7_qp import (Art7ConstrainedIK, solve_box_qp,
                                       rotation_log, BOUNDS_DEG)
from tcp_core import tcp_forward_matrix, multiply, pose_error
from tool_frames import tool_config, for_arm


class BoxQPTests(unittest.TestCase):
    def test_matches_exhaustive_active_sets(self):
        rng = np.random.default_rng(20261026)
        for _ in range(120):
            a = rng.normal(size=(3, 3)); h = a.T @ a+.1*np.eye(3)
            g = rng.normal(size=3)*4
            lo = rng.uniform(-2., -.1, 3); hi = rng.uniform(.1, 2., 3)
            actual = solve_box_qp(h, g, lo, hi)
            best = math.inf
            for status in itertools.product((-1, 0, 1), repeat=3):
                status = np.asarray(status)
                fixed, free = np.flatnonzero(status), np.flatnonzero(status == 0)
                x = np.zeros(3)
                x[status == -1] = lo[status == -1]; x[status == 1] = hi[status == 1]
                if len(free):
                    x[free] = np.linalg.solve(h[np.ix_(free, free)], -g[free]-h[np.ix_(free, fixed)] @ x[fixed])
                if np.all(x >= lo-1e-10) and np.all(x <= hi+1e-10):
                    best = min(best, .5*x @ h @ x+g @ x)
            self.assertAlmostEqual(float(.5*actual @ h @ actual+g @ actual), best, places=8)
            self.assertTrue(np.all(actual >= lo) and np.all(actual <= hi))

    def test_equality_bounds_and_kkt(self):
        rng = np.random.default_rng(20261027)
        for _ in range(100):
            a = rng.normal(size=(7, 7)); h = a.T @ a+.2*np.eye(7)
            g = rng.normal(size=7)*3
            lo, hi = np.full(7, -.3), np.full(7, .4)
            lo[2] = hi[2] = .15
            x = solve_box_qp(h, g, lo, hi); gradient = h @ x+g
            self.assertEqual(x[2], .15)
            for i in (0, 1, 3, 4, 5, 6):
                if x[i] <= lo[i]+1e-10:
                    self.assertGreaterEqual(gradient[i], -1e-7)
                elif x[i] >= hi[i]-1e-10:
                    self.assertLessEqual(gradient[i], 1e-7)
                else:
                    self.assertAlmostEqual(gradient[i], 0., places=7)

    def test_invalid_qps_rejected(self):
        for h, g, lo, hi in [(-np.eye(2), np.zeros(2), -np.ones(2), np.ones(2)),
                             (np.eye(2), np.zeros(2), np.ones(2), -np.ones(2)),
                             (np.eye(2), np.array([np.nan, 0.]), -np.ones(2), np.ones(2))]:
            with self.assertRaises(ValueError):
                solve_box_qp(h, g, lo, hi)


class ModelMathTests(unittest.TestCase):
    def test_fk_and_analytic_jacobian_match_independent_models(self):
        rng = np.random.default_rng(20261028)
        for mode, side in itertools.product(('flange', 'taccap'), ('left', 'right')):
            tools = tool_config(mode); solver = Art7ConstrainedIK(tools=tools, side=side)
            for _ in range(12):
                q = np.radians([rng.uniform(lo+1, hi-1) for lo, hi in BOUNDS_DEG])
                actual = solver.forward(q)
                expected = multiply(tcp_forward_matrix(q.tolist()), for_arm(tools, side))
                mm, deg = pose_error(actual, expected)
                self.assertLess(mm, 1e-9); self.assertLess(deg, 1e-5)
                jacobian = solver.jacobian(q)
                for i in range(7):
                    delta = np.zeros(7); delta[i] = 1e-6
                    plus, minus = solver.forward(q+delta), solver.forward(q-delta)
                    position = (plus[:3, 3]-minus[:3, 3])/2e-6
                    angular = rotation_log(plus[:3, :3] @ minus[:3, :3].T)/2e-6
                    np.testing.assert_allclose(jacobian[:3, i], position, atol=3e-7, rtol=1e-7)
                    np.testing.assert_allclose(jacobian[3:, i], angular, atol=1e-8, rtol=1e-7)

    def test_rotation_log_at_zero_and_pi(self):
        np.testing.assert_array_equal(rotation_log(np.eye(3)), np.zeros(3))
        for axis in (np.array([1., 0., 0.]), np.array([0., 1., 0.]),
                     np.array([0., 0., 1.]), np.array([1., 2., -3.])/math.sqrt(14)):
            r = 2*np.outer(axis, axis)-np.eye(3)
            log = rotation_log(r)
            self.assertAlmostEqual(float(np.linalg.norm(log)), math.pi, places=10)
            unit = log/math.pi
            np.testing.assert_allclose(2*np.outer(unit, unit)-np.eye(3), r, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
