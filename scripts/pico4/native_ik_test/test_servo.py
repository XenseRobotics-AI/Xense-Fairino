"""Offline fixed-goal regression from the 2026-10-26 oscillation diagnosis."""
import math
import unittest

import numpy as np

from scripts.pico4.native_ik_test.servo import Art7ServoIK
from tools.ik_experiments.dynamic_wrist_policy import PredictiveWristIK


class ServoTests(unittest.TestCase):
    def test_moving_peak_state_settles_without_repeated_late_overshoot(self):
        # Captured command position and measured velocity near intent 6796.
        # Treat the latter as initial command velocity for this deterministic
        # regression; it is not a claim to reconstruct the motor/controller.
        q = np.radians([69.3745778611, 60.2211482866, -65.5790660071,
                        104.4446288991, 93.7570244715, -13.6085905322, -12.4185811945])
        previous = np.radians([-6.5940594059, -8.9108910891, -6.5950413223,
                               -9.4214876033, 4.5742574257, -3.9207920792, -3.7425742574])
        goal = np.array([
            [-.9631504454107069, .2584600218647838, .07442873506121328, 481.5033016430969],
            [.26631005864992086, .8776361658743279, .39853947484695273, 102.86258494020602],
            [.03768517171292348, .40367459351198287, -.9141261676518744, 238.26784365660188],
            [0., 0., 0., 1.],
        ])
        solver = Art7ServoIK()
        policy = PredictiveWristIK(solver, feedback_age=.06,
                                   braking_acceleration=math.radians(30.))
        for index in range(650):
            result = policy.step(goal, q, .01, previous, measured=q, measured_velocity=previous,
                joint_velocity_limit=math.radians(10.), joint_acceleration_limit=math.radians(30.))
            self.assertTrue(result['feasible'], result['status'])
            self.assertLessEqual(float(np.max(np.abs(result['velocity']))), math.radians(10.)+1e-9)
            self.assertLessEqual(float(np.max(np.abs(result['acceleration']))), math.radians(30.)+1e-7)
            # The old greedy endpoint solver repeatedly left this band for
            # several seconds despite receiving exactly the same TCP goal.
            if index >= 250:
                self.assertLess(result['position_error_mm'], 1.)
                self.assertLess(result['orientation_error_deg'], 1.)
            if index >= 500:
                self.assertTrue(result['success'])
                self.assertLess(float(np.max(np.abs(result['velocity']))), 1e-10)
            policy.commit(result)
            q, previous = result['target'], result['velocity']

    def test_goal_error_is_reported_against_full_target(self):
        solver = Art7ServoIK()
        q = np.radians([45., 75., -100., 85., 85., 0., 0.])
        goal = solver.forward(q)
        goal[0, 3] += 30.
        policy = PredictiveWristIK(solver, feedback_age=.06,
                                   braking_acceleration=math.radians(30.))
        result = policy.step(goal, q, .01, np.zeros(7), measured=q,
            joint_velocity_limit=math.radians(10.), joint_acceleration_limit=math.radians(30.))
        self.assertTrue(result['feasible'])
        self.assertFalse(result['success'])
        _, mm, deg = solver._error(goal, solver.forward(result['target']))
        self.assertAlmostEqual(result['position_error_mm'], mm)
        self.assertAlmostEqual(result['orientation_error_deg'], deg)
        self.assertGreater(mm, 25.)


if __name__ == '__main__':
    unittest.main()
