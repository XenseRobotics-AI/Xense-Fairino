"""Offline regressions for all-seven-axis command stopping viability."""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'apps/fairino_tcp_probe'),
               str(ROOT/'apps/fairino_upper_probe')]
import numpy as np

from tools.ik_experiments.art7_native import Art7NativeIK
from tools.ik_experiments.dynamic_wrist_policy import PredictiveWristIK
from tcp_core import BOUNDS_DEG
from tool_frames import tool_config

HOME = np.radians([45., 75., -100., 85., 85., 0., 0.])
VMAX = np.full(7, math.radians(10.))
AMAX = np.full(7, math.radians(30.))


class JointBrakingTests(unittest.TestCase):
    def setUp(self):
        # Tight independent wrist bounds keep these fixtures far from the
        # coupled wrist envelope, isolating the ordinary joint-limit invariant.
        bounds = np.asarray(BOUNDS_DEG, dtype=float).copy()
        bounds[5:] = [-10., 10.]
        self.backend = Art7NativeIK(tools=tool_config('flange'), bounds_deg=bounds)
        self.policy = PredictiveWristIK(self.backend, feedback_age=.06,
                                       braking_acceleration=math.radians(30.))

    def test_directional_caps_at_every_limit_allow_inward_motion(self):
        for joint in range(7):
            for sign in (-1., 1.):
                q = HOME.copy()
                q[joint] = self.backend.upper[joint] if sign > 0 else self.backend.lower[joint]
                negative, positive = self.policy._joint_limit_speed_caps(q, .01, VMAX, AMAX)
                outward, inward = (positive, negative) if sign > 0 else (negative, positive)
                self.assertEqual(outward[joint], 0.)
                self.assertEqual(inward[joint], VMAX[joint])
                plan, status = self.policy._bounds(q, q, np.zeros(7), np.zeros(7), .01, VMAX, AMAX)
                self.assertIsNotNone(plan, (joint, sign, status))
                lower, upper = plan[2:4]
                self.assertAlmostEqual((upper if sign > 0 else lower)[joint], q[joint], places=14)
                self.assertGreater(abs((lower if sign > 0 else upper)[joint]-q[joint]), 0.)

    def test_cap_formula_and_variable_dt_braking_for_all_axes_and_directions(self):
        rng = np.random.default_rng(20261026)
        checked = 0
        for joint in range(7):
            for sign in (-1., 1.):
                for _ in range(12):
                    dt = float(rng.uniform(.001, .03))
                    braking = AMAX*float(rng.uniform(.25, 1.))
                    q = HOME.copy()
                    distance = math.radians(float(rng.uniform(.00001, 4.)))
                    limit = self.backend.upper[joint] if sign > 0 else self.backend.lower[joint]
                    q[joint] = limit-sign*distance
                    negative, positive = self.policy._joint_limit_speed_caps(q, dt, VMAX, braking)
                    speed = (positive if sign > 0 else negative)[joint]
                    self.assertLessEqual(dt*speed+speed**2/(2*braking[joint]), distance+1e-14)
                    q[joint] += sign*dt*speed
                    # Start at an accepted cap endpoint, then vary the next dt
                    # and use a command deceleration >= the assumed braking.
                    while speed > 0.:
                        next_dt = float(rng.uniform(.001, .03))
                        next_speed = max(0., speed-AMAX[joint]*next_dt)
                        q[joint] += sign*next_dt*next_speed
                        remaining = sign*(limit-q[joint])
                        self.assertGreaterEqual(remaining+1e-13, next_speed**2/(2*braking[joint]))
                        self.assertGreaterEqual(q[joint], self.backend.lower[joint]-1e-13)
                        self.assertLessEqual(q[joint], self.backend.upper[joint]+1e-13)
                        self.assertLessEqual(abs(next_speed-speed)/next_dt, AMAX[joint]+1e-12)
                        speed = next_speed
                    checked += 1
        self.assertEqual(checked, 168)

    def test_historical_j11_state_without_stopping_margin_is_explicitly_rejected(self):
        q = np.radians([-36.8542318914864, 91.50609831668407, 88.12792828260841,
                        109.9046959032687, 5.0429828314699465, -30.842119650947243,
                        -15.903718118061681])
        backend = Art7NativeIK(tools=tool_config('flange'), side='right')
        policy = PredictiveWristIK(backend, feedback_age=.06,
                                  braking_acceleration=math.radians(30.))
        previous = np.zeros(7); previous[3] = math.radians(10.)
        goal = backend.forward(q)
        for mode in ('step', 'brake'):
            policy.reset()
            args = (goal, q, .01, previous) if mode == 'step' else (q, .01, previous)
            result = getattr(policy, mode)(*args, measured=q, measured_velocity=previous,
                joint_velocity_limit=VMAX, joint_acceleration_limit=AMAX)
            self.assertFalse(result['feasible'], result)
            self.assertIsNone(result['target'])
            self.assertIn(result['status'], ('empty_period_bounds', 'no_braking_feasible_joint_interval'))

    def test_continuous_j4_outward_trajectory_prebrakes_and_release_can_always_stop(self):
        q = HOME.copy(); q[3] = math.radians(107.5)
        previous = np.zeros(7); capped = False
        for tick in range(500):
            plan, status = self.policy._bounds(q, q, previous, previous, .01, VMAX, AMAX)
            self.assertIsNotNone(plan, (tick, status))
            next_q = q.copy(); next_q[3] = plan[3][3]
            next_v = (next_q-q)/.01
            self.assertLessEqual(abs(next_v[3]-previous[3]), AMAX[3]*.01+1e-12)
            self.assertLessEqual(next_q[3], self.backend.upper[3]+1e-13)
            self.assertGreaterEqual(self.backend.upper[3]-next_q[3]+1e-13,
                                    next_v[3]**2/(2*AMAX[3]))
            capped |= previous[3] > next_v[3]+1e-9
            if tick % 11 == 0:
                brake_policy = PredictiveWristIK(self.backend, feedback_age=.06,
                    braking_acceleration=math.radians(30.))
                stop_q = next_q.copy(); stop_v = next_v.copy()
                for _ in range(100):
                    result = brake_policy.brake(stop_q, .01, stop_v, measured=stop_q,
                        measured_velocity=stop_v, joint_velocity_limit=VMAX,
                        joint_acceleration_limit=AMAX)
                    self.assertTrue(result['feasible'], (tick, result))
                    brake_policy.commit(result)
                    following = np.asarray(result['target'])
                    new_v = (following-stop_q)/.01
                    self.assertTrue(np.all(np.abs(new_v) <= np.abs(stop_v)+1e-11))
                    stop_q, stop_v = following, new_v
                    if result['command_stopped']: break
                else: self.fail('braking did not stop in 100 periods')
            q, previous = next_q, next_v
        self.assertTrue(capped)
        self.assertLess(self.backend.upper[3]-q[3], 1e-9)
        self.assertLess(abs(previous[3]), 1e-9)

    def test_configured_weaker_braking_is_conservative_and_stronger_is_clamped(self):
        q = HOME.copy(); q[3] = self.backend.upper[3]-math.radians(.001)
        args = (q, q, np.zeros(7), np.zeros(7), .01, VMAX, AMAX)
        intervals = []
        for braking in (math.radians(5.), math.radians(30.), math.radians(300.)):
            policy = PredictiveWristIK(self.backend, feedback_age=.06,
                                      braking_acceleration=braking)
            intervals.append(policy._bounds(*args)[0])
        self.assertLess(intervals[0][3][3], intervals[1][3][3])
        np.testing.assert_allclose(intervals[1][2], intervals[2][2], atol=0., rtol=0.)
        np.testing.assert_allclose(intervals[1][3], intervals[2][3], atol=0., rtol=0.)


if __name__ == '__main__':
    unittest.main()
