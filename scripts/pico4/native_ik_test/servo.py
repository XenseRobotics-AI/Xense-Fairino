"""Archived bounded-rate experiment; not used by the current PICO entry.

Static IK remains the native nonlinear solver. Online tracking requests a
Cartesian velocity proportional to the remaining full pose error, instead of
trying to remove that entire error in a single acceleration-bounded tick.
The C++ box QP selects a joint increment inside the predictive policy's bounds.
This is a command-space controller, not a certified motor dynamics model.
"""
import math
import time

import numpy as np

from tools.ik_experiments.art7_native import Art7NativeIK, solve_box_qp_native
from tools.ik_experiments.art7_qp import vector
from wrist_guard import PREDICTION_SECONDS


class Art7ServoIK(Art7NativeIK):
    def step(self, goal, reference, dt, previous_velocity, measured=None,
             measured_velocity=None, *, max_iterations=8,
             joint_velocity_limit=math.radians(30.),
             joint_acceleration_limit=math.radians(120.), period_lower=None,
             period_upper=None, allow_guard_retreat=False, reserve_velocity=None):
        started = time.perf_counter()
        goal, reference, measured, velocity = self._inputs(
            goal, reference, measured, measured_velocity)
        previous = vector(previous_velocity, 'previous_velocity')
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        if type(max_iterations) is not int or max_iterations < 1:
            raise ValueError('max_iterations must be a positive integer')
        if type(allow_guard_retreat) is not bool:
            raise ValueError('allow_guard_retreat must be boolean')
        vmax = np.broadcast_to(np.asarray(joint_velocity_limit, dtype=float), (7,)).copy()
        amax = np.broadcast_to(np.asarray(joint_acceleration_limit, dtype=float), (7,)).copy()
        if (not np.isfinite(vmax).all() or not np.isfinite(amax).all()
                or np.any(vmax <= 0) or np.any(amax <= 0)):
            raise ValueError('velocity and acceleration limits must be positive and finite')
        lower = np.maximum.reduce([self.lower, reference-vmax*dt,
                                   reference+(previous-amax*dt)*dt])
        upper = np.minimum.reduce([self.upper, reference+vmax*dt,
                                   reference+(previous+amax*dt)*dt])
        lower, upper = self._joint_bounds(lower, upper, period_lower, period_upper)

        def result(q, status, calls=0):
            value = self._result(started, goal, q, calls, calls, status, reference, dt, previous)
            value['backend'] = 'native_cpp17_resolved_rate'
            return value

        if np.any(lower > upper):
            return result(None, 'empty_period_bounds')
        tcp, jacobian = self._native_fk(reference, True)
        residual, mm, deg = self._error(goal, tcp)
        # Start slowing before the maximum joint speed needs its full braking
        # time. This preserves the complete external goal; only the desired
        # rate changes, and convergence is assessed against that full goal.
        response_time = max(.4, 1.5*float(np.max(vmax/amax)))
        fraction = -math.expm1(-dt/response_time)
        if mm <= .02 and deg <= .01:
            residual[:] = 0.
        jacobian[3:] *= self.rotation_scale
        hessian = jacobian.T @ jacobian+np.eye(7)
        gradient = -jacobian.T @ (fraction*residual)
        increment = solve_box_qp_native(hessian, gradient, lower-reference, upper-reference)
        q = reference+increment
        if not self._safe(q, reference, measured, velocity, lower, upper):
            return result(None, 'no_feasible_candidate', 1)
        extra = velocity if reserve_velocity is None else vector(reserve_velocity, 'reserve_velocity')
        reserve = np.maximum(np.abs(velocity[-2:]), np.abs(extra[-2:]))
        for start in (reference, measured):
            inward = bool(np.all(start[-2:]*velocity[-2:] <= 1e-10))
            if not self.guard._pair_motion(start[-2:].tolist(), q[-2:].tolist(),
                    tuple(np.degrees(reserve)*PREDICTION_SECONDS),
                    allow_retreat=allow_guard_retreat and inward):
                return result(None, 'no_feasible_candidate', 1)
        value = result(q, 'tracking_progress', 1)
        value['response_time_s'] = response_time
        return value
