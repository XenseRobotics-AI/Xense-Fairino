"""Predictive wrist and joint stopping bounds for ART7 constrained IK.

No guard is disabled or widened. The ordinary mode keeps a complete command
and measured path inside one of the original static wrist rectangles, while
reserving room for BOTH existing and next-command speed. The speed-history
bound assumes a monotone/no-overshoot joint response (e.g. a positive first-
order velocity filter) and a known maximum feedback age. These assumptions
are explicit; this is not a hardware-certified stopping-distance model.
Every joint also receives a directional command-velocity cap that reserves
space for the next command and subsequent bounded braking before its ordinary
joint limit. This command-space invariant is separate from the wrist's
feedback/tracking assumptions and does not certify physical motor braking.
"""
from collections import deque
import inspect
import math
import time

import numpy as np

try:
    from .art7_qp import Art7ConstrainedIK, vector, transform
except ImportError:
    from art7_qp import Art7ConstrainedIK, vector, transform
from wrist_guard import BOUNDARY, MARGIN_DEG, PREDICTION_SECONDS
from tcp_core import check_arm_limits


class PredictiveWristIK:
    """Backend-compatible wrapper; all joint quantities are radians.

    ``feedback_age`` bounds the total transport + feedback sampling age.
    For the speed witness, actual velocity is assumed to remain a convex
    combination of its last measured value and the command velocities sent
    since that sample. Position has the analogous convex-hull assumption.
    A controller with overshoot needs a separately established tracking bound.
    ``braking_acceleration`` is a modeling assumption, distinct from the upper
    command acceleration limit. None uses that command limit for the experiment.
    """
    def __init__(self, solver, feedback_age=.04, braking_acceleration=None):
        if not math.isfinite(feedback_age) or feedback_age < 0:
            raise ValueError('feedback_age must be finite and nonnegative')
        self.solver = solver
        self.feedback_age = float(feedback_age)
        self.braking_acceleration = braking_acceleration
        self._caps = np.radians(np.asarray(BOUNDARY, dtype=float)-MARGIN_DEG)
        self._caps.setflags(write=False)
        parameters = inspect.signature(solver.step).parameters
        self._native_bounds = 'period_lower' in parameters
        if not self._native_bounds and not isinstance(solver, Art7ConstrainedIK):
            raise ValueError('backend must support absolute period_lower/period_upper bounds')
        self.reset()

    @property
    def tools(self):
        return self.solver.tools

    @property
    def side(self):
        return self.solver.side

    @property
    def lower(self):
        return np.asarray(self.solver.lower, dtype=float)

    @property
    def upper(self):
        return np.asarray(self.solver.upper, dtype=float)

    def reset(self):
        self._clock = 0.
        self._history = deque()
        self._selected = None
        self._anchor = None
        self._pending = None
        # Keep tokens unique across reset so an old acceptance acknowledgement
        # cannot accidentally commit a different proposal in a new session.
        self._proposal_id = getattr(self, '_proposal_id', 0)

    @staticmethod
    def _same(a, b):
        return bool(np.allclose(a, b, atol=1e-11, rtol=0.))

    def commit(self, result=None):
        """Acknowledge the latest candidate AFTER receiver acceptance.

        Passing its result validates the proposal token. Explicit commit is
        preferred, especially for a stationary command: identical reference
        and velocity cannot distinguish a new hold frame from a retry.
        """
        if self._pending is None:
            raise ValueError('no pending IK proposal to commit')
        pending = self._pending
        if result is not None:
            if (not isinstance(result, dict) or result.get('proposal_id') != pending['id']
                    or not result.get('feasible') or result.get('target') is None
                    or not self._same(vector(result['target'], 'target'), pending['target'])):
                raise ValueError('commit requires the latest IK proposal')
        self._clock += pending['dt']
        self._history.append((self._clock, pending['target'].copy(), pending['velocity'].copy()))
        self._anchor = (pending['target'].copy(), pending['velocity'].copy())
        self._selected = pending['selected']
        oldest = self._clock-self.feedback_age-pending['dt']
        while self._history and self._history[0][0] < oldest:
            self._history.popleft()
        self._pending = None

    def _confirm_reference(self, q, previous):
        if self._anchor is None:
            self._anchor = (q.copy(), previous.copy())
            return
        at_anchor = self._same(q, self._anchor[0]) and self._same(previous, self._anchor[1])
        if self._pending is not None:
            at_proposal = (self._same(q, self._pending['target'])
                           and self._same(previous, self._pending['velocity']))
            if at_proposal and not at_anchor:
                self.commit()
                return
            if at_anchor:
                # Re-evaluation or a rejected candidate. Neither sent history
                # nor elapsed control time changes from a speculative solve.
                self._pending = None
                return
        elif at_anchor:
            return
        raise ValueError('reference/velocity does not acknowledge the last command; reset is required')

    def forward(self, q):
        return self.solver.forward(q)

    def jacobian(self, q):
        return self.solver.jacobian(q)

    def solve(self, *args, **kwargs):
        return self.solver.solve(*args, **kwargs)

    @staticmethod
    def _limits(value, name):
        result = np.broadcast_to(np.asarray(value, dtype=float), (7,)).copy()
        if not np.isfinite(result).all() or np.any(result <= 0):
            raise ValueError(name+' must be positive and finite')
        return result

    def _joint_limit_speed_caps(self, q, dt, vmax, braking):
        """Positive magnitudes toward lower/upper limits for the NEXT command.

        dt*w + w²/(2*a) <= remaining distance reserves the next command's
        displacement plus a continuous upper bound on its later discrete
        braking tail. Zero gap permits zero outward speed and free inward
        motion. Actual command deceleration is at least the assumed braking
        value because that value is capped by the command acceleration limit.
        """
        negative_gap = np.maximum(0., q-self.lower)
        positive_gap = np.maximum(0., self.upper-q)
        negative = 2.*negative_gap/(dt+np.sqrt(dt*dt+2.*negative_gap/braking))
        positive = 2.*positive_gap/(dt+np.sqrt(dt*dt+2.*positive_gap/braking))
        return np.minimum(vmax, negative), np.minimum(vmax, positive)

    def _bounds(self, q, measured, velocity, previous, dt, vmax, amax):
        base_lower = np.maximum.reduce((self.lower, q-vmax*dt,
                                       q+(previous-amax*dt)*dt))
        base_upper = np.minimum.reduce((self.upper, q+vmax*dt,
                                       q+(previous+amax*dt)*dt))
        if np.any(base_lower > base_upper):
            return None, 'empty_period_bounds'
        braking_all = amax if self.braking_acceleration is None else np.minimum(
            amax, self._limits(self.braking_acceleration, 'braking_acceleration'))
        negative_cap, positive_cap = self._joint_limit_speed_caps(q, dt, vmax, braking_all)
        base_lower = np.maximum(base_lower, q-negative_cap*dt)
        base_upper = np.minimum(base_upper, q+positive_cap*dt)
        if np.any(base_lower > base_upper):
            return None, 'no_braking_feasible_joint_interval'
        # Retain every command potentially newer than the delayed sample.
        oldest = self._clock-self.feedback_age
        positions = [np.abs(q[-2:]), np.abs(measured[-2:])]
        speeds = [np.abs(velocity[-2:]), np.abs(previous[-2:])]
        for stamp, historical_q, historical_v in self._history:
            if stamp < oldest:
                continue
            positions.append(np.abs(historical_q[-2:]))
            speeds.append(np.abs(historical_v[-2:]))
        position_bound = np.max(positions, axis=0)
        existing_speed_bound = np.max(speeds, axis=0)
        braking = braking_all[-2:]
        horizon = PREDICTION_SECONDS+self.feedback_age+dt
        response_horizon = self.feedback_age+dt
        # Evaluate the same independent rectangles in one NumPy batch. Invalid
        # rows are masked, never repaired. argmax preserves first-row tie order.
        gap = self._caps-position_bound
        valid = np.all(self._caps >= 0., axis=1) & np.all(gap >= -1e-12, axis=1)
        gap = np.maximum(0., gap)
        valid &= np.all(PREDICTION_SECONDS*existing_speed_bound <= gap+1e-12, axis=1)
        # Both branches of T*max(existing_speed,w) + response_horizon*w
        # + w^2/(2*a_brake) <= gap must hold.
        residual_gap = np.maximum(0., gap-PREDICTION_SECONDS*existing_speed_bound)
        fresh_cap = 2.*gap/(horizon+np.sqrt(horizon*horizon+2.*gap/braking))
        existing_cap = 2.*residual_gap/(response_horizon+np.sqrt(
            response_horizon*response_horizon+2.*residual_gap/braking))
        cap_speed = np.minimum(vmax[-2:], np.minimum(fresh_cap, existing_cap))
        lo = np.broadcast_to(base_lower, (len(self._caps), 7)).copy()
        hi = np.broadcast_to(base_upper, (len(self._caps), 7)).copy()
        lo[:, -2:] = np.maximum(lo[:, -2:], q[-2:]-cap_speed*dt)
        hi[:, -2:] = np.minimum(hi[:, -2:], q[-2:]+cap_speed*dt)
        future_position_caps = self._caps-PREDICTION_SECONDS*np.maximum(existing_speed_bound, cap_speed)
        lo[:, -2:] = np.maximum(lo[:, -2:], -future_position_caps)
        hi[:, -2:] = np.minimum(hi[:, -2:], future_position_caps)
        valid &= np.all(lo <= hi, axis=1)
        if not np.any(valid):
            return None, 'no_predictively_safe_wrist_rectangle'
        relative_speed = cap_speed/vmax[-2:]
        score = relative_speed.sum(axis=1)+2.*relative_speed.min(axis=1)
        if self._selected is not None:
            score[self._selected] += .02
        score[~valid] = -np.inf
        index = int(np.argmax(score))
        return (float(score[index]), index, lo[index].copy(), hi[index].copy(),
                self._caps[index].copy(), cap_speed[index].copy(),
                existing_speed_bound, position_bound), 'predictive_rectangle'

    def step(self, goal, reference, dt, previous_velocity, measured=None,
             measured_velocity=None, *, max_iterations=8,
             joint_velocity_limit=math.radians(30.),
             joint_acceleration_limit=math.radians(120.)):
        started = time.perf_counter()
        goal = transform(goal)
        q = vector(reference, 'reference')
        measured = q.copy() if measured is None else vector(measured, 'measured')
        check_arm_limits(measured.tolist())
        velocity = np.zeros(7) if measured_velocity is None else vector(measured_velocity, 'measured_velocity')
        previous = vector(previous_velocity, 'previous_velocity')
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        if type(max_iterations) is not int or max_iterations < 1:
            raise ValueError('max_iterations must be a positive integer')
        if np.any(q < self.lower-1e-12) or np.any(q > self.upper+1e-12):
            raise ValueError('reference is outside configured joint limits')
        vmax = self._limits(joint_velocity_limit, 'joint_velocity_limit')
        amax = self._limits(joint_acceleration_limit, 'joint_acceleration_limit')
        self._confirm_reference(q, previous)
        plan, status = self._bounds(q, measured, velocity, previous, dt, vmax, amax)
        if plan is None:
            return dict(success=False, feasible=False, target=None, q=None, tcp=None,
                status=status, reason=status, policy_status=status, position_error_mm=None,
                orientation_error_deg=None, iterations=0, qp_solves=0, qp_calls=0,
                elapsed_ms=1000*(time.perf_counter()-started), velocity=None, acceleration=None,
                selected_rectangle_deg=None, wrist_speed_cap_deg_s=None)
        _, selected, lower, upper, caps, cap_speed, existing_speed, position_bound = plan
        if self._native_bounds:
            result = self.solver.step(goal, q, dt, previous, measured=measured,
                measured_velocity=velocity, max_iterations=max_iterations,
                joint_velocity_limit=vmax, joint_acceleration_limit=amax,
                period_lower=lower, period_upper=upper)
        else:
            # Compatibility for the immutable first NumPy experiment. Only its
            # existing numerical core is invoked; no production code is altered.
            validated_goal, validated_q, validated_measured, validated_velocity = self.solver._inputs(
                goal, q, measured, velocity)
            result = self.solver._run(validated_goal, validated_q, validated_measured,
                validated_velocity, lower, upper, max_iterations=max_iterations,
                dt=dt, previous_velocity=previous)
        result = dict(result)
        result.update(policy_status=status, selected_rectangle_deg=np.degrees(caps),
            wrist_speed_cap_deg_s=np.degrees(cap_speed),
            current_speed_bound_deg_s=np.degrees(existing_speed),
            historical_position_bound_deg=np.degrees(position_bound),
            feedback_age_bound=self.feedback_age,
            elapsed_ms=1000*(time.perf_counter()-started))
        if result['feasible']:
            target = vector(result['target'], 'target')
            # Independently check the fixed absolute interval supplied to QP.
            if np.any(target < lower-1e-12) or np.any(target > upper+1e-12):
                raise AssertionError('backend violated predictive period bounds')
            self._proposal_id += 1
            self._pending = dict(id=self._proposal_id, target=target.copy(),
                velocity=(target-q)/dt, dt=dt, selected=selected)
            result['proposal_id'] = self._proposal_id
        return result

    def brake(self, reference, dt, previous_velocity, measured=None,
              measured_velocity=None, *, joint_velocity_limit=math.radians(30.),
              joint_acceleration_limit=math.radians(120.)):
        """Generate one bounded stopping step; never chase a former TCP target.

        Every commanded velocity component decreases toward zero without a
        sign reversal. ``stopped`` refers to COMMAND velocity; measured motor
        velocity can still be settling. Once stopped, the endpoint is held.
        The proposal is accepted only by ``commit`` or reference acknowledgement.
        """
        started = time.perf_counter()
        q = vector(reference, 'reference')
        measured = q.copy() if measured is None else vector(measured, 'measured')
        check_arm_limits(measured.tolist())
        velocity = np.zeros(7) if measured_velocity is None else vector(measured_velocity, 'measured_velocity')
        previous = vector(previous_velocity, 'previous_velocity')
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        if np.any(q < self.lower-1e-12) or np.any(q > self.upper+1e-12):
            raise ValueError('reference is outside configured joint limits')
        vmax = self._limits(joint_velocity_limit, 'joint_velocity_limit')
        amax = self._limits(joint_acceleration_limit, 'joint_acceleration_limit')
        self._confirm_reference(q, previous)
        plan, policy_status = self._bounds(q, measured, velocity, previous, dt, vmax, amax)
        next_velocity = np.sign(previous)*np.maximum(np.abs(previous)-amax*dt, 0.)
        desired_q = q+dt*next_velocity

        def failure(reason):
            return dict(success=False, feasible=False, target=None, q=None, tcp=None,
                status=reason, reason=reason, policy_status=policy_status, stopped=False,
                command_stopped=False, position_error_mm=None, orientation_error_deg=None,
                iterations=0, qp_solves=0, qp_calls=0, velocity=None, acceleration=None,
                selected_rectangle_deg=None, wrist_speed_cap_deg_s=None,
                elapsed_ms=1000*(time.perf_counter()-started))

        if plan is None:
            return failure(policy_status)
        _, selected, lower, upper, caps, cap_speed, existing_speed, position_bound = plan
        if np.any(desired_q < lower-1e-12) or np.any(desired_q > upper+1e-12):
            return failure('braking_target_outside_predictive_bounds')
        # The new goal is the FK of the analytically computed stopping command,
        # not the TCP at grip release. Equality bounds prohibit IK oscillation.
        goal = self.solver.forward(desired_q)
        if self._native_bounds:
            result = self.solver.step(goal, q, dt, previous, measured=measured,
                measured_velocity=velocity, max_iterations=1,
                joint_velocity_limit=vmax, joint_acceleration_limit=amax,
                period_lower=desired_q, period_upper=desired_q)
        else:
            validated_goal, validated_q, validated_measured, validated_velocity = self.solver._inputs(
                goal, q, measured, velocity)
            result = self.solver._run(validated_goal, validated_q, validated_measured,
                validated_velocity, desired_q, desired_q, max_iterations=1,
                dt=dt, previous_velocity=previous)
        result = dict(result)
        result.update(stopped=False, command_stopped=False, policy_status=policy_status,
            selected_rectangle_deg=np.degrees(caps), wrist_speed_cap_deg_s=np.degrees(cap_speed),
            current_speed_bound_deg_s=np.degrees(existing_speed),
            historical_position_bound_deg=np.degrees(position_bound),
            feedback_age_bound=self.feedback_age, goal_tcp=goal,
            desired_braking_velocity=next_velocity,
            elapsed_ms=1000*(time.perf_counter()-started))
        if result['feasible']:
            target = vector(result['target'], 'target')
            if (np.any(target < lower-1e-12) or np.any(target > upper+1e-12)
                    or not np.allclose(target, desired_q, atol=1e-12, rtol=0.)):
                raise AssertionError('backend violated the exact braking target')
            commanded_velocity = (target-q)/dt
            stopped = bool(np.max(np.abs(commanded_velocity)) <= 1e-10)
            result['solver_status'] = result['status']
            result.update(status='stopped' if stopped else 'stopping',
                          reason='command_velocity_zero' if stopped else 'bounded_braking',
                          stopped=stopped, command_stopped=stopped)
            self._proposal_id += 1
            self._pending = dict(id=self._proposal_id, target=target.copy(),
                velocity=commanded_velocity, dt=dt, selected=selected)
            result['proposal_id'] = self._proposal_id
        return result
