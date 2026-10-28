"""Archived bounded-step planner for historical offline comparisons only.

The current standalone robot entry uses direct constrained IK instead.

No ROS, sockets, subprocess transport, or robot actuation lives here. A caller
must validate and publish a complete proposal before calling commit(). The
reference is the last committed command, never a claim of measured arrival.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.ik_experiments.art7_native import Art7NativeIK
from tools.ik_experiments.art7_qp import vector, transform
from tools.ik_experiments.dynamic_wrist_policy import PredictiveWristIK
from tcp_core import (ARMS, MODEL_ID, check_arm_limits, finite_vector,
                      multiply, pose_error, tcp_forward_matrix)
from tool_frames import tool_config, validate_runtime, for_arm
from wrist_guard import WristGuard, PREDICTION_SECONDS

STATIONARY_SPEED_RAD_S = .005


def _plain(value):
    """Public proposals are JSON-compatible and detached from private state."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(item) for item in value]
    return value


def _clone_policy(policy):
    # Native solver/CDLL is immutable and shared. Only the small proposal and
    # accepted-command history are cloned; failed speculation cannot age them.
    result = copy.copy(policy)
    for name in ('_clock', '_history', '_selected', '_anchor', '_pending', '_proposal_id'):
        setattr(result, name, copy.deepcopy(getattr(policy, name)))
    return result


class BimanualNativePlanner:
    def __init__(self, tools=None, speed_deg_s=10., acceleration_deg_s2=30.,
                 feedback_age=.06, braking_deg_s2=30.):
        values = (speed_deg_s, acceleration_deg_s2, braking_deg_s2)
        if any(type(value) not in (int, float) or not math.isfinite(value) or value <= 0 for value in values):
            raise ValueError('speed, acceleration and braking limits must be positive and finite')
        if type(feedback_age) not in (int, float) or not math.isfinite(feedback_age) or feedback_age < 0:
            raise ValueError('feedback_age must be finite and nonnegative')
        if braking_deg_s2 > acceleration_deg_s2:
            raise ValueError('assumed braking acceleration cannot exceed the command acceleration limit')
        self.tools = copy.deepcopy(validate_runtime(tool_config() if tools is None else tools))
        self.speed_rad_s = math.radians(speed_deg_s)
        self.acceleration_rad_s2 = math.radians(acceleration_deg_s2)
        self.braking_rad_s2 = math.radians(braking_deg_s2)
        self.feedback_age = float(feedback_age)
        self.solvers = {side: Art7NativeIK(tools=self.tools, side=side) for side in ARMS}
        self.guard = WristGuard()
        self._policies = {}
        self._references = {}
        self._previous_velocities = {}
        self._pending = None
        self._proposal_id = 0
        self._initialized = False

    @property
    def initialized(self):
        return self._initialized

    @property
    def references(self):
        return {side: q.copy() for side, q in self._references.items()}

    @property
    def previous_velocities(self):
        return {side: v.copy() for side, v in self._previous_velocities.items()}

    @staticmethod
    def _hardware(hardware):
        if not isinstance(hardware, dict):
            raise ValueError('hardware must contain the current 20-axis feedback')
        values = {name: np.asarray(finite_vector(hardware.get(name), 20, name), dtype=float)
                  for name in ('position', 'command_position', 'velocity')}
        for offset in (0, 7):
            check_arm_limits(values['position'][offset:offset+7].tolist())
            check_arm_limits(values['command_position'][offset:offset+7].tolist())
        return values

    def reset(self, hardware):
        """Initialize after an independently confirmed settled receiver state.

        This module checks arm velocity and path admissibility. Freshness,
        controller enable/fault state and actual hold acknowledgement remain
        the receiver lifecycle's responsibility.
        """
        hw = self._hardware(hardware)
        if np.max(np.abs(hw['velocity'][:14])) > STATIONARY_SPEED_RAD_S:
            raise ValueError('both arms must be stationary before resetting native IK')
        references = {}; policies = {}
        for side, indices in ARMS.items():
            ids = np.asarray(indices)-1
            q, measured, velocity = (hw[name][ids] for name in ('command_position', 'position', 'velocity'))
            reserve = tuple(np.abs(np.degrees(velocity[-2:]))*PREDICTION_SECONDS)
            if (self.guard.clearance(q[-2:].tolist(), reserve=reserve) < -1e-9
                    or self.guard.clearance(measured[-2:].tolist(), reserve=reserve) < -1e-9):
                raise ValueError(f'{side}: initial wrist state is outside the reserved envelope')
            self.guard.arm_segment(q.tolist(), q.tolist(), velocity.tolist())
            self.guard.arm_segment(measured.tolist(), q.tolist(), velocity.tolist())
            expected = multiply(tcp_forward_matrix(q.tolist()), for_arm(self.tools, side))
            mm, deg = pose_error(expected, self.solvers[side].forward(q))
            if mm > .02 or deg > .01:
                raise ValueError(f'{side}: native FK disagrees with independent receiver model')
            references[side] = q.copy()
            policies[side] = PredictiveWristIK(self.solvers[side], feedback_age=self.feedback_age,
                                              braking_acceleration=self.braking_rad_s2)
        # Replace everything only after BOTH arms have passed initialization.
        self._references = references
        self._previous_velocities = {side: np.zeros(7) for side in ARMS}
        self._policies = policies
        self._pending = None
        self._initialized = True

    def propose(self, goals, hardware, dt):
        """Plan both arms without committing any solver or command history.

        Missing goals explicitly request bounded braking for those arms. Every
        feasible partial IK step reports the actual achieved TCP and keeps its
        separate success flag; it is not labeled a full target solution.
        """
        self._pending = None
        if not self._initialized:
            raise ValueError('reset with settled hardware is required before planning')
        if not isinstance(goals, dict) or not set(goals) <= set(ARMS):
            raise ValueError('goals must map left/right to finite TCP transforms')
        if type(dt) not in (int, float) or not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        validated_goals = {side: transform(goal) for side, goal in goals.items()}
        hw = self._hardware(hardware)
        staged = {side: _clone_policy(policy) for side, policy in self._policies.items()}
        arms = {}; results = {}; all_stopped = True
        for side, indices in ARMS.items():
            ids = np.asarray(indices)-1
            q = self._references[side].copy()
            previous = self._previous_velocities[side].copy()
            measured, velocity = hw['position'][ids], hw['velocity'][ids]
            kwargs = dict(measured=measured, measured_velocity=velocity,
                          joint_velocity_limit=self.speed_rad_s,
                          joint_acceleration_limit=self.acceleration_rad_s2)
            if side in validated_goals:
                result = staged[side].step(validated_goals[side], q, dt, previous, **kwargs)
            else:
                result = staged[side].brake(q, dt, previous, **kwargs)
            if not result.get('feasible') or result.get('target') is None:
                raise ValueError(f'{side}: native IK refused the whole bimanual proposal: '+str(result.get('status')))
            target = vector(result['target'], 'target')
            check_arm_limits(target.tolist())
            actual_velocity = (target-q)/dt
            actual_acceleration = (actual_velocity-previous)/dt
            if (np.max(np.abs(actual_velocity)) > self.speed_rad_s+1e-8
                    or np.max(np.abs(actual_acceleration)) > self.acceleration_rad_s2+1e-6):
                raise ValueError(f'{side}: proposal violated its fixed-period speed/acceleration limits')
            self.guard.arm_segment(q.tolist(), target.tolist(), velocity.tolist())
            self.guard.arm_segment(measured.tolist(), target.tolist(), velocity.tolist())
            independent = multiply(tcp_forward_matrix(target.tolist()), for_arm(self.tools, side))
            reported = transform(result['tcp'])
            mm, deg = pose_error(reported, independent)
            if mm > .02 or deg > .01:
                raise ValueError(f'{side}: proposal failed independent TCP FK validation')
            if side in validated_goals:
                error_mm, error_deg = pose_error(validated_goals[side], independent)
                if result.get('success') and (error_mm > .02 or error_deg > .01):
                    raise ValueError(f'{side}: a partial target was incorrectly reported as solved')
                result = dict(result, position_error_mm=error_mm, orientation_error_deg=error_deg)
            stopped = bool(np.max(np.abs(actual_velocity)) <= 1e-10)
            all_stopped &= stopped
            result = dict(result, requested=side in validated_goals, command_stopped=stopped)
            arms[side] = dict(reference=q.tolist(), target=target.tolist(),
                              tcp=np.asarray(independent).reshape(-1).tolist())
            results[side] = _plain(result)
        self._proposal_id += 1
        proposal = dict(proposal_id=self._proposal_id, model=MODEL_ID, tool_id=self.tools['id'],
                        dt=float(dt), arms=arms, results=results,
                        stopped=all_stopped, command_stopped=all_stopped)
        self._pending = dict(expected=copy.deepcopy(proposal), staged=staged)
        return proposal

    def commit(self, proposal):
        """Call only AFTER the whole bimanual target was validated and published."""
        if self._pending is None or not isinstance(proposal, dict):
            raise ValueError('there is no current bimanual proposal to commit')
        expected = self._pending['expected']
        try:
            matches = all(_plain(proposal.get(key)) == expected[key] for key in expected)
        except (TypeError, ValueError):
            matches = False
        if not matches:
            raise ValueError('commit requires the latest unchanged complete proposal')
        staged = self._pending['staged']
        # Commit on detached states first. Even an unexpected second-arm error
        # cannot partially advance the planner's accepted-command history.
        for side in ARMS:
            staged[side].commit(expected['results'][side])
        references = {side: np.asarray(expected['arms'][side]['target'], dtype=float) for side in ARMS}
        velocities = {side: np.asarray(expected['results'][side]['velocity'], dtype=float) for side in ARMS}
        self._policies = staged
        self._references = references
        self._previous_velocities = velocities
        self._pending = None


# Concise public alias for the standalone receiver.
NativePlanner = BimanualNativePlanner
