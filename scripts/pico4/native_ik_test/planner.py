"""Pure bimanual planning adapter for the standalone native-IK PICO test.

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

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.ik_experiments.art7_native import Art7NativeIK
from tools.ik_experiments.art7_qp import vector, transform
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


class BimanualNativePlanner:
    def __init__(self, tools=None):
        self.tools = copy.deepcopy(validate_runtime(tool_config() if tools is None else tools))
        self.solvers = {side: Art7NativeIK(tools=self.tools, side=side) for side in ARMS}
        self.guard = WristGuard()
        self._references = {}
        # Differences between committed position samples are diagnostics only.
        # They are never integrated, clipped, or used to limit a later solve.
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
        references = {}
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
        # Replace everything only after BOTH arms have passed initialization.
        self._references = references
        self._previous_velocities = {side: np.zeros(7) for side in ARMS}
        self._pending = None
        self._initialized = True

    def propose(self, goals, hardware, dt):
        """Solve full TCP targets, or hold the last published position.

        The original position-sample execution chain supplies the timing; dt
        is used only for reporting command differences. No velocity, acceleration,
        stopping-distance, or response-time box is added to the IK problem.
        Actual measured wrist velocity still supplies the original WristGuard
        prediction reserve for new movement. A missing goal republishes the
        exact accepted position, like the original receiver's stop_stream().
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
        arms = {}; results = {}; all_stopped = True
        for side, indices in ARMS.items():
            ids = np.asarray(indices)-1
            q = self._references[side].copy()
            measured, velocity = hw['position'][ids], hw['velocity'][ids]
            requested = side in validated_goals
            if requested:
                result = self.solvers[side].solve(validated_goals[side], q,
                    measured=measured, measured_velocity=velocity, allow_guard_retreat=True)
                if not result.get('feasible') or result.get('target') is None:
                    raise ValueError(f'{side}: native IK refused the whole bimanual proposal: '
                                     +str(result.get('status')))
                target = vector(result['target'], 'target')
                # This is the same two-path final validator as the original
                # receiver, in addition to the limits inside native IK.
                self.guard.arm_segment(q.tolist(), target.tolist(), velocity.tolist())
                self.guard.arm_segment(measured.tolist(), target.tolist(), velocity.tolist())
            else:
                # Do not turn a released Grip into a new trajectory or let a
                # measured velocity reserve prevent republishing an unchanged
                # accepted target. Hardware stop is confirmed by the receiver.
                target = q.copy()
                if self.guard.clearance(target[-2:].tolist()) < -1e-9:
                    raise ValueError(f'{side}: held command is outside the static wrist envelope')
                self.guard.arm_segment(q.tolist(), target.tolist(), [0.]*7)
                result = dict(success=True, feasible=True, q=target.copy(), target=target.copy(),
                    tcp=self.solvers[side].forward(target), status='holding', reason='input_released_hold',
                    iterations=0, qp_calls=0, qp_solves=0, elapsed_ms=0.,
                    position_error_mm=0., orientation_error_deg=0., backend='native_cpp17')
            check_arm_limits(target.tolist())
            independent = multiply(tcp_forward_matrix(target.tolist()), for_arm(self.tools, side))
            reported = transform(result['tcp'])
            mm, deg = pose_error(reported, independent)
            if mm > .02 or deg > .01:
                raise ValueError(f'{side}: proposal failed independent TCP FK validation')
            if requested:
                error_mm, error_deg = pose_error(validated_goals[side], independent)
                if result.get('success') and (error_mm > .02 or error_deg > .01):
                    raise ValueError(f'{side}: a partial target was incorrectly reported as solved')
                # Native solve chooses a feasible descent from the published
                # seed. Refuse a regressing partial answer rather than sending
                # an unexplained alternate configuration toward a fixed goal.
                origin = self.solvers[side].forward(q)
                old_error = self.solvers[side]._error(validated_goals[side], origin)[0]
                new_error = self.solvers[side]._error(validated_goals[side], np.asarray(independent))[0]
                before, after = float(old_error @ old_error), float(new_error @ new_error)
                if after > before+1e-7*(1.+before):
                    raise ValueError(f'{side}: native IK moved away from the full TCP target')
                result = dict(result, position_error_mm=error_mm, orientation_error_deg=error_deg)
            difference_velocity = (target-q)/dt
            if not np.isfinite(difference_velocity).all():
                raise ValueError('control period is too small for finite command diagnostics')
            stopped = bool(np.max(np.abs(target-q)) <= 1e-12)
            all_stopped &= stopped
            result = dict(result, requested=requested, command_stopped=stopped,
                          velocity=difference_velocity, acceleration=None,
                          wrist_reserve_deg=np.abs(np.degrees(velocity[-2:]))*PREDICTION_SECONDS,
                          solve_mode='full_target' if requested else 'hold_last_command')
            arms[side] = dict(reference=q.tolist(), target=target.tolist(),
                              tcp=np.asarray(independent).reshape(-1).tolist())
            results[side] = _plain(result)
        self._proposal_id += 1
        proposal = dict(proposal_id=self._proposal_id, model=MODEL_ID, tool_id=self.tools['id'],
                        dt=float(dt), arms=arms, results=results,
                        stopped=all_stopped, command_stopped=all_stopped)
        self._pending = dict(expected=copy.deepcopy(proposal))
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
        # Both references change only after publication of the complete
        # unchanged pair. The native static solver itself has no command state.
        references = {side: np.asarray(expected['arms'][side]['target'], dtype=float) for side in ARMS}
        velocities = {side: np.asarray(expected['results'][side]['velocity'], dtype=float) for side in ARMS}
        self._references = references
        self._previous_velocities = velocities
        self._pending = None


# Concise public alias for the standalone receiver.
NativePlanner = BimanualNativePlanner
