"""Full-target constrained IK adapter for the existing PICO position stream.

This adapter has no command history: ``references`` are the caller's previous
planned solutions, not an acknowledgement of robot publication or arrival.
The existing receiver independently checks the path from its actual last
published command and measured joints before publishing the complete packet.
"""
from __future__ import annotations

import copy
import math
import sys

import numpy as np

from paths import ROOT
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.ik_experiments.art7_native import Art7NativeIK
from tools.ik_experiments.art7_qp import transform, vector
from tcp_core import (ARMS, MODEL_ID, check_arm_limits, finite_vector,
                      multiply, pose_error, tcp_forward_matrix)
from tool_frames import for_arm, tool_config, validate_runtime
from vr_core import KIND, validate
from wrist_guard import WristGuard, PREDICTION_SECONDS


class NativeIK:
    """Load C++ eagerly, then plan complete requests without publishing them."""

    def __init__(self, tools=None):
        self.tools = copy.deepcopy(validate_runtime(tool_config() if tools is None else tools))
        # Construct before connecting/enabling: the first load may compile C++.
        self.solvers = {side: Art7NativeIK(tools=self.tools, side=side) for side in ARMS}
        self.guard = WristGuard()

    @staticmethod
    def _hardware(hardware):
        if not isinstance(hardware, dict):
            raise ValueError('native IK requires current 20-axis hardware feedback')
        hw = {name: finite_vector(hardware.get(name), 20, name)
              for name in ('position', 'command_position', 'velocity')}
        for joints in ARMS.values():
            for name in ('position', 'command_position'):
                check_arm_limits([hw[name][j-1] for j in joints])
        return hw

    def _entry(self, kin, hw, side, goal, reference, guard):
        joints = ARMS[side]
        reference = check_arm_limits(reference if reference is not None else
                                     [hw['command_position'][j-1] for j in joints])
        measured = [hw['position'][j-1] for j in joints]
        velocity = [hw['velocity'][j-1] for j in joints]
        solver = self.solvers[side]
        result = solver.solve(goal, reference, measured=measured,
                              measured_velocity=velocity, allow_guard_retreat=True)
        if not result.get('feasible') or result.get('target') is None:
            raise ValueError(f'{side}: native IK refused the complete request: '
                             +str(result.get('status')))
        target = check_arm_limits(vector(result['target'], 'target').tolist())
        # Retain the original final path checks in addition to the constraints
        # inside IK. The receiver repeats them against its published history.
        guard.arm_segment(reference, target, velocity)
        guard.arm_segment(measured, target, velocity)
        independent = np.asarray(multiply(tcp_forward_matrix(target), for_arm(self.tools, side)))
        reported = transform(result.get('tcp'))
        vendor, _ = kin.forward_tcp(target, side)
        for actual in (reported, transform(vendor)):
            mm, deg = pose_error(actual, independent)
            if mm > .02 or deg > .01:
                raise ValueError(f'{side}: native IK failed independent TCP FK validation')
        error_mm, error_deg = pose_error(goal, independent)
        if result.get('success') and (error_mm > .02 or error_deg > .01):
            raise ValueError(f'{side}: a partial native IK answer was incorrectly reported as solved')
        # A feasible partial answer may approach an unreachable target, but it
        # must not increase the solver's full-pose weighted error from the seed.
        origin = np.asarray(multiply(tcp_forward_matrix(reference), for_arm(self.tools, side)))
        before_error = solver._error(goal, origin)[0]
        after_error = solver._error(goal, independent)[0]
        before, after = float(before_error @ before_error), float(after_error @ after_error)
        if after > before+1e-7*(1.+before):
            raise ValueError(f'{side}: native IK moved away from the full TCP target')
        details = dict(backend='native_cpp17', status=str(result.get('status')),
                       feasible=True, success=bool(result.get('success')),
                       position_error_mm=float(error_mm), orientation_error_deg=float(error_deg),
                       iterations=int(result.get('iterations', 0)),
                       qp_calls=int(result.get('qp_calls', 0)),
                       elapsed_ms=float(result.get('elapsed_ms', 0.)), solve_mode='full_target',
                       requested_tcp=goal.reshape(16).tolist(),
                       wrist_reserve_deg=[abs(math.degrees(v))*PREDICTION_SECONDS
                                          for v in velocity[-2:]])
        # No wrist_scale: partial progress must retain the full input target,
        # never rebase the clutch onto a shortened target or accumulate drift.
        return dict(reference=reference, target=target,
                    tcp=independent.reshape(16).tolist(), native_ik=details)

    def request(self, kin, hardware, goals, sequence, stamp, references=None, guard=None):
        """Return the existing joint packet only after every requested arm passes.

        Missing arms are omitted so the receiver holds its exact last command.
        No dt, velocity/acceleration box, integration tail, or target scaling is
        introduced. Measured wrist speed retains the original prediction reserve.
        """
        if not isinstance(goals, dict) or not set(goals) <= set(ARMS):
            raise ValueError('goals must map left/right to finite TCP transforms')
        if not goals:
            return None
        if type(sequence) is not int or sequence <= 0:
            raise ValueError('native IK requires a positive integer request id')
        if type(stamp) not in (int, float) or not math.isfinite(stamp):
            raise ValueError('native IK requires a finite receiver timestamp')
        if validate_runtime(getattr(kin, 'tools', tool_config())) != self.tools:
            raise ValueError('native IK and client TCP tool configurations do not match')
        references = {} if references is None else references
        if not isinstance(references, dict) or not set(references) <= set(ARMS):
            raise ValueError('native IK references must map left/right to joint vectors')
        guard = self.guard if guard is None else guard
        if not guard.enabled:
            raise ValueError('native IK requires the coupled wrist guard to remain enabled')
        hw = self._hardware(hardware)
        validated_goals = {side: transform(goal) for side, goal in goals.items()}
        arms = {side: self._entry(kin, hw, side, goal, references.get(side), guard)
                for side, goal in validated_goals.items()}
        packet = dict(kind=KIND, model=MODEL_ID, id=sequence, created=stamp,
                      tool_id=self.tools['id'], ik_backend='native', arms=arms)
        validate(hw, packet, stamp, self.tools)
        return packet
