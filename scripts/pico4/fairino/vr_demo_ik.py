"""Demo-only IK without the coupled wrist envelope.

The production NativeIK adapter has no dependency on this module. Only the
explicit model demo opt-out uses this bounded least-squares solver. Individual
joint limits and independent TCP forward validation remain in force.
"""
import math

import numpy as np

import paths
from tcp_core import ARMS, MODEL_ID, check_arm_limits, multiply, pose_error, tcp_forward_matrix
from tool_frames import for_arm
from vr_core import KIND, validate
from tools.ik_experiments.art7_native import Art7NativeIK, solve_box_qp_native
from tools.ik_experiments.art7_qp import transform


class DemoBoxIK:
    def __init__(self, tools):
        self.tools = tools
        self.solvers = {side: Art7NativeIK(tools=tools, side=side) for side in ARMS}

    @staticmethod
    def solve(solver, goal, seed):
        goal = transform(goal)
        q = np.asarray(check_arm_limits(list(seed)))
        damping = .01
        # Internal numerical trust region, not a per-cycle motion limit.
        for iteration in range(60):
            error, mm, deg = solver._error(goal, solver.forward(q))
            if mm <= .02 and deg <= .01:
                break
            jacobian = solver.jacobian(q)
            jacobian[3:] *= solver.rotation_scale
            lower = np.maximum(solver.lower-q, -.2)
            upper = np.minimum(solver.upper-q, .2)
            delta = solve_box_qp_native(jacobian.T @ jacobian+damping*np.eye(7),
                                       -jacobian.T @ error, lower, upper)
            merit = float(error @ error)
            accepted = False
            for alpha in (1., .5, .25, .125, .0625):
                trial = np.clip(q+alpha*delta, solver.lower, solver.upper)
                residual = solver._error(goal, solver.forward(trial))[0]
                if float(residual @ residual) < merit-1e-12:
                    q = trial
                    damping = max(.001, damping*.5)
                    accepted = True
                    break
            if not accepted:
                damping *= 10.
                if damping > 1e5 or np.max(np.abs(delta)) < 1e-11:
                    break
        return check_arm_limits(q.tolist()), iteration+1

    def request(self, kin, hardware, goals, sequence, stamp, references=None, guard=None):
        references = references or {}
        arms = {}
        for side, goal in goals.items():
            solver = self.solvers[side]
            seed = references.get(side, [hardware['command_position'][j-1] for j in ARMS[side]])
            target, iterations = self.solve(solver, goal, seed)
            independent = np.asarray(multiply(tcp_forward_matrix(target), for_arm(self.tools, side)))
            mm, deg = pose_error(independent, solver.forward(target))
            if mm > .02 or deg > .01:
                raise ValueError('模型 IK 未通过独立正解校验')
            mm, deg = pose_error(goal, independent)
            before = solver._error(transform(goal), solver.forward(seed))[0]
            after = solver._error(transform(goal), independent)[0]
            if not math.isfinite(float(after @ after)) or float(after @ after) > float(before @ before)+1e-7:
                raise ValueError('模型 IK 增大了目标误差')
            arms[side] = dict(reference=list(seed), target=target, tcp=independent.reshape(16).tolist(),
                              native_ik=dict(success=mm <= .02 and deg <= .01,
                                  position_error_mm=float(mm), orientation_error_deg=float(deg),
                                  iterations=iterations, backend='demo_joint_box_only'))
        if not arms:
            return None
        packet = dict(kind=KIND, model=MODEL_ID, id=sequence, created=stamp,
                      tool_id=self.tools['id'], arms=arms)
        validate(hardware, packet, stamp, self.tools)
        return packet
