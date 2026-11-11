"""TRON2 C++ QP for the existing ART7 PICO position stream.

The upstream QP enforces individual joint limits. The mandatory real wrist
guard validates paths using measured feedback/velocity outside QP and retreats
the Cartesian goal when necessary. The Demo subclass may disable this guard.
No connection, publication or joint velocity/acceleration ramp is added here.
"""
import copy
import math
import sys

import numpy as np

from paths import ROOT
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tcp_core import ARMS, MODEL_ID, check_arm_limits, finite_vector, multiply, pose_error, tcp_forward_matrix
from tool_frames import for_arm, validate_runtime
from tools.ik_experiments.art7_qp import transform
from vr_core import KIND, validate
from vr_plan import interpolate_tcp
from wrist_guard import PREDICTION_SECONDS, WristBlocked, WristGuard


class TronIK:
    demo_only = False

    def __init__(self, tools):
        # Load optional native dependencies before connecting or enabling the robot.
        from tools.ik_experiments.tron2_art7 import Art7TronIK
        self.tools = copy.deepcopy(validate_runtime(tools))
        self.solver = Art7TronIK(self.tools)
        # Detect a model/tool mismatch locally before any real enable request.
        for side in ARMS:
            for q in ([0.]*7, np.radians([45., 75., -100., 85., 85., 20., -10.]).tolist()):
                independent = multiply(tcp_forward_matrix(q), for_arm(self.tools, side))
                mm, deg = pose_error(transform(self.solver.forward(side, q)), independent)
                if mm > .02 or deg > .01:
                    raise ValueError(f'{side}: TRON2 启动模型正解不一致')

    def _verified(self, kin, side, result):
        target = check_arm_limits(list(result['target']))
        actual = np.asarray(multiply(tcp_forward_matrix(target), for_arm(self.tools, side)))
        for reported in (transform(result['tcp']), transform(kin.forward_tcp(target, side)[0])):
            mm, deg = pose_error(reported, actual)
            if mm > .02 or deg > .01:
                raise ValueError(f'{side}: TRON2 IK 未通过独立 TCP 正解校验')
        return target, actual

    def _entry(self, kin, side, goal, result, hw, references, guard):
        reference = references[side]
        joints = ARMS[side]
        measured = [hw['position'][j-1] for j in joints]
        velocity = [hw['velocity'][j-1] for j in joints]
        origin = np.asarray(multiply(tcp_forward_matrix(reference), for_arm(self.tools, side)))
        full_target, full_tcp = self._verified(kin, side, result)
        scale = guard.approach_scale(reference, full_target)
        for fraction in (1., .5, .25, .125, .0625, .03125, 0.):
            amount = scale*fraction
            if amount == 1.:
                trial = result
                target, actual = full_target, full_tcp
            elif amount == 0.:
                trial = dict(target=reference, tcp=origin, iterations=0,
                             status='wrist_hold', elapsed_ms=0., qp_iterations=0)
                target, actual = self._verified(kin, side, trial)
            else:
                trial_goal = interpolate_tcp(origin, goal, amount)
                trial = self.solver.solve({side: trial_goal}, references)[side]
                target, actual = self._verified(kin, side, trial)
            try:
                guard.arm_segment(reference, target, velocity)
                guard.arm_segment(measured, target, velocity)
            except WristBlocked:
                continue
            mm, deg = pose_error(goal, actual)
            before_mm, before_deg = pose_error(goal, origin)
            # Upstream uses equally weighted metres and radians. Check with
            # independent ART7 FK; an unreachable target must not cause drift.
            before = (before_mm/1000.)**2 + math.radians(before_deg)**2
            after = (mm/1000.)**2 + math.radians(deg)**2
            if after > before + 1e-12*(1.+before):
                raise ValueError(f'{side}: TRON2 IK 增大了完整目标误差，保持当前姿态')
            details = dict(backend='tron2_cpp_qp', status=str(trial.get('status', '')),
                           success=mm <= .02 and deg <= .01,
                           position_error_mm=float(mm), orientation_error_deg=float(deg),
                           iterations=int(trial.get('iterations', 0)),
                           elapsed_ms=float(trial.get('elapsed_ms', 0.)),
                           qp_iterations=int(trial.get('qp_iterations', 0)),
                           requested_tcp=goal.reshape(16).tolist(),
                           coupled_wrist_in_qp=False, feasible=True,
                           wrist_reserve_deg=[abs(math.degrees(v))*PREDICTION_SECONDS
                                              for v in velocity[-2:]])
            # Same rebasing contract as vendor IK: rejected wrist displacement
            # is discarded, so a held controller cannot accumulate a backlog.
            return dict(reference=reference[:], target=target,
                        tcp=actual.reshape(16).tolist(), native_ik=details,
                        wrist_scale=amount)
        raise WristBlocked(f'{side}: TRON2 腕部路径不通过，请松开 Grip 后向内收腕')

    def request(self, kin, hardware, goals, sequence, stamp, references=None, guard=None):
        if not isinstance(goals, dict) or not set(goals) <= set(ARMS):
            raise ValueError('TRON2 目标须为 left/right TCP')
        if not goals:
            return None
        if type(sequence) is not int or sequence <= 0 or type(stamp) not in (int, float) or not math.isfinite(stamp):
            raise ValueError('TRON2 请求编号或时间无效')
        if validate_runtime(kin.tools) != self.tools:
            raise ValueError('TRON2 TCP 配置不一致')
        if not isinstance(hardware, dict):
            raise ValueError('TRON2 需要当前二十轴反馈')
        hw = {name: finite_vector(hardware.get(name), 20, name)
              for name in ('position', 'command_position', 'velocity')}
        references = {} if references is None else references
        if not isinstance(references, dict) or not set(references) <= set(ARMS):
            raise ValueError('TRON2 参考关节无效')
        seeds = {side: check_arm_limits(list(references.get(side,
                    [hw['command_position'][j-1] for j in joints]))) for side, joints in ARMS.items()}
        for joints in ARMS.values():
            check_arm_limits([hw['position'][j-1] for j in joints])
            check_arm_limits([hw['command_position'][j-1] for j in joints])
        goals = {side: transform(goal) for side, goal in goals.items()}
        guard = WristGuard() if guard is None else guard
        if not self.demo_only and not guard.enabled:
            raise ValueError('TRON2 实机腕部联合限位保护必须保持开启')
        results = self.solver.solve(goals, seeds)
        arms = {side: self._entry(kin, side, goal, results[side], hw, seeds, guard)
                for side, goal in goals.items()}
        packet = dict(kind=KIND, model=MODEL_ID, id=sequence, created=stamp,
                      tool_id=self.tools['id'], arms=arms, ik_backend='tron2')
        if self.demo_only:
            packet['demo_only'] = True
        validate(hw, packet, stamp, self.tools)
        return packet
