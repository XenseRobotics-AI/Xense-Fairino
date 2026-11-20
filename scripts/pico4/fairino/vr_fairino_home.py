"""Cancellable arm Home followed by a return along the session's fixed-pivot path."""
import math

import numpy as np

from vr_fairino_ik import (ARM_SLICE, AXES, RESTART_GAP, ROUNDING_RAD,
                           _assert_fk, _CycleFK)
from mounts import BODY_FROM_DH
from tcp_core import finite_vector
from tools.ik_experiments.fairino_qp_art7 import rigid_transform
from vr_home import HomeMotion

MIN_DURATION_S = 2.
# Fairino QP only. The shared home_pose.json (8 deg/s) still drives the other backends.
ARM_HOME_SPEED_DEG_S = 15.
TORSO_RETURN_DEG_S = 8.
QUINTIC_PEAK = 1.875
SETTLE_RAD = math.radians(.2)
SETTLE_RAD_S = .005
SETTLE_TIMEOUT_S = 5.


def return_duration(path, start, end):
    """Bound dq/ds along the whole knee/pitch/yaw line, s in [0,1]."""
    m = path.model
    knee = math.radians(max(start[0], end[0]))
    radius2 = m.l1**2 + m.l2**2 + 2*m.l1*m.l2*math.cos(knee)
    z_min = math.sqrt(radius2 - m.x**2)
    # J20 = atan2(x,z) - atan2(l2*sin(k), l1+l2*cos(k)).
    # The second atan2 has derivative 1/2 + (l2²-l1²)/(2r²) with respect to k.
    # The first has absolute derivative <= |x|*l1*l2/(r²*z).
    # Both r and z are smallest at max(k).
    # This bounds both |dJ20/dk| and |1+dJ20/dk| (J18 = pitch+C-k-J20).
    slope = .5 + abs(m.l2**2-m.l1**2)/(2*radius2) + abs(m.x)*m.l1*m.l2/(radius2*z_min)
    dk, dp, dy = abs(end-start)
    return max(MIN_DURATION_S, QUINTIC_PEAK*max(dy, dk, dp+slope*dk)/TORSO_RETURN_DEG_S)


class FairinoHomeMotion:
    """Same streaming interface as HomeMotion. The caller handles Grip and pause."""
    def __init__(self, home, hw, now, guard, backend):
        if not backend.demo_only and not guard.enabled:
            raise ValueError('Fairino QP 实机腕部联合限位保护必须保持开启')
        self.backend, self.guard = backend, guard
        self.arms = HomeMotion(dict(home, max_joint_speed_deg_s=ARM_HOME_SPEED_DEG_S), hw, now, guard)
        self.last_sample = now
        self.final_sent = False

    @property
    def returning_torso(self):
        return self.arms is None

    def _check_time(self, now):
        if not math.isfinite(now) or not 0. <= now-self.last_sample <= RESTART_GAP:
            raise ValueError('Home 更新中断，已停止；请重新触发复位')

    def complete(self, hw, now):
        self._check_time(now)
        if self.arms is not None:
            if not self.arms.complete(hw, now):
                return False
            path = self.backend.path
            q = np.array(finite_vector(hw['command_position'], 20, 'Home 当前指令'), dtype=float)
            if path is None or np.array_equal(q[AXES], path.start_target):
                self.backend.reset()
                return True
            self.start = path.coordinates(q)
            self.last_target = q[AXES].copy()
            q[AXES] = path.start_target
            self.end = path.coordinates(q)
            self.started = self.last_sample = now
            self.duration = return_duration(path, self.start, self.end)
            self.arms = None
        if self.final_sent:
            position = np.array(finite_vector(hw['position'], 20, 'Home 实测姿态'))
            velocity = np.array(finite_vector(hw['velocity'], 20, 'Home 实测速度'))
            if (max(abs(position[AXES]-self.last_target)) < SETTLE_RAD
                    and max(abs(velocity[AXES])) < SETTLE_RAD_S):
                self.backend.reset()
                return True
            if now-self.started > self.duration+SETTLE_TIMEOUT_S:
                raise ValueError('躯干 Home 结束后未确认到位，已停止')
        return False

    def request(self, kin, hw, now, sequence, stamp):
        self._check_time(now)
        backend = self.backend
        if self.arms is not None:
            packet = self.arms.request(kin, hw, now, sequence, stamp)
            if backend.demo_only:
                packet['demo_only'] = True
            self.last_sample = now
            return packet

        q = np.array(finite_vector(hw['command_position'], 20, 'Home 当前指令'), dtype=float)
        current = q[AXES].copy()
        path = backend.path
        path.coordinates(q)
        u = min(1., (now-self.started)/self.duration)
        if u >= 1.:
            q[AXES] = path.start_target
        elif u > 0.:
            s = u**3 * (10.+u*(-15.+6.*u))
            q[AXES] = path.joints(self.start+s*(self.end-self.start), q)[AXES]
        limit = math.radians(TORSO_RETURN_DEG_S)*(now-self.last_sample) + ROUNDING_RAD
        if any(max(abs(q[AXES]-reference)) > limit for reference in (current, self.last_target)):
            raise ValueError('躯干 Home 指令超出本周期限速')
        path.coordinates(q)

        # The hands stay in joint space. Certify their actual configuration
        # at this torso, including the same independent FK and collision gate.
        if backend._validated_tools(kin.tools) != backend.tools:
            raise ValueError('Fairino QP TCP 配置不一致')
        goals = {side: backend.solver.forward(side, q) for side in ARM_SLICE}
        hold = backend.solver.solve_arms(goals, q, max_iterations=1)
        if not hold['collision_free'] or not np.array_equal(hold['q'], q):
            raise ValueError('躯干 Home 最终姿态未通过机身避障检查')
        fk = _CycleFK(backend)
        world = fk.world(q)
        _assert_fk(world, rigid_transform(backend.solver.body_frame(q), 1e-6), '躯干 Home')
        entries = {}
        for side, arm in ARM_SLICE.items():
            target, tcp, _ = backend._verified(kin, side, q[arm], hold['tcp'][side],
                                                world @ BODY_FROM_DH[side], fk)
            self.guard.arm_segment(target, target, hw['velocity'][arm])
            self.guard.arm_segment(hw['position'][arm], target, hw['velocity'][arm])
            entries[side] = dict(reference=target[:], target=target, tcp=tcp.ravel().tolist())
        packet = backend.checked_packet(hw, entries, q, sequence, stamp, source='home')
        self.last_sample, self.last_target, self.final_sent = now, q[AXES], u >= 1.
        return packet
