"""Vendor IK for the complete sensitivity-scaled flange target."""
import numpy as np
from tcp_core import ARMS, MODEL_ID, check_arm_limits
from vr_core import KIND, matrix
from tool_frames import packet_tool_id
from wrist_guard import WristGuard, WristBlocked


def entry(kin, hw, side, goal, reference=None, guard=None):
    guard = guard if guard is not None else WristGuard()
    reference = ([hw["command_position"][j-1] for j in ARMS[side]]
                 if reference is None else check_arm_limits(reference))
    origin, arm_angle = kin.forward_tcp(reference, side)
    goal = np.asarray(goal, dtype=float)
    if goal.shape != (4, 4) or not np.isfinite(goal).all():
        raise ValueError("无效手柄目标")
    tcp = matrix(goal.flatten().tolist())
    target = kin.inverse_tcp(tcp, arm_angle, reference, side)
    check_arm_limits(target)
    scale = guard.approach_scale(reference, target)
    actual = [hw["position"][j-1] for j in ARMS[side]]
    velocity = [hw["velocity"][j-1] for j in ARMS[side]]
    for fraction in (1., .5, .25, .125, .0625, .03125, 0.):
        amount = scale*fraction
        # A cheap joint chord can reject an obviously oversized trial before
        # another vendor IK call. It is only a rejection heuristic: the actual
        # solved path is always checked below, never approved from this estimate.
        estimate = [a+amount*(b-a) for a, b in zip(reference, target)]
        try:
            guard.arm_segment(reference, estimate, velocity)
            guard.arm_segment(actual, estimate, velocity)
        except WristBlocked:
            continue
        candidate = (goal if amount == 1 else np.asarray(origin) if amount == 0
                     else interpolate_tcp(origin, goal, amount))
        q = target if amount == 1 else (reference[:] if amount == 0 else
            kin.inverse_tcp(matrix(candidate.flatten().tolist()), arm_angle, reference, side))
        check_arm_limits(q)
        try:
            guard.arm_segment(reference, q, velocity)
            guard.arm_segment(actual, q, velocity)
        except WristBlocked:
            continue
        return {"reference": reference, "target": q, "tcp": candidate.flatten().tolist(),
                "wrist_scale": amount}
    raise WristBlocked("腕部接近联合边界；已暂停，请松开 Grip 后向内收腕")


def interpolate_tcp(origin, goal, amount):
    # Re-solve IK for a smaller Cartesian step; never clamp q after solving.
    from lerobot.utils.rotation import Rotation
    from vr_cartesian import slerp
    origin, goal = np.asarray(origin), np.asarray(goal)
    result = np.eye(4)
    result[:3, 3] = origin[:3, 3]+amount*(goal[:3, 3]-origin[:3, 3])
    result[:3, :3] = Rotation.from_quat(slerp(
        Rotation.from_matrix(origin[:3, :3]).as_quat(),
        Rotation.from_matrix(goal[:3, :3]).as_quat(), amount)).as_matrix()
    return result


def request(kin, hw, goals, sequence, stamp, references=None, guard=None):
    if not goals:
        return None
    # Compute both before sending: partial success must never move just one arm.
    return {"kind": KIND, "model": MODEL_ID, "id": sequence, "created": stamp,
            "tool_id": packet_tool_id(kin),
            "arms": {side: entry(kin, hw, side, goal, (references or {}).get(side), guard)
                     for side, goal in goals.items()}}
