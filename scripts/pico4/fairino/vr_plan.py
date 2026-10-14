"""Vendor IK for the complete sensitivity-scaled flange target."""
import numpy as np
from tcp_core import ARMS, MODEL_ID, check_arm_limits
from vr_core import KIND, matrix


def entry(kin, hw, side, goal, reference=None):
    reference = ([hw["command_position"][j-1] for j in ARMS[side]]
                 if reference is None else check_arm_limits(reference))
    _, arm_angle = kin.forward_tcp(reference)
    goal = np.asarray(goal, dtype=float)
    if goal.shape != (4, 4) or not np.isfinite(goal).all():
        raise ValueError("无效手柄目标")
    tcp = matrix(goal.flatten().tolist())
    target = kin.inverse_tcp(tcp, arm_angle, reference)
    check_arm_limits(target)
    return {"reference": reference, "target": target, "tcp": goal.flatten().tolist()}


def request(kin, hw, goals, sequence, stamp, references=None):
    if not goals:
        return None
    # Compute both before sending: partial success must never move just one arm.
    return {"kind": KIND, "model": MODEL_ID, "id": sequence, "created": stamp,
            "arms": {side: entry(kin, hw, side, goal, (references or {}).get(side))
                     for side, goal in goals.items()}}
