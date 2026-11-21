"""Causal TCP resampling before IK: linear translation and shortest-arc SLERP.

Samples use the PC's monotonic receive clock. A small explicit playback delay
allows interpolation between received frames; missing future data is held,
never extrapolated. Buttons/Grip are processed immediately, outside this delay.
"""
from collections import deque
import math
import numpy as np
from lerobot.utils.rotation import Rotation
from tcp_core import ARMS


def slerp(a, b, u):
    a = np.asarray(a, dtype=float); a = a/np.linalg.norm(a)
    b = np.asarray(b, dtype=float); b = b/np.linalg.norm(b)
    dot = float(a @ b)
    if dot < 0.:
        b, dot = -b, -dot
    dot = np.clip(dot, -1., 1.)
    if dot > 1.-1e-10:
        result = (1.-u)*a+u*b
    else:
        theta = math.acos(dot)
        result = (math.sin((1.-u)*theta)*a+math.sin(u*theta)*b)/math.sin(theta)
    return result/np.linalg.norm(result)


class CartesianBuffer:
    def __init__(self, delay_ms=12.):
        if not math.isfinite(delay_ms) or not 0 <= delay_ms <= 200:
            raise ValueError("TCP 插值缓冲须为 0–200 ms")
        self.delay = delay_ms/1000.
        self.paths = {}
        self.last_metadata = {}

    def clear(self, side=None):
        if side is None:
            self.paths.clear()
            self.last_metadata.clear()
        else:
            self.paths.pop(side, None)
            self.last_metadata.pop(side, None)

    def push(self, side, stamp, goal, reset_position=False, reset_rotation=False):
        goal = np.asarray(goal, dtype=float)
        if goal.shape != (4, 4) or not np.isfinite(goal).all() or not math.isfinite(stamp):
            raise ValueError("无效 TCP 插值样本")
        paths = self.paths.setdefault(side, (deque(maxlen=128), deque(maxlen=128)))
        values = (goal[:3, 3].copy(), Rotation.from_matrix(goal[:3, :3]).as_quat())
        for path, value, reset in zip(paths, values, (reset_position, reset_rotation)):
            if reset:
                path.clear()
            if path and stamp < path[-1][0]:
                raise ValueError("TCP 样本时间倒退")
            if path and stamp == path[-1][0]:
                path.pop()
            path.append((stamp, value))

    @staticmethod
    def component(path, stamp, rotation=False):
        while len(path) > 2 and path[1][0] <= stamp:
            path.popleft()
        if stamp <= path[0][0]:
            return path[0][1], path[0][0], "hold_first"
        if stamp >= path[-1][0]:
            return path[-1][1], path[-1][0], "hold_latest"
        for (t0, a), (t1, b) in zip(path, list(path)[1:]):
            if t0 <= stamp <= t1:
                u = (stamp-t0)/(t1-t0)
                return (slerp(a, b, u) if rotation else a+(b-a)*u), stamp, "interpolated"
        raise ValueError("TCP 插值时间不在有效样本区间")

    def sample(self, now):
        goals = {}
        self.last_metadata = {}
        for side, (positions, rotations) in self.paths.items():
            p, tp, mp = self.component(positions, now-self.delay)
            r, tr, mr = self.component(rotations, now-self.delay, rotation=True)
            goal = np.eye(4)
            goal[:3, 3] = p
            goal[:3, :3] = Rotation.from_quat(r).as_matrix()
            goals[side] = goal
            self.last_metadata[side] = {"position_age_ms": 1000*(now-tp),
                "rotation_age_ms": 1000*(now-tr), "position_mode": mp, "rotation_mode": mr}
        return goals


class CartesianInput:
    """Map every received frame, then sample Cartesian goals once per IK tick."""
    def __init__(self, clutches, delay_ms=12., *, automatic=False):
        self.automatic = automatic
        self.clutches = clutches
        for clutch in clutches.values():
            clutch.automatic = automatic
        self.buffer = CartesianBuffer(delay_ms)
        self.references = {}

    def reset(self):
        self.buffer.clear()
        self.references.clear()
        for clutch in self.clutches.values():
            clutch.reset()

    def goals(self, frame, hw, kin, hands, sensitivity, freeze_buttons, now):
        position = hw["position" if self.automatic else "command_position"]
        current = {s: kin.forward_tcp([position[j-1] for j in js], s)[0]
                   for s, js in ARMS.items()}
        samples = frame.get("samples", [frame])
        if self.automatic and not all(c.engaged for c in self.clutches.values()):
            samples = [frame]  # Anchor to the latest pose, dropping all pre-hold history.
        for sample in samples:
            stamp = sample.get("received_monotonic", now)
            for side, joints in ARMS.items():
                c = self.clutches[side]
                previous = (c.engaged, c.frozen_position, c.frozen_rotation)
                hand = sample[hands[side]]
                fp, fr = freeze_buttons and hand["secondary"], freeze_buttons and hand["primary"]
                goal = c.target(hand, current[side], sensitivity,
                    max(abs(hw["velocity"][j-1]) for j in joints) < .005,
                    freeze_position=fp, freeze_rotation=fr)
                if goal is None:
                    self.buffer.clear(side)
                    self.references.pop(side, None)
                else:
                    self.buffer.push(side, stamp, goal,
                        reset_position=not previous[0] or fp != previous[1],
                        reset_rotation=not previous[0] or fr != previous[2])
        return self.buffer.sample(now)

    def accepted_plan(self, packet, frame=None, hands=None):
        # Previous solution is the IK reference to favor the same branch. This
        # is a planned position, not a claim that the robot already reached it.
        self.references = {s: v["target"][:] for s, v in packet["arms"].items()}
        if frame is not None and hands is not None:
            for side, value in packet["arms"].items():
                if value.get("wrist_scale", 1.) < 1.:
                    self.clutches[side].rebase(frame[hands[side]], np.asarray(value["tcp"]).reshape(4, 4))
                    self.buffer.clear(side)
