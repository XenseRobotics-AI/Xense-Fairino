"""Cancellable, streamed joint-space Home. Never commands head/waist/folding axes."""
import json
import math
from pathlib import Path

from tcp_core import ARMS, MODEL_ID, check_arm_limits, finite_vector
from vr_core import KIND, validate

DEFAULT_HOME = Path(__file__).with_name("home_pose.json")


def load_home(path=DEFAULT_HOME):
    data = json.loads(Path(path).read_text())
    if data.get("version") != 1 or data.get("model") != MODEL_ID:
        raise ValueError("Home 配置的版本或机器人模型不匹配")
    joints = data.get("joint_degrees")
    if not isinstance(joints, dict) or set(joints) != set(ARMS):
        raise ValueError("Home 必须同时提供左右七轴角度")
    target = {}
    for side in ARMS:
        deg = finite_vector(joints[side], 7, "Home 七轴角度")
        target[side] = check_arm_limits([math.radians(v) for v in deg])
    speed, duration = data.get("max_joint_speed_deg_s"), data.get("minimum_duration_s")
    if (type(speed) not in (int, float) or not math.isfinite(speed) or not 0 < speed <= 15.
            or type(duration) not in (int, float) or not math.isfinite(duration) or duration < 2.):
        raise ValueError("Home 时间/速度配置无效")
    return {**data, "target": target}


class HomeMotion:
    def __init__(self, home, hw, now):
        if max(abs(v) for v in finite_vector(hw["velocity"], 20, "Home 当前速度")) >= .005:
            raise ValueError("回 Home 前必须先停止并松开两侧 Grip")
        self.start = {}
        self.target = home["target"]
        for side, joints in ARMS.items():
            self.start[side] = check_arm_limits([hw["command_position"][j-1] for j in joints])
        delta = max(abs(math.degrees(b-a)) for side in ARMS for a, b in zip(self.start[side], self.target[side]))
        self.duration = max(home["minimum_duration_s"], 1.875*delta/home["max_joint_speed_deg_s"])
        self.started = self.last_sample = now
        self.settled_at = None

    def sample(self, now):
        if not math.isfinite(now) or now < self.last_sample or now-self.last_sample > .25:
            raise ValueError("Home 更新中断，已停止；请重新按右摇杆")
        self.last_sample = now
        u = max(0., min(1., (now-self.started)/self.duration))
        s = u*u*u*(10.+u*(-15.+6.*u))
        return {side: [a+(b-a)*s for a, b in zip(self.start[side], self.target[side])] for side in ARMS}

    def request(self, kin, hw, now, sequence, stamp):
        targets = self.sample(now)
        arms = {}
        for side, q in targets.items():
            tcp, _ = kin.forward_tcp(q)
            arms[side] = {"reference": [hw["command_position"][j-1] for j in ARMS[side]],
                          "target": q, "tcp": [v for row in tcp for v in row]}
        packet = {"kind": KIND, "model": MODEL_ID, "id": sequence,
                  "created": stamp, "arms": arms, "source": "home"}
        validate(hw, packet, stamp)
        return packet

    def complete(self, hw, now):
        if now-self.started < self.duration:
            return False
        close = (max(abs(hw["position"][j-1]-goal) for side, joints in ARMS.items()
                     for j, goal in zip(joints, self.target[side])) < math.radians(.2)
                 and max(abs(v) for v in hw["velocity"][:14]) < .005)
        if close:
            if self.settled_at is None:
                self.settled_at = now
            if now-self.settled_at >= .3:
                return True
        else:
            self.settled_at = None
        if now-self.started > self.duration+5.:
            raise ValueError("Home 轨迹结束后未确认到位，已停止")
        return False
