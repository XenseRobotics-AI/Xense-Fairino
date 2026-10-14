"""Single-axis jogging while all twenty upper-body axes hold position."""
import math
import sys

JOINTS = [f"j{i}" for i in range(1, 21)]
PROBE_JOINTS = JOINTS[:]
JOINT_GROUPS = {"left": tuple(range(1, 8)), "right": tuple(range(8, 15)),
                "body": tuple(range(15, 21))}
STEP_DEG = 0.5
ALL_MASK = (1 << len(JOINTS)) - 1
SPEED_DEG_S = 0.5
MIN_SPEED_DEG_S = 0.1
# GTK needs a finite adjustment bound; this is only the numeric type's range,
# not an application speed cap. The planners validate finite positive inputs.
MAX_SPEED_DEG_S = sys.float_info.max
TARGET_SPEED_DEG_S = 3.0
MAX_TARGET_SPEED_DEG_S = sys.float_info.max
FEEDBACK_TIMEOUT = 0.4
LINK_TIMEOUT = 0.45


class StepCancelled(Exception):
    pass


def snapshot(raw):
    names = list(raw["joint_names"])
    if len(names) != 20 or set(names) != set(JOINTS):
        raise RuntimeError("当前硬件不是已核对的 J1–J20 上半身")
    order = [names.index(j) for j in JOINTS]
    out = {"error": int(raw["hexerrorcode_curerrcode"])}
    for field in ("position", "velocity", "command_position", "stateword", "errorcode"):
        values = list(raw[field])
        if len(values) != 20 or not all(math.isfinite(v) for v in values):
            raise RuntimeError("无效的反馈：" + field)
        out[field] = [values[i] for i in order]
    out["enabled"] = [bool(int(v) & 4) for v in out["stateword"]]
    return out


def check_hardware(hw, age, gpio, selected=None, baseline=None,
                   allow_enabled=False, check_speed=True, held_positions=None,
                   moving_joints=None, moving_speed_limit=2.0, held_speed_limit=0.5):
    active = set(moving_joints) if moving_joints is not None else {selected}
    if hw is None or age > FEEDBACK_TIMEOUT:
        raise RuntimeError("上半身反馈缺失或超时")
    if gpio != [0, 0]:
        raise RuntimeError("外部急停输入未释放或不可读取")
    if hw["error"] or any(hw["errorcode"]):
        detail = ", ".join(f"J{i+1}={v}" for i, v in enumerate(hw["errorcode"]) if v)
        raise RuntimeError(f"上半身故障：0x{hw['error']:08X} {detail}")
    if not allow_enabled and any(hw["enabled"]):
        raise RuntimeError("启动前已有电机使能，拒绝接管")
    if check_speed:
        for i, velocity in enumerate(hw["velocity"]):
            limit = moving_speed_limit if i + 1 in active else held_speed_limit
            speed = math.degrees(velocity)
            if abs(speed) > limit:
                raise RuntimeError(f"J{i+1} 速度 {speed:+.3f}°/s 超过辨认程序阈值 {limit:.1f}°/s")
    if held_positions is not None:
        # The active joint follows its trajectory; all other axes keep their
        # last commanded targets, including the user's chosen body posture.
        for i in range(20):
            if i + 1 not in active and abs(math.degrees(hw["position"][i] - held_positions[i])) > 0.3:
                raise RuntimeError(f"J{i+1} 偏离保持目标超过 0.3°")
    elif baseline is not None:
        # Before activation has established hold targets, monitor the arms.
        for i in range(14):
            if abs(math.degrees(hw["position"][i] - baseline[i])) > 0.3:
                raise RuntimeError(f"J{i+1} 偏离双臂保持位置过多")


def check_packet(packet, received, now, allow_unfocused=False):
    if packet is None or now - received > LINK_TIMEOUT:
        raise RuntimeError("键控心跳中断")
    ack = packet.get("ack")
    if not isinstance(ack, (int, float)) or not math.isfinite(ack) or not 0 <= now - ack <= LINK_TIMEOUT:
        raise RuntimeError("SSH 往返通信超时")
    if packet.get("exit") or (not packet.get("focused") and not allow_unfocused):
        raise RuntimeError("窗口失焦或用户退出")


def step_duration(speed_deg_s):
    if (type(speed_deg_s) not in (int, float) or not math.isfinite(speed_deg_s)
            or speed_deg_s < MIN_SPEED_DEG_S):
        raise ValueError(f"点动速度必须是至少 {MIN_SPEED_DEG_S}°/s 的有限数值")
    duration = 1.875 * STEP_DEG / speed_deg_s
    if duration < 1e-9:
        raise ValueError("点动时间小于控制器时间戳的纳秒分辨率")
    return duration


def plan_step(hw, joint, direction, held_positions=None, speed_deg_s=SPEED_DEG_S):
    if type(joint) is not int or joint not in range(1, 21):
        raise ValueError("只允许点动 J1–J20")
    if type(direction) is not int or direction not in (-1, 1):
        raise ValueError("方向必须为 -1 或 +1")
    duration = step_duration(speed_deg_s)
    start = list(hw["position"] if held_positions is None else held_positions)
    if len(start) != 20 or not all(math.isfinite(x) for x in start):
        raise ValueError("无效的二十轴保持目标")
    start[joint - 1] = hw["position"][joint - 1]
    target = start[:]
    target[joint - 1] += math.radians(STEP_DEG) * direction
    return {"joint": joint, "mask": 1 << (joint - 1), "start": start,
            "target": target, "duration": duration, "speed_deg_s": float(speed_deg_s)}


def target_duration(distance_deg, speed_deg_s=TARGET_SPEED_DEG_S):
    if (type(speed_deg_s) not in (int, float) or not math.isfinite(speed_deg_s)
            or speed_deg_s < MIN_SPEED_DEG_S):
        raise ValueError(f"目标运动速度必须是至少 {MIN_SPEED_DEG_S}°/s 的有限数值")
    if type(distance_deg) not in (int, float) or not math.isfinite(distance_deg):
        raise ValueError("目标角度差必须是有限数值")
    # Zero velocity/acceleration endpoints give a quintic JTC segment.
    # max(d/ds (10s^3-15s^4+6s^5)) = 1.875, so the peak is <= speed.
    duration = max(0.1, 1.875 * abs(distance_deg) / speed_deg_s)
    if not math.isfinite(duration) or duration + 0.1 >= 2147483647:
        raise ValueError("目标运动时间超出控制器可表示范围")
    return duration


def plan_target(hw, joint, target_deg, held_positions=None, speed_deg_s=TARGET_SPEED_DEG_S):
    if type(joint) is not int or joint not in range(1, 21):
        raise ValueError("只允许调整 J1–J20")
    if type(target_deg) not in (int, float) or not math.isfinite(target_deg):
        raise ValueError("目标角度必须是有限数值（单位 °）")
    start = list(hw["position"] if held_positions is None else held_positions)
    if len(start) != 20 or not all(math.isfinite(x) for x in start):
        raise ValueError("无效的二十轴保持目标")
    start[joint-1] = hw["position"][joint-1]
    target = start[:]
    target[joint-1] = math.radians(target_deg)
    duration = target_duration(target_deg-math.degrees(start[joint-1]), speed_deg_s)
    return {"kind": "target", "joint": joint, "mask": 1 << (joint-1),
            "start": start, "target": target, "target_deg": float(target_deg),
            "duration": duration, "speed_deg_s": float(speed_deg_s)}


def plan_request(hw, request, held_positions=None):
    kind = request.get("kind", "step")
    if kind == "target":
        return plan_target(hw, request.get("joint"), request.get("target_deg"),
                           held_positions, request.get("speed_deg_s", TARGET_SPEED_DEG_S))
    if kind != "step":
        raise ValueError("未知的关节运动请求类型")
    return plan_step(hw, request.get("joint"), request.get("direction"),
                     held_positions, request.get("speed_deg_s", SPEED_DEG_S))


def fresh_request(request, now):
    if not isinstance(request, dict) or type(request.get("id")) is not int:
        raise RuntimeError("点动请求格式不正确")
    created = request.get("created")
    if not isinstance(created, (int, float)) or not math.isfinite(created) or not 0 <= now-created <= 0.5:
        raise ValueError("这次按键已过期，请重新点按")
