"""Pure control rules, shared by the desktop window and robot-side worker."""
import math
import copy
from collections import namedtuple

JOINTS = [f"j{i}" for i in range(1, 9)]
STEER = [0, 2, 4, 6]
DRIVE = [1, 3, 5, 7]
KEYS = frozenset("wasdjl")
FEEDBACK_TIMEOUT = 0.4
LINK_TIMEOUT = 0.45
DEFAULT_LINEAR = 0.06
DEFAULT_ANGULAR = 0.10
MIN_SPEED = 0.01
MAX_LINEAR = 0.20
MAX_ANGULAR = 0.40
DRIVE_COMMAND_LIMIT = 6.0
DRIVE_FEEDBACK_LIMIT = 7.0


def chassis_parameters(params):
    """One profile for installation and runtime verification."""
    params = copy.deepcopy(params)
    params['cmd_vel_topic'] = '/fairino_keyboard/cmd_vel'
    params['odom_topic'] = '/eight_dof_chassis_controller/odom'
    params['limits']['base']['linear'].update(max_velocity=MAX_LINEAR, max_acceleration=.20, max_jerk=.8)
    params['limits']['base']['angular'].update(max_velocity=MAX_ANGULAR, max_acceleration=.40, max_jerk=1.6)
    params['limits']['steering'].update(max_velocity=.35, max_acceleration=.7)
    params['limits']['drive'].update(max_velocity=DRIVE_COMMAND_LIMIT, max_acceleration=6., max_jerk=24.)
    return params


def check_speeds(linear, angular):
    for value, maximum, label in ((linear, MAX_LINEAR, "平移"),
                                  (angular, MAX_ANGULAR, "旋转")):
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not MIN_SPEED <= value <= maximum):
            raise RuntimeError(f"{label}速度必须在 {MIN_SPEED:g}–{maximum:g} 范围内")
    return float(linear), float(angular)


def check_speed_envelope(params):
    """Include simultaneous translation + yaw when checking wheel limits."""
    modules = params.get("modules", {})
    if len(modules) != 4:
        raise RuntimeError("缺少四轮几何配置，无法核对调速范围")
    for name, module in modules.items():
        radius = module["wheel_radius"]
        x, y, _ = module["base_to_steering"]["xyz"]
        offset = module.get("wheel_center_offset", 0.0)
        if not all(math.isfinite(v) for v in (radius, x, y, offset)) or radius <= 0:
            raise RuntimeError("轮组几何配置无效：" + name)
        worst = (MAX_LINEAR + MAX_ANGULAR * (math.hypot(x, y) + abs(offset))) / radius
        if worst > DRIVE_COMMAND_LIMIT:
            raise RuntimeError("调速范围超过轮组速度限制：" + name)


def key_twist(keys, linear=DEFAULT_LINEAR, angular=DEFAULT_ANGULAR):
    keys = set(keys)
    x = int("w" in keys) - int("s" in keys)
    y = int("a" in keys) - int("d" in keys)
    scale = linear / max(1.0, math.hypot(x, y))
    return x * scale, y * scale, angular * (int("j" in keys) - int("l" in keys))


class AxisInput(namedtuple('AxisInput', 'forward left yaw')):
    """Normalized continuous input; a neutral sample is false during startup."""
    __slots__ = ()

    def __bool__(self):
        return any(value != 0. for value in self)


def check_axes(values):
    if (not isinstance(values, (list, tuple)) or len(values) != 3
            or any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 1.
                   for v in values)):
        raise RuntimeError('底盘摇杆必须是三个 -1～1 的有限数值')
    if math.hypot(values[0], values[1]) > 1.+1e-9:
        raise RuntimeError('底盘平移摇杆超出单位圆')
    return AxisInput(*(float(v) for v in values))


def input_twist(value, linear=DEFAULT_LINEAR, angular=DEFAULT_ANGULAR):
    if isinstance(value, AxisInput):
        return value.forward*linear, value.left*linear, value.yaw*angular
    return key_twist(value, linear, angular)


def normalise_hardware(raw):
    names = list(raw["joint_names"])
    if len(names) != 8 or set(names) != set(JOINTS):
        raise ValueError("关节名称或数量不符合这台八电机底盘")
    order = [names.index(j) for j in JOINTS]
    result = {"error": int(raw["hexerrorcode_curerrcode"])}
    for field in ("position", "velocity", "command_position", "command_velocity",
                  "stateword", "errorcode"):
        values = list(raw[field])
        if len(values) != 8 or not all(math.isfinite(v) for v in values):
            raise ValueError("无效的反馈字段：" + field)
        result[field] = [values[i] for i in order]
    result["enabled"] = [bool(int(v) & 4) for v in result["stateword"]]
    return result


def check_feedback(hw, hw_age, upper, upper_age):
    if hw is None or hw_age > FEEDBACK_TIMEOUT:
        raise RuntimeError("底盘反馈缺失或超过 0.4 秒未更新")
    if upper is None or upper_age > FEEDBACK_TIMEOUT:
        raise RuntimeError("上身急停反馈缺失或超过 0.4 秒未更新")
    if upper != 0:
        raise RuntimeError(f"上身急停/故障：0x{upper:08X}")
    if hw["error"] or any(hw["errorcode"]):
        details = ", ".join(f"J{i+1}={v}" for i, v in enumerate(hw["errorcode"]) if v)
        raise RuntimeError(f"底盘故障 0x{hw['error']:08X} {details}")
    if any(abs(hw["position"][i]) > 1.678 for i in STEER):
        raise RuntimeError("转向角超过已核对的机械限位")
    if any(abs(hw["velocity"][i]) > DRIVE_FEEDBACK_LIMIT for i in DRIVE):
        raise RuntimeError("行走轮速度超过本程序的低速保护阈值")


def check_input(packet, received, now):
    if packet is None or now - received > LINK_TIMEOUT:
        raise RuntimeError("键控心跳中断")
    ack = packet.get("ack")
    if not isinstance(ack, (int, float)) or not math.isfinite(ack) or not 0 <= now - ack <= LINK_TIMEOUT:
        raise RuntimeError("往返通信超时，拒绝滞留指令")
    if packet.get("stop"):
        raise RuntimeError("已停止：用户停止")
    keys = packet.get("keys")
    if not isinstance(keys, list) or any(k not in KEYS for k in keys):
        raise RuntimeError("无效的键盘指令")
    if 'axes' in packet:
        if keys:
            raise RuntimeError('底盘摇杆与键盘指令不能混发')
        axes = check_axes(packet['axes'])
        return axes if packet.get('focused') else AxisInput(0., 0., 0.)
    # Focus loss removes velocity commands, while the owned session stays enabled.
    return keys if packet.get('focused') else []


def neutral_duration(positions):
    # Quintic interpolation has a peak speed of 1.875 * distance / duration.
    return max(4.0, 1.875 * max(abs(p) for p in positions) / 0.20)


def flatten_parameters(tree, prefix=""):
    result = {}
    for key, value in tree.items():
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            result.update(flatten_parameters(value, name))
        else:
            result[name] = value
    return result
