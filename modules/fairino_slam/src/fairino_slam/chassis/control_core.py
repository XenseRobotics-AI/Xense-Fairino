"""Shared mapping-drive and navigation limits, feedback guards and input rules.

Legacy ROS names remain compatible with the installed vendor controller.
"""
import math
import copy

JOINTS = [f"j{i}" for i in range(1, 9)]
STEER = [0, 2, 4, 6]
DRIVE = [1, 3, 5, 7]
KEYS = frozenset("wasdjl")
# Remote payloads inject the validated snapshot; local imports read config/chassis.yaml.
if '_CHASSIS_CONFIG' not in globals():
    from fairino_slam.configuration import load_chassis
    _CHASSIS_CONFIG = load_chassis()
_SPEED = _CHASSIS_CONFIG['speed']
_LIMITS = _CHASSIS_CONFIG['limits']
FEEDBACK_TIMEOUT = _CHASSIS_CONFIG['timeouts']['feedback']
LINK_TIMEOUT = _CHASSIS_CONFIG['timeouts']['link']
DEFAULT_LINEAR = _SPEED['default_linear']
DEFAULT_ANGULAR = _SPEED['default_angular']
MIN_SPEED = _SPEED['min_speed']
MAX_LINEAR = _SPEED['max_linear']
MAX_ANGULAR = _SPEED['max_angular']
DRIVE_COMMAND_LIMIT = _LIMITS['drive']['max_velocity']
DRIVE_FEEDBACK_LIMIT = _LIMITS['drive']['feedback_max_velocity']


def configuration_snapshot():
    """Copy the configuration captured when this process imported the core."""
    return copy.deepcopy(_CHASSIS_CONFIG)


def chassis_parameters(params):
    """One profile for installation and runtime verification."""
    params = copy.deepcopy(params)
    params['cmd_vel_topic'] = '/fairino_keyboard/cmd_vel'
    params['odom_topic'] = '/eight_dof_chassis_controller/odom'
    params['limits']['base']['linear'].update(max_velocity=MAX_LINEAR, **_LIMITS['linear'])
    params['limits']['base']['angular'].update(max_velocity=MAX_ANGULAR, **_LIMITS['angular'])
    params['limits']['steering'].update(**_LIMITS['steering'])
    params['limits']['drive'].update({key: value for key, value in _LIMITS['drive'].items()
                                     if key != 'feedback_max_velocity'})
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
        raise RuntimeError(f"底盘反馈缺失或超过 {FEEDBACK_TIMEOUT:g} 秒未更新")
    if upper is None or upper_age > FEEDBACK_TIMEOUT:
        raise RuntimeError(f"上身急停反馈缺失或超过 {FEEDBACK_TIMEOUT:g} 秒未更新")
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
        age = f"{(now-ack)*1000:.0f} ms" if isinstance(ack, (int, float)) and math.isfinite(ack) else "无有效 ACK"
        raise RuntimeError(f"往返通信超时，拒绝滞留指令（ACK {age}；阈值 {LINK_TIMEOUT*1000:.0f} ms）")
    if packet.get("stop"):
        raise RuntimeError("已停止：用户停止")
    keys = packet.get("keys")
    if not isinstance(keys, list) or any(k not in KEYS for k in keys):
        raise RuntimeError("无效的键盘指令")
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


def limit_planar_twist(values, linear=DEFAULT_LINEAR, angular=DEFAULT_ANGULAR):
    """Bound the XY norm, not each axis separately; accept only planar commands."""
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('无效的导航速度')
    x, y, z, roll, pitch, yaw = values
    if any(abs(v) > 1e-9 for v in (z, roll, pitch)):
        raise ValueError('导航只接受二维速度')
    scale = min(1., linear/max(math.hypot(x, y), 1e-12))
    return x*scale, y*scale, max(-angular, min(angular, yaw))
