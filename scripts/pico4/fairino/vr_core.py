"""Robot-side, standard-library-only validation of bimanual IK requests."""
import math
from tcp_core import MODEL_ID, ARMS, check_arm_limits, finite_vector, tcp_forward_matrix, pose_error, multiply
from tool_frames import tool_config, validate_runtime, for_arm
from tcp_jog import LEASE
from vr_lift import lift_axis

KIND = "fairino_pico_cartesian_v4"


def direct_positions(current, targets, held):
    """One t=0 target: JTC holds this sample, with no added joint ramp."""
    end = finite_vector(current, 20, "当前二十轴指令")
    end[14:] = finite_vector(held, 20, "二十轴保持位置")[14:]
    for side, joints in ARMS.items():
        goal = check_arm_limits(targets.get(side, [end[j-1] for j in joints]))
        for j, value in zip(joints, goal):
            end[j-1] = value
    return [(0., end)]


def matrix(value):
    values = finite_vector(value, 16, "TCP 矩阵")
    t = [values[i:i+4] for i in range(0, 16, 4)]
    if max(abs(a-b) for a, b in zip(t[3], [0., 0., 0., 1.])) > 1e-9:
        raise ValueError("无效齐次矩阵")
    r = [row[:3] for row in t[:3]]
    if any(abs(sum(r[i][k]*r[j][k] for k in range(3))-(i == j)) > 1e-6
           for i in range(3) for j in range(3)):
        raise ValueError("TCP 旋转矩阵不正交")
    det = sum(r[0][i]*(r[1][(i+1)%3]*r[2][(i+2)%3]-r[1][(i+2)%3]*r[2][(i+1)%3]) for i in range(3))
    if abs(det-1.) > 1e-6:
        raise ValueError("TCP 旋转矩阵不是正常旋转")
    return t


def validate(hw, request, now, tools=None):
    if not isinstance(request, dict) or request.get("kind") != KIND or request.get("model") != MODEL_ID:
        raise ValueError("双臂协议或运动学模型不匹配")
    tools = validate_runtime(tools if tools is not None else tool_config())
    if request.get('tool_id', 'flange') != tools['id']:
        raise ValueError('客户端与机器人端 TCP 工具配置不一致')
    if type(request.get("id")) is not int or request["id"] <= 0:
        raise ValueError("无效遥操请求序号")
    created = request.get("created")
    if type(created) not in (int, float) or not math.isfinite(created) or not 0 <= now-created <= LEASE:
        raise ValueError("PICO 目标超过 0.25 秒未更新")
    arms = request.get("arms")
    lift = lift_axis(request)
    if not isinstance(arms, dict) or (not arms and abs(lift) <= .2) or not set(arms) <= set(ARMS):
        raise ValueError("无效的双臂目标")
    targets = {}
    for side, entry in arms.items():
        if not isinstance(entry, dict):
            raise ValueError("无效手臂目标")
        check_arm_limits(entry.get("reference"))
        target = check_arm_limits(entry.get("target"))
        tcp = matrix(entry.get("tcp"))
        mm, deg = pose_error(tcp, multiply(tcp_forward_matrix(target), for_arm(tools, side)))
        if mm > .02 or deg > .01:
            raise ValueError("双臂目标未通过机器人端独立 TCP 正解校验")
        targets[side] = target
    return targets


