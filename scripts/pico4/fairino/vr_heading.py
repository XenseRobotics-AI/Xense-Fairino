"""Optional head-pose decoding and explicit yaw calibration; no device access."""
import json
import math

import numpy as np


TIMEOUT = .25
MIN_HORIZONTAL_FORWARD = .1  # Reject gaze within about 5.7 degrees of vertical.
# XRoboToolkit-Unity-Client v1.1.1, Assets/Scripts/TrackingData.cs:390 sends
# sensorState.status unchanged. Its bundled PXR_System.cs:59-69 documents
# sensor status 0=null, 1=3DoF, 3=6DoF. We only need orientation, so both known
# tracked states are usable. Do not interpret arbitrary nonzero values as valid.
HEAD_STATUSES = (1, 3)


def _number(value, message):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(message)
    return value


def _stamp(value):
    value = _number(value, '头显时间戳无效')
    if value <= 0:
        raise ValueError('头显时间戳无效')
    return value


def _pose(value):
    if isinstance(value, str):
        try:
            value = [float(part) for part in value.split(',')]
        except ValueError:
            raise ValueError('头显姿态格式无效') from None
    if not isinstance(value, list) or len(value) != 7:
        raise ValueError('头显姿态需要位置及 xyzw 四元数共 7 个数值')
    pose = [float(_number(part, '头显姿态包含无效数值')) for part in value]
    if not .9 < math.hypot(*pose[3:]) < 1.1:
        raise ValueError('头显姿态四元数无效')
    return pose


def decode_head(head, frame_stamp):
    """Decode this packet's optional Head without breaking controller teleop.

    Returns {pose: [x,y,z,qx,qy,qz,qw], stamp: nanoseconds, status: int}, or
    {error: Chinese explanation}. v1.1.1 omits Head.timeStampNs: in that case
    its pose is bound to the enclosing packet, never a separately cached pose.
    """
    try:
        frame_stamp = _stamp(frame_stamp)
        if head is None:
            raise ValueError('未收到头显 Head 数据，请在 PICO 中开启 Head Tracking 和 Send')
        if isinstance(head, str):
            try:
                head = json.loads(head)
            except (ValueError, TypeError):
                raise ValueError('头显 Head 数据格式无效') from None
        if not isinstance(head, dict):
            raise ValueError('头显 Head 数据格式无效')
        status = head.get('status')
        if type(status) is not int or status not in HEAD_STATUSES:
            raise ValueError('头显追踪状态无效或未确认，需要 status=1（3DoF）或 3（6DoF）')
        pose = _pose(head.get('pose'))
        stamp = _stamp(head.get('timeStampNs', frame_stamp))
        if abs(stamp-frame_stamp) > TIMEOUT*1e9:
            raise ValueError('头显姿态与当前手柄数据相差超过 0.25 秒，请等待新数据')
        return {'pose': pose, 'stamp': stamp, 'status': status}
    except (ValueError, OverflowError) as exc:
        return {'error': str(exc)}


def heading_alignment(frame, now, *, require_chassis_neutral=False):
    """Return (tracking-to-heading inverse yaw, yaw degrees) after input gates.

    The caller owns robot readiness/stationarity and resets old motion targets.
    This function only validates one packet; it does not change device state.
    Left-stick click is deliberately not a gate: it triggers this operation.
    With chassis control, also require neutral left XY (radial) and right X/Y;
    the caller must separately verify fresh, stationary chassis feedback.
    """
    if not isinstance(frame, dict):
        raise ValueError('尚未收到可用于朝向校准的 PICO 数据')
    now = _number(now, '朝向校准本机时间无效')
    received = _number(frame.get('received_monotonic'), '缺少 PICO 数据接收时间，不能校准朝向')
    if not 0 <= now-received <= TIMEOUT:
        raise ValueError('PICO 数据超过 0.25 秒未更新或接收时间异常，不能校准朝向')
    for side, label in (('left', '左'), ('right', '右')):
        hand = frame.get(side)
        if not isinstance(hand, dict):
            raise ValueError(f'缺少{label}手柄数据，不能校准朝向')
        grip = _number(hand.get('grip'), f'{label}手柄 Grip 数据无效')
        if not 0 <= grip <= 1:
            raise ValueError(f'{label}手柄 Grip 数据无效')
        if grip > .3:
            raise ValueError('朝向校准前请松开左右 Grip')
    right = frame['right']
    if right.get('joystick_available') is not True:
        raise ValueError('右摇杆数据不可用，不能确认身体升降已停止输入')
    axis = _number(right.get('axis_y'), '右摇杆数据无效')
    if abs(axis) > .2 or right.get('axis_click') is not False:
        raise ValueError('朝向校准前请将右摇杆回中并松开按压')
    if require_chassis_neutral:
        for side, label in (('left', '左'), ('right', '右')):
            hand = frame[side]
            if hand.get('joystick_available') is not True:
                raise ValueError(f'{label}摇杆数据不可用，不能确认底盘已停止输入')
            axes = []
            for key in ('axis_x', 'axis_y'):
                value = _number(hand.get(key), f'{label}摇杆数据无效')
                axes.append(value)
                if abs(value) > .2:
                    raise ValueError('朝向校准前请将左右摇杆回中，停止底盘和身体升降输入')
            if side == 'left' and math.hypot(*axes) > .2:
                raise ValueError('朝向校准前请将左摇杆回中，停止底盘平移输入')
    head = frame.get('head')
    if not isinstance(head, dict):
        raise ValueError('未收到头显 Head 数据，请在 PICO 中开启 Head Tracking 和 Send')
    if head.get('error'):
        raise ValueError(str(head['error']))
    # Revalidate normalized data as well: standalone callers must not bypass
    # status, pose or timestamp validation by constructing frame['head'].
    checked = decode_head({'pose': head.get('pose'), 'status': head.get('status'),
                           'timeStampNs': head.get('stamp')}, frame.get('stamp'))
    if 'error' in checked:
        raise ValueError(checked['error'])
    q = checked['pose'][3:]
    x, y, z, w = [part/math.hypot(*q) for part in q]
    # Forward = R(q) @ (0, 0, -1), in PICO tracking coordinates. Discard
    # vertical projection and all head translation; never use Euler roll/pitch.
    forward_x = -2*(x*z+y*w)
    forward_z = -(1-2*(x*x+y*y))
    if math.hypot(forward_x, forward_z) < MIN_HORIZONTAL_FORWARD:
        raise ValueError('头显视线接近竖直，无法确定前方；请平视后校准')
    yaw = math.atan2(-forward_x, -forward_z)
    c, s = math.cos(yaw), math.sin(yaw)
    alignment = np.array([[c, 0., -s], [0., 1., 0.], [s, 0., c]])
    return alignment, math.degrees(yaw)
