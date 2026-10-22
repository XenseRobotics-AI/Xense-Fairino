"""Opt-in joystick routing to the existing independently guarded chassis session."""
import math
import sys

from paths import ROOT
sys.path.insert(0, str(ROOT))
from apps.fairino_keyboard.axis_session import AxisSession, ZERO
from apps.fairino_keyboard.control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR
from control_modes import chassis_directions, CHASSIS_DIRECTIONS

DEADBAND = .2


def axis(value):
    if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 1.:
        raise ValueError('底盘摇杆必须是 -1～1 的有限数值')
    return float(value)


def joystick_axes(frame, rotation_priority=False, mode='first_person'):
    signs = chassis_directions(mode)
    if not frame or any(not frame.get(s, {}).get('joystick_available') for s in ('left', 'right')):
        raise ValueError('等待双手柄摇杆数据')
    left, right = frame['left'], frame['right']
    if left.get('axis_click') or right.get('axis_click'):
        raise ValueError('摇杆按下时底盘保持；释放并回中后继续')
    x, y, yaw = axis(left.get('axis_x')), axis(left.get('axis_y')), -axis(right.get('axis_x'))
    radius = math.hypot(x, y)
    if radius <= DEADBAND:
        forward = lateral = 0.
    else:
        scale = (min(radius, 1.)-DEADBAND)/(1.-DEADBAND)/radius
        forward, lateral = y*scale, -x*scale
    yaw = 0. if abs(yaw) <= DEADBAND else math.copysign((abs(yaw)-DEADBAND)/(1.-DEADBAND), yaw)
    if rotation_priority and yaw:
        forward = lateral = 0.
    return tuple(value*sign for value, sign in zip((forward, lateral, yaw), signs))


class ChassisTeleop:
    def __init__(self, args, password, session_factory=AxisSession):
        self.mode = getattr(args, 'mode', 'first_person')
        chassis_directions(self.mode)
        self.session = session_factory(getattr(args, 'chassis_host', '192.168.58.3'), args.user, password,
            linear=getattr(args, 'chassis_linear', DEFAULT_LINEAR),
            angular=getattr(args, 'chassis_angular', DEFAULT_ANGULAR))
        self.rotation_priority = bool(getattr(args, 'chassis_rotation_priority', False))
        self.note = '等待底盘回正并使能；摇杆保持回中'

    def start(self):
        self.session.start()
        return self

    def update(self, frame, active):
        state = self.session.snapshot()
        if state.get('error'):
            raise RuntimeError('底盘控制：'+state['error'])
        if not active or frame is None:
            self.session.set_axes(active=False)
            self.note = '底盘保持；摇杆回中后继续'
            return
        try:
            values = joystick_axes(frame, self.rotation_priority, self.mode)
        except ValueError as exc:
            self.session.set_axes(active=False)
            self.note = str(exc)
            return
        self.session.set_axes(values, frame.get('received_monotonic'), active=True)
        self.note = (CHASSIS_DIRECTIONS[self.mode] if state.get('armed')
                     else '等待底盘就绪及左杆、右杆横轴回中')

    def snapshot(self):
        state = self.session.snapshot()
        hardware = state.get('hardware') or {}
        return dict(mode=self.mode, ready=bool(state.get('ready')), phase=state.get('phase', ''),
                    enabled=sum(hardware.get('enabled', [])), command=state.get('command', ZERO),
                    axes=state.get('axes', ZERO), armed=bool(state.get('armed')), message=self.note,
                    received=state.get('received'), error=state.get('error'),
                    disabled_confirmed=state.get('disabled_confirmed', False),
                    cleanup_confirmed=state.get('cleanup_confirmed', False))

    def require_stationary(self, now):
        """Read cached feedback only; calibration must never guess at a stop."""
        state = self.session.snapshot()
        if state.get('error'):
            raise ValueError('底盘反馈异常，不能校准朝向：'+str(state['error']))
        if not state.get('ready'):
            raise ValueError('请等待底盘就绪并停稳后校准朝向')
        received = state.get('received')
        if (any(type(v) not in (int, float) or not math.isfinite(v) for v in (now, received))
                or not 0 <= now-received <= .25):
            raise ValueError('底盘反馈缺失、超过 0.25 秒或时间异常，不能确认停稳')
        hardware = state.get('hardware')
        velocity = hardware.get('velocity') if isinstance(hardware, dict) else None
        if (not isinstance(velocity, (list, tuple)) or len(velocity) != 8
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in velocity)):
            raise ValueError('底盘八轴速度反馈无效，不能确认停稳')
        if any(abs(v) >= .05 for v in velocity):
            raise ValueError('底盘尚未停稳，请等待全部轮组停止后校准朝向')
        command = state.get('command')
        if (not isinstance(command, (list, tuple)) or len(command) != 3
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in command)):
            raise ValueError('底盘速度指令反馈无效，不能确认停稳')
        if any(abs(v) > .001 for v in command):
            raise ValueError('底盘仍有速度指令，请将摇杆回中并等待停稳后校准朝向')

    def close(self):
        self.session.close()

    def finish(self, timeout=25.):
        return self.session.finish(timeout)
