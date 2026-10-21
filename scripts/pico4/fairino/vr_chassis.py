"""Opt-in joystick routing to the existing independently guarded chassis session."""
import math
import sys

from paths import ROOT
sys.path.insert(0, str(ROOT))
from apps.fairino_keyboard.axis_session import AxisSession, ZERO
from apps.fairino_keyboard.control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR

DEADBAND = .2


def axis(value):
    if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 1.:
        raise ValueError('底盘摇杆必须是 -1～1 的有限数值')
    return float(value)


def joystick_axes(frame, rotation_priority=False):
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
    return forward, lateral, yaw


class ChassisTeleop:
    def __init__(self, args, password, session_factory=AxisSession):
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
            values = joystick_axes(frame, self.rotation_priority)
        except ValueError as exc:
            self.session.set_axes(active=False)
            self.note = str(exc)
            return
        self.session.set_axes(values, frame.get('received_monotonic'), active=True)
        self.note = ('左杆全向平移；右杆横向旋转' if state.get('armed')
                     else '等待底盘就绪及左杆、右杆横轴回中')

    def snapshot(self):
        state = self.session.snapshot()
        hardware = state.get('hardware') or {}
        return dict(ready=bool(state.get('ready')), phase=state.get('phase', ''),
                    enabled=sum(hardware.get('enabled', [])), command=state.get('command', ZERO),
                    axes=state.get('axes', ZERO), armed=bool(state.get('armed')), message=self.note,
                    received=state.get('received'), error=state.get('error'),
                    disabled_confirmed=state.get('disabled_confirmed', False),
                    cleanup_confirmed=state.get('cleanup_confirmed', False))

    def close(self):
        self.session.close()

    def finish(self, timeout=25.):
        return self.session.finish(timeout)
