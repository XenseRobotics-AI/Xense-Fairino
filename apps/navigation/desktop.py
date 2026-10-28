"""Desktop navigation state and guarded chassis session, independent of GTK."""
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
for folder in (ROOT/'tools', ROOT/'apps/fairino_keyboard'):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))
from navigation_mode import execute
from control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR


def context(frame):
    loc = frame.get('localization') or {}
    if frame.get('map_mode') != 'saved' or not loc.get('active'):
        return None
    source, session = frame.get('saved_map_source'), loc.get('id')
    return (source, session) if source and session else None


def active_goal(state):
    return bool(state.get('goal') or (state.get('last_goal') or {}).get('status') in (1, 2, 3))


class Target:
    """A map/session-bound draft; entering coordinates never sends a command."""
    def __init__(self):
        self.pose = self.context = None
        self.points = []
        self.goal_id = None
        self.label = ''

    def clear(self):
        self.__init__()

    def set(self, values, frame):
        x, y, degrees = map(float, values)
        if not all(math.isfinite(v) for v in (x, y, degrees)):
            raise ValueError('请输入有限的 X、Y 和朝向数值')
        binding = context(frame)
        if binding is None:
            raise ValueError('请先选用地图并启动定位')
        self.pose = dict(x=x, y=y, yaw=math.radians(degrees))
        self.context, self.points, self.goal_id = binding, [], None
        self.label = '待发送'

    def reconcile(self, frame):
        if self.pose is not None and self.context != context(frame):
            self.clear()
            return True
        return False

    def request(self, frame):
        if self.reconcile(frame) or self.pose is None:
            raise ValueError('地图或定位会话已变化，请重新设置目标')
        return {**self.pose, 'session_id': self.context[1]}


class DriveSession:
    """Heartbeat follows the GUI; a stalled GUI latches stop and disables."""
    def __init__(self, host, user, password, command=execute, factory=None):
        self.args = SimpleNamespace(host=host, user=user, check=False, prepare_check=False,
                                    navigation=True, linear=DEFAULT_LINEAR, angular=DEFAULT_ANGULAR,
                                    log_directory=str(ROOT/'logs/navigation/drive'))
        self.password, self.command, self.factory = password, command, factory
        self.connection = None
        self.stop_event = threading.Event()
        self.done = threading.Event()
        self.pulse_time = time.monotonic()
        self.message = '正在检查并准备定位 / 导航底盘…'
        self.disabled_confirmed = False
        self.thread = threading.Thread(target=self.run, daemon=True)

    def pulse(self):
        self.pulse_time = time.monotonic()

    def stop(self):
        self.stop_event.set()

    @property
    def ready(self):
        c = self.connection
        return bool(c and not self.stop_event.is_set() and not c.done.is_set()
                    and c.latest.get('ready') and time.monotonic()-c.last_status < .5)

    def run(self):
        try:
            state = self.command('status', self.args.host, self.args.user, self.password)
            if active_goal(state) and not state.get('stale'):
                raise RuntimeError('请先取消已有导航目标，再接入底盘')
            if self.stop_event.is_set() or time.monotonic()-self.pulse_time > .75:
                return
            self.message = '正在准备导航节点，完成后才接入底盘…'
            self.command('prepare', self.args.host, self.args.user, self.password)
            if self.stop_event.is_set() or time.monotonic()-self.pulse_time > .75:
                return
            if self.factory is None:
                from keyboard_control import Connection
                self.factory = Connection
            self.connection = self.factory(self.args, password=self.password)
            c = self.connection
            while not c.done.is_set() and not self.stop_event.is_set():
                if time.monotonic()-self.pulse_time > .75:
                    self.message = '界面心跳中断，正在停止并下使能'
                    break
                c.send(keys=(), focused=True)
                # Motor readiness does not imply AMCL or Nav2 readiness.
                self.message = ('底盘控制连接已就绪' if c.latest.get('ready')
                                else c.latest.get('phase', '等待底盘反馈'))
                self.stop_event.wait(.05)
        except Exception as exc:
            self.message = '导航底盘未完成：'+str(exc)
        finally:
            c = self.connection
            if c is not None:
                c.finish()
                if not c.done.is_set():
                    self.message = '仍在等待底盘退出及下使能反馈'
                    while not c.done.wait(.2):
                        c.send(stop=True)
                    c.finish()
                self.disabled_confirmed = bool(c.done.is_set() and c.latest.get('disabled_confirmed'))
                self.message = ('导航底盘已停止并确认下使能' if self.disabled_confirmed
                                else '未确认底盘下使能，请检查底盘反馈')
                failure = getattr(c, 'last_error', '')
                if isinstance(failure, str) and failure:
                    self.message = failure+'；'+self.message
            self.password = None
            self.done.set()
