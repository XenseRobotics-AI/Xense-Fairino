"""Desktop navigation state and guarded chassis session, independent of GTK."""
import math
import threading
import time
from types import SimpleNamespace

from fairino_slam.paths import LOG_ROOT
from .client import execute, HEADING_MODES
from fairino_slam.chassis.control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR


def context(frame):
    loc = frame.get('localization') or {}
    if frame.get('map_mode') != 'saved' or not loc.get('active'):
        return None
    source, session = frame.get('saved_map_source'), loc.get('id')
    return (source, session) if source and session else None


def active_goal(state):
    return bool(state.get('goal') or (state.get('last_goal') or {}).get('status') in (1, 2, 3))


def navigation_status(state, goal_id=None, connecting=False):
    """Prefer the actual stop/wait cause over the stack's generic ready flag."""
    failure = state.get('failure') or {}
    if failure and failure.get('goal') in (goal_id, state.get('goal')) and failure.get('goal'):
        return failure.get('reason') or '保护停止'
    execution = state.get('execution') or {}
    if execution.get('goal') and execution.get('goal') in (goal_id, state.get('goal')):
        phase = execution.get('phase')
        if phase in ('planning', 'waiting', 'failed', 'canceled', 'completed'):
            return execution.get('reason') or '导航已停止'
    if active_goal(state):
        reason = state.get('reason')
        if reason and reason != '导航速度输出':
            return reason
        return '导航中'
    chassis = state.get('chassis_motion') or {}
    if connecting and chassis.get('source_state') == 'missing':
        return '正在接入底盘，等待协调反馈连接'
    last = state.get('last_goal') or {}
    if last.get('status') == 6:
        return '导航失败，目标已停止；请检查规划器和控制器状态'
    if last.get('status') == 5:
        return '目标已取消；不会自动恢复'
    if last.get('status') == 4:
        return '目标已到达'
    if state.get('localization_ready') and state.get('controller_ready'):
        return '就绪'
    return state.get('reason') or state.get('localization_reason') or '等待启动 / 就绪'


class Target:
    """A map/session-bound draft; entering coordinates never sends a command."""
    def __init__(self, heading_mode='hold_initial'):
        if heading_mode not in HEADING_MODES:
            raise ValueError('未知导航朝向策略：'+str(heading_mode))
        self.heading_mode = heading_mode
        self.pose = self.context = None
        self.points = []
        self.goal_id = None
        self.label = ''
        self.saved_point = None

    def clear(self):
        self.__init__(self.heading_mode)

    def set(self, values, frame):
        x, y = map(float, values[:2])
        if not all(math.isfinite(v) for v in (x, y)):
            raise ValueError('请输入有限的 X、Y 数值')
        binding = context(frame)
        if binding is None:
            raise ValueError('请先选用地图并启动定位')
        try:
            degrees = float(values[2])
        except (IndexError, TypeError, ValueError) as exc:
            raise ValueError('请输入目标朝向，单位度') from exc
        if not math.isfinite(degrees):
            raise ValueError('请输入有限的目标朝向，单位度')
        yaw = math.radians(degrees)
        self.saved_point = None
        self.pose = dict(x=x, y=y, yaw=yaw)
        self.context, self.points, self.goal_id = binding, [], None
        self.label = '待发送'

    def set_saved(self, point, frame):
        from .waypoints import map_binding, finite
        m, pose = point['map'], point['pose']
        if m != map_binding(frame, m['host'], m['user']):
            raise ValueError('位置点不属于当前地图或地图版本')
        if context(frame) is None:
            raise ValueError('请先启动当前地图的定位')
        if set(pose) != {'x', 'y', 'yaw'} or not all(map(finite, pose.values())):
            raise ValueError('保存点的位置或朝向无效')
        self.clear()
        self.pose = dict(pose)
        self.context = context(frame)
        self.saved_point = point
        self.label = '待发送 · '+point['name']

    def reconcile(self, frame):
        same_map = True
        if self.saved_point is not None:
            from .waypoints import map_binding
            m = self.saved_point['map']
            try:
                same_map = m == map_binding(frame, m['host'], m['user'])
            except ValueError:
                same_map = False
        if self.pose is not None and (self.context != context(frame) or not same_map):
            self.clear()
            return True
        return False

    def request(self, frame):
        if self.reconcile(frame) or self.pose is None:
            raise ValueError('地图或定位会话已变化，请重新设置目标')
        request = {'x': self.pose['x'], 'y': self.pose['y'], 'session_id': self.context[1],
                   'heading_mode': self.heading_mode}
        request['yaw'] = self.pose['yaw']
        if self.heading_mode == 'hold_initial':
            request['arrival_heading'] = 1
        if self.saved_point is not None:
            request.update(saved_pose=1, saved_map_source=self.saved_point['map']['source'],
                           saved_map_modified=self.saved_point['map']['modified'])
        return request

    def accept_heading(self, result):
        # In path mode, locked_yaw may describe the initial robot pose. Only an
        # explicit final goal_yaw can replace the user's requested orientation.
        keys = ('goal_yaw',)
        for key in keys:
            yaw = result.get(key)
            if (self.pose is not None and not isinstance(yaw, bool)
                    and isinstance(yaw, (int, float)) and math.isfinite(yaw)):
                self.pose['yaw'] = float(yaw)
                break


class DriveSession:
    """Heartbeat follows the GUI; a stalled GUI latches stop and disables."""
    def __init__(self, host, user, password, command=execute, factory=None):
        self.args = SimpleNamespace(host=host, user=user, check=False, prepare_check=False,
                                    navigation=True, linear=DEFAULT_LINEAR, angular=DEFAULT_ANGULAR,
                                    log_directory=str(LOG_ROOT/'navigation/drive'))
        self.password, self.command, self.factory = password, command, factory
        self.connection = None
        self.stop_event = threading.Event()
        self.done = threading.Event()
        self.pulse_time = time.monotonic()
        self.message = '正在检查并接入导航底盘…'
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
                from fairino_slam.chassis.transport import Connection
                self.factory = Connection
            self.connection = self.factory(self.args, password=self.password)
            c = self.connection
            while not c.done.is_set() and not self.stop_event.is_set():
                if time.monotonic()-self.pulse_time > .75:
                    self.message = '界面心跳中断，正在停止并下使能'
                    break
                c.send(keys=(), focused=True)
                self.message = c.latest.get('phase', '等待底盘反馈')
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
