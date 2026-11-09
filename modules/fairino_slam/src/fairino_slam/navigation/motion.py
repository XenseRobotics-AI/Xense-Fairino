"""Pure velocity gating; no ROS dependencies."""
import threading
from functools import wraps


def synchronized(method):
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        with self.lock:
            return method(self, *args, **kwargs)
    return wrapped


try:
    from fairino_slam.chassis.control_core import limit_planar_twist as bounded_twist
    from fairino_slam.navigation.runtime_policy import COMMAND_TIMEOUT
except ModuleNotFoundError:
    from control_core import limit_planar_twist as bounded_twist
    from runtime_policy import COMMAND_TIMEOUT


class CommandGate:
    """A failed goal cannot resume merely because localization recovers."""
    def __init__(self, linear, angular):
        self.lock = threading.RLock()
        self.linear, self.angular = linear, angular
        self.goal = None
        self.session = None
        self.failed_goal = None
        self.failure = None
        self.command = None
        self.received = 0.
        self.goal_since = 0.
        self.reason = '等待导航目标'
        self.execution = None
        self.resume_since = None

    @synchronized
    def set_goal(self, goal, now):
        if goal != self.goal:
            self.goal = goal
            self.session = None
            self.command = None
            self.goal_since = now
            self.execution = None
            self.resume_since = None
            if goal is not None and goal != self.failed_goal:
                self.failure = None

    @synchronized
    def receive(self, values, now):
        self.command = bounded_twist(values, self.linear, self.angular)
        self.received = now

    @synchronized
    def set_execution(self, phase, reason, now):
        """Only the validated BT stream may temporarily suspend command timeout."""
        if self.goal is None or self.failed_goal == self.goal:
            return
        if phase not in ('planning', 'waiting', 'following', 'failed', 'canceled', 'completed'):
            self.fail('导航执行状态无效；本次目标已停止', now)
            return
        previous = (self.execution or {}).get('phase')
        if previous in ('canceled', 'completed'):
            # A terminal state cannot be reopened while goal cancellation/status
            # acknowledgement is still in flight.
            return
        if phase != 'following' or previous != 'following':
            # Discard pre-pause commands and commands emitted during cancellation.
            self.command = None
        if phase == 'following' and previous != 'following':
            self.resume_since = now
        self.execution = {'goal': self.goal, 'phase': phase, 'reason': reason,
                          'received': now}
        if phase == 'failed':
            self.fail(reason or '路径跟随失败；本次目标已停止', now)

    @synchronized
    def fail(self, reason, now):
        # Preserve the first cause after action cancellation and later recovery.
        # A malformed command with no goal must not poison a future goal.
        if self.goal is not None and self.failed_goal != self.goal:
            self.failed_goal = self.goal
            self.failure = {
                'goal': self.goal, 'reason': reason, 'monotonic_time': now,
                'command_age': max(0., now-self.received) if self.command is not None else None}
            self.reason = reason

    @synchronized
    def output(self, now, ready, session, source_ok=True, readiness_reason=None):
        zero = (0., 0., 0.)
        if self.goal is None:
            self.reason = self.failure['reason'] if self.failure else '等待导航目标'
            return zero
        if self.failed_goal == self.goal:
            return zero
        if self.session is not None and session != self.session:
            self.fail('定位或里程计会话变化；本次目标已停止', now)
            return zero
        if not source_ok:
            self.fail('导航速度发布器缺失或来源不唯一；本次目标已停止', now)
            return zero
        if not ready:
            self.fail((readiness_reason or '定位或导航节点未就绪')+'；本次目标已停止', now)
            return zero
        self.session = session
        if self.execution is not None:
            if now-self.execution['received'] > COMMAND_TIMEOUT:
                self.fail('导航执行状态断流；本次目标已停止', now)
                return zero
            if self.execution['phase'] != 'following':
                self.reason = self.execution['reason']
                self.command = None
                return zero
            if self.command is None and self.resume_since is not None:
                if now-self.resume_since > COMMAND_TIMEOUT:
                    self.fail('恢复跟随后未收到新速度指令；本次目标已停止', now)
                else:
                    self.reason = '新路径已就绪，等待控制器输出'
                return zero
        if self.command is None or self.received < self.goal_since or now-self.received > COMMAND_TIMEOUT:
            self.reason = '等待新速度指令'
            # Startup permits a short planning interval. Once movement commands
            # existed, a broken stream latches until a new explicit goal.
            if self.command is not None and now-self.received > COMMAND_TIMEOUT:
                self.fail('导航速度指令断流（%.3f 秒 > %.3f 秒）；本次目标已停止'
                          % (now-self.received, COMMAND_TIMEOUT), now)
            return zero
        self.reason = '导航速度输出'
        return self.command
