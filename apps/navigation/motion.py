"""Pure velocity gating; no ROS dependencies."""
import copy
try:
    from apps.fairino_keyboard.control_core import limit_planar_twist as bounded_twist
except ModuleNotFoundError:
    from control_core import limit_planar_twist as bounded_twist


class CommandGate:
    """A failed goal cannot resume merely because localization recovers."""
    COMMAND_TIMEOUT = .3

    def __init__(self, linear, angular):
        self.linear, self.angular = linear, angular
        self.goal = None
        self.session = None
        self.failed_goal = None
        self.failure = None
        self.command = None
        self.received = 0.
        self.goal_since = 0.
        self.command_count = 0
        self.command_interval = None
        self.max_command_interval = 0.
        self.reason = '等待导航目标'

    def set_goal(self, goal, now):
        if goal != self.goal:
            self.goal = goal
            self.session = None
            self.command = None
            self.goal_since = now
            if goal is not None:
                self.command_count = 0
                self.command_interval = None
                self.max_command_interval = 0.
            if goal is not None and goal != self.failed_goal:
                self.failure = None

    def receive(self, values, now):
        command = bounded_twist(values, self.linear, self.angular)
        if self.goal is not None:
            if self.command is not None and self.received >= self.goal_since:
                self.command_interval = max(0., now-self.received)
                self.max_command_interval = max(self.max_command_interval, self.command_interval)
                # A late receive callback may run before the timer callback.
                # Do not erase the expired interval by replacing received first.
                if self.command_interval > self.COMMAND_TIMEOUT:
                    self.fail('导航速度指令断流（%.3f 秒 > 0.300 秒）；本次目标已停止'
                              % self.command_interval, now, code='command_timeout')
            self.command_count += 1
        self.command = command
        self.received = now

    def telemetry(self, now):
        return {'command_age': max(0., now-self.received) if self.command is not None else None,
                'command_interval': self.command_interval,
                'max_command_interval': self.max_command_interval,
                'command_count': self.command_count,
                'command_timeout': self.COMMAND_TIMEOUT}

    def capture_failure_context(self, context):
        """Freeze the first available diagnostics, independently of live status."""
        if self.failure is not None and 'context' not in self.failure:
            self.failure['context'] = copy.deepcopy(context)

    def fail(self, reason, now, code='unknown'):
        # Preserve the first cause after action cancellation and later recovery.
        # A malformed command with no goal must not poison a future goal.
        if self.goal is not None and self.failed_goal != self.goal:
            self.failed_goal = self.goal
            self.failure = {
                'goal': self.goal, 'reason': reason, 'code': code, 'monotonic_time': now,
                **self.telemetry(now)}
            self.reason = reason

    def output(self, now, ready, session, source_ok=True, readiness_reason=None):
        zero = (0., 0., 0.)
        if self.goal is None:
            self.reason = self.failure['reason'] if self.failure else '等待导航目标'
            return zero
        if self.failed_goal == self.goal:
            return zero
        if self.session is not None and session != self.session:
            self.fail('定位或里程计会话变化；本次目标已停止', now, code='session_changed')
            return zero
        if not source_ok:
            self.fail('导航速度发布器缺失或来源不唯一；本次目标已停止', now, code='command_source')
            return zero
        if not ready:
            self.fail((readiness_reason or '定位或导航节点未就绪')+'；本次目标已停止', now, code='not_ready')
            return zero
        self.session = session
        if self.command is None or self.received < self.goal_since or now-self.received > self.COMMAND_TIMEOUT:
            self.reason = '等待新速度指令'
            # Startup permits a short planning interval. Once movement commands
            # existed, a broken stream latches until a new explicit goal.
            if self.command is not None and now-self.received > self.COMMAND_TIMEOUT:
                self.fail('导航速度指令断流（%.3f 秒 > 0.300 秒）；本次目标已停止'
                          % (now-self.received), now, code='command_timeout')
            return zero
        self.reason = '导航速度输出'
        return self.command
