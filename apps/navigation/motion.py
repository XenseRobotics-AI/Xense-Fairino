"""Pure velocity gating; no ROS dependencies."""
try:
    from apps.fairino_keyboard.control_core import limit_planar_twist as bounded_twist
except ModuleNotFoundError:
    from control_core import limit_planar_twist as bounded_twist


class CommandGate:
    """A failed goal cannot resume merely because localization recovers."""
    def __init__(self, linear, angular):
        self.linear, self.angular = linear, angular
        self.goal = None
        self.session = None
        self.failed_goal = None
        self.failure = None
        self.command = None
        self.received = 0.
        self.goal_since = 0.
        self.reason = '等待导航目标'

    def set_goal(self, goal, now):
        if goal != self.goal:
            self.goal = goal
            self.session = None
            self.command = None
            self.goal_since = now
            if goal is not None and goal != self.failed_goal:
                self.failure = None

    def receive(self, values, now):
        self.command = bounded_twist(values, self.linear, self.angular)
        self.received = now

    def fail(self, reason, now):
        # Preserve the first cause after action cancellation and later recovery.
        # A malformed command with no goal must not poison a future goal.
        if self.goal is not None and self.failed_goal != self.goal:
            self.failed_goal = self.goal
            self.failure = {
                'goal': self.goal, 'reason': reason, 'monotonic_time': now,
                'command_age': max(0., now-self.received) if self.command is not None else None}
            self.reason = reason

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
        if self.command is None or self.received < self.goal_since or now-self.received > .3:
            self.reason = '等待新速度指令'
            # Startup permits a short planning interval. Once movement commands
            # existed, a broken stream latches until a new explicit goal.
            if self.command is not None and now-self.received > .3:
                self.fail('导航速度指令断流（%.3f 秒 > 0.300 秒）；本次目标已停止'
                          % (now-self.received), now)
            return zero
        self.reason = '导航速度输出'
        return self.command
