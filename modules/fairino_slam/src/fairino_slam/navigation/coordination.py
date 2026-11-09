"""Goal-scoped checks for observed chassis motion feedback; no ROS imports."""
import math
import threading



def publisher_state(publishers):
    """Keep discovery evidence; zero endpoints during connection is not duplication."""
    endpoints = sorted([
        dict(name=p.node_name, namespace=p.node_namespace,
             gid=bytes(p.endpoint_gid).hex())
        for p in publishers], key=lambda p: (p['namespace'], p['name'], p['gid']))
    count = len(endpoints)
    if count == 0:
        kind, reason = 'missing', '尚未发现底盘协调反馈发布者；请等待底盘接入完成'
    elif count > 1:
        kind, reason = 'duplicate', '底盘协调反馈发布者重复（%d 个）；请结束多余的底盘执行程序后重新接入' % count
    elif endpoints[0]['name'] != 'fairino_keyboard_guard' or endpoints[0]['namespace'] != '/':
        kind, reason = 'unexpected', '底盘协调反馈来源异常：'+endpoints[0]['namespace'].rstrip('/')+'/'+endpoints[0]['name']
    else:
        kind, reason = 'valid', None
    return dict(source_ok=kind == 'valid', source_state=kind, publisher_count=count,
                publishers=endpoints, source_reason=reason)


def wait_timeout_from_profile(path):
    """Read the deployed value once; an old partial deployment must fail clearly."""
    import yaml
    try:
        value = yaml.safe_load(path.read_text())['controller_server']['ros__parameters']['FollowPath']['coordination_wait_timeout']
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
        raise RuntimeError('缺少底盘协调配置，请完整重新部署导航') from exc
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= 60:
        raise ValueError('coordination_wait_timeout 必须在 (0, 60] 秒内')
    return float(value)


class GoalCoordination:
    """Never reinterpret a worker restart as recovery of an existing goal."""
    def __init__(self, feedback, wait_timeout):
        if isinstance(wait_timeout, bool) or not isinstance(wait_timeout, (int, float)) or not math.isfinite(wait_timeout) or not 0 < wait_timeout <= 60:
            raise ValueError('Invalid chassis coordination wait timeout')
        self.feedback = feedback
        self.wait_timeout = float(wait_timeout)
        self.goal = None
        self.session = None
        self.wait_since = None

    def receive(self, gate, message, now, now_ros, source_ok, source_details=None):
        # Consume every error event while a goal is active, even when several
        # callbacks run before the next output tick. A later healthy sample
        # must not erase a transient fault or a worker-session replacement.
        self.feedback.receive(message, now, now_ros)
        if gate.goal is not None:
            state = self.check(gate.goal, now, now_ros, source_ok, source_details)
            if not state['ready']:
                gate.fail(state['reason']+'；本次目标已停止', now)

    def check(self, goal, now, now_ros, source_ok, source_details=None):
        if goal != self.goal:
            self.goal = goal
            self.session = None
            self.wait_since = None
        state = dict(self.feedback.snapshot(now, now_ros))
        state.update(source_ok=bool(source_ok), wait_age=0.,
                     wait_limit=self.wait_timeout, goal_session=self.session)
        if source_details is not None:
            state.update(source_details)
        if not source_ok:
            reason = (source_details or {}).get('source_reason')
            if (source_details or {}).get('source_state') == 'missing' and goal is not None:
                reason = '底盘协调反馈发布者已断开；请重新接入底盘'
            state.update(ready=False, reason=reason or '底盘协调反馈发布器缺失或来源不唯一；请检查并重新接入底盘')
        if not state.get('ready'):
            return state
        if goal is None:
            # Idle health is useful to preflight; do not pin a future goal yet.
            self.session = None
            self.wait_since = None
            return state
        if self.session is None:
            self.session = state['session']
        elif state['session'] != self.session:
            state.update(ready=False, reason='底盘执行会话已变化，请重新接入后发送新目标')
            return state
        state['goal_session'] = self.session
        if state.get('drive_wait'):
            if self.wait_since is None:
                self.wait_since = now
            elapsed = now-self.wait_since
            state['wait_age'] = elapsed
            if elapsed < 0 or elapsed >= self.wait_timeout:
                state.update(ready=False, reason=(
                    '底盘等待行走指令落实超时（%.3f 秒，限值 %.3f 秒）；请检查轮组状态'
                    % (elapsed, self.wait_timeout)))
        else:
            self.wait_since = None
        return state


class FeedbackInbox:
    """Serialize receiver updates; only the control thread touches CommandGate."""
    class Goal:
        def __init__(self):
            self.goal = None
            self.failure = None

        def fail(self, reason, now):
            if self.goal is not None and self.failure is None:
                self.failure = dict(goal=self.goal, reason=reason, monotonic_time=now)

    def __init__(self, feedback, wait_timeout):
        self.lock = threading.RLock()
        self.coordination = GoalCoordination(feedback, wait_timeout)
        self.goal = self.Goal()
        self.source_ok = False
        self.source_details = None

    def set_goal(self, goal):
        with self.lock:
            if goal != self.goal.goal:
                self.goal.goal = goal
                self.goal.failure = None
                # Bind session/wait history at the goal boundary even if no
                # feedback arrives between canceling A and accepting B.
                self.coordination.goal = None
                self.coordination.session = None
                self.coordination.wait_since = None

    def set_source(self, source_ok, details=None):
        with self.lock:
            self.source_ok = bool(source_ok)
            self.source_details = details

    def receive(self, message, monotonic, ros_time):
        with self.lock:
            # Sample clocks after taking the lock: a newer callback must never
            # be compared with a timestamp sampled before it acquired the lock.
            self.coordination.receive(self.goal, message, monotonic(), ros_time(), self.source_ok, self.source_details)

    def observe(self, monotonic, ros_time):
        """Independent watchdog retains genuine gaps, even before fresh recovery."""
        with self.lock:
            now = monotonic()
            state = self.coordination.check(self.goal.goal, now, ros_time(), self.source_ok, self.source_details)
            if self.goal.goal is not None and not state['ready']:
                self.goal.fail(state['reason']+'；本次目标已停止', now)
            return state

    def check(self, gate, monotonic, ros_time, source_ok):
        with self.lock:
            self.set_goal(gate.goal)
            self.source_ok = bool(source_ok)
            state = self.observe(monotonic, ros_time)
            failure = self.goal.failure
            if failure is not None and failure['goal'] == gate.goal:
                gate.fail(failure['reason'], failure['monotonic_time'])
            return state
