"""Prepare Nav2 before chassis enable; activate through the existing ROS node."""
import json
from pathlib import Path
import time
import uuid

NAMES = ('planner_server', 'controller_server', 'bt_navigator')
NS = '/fairino_navigation/'


def configure():
    """Called only before a chassis connection exists. Never activate or drive."""
    import rclpy
    from lifecycle_msgs.srv import GetState, ChangeState
    rclpy.init()
    node = rclpy.create_node('fairino_navigation_prepare')
    def call(kind, topic, request, timeout):
        client = node.create_client(kind, topic)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError('导航节点未准备好：'+topic)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done():
                raise RuntimeError('导航配置超时：'+topic)
            return future.result()
        finally:
            node.destroy_client(client)
    try:
        for name in NAMES:
            topic = NS+name
            state = call(GetState, topic+'/get_state', GetState.Request(), 25).current_state.id
            if state == 1:
                request = ChangeState.Request()
                request.transition.id = 1  # configure -> inactive
                if not call(ChangeState, topic+'/change_state', request, 5).success:
                    raise RuntimeError('导航配置失败：'+name)
            elif state != 2:
                raise RuntimeError('导航节点不是待机状态，请先结束导航：'+name)
            if call(GetState, topic+'/get_state', GetState.Request(), 5).current_state.id != 2:
                raise RuntimeError('导航配置未确认：'+name)
    finally:
        node.destroy_node()
        rclpy.shutdown()


class LifecycleMonitor:
    """Nonblocking lifecycle checks and bounded, session-bound activation."""
    def __init__(self, node, root):
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.srv import ManageLifecycleNodes
        self.GetState, self.Manage = GetState, ManageLifecycleNodes
        self.clients = {name: node.create_client(GetState, NS+name+'/get_state') for name in NAMES}
        self.manager = node.create_client(ManageLifecycleNodes, NS+'lifecycle_manager/manage_nodes')
        self.root = Path(root)
        self.run_id = uuid.uuid4().hex
        self.states, self.pending = {}, {}
        self.last_poll = -10.
        self.activation = {}
        self.future = None

    def active(self, now):
        return all(name in self.states and self.states[name][0] == 3
                   and now-self.states[name][1] < 3 for name in NAMES)

    def prepared(self, now):
        return all(name in self.states and self.states[name][0] == 2
                   and now-self.states[name][1] < 3 for name in NAMES)

    def tick(self, now, healthy, session, busy):
        for name, (future, started) in list(self.pending.items()):
            if future.done():
                try:
                    self.states[name] = (future.result().current_state.id, now)
                except Exception:
                    self.states.pop(name, None)
                self.pending.pop(name)
            elif now-started > 2:
                future.cancel()
                self.pending.pop(name)
                self.states.pop(name, None)
        if now-self.last_poll >= 1:
            self.last_poll = now
            for name, client in self.clients.items():
                if name not in self.pending and client.service_is_ready():
                    self.pending[name] = (client.call_async(self.GetState.Request()), now)
        if self.future is not None:
            if time.time() > self.activation.get('expires', float('inf')):
                self.activation.update(state='failed', error='导航节点激活超时，请查询状态')
            if not healthy or self.activation.get('session') != list(session):
                self.activation.update(state='failed', error='激活期间定位或会话失效，请重新检查')
            if self.future.done():
                try:
                    ok = self.future.result().success
                    if self.activation.get('state') != 'failed':
                        self.activation.update(state='completed' if ok else 'failed',
                                               error='' if ok else '导航节点激活失败，请查看日志')
                except Exception as exc:
                    self.activation.update(state='failed', error=str(exc))
                self.future = None
            return
        try:
            request = json.loads((self.root/'activation-request.json').read_text())
        except (OSError, ValueError):
            return
        if request.get('id') == self.activation.get('id') or request.get('run_id') != self.run_id:
            return
        self.activation = {'id': request.get('id'), 'session': request.get('session'), 'state': 'failed'}
        if request.get('expires', 0) < time.time():
            self.activation['error'] = '导航激活请求已过期'
        elif not healthy or request.get('session') != list(session):
            self.activation['error'] = '定位或里程计会话未就绪'
        elif busy:
            self.activation['error'] = '已有导航目标，拒绝重复激活'
        elif self.active(now):
            self.activation.update(state='completed', error='')
        elif not self.prepared(now) or not self.manager.service_is_ready():
            self.activation['error'] = '导航尚未准备好，请结束导航后重新接入底盘'
        else:
            message = self.Manage.Request()
            message.command = 2  # RESUME activates the preconfigured nodes and bonds
            self.future = self.manager.call_async(message)
            self.activation.update(state='pending', error='', expires=request['expires'])
