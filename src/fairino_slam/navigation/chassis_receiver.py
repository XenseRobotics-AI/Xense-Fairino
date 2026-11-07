"""Dedicated chassis feedback reception; no motion commands or goal dispatch."""
import threading
import time


class ChassisReceiver:
    def __init__(self, rclpy, executor_factory, message_type, qos, inbox):
        self.node = self.executor = self.thread = self.subscription = self.timer = None
        self.error = None
        self.inbox = inbox
        self._closing = self._closed = False
        try:
            self.node = rclpy.create_node('velocity_gate_chassis', namespace='/fairino_navigation')
            self.subscription = self.node.create_subscription(
                message_type, '/fairino_navigation/chassis_motion_state', self.receive, qos)
            self.timer = self.node.create_timer(.05, self.observe)
            self.executor = executor_factory(context=self.node.context)
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self._spin, name='fairino-navigation-chassis',
                                           daemon=True)
            self.thread.start()
        except BaseException:
            self.close()
            raise

    def ros_time(self):
        return self.node.get_clock().now().nanoseconds*1e-9

    def receive(self, message):
        self.inbox.receive(message, time.monotonic, self.ros_time)

    def observe(self):
        self.inbox.observe(time.monotonic, self.ros_time)

    def check(self, gate, source_ok):
        state = self.inbox.check(gate, time.monotonic, self.ros_time, source_ok)
        if self.error:
            reason = '底盘反馈接收线程异常：'+self.error
            gate.fail(reason, time.monotonic())
            state.update(ready=False, reason=reason)
        return state

    def _spin(self):
        if self._closing:
            return
        try:
            self.executor.spin()
        except Exception as exc:
            if not self._closing and self.node.context.ok():
                self.error = type(exc).__name__+': '+str(exc)

    def status(self):
        return {'running': bool(self.thread and self.thread.is_alive()), 'error': self.error}

    def close(self):
        if self._closed:
            return
        self._closing = True
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.)
        if self.thread is not None and self.thread.ident is not None:
            self.thread.join(timeout=2.)
            if self.thread.is_alive():
                raise RuntimeError('底盘反馈接收线程未能停止；保留仍在使用的节点')
        if self.executor is not None and self.node is not None:
            self.executor.remove_node(self.node)
        if self.node is not None:
            self.node.destroy_node()
        self._closed = True
