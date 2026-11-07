"""Owned TF-only reception shared by navigation gates and short-lived requests."""
import threading


class TfReceiver:
    """Receive only TF on an owned node/executor; control callbacks stay serial."""
    def __init__(self, rclpy, tf2_ros, executor_factory, *,
                 node_name='velocity_gate_tf', thread_name='fairino-navigation-tf'):
        self.node = self.listener = self.executor = self.thread = None
        self.buffer = None
        self.error = None
        self._closing = self._closed = False
        try:
            self.node = rclpy.create_node(node_name, namespace='/fairino_navigation')
            self.buffer = tf2_ros.Buffer()
            # Own the executor/thread explicitly: Humble's unregister() only
            # removes subscriptions, whereas its private thread stops in __del__.
            self.listener = tf2_ros.TransformListener(self.buffer, self.node, spin_thread=False)
            self.executor = executor_factory(context=self.node.context)
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self._spin, name=thread_name,
                                           daemon=True)
            self.thread.start()
        except BaseException:
            self.close()
            raise

    def _spin(self):
        if self._closing:
            return
        try:
            self.executor.spin()
        except Exception as exc:
            if not self._closing and self.node.context.ok():
                self.error = type(exc).__name__ + ': ' + str(exc)

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
                raise RuntimeError('TF 接收线程未能停止；保留其节点，避免销毁仍在使用的资源')
        try:
            if self.listener is not None:
                self.listener.unregister()
        finally:
            try:
                if self.executor is not None and self.node is not None:
                    self.executor.remove_node(self.node)
            finally:
                if self.node is not None:
                    self.node.destroy_node()
                self._closed = True

