"""Dedicated TF ownership and cleanup are tested with fake ROS and real threads."""
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from fairino_slam.navigation import tf_receiver, velocity_gate


class Dependencies:
    def __init__(self, *, listener_error=False, add_error=False, spin_error=False):
        self.events = []
        self.entered = threading.Event()
        self.stop = threading.Event()
        self.exit_allowed = threading.Event()
        self.exit_allowed.set()
        self.callback_requested = threading.Event()
        self.callback_done = threading.Event()
        self.spin_error = spin_error
        self.add_error = add_error
        self.listener_error = listener_error
        self.context = NS(ok=lambda: True)
        self.node = NS(context=self.context, destroy_node=self.destroy_node)
        self.control_node = object()
        self.buffer = NS(latest_stamp=0.)
        self.nodes = []
        self.created = []
        self.listener_args = None
        self.executor_context = None
        self.worker = None
        self.rclpy = NS(create_node=self.create_node)
        self.tf2_ros = NS(Buffer=lambda: self.buffer, TransformListener=self.listener)

    def create_node(self, name, **kwargs):
        self.created.append((name, kwargs))
        self.events.append('create_node')
        return self.node

    def destroy_node(self):
        self.events.append(('destroy_node', bool(self.worker and self.worker.is_alive())))

    def listener(self, buffer, node, *, spin_thread):
        self.listener_args = (buffer, node, spin_thread)
        self.events.append('listener')
        if self.listener_error:
            raise RuntimeError('listener creation failed')
        return NS(unregister=lambda: self.events.append('unregister'))

    def executor(self, *, context):
        self.executor_context = context
        return NS(add_node=self.add_node, remove_node=self.remove_node,
                  spin=self.spin, shutdown=self.shutdown)

    def add_node(self, node):
        self.events.append('add_node')
        if self.add_error:
            raise RuntimeError('add node failed')
        self.nodes.append(node)

    def remove_node(self, node):
        self.events.append('remove_node')
        if node in self.nodes:
            self.nodes.remove(node)

    def spin(self):
        self.worker = threading.current_thread()
        self.entered.set()
        if self.spin_error:
            raise RuntimeError('receiver spin failed')
        while not self.stop.wait(.005):
            if self.callback_requested.is_set():
                self.buffer.latest_stamp = 101.
                self.callback_done.set()
                self.callback_requested.clear()
        self.exit_allowed.wait(3.)
        self.events.append('spin_exited')

    def shutdown(self, *, timeout_sec):
        self.events.append(('shutdown', timeout_sec))
        self.stop.set()
        return True

    def start(self, **kwargs):
        receiver = velocity_gate.TfReceiver(self.rclpy, self.tf2_ros, self.executor, **kwargs)
        if not self.entered.wait(2.):
            receiver.close()
            raise AssertionError('TF thread did not enter executor')
        return receiver

    def release(self):
        self.stop.set()
        self.exit_allowed.set()
        if self.worker:
            self.worker.join(2.)


class TfReceiverTests(unittest.TestCase):
    def fixture(self, **kwargs):
        deps = Dependencies(**kwargs)
        self.addCleanup(deps.release)
        return deps

    def test_receiver_owns_only_its_tf_node_and_independent_executor(self):
        deps = self.fixture()
        receiver = deps.start()
        self.addCleanup(receiver.close)
        self.assertEqual(deps.created, [
            ('velocity_gate_tf', {'namespace': '/fairino_navigation'})])
        self.assertEqual(deps.nodes, [receiver.node])
        self.assertNotIn(deps.control_node, deps.nodes)
        self.assertIs(deps.executor_context, deps.context)
        self.assertEqual(deps.listener_args, (receiver.buffer, receiver.node, False))
        self.assertTrue(receiver.thread.daemon)
        self.assertEqual(receiver.thread.name, 'fairino-navigation-tf')
        self.assertIsNot(receiver.thread, threading.current_thread())
        self.assertEqual(receiver.status(), {'running': True, 'error': None})
        self.assertIs(velocity_gate.TfReceiver, tf_receiver.TfReceiver)

    def test_short_lived_request_can_name_its_own_node_and_worker(self):
        deps = self.fixture()
        receiver = deps.start(node_name='navigation_command_tf',
                              thread_name='fairino-navigation-command-tf')
        self.addCleanup(receiver.close)
        self.assertEqual(deps.created, [
            ('navigation_command_tf', {'namespace': '/fairino_navigation'})])
        self.assertEqual(receiver.thread.name, 'fairino-navigation-command-tf')
        self.assertEqual(deps.nodes, [receiver.node])
        self.assertNotIn(deps.control_node, deps.nodes)
        receiver.close()
        self.assertEqual(deps.events[-1], ('destroy_node', False))

    def test_normal_context_shutdown_does_not_report_a_receiver_fault(self):
        deps = self.fixture(spin_error=True)
        deps.context.ok = lambda: False
        receiver = deps.start()
        self.addCleanup(receiver.close)
        receiver.thread.join(2.)
        self.assertEqual(receiver.status(), {'running': False, 'error': None})
        receiver.close()
        self.assertIn('unregister', deps.events)
        self.assertEqual(deps.events[-1], ('destroy_node', False))

    def test_tf_callback_progresses_while_control_thread_waits(self):
        deps = self.fixture()
        receiver = deps.start()
        self.addCleanup(receiver.close)
        deps.callback_requested.set()
        # Waiting here represents a blocked control callback. Only the owned
        # executor thread can advance the fake TF buffer.
        self.assertTrue(deps.callback_done.wait(2.))
        self.assertEqual(receiver.buffer.latest_stamp, 101.)
        self.assertTrue(receiver.status()['running'])

    def test_close_joins_before_removing_subscriptions_and_is_idempotent(self):
        deps = self.fixture()
        receiver = deps.start()
        join = receiver.thread.join
        def record_join(*args, **kwargs):
            deps.events.append('join')
            return join(*args, **kwargs)
        with patch.object(receiver.thread, 'join', side_effect=record_join):
            receiver.close()
        self.assertFalse(receiver.status()['running'])
        self.assertLess(deps.events.index(('shutdown', 2.)), deps.events.index('join'))
        self.assertLess(deps.events.index('spin_exited'), deps.events.index('unregister'))
        self.assertLess(deps.events.index('join'), deps.events.index('unregister'))
        self.assertLess(deps.events.index('unregister'), deps.events.index('remove_node'))
        self.assertEqual(deps.events[-1], ('destroy_node', False))
        before = list(deps.events)
        receiver.close()
        self.assertEqual(deps.events, before)

    def test_partial_initialization_failure_releases_owned_resources(self):
        for name, kwargs in (
                ('listener', {'listener_error': True}),
                ('executor add', {'add_error': True})):
            with self.subTest(stage=name):
                deps = self.fixture(**kwargs)
                with self.assertRaisesRegex(RuntimeError, 'failed'):
                    deps.start()
                self.assertEqual(deps.events[-1], ('destroy_node', False))
                self.assertFalse(deps.entered.is_set())
                if kwargs.get('add_error'):
                    self.assertIn('unregister', deps.events)

    def test_thread_start_failure_does_not_join_an_unstarted_thread(self):
        deps = self.fixture()
        with patch.object(tf_receiver.threading.Thread, 'start',
                          side_effect=RuntimeError('thread start failed')):
            with self.assertRaisesRegex(RuntimeError, 'thread start failed'):
                deps.start()
        self.assertIn('unregister', deps.events)
        self.assertEqual(deps.events[-1], ('destroy_node', False))
        self.assertFalse(deps.entered.is_set())

    def test_exception_after_thread_started_still_waits_before_destroying(self):
        deps = self.fixture()
        deps.exit_allowed.clear()
        original_start = threading.Thread.start
        original_join = threading.Thread.join
        def start_then_raise(thread):
            original_start(thread)
            self.assertTrue(deps.entered.wait(2.))
            raise RuntimeError('interrupted after thread start')
        def join_releases_callback(thread, *args, **kwargs):
            deps.events.append('join')
            deps.exit_allowed.set()
            return original_join(thread, *args, **kwargs)
        with patch.object(tf_receiver.threading.Thread, 'start', start_then_raise), \
             patch.object(tf_receiver.threading.Thread, 'join', join_releases_callback):
            with self.assertRaisesRegex(RuntimeError, 'interrupted after thread start'):
                deps.start()
        self.assertIn('join', deps.events)
        self.assertLess(deps.events.index('spin_exited'), deps.events.index('unregister'))
        self.assertEqual(deps.events[-1], ('destroy_node', False))

    def test_executor_failure_is_reported_and_its_last_tf_still_expires(self):
        deps = self.fixture(spin_error=True)
        receiver = deps.start()
        self.addCleanup(receiver.close)
        receiver.thread.join(2.)
        self.assertFalse(receiver.status()['running'])
        self.assertIn('RuntimeError: receiver spin failed', receiver.status()['error'])
        buffer = NS(lookup_transform=lambda *args: NS(
            header=NS(stamp=NS(sec=100, nanosec=0))))
        health = velocity_gate.localization_tf_health(buffer, 100.1, 99., object())
        self.assertTrue(health['ready'])
        self.assertTrue(velocity_gate.tf_health_at(health, 100.999)['ready'])
        self.assertFalse(velocity_gate.tf_health_at(health, 101.)['ready'])
        self.assertFalse(velocity_gate.tf_health_at(health, 101.1)['ready'])

    def test_close_keeps_resources_alive_if_worker_has_not_stopped(self):
        deps = self.fixture()
        deps.exit_allowed.clear()
        receiver = deps.start()
        try:
            with patch.object(receiver.thread, 'join', return_value=None):
                with self.assertRaisesRegex(RuntimeError, '未能停止'):
                    receiver.close()
            self.assertNotIn('unregister', deps.events)
            self.assertFalse(any(isinstance(e, tuple) and e[0] == 'destroy_node'
                                 for e in deps.events))
        finally:
            deps.exit_allowed.set()
            receiver.thread.join(2.)
            receiver.close()
        self.assertEqual(deps.events[-1], ('destroy_node', False))


if __name__ == '__main__':
    unittest.main()
