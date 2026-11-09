"""Real reception threads with fake ROS; no physical chassis access."""
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from fairino_slam.navigation.chassis_receiver import ChassisReceiver
from fairino_slam.navigation.coordination import FeedbackInbox
from fairino_slam.navigation.motion import CommandGate
from fairino_slam.chassis.motion_feedback import FeedbackMonitor
from tests.navigation.test_chassis_coordination import message


class InboxTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.inbox = FeedbackInbox(FeedbackMonitor(.4), 15.)
        self.gate = CommandGate(.1, .2)
        self.gate.set_goal('goal', self.now)
        self.inbox.set_goal('goal')
        self.inbox.set_source(True)

    def receive(self, sequence=1, **kwargs):
        self.inbox.receive(message(self.now+90, sequence, **kwargs),
                           lambda: self.now, lambda: self.now+90)

    def check(self):
        return self.inbox.check(self.gate, lambda: self.now, lambda: self.now+90, True)

    def test_new_feedback_during_control_stall_replaces_old_sample(self):
        self.receive(hardware_age=.16)
        # The control callback is busy for 0.3 s. Independent reception continues.
        self.now += .3
        self.receive(2, hardware_age=.01)
        state = self.check()
        self.assertTrue(state['ready'])
        self.assertEqual(state['sequence'], 2)
        self.assertIsNone(self.gate.failure)

    def test_real_expiry_still_latches_after_fresh_recovery(self):
        self.receive(hardware_age=.16)
        self.now += .25
        state = self.inbox.observe(lambda: self.now, lambda: self.now+90)
        self.assertFalse(state['ready'])
        self.assertAlmostEqual(state['hardware_total_age_sec'], .41)
        self.now += .01
        self.receive(2)
        self.assertTrue(self.check()['ready'])
        self.assertIn('硬件反馈已过期', self.gate.failure['reason'])

    def test_transient_fault_and_worker_restart_cannot_be_hidden_by_new_data(self):
        for fault in ('not_ready', 'session'):
            with self.subTest(fault=fault):
                self.setUp()
                self.receive()
                self.check()
                self.now += .01
                self.receive(2, **({'ready': False} if fault == 'not_ready' else {'session': 'b'*32}))
                self.now += .01
                self.receive(3)
                self.check()
                self.assertIsNotNone(self.gate.failure)

    def test_cancel_then_new_goal_clears_old_fault_and_wait_budget(self):
        self.receive(ready=False)
        self.check()
        self.assertIsNotNone(self.gate.failure)
        self.gate.set_goal(None, self.now)
        self.inbox.set_goal(None)
        self.now += .01
        self.receive(2)
        self.gate.set_goal('new', self.now)
        self.inbox.set_goal('new')
        self.assertTrue(self.check()['ready'])
        self.assertIsNone(self.gate.failure)

    def test_wait_timeout_keeps_original_limit(self):
        self.receive(waiting=True)
        for i in range(1, 152):
            self.now = 10.+i*.1
            self.receive(i+1, waiting=True)
            self.inbox.observe(lambda: self.now, lambda: self.now+90)
        self.check()
        self.assertIn('落实超时', self.gate.failure['reason'])

    def test_clock_is_sampled_after_lock_not_before_latest_callback(self):
        self.receive()
        def clock():
            self.assertTrue(self.inbox.lock._is_owned())
            return self.now
        self.assertTrue(self.inbox.check(self.gate, clock, lambda: self.now+90, True)['ready'])


class FakeRos:
    def __init__(self, spin_error=False, add_error=False):
        self.events = []
        self.started = threading.Event()
        self.pending = threading.Event()
        self.done = threading.Event()
        self.stop = threading.Event()
        self.exit_allowed = threading.Event()
        self.exit_allowed.set()
        self.spin_error, self.add_error = spin_error, add_error
        self.context = NS(ok=lambda: True)
        self.node = NS(context=self.context, create_subscription=self.subscribe,
                       create_timer=self.timer, destroy_node=self.destroy,
                       get_clock=lambda: NS(now=lambda: NS(nanoseconds=int((time.monotonic()+1000)*1e9))))
        self.rclpy = NS(create_node=self.create_node)
        self.worker = None
        self.sequence = 0
        self.inbox = FeedbackInbox(FeedbackMonitor(.4), 15.)
        self.inbox.set_source(True)
        self.qos = object()

    def create_node(self, name, namespace):
        self.events.append(('node', name, namespace))
        return self.node

    def subscribe(self, kind, topic, callback, qos):
        self.callback = callback
        self.events.append(('subscription', kind, topic, qos))
        return object()

    def timer(self, seconds, callback):
        self.observe = callback
        self.events.append(('timer', seconds))
        return object()

    def destroy(self):
        self.events.append(('destroy', bool(self.worker and self.worker.is_alive())))

    def executor(self, context):
        assert context is self.context
        return NS(add_node=self.add, remove_node=lambda n: self.events.append('remove'),
                  spin=self.spin, shutdown=self.shutdown)

    def add(self, node):
        self.events.append('add')
        if self.add_error:
            raise RuntimeError('add failed')

    def spin(self):
        self.worker = threading.current_thread()
        self.started.set()
        if self.spin_error:
            raise RuntimeError('receiver failed')
        while not self.stop.wait(.002):
            if self.pending.is_set():
                self.sequence += 1
                self.callback(message(time.monotonic()+1000, self.sequence))
                self.pending.clear()
                self.done.set()
        self.exit_allowed.wait(3.)
        self.events.append('exit')

    def shutdown(self, timeout_sec):
        self.events.append('shutdown')
        self.stop.set()

    def start(self):
        receiver = ChassisReceiver(self.rclpy, self.executor, object, self.qos, self.inbox)
        assert self.started.wait(2.)
        return receiver

    def release(self):
        self.stop.set()
        self.exit_allowed.set()
        if self.worker:
            self.worker.join(2.)


class ReceiverTests(unittest.TestCase):
    def deps(self, **kwargs):
        deps = FakeRos(**kwargs)
        self.addCleanup(deps.release)
        return deps

    def test_reception_continues_while_control_thread_is_waiting(self):
        deps = self.deps()
        receiver = deps.start()
        self.addCleanup(receiver.close)
        self.assertEqual(deps.events[0], ('node', 'velocity_gate_chassis', '/fairino_navigation'))
        self.assertEqual(deps.events[1][2], '/fairino_navigation/chassis_motion_state')
        self.assertIs(deps.events[1][3], deps.qos)
        self.assertIn(('timer', .05), deps.events)
        deps.pending.set()
        self.assertTrue(deps.done.wait(2.))
        gate = CommandGate(.1, .2)
        self.assertEqual(receiver.check(gate, True)['sequence'], 1)
        deps.done.clear()
        deps.pending.set()
        self.assertTrue(deps.done.wait(2.))
        self.assertEqual(receiver.check(gate, True)['sequence'], 2)
        self.assertTrue(receiver.status()['running'])
        self.assertNotEqual(receiver.thread.ident, threading.get_ident())

    def test_thread_failure_stops_goal_and_is_reported(self):
        deps = self.deps(spin_error=True)
        receiver = deps.start()
        self.addCleanup(receiver.close)
        receiver.thread.join(2.)
        gate = CommandGate(.1, .2)
        gate.set_goal('g', time.monotonic())
        self.assertFalse(receiver.check(gate, True)['ready'])
        self.assertIn('receiver failed', receiver.status()['error'])
        self.assertIsNotNone(gate.failure)

    def test_close_joins_before_destroy_and_is_idempotent(self):
        deps = self.deps()
        receiver = deps.start()
        receiver.close()
        self.assertLess(deps.events.index('exit'), deps.events.index('remove'))
        self.assertEqual(deps.events[-1], ('destroy', False))
        events = list(deps.events)
        receiver.close()
        self.assertEqual(deps.events, events)

    def test_add_node_and_thread_start_failures_release_resources(self):
        deps = self.deps(add_error=True)
        with self.assertRaisesRegex(RuntimeError, 'add failed'):
            deps.start()
        self.assertEqual(deps.events[-1], ('destroy', False))
        deps = self.deps()
        with patch('fairino_slam.navigation.chassis_receiver.threading.Thread.start',
                   side_effect=RuntimeError('start failed')):
            with self.assertRaisesRegex(RuntimeError, 'start failed'):
                deps.start()
        self.assertEqual(deps.events[-1], ('destroy', False))

    def test_hung_thread_is_not_destroyed_while_callback_is_running(self):
        deps = self.deps()
        deps.exit_allowed.clear()
        receiver = deps.start()
        try:
            with patch.object(receiver.thread, 'join', return_value=None):
                with self.assertRaisesRegex(RuntimeError, '未能停止'):
                    receiver.close()
            self.assertNotIn(('destroy', True), deps.events)
            self.assertNotIn(('destroy', False), deps.events)
        finally:
            deps.exit_allowed.set()
            receiver.thread.join(2.)
            receiver.close()
