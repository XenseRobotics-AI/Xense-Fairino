"""Reception proceeds during control/render work; actual stream loss still stops."""
import queue
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from fairino_slam.mapping.remote_stream import CallbackReceiver
from fairino_slam.navigation.motion import CommandGate
from fairino_slam.navigation.velocity_gate import gate_output, publish_checked, readiness_snapshot, receive_execution_state
from tests.navigation.test_gate_timing import Clock, Lifecycle, Monitor, Buffer
from fairino_slam.navigation.velocity_gate import localization_tf_health


class FakeInput:
    def __init__(self, spin_error=False, add_error=False):
        self.tasks = queue.Queue()
        self.stop = threading.Event()
        self.entered = threading.Event()
        self.exit_allowed = threading.Event()
        self.exit_allowed.set()
        self.events = []
        self.errors = []
        self.spin_error, self.add_error = spin_error, add_error
        self.context = NS(ok=lambda: True)
        self.node = NS(context=self.context, destroy_node=lambda: self.events.append('destroy'))
        self.rclpy = NS(create_node=self.create)
        self.worker = None

    def create(self, name, namespace):
        self.events.append((name, namespace))
        return self.node

    def executor(self, context):
        return NS(add_node=self.add, remove_node=lambda n: self.events.append('remove'),
                  shutdown=lambda timeout_sec: self.stop.set(), spin=self.spin)

    def add(self, node):
        if self.add_error:
            raise RuntimeError('add failed')

    def spin(self):
        self.worker = threading.current_thread()
        self.entered.set()
        if self.spin_error:
            raise RuntimeError('input executor failed')
        while not self.stop.is_set():
            try:
                task, done = self.tasks.get(timeout=.01)
            except queue.Empty:
                continue
            try:
                task()
            except BaseException as exc:
                self.errors.append(exc)
            finally:
                done.set()
        self.exit_allowed.wait(2.)
        self.events.append('exit')

    def dispatch(self, task):
        done = threading.Event()
        self.tasks.put((task, done))
        if not done.wait(2.):
            raise AssertionError('receiver did not process pending callback')
        if self.errors:
            raise self.errors.pop(0)

    def start(self):
        receiver = CallbackReceiver(self.rclpy, self.executor, 'input_test',
                                    lambda n: [NS(unregister=lambda: self.events.append('unregister'))])
        assert self.entered.wait(2.)
        return receiver


class InputReceiverTests(unittest.TestCase):
    def receiver(self, **kwargs):
        deps = FakeInput(**kwargs)
        receiver = deps.start()
        self.addCleanup(receiver.close)
        return deps, receiver

    def run_cycle(self, refresh, elapsed=.35):
        deps, receiver = self.receiver()
        clock = Clock()
        lock = threading.RLock()
        monitor = Monitor()
        gate = CommandGate(.06, .1)
        gate.set_goal('goal', clock.monotonic())
        gate.set_execution('following', '跟随', clock.monotonic())
        gate.receive([.03, 0, 0, 0, 0, 0], clock.monotonic())
        state = dict(id='session', initial_pose_stamp=990., active=True)
        odometry = dict(running=True, epoch='epoch', time=clock.wall())
        topics = {t: (clock.monotonic(), .3) for t in ('/scan', '/odom', '/front/scan', '/back/scan')}
        buffer = Buffer(1000., clock)
        def inputs():
            with lock:
                topics.update({t: (clock.monotonic(), .01) for t in topics})
                buffer.stamp = clock.ros()-.01
            gate.receive([.03, 0, 0, 0, 0, 0], clock.monotonic())
            if refresh == 'all':
                stamp = clock.ros()
                msg = NS(header=NS(stamp=NS(sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9))),
                         status=[NS(name='following', message='跟随')])
                with gate.lock:
                    receive_execution_state(gate, msg, clock.monotonic(), clock.ros(), 999., True)
        def busy_control():
            if refresh:
                deps.dispatch(inputs)  # Only the receiver thread can finish this.
        lifecycle = Lifecycle(clock, elapsed, on_tick=busy_control)
        def snapshot():
            health = localization_tf_health(buffer, clock.ros(), 990., object())
            with lock:
                return readiness_snapshot(monitor, state, odometry, topics, lambda: health,
                                          True, clock.monotonic, clock.ros, clock.wall)
        current = gate_output(gate, lifecycle, ('session', 990., 'epoch'), True,
                              snapshot, clock.monotonic)
        return current, gate, receiver

    def test_main_work_does_not_leave_fresh_sensors_and_execution_unprocessed(self):
        current, gate, receiver = self.run_cycle('all')
        self.assertEqual(current['output'], (.03, 0., 0.))
        self.assertIsNone(gate.failure)
        self.assertAlmostEqual(current['topic_ages']['/back/scan']['age'], .01)
        self.assertTrue(receiver.status()['running'])

    def test_live_receiver_cannot_hide_real_scan_loss(self):
        current, gate, _ = self.run_cycle(False)
        self.assertEqual(current['output'], (0., 0., 0.))
        self.assertIn('双雷达观测过期', gate.failure['reason'])

    def test_fresh_sensors_and_commands_do_not_hide_missing_execution(self):
        current, gate, _ = self.run_cycle('sensors', .7)
        self.assertTrue(current['ready'])
        self.assertEqual(current['output'], (0., 0., 0.))
        self.assertIn('执行状态断流', gate.failure['reason'])

    def test_failure_received_then_healthy_never_reopens_goal(self):
        deps, _ = self.receiver()
        gate = CommandGate(.06, .1)
        gate.set_goal('goal', 10.)
        def receive():
            gate.set_execution('failed', '控制器失败', 10.1)
            gate.set_execution('following', '跟随', 10.2)
            gate.receive([.03, 0, 0, 0, 0, 0], 10.2)
        deps.dispatch(receive)
        self.assertEqual(gate.output(10.2, True, 'session'), (0., 0., 0.))
        self.assertEqual(gate.failure['reason'], '控制器失败')

    def test_concurrent_pause_or_fault_wins_before_publication(self):
        for phase in ('waiting', 'failed'):
            with self.subTest(phase=phase):
                deps, _ = self.receiver()
                gate = CommandGate(.06, .1)
                gate.set_goal('goal', 10.)
                gate.set_execution('following', '跟随', 10.)
                gate.receive([.03, 0, 0, 0, 0, 0], 10.)
                self.assertEqual(gate.output(10.1, True, 'session'), (.03, 0., 0.))
                deps.dispatch(lambda: gate.set_execution(phase, '已停止', 10.2))
                published = []
                current = dict(ready=True, stack_ready=True, chassis_motion=dict(ready=True))
                publish_checked(gate, current, 'session', True, published.append, lambda: 10.2)
                self.assertEqual(published, [(0., 0., 0.)])

    def test_close_stops_callbacks_before_destroying_resources(self):
        deps, receiver = self.receiver()
        receiver.close()
        self.assertLess(deps.events.index('exit'), deps.events.index('unregister'))
        self.assertLess(deps.events.index('unregister'), deps.events.index('remove'))
        self.assertEqual(deps.events[-1], 'destroy')
        old = list(deps.events)
        receiver.close()
        self.assertEqual(deps.events, old)

    def test_init_and_spin_failures_are_visible_and_cleaned(self):
        deps = FakeInput(add_error=True)
        with self.assertRaisesRegex(RuntimeError, 'add failed'):
            deps.start()
        self.assertEqual(deps.events[-1], 'destroy')
        deps, receiver = self.receiver(spin_error=True)
        receiver.thread.join(2.)
        self.assertIn('input executor failed', receiver.status()['error'])
        self.assertFalse(receiver.status()['running'])

    def test_unstarted_and_running_thread_failures_preserve_lifetime_rules(self):
        deps = FakeInput()
        with patch('fairino_slam.mapping.remote_stream.threading.Thread.start',
                   side_effect=RuntimeError('start failed')):
            with self.assertRaisesRegex(RuntimeError, 'start failed'):
                deps.start()
        self.assertEqual(deps.events[-1], 'destroy')
        deps, receiver = self.receiver()
        deps.exit_allowed.clear()
        try:
            with patch.object(receiver.thread, 'join', return_value=None):
                with self.assertRaisesRegex(RuntimeError, '未能停止'):
                    receiver.close()
            self.assertNotIn('destroy', deps.events)
        finally:
            deps.exit_allowed.set()
            receiver.thread.join(2.)
