"""Continuous input uses the same transport, startup gate and speed envelope."""
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import control_core as core
import keyboard_control as desktop
import robot_worker as robot
from test_control import hardware
from apps.fairino_keyboard.axis_session import AxisSession


class AxisProtocolTests(unittest.TestCase):
    def worker(self, packet):
        w = robot.Worker.__new__(robot.Worker)
        w.args = NS(linear=.06, angular=.10)
        w.hw = core.normalise_hardware(hardware())
        w.hw_time = w.upper_time = w.received = time.monotonic()
        w.upper, w.input_error = 0, ''
        w.stop, w.input_lock = threading.Event(), threading.Lock()
        w.homing = w.ready = False
        w.packet = packet
        return w

    def packet(self, axes):
        return dict(keys=[], axes=axes, ack=time.monotonic(), focused=True, stop=False,
                    linear=.06, angular=.10)

    def test_real_transport_and_worker_preserve_continuous_direction(self):
        c = desktop.Connection.__new__(desktop.Connection)
        c.done, c.write_lock = threading.Event(), threading.Lock()
        c.last_status, c.speeds = 0., (.06, .10)
        c.latest = {'stamp': time.monotonic()}
        sent = []
        c.child = NS(sendline=lambda value: sent.append(json.loads(value)))
        c.send(axes=(.3, -.4, -.6))
        value = self.worker(sent[-1]).guard()
        self.assertTrue(value)
        self.assertEqual(core.input_twist(value), (.018, -.024, -.06))
        c.send(axes=(0., 0., 0.))
        self.assertFalse(self.worker(sent[-1]).guard())
        with self.assertRaisesRegex(RuntimeError, '不能混发'):
            c.send(keys=['w'], axes=(0., 0., 0.))

    def test_invalid_mixed_expired_and_unfocused_axes(self):
        for axes in ([1., 1., 0.], [float('nan'), 0., 0.], [True, 0., 0.], [0.], None):
            with self.subTest(axes=axes), self.assertRaises(RuntimeError):
                self.worker(self.packet(axes)).guard()
        p = self.packet([.2, .3, .4]); p['keys'] = ['w']
        with self.assertRaises(RuntimeError):
            self.worker(p).guard()
        p = self.packet([.2, .3, .4]); p['ack'] -= 1.
        with self.assertRaisesRegex(RuntimeError, '超时'):
            self.worker(p).guard()
        p = self.packet([.2, .3, .4]); p['focused'] = False
        self.assertEqual(core.input_twist(self.worker(p).guard()), (0., 0., 0.))

    def test_speed_changes_require_axes_neutral_and_keep_limits(self):
        p = self.packet([.5, 0., 0.]); p['linear'] = .2
        with self.assertRaisesRegex(RuntimeError, '回中'):
            self.worker(p).guard()
        p['axes'] = [0., 0., 0.]
        w = self.worker(p)
        self.assertFalse(w.guard())
        self.assertEqual(w.args.linear, .2)
        p['linear'] = .21
        with self.assertRaises(RuntimeError):
            w.guard()
        diagonal = core.check_axes([2**-.5, 2**-.5, 1.])
        vx, vy, wz = core.input_twist(diagonal, core.MAX_LINEAR, core.MAX_ANGULAR)
        self.assertAlmostEqual(math.hypot(vx, vy), .2)
        self.assertEqual(wz, .4)


class SessionGateTests(unittest.TestCase):
    def setUp(self):
        self.s = AxisSession('unused', 'unused', None)

    def test_waits_for_ready_and_a_new_neutral_sample(self):
        self.s.set_axes((0., 0., 0.), 10., True)
        self.assertEqual(self.s.command_at(10., False), (0., 0., 0.))
        self.s.command_at(10.01, True)
        self.assertFalse(self.s.snapshot()['armed'])
        self.s.set_axes((.5, 0., 0.), 10.02, True)
        self.assertEqual(self.s.command_at(10.02, True), (0., 0., 0.))
        self.s.set_axes((0., 0., 0.), 10.03, True)
        self.s.command_at(10.03, True)
        self.s.set_axes((.5, 0., 0.), 10.04, True)
        self.assertEqual(self.s.command_at(10.04, True), (.5, 0., 0.))

    def test_stale_sample_cannot_be_refreshed_by_repeated_update(self):
        self.s.set_axes((0., 0., 0.), 10., True)
        self.s.command_at(10., True)
        self.s.set_axes((.5, 0., 0.), 10.01, True)
        self.assertEqual(self.s.command_at(10.02, True), (.5, 0., 0.))
        self.assertEqual(self.s.command_at(10.3, True), (0., 0., 0.))
        self.s.set_axes((.5, 0., 0.), 10.01, True)
        self.assertEqual(self.s.command_at(10.31, True), (0., 0., 0.))
        self.s.set_axes((.5, 0., 0.), 10.32, True)
        self.assertEqual(self.s.command_at(10.32, True), (0., 0., 0.))
        self.s.set_axes((0., 0., 0.), 10.33, True)
        self.s.command_at(10.33, True)
        self.s.set_axes((.5, 0., 0.), 10.34, True)
        self.assertEqual(self.s.command_at(10.34, True), (.5, 0., 0.))

    def test_pause_reordered_and_missing_source_require_recenter(self):
        for bad in (None, float('nan'), 9.):
            self.s.set_axes((0., 0., 0.), 10., True)
            self.s.command_at(10., True)
            self.s.set_axes((.5, 0., 0.), bad, True)
            self.assertEqual(self.s.command_at(10.01, True), (0., 0., 0.))
        self.s.set_axes((0., 0., 0.), 11., True)
        self.s.command_at(11., True)
        self.s.set_axes(active=False)
        self.s.set_axes((.8, 0., 0.), 11.01, True)
        self.assertEqual(self.s.command_at(11.01, True), (0., 0., 0.))
        self.s.close()
        self.s.set_axes((0., 0., 0.), 12., True)
        self.assertEqual(self.s.command_at(12., True), (0., 0., 0.))

    def test_background_uses_real_sender_and_confirms_cleanup(self):
        sent, connected = [], threading.Event()
        class FakeConnection(desktop.Connection):
            def __init__(self, args, password=None):
                self.done, self.write_lock = threading.Event(), threading.Lock()
                self.events, self.speeds = queue.Queue(), (args.linear, args.angular)
                self.latest = dict(stamp=time.monotonic(), ready=True, hardware={'enabled': [True]*8})
                self.last_status = time.monotonic()
                self.child = NS(sendline=self.receive)
                connected.set()
            def receive(self, line):
                packet = json.loads(line)
                sent.append(packet)
                self.latest['stamp'] = self.last_status = time.monotonic()
            def finish(self):
                self.send(stop=True)
                self.latest.update(ready=False, disabled_confirmed=True)
                self.events.put({'error': '用户停止或 SSH 输入已断开'})
                self.done.set()
        s = AxisSession('unused', 'unused', None, connection_factory=FakeConnection).start()
        self.assertTrue(connected.wait(1.))
        deadline = time.monotonic()+1.
        while not s.snapshot()['armed'] and time.monotonic() < deadline:
            s.set_axes((0., 0., 0.), time.monotonic(), True)
            time.sleep(.005)
        self.assertTrue(s.snapshot()['armed'])
        s.set_axes((.2, -.3, .4), time.monotonic(), True)
        while not any(p.get('axes') == [.2, -.3, .4] for p in sent) and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertTrue(any(p.get('axes') == [.2, -.3, .4] for p in sent))
        self.assertTrue(s.finish())
        self.assertTrue(sent[-1]['stop'])
        self.assertTrue(all(p['focused'] for p in sent))
        self.assertFalse(s.snapshot()['ready'])

    def test_remote_fault_is_preserved_and_cleanup_still_runs(self):
        cleaned = threading.Event()
        class FaultConnection:
            def __init__(self, *args, **kwargs):
                self.events = queue.Queue()
                self.events.put({'error': '底盘故障 0x0E10FF01 J1=30'})
                self.done = threading.Event()
                self.latest = {'ready': False}
                self.last_status = time.monotonic()
            def finish(self):
                cleaned.set()
                self.latest['disabled_confirmed'] = True
        s = AxisSession('unused', 'unused', None, connection_factory=FaultConnection).start()
        self.assertTrue(cleaned.wait(1.))
        self.assertTrue(s.finish())
        self.assertIn('J1=30', s.snapshot()['error'])
        self.assertTrue(s.disabled_confirmed)

    def test_finish_reports_cleanup_separately_from_running_fault(self):
        s = AxisSession('unused', 'unused', None)
        s.failure = '本次运行故障已记录'
        s.disabled_confirmed = True
        s.cleanup_confirmed = False
        self.assertFalse(s.finish())
        s.cleanup_confirmed = True
        self.assertTrue(s.finish())
        self.assertEqual(s.snapshot()['error'], '本次运行故障已记录')


if __name__ == '__main__':
    unittest.main()
