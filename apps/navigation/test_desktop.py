"""Offline navigation UI policy, draw geometry and chassis-session regressions."""
import importlib
import math
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace as NS, ModuleType
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from apps.navigation.desktop import Target, DriveSession, context


def frame(session='A'):
    return {'map_mode': 'saved', 'saved_map_source': '/maps/test/map.yaml',
            'localization': {'active': True, 'id': session}}


class TargetTests(unittest.TestCase):
    def test_absolute_coordinates_and_degrees_without_command(self):
        target = Target()
        target.set(['-1.2', '3', '90'], frame())
        self.assertEqual(target.request(frame()), {'x': -1.2, 'y': 3., 'yaw': math.pi/2, 'session_id': 'A'})
        self.assertEqual(target.label, '待发送')
        self.assertIsNone(target.goal_id)

    def test_map_session_change_invalidates_target_and_path(self):
        for changed in (frame('B'), {**frame(), 'saved_map_source': 'different'}, {'map_mode': 'empty'}):
            target = Target()
            target.set(['1', '2', '0'], frame())
            target.points = [[0, 0], [1, 2]]
            self.assertTrue(target.reconcile(changed))
            self.assertIsNone(target.pose)
            self.assertEqual(target.points, [])
            with self.assertRaises(ValueError):
                target.request(changed)

    def test_invalid_or_unbound_coordinates_rejected(self):
        for values in (['nan', '0', '0'], ['1', 'inf', '0'], ['', '0', '0']):
            with self.assertRaises(ValueError):
                Target().set(values, frame())
        with self.assertRaises(ValueError):
            Target().set(['0', '0', '0'], {})
        self.assertIsNone(context({**frame(), 'map_mode': 'mapping'}))

    def test_cross_and_heading_live_in_map_not_rotated_grid_coordinates(self):
        import matplotlib
        matplotlib.use('Agg')
        from apps.map_viewer.viewer import MapPlot
        plot = MapPlot()
        plot.bounds = (-2., 2., -2., 2.)
        plot.set_navigation({'x': 4., 'y': -3., 'yaw': math.pi/2}, [[0, 0], [4, -3]], '待发送')
        self.assertEqual(list(plot.goal_marker.get_xdata()), [4.])
        self.assertEqual(list(plot.goal_marker.get_ydata()), [-3.])
        self.assertEqual(plot.goal_marker.get_marker(), 'x')
        self.assertAlmostEqual(plot.goal_arrow.xy[0], 4.)
        self.assertAlmostEqual(plot.goal_arrow.xy[1], -2.55)
        plot.fit()
        self.assertGreater(plot.ax.get_xlim()[1], 4.)
        plot.set_navigation()
        self.assertEqual(len(plot.goal_marker.get_xdata()), 0)
        self.assertIsNone(plot.goal_arrow)
        self.assertEqual(len(plot.path_artist.get_xdata()), 0)


class DriveTests(unittest.TestCase):
    def fake_connection(self):
        done = threading.Event()
        c = Mock(done=done, latest={'ready': True}, last_status=time.monotonic())
        def finish():
            c.latest = {'disabled_confirmed': True}
            done.set()
        c.finish.side_effect = finish
        return c

    def test_active_goal_prevents_motor_connection(self):
        factory = Mock()
        drive = DriveSession('host', 'user', 'password', command=lambda *a: {'goal': 'live'}, factory=factory)
        drive.run()
        factory.assert_not_called()
        self.assertTrue(drive.done.is_set())
        self.assertIn('取消', drive.message)

    def test_prepare_must_finish_before_motor_connection(self):
        c = self.fake_connection()
        order = []
        def command(action, *args):
            order.append(action)
            return {}
        def factory(*args, **kw):
            order.append('motor_connection')
            return c
        drive = DriveSession('host', 'user', 'password', command=command, factory=factory)
        c.send.side_effect = lambda **kw: drive.stop()
        drive.run()
        self.assertEqual(order, ['status', 'prepare', 'motor_connection'])

    def test_prepare_failure_or_cancel_never_connects_motors(self):
        for canceled in (False, True):
            factory = Mock()
            def command(action, *args):
                if action == 'prepare':
                    if canceled:
                        drive.stop()
                    else:
                        raise RuntimeError('configuration failed')
                return {}
            drive = DriveSession('host', 'user', 'password', command=command, factory=factory)
            drive.run()
            factory.assert_not_called()
            self.assertTrue(drive.done.is_set())

    def test_shutdown_retains_feedback_failure(self):
        c = self.fake_connection()
        c.last_error = '上身急停反馈缺失'
        drive = DriveSession('host', 'user', 'password', command=lambda *a: {}, factory=lambda *a, **k: c)
        c.send.side_effect = lambda **kw: drive.stop()
        drive.run()
        self.assertIn('上身急停反馈缺失', drive.message)
        self.assertIn('确认下使能', drive.message)

    def test_stop_during_connection_still_disables_and_never_sends_motion(self):
        c = self.fake_connection()
        drive = DriveSession('host', 'user', 'password', command=lambda *a: {})
        def factory(*a, **kw):
            drive.stop()
            return c
        drive.factory = factory
        drive.run()
        c.send.assert_not_called()
        c.finish.assert_called_once()
        self.assertTrue(drive.disabled_confirmed)

    def test_stale_gui_latches_stop_and_runs_cleanup(self):
        c = self.fake_connection()
        drive = DriveSession('host', 'user', 'password', command=lambda *a: {})
        def factory(*a, **kw):
            drive.pulse_time -= 2
            return c
        drive.factory = factory
        drive.run()
        c.send.assert_not_called()
        self.assertTrue(drive.disabled_confirmed)

    def test_heartbeat_has_no_keys_and_explicit_navigation_default_limits(self):
        c = self.fake_connection()
        drive = DriveSession('host', 'user', 'password', command=lambda *a: {}, factory=lambda *a, **k: c)
        c.send.side_effect = lambda **kw: drive.stop()
        drive.run()
        c.send.assert_called_once_with(keys=(), focused=True)
        self.assertTrue(drive.args.navigation)
        self.assertEqual((drive.args.linear, drive.args.angular), (.06, .1))
        self.assertTrue(drive.done.is_set())


class PanelPolicyTests(unittest.TestCase):
    """Exercise real callbacks with widgets replaced by inert test doubles, no GTK."""
    @classmethod
    def setUpClass(cls):
        gi, repo = ModuleType('gi'), ModuleType('gi.repository')
        gi.require_version = lambda *a: None
        repo.Gtk = NS(Box=object)
        repo.GLib = Mock()
        # Scope mocks to import only, so unrelated tests still detect missing GTK.
        with patch.dict(sys.modules, {'gi': gi, 'gi.repository': repo}):
            sys.modules.pop('apps.navigation.panel', None)
            cls.module = importlib.import_module('apps.navigation.panel')
        sys.modules.pop('apps.navigation.panel', None)

    def panel(self):
        p = self.module.NavigationPanel.__new__(self.module.NavigationPanel)
        p.map_panel = Mock(busy=False)
        p.map_panel.stream.snapshot.return_value = {'frame': frame(), 'received': time.monotonic()}
        p.target = Target()
        p.target.set(['1', '2', '0'], frame())
        p.drive, p.lease, p.drive_message = None, False, '未接入'
        p.closed = p.shutdown = p.busy = p.polling = p.owned = p.uncertain_goal = False
        p.generation = 2
        p.events = queue.Queue()
        p.state, p.received, p.next_poll = {}, 0., float('inf')
        p.status, p.message = Mock(), Mock()
        p.buttons = {name: Mock() for name in ('drive', 'start', 'end', 'plan', 'goal', 'cancel')}
        p.entries = [Mock(), Mock(), Mock()]
        p.acquire, p.release = Mock(return_value=False), Mock()
        return p

    def test_old_poll_cannot_clear_uncertain_new_goal(self):
        p = self.panel()
        p.uncertain_goal = True
        p.events.put(('status', (1, {'running': True}, None)))
        p.tick()
        self.assertTrue(p.uncertain_goal)
        self.assertEqual(p.received, 0.)

    def test_accepted_goal_locks_inputs_until_matching_terminal_status(self):
        p = self.panel()
        p.target.goal_id, p.target.label = 'new-goal', '已接收'
        p.events.put(('status', (2, {'last_goal': {'id': 'old-goal', 'status': 4}}, None)))
        p.tick()
        self.assertTrue(p.active())
        p.entries[0].set_sensitive.assert_called_with(False)
        p.events.put(('status', (2, {'last_goal': {'id': 'new-goal', 'status': 4}}, None)))
        p.tick()
        self.assertFalse(p.active())
        self.assertEqual(p.target.label, '已到达')

    def test_protective_stop_displays_first_cause_only_for_matching_goal(self):
        p = self.panel()
        p.target.goal_id, p.target.label = 'goal', '执行中'
        failure = {'goal': 'goal', 'reason': '导航速度指令断流（0.410 秒 > 0.300 秒）'}
        # Keep the controls locked until cancellation is actually acknowledged.
        p.events.put(('status', (2, {'last_goal': {'id': 'goal', 'status': 2},
                                    'goal': 'goal', 'failure': failure}, None)))
        p.tick()
        self.assertTrue(p.active())
        self.assertEqual(p.target.label, '执行中')
        p.events.put(('status', (2, {'last_goal': {'id': 'goal', 'status': 5},
                                    'failure': failure}, None)))
        p.tick()
        self.assertEqual(p.target.label, '保护停止')
        self.assertFalse(p.active())
        self.assertIn(failure['reason'], p.status.set_text.call_args.args[0])
        p.target.goal_id = 'new'
        p.events.put(('status', (2, {'last_goal': {'id': 'new', 'status': 5},
                                    'failure': failure}, None)))
        p.tick()
        self.assertEqual(p.target.label, '已取消')
        self.assertNotIn(failure['reason'], p.status.set_text.call_args.args[0])

    def test_base_lease_refusal_does_not_create_drive(self):
        p = self.panel()
        with patch.object(self.module, 'DriveSession') as factory:
            p.control('drive')
        factory.assert_not_called()
        p.acquire.assert_called_once_with('base', '导航')

    def test_start_requires_navigation_drive_and_ready_localization(self):
        p = self.panel()
        p.submit = Mock()
        p.control('start')
        p.submit.assert_not_called()
        self.assertFalse(p.owned)
        p.drive = Mock(ready=True)
        p.control('start')
        p.submit.assert_not_called()
        f = frame()
        f['localization']['ready'] = True
        p.map_panel.stream.snapshot.return_value = {'frame': f, 'received': time.monotonic()}
        p.control('start')
        p.submit.assert_called_once_with('start')
        self.assertTrue(p.owned)

    def test_start_button_disabled_without_navigation_owner(self):
        p = self.panel()
        f = frame()
        f['localization']['ready'] = True
        p.map_panel.stream.snapshot.return_value = {'frame': f, 'received': time.monotonic()}
        p.tick()
        p.buttons['start'].set_sensitive.assert_called_with(False)

    def test_end_is_available_during_pending_goal_and_stops_drive_immediately(self):
        p = self.panel()
        p.busy, p.drive = True, Mock()
        p.command_lock = threading.Lock()
        p.credentials = lambda: ('host', 'user', 'password')
        with patch.object(self.module, 'execute') as execute:
            p.control('end')
            deadline = time.monotonic()+2
            while p.events.empty() and time.monotonic() < deadline:
                time.sleep(.01)
        p.drive.stop.assert_called_once()
        self.assertEqual([c.args[0] for c in execute.call_args_list], ['cancel', 'stop'])
        self.assertTrue(p.shutdown)


if __name__ == '__main__':
    unittest.main()
