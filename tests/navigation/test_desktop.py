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

from fairino_slam.chassis.control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR
from fairino_slam.navigation.desktop import Target, DriveSession, context


def frame(session='A'):
    return {'map_mode': 'saved', 'saved_map_source': '/maps/test/map.yaml',
            'localization': {'active': True, 'id': session}}


class TargetTests(unittest.TestCase):
    def test_manual_coordinates_include_arrival_heading_without_command(self):
        target = Target('hold_initial')
        target.set(['-1.2', '3', '90'], frame())
        self.assertEqual(target.request(frame()), {'x': -1.2, 'y': 3., 'session_id': 'A', 'heading_mode': 'hold_initial', 'yaw': math.pi/2, 'arrival_heading': 1})
        self.assertEqual(target.pose['yaw'], math.pi/2)
        target.accept_heading({'locked_yaw': .7})
        self.assertEqual(target.pose['yaw'], math.pi/2)
        self.assertEqual(target.label, '待发送')
        self.assertIsNone(target.goal_id)

    def test_manual_arrival_heading_is_not_replaced_by_start_heading(self):
        target = Target('hold_initial')
        current = {**frame(), 'pose': [0., 0., .3]}
        target.set(['1', '2', '90'], current)
        self.assertEqual(target.pose['yaw'], math.pi/2)
        self.assertEqual(target.request(current)['yaw'], math.pi/2)
        target.accept_heading({'locked_yaw': .8})
        self.assertEqual(target.pose['yaw'], math.pi/2)

    def test_path_heading_uses_map_absolute_degrees_and_request_radians(self):
        target = Target('path_tangent')
        current = {**frame(), 'pose': [0., 0., -.4]}
        target.set(['1', '2', '90'], current)
        self.assertEqual(target.request(current), {
            'x': 1., 'y': 2., 'session_id': 'A',
            'heading_mode': 'path_tangent', 'yaw': math.pi/2})
        self.assertEqual(target.label, '待发送')
        self.assertIsNone(target.goal_id)
        target.accept_heading({'locked_yaw': -.4})
        self.assertEqual(target.pose['yaw'], math.pi/2)
        target.accept_heading({'goal_yaw': .9, 'locked_yaw': -.4})
        self.assertEqual(target.pose['yaw'], .9)

    def test_path_heading_is_required_and_finite(self):
        for values in (['1', '2'], ['1', '2', ''], ['1', '2', 'nan'],
                       ['1', '2', 'inf'], ['1', '2', '-inf']):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Target('path_tangent').set(values, frame())

    def test_goal_heading_precedes_legacy_lock_and_invalid_results_are_ignored(self):
        for mode in ('hold_initial', 'path_tangent'):
            target = Target(mode)
            target.set(['1', '2', '90'], frame())
            target.accept_heading({'goal_yaw': .2, 'locked_yaw': .8})
            self.assertEqual(target.pose['yaw'], .2)
            for invalid in (True, None, '0.3', float('nan'), float('inf')):
                target.accept_heading({'goal_yaw': invalid})
                self.assertEqual(target.pose['yaw'], .2)

    def test_clear_and_map_change_keep_selected_heading_mode(self):
        for mode in ('hold_initial', 'path_tangent'):
            target = Target(mode)
            target.set(['1', '2', '90'], frame())
            target.clear()
            self.assertEqual(target.heading_mode, mode)
            target.set(['1', '2', '90'], frame())
            self.assertTrue(target.reconcile(frame('B')))
            self.assertEqual(target.heading_mode, mode)

    def test_map_session_change_invalidates_target_and_path(self):
        for changed in (frame('B'), {**frame(), 'saved_map_source': 'different'}, {'map_mode': 'empty'}):
            target = Target('hold_initial')
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
                Target('hold_initial').set(values, frame())
        with self.assertRaises(ValueError):
            Target('hold_initial').set(['0', '0', '0'], {})
        self.assertIsNone(context({**frame(), 'map_mode': 'mapping'}))

    def test_cross_and_heading_live_in_map_not_rotated_grid_coordinates(self):
        import matplotlib
        matplotlib.use('Agg')
        from fairino_slam.mapping.viewer import MapPlot
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
        self.assertEqual((drive.args.linear, drive.args.angular), (DEFAULT_LINEAR, DEFAULT_ANGULAR))
        self.assertTrue(drive.done.is_set())


class Widget:
    """Minimal signal-aware GTK double: entry changes call real panel callbacks."""
    def __init__(self, **kwargs):
        self.text = kwargs.get('label', '')
        self.editable = self.sensitive = True
        self.callbacks = {}
        self.children = []
        self.placeholder = ''

    def pack_start(self, child, *args):
        self.children.append(child)

    def connect(self, signal, callback):
        self.callbacks[signal] = callback

    def set_text(self, value):
        self.text = value
        if 'changed' in self.callbacks:
            self.callbacks['changed'](self)

    def get_text(self):
        return self.text

    def set_editable(self, value):
        self.editable = value

    def set_sensitive(self, value):
        self.sensitive = value

    def set_placeholder_text(self, value):
        self.placeholder = value

    def set_width_chars(self, value):
        pass

    def set_line_wrap(self, value):
        pass


class PanelPolicyTests(unittest.TestCase):
    """Exercise real callbacks with widgets replaced by inert test doubles, no GTK."""
    @classmethod
    def setUpClass(cls):
        gi, repo = ModuleType('gi'), ModuleType('gi.repository')
        gi.require_version = lambda *a: None
        repo.Gtk = NS(Box=Widget, Entry=Widget, Button=Widget, Label=Widget,
                      Orientation=NS(VERTICAL='vertical'))
        repo.GLib = Mock()
        # Scope mocks to import only, so unrelated tests still detect missing GTK.
        with patch.dict(sys.modules, {'gi': gi, 'gi.repository': repo}):
            sys.modules.pop('fairino_slam.navigation.panel', None)
            cls.module = importlib.import_module('fairino_slam.navigation.panel')
            importlib.import_module('fairino_slam.navigation.waypoint_panel')
        sys.modules.pop('fairino_slam.navigation.panel', None)

    def panel(self, mode='hold_initial'):
        p = self.module.NavigationPanel.__new__(self.module.NavigationPanel)
        p.map_panel = Mock(busy=False)
        p.map_panel.stream.snapshot.return_value = {'frame': frame(), 'received': time.monotonic()}
        p.setting_waypoint = False
        p.waypoint_panel = Mock()
        p.target = Target(mode)
        p.target.set(['1', '2', '0'], frame())
        p.drive, p.lease, p.drive_message = None, False, '未接入'
        p.closed = p.shutdown = p.busy = p.polling = p.owned = p.uncertain_goal = False
        p.recovery = None
        p.recovery_lease = p.recovery_reserved = False
        p.command_lock = threading.Lock()
        p.generation = 2
        p.events = queue.Queue()
        p.state, p.received, p.next_poll = {}, 0., float('inf')
        p.status, p.message = Mock(), Mock()
        p.buttons = {name: Mock() for name in ('drive', 'start', 'end', 'plan', 'goal', 'cancel', 'recover')}
        p.entries = [Mock(), Mock(), Mock()]
        p.acquire, p.release = Mock(return_value=False), Mock()
        p.poll = Mock()  # Status polling is tested separately; never make SSH calls.
        return p

    def test_constructor_loads_configured_mode_and_entry_changes_never_send(self):
        for mode in ('hold_initial', 'path_tangent'):
            with self.subTest(mode=mode):
                map_panel = Mock(busy=False)
                map_panel.stream.snapshot.return_value = {
                    'frame': {**frame(), 'pose': [0., 0., -.4]}, 'received': time.monotonic()}
                with patch.object(self.module, 'configured_heading_mode', return_value=mode), \
                     patch('fairino_slam.navigation.waypoint_panel.WaypointPanel', return_value=Widget()):
                    p = self.module.NavigationPanel(map_panel)
                p.submit, p.poll = Mock(), Mock()
                yaw = p.entries[2]
                self.assertEqual(p.target.heading_mode, mode)
                self.assertTrue(yaw.editable)
                self.assertTrue(yaw.sensitive)
                self.assertIn('changed', yaw.callbacks)
                p.entries[0].set_text('1')
                p.entries[1].set_text('2')
                self.assertIsNone(p.target.pose)
                yaw.set_text('90')
                self.assertAlmostEqual(p.target.pose['yaw'], math.pi/2)
                p.submit.assert_not_called()
                p.control('plan')
                p.submit.assert_called_once_with('plan', p.target.request(p.frame()))
                p.submit.reset_mock()
                p.drive = Mock(ready=True)
                p.control('goal')
                p.submit.assert_called_once_with('goal', p.target.request(p.frame()))


    def test_saved_point_selection_fills_pose_without_dispatch_or_partial_entry_updates(self):
        from tests.navigation.test_waypoints import point, snapshot
        p = self.panel()
        p.entries = [Widget(), Widget(), Widget()]
        for entry in p.entries[:2]:
            entry.connect('changed', p.edit_target)
        p.submit = Mock()
        p.use_waypoint(point(), snapshot()['frame'])
        self.assertEqual([e.get_text() for e in p.entries], ['1.25', '-2.5', '90'])
        self.assertEqual(p.target.pose, point()['pose'])
        self.assertEqual(p.target.request(snapshot()['frame'])['saved_pose'], 1)
        p.submit.assert_not_called()
        self.assertFalse(p.setting_waypoint)
        p.target.goal_id, p.target.label = 'goal', '执行中'
        with self.assertRaisesRegex(ValueError, '执行操作'):
            p.use_waypoint(point('其他点'), snapshot()['frame'])

    def test_path_preview_preserves_user_text_and_never_substitutes_current_heading(self):
        p = self.panel('path_tangent')
        p.target.set(['1', '2', '90'], frame())
        p.events.put(('result', (p.generation, 'plan', p.target.context, {
            'points': [[0., 0.], [1., 2.]], 'goal_yaw': math.pi/2,
            'locked_yaw': -.4, 'heading_policy': 'path_tangent'}, None)))
        p.tick()
        self.assertEqual(p.target.pose['yaw'], math.pi/2)
        p.entries[2].set_text.assert_not_called()
        p.entries[2].set_sensitive.assert_called_with(True)
        self.assertEqual(p.target.points, [[0., 0.], [1., 2.]])

    def test_path_heading_edits_are_blocked_while_goal_is_active(self):
        p = self.panel('path_tangent')
        p.target.set(['1', '2', '90'], frame())
        p.target.goal_id, p.target.label = 'goal', '执行中'
        for entry, text in zip(p.entries, ['2', '3', '180']):
            entry.get_text.return_value = text
        p.edit_target()
        self.assertEqual(p.target.pose, {'x': 1., 'y': 2., 'yaw': math.pi/2})
        p.tick()
        p.entries[2].set_sensitive.assert_called_with(False)

    def test_invalid_path_heading_clears_draft_and_does_not_submit(self):
        p = self.panel('path_tangent')
        p.submit = Mock()
        for invalid in ('', 'nan', 'inf'):
            for entry, text in zip(p.entries, ['1', '2', invalid]):
                entry.get_text.return_value = text
            p.edit_target()
            self.assertIsNone(p.target.pose)
            self.assertEqual(p.target.heading_mode, 'path_tangent')
            p.control('plan')
            p.control('goal')
        p.submit.assert_not_called()

    def test_manual_heading_remains_editable_and_ignores_start_lock(self):
        p = self.panel()
        p.events.put(('result', (p.generation, 'plan', p.target.context,
                                {'points': [[0., 0.], [1., 2.]], 'locked_yaw': .5}, None)))
        p.tick()
        self.assertEqual(p.target.pose['yaw'], 0.)
        p.entries[2].set_text.assert_not_called()
        p.entries[2].set_sensitive.assert_called_with(True)

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
        failure = {'goal': 'goal', 'reason': '导航速度指令断流（0.610 秒 > 0.600 秒）'}
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


class ConnectingStatusTests(unittest.TestCase):
    def test_pending_discovery_is_explained_during_connection(self):
        from fairino_slam.navigation.desktop import navigation_status
        state = dict(chassis_motion=dict(source_state='missing'), reason='尚未发现反馈')
        self.assertEqual(navigation_status(state, connecting=True), '正在接入底盘，等待协调反馈连接')
        self.assertEqual(navigation_status(state, connecting=False), '尚未发现反馈')

    def test_connection_hint_never_hides_fault_or_duplicate_source(self):
        from fairino_slam.navigation.desktop import navigation_status
        state = dict(goal='a', failure=dict(goal='a', reason='底盘反馈中断'),
                     chassis_motion=dict(source_state='missing'))
        self.assertEqual(navigation_status(state, connecting=True), '底盘反馈中断')
        state = dict(chassis_motion=dict(source_state='duplicate'), reason='发布者重复')
        self.assertEqual(navigation_status(state, connecting=True), '发布者重复')
