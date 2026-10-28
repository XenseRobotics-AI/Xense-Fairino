"""Real GTK map interactions with offline data and all remote actions mocked."""
from contextlib import ExitStack
import copy
import math
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from apps.workbench import gtk_support
from gi.repository import Gtk, GLib
from apps.map_viewer.demo import DemoStream
from apps.map_viewer.panel import MapPanel


@unittest.skipUnless(Gtk.init_check()[0], 'GTK/display unavailable')
class MapPanelInteractionTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(GLib, 'timeout_add', return_value=0))
        self.map_action = self.stack.enter_context(patch('apps.map_viewer.panel.map_action'))
        self.navigation_action = self.stack.enter_context(patch('apps.navigation.panel.execute'))
        self.drive = self.stack.enter_context(patch('apps.navigation.panel.DriveSession'))
        self.temp = self.stack.enter_context(tempfile.TemporaryDirectory())
        self.stream = DemoStream()
        self.stream.elapsed = 20.
        self.state = self.stream.snapshot()
        self.stream.snapshot = lambda: self.state
        self.panel = MapPanel(self.stream, Path(self.temp), start_stream=False)
        self.addCleanup(self.panel.destroy)
        self.panel.show_all()

    def event(self, x=1., y=2., button=1, in_map=True):
        return NS(inaxes=self.panel.plot.ax if in_map else None, button=button, xdata=x, ydata=y)

    def begin_goal(self):
        self.panel.pick_navigation_goal()
        self.assertTrue(self.panel.pick_goal)
        self.panel.pose_press(self.event())

    def test_demo_actions_do_not_start_threads_send_commands_or_enable_hardware(self):
        p = self.panel
        for action in p.buttons:
            p.control(action)
        for action in ('drive', 'start', 'goal', 'cancel', 'end'):
            p.navigation.control(action)
        p.navigation.submit('goal')
        p.navigation.poll()
        self.assertTrue(p.close())
        self.map_action.assert_not_called()
        self.navigation_action.assert_not_called()
        self.drive.assert_not_called()
        self.assertFalse(self.stream.thread.is_alive())
        self.assertTrue(self.stream.stop.is_set())

    def test_demo_controls_and_mode_are_explicitly_simulated(self):
        p = self.panel
        self.assertIn('离线模拟', p.mode.get_text())
        self.assertIn('模拟数据', p.message.get_text())
        self.assertTrue(all(not button.get_sensitive() for button in p.buttons.values()))
        self.assertTrue(all(button.get_sensitive() for button in p.layer_buttons.values()))

    def test_default_workflow_keeps_localization_and_navigation_on_one_page(self):
        p = self.panel
        notebook = p.side_notebook
        labels = [notebook.get_tab_label_text(notebook.get_nth_page(i))
                  for i in range(notebook.get_n_pages())]
        self.assertEqual(labels, ['建图', '定位与导航', '图层'])
        self.assertEqual(notebook.get_current_page(), p.workflow_page)
        page = notebook.get_nth_page(p.workflow_page)
        for widget in (p.map_choice, p.prepare_drive, p.buttons['localize'],
                       p.buttons['initial-pose'], p.pose_coordinates,
                       p.navigation.buttons['start'], p.navigation.buttons['pick'],
                       p.navigation.buttons['plan']):
            self.assertTrue(widget.is_ancestor(page))
            self.assertTrue(widget.is_ancestor(p.workflow_scroll))
        self.assertIsNone(p.navigation.buttons['drive'].get_parent())
        self.assertIs(p.end_drive, p.navigation.buttons['end'])
        for name in ('goal', 'cancel', 'end'):
            button = p.navigation.buttons[name]
            self.assertTrue(button.is_ancestor(page))
            self.assertFalse(button.is_ancestor(p.workflow_scroll))
        self.assertTrue(p.navigation.status.is_ancestor(page))
        self.assertFalse(p.navigation.status.is_ancestor(p.workflow_scroll))
        self.assertFalse(p.navigation.target_coordinates.get_expanded())
        self.map_action.assert_not_called()
        self.navigation_action.assert_not_called()
        self.drive.assert_not_called()

    def test_freeze_retains_canvas_then_unfreeze_updates_robot(self):
        p = self.panel
        old_pose = p.plot.robot_shape.pose.copy()
        p.freeze.set_active(True)
        self.state['frame'] = copy.deepcopy(self.state['frame'])
        self.state['frame']['pose'][0] += .4
        self.state['received'] = time.monotonic()
        p.tick()
        self.assertEqual(p.plot.robot_shape.pose, old_pose)
        self.assertIn('画面已冻结', p.mode.get_text())
        p.freeze.set_active(False)
        p.tick()
        self.assertAlmostEqual(p.plot.robot_shape.pose[0], old_pose[0]+.4)

    def test_layers_survive_ticks_and_clear_only_removes_display_history(self):
        p = self.panel
        for name in ('scans', 'particles', 'uncertainty', 'paths', 'trail'):
            p.layer_buttons[name].set_active(False)
        p.layer_buttons['local_radar'].set_active(True)
        p.tick()
        self.assertTrue(p.plot.local_ax.get_visible())
        for artist in (p.plot.front, p.plot.particles, p.plot.uncertainty, p.plot.path_artist, p.plot.trail_artist):
            self.assertFalse(artist.get_visible())
        p.plot.trail.add([0., 0., 0.], 10., 'test')
        p.plot.trail.add([.1, 0., 0.], 10.1, 'test')
        p.clear_trail()
        self.assertEqual(len(p.plot.trail.samples), 0)
        self.map_action.assert_not_called()
        self.navigation_action.assert_not_called()

    def test_drag_only_sets_position_and_heading_without_sending_goal(self):
        self.begin_goal()
        self.panel.pose_release(self.event(x=1., y=3.))
        pose = self.panel.navigation.target.pose
        self.assertEqual(pose['x'], 1.)
        self.assertEqual(pose['y'], 2.)
        self.assertAlmostEqual(pose['yaw'], math.pi/2)
        self.assertIsNone(self.panel.navigation.target.goal_id)
        self.assertEqual(self.panel.side_notebook.get_current_page(), self.panel.workflow_page)
        self.assertFalse(self.panel.navigation.target_coordinates.get_expanded())
        self.assertIn('1.00, 2.00 m / 90.0°', self.panel.navigation.target_coordinates.get_label())
        self.map_action.assert_not_called()
        self.navigation_action.assert_not_called()

    def test_source_change_during_drag_discards_draft(self):
        self.begin_goal()
        self.state['frame']['saved_map_source'] = 'different-map.yaml'
        self.state['frame']['display_session'][1] = 'different-map.yaml'
        self.panel.pose_release(self.event(x=1., y=3.))
        self.assertIsNone(self.panel.navigation.target.pose)
        self.assertIn('变化', self.panel.message.get_text())

    def test_stale_connection_cannot_begin_or_finish_a_goal_drag(self):
        self.state['received'] -= 5
        self.panel.pick_navigation_goal()
        self.assertFalse(self.panel.pick_goal)
        self.state['received'] = time.monotonic()
        self.panel.tick()
        self.begin_goal()
        self.state['received'] -= 5
        self.panel.pose_release(self.event(x=1., y=3.))
        self.assertIsNone(self.panel.navigation.target.pose)
        self.assertNotIn('目标已填入', self.panel.message.get_text())

    def test_old_display_is_not_bound_to_a_new_live_map_before_redraw(self):
        self.panel.pick_navigation_goal()
        changed = copy.deepcopy(self.state)
        changed['frame']['saved_map_source'] = 'new-map.yaml'
        changed['frame']['display_session'][1] = 'new-map.yaml'
        self.state = changed
        self.panel.pose_press(self.event())
        self.panel.pose_release(self.event(x=1., y=3.))
        self.assertIsNone(self.panel.navigation.target.pose)
        self.assertNotIn('目标已填入', self.panel.message.get_text())

    def test_target_rejection_during_drag_is_not_reported_as_success(self):
        self.begin_goal()
        self.panel.navigation.busy = True
        self.panel.pose_release(self.event(x=1., y=3.))
        self.assertIsNone(self.panel.navigation.target.pose)
        self.assertNotIn('目标已填入', self.panel.message.get_text())

    def test_zoom_buttons_preserve_map_scale_and_leave_follow_mode(self):
        p = self.panel
        p.follow.set_active(True)
        before = p.plot.ax.get_xlim()
        p.zoom(1/1.3)
        self.assertFalse(p.follow.get_active())
        after = p.plot.ax.get_xlim()
        self.assertAlmostEqual((after[1]-after[0])/(before[1]-before[0]), 1/1.3)

    def begin_pose(self):
        self.panel.begin_initial_pose()
        self.assertTrue(self.panel.pick_pose)
        self.panel.pose_press(self.event())
        self.panel.pointer_move(self.event(x=1., y=3.))

    def test_initial_pose_arrow_tracks_drag_and_survives_live_refresh(self):
        self.begin_pose()
        p = self.panel
        self.assertEqual(p.plot.pose_preview.pose, [1., 2., math.pi/2])
        self.assertTrue(p.plot.pose_preview.arrow.get_visible())
        self.state['received'] = time.monotonic()
        p.tick()
        self.assertTrue(p.plot.pose_preview.arrow.get_visible())
        with patch.object(p, 'pose_dialog') as confirm:
            p.pose_release(self.event(x=2., y=2.))
        confirm.assert_called_once_with((1., 2., 0.))
        self.assertEqual(p.plot.pose_preview.pose, [1., 2., 0.])
        self.map_action.assert_not_called()
        self.navigation_action.assert_not_called()

    def test_cancel_short_drag_or_release_outside_clears_preview_without_sending(self):
        p = self.panel
        for action in ('escape', 'right', 'outside', 'short'):
            with self.subTest(action=action):
                self.begin_pose()
                if action == 'escape':
                    p.pick_key(NS(key='escape'))
                elif action == 'right':
                    p.pose_press(self.event(button=3))
                elif action == 'outside':
                    p.pose_release(self.event(in_map=False))
                else:
                    p.pose_release(self.event(x=1.01, y=2.))
                self.assertIsNone(p.plot.pose_preview.pose)
                self.assertFalse(p.plot.pose_preview.arrow.get_visible())
        self.map_action.assert_not_called()
        self.navigation_action.assert_not_called()

    def test_changed_session_clears_preview_during_drag_before_release(self):
        self.begin_pose()
        self.state['frame']['display_session'][1] = 'replacement-map.yaml'
        self.panel.tick()
        self.assertFalse(self.panel.pick_pose)
        self.assertFalse(self.panel.plot.pose_preview.arrow.get_visible())
        with patch.object(self.panel, 'pose_dialog') as confirm:
            self.panel.pose_release(self.event(x=1., y=3.))
        confirm.assert_not_called()

    def test_confirmation_does_not_accept_session_changed_while_dialog_open(self):
        self.begin_pose()
        p = self.panel
        def changed():
            self.state['frame']['display_session'][1] = 'replacement-map.yaml'
            p.tick()  # This also clears the mutable pick context.
            return Gtk.ResponseType.OK
        with patch('apps.map_viewer.panel.Gtk.Dialog') as dialog, patch.object(p, 'control') as control:
            dialog.return_value.run.side_effect = changed
            p.pose_release(self.event(x=1., y=3.))
        control.assert_not_called()
        self.assertIn('变化', p.message.get_text())
        self.assertFalse(p.plot.pose_preview.arrow.get_visible())

    def test_confirmed_pose_keeps_metres_heading_and_session_binding(self):
        self.begin_pose()
        p = self.panel
        with patch('apps.map_viewer.panel.Gtk.Dialog') as dialog, patch.object(p, 'control') as control:
            dialog.return_value.run.return_value = Gtk.ResponseType.OK
            p.pose_release(self.event(x=1., y=3.))
        control.assert_called_once_with('initial-pose', {'x': 1., 'y': 2., 'yaw': math.pi/2,
                                                        'session_id': 'offline-session'})
        self.assertFalse(p.plot.pose_preview.arrow.get_visible())
        self.map_action.assert_not_called()

    def test_unified_workflow_reuses_navigation_owner_for_prepare_and_stop(self):
        p = self.panel
        with patch.object(p.navigation, 'control') as control:
            p.prepare_drive.emit('clicked')
            p.end_drive.emit('clicked')
        self.assertEqual([call.args for call in control.call_args_list], [('drive',), ('end',)])
        self.drive.assert_not_called()

    def test_missing_input_cannot_start_a_blocking_localization_wait(self):
        p = self.panel
        p.demo = False
        self.assertTrue(p.localization_inputs_ready())
        self.state['frame']['localization']['active'] = False
        for topic in ('/eight_dof_chassis_controller/odom', '/odom', '/scan'):
            with self.subTest(topic=topic):
                item = self.state['frame']['topics'].pop(topic)
                with patch('apps.map_viewer.panel.threading.Thread') as worker:
                    p.control('localize')
                    p.tick()
                worker.assert_not_called()
                self.assertFalse(p.busy)
                self.assertFalse(p.buttons['localize'].get_sensitive())
                self.assertIn('准备底盘', p.message.get_text())
                self.state['frame']['topics'][topic] = item
        self.map_action.assert_not_called()

    def test_localization_waits_for_drive_startup_and_accepts_existing_healthy_source(self):
        p = self.panel
        self.assertTrue(p.localization_inputs_ready())
        p.navigation.drive = NS(ready=False)
        self.assertFalse(p.localization_inputs_ready())
        p.navigation.drive.ready = True
        self.assertTrue(p.localization_inputs_ready())
        p.navigation.drive = None
        self.state['received'] -= 3
        self.assertFalse(p.localization_inputs_ready())

    def test_shared_workflow_separates_drive_localization_and_navigation_readiness(self):
        p, nav = self.panel, self.panel.navigation
        p.demo = self.stream.args.demo = False
        nav.next_poll = float('inf')
        nav.drive = Mock(ready=False, message='正在回正')
        nav.drive.done.is_set.return_value = False
        loc = self.state['frame']['localization']
        loc.update(active=False, ready=False)

        nav.tick()
        p.tick()
        self.assertFalse(p.prepare_drive.get_sensitive())
        self.assertFalse(p.buttons['localize'].get_sensitive())
        self.assertFalse(nav.buttons['start'].get_sensitive())

        nav.drive.ready = True
        nav.tick()
        p.tick()
        self.assertTrue(p.buttons['localize'].get_sensitive())
        self.assertFalse(p.buttons['initial-pose'].get_sensitive())
        self.assertFalse(nav.buttons['start'].get_sensitive())

        loc.update(active=True, ready=False)
        nav.tick()
        p.tick()
        self.assertFalse(p.buttons['localize'].get_sensitive())
        self.assertTrue(p.buttons['initial-pose'].get_sensitive())
        self.assertFalse(nav.buttons['start'].get_sensitive())

        loc['ready'] = True
        nav.tick()
        p.tick()
        self.assertTrue(nav.buttons['start'].get_sensitive())
        self.assertFalse(nav.buttons['goal'].get_sensitive())
        self.assertTrue(p.end_drive.get_sensitive())
        self.map_action.assert_not_called()
        self.navigation_action.assert_not_called()
        self.drive.assert_not_called()

    def test_shared_end_remains_available_while_map_or_goal_request_is_busy(self):
        p, nav = self.panel, self.panel.navigation
        p.demo = self.stream.args.demo = False
        nav.next_poll = float('inf')
        nav.drive = Mock(ready=True, message='底盘就绪')
        nav.drive.done.is_set.return_value = False
        p.busy = nav.busy = True
        nav.tick()
        p.tick()
        self.assertFalse(p.prepare_drive.get_sensitive())
        self.assertFalse(p.buttons['initial-pose'].get_sensitive())
        self.assertFalse(nav.buttons['goal'].get_sensitive())
        self.assertTrue(p.end_drive.get_sensitive())
        with patch.object(nav, 'end') as end:
            p.end_drive.emit('clicked')
        end.assert_called_once()
        nav.shutdown = True
        nav.tick()
        p.tick()
        self.assertFalse(p.end_drive.get_sensitive())
        self.navigation_action.assert_not_called()

    def test_navigation_failure_remains_visible_with_details_collapsed(self):
        p, nav = self.panel, self.panel.navigation
        p.demo = self.stream.args.demo = False
        details = nav.details_box.get_parent()
        self.assertIsInstance(details, Gtk.Expander)
        self.assertFalse(details.get_expanded())
        nav.target.set(['1', '2', '0'], self.state['frame'])
        nav.busy = True
        cause = 'Start occupied：规划起点位于障碍膨胀区'
        nav.events.put(('result', ('plan', nav.target.context, {}, cause)))
        with patch.object(nav, 'poll'):
            nav.tick()
            p.tick()
        self.assertFalse(nav.busy)
        self.assertFalse(details.get_expanded())
        self.assertEqual(p.message.get_text(), '操作未完成：'+cause)
        self.assertEqual(nav.message.get_text(), p.message.get_text())
        self.assertTrue(p.message.get_visible())
        self.assertFalse(p.message.is_ancestor(p.workflow_scroll))
        self.map_action.assert_not_called()
        self.navigation_action.assert_not_called()
        self.drive.assert_not_called()


if __name__ == '__main__':
    unittest.main()
