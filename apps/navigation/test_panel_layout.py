"""Real GTK navigation layout and local-only demonstration behavior."""
from contextlib import ExitStack
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from apps.workbench import gtk_support
from gi.repository import Gtk, GLib
from apps.navigation.panel import NavigationPanel


@unittest.skipUnless(Gtk.init_check()[0], 'GTK/display unavailable')
class NavigationLayoutTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(GLib, 'timeout_add', return_value=0))
        self.execute = self.stack.enter_context(patch('apps.navigation.panel.execute'))
        self.drive = self.stack.enter_context(patch('apps.navigation.panel.DriveSession'))
        state = {'received': time.monotonic(), 'frame': {'map_mode': 'saved',
            'saved_map_source': '/demo/map.yaml',
            'localization': {'id': 'demo', 'active': True, 'ready': True}}}
        stream = NS(args=NS(demo=True, host='offline', user='demo'), control_password=None,
                    snapshot=lambda: state, preview_path=lambda pose: [[0., 0.], [pose['x'], pose['y']]])
        self.map_panel = Mock(stream=stream, busy=False)
        self.map_panel.plot = Mock()
        self.panel = NavigationPanel(self.map_panel)
        self.addCleanup(self.panel.destroy)

    def test_navigation_controls_fit_a_340_pixel_sidebar(self):
        window = Gtk.OffscreenWindow()
        self.addCleanup(window.destroy)
        window.set_size_request(340, 740)
        window.add(self.panel)
        window.show_all()
        while Gtk.events_pending():
            Gtk.main_iteration_do(False)
        self.assertLessEqual(self.panel.get_preferred_width()[0], 340)
        self.assertEqual(window.get_allocated_width(), 340)
        self.assertLessEqual(self.panel.get_allocated_width(), 340)
        self.assertEqual(set(self.panel.buttons), {'drive', 'start', 'end', 'pick', 'plan', 'goal', 'cancel'})
        self.assertTrue(self.panel.target_coordinates.get_expanded())
        for button in self.panel.buttons.values():
            x, y = button.translate_coordinates(self.panel, 0, 0)
            self.assertGreaterEqual(x, 0)
            self.assertLessEqual(x+button.get_allocated_width(), 340)
            self.assertLessEqual(y+button.get_allocated_height(), 740)

    def test_integrated_target_coordinates_can_be_edited_without_sending_motion(self):
        p = NavigationPanel(self.map_panel, integrated=True)
        self.addCleanup(p.destroy)
        self.assertIsNone(p.buttons['drive'].get_parent())
        self.assertFalse(p.target_coordinates.get_expanded())
        self.assertTrue(p.set_picked_target(1.2, -2.3, math.pi/2))
        self.assertFalse(p.target_coordinates.get_expanded())
        self.assertIn('1.20, -2.30 m / 90.0°', p.target_coordinates.get_label())
        self.assertIsNone(p.target.goal_id)
        p.target_coordinates.set_expanded(True)
        p.entries[0].set_text('2.5')
        p.entries[2].set_text('-45')
        self.assertEqual(p.target.pose['x'], 2.5)
        self.assertAlmostEqual(p.target.pose['yaw'], -math.pi/4)
        self.assertIsNone(p.target.goal_id)
        p.target_coordinates.set_expanded(False)
        self.assertIn('2.50, -2.30 m / -45.0°', p.target_coordinates.get_label())
        self.execute.assert_not_called()
        self.drive.assert_not_called()

    def test_integrated_status_ellipsis_preserves_full_reason_in_tooltip(self):
        p = NavigationPanel(self.map_panel, integrated=True)
        self.addCleanup(p.destroy)
        reason = '定位未启动；上次定位停止原因：原始轮速输入无新数据，请先检查数据源'
        p.status.set_text(reason)
        self.assertEqual(p.status.get_tooltip_text(), reason)
        p.message.set_text('地图或定位会话已变化，已清除旧目标和预览路径。')
        self.assertEqual(p.message.get_tooltip_text(), p.message.get_text())
        self.execute.assert_not_called()

    def test_map_picking_only_populates_target_and_demo_preview_is_local(self):
        self.assertTrue(self.panel.set_picked_target(1.2, -2.3, math.pi/2))
        self.assertAlmostEqual(self.panel.target.pose['yaw'], math.pi/2)
        self.assertEqual(self.panel.entries[2].get_text(), '90.0000')
        self.assertIsNone(self.panel.target.goal_id)
        self.execute.assert_not_called()
        self.panel.control('pick')
        self.map_panel.pick_navigation_goal.assert_called_once()
        self.panel.control('plan')
        self.assertEqual(self.panel.target.points, [[0., 0.], [1.2, -2.3]])
        self.assertEqual(self.panel.target.label, '离线演示路径')
        self.panel.tick()
        self.assertTrue(self.panel.buttons['plan'].get_sensitive())
        self.assertFalse(self.panel.buttons['goal'].get_sensitive())
        self.execute.assert_not_called()
        self.drive.assert_not_called()

    def test_demo_never_polls_enables_sends_goals_or_stops_real_navigation(self):
        for action in ('drive', 'start', 'goal', 'cancel', 'end'):
            self.panel.control(action)
        self.panel.submit('goal')
        self.panel.poll()
        self.panel.end()
        self.panel.tick()
        self.assertTrue(self.panel.close())
        self.execute.assert_not_called()
        self.drive.assert_not_called()
        self.assertFalse(self.panel.lease)

    def test_real_map_pick_never_sends_a_goal_and_locked_target_stays_unchanged(self):
        self.map_panel.stream.args.demo = False
        self.assertTrue(self.panel.set_picked_target(1., 2., 0.))
        self.panel.control('pick')
        self.map_panel.pick_navigation_goal.assert_called_once()
        self.panel.busy = True
        self.assertFalse(self.panel.set_picked_target(3., 4., 0.))
        self.assertEqual(self.panel.target.pose['x'], 1.)
        self.execute.assert_not_called()
        self.drive.assert_not_called()


if __name__ == '__main__':
    unittest.main()
