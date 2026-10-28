"""Offline UI lifecycle checks: previews must never connect or accept input."""
from contextlib import ExitStack
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from apps.workbench.app import Workbench, Gtk, Gdk, GLib
import keyboard_control
import upper_keyboard
import body_keyboard
import tcp_keyboard


@unittest.skipUnless(Gtk.init_check()[0], 'GTK/display unavailable')
class ControlPreviewTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        class HiddenWindow(Gtk.OffscreenWindow):
            def present(self):
                pass
        self.stack.enter_context(patch.object(Gtk, 'Window', HiddenWindow))
        self.stack.enter_context(patch.object(GLib, 'timeout_add', return_value=0))
        self.stack.enter_context(patch.object(GLib, 'timeout_add_seconds', return_value=0))
        self.real_refresh_routes = Workbench.refresh_routes
        self.stack.enter_context(patch.object(Workbench, 'refresh_routes', return_value=False))
        def model_stub(app, split):
            app.model = NS(update=lambda *a, **k: None, model=NS(body_state=lambda _: None))
            app.body_readout = Gtk.Label()
            app.hardware_labels = {}
        self.stack.enter_context(patch.object(Workbench, 'add_model', model_stub))
        for feature in (keyboard_control, upper_keyboard, body_keyboard, tcp_keyboard):
            self.stack.enter_context(patch.object(feature, 'Connection', side_effect=AssertionError('Preview connected to robot')))
        self.stack.enter_context(patch.object(tcp_keyboard, 'VendorKinematics', side_effect=AssertionError('Preview loaded live kinematics')))
        self.app = Workbench(NS(demo=False, base_host='192.168.58.3', upper_host='192.168.58.1',
                                user='fairino', page='map', live_arms=True))
        self.addCleanup(self.app.window.destroy)

    def attach_base(self):
        args = self.app.control_args('base')
        self.app.acquire('base', 'base')
        self.app.attach_control('base', args, keyboard_control, (), None)
        return self.app.controls['base']

    def test_all_panels_exist_without_connections_timers_or_live_feedback(self):
        self.assertEqual(set(self.app.previews), {'base', 'joints', 'body', 'tcp'})
        self.assertFalse(self.app.controls)
        self.assertFalse(self.app.leases)
        for name, control in self.app.previews.items():
            with self.subTest(name=name):
                self.assertTrue(control.window.get_visible())
                self.assertFalse(control.window.is_sensitive())
                self.assertIsNone(control.connection)
                self.assertIsNone(control.timer)
                self.assertFalse(control.ready)
                self.assertFalse(control.args.demo)
                self.assertFalse(self.app.control_buttons[name][1].is_sensitive())
        self.assertIsNone(self.app.previews['joints'].last_hw)
        self.assertIsNone(self.app.previews['body'].last_hw)
        self.assertIsNone(self.app.previews['tcp'].hw)
        self.assertFalse(self.app.key_event(None, NS(keyval=Gdk.KEY_w), 'key-press-event'))

    def test_controls_stay_grey_until_ready_and_ignore_keyboard_input(self):
        control = self.attach_base()
        self.assertFalse(control.window.is_sensitive())
        self.assertFalse(control.window.dispatch('key-press-event', NS(keyval=Gdk.KEY_w)))
        self.assertFalse(control.held)
        self.assertTrue(self.app.control_buttons['base'][1].is_sensitive())
        control.ready = True
        self.app.refresh_control_access()
        self.assertTrue(control.window.is_sensitive())
        # A transient busy state must not swallow a pressed button's release.
        control.ready = False
        self.app.refresh_control_access()
        self.assertTrue(control.window.is_sensitive())
        control.stopped = True
        self.app.refresh_control_access()
        self.assertFalse(control.window.is_sensitive())

    def test_ending_a_session_keeps_full_disabled_panel_and_allows_restart(self):
        self.attach_base()
        self.app.stop_control('base')
        self.assertNotIn('base', self.app.controls)
        self.assertNotIn('base', self.app.leases)
        self.assertEqual(len(self.app.page_controls['base'].get_children()), 1)
        self.assertFalse(self.app.previews['base'].window.is_sensitive())
        self.assertTrue(self.app.control_buttons['base'][0].is_sensitive())
        self.attach_base()
        self.assertEqual(len(self.app.page_controls['base'].get_children()), 1)
        self.assertFalse(self.app.controls['base'].window.is_sensitive())
        self.app.stop_control('base')

    def test_start_failure_preserves_disabled_panel_and_releases_ownership(self):
        self.app.acquire('upper', 'body')
        self.app.starting.add('body')
        self.app.events.put(('control_error', ('body', '模拟连接失败')))
        self.app.tick()
        self.assertFalse(self.app.starting)
        self.assertFalse(self.app.leases)
        self.assertFalse(self.app.previews['body'].window.is_sensitive())
        self.assertTrue(self.app.control_buttons['body'][0].is_sensitive())
        self.assertIn('模拟连接失败', self.app.message.get_text())

    def test_reconnecting_map_reparents_live_base_without_destroying_or_releasing_it(self):
        class MapStub(Gtk.Box):
            def __init__(self, stream, *args, **kwargs):
                super().__init__()
                self.stream = stream
                self.base_expander = Gtk.Expander()
                self.pack_start(self.base_expander, False, False, 0)

            def attach_base_controls(self, widget):
                self.base_expander.add(widget)

            def detach_base_controls(self):
                widget = self.base_expander.get_child()
                self.base_expander.remove(widget)
                return widget

            def close(self):
                return True

        control = self.attach_base()
        destroyed = []
        control.window.connect('destroy', lambda *_: destroyed.append(True))
        widget = self.app.base_controls_widget
        stream = NS(thread=Mock())
        for _ in range(2):
            self.app.mount_map_panel(MapStub, stream, start_stream=False)
            self.assertTrue(widget.is_ancestor(self.app.map_panel))
            self.assertTrue(self.app.map_panel.base_expander.get_expanded())
            self.assertTrue(self.app.disconnect_observers())
            self.assertIs(widget.get_parent(), self.app.map_container)
            self.assertIs(self.app.controls['base'], control)
            self.assertEqual(self.app.leases['base'], 'base')
            self.assertFalse(control.closing)
            self.assertFalse(destroyed)
        self.app.stop_control('base')

    def test_map_can_use_full_width_and_other_pages_restore_model(self):
        self.app.model_panel = Gtk.Box()
        self.app.model_panel.set_no_show_all(True)
        self.app.content_split.pack2(self.app.model_panel, resize=True, shrink=False)
        self.app.update_model_visibility()
        self.assertFalse(self.app.model_panel.get_visible())
        self.assertTrue(self.app.hardware_strip.get_visible())
        self.app.notebook.set_current_page(1)
        self.assertTrue(self.app.model_panel.get_visible())
        self.app.notebook.set_current_page(0)
        self.assertFalse(self.app.model_panel.get_visible())
        self.app.model_toggle.set_active(True)
        self.assertTrue(self.app.model_panel.get_visible())
        self.app.notebook.set_current_page(1)
        self.app.notebook.set_current_page(0)
        self.assertTrue(self.app.model_panel.get_visible())

    def test_demo_route_status_never_starts_network_worker(self):
        self.app.args.demo = True
        with patch('apps.workbench.app.threading.Thread') as thread, \
                patch('apps.workbench.app.route_info') as route:
            self.assertTrue(self.real_refresh_routes(self.app))
        thread.assert_not_called()
        route.assert_not_called()
        self.assertIn('离线演示', self.app.routes.get_text())

    def test_long_option_pages_scroll_without_expanding_window_or_changing_page_indexes(self):
        self.app.window.set_size_request(1366, 900)
        for name in ('gripper', 'teleop', 'maintenance'):
            scroll = self.app.page_scrolls[name]
            index = list(self.app.pages).index(name)
            self.assertIs(self.app.notebook.get_nth_page(index), scroll)
            self.assertTrue(self.app.pages[name].is_ancestor(scroll))
            extra = Gtk.Box()
            extra.set_size_request(-1, 2400)
            self.app.pages[name].pack_end(extra, False, False, 0)
            extra.show()
            self.app.notebook.set_current_page(index)
            self.app.window.check_resize()
            while Gtk.events_pending():
                Gtk.main_iteration_do(False)
            self.assertEqual(self.app.window.get_allocated_height(), 900)
            adjustment = scroll.get_vadjustment()
            self.assertGreater(adjustment.get_upper(), adjustment.get_page_size())
            adjustment.set_value(adjustment.get_upper()-adjustment.get_page_size())
            self.assertGreater(adjustment.get_value(), 0)
        self.app.notebook.set_current_page(0)
        self.assertFalse(self.app.controls)
        self.assertFalse(self.app.leases)

    def test_demo_map_fits_standard_laptop_windows(self):
        from apps.map_viewer.demo import DemoStream
        from apps.map_viewer.panel import MapPanel
        self.app.mount_map_panel(MapPanel, DemoStream(), start_stream=False)
        for width, height in ((1500, 940), (1366, 900)):
            with self.subTest(width=width, height=height):
                self.app.window.set_default_size(width, height)
                self.app.window.set_size_request(width, height)
                self.app.window.resize(width, height)
                self.app.window.check_resize()
                # OffscreenWindow processes a resize on the next frame clock.
                for _ in range(12):
                    while Gtk.events_pending():
                        Gtk.main_iteration_do(False)
                    time.sleep(.01)
                self.assertEqual(self.app.window.get_allocated_width(), width)
                self.assertEqual(self.app.window.get_allocated_height(), height)
                self.assertGreater(self.app.map_panel.canvas.get_allocated_height(), 200)
        self.assertFalse(self.app.map_panel.base_expander.get_expanded())
        self.assertFalse(self.app.controls)
        self.assertFalse(self.app.leases)


if __name__ == '__main__':
    unittest.main()
