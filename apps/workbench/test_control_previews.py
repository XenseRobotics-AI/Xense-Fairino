"""Offline UI lifecycle checks: previews must never connect or accept input."""
from contextlib import ExitStack
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

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


if __name__ == '__main__':
    unittest.main()
