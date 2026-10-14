"""Offscreen UI checks: no robot connection and no desktop focus changes."""
import json
import math
import queue
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import upper_keyboard as ui
from test_probe import hardware

try:
    import gi
    gi.require_version('Gtk', '3.0')
    from gi.repository import Gtk, Gdk
    GTK_AVAILABLE = Gtk.init_check()[0]
except ImportError:
    GTK_AVAILABLE = False


@unittest.skipUnless(GTK_AVAILABLE, 'GTK/display unavailable')
class GuiTests(unittest.TestCase):
    def setUp(self):
        class HiddenWindow(Gtk.OffscreenWindow):
            def present(self):
                pass
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name)
        (self.path/'joint_mapping.json').write_text(json.dumps({'joints': {
            'J15': {'observation': '头部', 'last_angle_deg': 10.62},
            'J16': {'observation': '腰部旋转', 'last_angle_deg': -.05}}}))
        self.here = patch.object(ui, 'HERE', self.path); self.here.start()
        with patch.object(Gtk, 'Window', HiddenWindow):
            self.w = ui.Window(NS(demo=True, group='left'))
        while Gtk.events_pending():
            Gtk.main_iteration_do(False)

    def tearDown(self):
        self.w.closing = True
        self.w.window.destroy()
        self.here.stop()
        self.tmp.cleanup()

    def test_tabs_and_number_keys_select_correct_joints_without_motion(self):
        w = self.w
        self.assertEqual(w.notebook.get_n_pages(), 3)
        self.assertEqual(set(w.notes), set(range(1, 21)))
        for page, group, first, last in ((0, 'left', 1, 7), (1, 'right', 8, 14), (2, 'body', 15, 20)):
            w.notebook.set_current_page(page)
            self.assertEqual(w.group, group)
            self.assertEqual(w.selected, first)
            w.key_press(None, NS(keyval=Gdk.keyval_from_name(str(last-first+1))))
            self.assertEqual(w.selected, last)
            w.key_press(None, NS(keyval=Gdk.KEY_1))
            self.assertEqual(w.selected, first)
            self.assertEqual(w.sequence, 0)
            self.assertIsNone(w.request)
        w.key_press(None, NS(keyval=Gdk.KEY_7))
        self.assertEqual(w.selected, 15)

    def test_pending_arm_jog_locks_tabs_and_preserves_captured_speed(self):
        w = self.w
        w.notebook.set_current_page(1)
        w.buttons[14].set_active(True)
        w.speed.set_value(.2)
        sent = []
        w.args.demo = False
        w.connection = NS(latest={'stamp': time.monotonic()}, events=queue.Queue(),
                          send=lambda *a, **kw: sent.append((a, kw)))
        w.step(1)
        self.assertEqual(w.request['joint'], 14)
        self.assertEqual(w.request['speed_deg_s'], .2)
        self.assertFalse(w.speed.get_sensitive())
        self.assertFalse(w.notebook.get_sensitive())
        w.speed.set_value(.8)
        w.key_press(None, NS(keyval=Gdk.KEY_1))
        self.assertEqual(w.selected, 14)
        self.assertEqual(w.request['speed_deg_s'], .2)
        w.connection.events.put({'phase': '完成', 'ready': True, 'last_request': w.sequence})
        w.tick()
        self.assertFalse(w.pending)
        self.assertTrue(w.notebook.get_sensitive())
        self.assertTrue(w.speed.get_sensitive())

    def test_editing_keys_do_not_jog_and_arm_notes_preserve_body_records(self):
        w = self.w
        for entry in (w.speed, w.notes[1], w.target_angle, w.target_speed):
            entry.grab_focus()
            for key in (Gdk.KEY_Left, Gdk.KEY_Right, Gdk.KEY_2):
                self.assertFalse(w.key_press(None, NS(keyval=key)))
            self.assertEqual(w.sequence, 0)
            self.assertEqual(w.selected, 1)
        w.notes[1].set_text('待现场辨认的肩部关节')
        w.save_notes()
        saved = json.loads((self.path/'joint_mapping.json').read_text())['joints']
        self.assertEqual(saved['J1']['observation'], '待现场辨认的肩部关节')
        self.assertEqual(saved['J15'], {'observation': '头部', 'last_angle_deg': 10.62})
        self.assertEqual(saved['J16']['observation'], '腰部旋转')

    def test_absolute_input_requires_execution_and_locks_captured_goal(self):
        w = self.w
        w.notebook.set_current_page(2)
        w.buttons[16].set_active(True)
        w.last_hw = hardware()
        w.copy_current_angle()
        self.assertAlmostEqual(float(w.target_angle.get_text()), math.degrees(w.last_hw['position'][15]), places=3)
        w.target_angle.set_text('-30.25')
        self.assertIsNone(w.request)
        sent = []
        w.args.demo = False
        w.connection = NS(latest={'stamp': time.monotonic()}, last_status=time.monotonic(),
                          events=queue.Queue(), send=lambda *a, **kw: sent.append((a, kw)))
        w.move_to_target()
        self.assertEqual(w.request['kind'], 'target')
        self.assertEqual(w.request['joint'], 16)
        self.assertEqual(w.request['target_deg'], -30.25)
        self.assertEqual(w.request['speed_deg_s'], 3.)
        self.assertFalse(w.target_angle.get_sensitive())
        self.assertFalse(w.target_speed.get_sensitive())
        self.assertFalse(w.move_button.get_sensitive())
        w.move_to_target(); w.step(1)
        self.assertEqual(w.sequence, 1)
        w.key_press(None, NS(keyval=Gdk.KEY_space))
        self.assertIsNone(w.request)
        self.assertEqual(w.cancel_count, 1)
        w.connection.events.put({'phase': '已中止', 'ready': True, 'last_request': w.sequence})
        w.tick()
        self.assertTrue(w.move_button.get_sensitive())
        w.buttons[17].set_active(True)
        self.assertEqual(w.target_angle.get_text(), '')
        self.assertIn('J17', w.target_joint.get_text())

    def test_invalid_target_and_stale_feedback_never_send(self):
        w = self.w
        for text in ('', 'abc', 'nan', 'inf', '1e309'):
            w.target_angle.set_text(text)
            w.move_to_target()
            self.assertEqual(w.sequence, 0)
        w.target_angle.set_text('20')
        w.last_hw = hardware(); w.args.demo = False
        w.connection = NS(last_status=time.monotonic()-1.)
        w.move_to_target()
        self.assertEqual(w.sequence, 0)
        self.assertIn('过期', w.error.get_text())

    def test_higher_speeds_are_not_clamped_and_disabled_checks_are_shown(self):
        w = self.w
        self.assertEqual(w.target_speed.get_value(), 3.)
        for widget in (w.speed, w.target_speed):
            widget.set_value(12.)
            self.assertEqual(widget.get_value(), 12.)
        w.last_hw = hardware(); w.args.demo = False
        w.connection = NS(latest={'stamp': time.monotonic()}, last_status=time.monotonic(),
                          events=queue.Queue(), send=lambda *a, **kw: None)
        w.target_angle.set_text('20')
        w.move_to_target()
        self.assertEqual(w.request['speed_deg_s'], 12.)
        w.connection.events.put({'phase': '运动中', 'hardware': hardware(), 'ready': False,
                                 'speed_protection': False, 'path_tracking_check': False})
        w.tick()
        self.assertIn('实测超速退出关闭', w.enable_status.get_text())
        self.assertIn('过程跟踪误差检查关闭', w.enable_status.get_text())


if __name__ == '__main__':
    unittest.main()
