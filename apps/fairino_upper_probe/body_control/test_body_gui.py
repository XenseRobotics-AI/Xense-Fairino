import json
import queue
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import body_keyboard as ui
from body_core import load_config

try:
    import gi
    gi.require_version('Gtk','3.0')
    from gi.repository import Gtk,GLib,Gdk
    GTK_AVAILABLE=Gtk.init_check()[0]
except ImportError:
    GTK_AVAILABLE=False


@unittest.skipUnless(GTK_AVAILABLE,'GTK/display unavailable')
class GuiTests(unittest.TestCase):
    def setUp(self):
        class HiddenWindow(Gtk.OffscreenWindow):
            def present(self):pass
        with patch.object(Gtk,'Window',HiddenWindow):
            self.w=ui.Window(NS(demo=True),load_config())
        while Gtk.events_pending():Gtk.main_iteration_do(False)

    def tearDown(self):
        self.w.closing=True
        GLib.source_remove(self.w.timer)
        self.w.window.destroy()

    def test_presets_do_not_execute_or_move(self):
        w=self.w
        self.assertEqual(w.target(),dict(height=0,forward=0,roll=0,yaw=0))
        start=w.last_hw['position'][:]
        for h in (100,50,0):
            w.preset(h)
            self.assertEqual(w.height.get_value(),h)
            self.assertEqual(w.last_hw['position'],start)
            self.assertEqual(w.sequence,0)
        self.assertEqual(w.speed.get_text(),'5')

    def test_tilt_gate_and_recenter_before_lowering(self):
        w=self.w
        w.preset(100)
        self.assertTrue(w.inputs['roll'].get_sensitive())
        w.inputs['roll'].set_value(15)
        w.height.set_value(89)
        self.assertEqual(w.inputs['roll'].get_value(),0)
        self.assertFalse(w.inputs['roll'].get_sensitive())
        w.height.set_value(100)
        w.inputs['roll'].set_value(-15)
        w.inputs['forward'].set_value(6)
        self.assertEqual(w.inputs['roll'].get_value(),0)
        self.assertFalse(w.inputs['roll'].get_sensitive())

    def test_execute_captures_speed_locks_controls_and_space_cancels(self):
        w=self.w
        w.preset(100)
        w.speed.set_text('20')
        w.execute()
        self.assertEqual(w.demo_plan['speed_deg_s'],20)
        self.assertTrue(w.pending)
        self.assertFalse(w.controls.get_sensitive())
        w.execute()
        self.assertEqual(w.sequence,1)
        w.key_press(None,NS(keyval=Gdk.KEY_space))
        self.assertIsNone(w.demo_plan)
        self.assertFalse(w.pending)
        self.assertTrue(w.controls.get_sensitive())

    def test_invalid_speed_never_sends(self):
        w=self.w
        for bad in ('nan','inf','0','-5','abc'):
            w.speed.set_text(bad);w.execute()
            self.assertEqual(w.sequence,0)
            self.assertFalse(w.pending)

    def test_heartbeat_does_not_depend_on_window_focus(self):
        w=self.w;sent=[]
        w.connection=NS(send=lambda *args,**kwargs:sent.append((args,kwargs)))
        w.send()
        self.assertTrue(sent[-1][1]['focused'])
        self.assertFalse(sent[-1][1]['exit'])

    def test_worker_source_is_complete(self):
        args=NS(check=True,prepare_check=False)
        command=ui.remote_command(args,wmodel:=load_config())
        self.assertIn('ROS_DOMAIN_ID=0',command)
        self.assertIn('UPPERPROBE_CONNECTED',command)
        self.assertIn('body_robot.py',command)

    def test_calibration_uses_read_only_backend(self):
        import shlex
        args=NS(check=False,prepare_check=False,calibrate_upright=True)
        command=ui.remote_command(args,load_config())
        self.assertIn('--check',command)
        self.assertNotIn('--prepare-check',command)

    def test_unconfirmed_live_entry_never_connects(self):
        import tempfile
        from pathlib import Path
        config=dict(load_config().config,upright_confirmed=False)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'config.json'
            path.write_text(json.dumps(config))
            with patch.object(ui.sys,'argv',['body_keyboard.py','--config',str(path)]), \
                 patch.object(ui,'Connection',side_effect=AssertionError('不应连接机器人')):
                with self.assertRaisesRegex(ValueError,'竖直参考姿态尚未确认'):
                    ui.main()


if __name__=='__main__':unittest.main()
