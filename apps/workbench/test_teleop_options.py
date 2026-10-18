"""Explicit auxiliary options and gripper ownership, without starting subprocesses."""
from contextlib import ExitStack
import io
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from apps.workbench.teleop import TeleopPanel, Gtk, GLib
from apps.gripper_control.panel import GripperPanel


@unittest.skipUnless(Gtk.init_check()[0], 'GTK/display unavailable')
class TeleopOptionsTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(GLib, 'timeout_add', return_value=0))
        self.stack.enter_context(patch('apps.workbench.teleop.threading.Thread'))
        self.acquired, self.released = [], []
        def acquire(part, owner):
            self.acquired.append(part)
            return True
        self.panel = TeleopPanel(lambda _: ('host', 'user', 'fake-password'), acquire,
                                 lambda part, owner:self.released.append(part))
        self.addCleanup(self.panel.destroy)
        process = NS(stdin=io.StringIO(), poll=lambda:None)
        self.launch = self.stack.enter_context(patch('apps.workbench.teleop.subprocess.Popen', return_value=process))

    def test_auxiliary_devices_are_explicit_and_default_launch_keeps_them_off(self):
        self.assertFalse(self.panel.grippers.get_active())
        self.assertFalse(self.panel.body_lift.get_active())
        self.panel.start()
        argv = self.launch.call_args.args[0]
        self.assertNotIn('--grippers', argv)
        self.assertNotIn('--body-lift', argv)
        self.assertEqual(self.acquired, ['upper'])

    def test_checked_options_share_one_arm_process_and_acquire_gripper_owner(self):
        self.panel.grippers.set_active(True)
        self.panel.body_lift.set_active(True)
        self.panel.gripper_serials['right'].set_text('EXPLICIT-SN')
        self.panel.start()
        argv = self.launch.call_args.args[0]
        self.assertIn('--grippers', argv)
        self.assertIn('--body-lift', argv)
        self.assertEqual(argv[argv.index('--right-gripper-sn')+1], 'EXPLICIT-SN')
        self.assertEqual(self.acquired, ['upper', 'gripper'])
        self.assertEqual(self.launch.call_count, 1)
        self.panel.events.put(('vr_exit', 0))
        self.panel.tick()
        self.assertEqual(self.released, ['upper', 'gripper'])

    def test_gripper_conflict_does_not_start_robot_or_keep_upper_lock(self):
        self.panel.grippers.set_active(True)
        self.panel.acquire = lambda part, owner: part != 'gripper'
        self.panel.start()
        self.launch.assert_not_called()
        self.assertEqual(self.released, ['upper'])

    def test_debug_panel_is_visible_but_passive_before_connect(self):
        with patch('apps.gripper_control.panel.Session') as session:
            panel = GripperPanel(lambda _:None, lambda *_:True, lambda *_:None)
            self.addCleanup(panel.destroy)
            for widgets in panel.widgets.values():
                for button in widgets['buttons'].values():
                    self.assertFalse(button.get_sensitive())
                self.assertFalse(widgets['target'].get_sensitive())
            self.assertIsNone(panel.session)
            session.assert_not_called()


if __name__ == '__main__':
    unittest.main()
