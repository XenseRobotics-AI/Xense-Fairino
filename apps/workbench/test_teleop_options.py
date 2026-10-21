"""Explicit auxiliary options and gripper ownership, without starting subprocesses."""
from contextlib import ExitStack
import io
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from apps.workbench.teleop import TeleopPanel, Gtk, GLib
from apps.gripper_control.panel import GripperPanel
from apps.fairino_tcp_probe.tool_frames import load_geometry
from apps.workbench.tool_settings import ToolSettings
from apps.fairino_keyboard.control_core import (DEFAULT_LINEAR, DEFAULT_ANGULAR,
                                                MIN_SPEED, MAX_LINEAR, MAX_ANGULAR)


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
        self.assertFalse(self.panel.chassis.get_active())
        self.panel.start()
        argv = self.launch.call_args.args[0]
        self.assertNotIn('--grippers', argv)
        self.assertNotIn('--body-lift', argv)
        self.assertNotIn('--chassis', argv)
        self.assertEqual(self.acquired, ['upper'])
        self.assertEqual(argv[argv.index('--home-preset')+1], 'default')

    def test_scrollable_settings_keep_stop_and_pause_visible_on_an_850px_panel(self):
        window = Gtk.OffscreenWindow()
        self.addCleanup(window.destroy)
        window.set_size_request(1100, 850)
        window.add(self.panel)
        self.panel.status.set_text('遥操就绪 · 使能 20/20\nTCP：法兰中心 · 双手柄已连接')
        self.panel.gripper_status.set_text('夹爪反馈\n左爪 50% · 右爪 50%\n身体 50% / 200 mm')
        self.panel.chassis_status.set_text('底盘就绪 · 使能 8/8\n指令 vx 0 / vy 0 m/s · 旋转 0 rad/s\n回中保持使能')
        window.show_all()
        while Gtk.events_pending():
            Gtk.main_iteration_do(False)
        self.assertLessEqual(self.panel.get_preferred_height()[0], 850)
        self.assertEqual(self.panel.get_allocated_height(), 850)
        self.assertTrue(self.panel.chassis.is_ancestor(self.panel.settings_scroll))
        self.assertTrue(self.panel.body_lift.get_visible())
        self.assertFalse(self.panel.chassis_linear.get_sensitive())
        controls = [self.panel.stop_button, *self.panel.command_buttons]
        positions = [button.translate_coordinates(self.panel, 0, 0)[1] for button in controls]
        bottom_of_settings = self.panel.settings_scroll.get_allocated_height()
        for button, y in zip(controls, positions):
            self.assertGreaterEqual(y, bottom_of_settings)
            self.assertLessEqual(y+button.get_allocated_height(), 850)
            self.assertFalse(button.is_ancestor(self.panel.settings_scroll))
        adjustment = self.panel.settings_scroll.get_vadjustment()
        self.assertGreater(adjustment.get_upper(), adjustment.get_page_size())
        adjustment.set_value(adjustment.get_upper()-adjustment.get_page_size())
        while Gtk.events_pending():
            Gtk.main_iteration_do(False)
        self.assertEqual([button.translate_coordinates(self.panel, 0, 0)[1] for button in controls], positions)

    def test_wrist_module_defaults_on_and_hot_switch_uses_same_process(self):
        self.assertTrue(self.panel.wrist_controls.toggle.get_active())
        self.panel.start()
        self.assertNotIn('--no-wrist-guard', self.launch.call_args.args[0])
        self.panel.wrist_controls.toggle.set_active(False)
        self.panel.wrist_controls.toggle.set_active(True)
        records = [json.loads(line) for line in self.panel.process.stdin.getvalue().splitlines()]
        self.assertEqual([r['command'] for r in records if 'command' in r], ['guard_off', 'guard_on'])
        self.assertEqual(self.launch.call_count, 1)

    def test_explicit_disabled_module_is_passed_to_standalone_client(self):
        self.panel.wrist_controls.toggle.set_active(False)
        self.panel.start()
        self.assertIn('--no-wrist-guard', self.launch.call_args.args[0])

    def test_selected_home_and_independent_opening_limits_reach_shared_cli(self):
        self.panel.home_preset.set_active_id('right_mirrored')
        self.panel.grippers.set_active(True)
        self.panel.gripper_limits['left'].set_value(60.)
        self.panel.gripper_limits['right'].set_value(35.)
        self.panel.start()
        argv = self.launch.call_args.args[0]
        self.assertEqual(argv[argv.index('--home-preset')+1], 'right_mirrored')
        for side, value in [('left', 60.), ('right', 35.)]:
            self.assertEqual(float(argv[argv.index(f'--{side}-gripper-max-open-percent')+1]), value)
        self.panel.tick()
        self.assertFalse(self.panel.home_preset.get_sensitive())
        self.assertTrue(all(not v.get_sensitive() for v in self.panel.gripper_limits.values()))

    def test_compact_feedback_preserves_source_age_and_is_cleared_on_exit(self):
        record = dict(received=10., devices={'right': dict(connected=True, position=.27, age_ms=50.)})
        self.panel.events.put(('vr', 'WORKBENCH_GRIPPERS '+json.dumps(record)))
        self.panel.tick()
        self.assertEqual(self.panel.gripper_feedback, record)
        self.panel.events.put(('vr_exit', 0))
        self.panel.tick()
        self.assertEqual(self.panel.gripper_feedback, {})

    def test_checked_options_share_one_arm_process_and_acquire_gripper_owner(self):
        self.panel.grippers.set_active(True)
        self.panel.gripper_speed.set_value(.85)
        self.panel.body_lift.set_active(True)
        self.panel.gripper_serials['right'].set_text('EXPLICIT-SN')
        self.panel.start()
        argv = self.launch.call_args.args[0]
        self.assertIn('--grippers', argv)
        self.assertEqual(float(argv[argv.index('--gripper-speed-rad-s')+1]), .85)
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

    def test_chassis_settings_share_keyboard_limits_and_reach_same_client(self):
        calls = []
        def credentials(part):
            calls.append(part)
            return ('192.168.58.3' if part == 'base' else '192.168.58.1', 'fairino', 'fake-password')
        self.panel.credentials = credentials
        self.assertEqual(self.panel.chassis_linear.get_value(), DEFAULT_LINEAR)
        self.assertEqual(self.panel.chassis_angular.get_value(), DEFAULT_ANGULAR)
        for widget, maximum in ((self.panel.chassis_linear, MAX_LINEAR),
                                (self.panel.chassis_angular, MAX_ANGULAR)):
            self.assertEqual(widget.get_adjustment().get_lower(), MIN_SPEED)
            self.assertEqual(widget.get_adjustment().get_upper(), maximum)
            self.assertFalse(widget.get_sensitive())
        self.panel.chassis.set_active(True)
        self.assertTrue(self.panel.chassis_linear.get_sensitive())
        self.panel.chassis_linear.set_value(.12)
        self.panel.chassis_angular.set_value(.25)
        self.panel.start()
        argv = self.launch.call_args.args[0]
        self.assertEqual(calls, ['upper', 'base'])
        self.assertIn('--chassis', argv)
        self.assertEqual(argv[argv.index('--host')+1], '192.168.58.1')
        self.assertEqual(argv[argv.index('--chassis-host')+1], '192.168.58.3')
        self.assertEqual(float(argv[argv.index('--chassis-linear')+1]), .12)
        self.assertEqual(float(argv[argv.index('--chassis-angular')+1]), .25)
        self.assertEqual(self.acquired, ['upper', 'base'])
        self.assertEqual(self.launch.call_count, 1)
        for widget in (self.panel.chassis, self.panel.chassis_linear, self.panel.chassis_angular,
                       self.panel.body_lift, self.panel.home, self.panel.home_preset, self.panel.grippers):
            self.assertFalse(widget.get_sensitive())

    def test_chassis_conflict_rolls_back_upper_and_gripper_without_starting(self):
        self.panel.chassis.set_active(True)
        self.panel.grippers.set_active(True)
        self.panel.acquire = lambda part, owner: part != 'base'
        self.panel.start()
        self.launch.assert_not_called()
        self.assertEqual(self.released, ['upper', 'gripper'])
        self.assertFalse(self.panel.upper_lease or self.panel.gripper_lease or self.panel.base_lease)

    def test_missing_or_distinct_chassis_credentials_cannot_start_partial_session(self):
        self.panel.chassis.set_active(True)
        for base in (None, ('base', 'different-user', 'fake-password'), ('base', 'user', 'different-password')):
            with self.subTest(base=base):
                self.panel.credentials = lambda part: ('upper', 'user', 'fake-password') if part == 'upper' else base
                self.panel.start()
                self.launch.assert_not_called()
                self.assertEqual(self.acquired, [])
        buffer = self.panel.log.get_buffer()
        text = buffer.get_text(buffer.get_start_iter(), buffer.get_end_iter(), False)
        self.assertIn('凭据必须一致', text)
        self.assertNotIn('fake-password', text)
        self.assertNotIn('different-password', text)

    def test_launch_failure_releases_all_acquired_parts(self):
        self.panel.chassis.set_active(True)
        self.panel.grippers.set_active(True)
        self.launch.side_effect = OSError('test process launch failed')
        self.panel.start()
        self.assertEqual(self.acquired, ['upper', 'gripper', 'base'])
        self.assertEqual(self.released, ['upper', 'gripper', 'base'])
        self.assertFalse(self.panel.upper_lease or self.panel.gripper_lease or self.panel.base_lease)

    def test_exit_releases_all_parts_once_and_blocks_restart_until_exit_event(self):
        self.panel.chassis.set_active(True)
        self.panel.grippers.set_active(True)
        self.panel.start()
        self.panel.process.poll = lambda: 0
        self.panel.start()
        self.assertEqual(self.launch.call_count, 1)
        self.panel.events.put(('vr_exit', 0))
        self.panel.tick()
        self.assertEqual(self.released, ['upper', 'gripper', 'base'])
        self.assertTrue(self.panel.chassis.get_sensitive())
        self.assertTrue(self.panel.chassis_linear.get_sensitive())
        self.assertIn('下使能结果见下方日志', self.panel.chassis_status.get_text())
        self.panel.events.put(('vr_exit', 0))
        self.panel.tick()
        self.assertEqual(self.released, ['upper', 'gripper', 'base'])

    def test_chassis_feedback_shows_enabled_count_actual_command_and_fault(self):
        state = dict(phase='遥操就绪', sides='双臂', sensitivity=.25, enabled=20,
                     send_hz=450., accepted_hz=450., chassis=dict(phase='底盘就绪', enabled=8,
                     command=[.06, -.02, .1], error='', message='摇杆回中保持'))
        self.panel.events.put(('vr', 'WORKBENCH_VR '+json.dumps(state)))
        self.panel.tick()
        text = self.panel.chassis_status.get_text()
        for part in ('底盘就绪', '8/8', 'vx +0.060', 'vy -0.020', '+0.100 rad/s', '摇杆回中保持'):
            self.assertIn(part, text)
        state['chassis']['error'] = '底盘反馈超时'
        self.panel.events.put(('vr', 'WORKBENCH_VR '+json.dumps(state)))
        self.panel.tick()
        self.assertIn('故障：底盘反馈超时', self.panel.chassis_status.get_text())

    def test_close_sends_stop_and_keeps_base_owned_until_process_exit(self):
        self.panel.chassis.set_active(True)
        self.panel.start()
        self.panel.close()
        records = [json.loads(line) for line in self.panel.process.stdin.getvalue().splitlines()]
        self.assertEqual(records[-1], {'command': 'stop'})
        self.assertTrue(self.panel.base_lease)
        self.assertEqual(self.released, [])
        self.panel.process.poll = lambda: 0
        self.panel.events.put(('vr_exit', 0))
        self.panel.tick()
        self.assertEqual(self.released, ['upper', 'base'])

    def test_unconfirmed_tcp_never_acquires_robot_or_starts_process(self):
        data = load_geometry()
        for side in data['arms']:
            data['arms'][side]['confirmed'] = False
        self.panel.tcp_tool.set_active_id('taccap')
        with patch('apps.workbench.teleop.load_geometry', return_value=data):
            self.panel.start()
        self.launch.assert_not_called()
        self.assertEqual(self.acquired, [])

    def test_verified_tool_selection_reaches_cli_and_locks_during_motion(self):
        data = load_geometry()
        for side in data['arms']:
            data['arms'][side]['confirmed'] = True
        self.panel.tcp_tool.set_active_id('taccap')
        self.assertTrue(self.panel.grippers.get_active())
        with patch('apps.workbench.teleop.load_geometry', return_value=data):
            self.panel.start()
        argv = self.launch.call_args.args[0]
        self.assertEqual(argv[argv.index('--tcp-tool')+1], 'taccap')
        self.panel.tick()
        self.assertFalse(self.panel.tcp_tool.get_sensitive())
        self.assertFalse(self.panel.tool_settings.get_sensitive())
        self.assertFalse(self.panel.gripper_speed.get_sensitive())

    def test_geometry_change_requires_fresh_confirmation_and_updates_preview(self):
        seen = []
        dialog = ToolSettings(preview=seen.append)
        self.addCleanup(dialog.destroy)
        for row in dialog.rows.values():
            row['confirmed'].set_active(True)
        dialog.rows['left']['angle'].set_value(45.)
        self.assertFalse(dialog.data()['arms']['left']['confirmed'])
        self.assertTrue(dialog.data()['arms']['right']['confirmed'])
        self.assertEqual(seen[-1]['arms']['left']['roll_deg'], 45.)
        dialog.tip[2].set_value(-150.)
        self.assertFalse(any(s['confirmed'] for s in dialog.data()['arms'].values()))

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

    def gripper_state(self, diagnostics=None):
        panel = GripperPanel(lambda _:None, lambda *_:True, lambda *_:None)
        self.addCleanup(panel.destroy)
        unit = dict(connected=True, controlled=False, enabled=False, sn='TCGU01A28Z0018s',
                    port='/dev/right', position=.92, state='未使能', torque_nm=0., temperature_c=27.)
        if diagnostics is not None:
            unit['diagnostics'] = diagnostics
        state = dict(received=time.monotonic(), devices={'right':unit})
        panel.session = NS(snapshot=lambda:state, finished=lambda:False, stop=threading.Event())
        panel.tick()
        return panel

    def test_missing_diagnostics_never_appear_as_a_pass(self):
        panel = self.gripper_state()
        text = panel.widgets['right']['diagnostics'].get_text()
        self.assertIn('尚未更新', text)
        self.assertNotIn('通过', text)

    def test_connected_device_shows_preflight_failure_separately_from_disconnection(self):
        panel = self.gripper_state(dict(firmware='1.2.6', issues=['电机历史故障=0x00000004（欠压）']))
        self.assertIn('使能检查未通过', panel.state.get_text())
        self.assertIn('欠压', panel.state.get_text())
        self.assertIn('92.0%', panel.widgets['right']['info'].get_text())
        self.assertFalse(panel.widgets['right']['buttons']['target'].get_sensitive())


if __name__ == '__main__':
    unittest.main()
