"""PICO model routing, mode separation, and wrist-envelope display; no robot I/O."""
from contextlib import ExitStack
import io
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from apps.workbench.teleop import TeleopPanel, Gtk, GLib
from apps.workbench.model_view import (ModelView, DEMO_ARM_COLOR, DEMO_COLLISION_COLOR)


def sample_state(**updates):
    state = dict(hardware=dict(joint_names=[f'j{i}' for i in range(1, 21)],
                               position=[0.]*20),
                 grippers={'left': .25, 'right': .75}, phase='跟随',
                 sensitivity=.25, wrist_guard_enabled=True,
                 ik_backend='native', ik_requested_backend='native',
                 wrist_collision={'left': False, 'right': False})
    state.update(updates)
    return state


@unittest.skipUnless(Gtk.init_check()[0], 'GTK/display unavailable')
class ModelDemoPanelTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(GLib, 'timeout_add', return_value=0))
        self.stack.enter_context(patch('apps.workbench.teleop.threading.Thread'))
        self.credentials = Mock(return_value=('host', 'user', 'fake-password'))
        self.acquire, self.release = Mock(return_value=True), Mock()
        self.panel = TeleopPanel(self.credentials, self.acquire, self.release)
        self.addCleanup(self.panel.destroy)
        self.process = NS(stdin=io.StringIO(), poll=lambda: None)
        self.launch = self.stack.enter_context(patch('apps.workbench.teleop.subprocess.Popen',
                                                    return_value=self.process))

    def demo(self):
        self.panel.runtime_target.set_active_id('model')

    def frame(self, state=None):
        self.panel.events.put(('demo', 'WORKBENCH_DEMO '+json.dumps(state or sample_state())))
        self.panel.tick()

    def test_default_real_and_whole_console_demo_cannot_select_real(self):
        self.assertFalse(self.panel.model_demo_selected())
        other = TeleopPanel(self.credentials, self.acquire, self.release, demo=True)
        self.addCleanup(other.destroy)
        self.assertTrue(other.model_demo_selected())
        self.assertFalse(other.runtime_target.get_sensitive())
        self.assertFalse(other.runtime_target.set_active_id('robot'))
        other.start()
        self.credentials.assert_not_called()
        self.acquire.assert_not_called()

    def test_demo_launch_is_local_and_never_asks_for_or_transmits_credentials(self):
        self.demo()
        self.panel.grippers.set_active(True)
        self.panel.body_lift.set_active(True)
        self.panel.teleop_mode.set_active_id('first_person')
        self.panel.gripper_limits['left'].set_value(70.)
        self.panel.start()
        args = self.launch.call_args.args[0]
        self.assertTrue(args[0].endswith('/.local/pico4/env/bin/python'))
        self.assertTrue(args[2].endswith('/vr_demo.py'))
        self.assertIn('--freeze-buttons', args)
        self.assertEqual(args[args.index('--ik-backend')+1], 'native')
        self.assertIn('--grippers', args)
        self.assertIn('--body-lift', args)
        self.assertEqual(args[args.index('--mode')+1], 'first_person')
        self.assertEqual(float(args[args.index('--left-gripper-max-open-percent')+1]), 70.)
        for forbidden in ('--host', '--user', '--chassis', '--left-gripper-sn'):
            self.assertNotIn(forbidden, args)
        self.assertEqual(self.process.stdin.getvalue(), '')
        self.credentials.assert_not_called()
        self.acquire.assert_not_called()
        self.release.assert_not_called()
        self.assertFalse(self.panel.chassis.get_sensitive())
        self.assertFalse(self.panel.runtime_target.get_sensitive())
        self.assertFalse(self.panel.ik_backend.get_sensitive())

    def test_demo_allows_all_ik_backends_and_guard_choices_before_launch(self):
        self.demo()
        for backend in ('native', 'vendor', 'tron2'):
            for guarded in (True, False):
                with self.subTest(backend=backend, guarded=guarded):
                    self.assertTrue(self.panel.ik_backend.get_sensitive())
                    self.panel.ik_backend.set_active_id(backend)
                    self.assertTrue(self.panel.wrist_controls.toggle.get_sensitive())
                    self.panel.wrist_controls.toggle.set_active(guarded)
                    self.launch.return_value = NS(stdin=io.StringIO(), poll=lambda: None)
                    self.panel.start()
                    args = self.launch.call_args.args[0]
                    self.assertEqual(args[args.index('--ik-backend')+1], backend)
                    self.assertEqual('--disable-wrist-guard' in args, not guarded)
                    self.assertEqual(self.panel.session_ik_backend, backend)
                    self.assertFalse(self.panel.ik_backend.get_sensitive())
                    self.credentials.assert_not_called()
                    self.acquire.assert_not_called()
                    self.assertEqual(self.panel.process.stdin.getvalue(), '')
                    self.panel.process.poll = lambda: 0
                    self.panel.tick()
                    self.assertFalse(self.panel.ik_backend.get_sensitive())  # exit notification pending
                    self.panel.events.put(('demo_exit', 0))
                    self.panel.tick()
                    self.assertTrue(self.panel.ik_backend.get_sensitive())

    def test_tron2_switching_from_demo_to_robot_retains_backend_and_restores_guard(self):
        self.assertTrue(self.panel.ik_backend.set_active_id('tron2'))
        self.assertEqual(self.panel.ik_backend.get_active_text(), 'TRON2 QP IK')
        self.demo()
        self.assertTrue(self.panel.ik_backend.set_active_id('tron2'))
        self.assertEqual(self.panel.ik_backend.get_active_text(), 'TRON2 QP IK（Demo）')
        self.panel.wrist_controls.toggle.set_active(False)
        self.panel.runtime_target.set_active_id('robot')
        self.assertEqual(self.panel.ik_backend.get_active_id(), 'tron2')
        self.assertEqual(self.panel.ik_backend.get_active_text(), 'TRON2 QP IK')
        self.assertTrue(self.panel.wrist_controls.toggle.get_active())
        self.assertFalse(self.panel.wrist_controls.toggle.get_sensitive())
        self.launch.assert_not_called()
        self.credentials.assert_not_called()
        self.acquire.assert_not_called()

    def test_forced_unknown_backend_is_rejected_before_credentials_or_launch(self):
        # Even a programmatically injected row must not pass the launch boundary.
        self.panel.ik_backend.append('unknown', 'forced unknown backend')
        self.panel.ik_backend.set_active_id('unknown')
        self.panel.start()
        self.launch.assert_not_called()
        self.credentials.assert_not_called()
        self.acquire.assert_not_called()
        buffer = self.panel.log.get_buffer()
        text = buffer.get_text(buffer.get_start_iter(), buffer.get_end_iter(), True)
        self.assertIn('请选择有效的 IK 解算器', text)

    def test_demo_session_keeps_tron2_until_exit_even_if_target_widget_is_changed(self):
        self.demo()
        self.panel.ik_backend.set_active_id('tron2')
        self.panel.start()
        self.panel.runtime_target.set_active_id('robot')
        self.assertEqual(self.panel.ik_backend.get_active_id(), 'tron2')
        self.assertEqual(self.panel.session_ik_backend, 'tron2')
        self.process.poll = lambda: 0
        self.panel.events.put(('demo_exit', 0))
        self.panel.tick()
        self.assertEqual(self.panel.ik_backend.get_active_id(), 'tron2')
        self.assertEqual(self.panel.ik_backend.get_active_text(), 'TRON2 QP IK')
        self.assertTrue(self.panel.wrist_controls.toggle.get_active())
        self.assertFalse(self.panel.wrist_controls.toggle.get_sensitive())
        self.credentials.assert_not_called()
        self.acquire.assert_not_called()

    def test_demo_preserves_selected_vendor_and_whole_console_demo_allows_choice(self):
        self.panel.ik_backend.set_active_id('vendor')
        self.demo()
        self.assertEqual(self.panel.ik_backend.get_active_id(), 'vendor')
        self.assertTrue(self.panel.ik_backend.get_sensitive())
        other = TeleopPanel(self.credentials, self.acquire, self.release, demo=True)
        self.addCleanup(other.destroy)
        self.assertTrue(other.ik_backend.get_sensitive())
        other.ik_backend.set_active_id('vendor')
        other.start()
        args = self.launch.call_args.args[0]
        self.assertEqual(args[args.index('--ik-backend')+1], 'vendor')
        self.assertEqual(other.session_ik_backend, 'vendor')

    def test_whole_console_demo_includes_tron2(self):
        other = TeleopPanel(self.credentials, self.acquire, self.release, demo=True)
        self.addCleanup(other.destroy)
        self.assertTrue(other.ik_backend.set_active_id('tron2'))
        other.start()
        args = self.launch.call_args.args[0]
        self.assertEqual(args[args.index('--ik-backend')+1], 'tron2')
        self.assertEqual(other.session_target, 'model')
        self.credentials.assert_not_called()
        self.acquire.assert_not_called()

    def test_demo_status_reports_actual_algorithm_and_independent_guard(self):
        self.demo()
        self.panel.start()
        cases = (
            ('native', True, '联合限位 IK'),
            ('demo_joint_box_only', False, '本地数值 IK（联合限位关闭）'),
            ('vendor', True, '原厂 IK + 腕部联合限位检查'),
            ('vendor', False, '原厂 IK（联合限位关闭）'),
            ('tron2', True, 'TRON2 QP IK + 腕部联合限位检查'),
            ('tron2', False, 'TRON2 QP IK（联合限位关闭）'),
        )
        for backend, enabled, label in cases:
            with self.subTest(backend=backend, enabled=enabled):
                self.frame(sample_state(ik_backend=backend, wrist_guard_enabled=enabled))
                self.assertIn('IK：'+label, self.panel.status.get_text())
        self.assertNotIn('450 Hz', self.panel.ik_backend.get_tooltip_text())
        self.frame(sample_state(ik_backend='vendor', wrist_limited=True))
        self.assertIn('联合限位已缩小目标', self.panel.status.get_text())
        self.assertNotIn('完整手柄目标仍保留', self.panel.status.get_text())

    def test_running_vendor_demo_remains_vendor_while_guard_changes(self):
        self.demo()
        self.panel.ik_backend.set_active_id('vendor')
        self.panel.start()
        self.panel.wrist_controls.toggle.set_active(False)
        self.assertIn('guard_off', self.process.stdin.getvalue())
        self.frame(sample_state(ik_backend='vendor', ik_requested_backend='vendor',
                                wrist_guard_enabled=False))
        self.assertEqual(self.panel.session_ik_backend, 'vendor')
        self.assertEqual(self.panel.ik_backend.get_active_id(), 'vendor')
        self.assertTrue(self.panel.wrist_controls.toggle.get_sensitive())
        self.assertFalse(self.panel.ik_backend.get_sensitive())
        self.assertIn('原厂 IK（联合限位关闭）', self.panel.status.get_text())

    def test_demo_receiver_service_is_available_without_robot_credentials(self):
        self.demo()
        self.panel.start_service()
        self.assertEqual(self.launch.call_args.args[0][-2:], ['pico', 'service'])
        self.credentials.assert_not_called()
        self.acquire.assert_not_called()

    def test_valid_unconfirmed_tool_can_be_simulated_but_cannot_start_real_robot(self):
        from apps.fairino_tcp_probe.tool_frames import load_geometry
        geometry = load_geometry()
        for side, angle in (('left', 90.), ('right', -90.)):
            geometry['arms'][side].update(installed=True, confirmed=False, roll_deg=angle)
        with patch('apps.workbench.teleop.load_geometry', return_value=geometry):
            self.demo()
            self.panel.tcp_tool.set_active_id('taccap')
            for backend in ('native', 'vendor'):
                with self.subTest(backend=backend):
                    self.panel.ik_backend.set_active_id(backend)
                    self.launch.return_value = NS(stdin=io.StringIO(), poll=lambda: None)
                    self.panel.start()
                    args = self.launch.call_args.args[0]
                    self.assertEqual(args[args.index('--tcp-tool')+1], 'taccap')
                    self.assertEqual(self.panel.session_ik_backend, backend)
                    self.credentials.assert_not_called()
                    self.acquire.assert_not_called()
                    self.panel.process.poll = lambda: 0
                    self.panel.events.put(('demo_exit', 0))
                    self.panel.tick()
            self.panel.runtime_target.set_active_id('robot')
            self.launch.reset_mock()
            for backend in ('native', 'vendor'):
                with self.subTest(real_backend=backend):
                    self.panel.ik_backend.set_active_id(backend)
                    self.panel.start()
                    self.launch.assert_not_called()
                    self.credentials.assert_not_called()
                    self.acquire.assert_not_called()

    def test_demo_still_rejects_invalid_tool_geometry(self):
        from apps.fairino_tcp_probe.tool_frames import load_geometry
        geometry = load_geometry()
        geometry['arms']['left'].update(confirmed=False, roll_deg=math.nan)
        with patch('apps.workbench.teleop.load_geometry', return_value=geometry):
            self.demo()
            self.panel.tcp_tool.set_active_id('taccap')
            self.panel.start()
        self.launch.assert_not_called()
        self.credentials.assert_not_called()
        self.acquire.assert_not_called()

    def test_mode_change_and_second_start_cannot_replace_active_session(self):
        self.demo()
        self.panel.start()
        self.panel.runtime_target.set_active_id('robot')
        self.assertTrue(self.panel.model_demo_selected())
        self.panel.start()
        self.assertEqual(self.launch.call_count, 1)
        self.credentials.assert_not_called()

    def test_model_guard_toggle_is_optional_and_real_native_is_always_protected(self):
        self.demo()
        self.assertTrue(self.panel.wrist_controls.toggle.get_sensitive())
        self.panel.wrist_controls.toggle.set_active(False)
        self.panel.start()
        self.assertIn('--disable-wrist-guard', self.launch.call_args.args[0])
        self.panel.wrist_controls.toggle.set_active(True)
        self.assertIn('guard_on', self.process.stdin.getvalue())
        self.frame(sample_state(wrist_guard_enabled=False,
                                wrist_collision={'left': True, 'right': False}))
        self.assertFalse(self.panel.wrist_controls.toggle.get_active())
        self.assertEqual(self.process.stdin.getvalue().count('guard_on'), 1)
        self.assertNotIn('guard_off', self.process.stdin.getvalue())
        self.process.poll = lambda: 0
        self.panel.events.put(('demo_exit', 0))
        self.panel.tick()
        self.panel.runtime_target.set_active_id('robot')
        self.assertTrue(self.panel.wrist_controls.toggle.get_active())
        self.assertFalse(self.panel.wrist_controls.toggle.get_sensitive())

    def test_only_fresh_model_feedback_is_exposed_and_exit_holds_last_pose(self):
        self.demo()
        self.panel.start()
        state = sample_state(wrist_collision={'left': True, 'right': False})
        state['hardware']['position'][0] = .1
        self.frame(state)
        snapshot = self.panel.model_snapshot()
        self.assertTrue(snapshot['fresh'])
        self.assertTrue(snapshot['active'])
        self.assertEqual(snapshot['hardware']['position'][0], .1)
        self.assertTrue(snapshot['wrist_collision']['left'])
        self.assertIn('模拟关节', self.panel.status.get_text())
        self.panel.demo_received = time.monotonic()-1.
        self.assertIsNone(self.panel.model_snapshot()['hardware'])
        self.assertEqual(self.panel.model_snapshot()['grippers'], {})
        self.process.poll = lambda: 0
        self.panel.events.put(('demo_exit', 0))
        self.panel.tick()
        snapshot = self.panel.model_snapshot()
        self.assertFalse(snapshot['active'])
        self.assertFalse(snapshot['fresh'])
        self.assertTrue(snapshot['wrist_collision']['left'])
        self.assertEqual(self.panel.demo_state['hardware']['position'][0], .1)
        self.release.assert_not_called()

    def test_invalid_demo_feedback_cannot_replace_valid_pose(self):
        self.demo()
        self.panel.start()
        self.frame()
        previous = self.panel.demo_state
        bad = []
        missing_joint = sample_state()
        missing_joint['hardware']['joint_names'][0] = 'j999'
        bad.append(missing_joint)
        nan = sample_state()
        nan['hardware']['position'][0] = math.nan
        bad.append(nan)
        bad.extend((sample_state(grippers={'left': 1.1}),
                    sample_state(wrist_collision={'left': 'yes'}),
                    sample_state(sensitivity=math.inf)))
        for state in bad:
            with self.subTest(state=state):
                self.frame(state)
                self.assertIs(self.panel.demo_state, previous)

    def test_real_status_never_overwrites_active_demo_and_stop_has_no_disable_claim(self):
        self.demo()
        self.panel.start()
        self.frame()
        before = self.panel.status.get_text()
        self.panel.events.put(('vr_exit', 0))
        self.panel.events.put(('vr', 'WORKBENCH_VR '+json.dumps(dict(phase='实机'))))
        self.panel.tick()
        self.assertEqual(self.panel.status.get_text(), before)
        self.assertTrue(self.panel.model_demo_selected())
        self.panel.command('stop')
        self.assertEqual(json.loads(self.process.stdin.getvalue()), {'command': 'stop'})
        self.assertNotIn('下使能', self.panel.status.get_text())

    def test_exit_queue_must_be_drained_before_restart(self):
        self.demo()
        self.panel.start()
        self.process.poll = lambda: 0
        self.panel.start()
        self.assertEqual(self.launch.call_count, 1)
        self.panel.events.put(('demo_exit', 0))
        self.panel.tick()
        self.panel.runtime_target.set_active_id('robot')
        self.assertIsNone(self.panel.model_snapshot())
        self.launch.return_value = NS(stdin=io.StringIO(), poll=lambda: None)
        self.panel.start()
        self.assertEqual(self.launch.call_count, 2)
        self.credentials.assert_called_once_with('upper')
        self.acquire.assert_called_once_with('upper', '遥操')


class ModelDemoColorTests(unittest.TestCase):
    def setUp(self):
        self.view = NS(demo=True, fresh=True, wrist_collision={},
                       tint=NS(get_active=lambda: False), gripper_fresh=set())

    def color(self, link, **kwargs):
        visual = NS(link=link, tool_part='', color=(.9, .9, .9), side='')
        visual.__dict__.update(kwargs)
        return ModelView.visual_color(self.view, visual)

    def test_demo_arms_are_light_green_even_without_group_tint_or_new_input(self):
        for fresh in (True, False):
            self.view.fresh = fresh
            for link in ('left_shoulder_Link', 'right_upperarm1_Link', 'left_waist1_Link'):
                self.assertEqual(self.color(link), DEMO_ARM_COLOR)

    def test_only_crossed_wrist_centers_turn_red_and_restore_when_clear(self):
        self.view.wrist_collision = {'left': True, 'right': False}
        for link in ('left_waist1_Link', 'left_waist2_Link'):
            self.assertEqual(self.color(link), DEMO_COLLISION_COLOR)
        self.assertEqual(self.color('left_forearm1_Link'), DEMO_ARM_COLOR)
        self.assertEqual(self.color('right_waist1_Link'), DEMO_ARM_COLOR)
        self.view.wrist_collision['left'] = False
        self.assertEqual(self.color('left_waist1_Link'), DEMO_ARM_COLOR)

    def test_demo_flags_do_not_recolor_real_model_or_tool_tcp(self):
        self.view.wrist_collision = {'left': True}
        self.view.demo = False
        self.assertEqual(self.color('left_waist1_Link'), (.9, .9, .9))
        self.view.demo = True
        self.assertEqual(self.color('left_waist1_Link', tool_part='tcp'), (.9, .9, .9))

    def test_switching_feedback_source_clears_previous_pose(self):
        view = NS(demo=False, positions={'j1': .4}, gripper_positions={'left': .2},
                  received_pose=True, wrist_collision={'left': True},
                  update_tool_note=Mock(), render_error=None,
                  title=Mock(), area=Mock())
        ModelView.update(view, None, demo=True)
        self.assertEqual(view.positions, {})
        self.assertFalse(view.received_pose)
        self.assertEqual(view.wrist_collision, {})
        self.assertIn('模型 Demo', view.title.set_text.call_args.args[0])
        view.positions = {'j1': .7}
        ModelView.update(view, None, demo=False)
        self.assertEqual(view.positions, {})
        self.assertNotIn('模型 Demo', view.title.set_text.call_args.args[0])


if __name__ == '__main__':
    unittest.main()
