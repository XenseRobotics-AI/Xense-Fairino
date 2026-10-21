"""Exit causes survive cleanup; tests never open SSH or enable hardware."""
from contextlib import ExitStack
import io
import json
from pathlib import Path
import signal
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import paths
from upper_keyboard import Connection
from vr_client import main
from vr_gui_ipc import GuiInput
from vr_exit import ExitLog, connection_state


class ExitTests(unittest.TestCase):
    def run_client(self, outcome=None, gui_reason=None, finish_error=None, grippers=False,
                   keyboard_stop=False, hardware_ready=True, closed_output=False,
                   chassis=False, chassis_confirmed=True, chassis_start_error=None,
                   chassis_close_error=None, chassis_finish_error=None, chassis_state=None,
                   mode='--execute'):
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            root = Path(directory)
            peer = NS(latest={'disabled_confirmed': True, 'hardware': {'enabled': [False]*20}},
                      last_status=time.monotonic(), log_path=root/'remote.jsonl')
            if not hardware_ready:
                peer.latest = {'hardware': None, 'ready': False}
                peer.last_status = 0.
            stream = Mock()
            lifecycle = []
            stream.close.side_effect = lambda: lifecycle.append('stream.close')
            gripper = Mock()
            gripper.start.return_value = gripper
            gripper.finish.return_value = True
            gripper_factory = Mock(return_value=gripper)
            base = Mock()
            def chassis_start():
                lifecycle.append('chassis.start')
                if chassis_start_error:
                    raise chassis_start_error
                return base
            def chassis_close():
                lifecycle.append('chassis.close')
                if chassis_close_error:
                    raise chassis_close_error
            def chassis_finish():
                lifecycle.append('chassis.finish')
                if chassis_finish_error:
                    raise chassis_finish_error
                return chassis_confirmed
            base.start.side_effect = chassis_start
            base.close.side_effect = chassis_close
            base.finish.side_effect = chassis_finish
            def chassis_snapshot():
                lifecycle.append('chassis.snapshot')
                return (dict(disabled_confirmed=chassis_confirmed,
                             cleanup_confirmed=chassis_confirmed, error='')
                        if chassis_state is None else dict(chassis_state))
            base.snapshot.side_effect = chassis_snapshot
            chassis_factory = Mock(return_value=base)
            before_cleanup = []
            def records():
                return [json.loads(line) for path in root.rglob('*.exit.jsonl')
                        for line in path.read_text().splitlines()]
            def finish():
                lifecycle.append('upper.finish')
                before_cleanup.extend(records())
                if finish_error:
                    raise finish_error
            peer.finish = Mock(side_effect=finish)
            def operate(args, *_):
                if chassis and mode == '--execute':
                    self.assertIs(args.chassis_control, base)
                if keyboard_stop:
                    args.exit_reason = 'keyboard_stop'
                if outcome is not None:
                    raise outcome
                return 0
            stdout, stderr = io.StringIO(), io.StringIO()
            if closed_output:
                stdout = Mock()
                stderr = Mock()
                stdout.write.side_effect = BrokenPipeError('stdout closed')
                stderr.write.side_effect = BrokenPipeError('stderr closed')
            for target, value in (
                ('vr_client.ROOT', root), ('vr_client.VendorKinematics', Mock()),
                ('vr_client.Connection', Mock(return_value=peer)),
                ('vr_client.Stream', Mock(return_value=stream)),
                ('vr_client.wait_for_input', Mock()),
                ('vr_client.operate', Mock(side_effect=operate)),
                ('vr_link_check.run_client', Mock(side_effect=operate)),
                ('vr_client.getpass.getpass', Mock(return_value='test-password-not-for-logs')),
                ('vr_grippers.GripperTeleop', gripper_factory),
                ('vr_chassis.ChassisTeleop', chassis_factory),
                ('sys.stdout', stdout), ('sys.stderr', stderr)):
                stack.enter_context(patch(target, value))
            argv = [mode, '--no-auto-home']
            if grippers:
                argv.append('--grippers')
            if chassis:
                argv.append('--chassis')
            if gui_reason:
                gui = NS(exit_reason=gui_reason, exit_monotonic=123.,
                         authenticate=Mock(return_value='test-password-not-for-logs'))
                stack.enter_context(patch('vr_gui_ipc.GuiInput', return_value=gui))
                argv.append('--gui-stdio')
            previous = signal.getsignal(signal.SIGTERM)
            code = main(argv)
            self.assertIs(signal.getsignal(signal.SIGTERM), previous)
            result = records()
            self.assertNotIn('test-password-not-for-logs', json.dumps(result))
            self.assertEqual(before_cleanup[-1]['event'], 'stopping')
            self.assertTrue(all('client_monotonic' in row for row in result))
            peer.finish.assert_called_once()
            if mode == '--execute':
                stream.open.assert_called_once()
                stream.close.assert_called_once()
            else:
                stream.open.assert_not_called()
                stream.close.assert_not_called()
            if grippers and mode == '--execute':
                gripper.finish.assert_called_once()
            else:
                gripper_factory.assert_not_called()
            if chassis and mode == '--execute':
                chassis_factory.assert_called_once()
                base.start.assert_called_once()
                base.close.assert_called_once()
                base.finish.assert_called_once()
                base.snapshot.assert_called_once()
                self.assertLess(lifecycle.index('chassis.close'), lifecycle.index('upper.finish'))
                self.assertLess(lifecycle.index('chassis.finish'), lifecycle.index('chassis.snapshot'))
                self.assertLess(lifecycle.index('chassis.finish'), lifecycle.index('stream.close'))
            else:
                chassis_factory.assert_not_called()
            return code, result

    def test_normal_completion_and_keyboard_stop_are_logged(self):
        for keyboard_stop in (False, True):
            code, rows = self.run_client(keyboard_stop=keyboard_stop)
            self.assertEqual(code, 0)
            self.assertEqual(rows[-2]['reason'], 'keyboard_stop' if keyboard_stop else 'completed')
            self.assertTrue(rows[-1]['upper_disabled_confirmed'])

    def test_chassis_requires_explicit_opt_in_and_stops_before_upper_cleanup(self):
        for enabled in (False, True):
            code, rows = self.run_client(chassis=enabled)
            self.assertEqual(code, 0)
            self.assertEqual(rows[0]['chassis_requested'], enabled)
            self.assertEqual(rows[-1]['chassis_disabled_confirmed'], True if enabled else None)

    def test_readonly_prepare_and_link_checks_never_create_chassis_or_gripper_sessions(self):
        for mode in ('--check', '--prepare-check', '--link-check'):
            with self.subTest(mode=mode):
                code, rows = self.run_client(chassis=True, grippers=True, mode=mode)
                self.assertEqual(code, 0)
                self.assertFalse(rows[0]['execute'])
                self.assertIsNone(rows[-1]['upper_disabled_confirmed'])
                self.assertIsNone(rows[-1]['chassis_disabled_confirmed'])
                self.assertIsNone(rows[-1]['grippers_disabled_confirmed'])

    def test_chassis_cleanup_survives_control_fault_interrupt_and_upper_cleanup_failure(self):
        for outcome, upper_error, expected in (
            (RuntimeError('control fault'), None, 'exception'),
            (KeyboardInterrupt(), None, 'keyboard_interrupt'),
            (RuntimeError('original control fault'), OSError('upper cleanup failed'), 'exception')):
            with self.subTest(reason=expected, cleanup_error=upper_error):
                code, rows = self.run_client(outcome, chassis=True, grippers=True,
                                             finish_error=upper_error, closed_output=True)
                self.assertEqual(rows[-2]['reason'], expected)
                self.assertTrue(rows[-1]['chassis_disabled_confirmed'])
                self.assertTrue(rows[-1]['grippers_disabled_confirmed'])
                self.assertEqual(code, 0 if expected == 'keyboard_interrupt' else 1)

    def test_unconfirmed_chassis_shutdown_marks_whole_session_failed(self):
        code, rows = self.run_client(chassis=True, chassis_confirmed=False, grippers=True)
        self.assertEqual(code, 1)
        self.assertFalse(rows[-1]['chassis_disabled_confirmed'])
        self.assertTrue(rows[-1]['upper_disabled_confirmed'])
        self.assertTrue(rows[-1]['grippers_disabled_confirmed'])
        self.assertEqual(rows[-1]['return_code'], 1)

    def test_chassis_disabled_and_cleanup_confirmation_are_logged_separately(self):
        state = dict(disabled_confirmed=True, cleanup_confirmed=False, enabled=0,
                     error='', phase='底盘已下使能，里程计清理未确认')
        code, rows = self.run_client(chassis=True, chassis_confirmed=False, chassis_state=state)
        self.assertEqual(code, 1)
        self.assertTrue(rows[-1]['chassis_disabled_confirmed'])
        self.assertFalse(rows[-1]['chassis_cleanup_confirmed'])
        self.assertEqual(rows[-1]['chassis'], state)

    def test_chassis_fault_after_final_control_frame_cannot_be_reported_as_success(self):
        state = dict(disabled_confirmed=True, cleanup_confirmed=True, enabled=0,
                     error='最后控制帧之后检测到底盘反馈中断', phase='已退出并下使能')
        code, rows = self.run_client(chassis=True, chassis_confirmed=True, chassis_state=state)
        self.assertEqual(rows[-2]['reason'], 'completed')  # No exception in the control loop.
        self.assertEqual(code, 1)
        self.assertEqual(rows[-1]['return_code'], 1)
        self.assertTrue(rows[-1]['chassis_disabled_confirmed'])
        self.assertTrue(rows[-1]['chassis_cleanup_confirmed'])
        self.assertEqual(rows[-1]['chassis']['error'], state['error'])

    def test_chassis_start_or_close_failure_still_finishes_all_channels(self):
        for stage in ('start', 'close', 'finish'):
            with self.subTest(stage=stage):
                code, rows = self.run_client(chassis=True, grippers=True,
                    **{'chassis_'+stage+'_error': OSError('chassis '+stage+' failed')})
                self.assertEqual(code, 1)
                self.assertTrue(rows[-1]['grippers_disabled_confirmed'])
                if stage == 'start':
                    self.assertEqual(rows[-2]['error'], 'chassis start failed')
                else:
                    key = 'chassis_stop' if stage == 'close' else 'chassis_finish'
                    self.assertEqual(rows[-1]['cleanup_errors'][key]['error'], 'chassis '+stage+' failed')

    def test_exception_is_written_before_cleanup_and_keeps_first_cause(self):
        code, rows = self.run_client(RuntimeError('original feedback failure'),
                                     finish_error=OSError('cleanup failed'), grippers=True)
        self.assertEqual(code, 1)
        self.assertEqual(rows[-2]['error'], 'original feedback failure')
        self.assertEqual(rows[-2]['error_type'], 'RuntimeError')
        self.assertEqual(rows[-1]['first_reason'], 'exception')
        self.assertEqual(rows[-1]['cleanup_errors']['upper_connection']['error'], 'cleanup failed')
        self.assertTrue(rows[-1]['grippers_disabled_confirmed'])

    def test_keyboard_interrupt_gui_stop_and_pipe_close_are_distinct(self):
        for gui_reason in (None, 'gui_stop', 'parent_pipe_closed'):
            code, rows = self.run_client(KeyboardInterrupt(), gui_reason=gui_reason)
            self.assertEqual(code, 0)
            self.assertEqual(rows[-2]['reason'], gui_reason or 'keyboard_interrupt')

    def test_closed_output_cannot_skip_any_channel_cleanup_or_hide_first_cause(self):
        for outcome, expected in ((None, 'completed'), (KeyboardInterrupt(), 'keyboard_interrupt'),
                                  (RuntimeError('first failure'), 'exception')):
            with self.subTest(reason=expected):
                code, rows = self.run_client(outcome, grippers=True, closed_output=True)
                self.assertEqual(rows[-2]['reason'], expected)
                self.assertTrue(rows[-1]['grippers_disabled_confirmed'])
                self.assertEqual(code, int(outcome is not None and expected == 'exception'))
        code, rows = self.run_client(RuntimeError('first failure'), grippers=True,
                                     closed_output=True, finish_error=OSError('cleanup failure'))
        self.assertEqual(code, 1)
        self.assertEqual(rows[-2]['error'], 'first failure')
        self.assertEqual(rows[-1]['cleanup_errors']['upper_connection']['error'], 'cleanup failure')

    def test_missing_first_hardware_frame_does_not_block_cleanup(self):
        code, rows = self.run_client(RuntimeError('first frame not ready'), hardware_ready=False)
        self.assertEqual(code, 1)
        self.assertEqual(rows[-2]['error'], 'first frame not ready')
        self.assertIsNone(rows[-2]['connection']['enabled_count'])
        self.assertIsNone(rows[-2]['connection']['feedback_age_s'])
        self.assertFalse(rows[-1]['upper_disabled_confirmed'])

    def test_incomplete_or_invalid_diagnostic_state_never_raises(self):
        for latest in ({}, {'hardware': None}, {'hardware': {}},
                       {'hardware': {'enabled': None}}, {'hardware': []}, None):
            with self.subTest(latest=latest):
                peer = NS(latest=latest, last_status=0., log_path=Path('remote.jsonl'))
                state = connection_state(peer)
                self.assertIsNone(state['enabled_count'])
                json.dumps(state, allow_nan=False)
        peer = NS(latest={'stamp': float('nan')}, last_status=float('inf'), log_path=Path('r.jsonl'))
        json.dumps(connection_state(peer), allow_nan=False)
        self.assertEqual(connection_state(object()), {'snapshot_error': 'AttributeError'})

    def test_log_directory_or_output_failure_is_nonfatal(self):
        with tempfile.TemporaryDirectory() as directory:
            blocked = Path(directory)/'file'
            blocked.write_text('not a directory')
            log = ExitLog(blocked)  # Initializing evidence has no filesystem side effects.
            output = Mock()
            output.write.side_effect = BrokenPipeError('closed output')
            with patch('sys.stderr', output):
                log.write('stopping', reason='original failure')

    def test_gui_input_marks_the_reason_before_signalling_main(self):
        for source, reason in (
            ('{"command":"stop"}\n', 'gui_stop'), ('', 'parent_pipe_closed'),
            ('{"command":"bad"}\n', 'gui_input_invalid'), ('broken\n', 'gui_input_invalid')):
            with self.subTest(reason=reason):
                gui = GuiInput(io.StringIO(source))
                def signal_main(*_):
                    self.assertEqual(gui.exit_reason, reason)
                    self.assertIsInstance(gui.exit_monotonic, float)
                with patch('vr_gui_ipc.os.kill', side_effect=signal_main) as kill:
                    gui.read()
                kill.assert_called_once()

    def test_transport_identifies_local_feedback_timeout_and_explicit_exit(self):
        peer = object.__new__(Connection)
        peer.done, peer.write_lock = threading.Event(), threading.Lock()
        peer.latest, peer.last_status, peer.feedback_timeout = {'stamp': 1.}, 1., .5
        peer.wrist, peer.child = None, Mock()
        for explicit, expected in ((False, 'feedback_timeout'), (True, 'client_exit')):
            with patch('upper_keyboard.time.monotonic', return_value=2.):
                peer.send(exit=explicit)
            packet = json.loads(peer.child.sendline.call_args.args[0])
            self.assertTrue(packet['exit'])
            self.assertEqual(packet['exit_reason'], expected)
        peer.last_status = 1.9
        with patch('upper_keyboard.time.monotonic', return_value=2.):
            peer.send()
        self.assertNotIn('exit_reason', json.loads(peer.child.sendline.call_args.args[0]))


if __name__ == '__main__':
    unittest.main()
