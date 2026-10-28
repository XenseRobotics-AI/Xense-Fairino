"""Backend defaults and startup ordering; every I/O entry point is mocked."""
import contextlib
import io
from pathlib import Path
import queue
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import paths
from test_tcp import hardware
from vendor_kinematics import VendorKinematics
from vr_client import main, operate
from vr_timing import Timing


class NativeLaunchTests(unittest.TestCase):
    def launch(self, argv, build_error=None):
        calls, captured = [], {}
        peer = Mock(latest={'disabled_confirmed': True})
        stream = Mock()
        stream.open.side_effect = lambda: calls.append('pico_open')
        def build(**_):
            calls.append('native_load')
            if build_error:
                raise build_error
            return Mock()
        def connect(args, **_):
            calls.append('robot_connect')
            captured['timing'] = Timing.from_args(args)
            captured['args'] = args
            return peer
        with tempfile.TemporaryDirectory() as directory, contextlib.ExitStack() as stack:
            stack.enter_context(patch('vr_client.ROOT', Path(directory)))
            stack.enter_context(patch('vr_client.VendorKinematics', Mock()))
            native = stack.enter_context(patch('vr_native_ik.NativeIK', side_effect=build))
            connection = stack.enter_context(patch('vr_client.Connection', side_effect=connect))
            stack.enter_context(patch('vr_client.Stream', return_value=stream))
            stack.enter_context(patch('vr_client.wait_for_input'))
            stack.enter_context(patch('vr_client.operate', return_value=0))
            stack.enter_context(patch('vr_link_check.run_client', return_value=0))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(io.StringIO()))
            captured['returncode'] = main(argv)
        return captured, calls, native, connection, stream

    def test_default_native_compiles_before_input_and_robot_connection(self):
        captured, calls, native, _, _ = self.launch(['--execute', '--no-auto-home'])
        self.assertEqual(captured['returncode'], 0)
        self.assertEqual(calls, ['native_load', 'pico_open', 'robot_connect'])
        self.assertEqual(captured['args'].ik_backend, 'native')
        self.assertEqual(captured['args'].sensitivity, .25)
        self.assertEqual(captured['timing'], Timing(100., 200., 100., 0., 12.))
        native.assert_called_once()

    def test_vendor_fallback_never_loads_native_and_keeps_console_rate(self):
        captured, calls, native, _, _ = self.launch(['--execute', '--ik-backend', 'vendor'])
        native.assert_not_called()
        self.assertEqual(calls, ['pico_open', 'robot_connect'])
        self.assertEqual(captured['timing'], Timing(450., 500., 100., 0., 12.))

    def test_explicit_rates_and_alias_override_backend_defaults(self):
        for backend in ('native', 'vendor'):
            with self.subTest(backend=backend):
                captured, _, _, _, _ = self.launch(['--check', '--ik-backend', backend,
                    '--interpolation-hz', '80', '--receiver-hz', '180', '--feedback-hz', '50'])
                self.assertEqual(captured['timing'], Timing(80., 180., 50., 0., 12.))

    def test_readonly_modes_do_not_load_native_or_open_input(self):
        for mode in ('--check', '--prepare-check', '--link-check'):
            with self.subTest(mode=mode):
                captured, calls, native, _, stream = self.launch([mode])
                self.assertEqual(captured['returncode'], 0)
                self.assertEqual(calls, ['robot_connect'])
                native.assert_not_called()
                stream.open.assert_not_called()

    def test_failed_native_build_stops_before_any_connection_and_no_fallback(self):
        captured, calls, native, connection, stream = self.launch(
            ['--execute'], RuntimeError('test compiler failure'))
        self.assertEqual(captured['returncode'], 1)
        self.assertEqual(calls, ['native_load'])
        native.assert_called_once()
        connection.assert_not_called()
        stream.open.assert_not_called()

    def test_native_guard_off_rejected_before_loading_or_connecting(self):
        with patch('vr_client.Connection') as connection, patch('vr_native_ik.NativeIK') as native, \
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as result:
                main(['--execute', '--no-wrist-guard'])
            self.assertEqual(result.exception.code, 2)
            connection.assert_not_called()
            native.assert_not_called()
        captured, _, _, _, _ = self.launch(['--execute', '--ik-backend', 'vendor', '--no-wrist-guard'])
        self.assertEqual(captured['returncode'], 0)
        self.assertTrue(captured['args'].no_wrist_guard)

    def test_real_readonly_loop_does_not_require_a_native_planner(self):
        with tempfile.TemporaryDirectory() as directory:
            done = threading.Event()
            done.set()
            events = queue.Queue()
            events.put({'checked': True, 'hardware': hardware()})
            connection = NS(events=events, done=done, log_path=Path(directory)/'check.jsonl')
            args = NS(execute=False, sensitivity=.25, ik_backend='native')
            with patch('vr_client.keyboard', contextlib.nullcontext), \
                    patch('vr_client.verify_both', return_value=({}, {})), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(operate(args, VendorKinematics(), None, connection), 0)


if __name__ == '__main__':
    unittest.main()
