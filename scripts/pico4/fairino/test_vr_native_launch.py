"""Backend defaults and startup ordering; every I/O entry point is mocked."""
import contextlib
import ast
import base64
import io
import json
from pathlib import Path
import queue
import shlex
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
import zlib
from unittest.mock import Mock, patch

import paths
from test_tcp import hardware
from vendor_kinematics import VendorKinematics
from vr_client import configure_ik, main, operate, remote_command
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
        def build_tron(**_):
            calls.append('tron2_load')
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
            tron = stack.enter_context(patch('vr_tron_ik.TronIK', side_effect=build_tron))
            connection = stack.enter_context(patch('vr_client.Connection', side_effect=connect))
            stack.enter_context(patch('vr_client.Stream', return_value=stream))
            stack.enter_context(patch('vr_client.wait_for_input'))
            stack.enter_context(patch('vr_client.operate', return_value=0))
            stack.enter_context(patch('vr_link_check.run_client', return_value=0))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(io.StringIO()))
            captured['returncode'] = main(argv)
            captured['tron_load_count'] = tron.call_count
        return captured, calls, native, connection, stream

    def test_default_native_compiles_before_input_and_robot_connection(self):
        captured, calls, native, _, _ = self.launch(['--execute', '--no-auto-home'])
        self.assertEqual(captured['returncode'], 0)
        self.assertEqual(calls, ['native_load', 'pico_open', 'robot_connect'])
        self.assertEqual(captured['args'].ik_backend, 'native')
        self.assertEqual(captured['args'].sensitivity, .25)
        self.assertEqual(captured['timing'], Timing(500., 500., 100., 0., 12.))
        native.assert_called_once()

    def test_vendor_fallback_never_loads_native_and_keeps_console_rate(self):
        captured, calls, native, _, _ = self.launch(['--execute', '--ik-backend', 'vendor'])
        native.assert_not_called()
        self.assertEqual(calls, ['pico_open', 'robot_connect'])
        self.assertEqual(captured['timing'], Timing(450., 500., 100., 0., 12.))

    def test_held_speed_cli_defaults_on_and_can_disable_for_each_backend(self):
        for backend in ('native', 'vendor', 'tron2'):
            for disabled in (False, True):
                with self.subTest(backend=backend, disabled=disabled):
                    argv = ['--execute', '--ik-backend', backend]
                    if disabled:
                        argv.append('--no-held-speed-check')
                    captured, *_ = self.launch(argv)
                    self.assertEqual(captured['returncode'], 0)
                    self.assertEqual(captured['args'].no_held_speed_check, disabled)
                    self.assertFalse(captured['args'].no_wrist_guard)

    def test_tron_loads_before_input_and_robot_connection(self):
        captured, calls, native, _, _ = self.launch(['--execute', '--ik-backend', 'tron2'])
        self.assertEqual(captured['returncode'], 0)
        self.assertEqual(calls, ['tron2_load', 'pico_open', 'robot_connect'])
        self.assertEqual(captured['args'].ik_backend, 'tron2')
        self.assertEqual(captured['tron_load_count'], 1)
        self.assertEqual(captured['timing'], Timing(500., 500., 100., 0., 12.))
        self.assertIsNotNone(captured['args'].tron_ik)
        native.assert_not_called()

    def test_all_backends_keep_tcp_buffer_and_no_receiver_joint_interpolation(self):
        for backend in ('native', 'vendor', 'tron2'):
            for rate in (100., 80.):
                with self.subTest(backend=backend, rate=rate):
                    args = NS(ik_backend=backend, execute=True, command_hz=rate,
                              receiver_hz=500., feedback_hz=100.)
                    command = remote_command(args)
                    setup = shlex.split(command)[2]
                    loader = shlex.split(setup.split('exec python3 -u -c ', 1)[1])[0]
                    tree = ast.parse(loader)
                    argv = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                                and ast.unparse(n.targets[0]) == 'sys.argv')
                    self.assertNotIn('--tron2-interpolation-period', argv)
                    self.assertEqual(float(argv[argv.index('--command-hz')+1]), rate)
                    self.assertEqual(float(argv[argv.index('--interpolation-delay-ms')+1]), 12.)
                    self.assertEqual(float(argv[argv.index('--horizon-ms')+1]), 0.)
                    encoded = next(n.args[0].value for n in ast.walk(tree) if isinstance(n, ast.Call)
                                   and ast.unparse(n.func) == 'base64.b64decode')
                    bundle = json.loads(zlib.decompress(base64.b64decode(encoded)))
                    self.assertNotIn('vr_interpolation', bundle)
                    self.assertEqual(bundle['vr_robot'], Path(__file__).with_name('vr_robot.py').read_text())

    def test_tron_missing_dependency_never_opens_input_or_robot_or_falls_back(self):
        captured, calls, native, connection, stream = self.launch(
            ['--execute', '--ik-backend', 'tron2'], RuntimeError('TRON2 dependency missing'))
        self.assertEqual(captured['returncode'], 1)
        self.assertEqual(calls, ['tron2_load'])
        self.assertEqual(captured['tron_load_count'], 1)
        native.assert_not_called()
        connection.assert_not_called()
        stream.open.assert_not_called()

    def test_tron_readonly_checks_do_not_load_qp_or_open_input(self):
        for mode in ('--check', '--prepare-check', '--link-check'):
            with self.subTest(mode=mode):
                captured, calls, native, _, stream = self.launch([mode, '--ik-backend', 'tron2'])
                self.assertEqual(captured['returncode'], 0)
                self.assertEqual(calls, ['robot_connect'])
                self.assertEqual(captured['tron_load_count'], 0)
                native.assert_not_called()
                stream.open.assert_not_called()

    def test_tron_guard_off_fails_before_any_initialization(self):
        with patch('vr_client.Connection') as connection, patch('vr_tron_ik.TronIK') as tron, \
             patch('vr_client.Stream') as stream, patch('vr_client.VendorKinematics') as vendor, \
             patch('vr_client.ExitLog') as log, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as result:
                main(['--execute', '--ik-backend', 'tron2', '--no-wrist-guard'])
            self.assertEqual(result.exception.code, 2)
            for entry in (connection, tron, stream, vendor, log):
                entry.assert_not_called()

    def test_explicit_rates_and_alias_override_backend_defaults(self):
        for backend in ('native', 'vendor', 'tron2'):
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

    def test_unknown_backend_rejected_in_every_robot_cli_mode_before_any_io(self):
        for mode in ('--execute', '--check', '--prepare-check', '--link-check'):
            with self.subTest(mode=mode), patch('vr_client.Connection') as connection, \
                    patch('vr_client.Stream') as stream, patch('vr_client.VendorKinematics') as vendor, \
                    patch('vr_native_ik.NativeIK') as native, patch('vr_client.ExitLog') as log, \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as result:
                    main([mode, '--ik-backend', 'demo_joint_box_only'])
                self.assertEqual(result.exception.code, 2)
                for entry in (connection, stream, vendor, native, log):
                    entry.assert_not_called()

    def test_direct_robot_entry_points_do_not_silently_fall_back_to_vendor(self):
        for backend in ('demo_joint_box_only', 'unknown', None):
            for entry in (configure_ik, remote_command,
                          lambda args: operate(args, None, None, None)):
                with self.subTest(backend=backend, entry=entry):
                    with self.assertRaisesRegex(ValueError, '实机 IK'):
                        entry(NS(ik_backend=backend))

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
