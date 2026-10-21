"""Identity, linear targets, SDK lifecycle and asynchronous link regressions; no hardware."""
import ast
import base64
from contextlib import contextmanager
import json
import io
from pathlib import Path
import queue
import shlex
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import zlib

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gripper_core import TriggerMapping, select_devices, healthy_sample
from worker import Device, Worker, LinkFailure, check_link
from apps.gripper_control.client import Session, remote_command


def endpoint(sn='TCGU01A28Z0015s', side='Left', port='/dev/left', role='Follower'):
    return dict(sn=sn, side=side, port=port, role=role, in_use=False)


class FakeMotor:
    def __init__(self, events):
        self.events = events
        self.enabled = False
        self.position = .4
        self.fault = NS(version=1, report_flags=0x1D, source_mask=4, protocol_mode=2,
                        motor_fault_code=0, motor_latched_fault_code=0,
                        firmware_fault_code=0, firmware_latched_fault_code=0x0303)

    def get_spec(self):
        return NS(stall_cont_torque_nm=1.1, v_max_rad_s=10.)

    def fault_report(self):
        return self.fault

    def read_status(self, **_):
        return NS(actual_pos=self.position, actual_vel=0., actual_torque=0., motor_temp_c=30.,
                  status=int(self.enabled))

    def enable(self):
        self.events.append('enable')
        self.enabled = True

    def disable(self):
        self.events.append('disable')
        self.enabled = False


class FakeGripper:
    def __init__(self, events, **_):
        self.events = events
        self.motor = FakeMotor(events)
        self.firmware_sn = endpoint()['sn']
        self.firmware_version = 'FirmwareVersion(1.2.16.0)'
        self.audit = NS(ok=True, effective=NS(cont_torque_nm=1.1, peak_torque_nm=1.8), detail='',
                        stored=NS(cont_torque_nm=1.1, peak_torque_nm=1.8),
                        recommended=NS(cont_torque_nm=1.1, peak_torque_nm=1.8))

    def position_map(self):
        return NS(max_open_rad=1., min_open_rad=0.)

    def audit_envelope(self):
        return self.audit

    def rad_to_pos(self, value):
        return value

    def close(self):
        self.events.append('close_port')


class FakeController:
    def __init__(self, gripper, cfg):
        if cfg.close_preload_nm > cfg.grasp_torque_nm:
            raise ValueError('SDK preload must fit within grasp torque')
        self.g, self.cfg = gripper, cfg
        self.target = None
        self.age = 1.
        self.fault = ''

    def start(self):
        self.g.events.append('start_at_measured_position')
        self.target = self.g.motor.position

    def stop(self):
        self.g.events.append('stop_zero_disable')
        self.g.motor.enabled = False

    def set_target(self, value):
        self.g.events.append(('target', value))
        self.target = self.g.motor.position = value

    def hold_position(self):
        self.g.events.append('hold')
        self.target = self.g.motor.position

    def snapshot(self):
        sample = self.g.motor.read_status()
        return NS(observation=NS(valid=True, age_ms=self.age, position=sample.actual_pos,
                                 velocity=0., torque=0., motor_temp_c=30., status=sample.status),
                  fault_reason=self.fault, target_position=self.target,
                  state=NS(name='HOLDING_POSITION'), grasp_torque_nm=self.cfg.grasp_torque_nm)


def fake_sdk(events):
    return NS(FollowerGripper=lambda **kw: FakeGripper(events, **kw),
              ForcePositionConfig=NS(for_spec=lambda _: NS(close_preload_nm=.25)),
              ForcePositionController=FakeController)


class IdentityTests(unittest.TestCase):
    def test_missing_side_never_reuses_the_other_device(self):
        self.assertEqual(set(select_devices([endpoint()])), {'left'})
        self.assertFalse(select_devices([endpoint(role='Leader')]))
        self.assertFalse(select_devices([endpoint(side='Unknown')]))

    def test_duplicate_identity_requires_explicit_sn_and_override_is_exact(self):
        endpoints = [endpoint(), endpoint(sn='OTHERs', port='/dev/other')]
        with self.assertRaises(ValueError):
            select_devices(endpoints)
        chosen = select_devices([endpoint()], {'right': endpoint()['sn']})
        self.assertEqual(set(chosen), {'right'})
        self.assertEqual(chosen['right']['sn'], endpoint()['sn'])

    def test_explicit_missing_or_repeated_assignment_is_rejected(self):
        for assignments in ({'left': 'missing'}, {'left': endpoint()['sn'], 'right': endpoint()['sn']}):
            with self.assertRaises(ValueError):
                select_devices([endpoint()], assignments)
        right = select_devices([endpoint(side='Unknown')], {'right': endpoint()['sn']})
        self.assertEqual(right['right']['port'], '/dev/left')


class DeviceTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.device = Device(fake_sdk(self.events), endpoint())

    def test_read_only_connect_and_close_do_not_enable_or_disable(self):
        self.assertFalse(self.device.last['enabled'])
        self.device.close()
        self.assertEqual(self.events, ['close_port'])

    def test_idle_feedback_age_and_calibrated_motor_span_are_real_measurements(self):
        with patch('worker.time.monotonic', return_value=10.), \
             patch.object(FakeGripper, 'position_map', return_value=NS(min_open_rad=.2, max_open_rad=1.4)):
            device = Device(fake_sdk(self.events), endpoint())
        with patch('worker.time.monotonic', return_value=10.25):
            snapshot = device.snapshot()
        self.assertAlmostEqual(snapshot['age_ms'], 250.)
        self.assertAlmostEqual(snapshot['open_span_rad'], 1.2)
        self.assertAlmostEqual(snapshot['opening_angle_rad'], .48)
        with patch('worker.time.monotonic', return_value=10.4):
            self.assertAlmostEqual(device.snapshot()['age_ms'], 400.)
            device.gripper.motor.position = .7
            device.last = device.read_idle()
            self.assertEqual(device.snapshot()['age_ms'], 0.)
            self.assertEqual(device.snapshot()['position'], .7)
        device.close()
        self.assertEqual(self.events, ['close_port'])

    def test_seed_then_enable_continuous_target_hold_and_cleanup(self):
        self.device.enable(1.1, .25)
        self.assertEqual(self.events, ['start_at_measured_position', 'enable'])
        self.assertEqual(self.device.snapshot()['target'], .4)
        for value in (0., .25, .5, .75, 1.):
            self.device.target(value)
            self.assertEqual(self.device.snapshot()['position'], value)
        self.device.hold()
        self.device.close()
        self.assertEqual(self.events[-3:], ['stop_zero_disable', 'disable', 'close_port'])

    def test_ratings_faults_bad_envelope_or_enabled_motor_block_enable(self):
        for mutate in (lambda d: setattr(d.gripper.motor, 'enabled', True),
                       lambda d: setattr(d.gripper.motor.fault, 'firmware_fault_code', 1),
                       lambda d: setattr(d.gripper.motor.fault, 'report_flags', 0),
                       lambda d: setattr(d.gripper.audit, 'ok', False)):
            d = Device(fake_sdk([]), endpoint())
            mutate(d)
            with self.assertRaises(ValueError):
                d.enable(1.1, .25)
            self.assertIsNone(d.controller)
        with self.assertRaises(ValueError):
            self.device.enable(1.2, .25)
        self.assertNotIn('enable', self.events)

    def test_nonfinite_target_and_stale_or_faulty_state_cannot_drive(self):
        self.device.enable(1.1, .25)
        for value in (-.1, 1.1, True, float('nan')):
            with self.assertRaises(ValueError):
                self.device.target(value)
        self.device.controller.age = 300
        with self.assertRaises(RuntimeError):
            self.device.snapshot()
        self.device.close()
        self.assertFalse(self.device.gripper.motor.enabled)

    def test_right_gripper_capture_reports_each_gate_without_sending_enable(self):
        self.device.firmware = 'FirmwareVersion(1.2.6.0)'
        self.device.gripper.motor.fault.report_flags = 0x15
        self.device.gripper.motor.fault.motor_latched_fault_code = 0x04
        self.device.gripper.motor.fault.firmware_latched_fault_code = 0
        self.device.gripper.audit.ok = False
        self.device.gripper.audit.stored.peak_torque_nm = 2.
        with self.assertRaises(ValueError) as result:
            self.device.enable(1.1, .25)
        text = str(result.exception)
        for expected in ('未发送使能', 'flags=0x15', 'FwValid', '0x00000004', '欠压', '2.000', '1.800'):
            self.assertIn(expected, text)
        self.assertNotIn('enable', self.events)
        self.assertIsNone(self.device.controller)
        self.assertEqual(len(self.device.diagnostics()['issues']), 3)

    def test_cached_diagnostics_do_not_poll_the_serial_bus_while_running(self):
        self.device.enable(1.1, .25)
        with patch.object(self.device.gripper.motor, 'fault_report', side_effect=AssertionError('serial poll')), \
             patch.object(self.device.gripper, 'audit_envelope', side_effect=AssertionError('serial poll')):
            self.assertEqual(self.device.diagnostics()['issues'], [])
            self.assertTrue(self.device.snapshot()['enabled'])

    def test_low_torque_also_bounds_the_sdk_closed_endpoint_preload(self):
        for torque, expected_preload in ((.01, .01), (.2, .2), (.25, .25), (1.1, .25)):
            with self.subTest(torque=torque):
                device = Device(fake_sdk([]), endpoint())
                device.enable(torque, .1)
                self.assertEqual(device.controller.cfg.close_preload_nm, expected_preload)
                device.close()

    def zero_fault_capture(self, flags=0x15):
        self.device.gripper.motor.fault = NS(
            version=1, report_flags=flags, source_mask=1, protocol_mode=2,
            motor_fault_code=0, motor_latched_fault_code=0,
            firmware_fault_code=0, firmware_latched_fault_code=0)

    def test_reviewed_post_power_capture_accepts_zero_firmware_fault_without_flag(self):
        for flags in (0x15, 0x17):
            with self.subTest(flags=flags):
                self.zero_fault_capture(flags)
                self.assertEqual(self.device.diagnostics()['issues'], [])
        self.device.enable(1.1, .25)
        self.assertEqual(self.events, ['start_at_measured_position', 'enable'])
        self.device.close()
        self.assertFalse(self.device.gripper.motor.enabled)

    def test_zero_firmware_rule_never_accepts_unknown_firmware_layout_or_mcu_sources(self):
        for field, value in (('version', 0), ('version', 2), ('protocol_mode', 0),
                             ('source_mask', 2), ('source_mask', 4), ('source_mask', 8),
                             ('firmware_fault_code', 0x0202),
                             ('firmware_latched_fault_code', 0x0303)):
            with self.subTest(field=field, value=value):
                self.zero_fault_capture()
                setattr(self.device.gripper.motor.fault, field, value)
                with self.assertRaises(ValueError):
                    self.device.enable(1.1, .25)
                self.assertNotIn('enable', self.events)
        self.zero_fault_capture()
        for firmware in ('未知', 'FirmwareVersion(1.2.6.0)', 'FirmwareVersion(1.2.17.0)'):
            with self.subTest(firmware=firmware):
                self.device.firmware = firmware
                with self.assertRaisesRegex(ValueError, 'FwValid'):
                    self.device.enable(1.1, .25)
        self.assertNotIn('enable', self.events)

    def test_zero_firmware_rule_preserves_motor_validity_fault_and_envelope_guards(self):
        for flags in (0x00, 0x11, 0x14, 0x35, 0x55):
            with self.subTest(flags=flags):
                self.zero_fault_capture(flags)
                with self.assertRaises(ValueError):
                    self.device.enable(1.1, .25)
                self.assertNotIn('enable', self.events)
        for field in ('motor_fault_code', 'motor_latched_fault_code'):
            self.zero_fault_capture()
            setattr(self.device.gripper.motor.fault, field, 4)
            with self.assertRaisesRegex(ValueError, '欠压'):
                self.device.enable(1.1, .25)
        self.zero_fault_capture()
        self.device.gripper.audit.ok = False
        with self.assertRaisesRegex(ValueError, '力矩包络'):
            self.device.enable(1.1, .25)
        self.assertNotIn('enable', self.events)


class WorkerTests(unittest.TestCase):
    def test_whole_stream_validated_before_any_jaw_moves(self):
        events = []
        worker = Worker(fake_sdk(events))
        worker.devices['left'] = Device(worker.sdk, endpoint())
        worker.devices['left'].enable(1.1, .25)
        for targets in ({'left': .1, 'right': .2}, {'left': float('nan')}):
            with self.assertRaises(ValueError):
                worker.command(dict(id=1, created=10., action='stream', targets=targets), 10.1)
        self.assertFalse(any(isinstance(e, tuple) for e in events))
        with patch('worker.emit'):
            worker.close()
        self.assertFalse(worker.devices['left'].gripper.motor.enabled)

    def test_stale_commands_are_rejected_even_with_fresh_transport(self):
        worker = Worker(fake_sdk([]))
        with self.assertRaisesRegex(ValueError, '0.25'):
            worker.command(dict(id=1, created=10., action='enable', side='both'), 10.3)

    def test_real_worker_loop_holds_on_target_loss_and_disables_on_link_loss(self):
        events, outputs = [], queue.Queue()
        class Input:
            def __init__(self):
                self.lines = queue.Queue()
            def __iter__(self):
                while True:
                    line = self.lines.get()
                    if line is None:
                        return
                    yield line
        source = Input()
        worker = Worker(fake_sdk(events))
        def connect():
            worker.endpoints = []
            worker.devices['left'] = Device(worker.sdk, endpoint())
        worker.connect = connect
        acknowledge = [True]
        latest = [0.]
        def emit(**value):
            outputs.put(value)
            if value.get('stamp'):
                latest[0] = value['stamp']
                if acknowledge[0]:
                    source.lines.put(json.dumps({'ack': latest[0]})+'\n')
        result = []
        def run():
            try:
                worker.run()
            except RuntimeError as exc:
                result.append(str(exc))
            finally:
                worker.close()
                source.lines.put(None)
        def wait_for(predicate):
            deadline = time.monotonic()+3.
            while time.monotonic() < deadline:
                value = outputs.get(timeout=3.)
                if predicate(value):
                    return value
            self.fail('worker simulation timed out')
        with patch('worker.sys.stdin', source), patch('worker.emit', emit):
            thread = threading.Thread(target=run, daemon=True)
            thread.start()
            try:
                wait_for(lambda value: 'stamp' in value)
                source.lines.put(json.dumps({'ack': latest[0], 'request':dict(
                    action='enable', side='left', id=1, created=latest[0])})+'\n')
                wait_for(lambda v:v.get('devices', {}).get('left', {}).get('controlled'))
                source.lines.put(json.dumps({'ack': latest[0], 'request':dict(
                    action='stream', targets={'left': .27}, id=2, created=latest[0])})+'\n')
                wait_for(lambda v:v.get('devices', {}).get('left', {}).get('position') == .27)
                wait_for(lambda v:'stream_paused' in v)
                self.assertIn('hold', events)
                self.assertTrue(worker.devices['left'].gripper.motor.enabled)
                acknowledge[0] = False
                thread.join(3.)
                self.assertFalse(thread.is_alive())
                self.assertTrue(any('超时' in value for value in result))
                self.assertFalse(worker.devices['left'].gripper.motor.enabled)
                self.assertIn('close_port', events)
            finally:
                worker.exit.set()
                source.lines.put(None)
                thread.join(3.)


class LinkDiagnosticsTests(unittest.TestCase):
    def test_heartbeat_and_feedback_timeouts_are_distinct(self):
        for received, ack, reason in ((9., 9.9, 'heartbeat_timeout'),
                                      (9.9, 9., 'feedback_ack_timeout')):
            with self.subTest(reason=reason), self.assertRaises(LinkFailure) as caught:
                check_link({'ack': ack}, received, 10., 8., 8.6)
            failure = caught.exception
            self.assertEqual(failure.reason, reason)
            self.assertAlmostEqual(failure.diagnostics['heartbeat_age_s'], 10.-received)
            self.assertAlmostEqual(failure.diagnostics['feedback_ack_age_s'], 10.-ack)
            self.assertEqual(failure.diagnostics['server_monotonic'], 10.)
            self.assertEqual(failure.diagnostics['timeout_s'], .6)

    def test_invalid_ack_is_not_reported_as_network_timeout(self):
        for ack in (None, True, '9.9', float('nan'), float('inf'), 10.01, 0.):
            with self.subTest(ack=ack), self.assertRaises(LinkFailure) as caught:
                check_link({'ack': ack}, 9.9, 10., 8., 8.6)
            self.assertEqual(caught.exception.reason, 'feedback_ack_invalid')
            json.dumps(caught.exception.diagnostics, allow_nan=False)

    def test_existing_grace_and_timeout_limits_are_preserved(self):
        self.assertIsNone(check_link({}, 7., 10., 9.5, 10.1))
        self.assertIsNone(check_link({'ack': 8.}, 9.9, 10., 8., 10.1))
        with self.assertRaises(LinkFailure):
            check_link({'ack': 8.}, 9.9, 10.1001, 8., 10.1)
        self.assertIsNone(check_link({'ack': 9.4001}, 9.4001, 10., 8., 8.6))
        with self.assertRaises(LinkFailure):
            check_link({'ack': 9.3999}, 9.9, 10., 8., 8.6)

    def test_exit_request_wins_over_stale_heartbeat_and_ack(self):
        detail = check_link({'exit': True, 'exit_reason': 'client_exit', 'ack': 1.},
                            1., 10., 8., 8.6)
        self.assertEqual(detail['reason'], 'exit_request')
        self.assertEqual(detail['client_exit_reason'], 'client_exit')
        self.assertEqual(detail['heartbeat_age_s'], 9.)

    def test_input_exit_eof_and_invalid_packet_have_specific_causes(self):
        for source, expected in (
            ('{"exit":true,"exit_reason":"feedback_timeout","ack":1}\n', 'exit_request'),
            ('', 'stdin_closed'), ('not json\n', 'input_invalid'), ('[]\n', 'input_invalid')):
            with self.subTest(expected=expected), patch('worker.sys.stdin', io.StringIO(source)):
                worker = Worker(fake_sdk([]))
                worker.read()
                self.assertTrue(worker.exit.is_set())
                self.assertEqual(worker.exit_detail['reason'], expected)
                self.assertIsInstance(worker.exit_detail['server_monotonic'], float)
                if expected == 'exit_request':
                    self.assertEqual(worker.exit_detail['client_exit_reason'], 'feedback_timeout')

    def test_normal_exit_emits_reason_and_does_not_raise_timeout(self):
        worker = Worker(fake_sdk([]))
        def explicit_exit():
            worker.request_exit('exit_request', client_exit_reason='client_exit')
        with patch.object(worker, '_run', side_effect=explicit_exit), patch('worker.emit') as output:
            worker.run()
        self.assertEqual(output.call_args.kwargs['exit_reason']['reason'], 'exit_request')


class TriggerTests(unittest.TestCase):
    def frame(self, left=0., right=0.):
        return {side: dict(trigger=value, trigger_available=True)
                for side, value in [('left', left), ('right', right)]}

    def test_linear_mapping_and_mirror_routing_with_release_to_arm(self):
        mapping = TriggerMapping()
        hands = {'left': 'right', 'right': 'left'}
        self.assertFalse(mapping.update(self.frame(1., 1.), hands, ['left', 'right']))
        self.assertFalse(mapping.update(self.frame(), hands, ['left', 'right']))
        self.assertEqual(mapping.update(self.frame(.75, .25), hands, ['left', 'right']),
                         {'left': .75, 'right': .25})
        self.assertEqual(mapping.update(self.frame(), hands, ['left', 'right']), {'left': 1., 'right': 1.})
        mapping.reset()
        self.assertFalse(mapping.update(self.frame(1., 1.), hands, ['left']))

    def test_missing_trigger_is_not_synthesized_as_an_open_command(self):
        with self.assertRaises(ValueError):
            TriggerMapping().update({'left': {}}, {'left': 'left'}, ['left'])

    def test_independent_limits_follow_robot_side_and_preserve_release_to_arm(self):
        mapping = TriggerMapping({'left': .6, 'right': .3})
        hands = {'left': 'right', 'right': 'left'}
        self.assertEqual(mapping.update(self.frame(), hands, hands), {})
        for pressed in (.1, .5, 1., 0.):
            value = mapping.update(self.frame(pressed, pressed), hands, hands)
            self.assertAlmostEqual(value['left'], .6*(1.-pressed))
            self.assertAlmostEqual(value['right'], .3*(1.-pressed))
        mapping.reset()
        self.assertEqual(mapping.update(self.frame(1., 1.), hands, hands), {})
        self.assertEqual(mapping.max_openings, {'left': .6, 'right': .3})

    def test_zero_full_and_invalid_opening_limits(self):
        hands = {'left': 'left', 'right': 'right'}
        mapping = TriggerMapping({'left': 0., 'right': 1.})
        mapping.update(self.frame(), hands, hands)
        self.assertEqual(mapping.update(self.frame(.5, .5), hands, hands), {'left': 0., 'right': .5})
        for value in (-.1, 1.1, float('nan'), True, None):
            with self.assertRaises(ValueError):
                TriggerMapping({'left': value, 'right': 1.})
        for value in ({'left': .5}, [], {}):
            with self.assertRaises(ValueError):
                TriggerMapping(value)


class TransportTests(unittest.TestCase):
    def test_bundled_worker_compiles_and_uses_installed_robot_sdk_without_ros(self):
        setup = shlex.split(remote_command())[2]
        self.assertIn('/home/fairino/umi/.venv/bin/python', setup)
        self.assertNotIn('/opt/ros', setup)
        source = shlex.split(setup.split(' -u -c ', 1)[1])[0]
        tree = ast.parse(source)
        encoded = next(n.args[0].value for n in ast.walk(tree) if isinstance(n, ast.Call)
                       and ast.unparse(n.func) == 'base64.b64decode')
        bundle = json.loads(zlib.decompress(base64.b64decode(encoded)))
        for name, text in bundle.items():
            compile(text, name, 'exec')

    @contextmanager
    def session_peer(self, drop_actions=()):
        """Use the real Session + Connection/JSON path; replace only SSH and SDK."""
        sys.path.insert(0, str(ROOT/'apps/fairino_upper_probe'))
        import upper_keyboard  # Load before replacing the session's log root.
        from apps.gripper_control import client
        events, packets, lines = [], [], queue.Queue()
        worker = Worker(fake_sdk(events))
        worker.endpoints = []
        worker.devices['left'] = Device(worker.sdk, endpoint())

        def output(**value):
            lines.put(upper_keyboard.PREFIX+json.dumps(value)+'\n')

        class Peer:
            closed = False

            def expect(self, *_):
                worker.report()
                return 0  # SSH authenticated, worker ready.

            def readline(self):
                return lines.get()

            def sendline(self, line):
                packet = json.loads(line)
                packets.append(packet)
                if packet.get('exit'):
                    self.close()
                    return
                request = packet.get('request')
                if (request and request['id'] > worker.last_request
                        and request['action'] not in drop_actions):
                    worker.last_request = request['id']
                    try:
                        worker.command(request, time.monotonic())
                    except ValueError as exc:
                        output(rejected=str(exc), request_id=worker.last_request)
                worker.report()

            def close(self, **_):
                if not self.closed:
                    self.closed = True
                    worker.close()
                    lines.put('')  # Remote EOF after the disable confirmation.

        peer = Peer()
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(client, 'ROOT', Path(directory)), \
             patch('pexpect.spawn', return_value=peer), patch('worker.emit', output):
            session = Session('unused', 'unused', 'not-a-real-password').start()
            try:
                self.wait_for(lambda: session.snapshot().get('ready'))
                yield session, worker, events, packets
            finally:
                session.close()
                session.thread.join(2.)
                peer.close()
                self.assertFalse(session.thread.is_alive(), 'session cleanup did not complete')

    def wait_for(self, condition):
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(.005)
        self.fail('shared transport simulation timed out')

    def test_background_transport_handles_ui_actions_then_stream_and_disable(self):
        with self.session_peer() as (session, worker, events, packets):
            self.assertNotIn('enable', events)  # Connecting remains read-only.
            session.command('enable', 'left')
            self.wait_for(lambda: session.snapshot().get('last_request') == 1
                          and not session.snapshot()['busy'])
            self.assertTrue(worker.devices['left'].gripper.motor.enabled)
            self.assertEqual(events, ['start_at_measured_position', 'enable'])
            session.command('target', 'left', opening=.65)
            self.wait_for(lambda: ('target', .65) in events and not session.snapshot()['busy'])
            session.set_targets({'left': .37})
            self.wait_for(lambda: ('target', .37) in events)
            session.set_targets(None)
            self.wait_for(lambda: 'hold' in events)
            self.assertTrue(worker.devices['left'].gripper.motor.enabled)
            session.command('disable', 'left')
            self.wait_for(lambda: not session.snapshot()['busy']
                          and not worker.devices['left'].gripper.motor.enabled)
            self.assertTrue(all('wrist_guard' not in packet for packet in packets))
        self.assertFalse(session.failure)
        self.assertTrue(session.disabled_confirmed)

    def test_pico_trigger_reaches_sdk_through_the_same_session_and_transport(self):
        sys.path.insert(0, str(ROOT/'scripts/pico4/fairino'))
        from vr_grippers import GripperTeleop
        with self.session_peer() as (session, worker, events, packets):
            control = GripperTeleop(NS(host='unused', user='unused'), None,
                                   session_factory=lambda *_, **__: session)
            hands = {'left': 'right', 'right': 'left'}
            def frame(trigger):
                return {side: dict(trigger=trigger, trigger_available=True) for side in hands}
            control.update(frame(0.), hands, True)
            self.wait_for(lambda: session.snapshot()['devices']['left']['controlled']
                          and not session.snapshot()['busy'])
            control.update(frame(0.), hands, True)  # Release-to-arm; keep measured position.
            self.assertFalse(any(isinstance(e, tuple) for e in events))
            control.update(frame(.75), hands, True)
            self.wait_for(lambda: ('target', .25) in events)
            control.update(None, hands, False)
            self.wait_for(lambda: 'hold' in events)
            self.assertTrue(worker.devices['left'].gripper.motor.enabled)
        self.assertFalse(session.failure)
        self.assertTrue(session.disabled_confirmed)

    def test_unacknowledged_commands_expire_even_when_feedback_is_live(self):
        for action in ('enable', 'disable'):
            with self.subTest(action=action), \
                 patch('apps.gripper_control.client.COMMAND_TIMEOUT', .1):
                with self.session_peer(drop_actions={action}) as (session, worker, events, packets):
                    if action == 'disable':
                        session.command('enable', 'left')
                        self.wait_for(lambda: session.snapshot()['devices']['left']['controlled']
                                      and not session.snapshot()['busy'])
                    session.command(action, 'left')
                    self.wait_for(session.finished)
                    self.assertIn('秒未确认', session.failure)
                    self.assertFalse(session.snapshot()['busy'])
                    self.assertTrue(session.stop.is_set())
                    self.assertTrue(session.disabled_confirmed)
                    self.assertFalse(worker.devices['left'].gripper.motor.enabled)
                    self.assertTrue(any(p['exit'] for p in packets))
                    self.assertTrue(all(p['request'] is None or p['request']['action'] != 'stream'
                                        for p in packets))
                    if action == 'enable':
                        self.assertNotIn('enable', events)
                    else:
                        self.assertIn('stop_zero_disable', events)

    def test_worker_rejection_is_reported_and_never_leaves_enable_pending(self):
        with self.session_peer() as (session, worker, events, packets):
            worker.devices['left'].gripper.motor.fault.motor_fault_code = 4
            session.command('enable', 'left')
            self.wait_for(lambda: session.snapshot().get('last_request') == 1
                          and not session.snapshot()['busy'])
            self.assertIn('欠压', session.snapshot()['message'])
            self.assertNotIn('enable', events)
            self.assertFalse(worker.devices['left'].gripper.motor.enabled)
        self.assertTrue(session.disabled_confirmed)


if __name__ == '__main__':
    unittest.main()
