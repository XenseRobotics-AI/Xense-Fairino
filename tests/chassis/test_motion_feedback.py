"""Offline producer/consumer contract: no ROS imports, SSH, or hardware actions."""
import ast
import copy
import json
from pathlib import Path
import shlex
import sys
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from fairino_slam.chassis import motion_feedback as mf, robot_worker as rw, transport

FIXTURE = Path(__file__).parent/'fixtures/chassis_motion_state.json'


class DiagnosticStatusStub(NS):
    """Match ROS's byte constants and field setter without importing ROS."""
    OK = b'\x00'
    ERROR = b'\x02'

    @property
    def level(self):
        return getattr(self, '_level', self.OK)

    @level.setter
    def level(self, value):
        if not isinstance(value, (bytes, bytearray)) or len(value) != 1:
            raise AssertionError("The 'level' field must be a single byte")
        self._level = value


def message(**changes):
    data = json.loads(FIXTURE.read_text())
    status = data['status'][0]
    status['level'] = bytes([status['level']])  # JSON stores the octet as a number.
    fields = {v['key']: v['value'] for v in status['values']}
    for key, value in changes.items():
        if key in ('session', 'level', 'message'):
            status[{'session': 'hardware_id'}.get(key, key)] = value
        else:
            fields[key] = str(value)
    status['values'] = [NS(key=k, value=v) for k, v in fields.items()]
    return NS(header=NS(stamp=NS(**data['header']['stamp'])), status=[NS(**status)])


class FeedbackMonitorTests(unittest.TestCase):
    def test_fixture_wait_is_healthy_not_vendor_phase(self):
        monitor = mf.FeedbackMonitor()
        self.assertTrue(monitor.receive(message(), 20., 100.123))
        value = monitor.snapshot(20.1, 100.223)
        self.assertTrue(value['ready'])
        self.assertTrue(value['drive_wait'])
        self.assertEqual(value['session'], 'a'*32)
        self.assertAlmostEqual(value['received_age'], .1)
        self.assertEqual(value['requested_vy'], .01)
        self.assertEqual(value['sequence'], 1)

    def test_level_is_exactly_one_ros_byte_and_invalid_messages_fail_closed(self):
        for level in (b'\x00', bytearray([0])):
            with self.subTest(valid=level):
                monitor = mf.FeedbackMonitor()
                self.assertTrue(monitor.receive(message(level=level), 20., 100.123))
                self.assertTrue(monitor.snapshot(20., 100.123)['ready'])
        for level in (0, 2, False, None, '', '0', b'', b'\x00\x00', [0]):
            with self.subTest(invalid=level):
                monitor = mf.FeedbackMonitor()
                self.assertTrue(monitor.receive(message(), 20., 100.123))
                self.assertFalse(monitor.receive(message(sequence='2', level=level), 20.1, 100.223))
                state = monitor.snapshot(20.1, 100.223)
                self.assertFalse(state['ready'])
                self.assertIn('level byte', state['reason'])
                self.assertAlmostEqual(state['received_age'], .1)

    def test_idle_is_ready_before_first_goal(self):
        monitor = mf.FeedbackMonitor()
        self.assertTrue(monitor.receive(message(requested_vx='0', requested_vy='0',
                                               drive_wait='0', message='idle'), 20., 100.123))
        self.assertTrue(monitor.snapshot(20., 100.123)['ready'])

    def test_active_and_fault_states_are_distinct(self):
        monitor = mf.FeedbackMonitor()
        self.assertTrue(monitor.receive(message(drive_command_peak='.2', drive_wait='0',
                                               message='drive_active'), 20., 100.123))
        self.assertFalse(monitor.snapshot(20., 100.123)['drive_wait'])
        self.assertTrue(monitor.receive(message(sequence='2', ready='0', drive_wait='0',
                                               level=b'\x02', message='not_ready'), 20.01, 100.133))
        value = monitor.snapshot(20.01, 100.133)
        self.assertFalse(value['ready'])
        self.assertEqual(value['requested_vx'], .02)

    def test_missing_receipt_and_ros_age_each_fail_closed(self):
        self.assertFalse(mf.FeedbackMonitor().snapshot(20., 100.)['ready'])
        for mono, ros in ((20.41, 100.123), (20., 100.524), (19.9, 100.123), (20., 100.0)):
            with self.subTest(mono=mono, ros=ros):
                monitor = mf.FeedbackMonitor()
                monitor.receive(message(), 20., 100.123)
                self.assertFalse(monitor.snapshot(mono, ros)['ready'])

    def test_underlying_hardware_age_accumulates_after_receipt(self):
        monitor = mf.FeedbackMonitor()
        self.assertTrue(monitor.receive(message(hardware_age_sec='.3'), 20., 100.123))
        self.assertTrue(monitor.snapshot(20.09, 100.213)['ready'])
        self.assertFalse(monitor.snapshot(20.11, 100.233)['ready'])

    def test_transport_delay_counts_toward_hardware_freshness(self):
        monitor = mf.FeedbackMonitor()
        self.assertFalse(monitor.receive(message(hardware_age_sec='.3'), 20., 100.323))
        self.assertTrue(monitor.receive(message(hardware_age_sec='.2'), 20., 100.223))
        self.assertFalse(monitor.snapshot(20.11, 100.334)['ready'])

    def test_invalid_monitor_timeout_is_rejected(self):
        for value in (True, 0, -.1, None, '0.4', float('nan'), float('inf')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                mf.FeedbackMonitor(value)

    def test_replay_or_regression_is_invalid_and_does_not_refresh_receipt(self):
        monitor = mf.FeedbackMonitor()
        monitor.receive(message(sequence='5'), 20., 100.123)
        for sequence in ('5', '4'):
            self.assertFalse(monitor.receive(message(sequence=sequence), 20.1, 100.223))
            self.assertFalse(monitor.snapshot(20.1, 100.223)['ready'])
            self.assertAlmostEqual(monitor.snapshot(20.1, 100.223)['received_age'], .1)
        self.assertTrue(monitor.receive(message(sequence='6'), 20.2, 100.323))
        self.assertTrue(monitor.snapshot(20.2, 100.323)['ready'])

    def test_new_worker_session_is_exposed_for_gate_goal_binding(self):
        monitor = mf.FeedbackMonitor()
        monitor.receive(message(sequence='9'), 20., 100.123)
        self.assertTrue(monitor.receive(message(session='b'*32), 20.01, 100.133))
        self.assertEqual(monitor.snapshot(20.01, 100.133)['session'], 'b'*32)
        self.assertFalse(monitor.receive(message(sequence='8'), 20.02, 100.143))

    def test_malformed_schema_boolean_numeric_session_and_sequence_are_invalid(self):
        for changes in ({'schema_version':'2'}, {'schema_version':'1','extra':'1'},
                        {'ready':'true'}, {'drive_wait':'2'}, {'sequence':'0'},
                        {'sequence':'1.0'}, {'session':'bad'}, {'hardware_age_sec':'-.1'},
                        {'hardware_age_sec':'.41'}, {'drive_velocity_peak':'-1'},
                        {'drive_command_peak':'nan'}, {'requested_wz':'inf'},
                        {'level':b'\x02'}, {'drive_wait':'0'}, {'message':'ALIGN_ONLY'}):
            with self.subTest(changes=changes):
                monitor = mf.FeedbackMonitor()
                self.assertFalse(monitor.receive(message(**changes), 20., 100.123))
                self.assertFalse(monitor.snapshot(20., 100.123)['ready'])

    def test_duplicate_missing_fields_and_multiple_status_rejected(self):
        for mutation in (lambda m:m.status[0].values.append(m.status[0].values[0]),
                         lambda m:m.status[0].values.pop(),
                         lambda m:m.status.append(m.status[0])):
            data = message()
            mutation(data)
            self.assertFalse(mf.FeedbackMonitor().receive(data, 20., 100.123))

    def test_nonpositive_ros_clock_or_stamp_is_rejected(self):
        for ros in (0., -.01):
            data = message()
            data.header.stamp = NS(sec=0, nanosec=0)
            monitor = mf.FeedbackMonitor()
            self.assertFalse(monitor.receive(data, 20., ros))
            self.assertFalse(monitor.snapshot(20., ros)['ready'])
        data = message()
        data.header.stamp = NS(sec=0, nanosec=0)
        self.assertFalse(mf.FeedbackMonitor().receive(data, 20., .01))
        monitor = mf.FeedbackMonitor()
        monitor.receive(message(), 20., 100.123)
        self.assertFalse(monitor.snapshot(20., 0.)['ready'])

    def test_stale_future_and_invalid_header_rejected(self):
        for sec, nano in ((99,0),(101,0),(100,-1),(100,1000000000),(True,0)):
            data = message()
            data.header.stamp = NS(sec=sec,nanosec=nano)
            self.assertFalse(mf.FeedbackMonitor().receive(data, 20., 100.123))


class ProducerTests(unittest.TestCase):
    def hardware(self):
        return dict(enabled=[True]*8, command_velocity=[0.]*8, velocity=[0.]*8)

    def test_wait_depends_on_actual_command_not_measured_drive_speed(self):
        hw = self.hardware()
        hw['velocity'][1] = .3
        name, value = mf.motion_values(hw, (.02, .01, 0), True, .01)
        self.assertEqual(name, 'drive_wait')
        self.assertTrue(value['drive_wait'])
        self.assertEqual(value['drive_velocity_peak'], .3)
        hw['command_velocity'][5] = -.1
        self.assertEqual(mf.motion_values(hw, (.02,.01,0), True, .01)[0], 'drive_active')

    def test_idle_rotation_threshold_disabled_and_invalid_values(self):
        hw = self.hardware()
        self.assertEqual(mf.motion_values(hw, (0,0,0), True, .01)[0], 'idle')
        self.assertEqual(mf.motion_values(hw, (0,0,.01), True, .01)[0], 'drive_wait')
        self.assertEqual(mf.motion_values(hw, (1e-4,0,0), True, .01)[0], 'idle')
        hw['enabled'][1] = False
        self.assertEqual(mf.motion_values(hw, (.1,0,0), True, .01)[0], 'not_ready')
        for request in ((float('nan'),0,0),(1,2)):
            with self.assertRaises(ValueError):
                mf.motion_values(hw, request, True, .01)

    def worker(self):
        w = rw.Worker.__new__(rw.Worker)
        w.motion_pub = Mock()
        w.motion_session, w.motion_sequence, w.motion_published = 'a'*32, 0, 0.
        w.ready, w.hw, w.commanded = True, self.hardware(), (.02,.01,0)
        w.hw_time, w.upper_time = 19.98, 19.97
        w.input_lock, w.packet = threading.Lock(), {'focused':True}
        w.stop = threading.Event()
        w.guard = Mock()
        w.node = Mock()
        w.node.get_clock().now().to_msg.return_value = NS(sec=100,nanosec=123000000)
        return w

    def publish(self, w, now=20., force=False):
        def array():
            return NS(header=NS(), status=[])
        with patch.object(rw.time,'monotonic',return_value=now), patch.dict(
                rw.__dict__, DiagnosticArray=array, DiagnosticStatus=DiagnosticStatusStub, KeyValue=NS):
            w.publish_motion_state(force)
        return w.motion_pub.publish.call_args.args[0]

    def test_diagnostic_status_stub_rejects_old_integer_assignment(self):
        status = DiagnosticStatusStub()
        for value in (0, 2, b'', b'\x00\x00'):
            with self.subTest(value=value), self.assertRaises(AssertionError):
                status.level = value
        status.level = DiagnosticStatusStub.ERROR
        self.assertEqual(status.level, b'\x02')

    def test_startup_without_hardware_publishes_not_ready_without_stopping(self):
        w = self.worker()
        w.ready, w.hw, w.commanded = False, None, (0., 0., 0.)
        w.hw_time = w.upper_time = 0.
        data = self.publish(w)
        self.assertFalse(w.stop.is_set())
        self.assertEqual(data.status[0].level, DiagnosticStatusStub.ERROR)
        self.assertEqual(data.status[0].message, 'not_ready')
        self.assertEqual(w.motion_sequence, 1)

    def test_worker_round_trip_and_throttle(self):
        w = self.worker()
        data = self.publish(w)
        monitor = mf.FeedbackMonitor()
        self.assertTrue(monitor.receive(data,20.,100.123))
        self.assertTrue(monitor.snapshot(20.,100.123)['drive_wait'])
        self.publish(w,20.02)
        w.motion_pub.publish.assert_called_once()
        data = self.publish(w,20.06)
        self.assertEqual(dict((v.key,v.value) for v in data.status[0].values)['sequence'],'2')

    def test_not_ready_on_guard_focus_stop_and_cleanup(self):
        for change in ('guard','focus','stop','ready'):
            w = self.worker()
            if change == 'guard':w.guard.side_effect=RuntimeError('hardware fault')
            elif change == 'focus':w.packet['focused']=False
            elif change == 'stop':w.stop.set()
            else:w.ready=False
            with self.subTest(change=change):
                data=self.publish(w)
                self.assertEqual(data.status[0].level,DiagnosticStatusStub.ERROR)
                self.assertEqual(data.status[0].message,'not_ready')
        w=self.worker();w.owned=False
        with patch.object(w,'publish_motion_state') as publish:
            self.assertTrue(w.cleanup())
            publish.assert_called_once_with(force=True)
            self.assertFalse(w.ready)

    def test_publication_error_latches_worker_stop(self):
        w=self.worker()
        w.motion_pub.publish.side_effect=RuntimeError('DDS failure')
        self.publish(w)
        self.assertTrue(w.stop.is_set())
        self.assertIn('DDS failure',w.input_error)

    def test_manual_worker_has_no_diagnostic_publisher_or_dependencies(self):
        w=rw.Worker.__new__(rw.Worker)
        w.motion_pub=None
        w.publish_motion_state()
        tree=ast.parse(Path(rw.__file__).read_text())
        load=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='load_ros')
        diagnostic=next(n for n in ast.walk(load) if isinstance(n,ast.ImportFrom) and n.module=='diagnostic_msgs.msg')
        branch=next(n for n in load.body if isinstance(n,ast.If) and isinstance(n.test,ast.Name) and n.test.id=='navigation')
        self.assertIn(diagnostic,branch.body)

    def test_ssh_bundle_loads_feedback_before_worker_without_executing_worker(self):
        args=NS(linear=.1,angular=.2,navigation=True,check=False,prepare_check=False)
        script=shlex.split(transport.remote_command(args))[2]
        loader=shlex.split(script.splitlines()[-1])[-1]
        tree=ast.parse(loader)
        # Only remove the final invocation; all bundled dependency code runs offline.
        self.assertIn("bundle['robot_worker.py']",ast.unparse(tree.body[-1]))
        tree.body.pop()
        with patch.dict(sys.modules), patch.object(sys,'argv',[]):
            scope={}
            exec(compile(tree,'offline_bundle','exec'),scope)
            self.assertIn('motion_feedback',sys.modules)
            self.assertTrue(hasattr(sys.modules['motion_feedback'],'FeedbackMonitor'))


if __name__=='__main__':
    unittest.main()
