"""Offline sender tests: no PICO SDK construction, SSH, ROS or motors."""
import copy
import contextlib
import io
import json
import os
from pathlib import Path
import queue
import sys
import subprocess
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.pico4.native_ik_test.client import InputTargets, checked_args, drain_stop, operate
from scripts.pico4.native_ik_test.transport import source_bundle, remote_command
from tcp_core import tcp_forward_matrix
from scripts.pico4.native_ik_test.native_input import NativeClutch


class Kinematics:
    def forward_tcp(self, q, side):
        return tcp_forward_matrix(q), 0.


def hardware():
    import math
    q = [math.radians(v) for v in (45, 75, -100, 85, 135, 10, 10)]*2+[0.]*6
    return dict(position=q[:], command_position=q[:], velocity=[0.]*20)


def frame(now=10., grip=0., position=0.):
    hand = dict(pose=[position, 0., 0., 0., 0., 0., 1.], grip=grip,
                primary=False, secondary=False)
    return dict(stamp=now*1e9, received_monotonic=now,
                left=copy.deepcopy(hand), right=copy.deepcopy(hand))


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.args = checked_args(['--interpolation-delay-ms', '0'])
        self.controls = InputTargets(self.args)
        self.status = dict(stamp=9.99, hardware=hardware(), ready=True,
                           native_test=dict(need_rearm=False))
        self.kin = Kinematics()

    def request(self, f):
        return self.controls.request(f, self.status, self.kin, f['received_monotonic'])

    def test_default_is_read_only_and_no_guard_disable_switch(self):
        self.assertFalse(self.args.execute)
        self.assertFalse(hasattr(self.args, 'speed_deg_s'))
        self.assertFalse(hasattr(self.args, 'acceleration_deg_s2'))
        self.assertFalse(hasattr(self.args, 'braking_deg_s2'))
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            checked_args(['--no-wrist-guard'])

    def test_requires_neutral_after_start_and_pause(self):
        self.assertIsNone(self.request(frame(grip=1)))
        self.assertIsNone(self.request(frame()))
        self.assertIsNotNone(self.request(frame(grip=1)))
        self.controls.pause()
        self.assertIsNone(self.request(frame(grip=1)))

    def test_preserves_full_unsolved_tcp_during_grip(self):
        self.request(frame())
        initial = self.request(frame(grip=1))
        moved = self.request(frame(10.01, 1., .01))
        repeated = self.request(frame(10.02, 1., .01))
        self.assertNotEqual(initial['goals'], moved['goals'])
        self.assertEqual(moved['goals'], repeated['goals'])
        self.assertNotIn('arms', moved)

    def test_one_side_release_removes_only_that_goal(self):
        self.request(frame())
        self.request(frame(grip=1))
        f = frame(10.01, 1.)
        f['left']['grip'] = 0.
        result = self.request(f)
        self.assertEqual(set(result['goals']), {'right'})

    def test_all_release_is_none_to_trigger_server_brake(self):
        self.request(frame())
        self.request(frame(grip=1))
        self.assertIsNone(self.request(frame(10.02)))

    def test_single_arm_option(self):
        self.args.arm = 'left'
        self.request(frame())
        self.assertEqual(set(self.request(frame(grip=1))['goals']), {'left'})

    def test_continuous_rotation_domain_only_applies_to_selected_arm(self):
        args = checked_args(['--arm', 'left', '--mode', 'mirror_same_direction'])
        controls = InputTargets(args)
        self.assertIsInstance(controls.clutches['left'], NativeClutch)
        self.assertNotIsInstance(controls.clutches['right'], NativeClutch)
        self.assertEqual(controls.hands['left'], 'right')

    def test_old_pico_frame_is_not_given_new_lease(self):
        self.request(frame())
        f = frame(9.95, 1.)
        result = self.controls.request(f, self.status, self.kin, 10.)
        self.assertAlmostEqual(result['created'], 9.94)
        with self.assertRaises(ValueError):
            self.controls.request(f, self.status, self.kin, 10.2)
        self.assertTrue(self.controls.paused)

    def run_operate_with_callback_frames(self, frame_age=0., startup_check_delay=.004):
        """Run the real sender loop with frames delivered after each tick starts."""
        clock = [10.]
        frames = []
        sent = []
        connection = SimpleNamespace(events=queue.Queue(), done=threading.Event(),
            latest=copy.deepcopy(self.status), last_status=clock[0],
            wrist=SimpleNamespace(ready=lambda _: True, released=True),
            send=lambda **packet: sent.append(packet))

        def read():
            clock[0] += .002  # The callback ran after operate() sampled its tick time.
            value = frame(clock[0]-frame_age, grip=0. if not frames else 1.)
            frames.append(value)
            return value

        def startup_check(*_):
            clock[0] += startup_check_delay
            return {'verified': True}, {}

        self.args.execute = True
        actions = io.StringIO()
        with patch('scripts.pico4.native_ik_test.client.time.monotonic', lambda: clock[0]), \
                patch('scripts.pico4.native_ik_test.client.time.sleep'), \
                patch('scripts.pico4.native_ik_test.client.key', side_effect=['', '', 'q']), \
                patch('scripts.pico4.native_ik_test.client.verify_both', startup_check), \
                contextlib.redirect_stdout(io.StringIO()):
            operate(self.args, self.kin, SimpleNamespace(read=read), connection, actions)
        return sent, [json.loads(line) for line in actions.getvalue().splitlines()]

    def test_operate_accepts_callback_frame_newer_than_loop_start(self):
        sent, events = self.run_operate_with_callback_frames()
        self.assertEqual(len(sent), 2)
        self.assertEqual(sent[0]['kinematics'], {'verified': True})
        self.assertIsNone(sent[0]['request'])  # Neutral frame releases the local latch.
        self.assertEqual(set(sent[1]['request']['goals']), {'left', 'right'})
        self.assertFalse(any(event['event'] == 'input_pause' for event in events))

    def test_operate_still_rejects_expired_frame_and_records_signed_age(self):
        sent, events = self.run_operate_with_callback_frames(frame_age=.151)
        self.assertTrue(all(packet['request'] is None for packet in sent))
        self.assertTrue(all(packet['kinematics'] is None for packet in sent))
        pause = next(event for event in events if event['event'] == 'input_pause')
        self.assertAlmostEqual(pause['frame_age_ms'], 155.)
        self.assertEqual(pause['input_timeout_ms'], 150.)
        self.assertEqual(pause['grip'], {'left': 0., 'right': 0.})
        self.assertTrue(pause['client_paused'])

    def test_operate_counts_startup_check_time_in_frame_age(self):
        sent, events = self.run_operate_with_callback_frames(startup_check_delay=.151)
        self.assertIsNone(sent[0]['request'])
        self.assertIsNone(sent[0]['kinematics'])
        self.assertIsNone(sent[1]['request'])  # Held Grip cannot clear the expired-frame latch.
        pause = next(event for event in events if event['event'] == 'input_pause')
        self.assertAlmostEqual(pause['frame_age_ms'], 151.)

    def test_future_frame_timestamp_is_still_rejected(self):
        with self.assertRaises(ValueError):
            self.controls.request(frame(10.001), self.status, self.kin, 10.)
        self.assertTrue(self.controls.paused)

    def test_rearm_and_wrist_ack_block_goals(self):
        self.request(frame())
        self.status['native_test']['need_rearm'] = True
        self.assertIsNone(self.request(frame(grip=1)))
        self.status['native_test']['need_rearm'] = False
        self.assertIsNone(self.request(frame(grip=1)))
        self.request(frame())
        self.assertIsNone(self.controls.request(frame(grip=1), self.status,
                                                self.kin, 10., wrist_ready=False))

    def test_does_not_rearm_before_measured_stop(self):
        self.status['hardware']['velocity'][0] = .01
        self.assertIsNone(self.request(frame()))
        self.assertTrue(self.controls.paused)

    def test_reject_invalid_motion_limits(self):
        for argv in (['--speed-deg-s', '10'], ['--command-hz', '450'],
                     ['--braking-deg-s2', '30'], ['--sensitivity', '2'],
                     ['--acceleration-deg-s2', '30']):
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                checked_args(argv)

    def test_exit_during_compile_never_fills_unread_stdin(self):
        class Compiling:
            latest = {}
            def send(self, **kwargs):
                raise AssertionError('remote stdin reader has not started')
        self.assertTrue(drain_stop(Compiling()))

    def test_bundle_has_exact_native_sources_and_no_production_client(self):
        bundle = source_bundle()
        self.assertIn('tools/ik_experiments/art7_native.cpp', bundle)
        self.assertIn('scripts/pico4/native_ik_test/server.py', bundle)
        self.assertNotIn('scripts/pico4/fairino/vr_client.py', bundle)
        self.assertNotIn('scripts/pico4/native_ik_test/servo.py', bundle)
        self.assertNotIn('tools/ik_experiments/dynamic_wrist_policy.py', bundle)
        for name, contents in bundle.items():
            self.assertEqual(contents, (ROOT/name).read_text())

    def test_remote_command_quotes_json_and_keeps_default_read_only(self):
        command = remote_command(self.args)
        self.assertIn('--check', command)
        self.assertLess(len(command.encode()), 120000)
        self.args.execute = True
        self.assertNotIn('--check', remote_command(self.args))

    def test_remote_source_tree_loads_without_pc_modules_or_ros(self):
        with tempfile.TemporaryDirectory(prefix='native-bundle-offline-') as name:
            for relative, source in source_bundle().items():
                target = Path(name)/relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(source)
            script = '''
from scripts.pico4.native_ik_test.server import arguments
from scripts.pico4.native_ik_test.planner import NativePlanner
from math import radians
p=NativePlanner()
q=[radians(v) for v in (45,75,-100,85,135,10,10)]*2+[0.]*6
hw=dict(position=q,command_position=q,velocity=[0.]*20)
p.reset(hw)
proposal=p.propose({},hw,.01)
p.commit(proposal)
assert proposal['command_stopped']
assert arguments(['--check']).check
print('bundle_native_ok')
'''
            env = dict(os.environ, PYTHONPATH=name, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
            completed = subprocess.run([sys.executable, '-c', script], cwd=name, env=env,
                capture_output=True, text=True, timeout=90)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn('bundle_native_ok', completed.stdout)


if __name__ == '__main__':
    unittest.main()
