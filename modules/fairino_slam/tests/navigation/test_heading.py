"""Explicit terminal headings and held headings both require validated live TF."""
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from fairino_slam.navigation import client, navigation_remote as remote


def transform(yaw=.7, stamp=9.9):
    return NS(header=NS(frame_id='map', stamp=NS(sec=int(stamp), nanosec=round((stamp % 1)*1e9))),
              child_frame_id='base_footprint',
              transform=NS(translation=NS(x=1., y=2., z=0.),
                           rotation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))))


SESSION = {'id': 'A', 'map': '/maps/A/map.yaml', 'initial_pose_stamp': 9.,
           'odometry_epoch': 'epoch', 'run_id': 'run', 'protection': '[null, null]'}
STATE = {'session': 'A', 'session_token': ['A', 9., 'epoch'], 'run_id': 'run',
         'localization_ready': True, 'controller_ready': True,
         'running': True, 'stale': False, 'goal': None, 'last_goal': {},
         'failed_goal': None, 'failure': None}


def write_heading_mode(folder, mode='hold_initial'):
    (Path(folder)/'nav2.yaml').write_text(json.dumps({
        'controller_server': {'ros__parameters': {'FollowPath': {'heading_mode': mode}}}}))


class ChassisMotionPreflightTests(unittest.TestCase):
    def test_feedback_failure_is_explicit_in_readiness_and_session_preflight(self):
        state = {**STATE, 'controller_ready': False,
                 'chassis_motion': {'ready': False, 'reason': '底盘运动反馈已过期'}}
        with patch.object(remote, 'status', return_value=state):
            for operation in (remote.require_ready, remote.session_snapshot):
                with self.subTest(operation=operation.__name__), \
                     self.assertRaisesRegex(RuntimeError, '底盘协调尚未就绪：底盘运动反馈已过期'):
                    operation()

    def test_activation_rejects_feedback_before_writing_any_request(self):
        state = {**STATE, 'chassis_motion': {'ready': False, 'reason': '缺少底盘运动反馈'}}
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(remote, 'HERE', Path(folder)), \
             patch.object(remote, 'status', return_value=state), \
             patch.object(remote.uuid, 'uuid4') as request_id:
            with self.assertRaisesRegex(RuntimeError, '缺少底盘运动反馈'):
                remote.activate()
            request_id.assert_not_called()
            self.assertFalse((Path(folder)/'activation-request.json').exists())

    def test_ready_feedback_and_legacy_status_are_accepted(self):
        for state in (STATE, {**STATE, 'chassis_motion': {'ready': True}}):
            with self.subTest(state=state), patch.object(remote, 'status', return_value=state):
                self.assertEqual(remote.require_ready(), state)

    def test_malformed_present_feedback_fails_closed(self):
        for feedback in (None, [], {}, {'ready': 1}, {'ready': False, 'reason': []}):
            with self.subTest(feedback=feedback), \
                 self.assertRaisesRegex(RuntimeError, '底盘协调尚未就绪'):
                remote.require_chassis_motion({**STATE, 'chassis_motion': feedback})

    def test_prepare_does_not_require_worker_feedback(self):
        state = {**STATE, 'prepared': True,
                 'chassis_motion': {'ready': False, 'reason': '尚未接入底盘'}}
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(Path, 'home', return_value=Path(folder)), \
             patch.object(remote, 'status', return_value=state), \
             patch.object(remote.subprocess, 'run') as run, \
             patch('fairino_slam.navigation.lifecycle.configure') as configure, \
             patch.object(remote.time, 'monotonic', side_effect=(0., 0., .1, .2, 3., 3.)), \
             patch.object(remote.time, 'sleep'):
            self.assertIn('尚未激活或使能', remote.prepare()['message'])
            configure.assert_called_once()
            run.assert_called_once_with(['sudo', '-n', 'systemctl', 'start', remote.UNIT], check=True)


class DeployedHeadingModeTests(unittest.TestCase):
    def test_reads_only_explicit_known_deployed_policy(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(remote, 'HERE', Path(folder)):
            for mode in remote.HEADING_MODES:
                write_heading_mode(folder, mode)
                self.assertEqual(remote.deployed_heading_mode(), mode)
            for mode in (None, '', 'unknown', True, ['hold_initial']):
                write_heading_mode(folder, mode)
                with self.subTest(mode=mode), self.assertRaisesRegex(RuntimeError, '策略无效'):
                    remote.deployed_heading_mode()

    def test_missing_malformed_or_incomplete_config_never_defaults_to_hold(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(remote, 'HERE', Path(folder)):
            path = Path(folder)/'nav2.yaml'
            with self.assertRaisesRegex(RuntimeError, '无法读取'):
                remote.deployed_heading_mode()
            for text in ('invalid: [', '{}', 'null', '[]'):
                path.write_text(text)
                with self.subTest(text=text), self.assertRaises(RuntimeError):
                    remote.deployed_heading_mode()

    def test_omitted_expectation_only_accepts_legacy_hold_policy(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(remote, 'HERE', Path(folder)):
            write_heading_mode(folder)
            self.assertEqual(remote.require_heading_mode(), 'hold_initial')
            write_heading_mode(folder, 'path_tangent')
            with self.assertRaisesRegex(RuntimeError, '策略不一致'):
                remote.require_heading_mode()
            self.assertEqual(remote.require_heading_mode('path_tangent'), 'path_tangent')

    def test_status_reports_config_failure_without_claiming_hold_mode(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(remote, 'HERE', Path(folder)), \
             patch.object(remote.subprocess, 'run', return_value=NS(returncode=1)):
            state = remote.status()
            self.assertIsNone(state['heading_mode'])
            self.assertIn('nav2.yaml', state['heading_mode_error'])
            write_heading_mode(folder, 'path_tangent')
            state = remote.status()
            self.assertEqual(state['heading_mode'], 'path_tangent')
            self.assertNotIn('heading_mode_error', state)


class HeadingValidationTests(unittest.TestCase):
    def test_valid_pose_yields_actual_yaw(self):
        pose = remote.heading_from_transform(transform(.7), 10., 9.)
        self.assertEqual((pose['x'], pose['y']), (1., 2.))
        self.assertAlmostEqual(pose['yaw'], .7)

    def test_tf_between_old_and_new_age_limit_is_accepted(self):
        self.assertAlmostEqual(remote.heading_from_transform(transform(stamp=9.4), 10., 9.)['yaw'], .7)
        self.assertAlmostEqual(remote.heading_from_transform(transform(stamp=9.0001), 10., 9.)['yaw'], .7)

    def test_invalid_full_pose_time_frame_or_quaternion_is_rejected(self):
        cases = []
        for stamp in (0., 8.9, 9., 10.1):
            cases.append(transform(stamp=stamp))
        for attribute, value in (('x', float('nan')), ('y', float('inf')), ('z', float('nan'))):
            msg = transform()
            setattr(msg.transform.translation, attribute, value)
            cases.append(msg)
        msg = transform(); msg.transform.rotation.w = 0.; cases.append(msg)
        msg = transform(); msg.transform.rotation.z = float('nan'); cases.append(msg)
        msg = transform(); msg.header.frame_id = 'odom'; cases.append(msg)
        msg = transform(); msg.child_frame_id = 'base_link'; cases.append(msg)
        for index, msg in enumerate(cases):
            with self.subTest(index=index), self.assertRaises(ValueError):
                remote.heading_from_transform(msg, 10., 9.)

    def test_tf_failure_codes_separate_stale_preinit_future_and_bad_clock(self):
        for stamp, now, initial, code in (
                (9., 10., 9., 'stale'), (8.9, 10., 9., 'pre_initialization'),
                (10.1, 10., 9., 'future'), (9.9, float('nan'), 9., 'invalid_clock'),
                (0., 10., 9., 'invalid_stamp'), (9.9, 10., 0., 'invalid_initialization')):
            with self.subTest(code=code), self.assertRaises(remote.HeadingTransformError) as error:
                remote.heading_from_transform(transform(stamp=stamp), now, initial)
            self.assertEqual(error.exception.reason_code, code)
            for field in ('age=', 'stamp=', 'initial=', '< 1.000s'):
                self.assertIn(field, str(error.exception))

    def test_session_includes_initialization_epoch_and_runtime(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'localization.json'
            data = {'active': True, 'id': 'A', 'map': SESSION['map'],
                    'epoch': 'epoch', 'initial_pose_stamp': 9.}
            path.write_text(json.dumps(data))
            with patch.object(remote, 'SESSION_FILE', path), patch.object(remote, 'status', return_value=STATE):
                self.assertEqual(remote.heading_session('A'), SESSION)
                with self.assertRaises(RuntimeError):
                    remote.heading_session('B')
                path.write_text(json.dumps({**data, 'initial_pose_stamp': 10.}))
                with self.assertRaises(RuntimeError):
                    remote.heading_session('A')

    def test_tf_capture_spins_and_rejects_changed_session(self):
        rclpy = ModuleType('rclpy')
        rclpy.spin_once = Mock()
        time_module = ModuleType('rclpy.time'); time_module.Time = Mock()
        tf2 = ModuleType('tf2_ros')
        tf2.TransformException = type('TransformException', (Exception,), {})
        buffer = Mock(); buffer.lookup_transform.return_value = transform()
        receiver = NS(buffer=buffer, status=Mock(return_value={'running': True, 'error': None}),
                      close=Mock())
        node = Mock(); node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
        with patch.dict(sys.modules, {'rclpy': rclpy, 'rclpy.time': time_module, 'tf2_ros': tf2}), \
             patch.object(remote, 'session_snapshot', return_value=(SESSION, STATE)), \
             patch.object(remote, 'create_heading_receiver', return_value=receiver):
            self.assertAlmostEqual(remote.capture_heading(node, SESSION)['yaw'], .7)
            rclpy.spin_once.assert_called_with(node, timeout_sec=.05)
            receiver.close.assert_called_once()
        receiver.close.reset_mock()
        changed = {**SESSION, 'odometry_epoch': 'new'}
        with patch.dict(sys.modules, {'rclpy': rclpy, 'rclpy.time': time_module, 'tf2_ros': tf2}), \
             patch.object(remote, 'session_snapshot', side_effect=[(SESSION, STATE), (changed, STATE)]), \
             patch.object(remote, 'create_heading_receiver', return_value=receiver):
            with self.assertRaisesRegex(RuntimeError, '会话已变化'):
                remote.capture_heading(node, SESSION)
            receiver.close.assert_called_once()


class ActionHeadingTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.deployed_folder = Path(temporary.name)
        write_heading_mode(self.deployed_folder)
        patcher = patch.object(remote, 'HERE', self.deployed_folder)
        patcher.start(); self.addCleanup(patcher.stop)
        self.sent = []
        self.tf = transform()
        self.tf.transform.translation.x = self.tf.transform.translation.y = 0.
        self.receiver = NS(buffer=Mock(), status=Mock(return_value={'running': True, 'error': None}),
                           close=Mock())
        self.receiver.buffer.lookup_transform.return_value = self.tf
        patcher = patch.object(remote, 'create_heading_receiver', return_value=self.receiver)
        patcher.start(); self.addCleanup(patcher.stop)
        self.node = Mock()
        self.node.get_clock.return_value.now.return_value.to_msg.return_value = NS(sec=10, nanosec=0)
        self.node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
        rclpy = ModuleType('rclpy')
        rclpy.init = Mock(); rclpy.shutdown = Mock()
        rclpy.create_node = Mock(return_value=self.node)
        rclpy.spin_until_future_complete = Mock()
        action_module = ModuleType('rclpy.action')
        self.action_client = Mock()
        self.action_client.wait_for_server.return_value = True
        self.action_client.send_goal_async.side_effect = self.send
        action_module.ActionClient = Mock(return_value=self.action_client)
        nav = ModuleType('nav2_msgs.action')
        def pose():
            return NS(header=NS(frame_id='', stamp=None),
                      pose=NS(position=NS(x=0., y=0., z=0.),
                              orientation=NS(x=0., y=0., z=0., w=1.)))
        nav.NavigateToPose = NS(Goal=lambda: NS(pose=pose()))
        nav.ComputePathToPose = NS(Goal=lambda: NS(goal=pose()))
        services = ModuleType('action_msgs.srv'); services.CancelGoal = NS(Request=lambda: NS())
        time_module = ModuleType('rclpy.time'); time_module.Time = Mock()
        patcher = patch.dict(sys.modules, {'rclpy': rclpy, 'rclpy.action': action_module,
                                          'rclpy.time': time_module,
                                          'nav2_msgs.action': nav, 'action_msgs.srv': services})
        patcher.start(); self.addCleanup(patcher.stop)
        patcher = patch.object(remote, 'session_snapshot', return_value=(SESSION, STATE))
        patcher.start(); self.addCleanup(patcher.stop)

    @staticmethod
    def future(value):
        return NS(done=lambda: True, result=lambda: value)

    def send(self, goal):
        self.sent.append(goal)
        result = NS(status=4, result=NS(path=NS(poses=[NS(pose=NS(position=NS(x=1., y=2.)))])))
        handle = NS(accepted=True, goal_id=NS(uuid=bytes(range(16))),
                    get_result_async=lambda: self.future(result))
        return self.future(handle)

    def invoke(self, action='goal', extra=()):
        with patch.object(sys, 'argv', ['navigation_remote', action, '--x', '1', '--y', '2', *extra]):
            return remote.main()

    def test_goal_ignores_requested_heading_and_sends_fresh_quaternion(self):
        sample = {'x': 0., 'y': 0., 'yaw': .7, 'stamp': 9.9}
        with patch.object(remote, 'capture_heading', return_value=sample):
            result = self.invoke(extra=('--yaw', '-1.4'))
        rotation = self.sent[0].pose.pose.orientation
        self.assertAlmostEqual(rotation.z, math.sin(.35))
        self.assertAlmostEqual(rotation.w, math.cos(.35))
        self.assertEqual(result['heading_policy'], 'hold_initial')
        self.assertEqual(result['locked_yaw'], .7)
        self.assertEqual(result['goal_yaw'], .7)
        self.assertTrue(result['requested_yaw_ignored'])
        self.assertIn('忽略', result['message'])

    def test_path_tangent_uses_requested_terminal_yaw_but_still_captures_live_pose(self):
        write_heading_mode(self.deployed_folder, 'path_tangent')
        sample = {'x': 0., 'y': 0., 'yaw': .7, 'stamp': 9.9}
        with patch.object(remote, 'capture_heading', return_value=sample) as capture:
            result = self.invoke(extra=('--heading-mode', 'path_tangent', '--yaw', '-1.4'))
        capture.assert_called_once()
        rotation = self.sent[0].pose.pose.orientation
        self.assertAlmostEqual(rotation.z, math.sin(-.7))
        self.assertAlmostEqual(rotation.w, math.cos(-.7))
        self.assertEqual(result['heading_policy'], 'path_tangent')
        self.assertEqual(result['goal_yaw'], -1.4)
        self.assertEqual(result['locked_yaw'], -1.4)
        self.assertEqual(result['start_pose'], sample)
        self.assertFalse(result['requested_yaw_ignored'])

    def test_large_finite_terminal_yaw_preserves_input_without_infinite_diagnostic(self):
        write_heading_mode(self.deployed_folder, 'path_tangent')
        sample = {'x': 0., 'y': 0., 'yaw': .7, 'stamp': 9.9}
        with patch.object(remote, 'capture_heading', return_value=sample):
            result = self.invoke(extra=('--heading-mode', 'path_tangent', '--yaw', '1e308'))
        self.assertEqual(result['goal_yaw'], 1e308)
        self.assertIn('rad', result['message'])
        self.assertNotIn('inf', result['message'])
        rotation = self.sent[0].pose.pose.orientation
        self.assertTrue(math.isfinite(rotation.z) and math.isfinite(rotation.w))
        json.dumps(result, allow_nan=False)

    def test_path_tangent_requires_explicit_finite_target_yaw(self):
        write_heading_mode(self.deployed_folder, 'path_tangent')
        for yaw_args in ((), ('--yaw', 'nan'), ('--yaw', 'inf'), ('--yaw=-inf',)):
            with self.subTest(yaw=yaw_args), self.assertRaisesRegex(RuntimeError, '有限的目标 --yaw'):
                self.invoke(extra=('--heading-mode', 'path_tangent', *yaw_args))
        self.assertEqual(self.sent, [])

    def test_policy_mismatch_or_old_client_on_path_tangent_never_sends(self):
        for deployed, requested in (('hold_initial', 'path_tangent'),
                                    ('path_tangent', 'hold_initial'), ('path_tangent', None)):
            write_heading_mode(self.deployed_folder, deployed)
            extra = ('--yaw', '1.2')+(() if requested is None else ('--heading-mode', requested))
            with self.subTest(deployed=deployed, requested=requested), \
                 self.assertRaisesRegex(RuntimeError, '策略不一致'):
                self.invoke(extra=extra)
        self.assertEqual(self.sent, [])

    def test_execute_resamples_after_preview(self):
        self.receiver.buffer.lookup_transform.side_effect = [transform(.2, 9.8), transform(.6, 9.9)]
        with patch.object(remote, 'capture_heading', side_effect=[
                {'x': 0., 'y': 0., 'yaw': .2, 'stamp': 9.8},
                {'x': 0., 'y': 0., 'yaw': .6, 'stamp': 9.9}]) as capture:
            preview = self.invoke('plan')
            execute = self.invoke('goal')
        self.assertEqual(capture.call_count, 2)
        self.assertAlmostEqual(preview['locked_yaw'], .2)
        self.assertAlmostEqual(execute['locked_yaw'], .6)
        self.assertAlmostEqual(self.sent[0].goal.pose.orientation.z, math.sin(.1))
        self.assertAlmostEqual(self.sent[1].pose.pose.orientation.z, math.sin(.3))

    def test_unavailable_or_invalid_heading_never_sends_goal(self):
        with patch.object(remote, 'capture_heading', side_effect=RuntimeError('TF unavailable')):
            with self.assertRaisesRegex(RuntimeError, 'TF unavailable'):
                self.invoke()
        self.assertEqual(self.sent, [])

    def test_session_change_after_sample_never_sends_goal(self):
        changed = {**SESSION, 'run_id': 'new'}
        with patch.object(remote, 'session_snapshot', side_effect=[(SESSION, STATE), (changed, STATE)]), \
             patch.object(remote, 'capture_heading', return_value={'x': 0., 'y': 0., 'yaw': .7, 'stamp': 9.9}):
            with self.assertRaisesRegex(RuntimeError, '会话已变化'):
                self.invoke()
        self.assertEqual(self.sent, [])

    def test_sample_expired_before_action_send_is_rejected(self):
        self.receiver.buffer.lookup_transform.return_value = transform(stamp=9.)
        with patch.object(remote, 'capture_heading', return_value={
                'x': 0., 'y': 0., 'yaw': .7, 'stamp': 9.}):
            with self.assertRaisesRegex(RuntimeError, '已过期'):
                self.invoke()
        self.assertEqual(self.sent, [])

    def test_cli_accepts_xy_without_yaw(self):
        with patch.object(sys, 'argv', ['fairino-slam navigation', 'goal', '--x', '1', '--y', '2']), \
             patch.dict('os.environ', {'FAIRINO_SSH_PASSWORD': 'test'}), \
             patch.object(client, 'configured_heading_mode', return_value='hold_initial'), \
             patch.object(client, 'execute', return_value={'message': 'ok'}) as execute, \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(client.main(), 0)
        self.assertEqual(execute.call_args.args[-1],
                         {'x': 1., 'y': 2., 'heading_mode': 'hold_initial'})

    def test_path_tangent_cli_passes_explicit_policy_and_user_yaw(self):
        with patch.object(sys, 'argv', ['fairino-slam navigation', 'goal',
                                      '--x', '1', '--y', '2', '--yaw', '1.2']), \
             patch.dict('os.environ', {'FAIRINO_SSH_PASSWORD': 'test'}), \
             patch.object(client, 'configured_heading_mode', return_value='path_tangent'), \
             patch.object(client, 'execute', return_value={'message': 'ok'}) as execute, \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(client.main(), 0)
        self.assertEqual(execute.call_args.args[-1],
                         {'x': 1., 'y': 2., 'yaw': 1.2, 'heading_mode': 'path_tangent'})


if __name__ == '__main__':
    unittest.main()
