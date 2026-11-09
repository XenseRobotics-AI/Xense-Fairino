"""Bounded preflight/preview retries cannot resend a navigation motion goal."""
import copy
from collections import deque
import json
import math
from pathlib import Path
import sys
import subprocess
import tempfile
import threading
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from fairino_slam.navigation import navigation_remote as remote
from tests.navigation.test_heading import STATE, transform, write_heading_mode


class Future:
    def __init__(self, value, delay=0., callback=None):
        self.value, self.delay, self.callback = value, delay, callback
        self.complete = False

    def done(self):
        return self.complete

    def result(self):
        return self.value


class NavigationRetryTests(unittest.TestCase):
    def setUp(self):
        self.clock = 100.
        self.state = copy.deepcopy(STATE)
        self.session = {'id': 'A', 'active': True, 'initial_pose_stamp': 9.,
                        'map': '/maps/A/map.yaml', 'epoch': 'epoch'}
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.session_file = Path(temporary.name)/'session.json'
        self.deployed_folder = Path(temporary.name)
        write_heading_mode(self.deployed_folder)
        self.patch(remote, 'HERE', self.deployed_folder)
        self.save_session()
        self.patch(remote, 'SESSION_FILE', self.session_file)
        self.patch(remote, 'status', side_effect=lambda: copy.deepcopy(self.state))
        self.patch(remote.time, 'monotonic', side_effect=lambda: self.clock)
        self.sent, self.handles, self.captures = [], [], []
        self.outcomes = []
        self.server_delay = 0.
        self.cancel_delay = 0.
        self.cancel_error = False
        self.cancel_count = 0
        self.capture_delay = 0.
        self.after_spin = None

        self.node = Mock()
        self.node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
        self.node.get_clock.return_value.now.return_value.to_msg.return_value = NS(sec=10, nanosec=0)
        self.rclpy = ModuleType('rclpy')
        self.rclpy.init = Mock()
        self.rclpy.shutdown = Mock()
        self.rclpy.create_node = Mock(return_value=self.node)
        self.rclpy.spin_once = Mock(side_effect=self.spin_once)
        self.rclpy.spin_until_future_complete = Mock(side_effect=self.spin_future)
        action = ModuleType('rclpy.action')
        self.client = Mock()
        self.client.wait_for_server.side_effect = self.server_ready
        self.client.send_goal_async.side_effect = self.send
        action.ActionClient = Mock(return_value=self.client)
        nav = ModuleType('nav2_msgs.action')
        def pose():
            return NS(header=NS(frame_id='', stamp=None),
                      pose=NS(position=NS(x=0., y=0., z=0.),
                              orientation=NS(x=0., y=0., z=0., w=1.)))
        nav.ComputePathToPose = NS(Goal=lambda: NS(goal=pose()))
        nav.NavigateToPose = NS(Goal=lambda: NS(pose=pose()))
        services = ModuleType('action_msgs.srv')
        services.CancelGoal = NS(Request=lambda: NS())
        time_module = ModuleType('rclpy.time')
        time_module.Time = Mock()
        self.tf2 = ModuleType('tf2_ros')
        self.tf2.TransformException = type('TransformException', (Exception,), {})
        self.tf2.LookupException = type('LookupException', (self.tf2.TransformException,), {})
        self.tf2.Buffer = Mock(return_value=Mock())
        self.tf2.Buffer.return_value.lookup_transform.return_value = transform()
        self.receiver = NS(buffer=self.tf2.Buffer.return_value,
                           status=Mock(return_value={'running': True, 'error': None}), close=Mock())
        self.receiver_factory = self.patch(remote, 'create_heading_receiver', return_value=self.receiver)
        self.modules = {'rclpy': self.rclpy, 'rclpy.action': action, 'rclpy.time': time_module,
                        'nav2_msgs.action': nav, 'action_msgs.srv': services,
                        'tf2_ros': self.tf2}
        patcher = patch.dict(sys.modules, self.modules)
        patcher.start()
        self.addCleanup(patcher.stop)

    def patch(self, target, name, *args, **kwargs):
        patcher = patch.object(target, name, *args, **kwargs)
        result = patcher.start()
        self.addCleanup(patcher.stop)
        return result

    def save_session(self):
        self.session_file.write_text(json.dumps(self.session))

    def spin_once(self, node, timeout_sec):
        self.clock += timeout_sec
        if self.after_spin:
            self.after_spin()

    def spin_future(self, node, future, timeout_sec):
        self.clock += min(future.delay, timeout_sec)
        future.complete = future.delay <= timeout_sec
        if future.complete and future.callback:
            future.callback()

    def server_ready(self, timeout_sec):
        self.clock += min(self.server_delay, timeout_sec)
        return self.server_delay <= timeout_sec

    def send(self, goal):
        outcome = self.outcomes.pop(0) if self.outcomes else {}
        self.sent.append((self.clock, goal))
        points = [] if outcome.get('empty') else [NS(pose=NS(position=NS(x=1., y=2.)))]
        result = NS(status=outcome.get('status', 4), result=NS(path=NS(poses=points)))
        handle = NS(accepted=outcome.get('accepted', True), goal_id=NS(uuid=bytes(range(16))))
        handle.get_result_async = Mock(return_value=Future(
            result, outcome.get('result_delay', 0.), outcome.get('after_result')))
        handle.cancel_goal_async = Mock(side_effect=self.cancel)
        self.handles.append(handle)
        return Future(handle, outcome.get('ack_delay', 0.))

    def cancel(self):
        self.cancel_count += 1
        if self.cancel_error:
            raise RuntimeError('cancel failed')
        return Future(NS(goals_canceling=[]), self.cancel_delay)

    def capture(self, node, expected, *, deadline, receiver):
        self.clock += self.capture_delay
        sample = {'x': 0., 'y': 0., 'yaw': .2+len(self.captures)*.1, 'stamp': 9.9}
        self.captures.append((expected, deadline, sample, receiver))
        self.receiver.buffer.lookup_transform.return_value = transform(sample['yaw'], sample['stamp'])
        return sample

    def invoke(self, action='plan', real_capture=False, extra=()):
        with patch.object(sys, 'argv', ['navigation_remote', action, '--x', '1', '--y', '2', *extra]):
            if real_capture:
                return remote.main()
            with patch.object(remote, 'capture_heading', side_effect=self.capture):
                return remote.main()


    def saved_args(self):
        selected = self.deployed_folder/'map.yaml'
        selected.write_text('image: map.pgm\n')
        self.patch(remote, 'SAVED_MAP', selected)
        (self.deployed_folder/'navigate_to_saved_pose.xml').write_text('<root/>')
        return ('--saved-pose', '1', '--yaw', '1.2',
                '--saved-map-source', str(selected.resolve()),
                '--saved-map-modified', str(selected.stat().st_mtime))

    def test_saved_goal_preserves_yaw_and_uses_two_stage_tree_once(self):
        args = self.saved_args()
        result = self.invoke('goal', extra=args)
        self.assertEqual(len(self.sent), 1)
        goal = self.sent[0][1]
        self.assertEqual(goal.behavior_tree, str(self.deployed_folder/'navigate_to_saved_pose.xml'))
        self.assertAlmostEqual(goal.pose.pose.orientation.z, math.sin(.6))
        self.assertEqual(result['goal_yaw'], 1.2)
        self.assertFalse(result['requested_yaw_ignored'])
        self.assertIn('再对齐目标朝向', result['message'])

    def test_manual_goal_uses_arrival_heading_tree_without_saved_map_metadata(self):
        (self.deployed_folder/'navigate_to_saved_pose.xml').write_text('<root/>')
        result = self.invoke('goal', extra=('--arrival-heading','1','--yaw','-1.0'))
        self.assertEqual(len(self.sent), 1)
        goal = self.sent[0][1]
        self.assertEqual(goal.behavior_tree, str(self.deployed_folder/'navigate_to_saved_pose.xml'))
        self.assertAlmostEqual(goal.pose.pose.orientation.z, math.sin(-.5))
        self.assertEqual(result['goal_yaw'], -1.)
        self.assertFalse(result['requested_yaw_ignored'])

    def test_manual_arrival_heading_rejects_missing_yaw_and_undeployed_tree(self):
        with self.assertRaises(RuntimeError):
            self.invoke('goal', extra=('--arrival-heading','1'))
        with self.assertRaisesRegex(RuntimeError, '尚未部署'):
            self.invoke('goal', extra=('--arrival-heading','1','--yaw','0'))
        self.assertEqual(self.sent, [])

    def test_saved_preview_has_saved_yaw_but_never_dispatches_motion(self):
        args = self.saved_args()
        result = self.invoke('plan', extra=args)
        self.assertTrue(hasattr(self.sent[0][1], 'goal'))
        self.assertFalse(hasattr(self.sent[0][1], 'behavior_tree'))
        self.assertEqual(result['goal_yaw'], 1.2)

    def test_saved_goal_rejects_changed_map_and_missing_components_before_send(self):
        args = self.saved_args()
        with self.assertRaisesRegex(RuntimeError, '地图版本'):
            self.invoke('goal', extra=(*args[:-1], '1'))
        self.assertEqual(self.sent, [])
        (self.deployed_folder/'navigate_to_saved_pose.xml').unlink()
        with self.assertRaisesRegex(RuntimeError, '尚未部署'):
            self.invoke('goal', extra=args)
        self.assertEqual(self.sent, [])

    def test_saved_map_is_rechecked_after_blocking_preflight(self):
        args = self.saved_args()
        checks = Mock(side_effect=[None, RuntimeError('地图已变化')])
        with patch.object(remote, 'require_saved_map', checks):
            with self.assertRaisesRegex(RuntimeError, '地图已变化'):
                self.invoke('goal', extra=args)
        self.assertEqual(self.sent, [])
        self.assertEqual(checks.call_count, 2)

    def stale_tf(self):
        reason = '定位 TF 已过期 1.100 秒：map → base_footprint；要求时间差小于 1.0 秒'
        self.state.update(localization_ready=False, localization_reason=reason,
                          localization_health={'reason': reason},
                          tf={'ready': False, 'initial_pose_stamp': 9., 'reason': reason,
                              'transforms': {'map_base': {'stamp': 9.1, 'age': 1.1}}})

    def test_confirmed_preview_abort_retries_then_succeeds_with_fresh_heading(self):
        self.outcomes = [{'status': 6}, {'status': 4}]
        result = self.invoke()
        self.assertEqual(result['attempts'], 2)
        self.assertEqual(len(self.sent), 2)
        self.assertGreaterEqual(self.sent[1][0]-self.sent[0][0], remote.PREVIEW_RETRY_DELAY)
        self.assertEqual(len(self.captures), 2)
        self.assertAlmostEqual(result['locked_yaw'], .3)
        self.assertEqual(self.cancel_count, 0)

    def test_path_tangent_preview_retry_keeps_target_yaw_across_new_tf_samples(self):
        write_heading_mode(self.deployed_folder, 'path_tangent')
        self.outcomes = [{'status': 6}, {'status': 4}]
        result = self.invoke(extra=('--heading-mode', 'path_tangent', '--yaw', '1.2'))
        self.assertEqual(len(self.sent), 2)
        self.assertNotEqual(self.captures[0][2]['yaw'], self.captures[1][2]['yaw'])
        for _, goal in self.sent:
            self.assertAlmostEqual(goal.goal.pose.orientation.z, math.sin(.6))
            self.assertAlmostEqual(goal.goal.pose.orientation.w, math.cos(.6))
        self.assertEqual(result['goal_yaw'], 1.2)
        self.assertEqual(result['heading_policy'], 'path_tangent')
        self.assertFalse(result['requested_yaw_ignored'])

    def test_path_tangent_requires_live_tf_and_never_sends_when_it_stays_stale(self):
        write_heading_mode(self.deployed_folder, 'path_tangent')
        self.stale_tf()
        with self.assertRaisesRegex(RuntimeError, '等待已结束'):
            self.invoke('goal', real_capture=True,
                        extra=('--heading-mode', 'path_tangent', '--yaw', '1.2'))
        self.assertEqual(self.sent, [])
        self.assertAlmostEqual(self.clock-100., remote.TF_READY_WAIT)

    def test_deployed_policy_change_after_abort_prevents_resend(self):
        self.outcomes = [{'status': 6, 'after_result': lambda:
                          write_heading_mode(self.deployed_folder, 'path_tangent')}]
        with self.assertRaisesRegex(RuntimeError, '策略不一致'):
            self.invoke()
        self.assertEqual(len(self.sent), 1)

    def test_path_tangent_ack_timeout_does_not_resend_motion_goal(self):
        write_heading_mode(self.deployed_folder, 'path_tangent')
        self.outcomes = [{'ack_delay': 8.}]
        with self.assertRaisesRegex(RuntimeError, '导航请求超时'):
            self.invoke('goal', extra=('--heading-mode', 'path_tangent', '--yaw', '1.2'))
        self.assertEqual(len(self.sent), 1)

    def test_preview_stops_after_three_confirmed_aborts(self):
        self.outcomes = [{'status': 6}]*4
        with self.assertRaisesRegex(RuntimeError, '3/3'):
            self.invoke()
        self.assertEqual(len(self.sent), 3)

    def test_other_result_statuses_and_empty_success_are_not_retried(self):
        for outcome in ({'status': 0}, {'status': 1}, {'status': 5},
                        {'status': 4, 'empty': True}, {'accepted': False}):
            with self.subTest(outcome=outcome):
                self.outcomes = [outcome]
                self.sent.clear()
                with self.assertRaises(RuntimeError):
                    self.invoke()
                self.assertEqual(len(self.sent), 1)

    def test_goal_is_sent_once_and_does_not_wait_for_or_retry_motion_result(self):
        self.outcomes = [{'status': 6}]
        self.invoke('goal')
        self.assertEqual(len(self.sent), 1)
        self.handles[0].get_result_async.assert_not_called()

    def test_ack_timeout_never_retries_preview_or_motion_goal(self):
        for action in ('plan', 'goal'):
            with self.subTest(action=action):
                self.outcomes = [{'ack_delay': 8.}]
                self.sent.clear()
                with self.assertRaisesRegex(RuntimeError, '导航请求超时'):
                    self.invoke(action)
                self.assertEqual(len(self.sent), 1)
                self.assertEqual(self.cancel_count, 0)

    def test_result_timeout_never_retries_even_if_cancel_acknowledged_or_fails(self):
        for cancel_error in (False, True):
            with self.subTest(cancel_error=cancel_error):
                self.cancel_error = cancel_error
                self.outcomes = [{'result_delay': 15.}]
                self.sent.clear()
                before = self.cancel_count
                with self.assertRaisesRegex(RuntimeError, '导航请求超时'):
                    self.invoke()
                self.assertEqual(len(self.sent), 1)
                self.assertEqual(self.cancel_count, before+1)

    def test_total_preview_budget_includes_server_capture_ack_and_result(self):
        self.server_delay = 5.
        self.capture_delay = 2.
        self.cancel_delay = 3.
        self.outcomes = [{'status': 6, 'result_delay': 10.}]*3
        with self.assertRaisesRegex(RuntimeError, '导航请求超时'):
            self.invoke()
        self.assertEqual(len(self.sent), 3)
        self.assertAlmostEqual(self.clock-100., remote.PREVIEW_TIMEOUT+3.)
        self.assertEqual(self.cancel_count, 1)

    def test_historical_protection_failure_does_not_block_explicit_new_goal(self):
        self.state.update(failed_goal='old', failure={'goal': 'old', 'reason': 'old failure'})
        self.invoke('goal')
        self.assertEqual(len(self.sent), 1)

    def test_new_protection_failure_after_abort_prevents_retry(self):
        def fail():
            self.state.update(failed_goal='new', failure={'goal': 'new', 'reason': 'protective stop'})
        self.outcomes = [{'status': 6, 'after_result': fail}]
        with self.assertRaisesRegex(RuntimeError, '保护故障'):
            self.invoke()
        self.assertEqual(len(self.sent), 1)

    def test_existing_goal_or_unknown_goal_status_blocks_even_preview(self):
        for goal, last in (('active', {}), (None, {'id': 'active', 'status': 1}),
                           (None, {'id': 'active', 'status': 2}),
                           (None, {'id': 'active', 'status': 3}),
                           (None, {'id': 'unknown', 'status': 0}), (None, None)):
            with self.subTest(goal=goal, last=last):
                self.state.update(goal=goal, last_goal=last)
                with self.assertRaises(RuntimeError):
                    self.invoke()
                self.assertEqual(self.sent, [])

    def test_session_map_epoch_initialization_runtime_changes_prevent_retry(self):
        for field in ('id', 'map', 'epoch', 'initial', 'run'):
            with self.subTest(field=field):
                original_state, original_session = copy.deepcopy(STATE), copy.deepcopy(self.session)
                def change():
                    if field == 'run':
                        self.state['run_id'] = 'new-run'
                    elif field == 'map':
                        self.session['map'] = '/maps/B/map.yaml'
                    elif field == 'epoch':
                        self.session['epoch'] = 'new-epoch'
                        self.state['session_token'][2] = 'new-epoch'
                    elif field == 'initial':
                        self.session['initial_pose_stamp'] = 9.2
                        self.state['session_token'][1] = 9.2
                    else:
                        self.session['id'] = self.state['session'] = 'B'
                        self.state['session_token'][0] = 'B'
                    self.save_session()
                self.outcomes = [{'status': 6, 'after_result': change}]
                self.sent.clear()
                with self.assertRaises(RuntimeError):
                    self.invoke()
                self.assertEqual(len(self.sent), 1)
                self.state, self.session = original_state, original_session
                self.save_session()

    def test_service_stop_during_retry_delay_prevents_second_send(self):
        self.outcomes = [{'status': 6}]
        self.after_spin = lambda: self.state.update(controller_ready=False)
        with self.assertRaisesRegex(RuntimeError, '控制服务未就绪'):
            self.invoke()
        self.assertEqual(len(self.sent), 1)

    def test_known_stale_gate_tf_can_recover_before_single_motion_send(self):
        self.stale_tf()
        self.after_spin = lambda: self.state.update(localization_ready=True)
        self.invoke('goal', real_capture=True)
        self.assertEqual(len(self.sent), 1)
        self.assertLess(self.clock-100., remote.TF_READY_WAIT)
        self.receiver.close.assert_called_once()

    def test_gate_and_command_tf_share_one_three_second_wait(self):
        expected = remote.heading_session()
        self.stale_tf()
        def advance():
            if self.clock >= 101.8:
                self.state['localization_ready'] = True
        self.after_spin = advance
        self.tf2.Buffer.return_value.lookup_transform.side_effect = self.tf2.LookupException('missing')
        with self.assertRaisesRegex(RuntimeError, '等待已结束'):
            remote.capture_heading(self.node, expected)
        self.assertAlmostEqual(self.clock-100., remote.TF_READY_WAIT)
        self.assertEqual(self.sent, [])
        self.receiver.close.assert_called_once()

    def test_real_receiver_drains_tf_queue_during_slow_status_checks(self):
        # Model a 50 Hz TF stream with a depth-100 DDS queue. One callback
        # after each 0.2 s status check would keep consuming samples ~2 s old.
        queue = deque((98.+.02*i for i in range(100)), maxlen=100)
        lock = threading.Lock()
        pending, drained, stopped = threading.Event(), threading.Event(), threading.Event()
        entered = threading.Event()
        callback_threads, oldest_ages = [], []
        latest = [98.]
        tf_node = NS(context=NS(ok=lambda: True), destroy_node=Mock())
        buffer = NS(lookup_transform=lambda *args: transform(.8, latest[0]))
        listener = NS(unregister=Mock())
        def spin():
            entered.set()
            while True:
                pending.wait(1.)
                pending.clear()
                if stopped.is_set():
                    return
                while True:
                    with lock:
                        if not queue:
                            break
                        latest[0] = queue.popleft()
                    callback_threads.append(threading.get_ident())
                drained.set()
        def shutdown(**kwargs):
            stopped.set()
            pending.set()
        executor = NS(add_node=Mock(), remove_node=Mock(), spin=spin, shutdown=shutdown)
        receiver = remote.TfReceiver(
            NS(create_node=lambda *args, **kwargs: tf_node),
            NS(Buffer=lambda: buffer, TransformListener=lambda *args, **kwargs: listener),
            lambda **kwargs: executor, node_name='navigation_command_tf',
            thread_name='test-navigation-heading-tf')
        self.addCleanup(receiver.close)
        self.assertTrue(entered.wait(1.))
        self.node.get_clock.return_value.now.side_effect = lambda: NS(nanoseconds=int(self.clock*1e9))
        expected = remote.heading_session()
        def slow_status():
            before = self.clock
            self.clock += .2
            with lock:
                queue.extend(before+.02*i for i in range(1, 11))
                oldest_ages.append(self.clock-queue[0])
            drained.clear()
            pending.set()
            # Main thread blocks here, like the synchronous systemctl query.
            # The action-node spin mock never dispatches these TF callbacks.
            self.assertTrue(drained.wait(1.))
            self.assertAlmostEqual(latest[0], self.clock)
            return copy.deepcopy(self.state)
        with patch.object(remote, 'status', side_effect=slow_status):
            pose = remote.capture_heading(self.node, expected, receiver=receiver)
        self.assertGreater(oldest_ages[0], remote.TF_MAX_AGE)
        self.assertAlmostEqual(pose['stamp'], self.clock)
        self.assertLess(self.clock-100., remote.TF_READY_WAIT)
        self.assertTrue(callback_threads)
        self.assertNotIn(threading.get_ident(), callback_threads)
        self.assertIsNot(receiver.node, self.node)
        self.assertTrue(receiver.status()['running'])
        receiver.close()
        self.assertFalse(receiver.thread.is_alive())

    def test_capture_reads_tf_after_last_blocking_session_check(self):
        expected = remote.heading_session()
        self.receiver.buffer.lookup_transform.return_value = transform(.2, 9.2)
        calls = []
        def check(*args, **kwargs):
            calls.append(True)
            if len(calls) == 2:
                self.clock += .2
                self.node.get_clock.return_value.now.return_value.nanoseconds = 11_000_000_000
                self.receiver.buffer.lookup_transform.return_value = transform(.8, 10.95)
            return copy.deepcopy(self.state)
        with patch.object(remote, 'check_session', side_effect=check):
            pose = remote.capture_heading(self.node, expected, receiver=self.receiver)
        self.assertAlmostEqual(pose['stamp'], 10.95)
        self.assertAlmostEqual(pose['yaw'], .8)
        self.receiver.buffer.lookup_transform.assert_called_once()
        self.receiver.close.assert_not_called()

    def test_send_refreshes_tf_after_blocking_deployed_policy_check(self):
        original = remote.require_heading_mode
        calls = []
        def mode(expected=None):
            result = original(expected)
            calls.append(result)
            if len(calls) == 2:
                self.clock += .2
                self.receiver.buffer.lookup_transform.return_value = transform(.9, 9.99)
            return result
        with patch.object(remote, 'require_heading_mode', side_effect=mode):
            result = self.invoke('goal')
        self.assertAlmostEqual(self.captures[0][2]['yaw'], .2)
        self.assertAlmostEqual(result['goal_yaw'], .9)
        self.assertAlmostEqual(result['heading_stamp'], 9.99)
        self.assertEqual(len(self.sent), 1)

    def test_receiver_starts_before_server_wait_and_is_shared_until_final_cleanup(self):
        events = []
        self.receiver_factory.side_effect = lambda: events.append('receiver') or self.receiver
        self.client.wait_for_server.side_effect = lambda **kwargs: events.append('server') or True
        self.receiver.close.side_effect = lambda: events.append('close')
        self.node.destroy_node.side_effect = lambda: events.append('destroy_command')
        self.rclpy.shutdown.side_effect = lambda: events.append('shutdown')
        self.outcomes = [{'status': 6}, {'status': 4}]
        self.invoke()
        self.receiver_factory.assert_called_once()
        self.assertTrue(all(item[3] is self.receiver for item in self.captures))
        self.assertEqual(events[:2], ['receiver', 'server'])
        self.assertEqual(events[-3:], ['close', 'destroy_command', 'shutdown'])
        self.receiver.close.assert_called_once()

    def test_receiver_is_closed_when_action_service_is_unavailable(self):
        self.server_delay = 6.
        with self.assertRaisesRegex(RuntimeError, '导航动作服务不可用'):
            self.invoke()
        self.receiver.close.assert_called_once()
        self.node.destroy_node.assert_called_once()
        self.rclpy.shutdown.assert_called_once()
        self.assertEqual(self.sent, [])

    def test_dead_receiver_does_not_reuse_cached_transform(self):
        expected = remote.heading_session()
        self.receiver.status.return_value = {'running': False, 'error': 'executor failed'}
        with self.assertRaisesRegex(RuntimeError, '接收器未运行'):
            remote.capture_heading(self.node, expected)
        self.receiver.buffer.lookup_transform.assert_not_called()
        self.receiver.close.assert_called_once()

    def test_stale_tf_timeout_reports_age_and_exact_initialization_values(self):
        expected = remote.heading_session()
        self.receiver.buffer.lookup_transform.return_value = transform(stamp=9.)
        with self.assertRaisesRegex(RuntimeError, '无法锁定当前朝向') as error:
            remote.capture_heading(self.node, expected)
        for text in ('当前定位 TF 已过期', 'age=1.000000s', 'stamp=9.000000', 'initial=9.000000'):
            self.assertIn(text, str(error.exception))
        self.assertNotIn('早于本次初始化', str(error.exception))
        self.assertAlmostEqual(self.clock-100., remote.TF_READY_WAIT)
        self.receiver.close.assert_called_once()

    def test_preinit_future_and_invalid_clock_fail_immediately_without_wait_prefix(self):
        expected = remote.heading_session()
        for stamp, now, message in ((8.9, 10., '早于本次初始化'),
                                     (10.1, 10., '时间超前'), (9.9, 0., '时钟无效')):
            with self.subTest(message=message):
                self.receiver.buffer.lookup_transform.return_value = transform(stamp=stamp)
                self.node.get_clock.return_value.now.return_value.nanoseconds = int(now*1e9)
                before = self.clock
                with self.assertRaisesRegex(RuntimeError, message) as error:
                    remote.capture_heading(self.node, expected, receiver=self.receiver)
                self.assertNotIn('无法锁定', str(error.exception))
                self.assertLess(self.clock-before, .1)
                self.assertEqual(self.sent, [])

    def test_invalid_or_unknown_localization_failure_is_not_waited_or_sent(self):
        variants = [
            {'localization_ready': False, 'localization_reason': '雷达或里程计断流'},
            {'running': False}, {'stale': True}, {'controller_ready': False},
            {'localization_ready': None},
        ]
        for update in variants:
            with self.subTest(update=update):
                self.state = {**copy.deepcopy(STATE), **update}
                before = self.clock
                with self.assertRaises(RuntimeError):
                    self.invoke(real_capture=True)
                self.assertEqual(self.sent, [])
                self.assertEqual(self.clock, before)

    def test_tf_prefix_without_known_structured_evidence_is_not_retryable(self):
        self.stale_tf()
        self.state['tf']['transforms']['map_base']['age'] = float('nan')
        with self.assertRaises(RuntimeError):
            remote.heading_session(allow_tf_wait=True)
        self.stale_tf()
        self.state['localization_health']['reason'] = '雷达或里程计断流'
        with self.assertRaises(RuntimeError):
            remote.heading_session(allow_tf_wait=True)

    def test_invalid_buffer_tf_aborts_without_using_full_wait(self):
        expected = remote.heading_session()
        invalid = transform()
        invalid.header.frame_id = 'odom'
        self.tf2.Buffer.return_value.lookup_transform.return_value = invalid
        with self.assertRaisesRegex(RuntimeError, '坐标系或位姿无效'):
            remote.capture_heading(self.node, expected)
        self.assertLess(self.clock-100., .1)

    def test_session_or_failure_change_during_heading_wait_aborts(self):
        for change in (lambda: self.state.update(run_id='new-run'),
                       lambda: self.state.update(failed_goal='new', failure={'goal': 'new'})):
            with self.subTest(change=change):
                self.state = copy.deepcopy(STATE)
                expected = remote.heading_session()
                self.stale_tf()
                self.after_spin = change
                with self.assertRaisesRegex(RuntimeError, '会话已变化'):
                    remote.capture_heading(self.node, expected)
                self.assertEqual(self.sent, [])


class StatusProbeTests(unittest.TestCase):
    def test_systemctl_probe_has_short_timeout_and_unknown_state_fails_closed(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(remote, 'HERE', Path(folder)), patch.object(
                    remote.subprocess, 'run',
                    side_effect=subprocess.TimeoutExpired('systemctl', 1.)) as run:
                with self.assertRaisesRegex(RuntimeError, '状态未知'):
                    remote.status()
                self.assertEqual(run.call_args.kwargs['timeout'], 1.)



if __name__ == '__main__':
    unittest.main()
