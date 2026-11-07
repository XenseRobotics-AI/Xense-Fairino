"""Recovery UI/session regression checks without GTK, SSH, or robot commands."""
import queue
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from fairino_slam.navigation.recovery import RecoverySession
from fairino_slam.ui.mapping_session import ChassisOwnership
from . import test_desktop as desktop_tests


class RecoverySessionTests(unittest.TestCase):
    def make(self, **kwargs):
        self.runner = kwargs.pop('runner', Mock(return_value={'ok': True}))
        self.lock = kwargs.pop('command_lock', threading.Lock())
        self.task = RecoverySession(
            'robot', 'fairino', 'secret', self.lock,
            kwargs.pop('local_ready', lambda: True), runner=self.runner,
            wait_timeout=kwargs.pop('wait_timeout', .4), **kwargs)
        self.addCleanup(self.finish_task)
        return self.task

    def finish_task(self):
        if self.task.thread.is_alive():
            self.task.reject('test cleanup')
            self.task.thread.join(1)

    def test_command_and_local_sessions_must_finish_before_lease_request(self):
        local_done = threading.Event()
        lock = threading.Lock()
        lock.acquire()
        task = self.make(command_lock=lock, local_ready=local_done.is_set)
        task.thread.start()
        self.assertFalse(task.ready_to_acquire.wait(.03))
        self.runner.assert_not_called()
        lock.release()
        self.assertFalse(task.ready_to_acquire.wait(.03))
        local_done.set()
        self.assertTrue(task.ready_to_acquire.wait(1))
        self.runner.assert_not_called()
        task.authorize()
        self.assertTrue(task.done.wait(1))
        self.runner.assert_called_once_with('robot', 'fairino', 'secret')
        self.assertIsNone(task.password)
        self.assertIsNone(task.error)

    def test_timeout_never_runs_recovery_or_releases_someone_elses_lock(self):
        lock = threading.Lock()
        lock.acquire()
        task = self.make(command_lock=lock, wait_timeout=.04)
        task.thread.start()
        self.assertTrue(task.done.wait(1))
        self.runner.assert_not_called()
        self.assertTrue(lock.locked())
        self.assertIn('未接管', task.error)
        lock.release()

    def test_unfinished_local_session_times_out_without_remote_call(self):
        task = self.make(local_ready=lambda: False, wait_timeout=.04)
        task.thread.start()
        self.assertTrue(task.done.wait(1))
        self.runner.assert_not_called()
        self.assertFalse(self.lock.locked())
        self.assertFalse(task.ready_to_acquire.is_set())

    def test_lease_refusal_blocks_remote_call(self):
        task = self.make()
        task.thread.start()
        self.assertTrue(task.ready_to_acquire.wait(1))
        task.reject('external owner')
        self.assertTrue(task.done.wait(1))
        self.runner.assert_not_called()
        self.assertEqual(task.error, 'external owner')

    def test_ambiguous_remote_exception_is_not_retried_and_releases_lock(self):
        task = self.make(runner=Mock(side_effect=RuntimeError('SSH result unknown')))
        task.authorize()
        task.thread.start()
        self.assertTrue(task.done.wait(1))
        self.runner.assert_called_once()
        self.assertEqual(task.error, 'SSH result unknown')
        self.assertFalse(self.lock.locked())

    def test_failed_and_malformed_reports_cannot_be_reported_as_success(self):
        for report in ({'ok': False, 'failure': 'emergency active'}, {}, None, {'ok': 1}):
            with self.subTest(report=report):
                task = self.make(runner=Mock(return_value=report))
                task.authorize()
                task.thread.start()
                self.assertTrue(task.done.wait(1))
                self.assertTrue(task.error)
                self.runner.assert_called_once()

    def test_navigation_command_lock_stays_held_during_remote_recovery(self):
        def run(*_):
            self.assertFalse(self.lock.acquire(blocking=False))
            return {'ok': True}
        task = self.make(runner=run)
        task.authorize()
        task.thread.start()
        self.assertTrue(task.done.wait(1))
        self.assertIsNone(task.error)
        self.assertFalse(self.lock.locked())


class RecoveryPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        desktop_tests.PanelPolicyTests.setUpClass()
        cls.module = desktop_tests.PanelPolicyTests.module

    def panel(self):
        p = desktop_tests.PanelPolicyTests().panel()
        ownership = ChassisOwnership()
        p.map_panel.chassis_ownership = ownership
        p.map_panel.mapping_control = NS(busy=False, stop=Mock())
        p.acquire, p.release = ownership.acquire, ownership.release
        p.credentials = lambda: ('robot', 'fairino', 'secret')
        return p

    def start(self, p, runner, wait_timeout=.4):
        def factory(*args, **kwargs):
            return RecoverySession(*args, **kwargs, runner=runner, wait_timeout=wait_timeout)
        with patch.object(self.module, 'RecoverySession', side_effect=factory):
            p.control('recover')
        self.addCleanup(self.stop_task, p)

    def stop_task(self, p):
        if p.recovery and p.recovery.thread.is_alive():
            p.recovery.reject('test cleanup')
            p.recovery.thread.join(1)

    def finish(self, p):
        deadline = time.monotonic() + 2
        with patch('fairino_slam.maintenance.recovery.report_lines', return_value=['日志：/tmp/recovery']):
            while p.recovery is not None and time.monotonic() < deadline:
                p.tick()
                time.sleep(.005)
        self.assertIsNone(p.recovery)

    def test_single_click_stops_existing_drive_then_recovers_once_without_reconnect(self):
        p = self.panel()
        drive = Mock(done=threading.Event())
        drive.stop.side_effect = drive.done.set
        p.drive, p.lease = drive, True
        p.map_panel.chassis_ownership.acquire('base', '导航')
        runner = Mock(return_value={'ok': True})
        self.start(p, runner)
        p.control('recover')
        p.control('drive')
        p.control('goal')
        drive.stop.assert_called_once()
        p.map_panel.mapping_control.stop.assert_called_once()
        self.assertTrue(p.locked())
        self.finish(p)
        runner.assert_called_once()
        self.assertIsNone(p.target.goal_id)
        self.assertIsNone(p.target.pose)
        self.assertEqual(p.target.points, [])
        self.assertIsNone(p.drive)
        self.assertIsNone(p.map_panel.chassis_ownership.owner)
        self.assertIsNone(p.map_panel.chassis_ownership.reserved_for)
        self.assertFalse(p.active())
        self.assertIn('重新接入', p.message.set_text.call_args.args[0])
        self.assertIn('初始化定位', p.message.set_text.call_args.args[0])
        self.assertIn('\n日志', p.message.set_text.call_args.args[0])

    def test_mapping_lease_is_retained_until_exit_and_new_starts_are_blocked(self):
        p = self.panel()
        ownership = p.map_panel.chassis_ownership
        mapping = p.map_panel.mapping_control
        mapping.busy = True
        ownership.acquire('base', '建图键控')
        runner = Mock(return_value={'ok': True})
        self.start(p, runner)
        time.sleep(.03)
        p.tick()
        self.assertFalse(ownership.acquire('base', '导航'))
        self.assertEqual(ownership.owner, '建图键控')
        runner.assert_not_called()
        mapping.busy = False
        ownership.release('base', '建图键控')
        self.assertFalse(ownership.acquire('base', '建图键控'))
        self.finish(p)
        runner.assert_called_once()

    def test_failed_local_exit_does_not_release_live_drive_lease(self):
        p = self.panel()
        p.drive = Mock(done=threading.Event(), message='等待旧会话退出')
        p.lease = True
        p.map_panel.chassis_ownership.acquire('base', '导航')
        runner = Mock()
        self.start(p, runner, wait_timeout=.04)
        self.finish(p)
        runner.assert_not_called()
        self.assertEqual(p.map_panel.chassis_ownership.owner, '导航')
        self.assertTrue(p.lease)
        self.assertIsNotNone(p.drive)
        self.assertTrue(p.uncertain_goal)
        self.assertIsNone(p.target.goal_id)

    def test_old_goal_and_stop_results_do_not_revive_target_during_recovery(self):
        p = self.panel()
        old_generation, old_context = p.generation, p.target.context
        gate = threading.Event()
        def run(*_):
            gate.wait(1)
            return {'ok': True}
        self.start(p, Mock(side_effect=run))
        p.events.put(('result', (old_generation, 'goal', old_context, {'goal_id': 'late'}, None)))
        p.events.put(('ended', (old_generation, [], False)))
        p.tick()
        self.assertIsNone(p.target.goal_id)
        self.assertTrue(p.busy)
        gate.set()
        self.finish(p)
        p.events.put(('result', (old_generation, 'goal', old_context, {'goal_id': 'later'}, None)))
        p.tick()
        self.assertIsNone(p.target.goal_id)
        self.assertFalse(p.active())

    def test_queued_commands_superseded_by_recovery_are_never_sent_later(self):
        for action in ('goal', 'end'):
            with self.subTest(action=action):
                p = self.panel()
                with patch.object(self.module.threading, 'Thread') as thread:
                    if action == 'end':
                        p.end()
                    else:
                        p.submit('goal', pose={'x': 1., 'y': 0.})
                    pending = thread.call_args.kwargs['target']
                self.start(p, Mock(return_value={'ok': True}))
                self.finish(p)
                with patch.object(self.module, 'execute') as command:
                    pending()
                command.assert_not_called()
                self.assertIsNone(p.target.goal_id)

    def test_failed_recovery_keeps_old_goal_cleared_and_shows_reason(self):
        p = self.panel()
        self.start(p, Mock(return_value={'ok': False, 'failure': 'fault persists'}))
        self.finish(p)
        self.assertIsNone(p.target.goal_id)
        self.assertIsNone(p.target.pose)
        self.assertTrue(p.uncertain_goal)
        self.assertIn('fault persists', p.message.set_text.call_args.args[0])
        self.assertIn('日志', p.message.set_text.call_args.args[0])

    def test_close_does_not_release_lease_while_remote_recovery_is_running(self):
        p = self.panel()
        entered, release = threading.Event(), threading.Event()
        def run(*_):
            entered.set()
            release.wait(1)
            return {'ok': True}
        self.start(p, run)
        deadline = time.monotonic()+1
        while not entered.is_set() and time.monotonic()<deadline:
            p.tick()
            time.sleep(.005)
        self.assertTrue(entered.is_set())
        self.assertFalse(p.close())
        self.assertEqual(p.map_panel.chassis_ownership.owner, '恢复接入')
        self.assertFalse(p.closed)
        release.set()
        self.finish(p)
        self.assertTrue(p.close())

    def test_recovery_button_remains_available_when_navigation_status_is_unavailable(self):
        p = self.panel()
        p.state, p.received = {}, 0.
        p.tick()
        p.buttons['recover'].set_sensitive.assert_called_with(True)


if __name__ == '__main__':
    unittest.main()
