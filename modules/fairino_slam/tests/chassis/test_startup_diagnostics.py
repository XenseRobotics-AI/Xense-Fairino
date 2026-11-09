"""Startup failures retain their real cause, without ROS, SSH or hardware."""
import contextlib
import errno
import fcntl
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from fairino_slam.chassis import robot_worker as rw


class StartupDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        # Keep tests independent of the platform ROS installation.
        loader = patch.object(rw, 'load_ros')
        loader.start()
        self.addCleanup(loader.stop)

    def failed_lock(self, error, *, operation):
        output = io.StringIO()
        opened = Mock()
        ros = Mock()
        with patch.object(rw, 'rclpy', ros, create=True), patch.object(rw, 'Worker') as worker, \
                patch.object(rw, 'open', create=True,
                             side_effect=error if operation == 'open' else None,
                             return_value=opened) as opener, \
                patch.object(rw.fcntl, 'flock',
                             side_effect=error if operation == 'flock' else None) as flock, \
                patch.object(rw.sys, 'argv', ['robot_worker.py']), \
                contextlib.redirect_stdout(output):
            self.assertEqual(rw.main(), 1)
            ros.init.assert_not_called()
            ros.shutdown.assert_not_called()
            worker.assert_not_called()  # No worker cleanup or enable side effect.
            opener.assert_called_once_with('/home/fairino/.fairino_keyboard.lock', 'a')
            if operation == 'open':
                flock.assert_not_called()
                opened.close.assert_not_called()
            else:
                flock.assert_called_once_with(opened, fcntl.LOCK_EX | fcntl.LOCK_NB)
                opened.close.assert_called_once()
        lines = output.getvalue().splitlines()
        self.assertEqual(len(lines), 1)
        self.assertTrue(lines[0].startswith(rw.PREFIX))
        row = json.loads(lines[0][len(rw.PREFIX):])
        self.assertEqual(row['stage'], 'lock')
        self.assertEqual(row['errno'], error.errno)
        self.assertEqual(row['path'], '/home/fairino/.fairino_keyboard.lock')
        self.assertEqual(row['operation'], operation)
        return row

    def test_conflicting_owner_is_reported_before_any_ros_or_cleanup(self):
        row = self.failed_lock(BlockingIOError(errno.EAGAIN, 'Resource temporarily unavailable'),
                               operation='flock')
        self.assertIn('其他会话占用', row['error'])

    def test_readonly_filesystem_is_reported_before_any_ros_or_cleanup(self):
        row = self.failed_lock(OSError(errno.EROFS, 'Read-only file system'), operation='open')
        self.assertIn('文件系统只读', row['error'])

    def test_other_open_error_keeps_errno_and_is_not_called_an_owner_conflict(self):
        row = self.failed_lock(PermissionError(errno.EACCES, 'Permission denied'), operation='open')
        self.assertIn('Permission denied', row['error'])
        self.assertNotIn('其他会话占用', row['error'])

    def test_unexpected_flock_error_still_closes_handle_and_reports_cause(self):
        row = self.failed_lock(OSError(errno.EIO, 'Input/output error'), operation='flock')
        self.assertIn('Input/output error', row['error'])
        self.assertNotIn('其他会话占用', row['error'])

    def test_acquired_lock_is_held_through_cleanup_then_closed_on_success_and_failure(self):
        for run_error in (None, RuntimeError('test control failure')):
            with self.subTest(run_error=run_error), tempfile.TemporaryDirectory() as directory:
                lock_path = str(Path(directory)/'lock')
                held = []
                def opener(path, mode):
                    stream = open(path, mode)
                    held.append(stream)  # Retain a reference: GC cannot release it for main.
                    return stream
                def cleanup():
                    with open(lock_path, 'a') as another:
                        with self.assertRaises(BlockingIOError):
                            fcntl.flock(another, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    return True
                worker = Mock()
                worker.run.side_effect = run_error
                worker.cleanup.side_effect = cleanup
                worker.output.unfinished_tasks = 0
                ros = Mock()
                with patch.object(rw, 'LOCK_PATH', lock_path), \
                        patch.object(rw, 'open', side_effect=opener, create=True), \
                        patch.object(rw, 'rclpy', ros, create=True), \
                        patch.object(rw, 'Worker', return_value=worker), \
                        patch.object(rw.signal, 'signal'), \
                        patch.object(rw.sys, 'argv', ['robot_worker.py']):
                    self.assertEqual(rw.main(), int(run_error is not None))
                self.assertEqual(len(held), 1)
                self.assertTrue(held[0].closed)
                with open(lock_path, 'a') as another:
                    fcntl.flock(another, fcntl.LOCK_EX | fcntl.LOCK_NB)
                worker.cleanup.assert_called_once()
                worker.node.destroy_node.assert_called_once()
                ros.shutdown.assert_called_once()
                if run_error:
                    worker.emit.assert_called_once_with(error='test control failure')

    def test_ros_initialization_failure_still_releases_acquired_lock(self):
        opened = Mock()
        ros = Mock()
        ros.init.side_effect = RuntimeError('ROS init failed')
        with patch.object(rw, 'open', return_value=opened, create=True), \
                patch.object(rw.fcntl, 'flock'), patch.object(rw, 'rclpy', ros, create=True), \
                patch.object(rw, 'Worker') as worker, \
                patch.object(rw.sys, 'argv', ['robot_worker.py']):
            with self.assertRaisesRegex(RuntimeError, 'ROS init failed'):
                rw.main()
        opened.close.assert_called_once()
        worker.assert_not_called()

    def test_readonly_check_preserves_existing_lock_free_behavior(self):
        worker = Mock()
        worker.output.unfinished_tasks = 0
        worker.cleanup.return_value = True
        with patch.object(rw, 'open', create=True) as opener, \
                patch.object(rw.fcntl, 'flock') as flock, \
                patch.object(rw, 'rclpy', Mock(), create=True), \
                patch.object(rw, 'Worker', return_value=worker), \
                patch.object(rw.signal, 'signal'), \
                patch.object(rw.sys, 'argv', ['robot_worker.py', '--check']):
            self.assertEqual(rw.main(), 0)
            opener.assert_not_called()
            flock.assert_not_called()

    def service_worker(self):
        worker = rw.Worker.__new__(rw.Worker)
        worker.phase = '底盘已使能，等待恢复里程计'
        worker.clients = {}
        future = Mock()
        future.done.return_value = False
        client = Mock()
        client.service_is_ready.return_value = True
        client.call_async.return_value = future
        worker.node = NS(create_client=Mock(return_value=client))
        return worker, future

    def test_service_timeout_includes_endpoint_and_current_phase(self):
        worker, future = self.service_worker()
        clock = [10.]
        worker.pump = lambda *args: clock.__setitem__(0, clock[0]+.5)
        with patch.object(rw.time, 'monotonic', side_effect=lambda: clock[0]):
            with self.assertRaisesRegex(RuntimeError, 'ROS 请求超时：/fairino_odometry/resume.*等待恢复里程计'):
                worker.service(object, '/fairino_odometry/resume', object())
        future.result.assert_not_called()

    def test_guard_failure_during_service_is_not_relabelled_as_timeout(self):
        worker, _ = self.service_worker()
        failure = RuntimeError('底盘硬件故障')
        worker.pump = Mock(side_effect=failure)
        with self.assertRaises(RuntimeError) as caught:
            worker.service(object, '/fairino_odometry/resume', object())
        self.assertIs(caught.exception, failure)
        self.assertNotIn('超时', str(caught.exception))

    def test_completed_service_result_is_unchanged_and_still_guarded(self):
        worker, future = self.service_worker()
        future.done.return_value = True
        result = NS(success=True)
        future.result.return_value = result
        worker.guard = Mock()
        self.assertIs(worker.service(object, '/fairino_odometry/resume', object()), result)
        worker.guard.assert_called_once()


if __name__ == '__main__':
    unittest.main()
