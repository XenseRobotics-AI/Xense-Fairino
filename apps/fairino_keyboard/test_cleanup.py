"""ROS publication/cancellation failures must not skip motor disable."""
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import robot_worker as rw


class CleanupTests(unittest.TestCase):
    def exercise(self, zero_failure=False, cancel_failure=False, disable_failure=False,
                 odom_failure=False, emit_failure=False, status_failure=False):
        w = rw.Worker.__new__(rw.Worker)
        w.owned = w.ready = w.enable_attempted = True
        w.hw, w.hw_time = {'enabled': [True]*8}, time.monotonic()
        w.loaded, w.param_dir = [rw.ANGLE], None
        trace, messages, statuses = [], [], []
        def zero():
            trace.append('zero')
            if zero_failure:
                raise RuntimeError('simulated closed publisher')
        def cancel():
            trace.append('cancel')
            if cancel_failure:
                raise RuntimeError('simulated action failure')
        def command(value, guarded):
            self.assertFalse(guarded)
            trace.append(value)
            if disable_failure:
                raise RuntimeError('simulated disable failure')
            w.hw['enabled'] = [False]*8
        def odometry(action, guarded):
            self.assertFalse(guarded)
            trace.append('odom:'+action)
            if odom_failure:
                raise RuntimeError('simulated odom failure')
        w.zero, w.command, w.odometry = zero, command, odometry
        w.trajectory = NS(accepted=True, cancel_goal_async=cancel)
        w.pump = lambda *a: None
        def emit(**kw):
            messages.append(kw)
            if emit_failure:
                raise BrokenPipeError('simulated diagnostic pipe closed')
        def status(*a, **kw):
            if status_failure:
                raise BrokenPipeError('simulated final status pipe closed')
            statuses.append(kw)
        w.emit, w.status = emit, status
        w.controllers = lambda *a: {rw.CHASSIS: 'active', rw.ANGLE: 'active'}
        w.switch = lambda *a: trace.append('switch')
        w.service = lambda *a: (trace.append('unload') or NS(ok=True))
        with patch.dict(rw.__dict__, {'UnloadController': NS(Request=NS)}):
            result = w.cleanup()
        return result, trace, messages, statuses

    def test_publication_and_cancel_failures_still_disable_and_cleanup(self):
        for zero, cancel in ((True, False), (False, True), (True, True)):
            with self.subTest(zero=zero, cancel=cancel):
                result, trace, messages, statuses = self.exercise(zero, cancel)
                self.assertTrue(result)
                self.assertEqual(trace, ['zero', 'cancel', 'disable()', 'odom:pause', 'switch', 'unload'])
                self.assertEqual(sum('cleanup_warning' in m for m in messages), int(zero)+int(cancel))
                self.assertTrue(statuses[-1]['disabled_confirmed'])
                self.assertTrue(statuses[-1]['cleanup_confirmed'])

    def test_failed_disable_is_not_reported_confirmed_or_followed_by_unload(self):
        result, trace, messages, statuses = self.exercise(True, True, disable_failure=True)
        self.assertFalse(result)
        self.assertIn('disable()', trace)
        self.assertIn('odom:pause', trace)
        self.assertIn('switch', trace)
        self.assertNotIn('unload', trace)
        self.assertFalse(statuses[-1]['disabled_confirmed'])
        self.assertTrue(any('下使能失败' in m.get('error', '') for m in messages))

    def test_odom_cleanup_failure_is_distinct_from_successful_motor_disable(self):
        result, trace, messages, statuses = self.exercise(odom_failure=True)
        self.assertFalse(result)
        self.assertTrue(statuses[-1]['disabled_confirmed'])
        self.assertFalse(statuses[-1]['cleanup_confirmed'])
        self.assertIn('unload', trace)

    def test_broken_warning_channel_does_not_skip_disable_or_fake_success(self):
        result, trace, messages, statuses = self.exercise(True, True, emit_failure=True)
        self.assertFalse(result)
        self.assertEqual(trace, ['zero', 'cancel', 'disable()', 'odom:pause', 'switch', 'unload'])
        self.assertTrue(statuses[-1]['disabled_confirmed'])
        self.assertFalse(statuses[-1]['cleanup_confirmed'])

    def test_missing_final_status_does_not_report_success(self):
        result, trace, messages, statuses = self.exercise(status_failure=True)
        self.assertFalse(result)
        self.assertIn('disable()', trace)
        self.assertIn('unload', trace)
        self.assertFalse(statuses)


if __name__ == '__main__':
    unittest.main()
