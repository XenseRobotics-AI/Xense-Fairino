"""ROS publication/cancellation failures must not skip motor disable."""
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import robot_worker as rw
from control_core import normalise_hardware
from test_control import hardware


class CleanupTests(unittest.TestCase):
    def exercise(self, zero_failure=False, cancel_failure=False, disable_failure=False,
                 odom_failure=False, emit_failure=False, status_failure=False,
                 unload_failure=False, unload_error=False, unload_sticky=False,
                 disable_unconfirmed=False):
        w = rw.Worker.__new__(rw.Worker)
        w.owned = w.enable_attempted = True
        # Exercise teardown of an incomplete startup. Healthy ready-session
        # odometry preservation is covered separately in test_cleanup_odometry.
        w.ready = False
        w.hw = normalise_hardware(hardware())
        w.hw['enabled'] = [True]*8
        w.hw_time = w.upper_time = time.monotonic()
        w.upper = 0
        w.loaded, w.param_dir = [rw.ANGLE], None
        trace, messages, statuses = [], [], []
        controllers = {rw.CHASSIS: 'active', rw.ANGLE: 'active'}
        controller_reads = []
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
            if not disable_unconfirmed:
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
            statuses.append(dict(kw, phase=w.phase, controllers=controllers.copy(),
                                 controller_reads=controller_reads[:]))
        w.emit, w.status = emit, status
        def list_controllers(guarded=True):
            self.assertFalse(guarded)
            controller_reads.append(controllers.copy())
            return controllers.copy()
        def switch(activate, deactivate, guarded):
            self.assertFalse(guarded)
            trace.append('switch')
            for name in deactivate:
                self.assertIn(name, controllers)
                controllers[name] = 'inactive'
            for name in activate:
                controllers[name] = 'active'
        def service(kind, path, request, guarded):
            self.assertFalse(guarded)
            self.assertEqual(path, '/controller_manager/unload_controller')
            self.assertEqual(request.name, rw.ANGLE)  # Never unload the vendor instance.
            self.assertEqual(controllers.get(request.name), 'inactive')
            trace.append('unload')
            if unload_error:
                raise RuntimeError('simulated unload service failure')
            if unload_failure:
                return NS(ok=False)
            if not unload_sticky:
                del controllers[request.name]
            return NS(ok=True)
        w.controllers, w.switch, w.service = list_controllers, switch, service
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
                self.assertNotIn(rw.ANGLE, statuses[-1]['controllers'])
                self.assertEqual(statuses[-1]['controllers'][rw.CHASSIS], 'inactive')
                self.assertGreaterEqual(len(statuses[-1]['controller_reads']), 2)
                self.assertNotIn(rw.ANGLE, statuses[-1]['controller_reads'][-1])

    def test_failed_disable_is_not_reported_confirmed_or_followed_by_unload(self):
        for failure in ('disable_failure', 'disable_unconfirmed'):
            with self.subTest(failure=failure):
                result, trace, messages, statuses = self.exercise(True, True, **{failure: True})
                self.assertFalse(result)
                self.assertIn('disable()', trace)
                self.assertIn('odom:pause', trace)
                self.assertIn('switch', trace)
                self.assertNotIn('unload', trace)
                self.assertFalse(statuses[-1]['disabled_confirmed'])
                self.assertFalse(statuses[-1]['cleanup_confirmed'])
                self.assertIn(rw.ANGLE, statuses[-1]['controllers'])
                self.assertTrue(any('下使能' in m.get('error', '') for m in messages))

    def test_failed_controller_unload_is_distinct_from_successful_motor_disable(self):
        for failure in ('unload_failure', 'unload_error'):
            with self.subTest(failure=failure):
                result, trace, messages, statuses = self.exercise(**{failure: True})
                self.assertFalse(result)
                self.assertIn('disable()', trace)
                self.assertIn('unload', trace)
                self.assertTrue(statuses[-1]['disabled_confirmed'])
                self.assertFalse(statuses[-1]['cleanup_confirmed'])
                self.assertIn('清理未完成', statuses[-1]['phase'])
                self.assertEqual(statuses[-1]['controllers'][rw.ANGLE], 'inactive')
                self.assertTrue(any('控制器' in m.get('error', '') for m in messages))

    def test_success_reply_without_removing_controller_is_not_cleanup_confirmation(self):
        result, trace, messages, statuses = self.exercise(unload_sticky=True)
        self.assertFalse(result)
        self.assertIn('unload', trace)
        self.assertTrue(statuses[-1]['disabled_confirmed'])
        self.assertFalse(statuses[-1]['cleanup_confirmed'])
        self.assertIn('清理未完成', statuses[-1]['phase'])
        self.assertEqual(statuses[-1]['controllers'][rw.ANGLE], 'inactive')
        self.assertGreaterEqual(len(statuses[-1]['controller_reads']), 2)
        self.assertEqual(statuses[-1]['controller_reads'][-1][rw.ANGLE], 'inactive')
        self.assertTrue(any(rw.ANGLE in m.get('error', '') for m in messages))

    def test_odom_cleanup_failure_is_distinct_from_successful_motor_disable(self):
        result, trace, messages, statuses = self.exercise(odom_failure=True)
        self.assertFalse(result)
        self.assertTrue(statuses[-1]['disabled_confirmed'])
        self.assertFalse(statuses[-1]['cleanup_confirmed'])
        self.assertIn('清理未完成', statuses[-1]['phase'])
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
