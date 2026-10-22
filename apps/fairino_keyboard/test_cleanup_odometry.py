"""Motor shutdown must not destroy healthy wheel odometry."""
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import robot_worker as rw
from control_core import normalise_hardware
from test_control import hardware


class CleanupOdometryTests(unittest.TestCase):
    def worker(self, ready=True, disabled=True, fault=False, fresh=True, active=True,
               zero_failure=False, cancel_failure=False, emit_failure=False,
               unload_sticky=False, chassis_drops=False):
        w = rw.Worker.__new__(rw.Worker)
        w.owned, w.ready, w.enable_attempted = True, ready, True
        w.hw = normalise_hardware(hardware())
        w.hw['enabled'] = [True]*8
        w.hw['error'] = int(fault)
        w.hw['position'] = [0.]*8
        w.hw['velocity'] = [0.]*8
        w.hw_time = w.upper_time = time.monotonic()-(0 if fresh else 2)
        w.upper = 0
        w.loaded = [rw.CHASSIS, rw.ANGLE]
        w.trajectory = w.param_dir = None
        w.zero = Mock(side_effect=RuntimeError('zero publish failed') if zero_failure else None)
        def command(value, guarded=True):
            self.assertEqual(value, 'disable()')
            self.assertFalse(guarded)
            w.hw['enabled'] = [not disabled]*8
        w.command = Mock(side_effect=command)
        w.pump = Mock()
        w.emit = Mock(side_effect=BrokenPipeError('report pipe closed') if emit_failure else None)
        w.status = Mock()
        w.odometry = Mock()
        w.controller_states = {rw.CHASSIS: 'active' if active else 'inactive', rw.ANGLE: 'active'}
        def switch(activate, deactivate, guarded=True):
            self.assertFalse(guarded)
            for name in deactivate:
                w.controller_states[name] = 'inactive'
            for name in activate:
                w.controller_states[name] = 'active'
        w.switch = Mock(side_effect=switch)
        w.controllers = Mock(side_effect=lambda *a: w.controller_states.copy())
        def service(kind, path, request, guarded=True):
            self.assertEqual(path, '/controller_manager/unload_controller')
            self.assertEqual(request.name, rw.ANGLE)
            self.assertEqual(w.controller_states[request.name], 'inactive')
            self.assertFalse(guarded)
            if not unload_sticky:
                del w.controller_states[request.name]
            if chassis_drops:
                w.controller_states[rw.CHASSIS] = 'inactive'
            return NS(ok=True)
        w.service = Mock(side_effect=service)
        if cancel_failure:
            w.trajectory = NS(accepted=True, cancel_goal_async=Mock(
                side_effect=RuntimeError('trajectory cancel failed')))
        return w

    def cleanup(self, worker):
        with patch.dict(rw.__dict__, {'UnloadController': NS(Request=NS)}):
            return worker.cleanup()

    def test_disabled_healthy_session_keeps_same_odom_without_resume_or_reset(self):
        w = self.worker()
        self.assertTrue(self.cleanup(w))
        w.zero.assert_called_once()
        w.command.assert_called_once_with('disable()', guarded=False)
        w.odometry.assert_not_called()
        w.switch.assert_called_once_with([], [rw.ANGLE], False)
        self.assertTrue(w.status.call_args.kwargs['odometry_preserved'])
        self.assertTrue(w.status.call_args.kwargs['cleanup_confirmed'])
        self.assertEqual(w.status.call_args.kwargs['remaining_controllers'], [])
        self.assertEqual(w.controller_states, {rw.CHASSIS: 'active'})
        self.assertGreaterEqual(w.controllers.call_count, 2)
        self.assertFalse(w.ready)

    def test_failed_or_incomplete_startup_still_pauses_odom_and_deactivates(self):
        w = self.worker(ready=False)
        self.assertTrue(self.cleanup(w))
        w.odometry.assert_called_once_with('pause', guarded=False)
        self.assertEqual(set(w.switch.call_args.args[1]), {rw.CHASSIS, rw.ANGLE})
        self.assertFalse(w.status.call_args.kwargs['odometry_preserved'])

    def test_no_preservation_without_fresh_confirmed_disable(self):
        for args in ({'disabled': False}, {'fresh': False}):
            with self.subTest(args=args):
                w = self.worker(**args)
                self.assertFalse(self.cleanup(w))
                w.odometry.assert_called_once_with('pause', guarded=False)
                self.assertIn(rw.CHASSIS, w.switch.call_args.args[1])

    def test_hardware_fault_still_pauses_even_after_disable(self):
        w = self.worker(fault=True)
        self.cleanup(w)
        w.odometry.assert_called_once_with('pause', guarded=False)
        self.assertIn(rw.CHASSIS, w.switch.call_args.args[1])

    def test_does_not_activate_or_resume_an_inactive_chassis(self):
        w = self.worker(active=False)
        self.cleanup(w)
        w.odometry.assert_called_once_with('pause', guarded=False)
        self.assertEqual(w.switch.call_args.args[0], [])
        self.assertFalse(w.status.call_args.kwargs['odometry_preserved'])

    def test_failed_zero_publish_disables_and_pauses_instead_of_preserving(self):
        w = self.worker(zero_failure=True)
        self.assertTrue(self.cleanup(w))
        w.command.assert_called_once_with('disable()', guarded=False)
        w.odometry.assert_called_once_with('pause', guarded=False)
        self.assertFalse(w.status.call_args.kwargs['odometry_preserved'])
        self.assertTrue(w.status.call_args.kwargs['disabled_confirmed'])
        self.assertTrue(w.status.call_args.kwargs['cleanup_confirmed'])
        self.assertEqual(w.controller_states, {rw.CHASSIS: 'inactive'})

    def test_preserved_odometry_does_not_mask_a_false_successful_temporary_unload(self):
        w = self.worker(unload_sticky=True)
        self.assertFalse(self.cleanup(w))
        w.odometry.assert_not_called()
        self.assertTrue(w.status.call_args.kwargs['odometry_preserved'])
        self.assertTrue(w.status.call_args.kwargs['disabled_confirmed'])
        self.assertFalse(w.status.call_args.kwargs['cleanup_confirmed'])
        self.assertEqual(w.status.call_args.kwargs['remaining_controllers'], [rw.ANGLE])
        self.assertEqual(w.controller_states[rw.ANGLE], 'inactive')
        self.assertEqual(w.controller_states[rw.CHASSIS], 'active')
        self.assertGreaterEqual(w.controllers.call_count, 2)

    def test_preservation_is_not_reported_if_chassis_stops_during_cleanup(self):
        w = self.worker(chassis_drops=True)
        self.assertFalse(self.cleanup(w))
        self.assertFalse(w.status.call_args.kwargs['odometry_preserved'])
        self.assertFalse(w.status.call_args.kwargs['cleanup_confirmed'])
        self.assertTrue(w.status.call_args.kwargs['disabled_confirmed'])
        self.assertEqual(w.controller_states, {rw.CHASSIS: 'inactive'})
        self.assertFalse(any(call.args[0] == 'resume' for call in w.odometry.call_args_list))

    def test_broken_reporting_still_disables_and_unloads_while_preserving_odometry(self):
        w = self.worker(cancel_failure=True, emit_failure=True)
        self.assertFalse(self.cleanup(w))
        w.command.assert_called_once_with('disable()', guarded=False)
        w.service.assert_called_once()
        w.odometry.assert_not_called()
        self.assertTrue(w.status.call_args.kwargs['disabled_confirmed'])
        self.assertFalse(w.status.call_args.kwargs['cleanup_confirmed'])
        self.assertTrue(w.status.call_args.kwargs['odometry_preserved'])
        self.assertEqual(w.controller_states, {rw.CHASSIS: 'active'})

    def test_upper_fault_or_stale_upper_feedback_prevents_preservation(self):
        for fault in (True, False):
            with self.subTest(fault=fault):
                w = self.worker()
                if fault:
                    w.upper = 1
                else:
                    w.upper_time -= 2
                self.assertTrue(self.cleanup(w))
                w.odometry.assert_called_once_with('pause', guarded=False)
                self.assertFalse(w.status.call_args.kwargs['odometry_preserved'])
                self.assertEqual(w.controller_states, {rw.CHASSIS: 'inactive'})


if __name__ == '__main__':
    unittest.main()
