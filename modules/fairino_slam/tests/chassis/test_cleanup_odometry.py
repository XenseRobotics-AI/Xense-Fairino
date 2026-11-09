"""Motor shutdown must not destroy healthy wheel odometry."""
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from fairino_slam.chassis import robot_worker as rw
from fairino_slam.chassis.control_core import normalise_hardware
from .helpers import hardware


class CleanupOdometryTests(unittest.TestCase):
    def worker(self, ready=True, disabled=True, fault=False, fresh=True, active=True):
        w = rw.Worker.__new__(rw.Worker)
        w.owned, w.ready, w.enable_attempted = True, ready, True
        w.hw = normalise_hardware(hardware())
        w.hw['enabled'] = [not disabled]*8
        w.hw['error'] = int(fault)
        w.hw['position'] = [0.]*8
        w.hw['velocity'] = [0.]*8
        w.hw_time = w.upper_time = time.monotonic()-(0 if fresh else 2)
        w.upper = 0
        w.loaded = [rw.CHASSIS, rw.ANGLE]
        w.trajectory = w.param_dir = None
        w.zero = Mock()
        w.command = Mock()
        w.pump = Mock()
        w.emit = Mock()
        w.status = Mock()
        w.odometry = Mock()
        w.switch = Mock()
        w.controllers = Mock(return_value={rw.CHASSIS: 'active' if active else 'inactive',
                                            rw.ANGLE: 'active'})
        w.service = Mock(return_value=NS(ok=True))
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


if __name__ == '__main__':
    unittest.main()
