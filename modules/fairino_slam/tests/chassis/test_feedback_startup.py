"""Startup discovery regressions: no ROS, network, or motor commands."""
import threading
import unittest
from unittest.mock import patch

from fairino_slam.chassis import robot_worker as rw


class FeedbackStartupTests(unittest.TestCase):
    def worker(self):
        w = rw.Worker.__new__(rw.Worker)
        w.stop = threading.Event()
        w.input_error = ""
        w.hw = w.upper = None
        w.hw_time = w.upper_time = 0.0
        w.owned = w.enable_attempted = False
        w.clock = 100.0
        w.guarded = False
        w.status = lambda *args, **kwargs: None
        def guard(*args):
            w.guarded = True
            if w.upper:
                raise RuntimeError("上身急停/故障")
        w.guard = guard
        return w

    def test_discovery_slower_than_two_seconds_is_accepted(self):
        w = self.worker()
        def pump(*args):
            w.clock += 0.02
            w.hw, w.hw_time = {}, w.clock
            if w.clock >= 102.8:
                w.upper, w.upper_time = 0, w.clock
        w.pump = pump
        with patch.object(rw.time, "monotonic", lambda: w.clock):
            w.wait_initial_feedback()
        self.assertTrue(w.guarded)
        self.assertGreaterEqual(w.clock, 102.8)
        self.assertFalse(w.owned or w.enable_attempted)

    def test_missing_upper_times_out_without_ownership(self):
        w = self.worker()
        def pump(*args):
            w.clock += 0.02
            w.hw, w.hw_time = {}, w.clock
        w.pump = pump
        with patch.object(rw.time, "monotonic", lambda: w.clock):
            with self.assertRaisesRegex(RuntimeError, "等待上半身急停状态"):
                w.wait_initial_feedback(timeout=3)
        self.assertFalse(w.guarded or w.owned or w.enable_attempted)

    def test_stale_upper_does_not_unlock(self):
        w = self.worker()
        w.upper, w.upper_time = 0, 99.0
        def pump(*args):
            w.clock += 0.02
            w.hw, w.hw_time = {}, w.clock
        w.pump = pump
        with patch.object(rw.time, "monotonic", lambda: w.clock):
            with self.assertRaisesRegex(RuntimeError, "上半身急停状态"):
                w.wait_initial_feedback(timeout=1)
        self.assertFalse(w.guarded)

    def test_real_estop_still_blocks_startup(self):
        w = self.worker()
        w.hw, w.upper = {}, 123
        w.hw_time = w.upper_time = w.clock
        with patch.object(rw.time, "monotonic", lambda: w.clock):
            with self.assertRaisesRegex(RuntimeError, "急停/故障"):
                w.wait_initial_feedback()

    def test_feedback_error_during_discovery_keeps_its_cause(self):
        w = self.worker()
        def pump(*args):
            w.input_error = "底盘运动反馈发布失败：simulated byte field error"
            w.stop.set()
        w.pump = pump
        with patch.object(rw.time, "monotonic", lambda: w.clock):
            with self.assertRaisesRegex(RuntimeError, "simulated byte field error"):
                w.wait_initial_feedback()
        self.assertFalse(w.guarded or w.owned or w.enable_attempted)

    def test_stop_cancels_discovery_immediately(self):
        w = self.worker()
        w.stop.set()
        with self.assertRaisesRegex(RuntimeError, "启动已取消"):
            w.wait_initial_feedback()


if __name__ == "__main__":
    unittest.main()
