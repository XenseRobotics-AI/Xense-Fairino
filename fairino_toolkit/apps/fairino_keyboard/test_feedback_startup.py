"""Startup discovery regressions: no ROS, network, or motor commands."""
import ast
import json
import subprocess
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import robot_worker as rw


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

    def test_stop_cancels_discovery_immediately(self):
        w = self.worker()
        w.stop.set()
        with self.assertRaisesRegex(RuntimeError, "启动已取消"):
            w.wait_initial_feedback()


class RelayNetworkStartupTests(unittest.TestCase):
    def functions(self):
        path = Path(__file__).parent / "diagnostics/20261008-feedback/upper_estop_relay.py"
        tree = ast.parse(path.read_text())
        functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)
                     and n.name in ("network_ready", "wait_for_network")]
        scope = {"json": json, "subprocess": subprocess, "print": lambda *a, **k: None}
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), "exec"), scope)
        return scope

    def test_ip_and_carrier_are_both_required(self):
        scope = self.functions()
        for flags, address, expected in [
            (["UP"], "192.168.58.1", False),
            (["UP", "LOWER_UP"], "192.168.31.1", False),
            (["UP", "LOWER_UP"], "192.168.58.1", True),
        ]:
            data = [{"flags": flags, "addr_info": [{"local": address}]}]
            with patch.object(subprocess, "run", return_value=NS(stdout=json.dumps(data))):
                self.assertEqual(scope["network_ready"](), expected)

    def test_link_flap_restarts_stability_wait(self):
        scope = self.functions()
        now = [0.0]
        scope["time"] = NS(monotonic=lambda: now[0], sleep=lambda dt: now.__setitem__(0, now[0]+dt))
        scope["network_ready"] = lambda: 0.4 <= now[0] < 0.8 or now[0] >= 2.0
        scope["wait_for_network"]()
        self.assertGreaterEqual(now[0], 3.0)
        self.assertLess(now[0], 3.5)


if __name__ == "__main__":
    unittest.main()
