"""Offline regression checks: no ROS, SSH or robot commands are used."""
import contextlib
import copy
import io
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import clear_errors as client
import fault_reset_remote as worker


def message(word=0x2240, error=0):
    return SimpleNamespace(joint_names=[f"j{i}" for i in range(1, 9)],
                           stateword=[word] * 8, errorcode=[error] * 8,
                           position=[0.] * 8, velocity=[0.] * 8,
                           hexerrorcode_curerrcode=0x0E10FF01 if error else 0)


class Guards(unittest.TestCase):
    def test_fault_reaction_is_not_operation_enabled(self):
        for word in (0x220F, 0x130F):
            self.assertEqual(worker.drive_state(word), "fault_reaction")
            s = worker.snapshot(message(word, 30), 8)
            worker.require_idle(s)
            self.assertEqual(s["enabled_axes"], [])
            self.assertFalse(worker.is_clear(s))

    def test_enabled_and_quick_stop_rejected(self):
        for word in (0x2237, 0x1337, 0x2207, 0x0000):
            with self.assertRaises(RuntimeError):
                worker.require_idle(worker.snapshot(message(word), 8))

    def test_invalid_feedback_rejected(self):
        for key, value in (("joint_names", ["j1"] * 8), ("stateword", [0]),
                           ("position", [float("nan")] * 8), ("velocity", [float("inf")] * 8)):
            m = message()
            setattr(m, key, value)
            with self.assertRaises(RuntimeError):
                worker.snapshot(m, 8)
        m = message()
        m.velocity[0] = .1
        with self.assertRaises(RuntimeError):
            worker.require_idle(worker.snapshot(m, 8))

    def test_estop_and_relay_fail_closed(self):
        worker.require_interlock("upper", gpio={"56": "0", "57": "0"})
        worker.require_interlock("base", upper_error=0, upper_age=.1)
        for kwargs in ({"part": "upper", "gpio": None},
                       {"part": "upper", "gpio": {"56": "1", "57": "0"}},
                       {"part": "base", "upper_error": 30, "upper_age": .1},
                       {"part": "base", "upper_error": 0, "upper_age": .5}):
            with self.assertRaises(RuntimeError):
                worker.require_interlock(**kwargs)


class SingleShot(unittest.TestCase):
    def simulate(self, *, execute=True, blocked=False, result_code="0xC000",
                 stays_faulted=False, accepted=True, stale=False, action_timeout=False,
                 initially_clear=False):
        session = worker.ResetSession.__new__(worker.ResetSession)
        session.part, session.execute, session.node = "upper", execute, None
        session.report = {"ok": False, "send_attempted": False}
        clock = [0.]
        bad = worker.snapshot(message(0x2208, 30), 8)
        good = worker.snapshot(message(0x2221), 8)
        session.state = good if initially_clear else bad

        def fresh(timeout=5.):
            clock[0] += .1
            if session.report["send_attempted"]:
                if stale:
                    raise RuntimeError("反馈中断")
                session.state = bad if stays_faulted else good
            return session.state

        session.fresh = fresh
        session.preflight = Mock(side_effect=RuntimeError("急停") if blocked else None)
        session.controllers = Mock()
        session.interlock = Mock()
        result = SimpleNamespace(status=4, result=SimpleNamespace(error_code=result_code))
        handle = SimpleNamespace(accepted=accepted, get_result_async=lambda: result)
        action = Mock()
        action.wait_for_server.return_value = True
        action.send_goal_async.return_value = handle
        session.wait = Mock(side_effect=[handle, RuntimeError("超时")] if action_timeout
                            else lambda future, *a, **k: future)
        modules = {
            "rclpy.action": SimpleNamespace(ActionClient=lambda *a: action),
            "fairino_hardware_command_msgs.action": SimpleNamespace(StringCommand=SimpleNamespace(Goal=SimpleNamespace)),
        }
        error = None
        with patch.dict(sys.modules, modules), patch.object(worker.time, "monotonic", lambda: clock[0]), contextlib.redirect_stdout(io.StringIO()):
            try:
                session.run()
            except RuntimeError as exc:
                error = str(exc)
        return session, action, error

    def test_check_cannot_send(self):
        s, a, e = self.simulate(execute=False)
        a.send_goal_async.assert_not_called()
        self.assertTrue(s.report["ok"])

    def test_interlock_blocks_send(self):
        s, a, e = self.simulate(blocked=True)
        a.send_goal_async.assert_not_called()
        self.assertIsNotNone(e)

    def test_already_clear_skips_command(self):
        s, a, e = self.simulate(initially_clear=True)
        a.send_goal_async.assert_not_called()
        self.assertTrue(s.report["ok"])

    def test_single_reset_and_feedback_verification(self):
        s, a, e = self.simulate()
        a.send_goal_async.assert_called_once()
        self.assertEqual(a.send_goal_async.call_args.args[0].command, "reseterror()")
        self.assertIsNone(e)
        self.assertTrue(s.report["cleared_and_disabled"])
        self.assertTrue(s.report["ok"])

    def test_rejected_timeout_stale_or_uncleared_never_retries(self):
        for kwargs in ({"accepted": False}, {"action_timeout": True}, {"stale": True},
                       {"stays_faulted": True}, {"result_code": "0xC012"}):
            with self.subTest(kwargs=kwargs):
                s, a, e = self.simulate(**kwargs)
                a.send_goal_async.assert_called_once()
                self.assertIsNotNone(e)
                self.assertFalse(s.report["ok"])


class Client(unittest.TestCase):
    def test_all_stops_if_upper_reset_fails(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(client, "ROOT", Path(folder)), contextlib.redirect_stdout(io.StringIO()):
            with patch.object(client, "run_part", return_value={"ok": False, "failure": "未清除"}) as run:
                self.assertEqual(client.main(["--execute"]), 1)
                run.assert_called_once_with("upper", True, "fairino")

    def test_defaults_are_read_only_and_inspect_both(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(client, "ROOT", Path(folder)), contextlib.redirect_stdout(io.StringIO()):
            with patch.object(client, "run_part", side_effect=lambda *a: {"ok": True}) as run:
                self.assertEqual(client.main([]), 0)
                self.assertEqual([c.args for c in run.call_args_list],
                                 [("upper", False, "fairino"), ("base", False, "fairino")])

    def test_missing_or_duplicate_results_are_not_success(self):
        for out in ("", worker.PREFIX + "{}\n" + worker.PREFIX + "{}"):
            with self.assertRaises(RuntimeError):
                client.parse_report(out)


if __name__ == "__main__":
    unittest.main()
