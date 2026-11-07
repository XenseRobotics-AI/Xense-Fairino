"""Recovery transport/result tests never contact ROS or a robot."""
import ast
import base64
import copy
import json
from pathlib import Path
import shlex
import tempfile
import unittest
from unittest.mock import patch
import zlib

from fairino_slam.maintenance import recovery


def complete():
    return {
        "ok": True, "phase": "complete", "part": "base",
        "cleared_and_disabled": True, "temporary_controllers_removed": True,
        "navigation_stopped": True, "enable_sent": False, "motion_sent": False,
    }


def output(report):
    return recovery.PREFIX + json.dumps(report) + "\n"


class RecoveryReportTests(unittest.TestCase):
    def test_complete_result_and_failure_are_distinct(self):
        self.assertTrue(recovery.parse_report(output(complete()), True)["ok"])
        failure = {"ok": False, "phase": "lock", "failure": "控制权仍被占用"}
        self.assertEqual(recovery.parse_report(output(failure), True), failure)

    def test_empty_multiple_malformed_and_nonfinite_results_are_uncertain(self):
        cases = ("", "SSH closed\n", output(complete())*2,
                 recovery.PREFIX+"null\n", recovery.PREFIX+'{"ok":NaN}\n',
                 recovery.PREFIX+'{"ok":1}\n', recovery.PREFIX+"{\n")
        for text in cases:
            with self.subTest(text=text), self.assertRaises(RuntimeError):
                recovery.parse_report(text, True)

    def test_partial_or_unsafe_success_never_counts_as_recovered(self):
        for key, value in (
            ("phase", "disable"), ("cleared_and_disabled", False),
            ("temporary_controllers_removed", None), ("navigation_stopped", 1),
            ("enable_sent", True), ("motion_sent", True),
        ):
            report = complete();report[key] = value
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                recovery.parse_report(output(report), True)
        report = complete();del report["enable_sent"]
        with self.assertRaises(RuntimeError):
            recovery.parse_report(output(report), True)

    def test_readonly_report_does_not_claim_recovery(self):
        report = {"ok": True, "phase": "preflight", "ready_to_recover": False}
        self.assertEqual(recovery.parse_report(output(report), False), report)


class RecoveryClientTests(unittest.TestCase):
    def test_exactly_one_explicit_attempt_and_password_not_logged(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(recovery, "LOG_ROOT", Path(directory)), \
             patch.object(recovery, "remote_command", return_value="test-worker"), \
             patch.object(recovery, "run", return_value=output(complete())) as run:
            result = recovery.run_recovery("192.168.58.3", "fairino", "secret", execute=True)
            run.assert_called_once_with("192.168.58.3", "test-worker", user="fairino",
                                        password="secret", timeout=180, check=False)
            self.assertTrue(result["ok"])
            self.assertTrue(result["execute"])
            text = (Path(result["log_directory"])/"result.json").read_text()
            self.assertNotIn("secret", text)
            self.assertNotIn("password", text)

    def test_transport_uncertainty_is_logged_and_never_retried(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(recovery, "LOG_ROOT", Path(directory)), \
             patch.object(recovery, "remote_command", return_value="test-worker"), \
             patch.object(recovery, "run", side_effect=TimeoutError("ssh lost secret")) as run:
            with self.assertRaisesRegex(RuntimeError, "未自动重试"):
                recovery.run_recovery("robot", "fairino", "secret")
            run.assert_called_once()
            logs = list(Path(directory).rglob("result.json"))
            self.assertEqual(len(logs), 1)
            saved = json.loads(logs[0].read_text())
            self.assertTrue(saved["result_uncertain"])
            self.assertFalse(saved["ok"])
            self.assertFalse(saved["automatic_retry"])
            self.assertNotIn("secret", logs[0].read_text())

    def test_incomplete_response_is_not_retried(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(recovery, "LOG_ROOT", Path(directory)), \
             patch.object(recovery, "remote_command", return_value="test-worker"), \
             patch.object(recovery, "run", return_value="connection closed") as run:
            with self.assertRaisesRegex(RuntimeError, "未自动重试"):
                recovery.run_recovery("robot", "fairino", None)
            run.assert_called_once()

    def test_invalid_ssh_destination_rejected_before_any_io(self):
        for host, user in (("-oProxyCommand=x", "fairino"), ("host name", "fairino"),
                           ("robot", "-root"), ("robot", "root;id")):
            with self.subTest(host=host, user=user), patch.object(recovery, "run") as run:
                with self.assertRaises(ValueError):
                    recovery.run_recovery(host, user, None)
                run.assert_not_called()

    def test_report_text_explains_result_and_does_not_offer_resume(self):
        report = complete()
        report.update(execute=True, odometry_paused=True, log_directory="/tmp/log")
        text = "\n".join(recovery.report_lines(report))
        self.assertIn("可重新接入", text)
        self.assertIn("不会自动继续旧目标", text)
        self.assertIn("重新初始化定位", text)
        self.assertIn("/tmp/log", text)
        report.update(ok=False, failure="硬件反馈中断", phase="verify_final")
        self.assertIn("硬件反馈中断", "\n".join(recovery.report_lines(report)))

    def test_cli_defaults_to_check_and_execute_is_explicit(self):
        for argv, execute in (([], False), (["--execute"], True)):
            report = complete()
            report.update(execute=execute)
            with self.subTest(argv=argv), \
                 patch.object(recovery, "run_recovery", return_value=report) as run, \
                 patch("builtins.print"):
                self.assertEqual(recovery.main(argv), 0)
                run.assert_called_once_with("192.168.58.3", "fairino", execute=execute)

    def test_bundle_contains_current_standalone_sources_without_installed_package(self):
        for execute in (False, True):
            command = recovery.remote_command(execute)
            loader = shlex.split(command.splitlines()[-1])[-1]
            tree = ast.parse(loader)
            encoded = max((node.value for node in ast.walk(tree)
                           if isinstance(node, ast.Constant) and isinstance(node.value, str)), key=len)
            bundle = json.loads(zlib.decompress(base64.b64decode(encoded)))
            self.assertEqual(set(bundle), {"fault_reset_remote.py", "recovery_remote.py"})
            for name, text in bundle.items():
                self.assertEqual(text, (recovery.HERE/name).read_text())
                ast.parse(text)
                self.assertNotIn("from fairino_slam", text)
            self.assertIn("execute="+repr(execute), loader)
            self.assertLess(len(command.encode()), 120000)
            self.assertNotIn("systemctl restart", command)


if __name__ == "__main__":
    unittest.main()
