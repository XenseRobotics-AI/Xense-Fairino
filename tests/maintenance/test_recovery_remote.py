"""Offline recovery tests: fake ROS and service state; no robot access."""
import contextlib
import copy
import fcntl
import io
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from fairino_slam.maintenance import fault_reset_remote as base
from fairino_slam.maintenance import recovery_remote as worker


COMMAND = "fairino_hardware_command_controller"
FACTORY = "eight_dof_chassis_controller"
TEMPLATES = dict(worker.TEMPORARY_CONTROLLERS)
INACTIVE = {"LoadState": "not-found", "ActiveState": "inactive",
            "SubState": "dead", "FragmentPath": "", "DropInPaths": ""}


def hardware(word=0x2240, error=0, velocity=0., command=0.):
    return base.snapshot(NS(
        joint_names=[f"j{i}" for i in range(1, 9)],
        stateword=[word] * 8, errorcode=[error] * 8,
        position=[0.] * 8, velocity=[velocity] * 8,
        command_velocity=[command] * 8,
        hexerrorcode_curerrcode=0x0E100001 if error else 0), 8)


def controllers(extra=None):
    result = [NS(name=COMMAND, state="active", type="command")]
    result.extend(NS(name=name, state=state, type=TEMPLATES.get(name, "factory"))
                  for name, state in (extra or {}).items())
    return result


class ControllerGuards(unittest.TestCase):
    def test_known_motion_allowed_only_before_stop(self):
        values = controllers({name: "active" for name in worker.MOTION_CONTROLLERS})
        self.assertIn(FACTORY, worker.validate_controllers(values, True))
        with self.assertRaisesRegex(RuntimeError, "运动控制器"):
            worker.validate_controllers(values)

    def test_unknown_active_is_rejected_in_preflight(self):
        with self.assertRaisesRegex(RuntimeError, "foreign_controller"):
            worker.validate_controllers(controllers({"foreign_controller": "active"}), True)
        worker.validate_controllers(controllers({"foreign_controller": "inactive"}), True)

    def test_reserved_temporary_controller_type_must_match(self):
        values = controllers({"fairino_keyboard_angle": "inactive"})
        values[1].type = "unrelated_controller"
        with self.assertRaisesRegex(RuntimeError, "类型不匹配"):
            worker.validate_controllers(values, True)

    def test_missing_command_controller_duplicate_and_unknown_state(self):
        for values in ([], controllers() * 2,
                       controllers({"known": "transitioning"}),
                       [NS(name=COMMAND, state="inactive", type="command")]):
            with self.subTest(values=values), self.assertRaises(RuntimeError):
                worker.validate_controllers(values, True)


class NavigationIdentity(unittest.TestCase):
    def test_not_found_inactive_and_masked_or_damaged(self):
        result = NS(returncode=1, stdout="LoadState=not-found\nActiveState=inactive\n")
        with patch.object(worker.subprocess, "run", return_value=result):
            self.assertEqual(worker.read_navigation_state()["LoadState"], "not-found")
        for load in ("masked", "error", "bad-setting", "not-found"):
            result = NS(returncode=0, stdout=f"LoadState={load}\nActiveState=active\n")
            with self.subTest(load=load), patch.object(worker.subprocess, "run", return_value=result):
                with self.assertRaisesRegex(RuntimeError, "部署"):
                    worker.read_navigation_state()

    def test_only_exact_managed_identity_may_be_stopped(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / worker.UNIT
            good = ("# Managed by fairino-robot-toolkit: explicit 2D navigation\n"
                    "[Service]\nUser=fairino\nGroup=fairino\n"
                    "ExecStart=/bin/bash " + worker.RUNTIME_SCRIPT + "\n")
            state = {**INACTIVE, "LoadState": "loaded", "ActiveState": "active",
                     "FragmentPath": str(path)}
            with patch.object(worker, "UNIT_PATH", path):
                path.write_text(good)
                worker.require_managed_navigation(state)
                for bad in (good.replace("User=fairino", "User=root"),
                            good + "ExecStart=/bin/true\n",
                            good.replace("# Managed", "# Unmanaged"),
                            good.replace(worker.RUNTIME_SCRIPT, "/tmp/runtime.sh")):
                    path.write_text(bad)
                    with self.assertRaises(RuntimeError):
                        worker.require_managed_navigation(state)
                path.write_text(good)
                with self.assertRaises(RuntimeError):
                    worker.require_managed_navigation({**state, "DropInPaths": "/tmp/override.conf"})
                path.write_text("")
                result = NS(returncode=0, stdout="\n".join(f"{k}={v}" for k, v in state.items()))
                with patch.object(worker.subprocess, "run", return_value=result):
                    with self.assertRaisesRegex(RuntimeError, "文件为空"):
                        worker.read_navigation_state()

    def test_inactive_does_not_send_stop(self):
        session = worker.RecoverySession.__new__(worker.RecoverySession)
        session.report = {}
        with patch.object(worker, "read_navigation_state", return_value=INACTIVE), \
                patch.object(worker.subprocess, "Popen") as popen:
            self.assertFalse(session.stop_navigation())
        popen.assert_not_called()
        self.assertTrue(session.report["navigation_stopped"])

    def test_active_stop_only_once_then_verify(self):
        session = worker.RecoverySession.__new__(worker.RecoverySession)
        session.report = {}
        session.healthy_stationary = Mock()
        process = Mock(returncode=0)
        process.poll.return_value = 0
        process.communicate.return_value = ("", "")
        active = {**INACTIVE, "LoadState": "loaded", "ActiveState": "active"}
        with patch.object(worker, "read_navigation_state", side_effect=[active, INACTIVE]), \
                patch.object(worker, "require_managed_navigation") as managed, \
                patch.object(worker.subprocess, "Popen", return_value=process) as popen:
            self.assertTrue(session.stop_navigation())
        managed.assert_called_once_with(active)
        popen.assert_called_once()
        self.assertEqual(popen.call_args.args[0], ["sudo", "-n", "systemctl", "stop", worker.UNIT])
        self.assertEqual(session.healthy_stationary.call_count, 2)

    def test_failed_stop_is_not_retried(self):
        session = worker.RecoverySession.__new__(worker.RecoverySession)
        session.report = {}
        session.healthy_stationary = Mock()
        process = Mock(returncode=1)
        process.poll.return_value = 1
        process.communicate.return_value = ("", "permission denied")
        with patch.object(worker, "read_navigation_state", return_value={**INACTIVE, "ActiveState": "active"}), \
                patch.object(worker, "require_managed_navigation"), \
                patch.object(worker.subprocess, "Popen", return_value=process) as popen:
            with self.assertRaisesRegex(RuntimeError, "permission denied"):
                session.stop_navigation()
        self.assertEqual(popen.call_count, 1)


class FakeRecovery:
    """Exercise real orchestration plus inherited single-shot reset/disable."""
    def __init__(self, *, execute=True, initial=None, unknown=False,
                 upper_error=0, pause_available=True, pause_success=True,
                 fail_reset=False, fail_disable=False, fail_unload=False,
                 fresh_failure=None, late_fault=False, restarted=False):
        self.session = s = worker.RecoverySession.__new__(worker.RecoverySession)
        s.part, s.execute, s.upper_time, s.upper_error = "base", execute, 1., upper_error
        s.report = dict(part="base", mode="recover" if execute else "check",
                        ok=False, actions=[], send_attempted=False,
                        enable_sent=False, motion_sent=False,
                        ready_to_recover=False, navigation_stopped=False,
                        temporary_controllers_removed=False)
        s.state = initial or hardware(0x2208, 30)
        s.received, s.read_error = 0., None
        self.clock = 1.
        self.commands, self.requests = [], []
        self.controller_items = controllers({
            FACTORY: "active", "angle_controller": "active",
            **{name: "active" for name in TEMPLATES},
            **({"foreign_controller": "active"} if unknown else {})})
        self.fail_unload = fail_unload

        def fresh(timeout=5.):
            self.clock += .1
            s.received = s.upper_time = self.clock
            if fresh_failure and fresh_failure == s.report.get("phase"):
                raise RuntimeError("没有持续更新的硬件反馈")
            if late_fault and s.report.get("phase") == "verify_final":
                s.state = hardware(0x2208, 30)
            s.report["latest"] = s.state
            return s.state
        s.fresh = fresh

        def spin(*args, **kwargs):
            self.clock += .02
            s.received = s.upper_time = self.clock
        s.ros = NS(spin_once=spin)

        def future(value):
            return NS(done=lambda: True, result=lambda: value)

        owner = self
        class Client:
            def __init__(self, topic):
                self.topic = topic
            def wait_for_service(self, **kwargs):
                return True
            def service_is_ready(self):
                return pause_available
            def call_async(self, request):
                owner.requests.append((self.topic, request))
                if self.topic.endswith("/list_controllers"):
                    return future(NS(controller=copy.deepcopy(owner.controller_items)))
                if self.topic.endswith("/pause"):
                    return future(NS(success=pause_success, message="pause rejected"))
                if self.topic.endswith("/switch_controller"):
                    assert request.activate_controllers == []
                    assert request.strictness == 2
                    for item in owner.controller_items:
                        if item.name in request.deactivate_controllers:
                            item.state = "inactive"
                    return future(NS(ok=True))
                if self.topic.endswith("/unload_controller"):
                    if not fail_unload:
                        owner.controller_items[:] = [c for c in owner.controller_items if c.name != request.name]
                    return future(NS(ok=not fail_unload))
                raise AssertionError("unexpected service: " + self.topic)
        s.node = NS(create_client=lambda kind, topic: Client(topic),
                    destroy_client=Mock(), destroy_node=Mock())

        class Action:
            def wait_for_server(self, **kwargs):
                return True
            def send_goal_async(self, goal):
                owner.commands.append(goal.command)
                def result():
                    if goal.command == "reseterror()":
                        if fail_reset:
                            raise RuntimeError("ROS 请求超时；结果不确定，未自动重试")
                        s.state = hardware(0x2237)
                    elif goal.command == "disable()":
                        assert base.faults_clear(s.state)
                        if fail_disable:
                            raise RuntimeError("下使能结果不确定")
                        s.state = hardware()
                    else:
                        raise AssertionError("forbidden action: " + goal.command)
                    return future(NS(status=4, result=NS(error_code="0xC000")))
                return future(NS(accepted=True, get_result_async=result))

        def request():
            return NS(timeout=NS(sec=0))
        self.modules = {
            "controller_manager_msgs.srv": NS(
                ListControllers=NS(Request=request), SwitchController=NS(Request=request),
                UnloadController=NS(Request=request)),
            "std_srvs.srv": NS(Trigger=NS(Request=request)),
            "rclpy.action": NS(ActionClient=lambda *args: Action()),
            "fairino_hardware_command_msgs.action": NS(StringCommand=NS(Goal=NS)),
        }
        self.restarted = restarted

    def run(self):
        nav_calls = [0]
        def navigation():
            nav_calls[0] += 1
            if self.restarted and nav_calls[0] >= 3:
                return {**INACTIVE, "LoadState": "loaded", "ActiveState": "active"}
            return dict(INACTIVE)
        with patch.dict(sys.modules, self.modules), \
                patch.object(worker.time, "monotonic", lambda: self.clock), \
                patch.object(worker, "read_navigation_state", side_effect=navigation), \
                contextlib.redirect_stdout(io.StringIO()):
            try:
                self.session.run()
            except RuntimeError as exc:
                self.error = str(exc)
            else:
                self.error = None
        return self

    def mutations(self):
        return [(topic, request) for topic, request in self.requests
                if not topic.endswith("/list_controllers")]


class RecoveryWorkflow(unittest.TestCase):
    def test_recovers_single_shot_disables_then_unloads_only_temporary(self):
        fake = FakeRecovery().run()
        self.assertIsNone(fake.error)
        self.assertEqual(fake.commands, ["reseterror()", "disable()"])
        report = fake.session.report
        for field in ("ok", "cleared_and_disabled", "temporary_controllers_removed", "navigation_stopped"):
            self.assertTrue(report[field])
        self.assertEqual(report["phase"], "complete")
        self.assertFalse(report["enable_sent"])
        self.assertFalse(report["motion_sent"])
        self.assertEqual([topic.rsplit("/", 1)[-1] for topic, _ in fake.mutations()],
                         ["pause", "switch_controller", "unload_controller", "unload_controller"])
        removed = [req.name for topic, req in fake.mutations() if topic.endswith("/unload_controller")]
        self.assertEqual(set(removed), set(TEMPLATES))
        remaining = {item.name: item.state for item in fake.controller_items}
        self.assertEqual(remaining[FACTORY], "inactive")
        self.assertEqual(remaining["angle_controller"], "inactive")
        self.assertEqual(remaining[COMMAND], "active")

    def test_healthy_disabled_skips_reset_and_disable(self):
        fake = FakeRecovery(initial=hardware()).run()
        self.assertIsNone(fake.error)
        self.assertEqual(fake.commands, [])
        self.assertTrue(fake.session.report["ok"])

    def test_fault_clear_enabled_disables_without_reset(self):
        fake = FakeRecovery(initial=hardware(0x2237)).run()
        self.assertIsNone(fake.error)
        self.assertEqual(fake.commands, ["disable()"])

    def test_readonly_reports_ready_without_mutation(self):
        fake = FakeRecovery(execute=False).run()
        self.assertIsNone(fake.error)
        self.assertEqual(fake.mutations(), [])
        self.assertEqual(fake.commands, [])
        self.assertEqual(fake.session.report["actions"], [])
        self.assertTrue(fake.session.report["ready_to_recover"])
        self.assertFalse(fake.session.report["cleared_and_disabled"])

    def test_all_preflight_blocks_precede_mutations(self):
        fatal = hardware(0x2208, 30)
        fatal["error_hex"] = "0x0F100001"
        cases = [dict(unknown=True), dict(upper_error=30),
                 dict(initial=hardware(velocity=.006)),
                 dict(initial=hardware(command=.006)),
                 dict(initial=hardware(0x2207)), dict(initial=fatal),
                 dict(fresh_failure="preflight")]
        for args in cases:
            with self.subTest(args=args):
                fake = FakeRecovery(**args).run()
                self.assertIsNotNone(fake.error)
                self.assertEqual(fake.commands, [])
                self.assertEqual(fake.mutations(), [])
                self.assertEqual(fake.session.report["actions"], [])

    def test_readonly_block_is_structured_without_mutations(self):
        fake = FakeRecovery(execute=False, unknown=True).run()
        self.assertIsNone(fake.error)
        self.assertTrue(fake.session.report["ok"])
        self.assertFalse(fake.session.report["ready_to_recover"])
        self.assertIn("foreign_controller", fake.session.report["blocked_reason"])
        self.assertEqual(fake.mutations(), [])

    def test_missing_or_failed_pause_prevents_reset_and_switch(self):
        for args in (dict(pause_available=False), dict(pause_success=False)):
            with self.subTest(args=args):
                fake = FakeRecovery(**args).run()
                self.assertIsNotNone(fake.error)
                self.assertEqual(fake.commands, [])
                self.assertFalse(any(topic.endswith("/switch_controller") for topic, _ in fake.mutations()))
                self.assertEqual(fake.session.report["actions"][-1]["status"], "failed")

    def test_uncertain_reset_or_disable_not_retried_and_never_unloads(self):
        for args, expected in ((dict(fail_reset=True), ["reseterror()"]),
                               (dict(fail_disable=True), ["reseterror()", "disable()"])):
            with self.subTest(args=args):
                fake = FakeRecovery(**args).run()
                self.assertIsNotNone(fake.error)
                self.assertEqual(fake.commands, expected)
                self.assertFalse(any(topic.endswith("/unload_controller") for topic, _ in fake.mutations()))
                self.assertFalse(fake.session.report["ok"])

    def test_fault_recurrence_and_stale_final_feedback_block_cleanup(self):
        for args in (dict(late_fault=True), dict(fresh_failure="verify_final")):
            with self.subTest(args=args):
                fake = FakeRecovery(**args).run()
                self.assertIsNotNone(fake.error)
                self.assertFalse(fake.session.report["ok"])
                self.assertFalse(any(topic.endswith("/unload_controller") for topic, _ in fake.mutations()))

    def test_failed_unload_is_not_reported_as_ready(self):
        fake = FakeRecovery(fail_unload=True).run()
        self.assertIsNotNone(fake.error)
        self.assertFalse(fake.session.report["ok"])
        self.assertTrue(base.is_clear(fake.session.state))
        self.assertEqual(fake.session.report["actions"][-1]["status"], "failed")

    def test_navigation_restart_prevents_complete_report(self):
        fake = FakeRecovery(restarted=True).run()
        self.assertIn("重新启动", fake.error)
        self.assertFalse(fake.session.report["ok"])


class Entrypoint(unittest.TestCase):
    def parse(self, output):
        rows = [line for line in output.splitlines() if line.startswith(worker.PREFIX)]
        self.assertEqual(len(rows), 1)
        return json.loads(rows[0][len(worker.PREFIX):])

    def test_occupied_lock_has_no_ros_or_side_effects_and_is_not_deleted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / ".fairino_keyboard.lock"
            with path.open("a") as owner:
                fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
                output = io.StringIO()
                with patch.object(worker.Path, "home", return_value=Path(directory)), \
                        patch.object(worker, "RecoverySession") as session, \
                        contextlib.redirect_stdout(output):
                    self.assertEqual(worker.main(execute=True), 1)
                session.assert_not_called()
                report = self.parse(output.getvalue())
                self.assertFalse(report["ok"])
                self.assertIn("占用控制权", report["failure"])
                self.assertTrue(path.exists())

    def test_cleanup_failure_still_emits_exactly_one_failure_report(self):
        node = NS(destroy_node=Mock(side_effect=RuntimeError("destroy failed")))
        session = NS(report={"ok": True, "phase": "complete"}, state=None,
                     node=node, run=Mock())
        ros = NS(init=Mock(), shutdown=Mock(side_effect=RuntimeError("shutdown failed")))
        output = io.StringIO()
        with patch.dict(sys.modules, {"rclpy": ros}), \
                patch.object(worker, "RecoverySession", return_value=session), \
                contextlib.redirect_stdout(output):
            self.assertEqual(worker.main(False), 1)
        report = self.parse(output.getvalue())
        self.assertFalse(report["ok"])
        self.assertEqual(len(report["cleanup_errors"]), 2)
        ros.shutdown.assert_called_once()

    def test_interrupt_during_cleanup_still_releases_control_lock(self):
        with tempfile.TemporaryDirectory() as directory:
            node = NS(destroy_node=Mock(side_effect=KeyboardInterrupt))
            session = NS(report={"ok": True, "phase": "complete"}, state=None,
                         node=node, run=Mock())
            ros = NS(init=Mock(), shutdown=Mock())
            output = io.StringIO()
            with patch.object(worker.Path, "home", return_value=Path(directory)), \
                    patch.dict(sys.modules, {"rclpy": ros}), \
                    patch.object(worker, "RecoverySession", return_value=session), \
                    contextlib.redirect_stdout(output):
                self.assertEqual(worker.main(True), 1)
            self.assertFalse(self.parse(output.getvalue())["ok"])
            ros.shutdown.assert_called_once()
            with (Path(directory) / ".fairino_keyboard.lock").open("a") as probe:
                fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def test_exception_after_ok_still_fails_and_restores_signal_handlers(self):
        session = NS(report={"ok": True, "phase": "final_check"}, state=None,
                     node=NS(destroy_node=Mock()), run=Mock(side_effect=RuntimeError("late failure")))
        ros = NS(init=Mock(), shutdown=Mock())
        output = io.StringIO()
        old = {sig: worker.signal.getsignal(sig) for sig in
               (worker.signal.SIGTERM, worker.signal.SIGHUP)}
        with patch.dict(sys.modules, {"rclpy": ros}), \
                patch.object(worker, "RecoverySession", return_value=session), \
                contextlib.redirect_stdout(output):
            self.assertEqual(worker.main(False), 1)
        self.assertFalse(self.parse(output.getvalue())["ok"])
        for sig, handler in old.items():
            self.assertEqual(worker.signal.getsignal(sig), handler)

    def test_termination_handler_cleans_up_and_reports_uncertain_result(self):
        session = NS(report={"ok": False, "phase": "reset"}, state=None,
                     node=NS(destroy_node=Mock()))
        def interrupt():
            handler = worker.signal.getsignal(worker.signal.SIGTERM)
            handler(worker.signal.SIGTERM, None)
        session.run = interrupt
        ros = NS(init=Mock(), shutdown=Mock())
        output = io.StringIO()
        with patch.dict(sys.modules, {"rclpy": ros}), \
                patch.object(worker, "RecoverySession", return_value=session), \
                contextlib.redirect_stdout(output):
            self.assertEqual(worker.main(False), 1)
        report = self.parse(output.getvalue())
        self.assertFalse(report["ok"])
        self.assertIn("退出信号", report["failure"])
        session.node.destroy_node.assert_called_once()
        ros.shutdown.assert_called_once()

    def test_interrupted_operation_is_structured_and_cleans_up(self):
        session = NS(report={"ok": False, "phase": "reset"}, state=None,
                     node=NS(destroy_node=Mock()), run=Mock(side_effect=KeyboardInterrupt))
        ros = NS(init=Mock(), shutdown=Mock())
        output = io.StringIO()
        with patch.dict(sys.modules, {"rclpy": ros}), \
                patch.object(worker, "RecoverySession", return_value=session), \
                contextlib.redirect_stdout(output):
            self.assertEqual(worker.main(False), 1)
        self.assertIn("用户中断", self.parse(output.getvalue())["failure"])
        session.node.destroy_node.assert_called_once()
        ros.shutdown.assert_called_once()


if __name__ == "__main__":
    unittest.main()
