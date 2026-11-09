"""Explicit chassis recovery; never enables motors, moves or resumes a goal."""
import fcntl
import json
import signal
from pathlib import Path
import subprocess
import sys
import time

if __package__:
    from .fault_reset_remote import (
        ResetSession, ALLOWED_CONTROLLERS, require_resettable,
        require_stationary, require_idle, faults_clear, is_clear)
else:
    from fault_reset_remote import (
        ResetSession, ALLOWED_CONTROLLERS, require_resettable,
        require_stationary, require_idle, faults_clear, is_clear)

PREFIX = "CHASSIS_RECOVERY_RESULT "
UNIT = "fairino-navigation.service"
UNIT_PATH = Path("/etc/systemd/system") / UNIT
RUNTIME_SCRIPT = "/home/fairino/.local/share/fairino-navigation/runtime.sh"
TEMPORARY_CONTROLLERS = {
    "fairino_keyboard_angle": "joint_trajectory_controller/JointTrajectoryController",
    "fairino_keyboard_chassis": "eight_dof_chassis_controller/EightDofChassisController",
}
MOTION_CONTROLLERS = frozenset(TEMPORARY_CONTROLLERS) | {
    "eight_dof_chassis_controller", "angle_controller", "velocity_controller",
}


def validate_controllers(controllers, allow_motion=False):
    states = {}
    for controller in controllers:
        name, state = controller.name, controller.state
        if not name or name in states or state not in {"active", "inactive", "unconfigured", "finalized"}:
            raise RuntimeError("控制器清单不完整、重复或状态不明确")
        if name in TEMPORARY_CONTROLLERS and getattr(controller, "type", None) != TEMPORARY_CONTROLLERS[name]:
            raise RuntimeError("遗留控制器类型不匹配，未接管：" + name)
        states[name] = state
    active = {name for name, state in states.items() if state == "active"}
    allowed = ALLOWED_CONTROLLERS | (MOTION_CONTROLLERS if allow_motion else set())
    unexpected = active - allowed
    if unexpected:
        raise RuntimeError("仍有未知或未停用的运动控制器：" + ", ".join(sorted(unexpected)))
    if states.get("fairino_hardware_command_controller") != "active":
        raise RuntimeError("厂家指令控制器未激活，无法确认恢复操作")
    return states


def read_navigation_state():
    result = subprocess.run(
        ["systemctl", "show", UNIT,
         "--property=LoadState,ActiveState,SubState,FragmentPath,DropInPaths"],
        capture_output=True, text=True, timeout=5.)
    values = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    if values.get("LoadState") == "not-found" and values.get("ActiveState") == "inactive":
        return values
    if result.returncode or values.get("LoadState") != "loaded":
        raise RuntimeError("导航服务缺失有效定义、被屏蔽或已损坏；请先检查部署，恢复接入不会改写服务文件")
    if values.get("FragmentPath") == str(UNIT_PATH):
        if UNIT_PATH.is_symlink() or not UNIT_PATH.is_file() or not UNIT_PATH.stat().st_size:
            raise RuntimeError("导航服务文件为空、被屏蔽或损坏；请先恢复部署")
    if values.get("ActiveState") not in {"active", "activating", "deactivating", "inactive", "failed"}:
        raise RuntimeError("导航服务状态不明确，未发送停止请求")
    return values


def require_managed_navigation(values):
    if values.get("FragmentPath") != str(UNIT_PATH) or values.get("DropInPaths"):
        raise RuntimeError("导航服务路径或附加配置不属于已知部署，未停止服务")
    if UNIT_PATH.is_symlink() or not UNIT_PATH.is_file():
        raise RuntimeError("导航服务文件不是本工具管理的普通文件，未停止服务")
    lines = UNIT_PATH.read_text().splitlines()
    required = {
        "User=": "User=fairino",
        "Group=": "Group=fairino",
        "ExecStart=": "ExecStart=/bin/bash " + RUNTIME_SCRIPT,
    }
    if "# Managed by fairino-robot-toolkit: explicit 2D navigation" not in lines:
        raise RuntimeError("导航服务不由本工具管理，未停止服务")
    for prefix, expected in required.items():
        matches = [line.strip() for line in lines if line.strip().startswith(prefix)]
        if matches != [expected]:
            raise RuntimeError("导航服务执行身份或启动入口不匹配，未停止服务")


class RecoverySession(ResetSession):
    def __init__(self, execute=False):
        super().__init__("base", execute)
        self.report.update(
            mode="recover" if execute else "check", actions=[],
            navigation_stopped=False, temporary_controllers_removed=False,
            ready_to_recover=False, enable_sent=False, motion_sent=False)

    def action(self, name, callback):
        entry = {"action": name, "status": "attempted"}
        self.report["actions"].append(entry)
        try:
            result = callback()
        except (Exception, KeyboardInterrupt) as exc:
            entry.update(status="failed", failure=str(exc) or "用户中断；结果需复查")
            raise
        entry["status"] = "not_needed" if result is False else "complete"
        return result

    def controller_states(self, allow_motion=False):
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        try:
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError("无法查询控制器状态")
            result = self.wait(client.call_async(ListControllers.Request()), 5.)
            states = validate_controllers(result.controller, allow_motion)
            self.report["active_controllers"] = [name for name, state in states.items() if state == "active"]
            return states
        finally:
            self.node.destroy_client(client)

    def controllers(self, allow_base_stop=False):
        # Parent reset/disable operations may only run AFTER all motion
        # controllers have been confirmed inactive.
        if allow_base_stop:
            raise RuntimeError("恢复中必须先显式停用运动控制器")
        states = self.controller_states()
        return [name for name, state in states.items() if state == "active"]

    def healthy_stationary(self, *, disabled=False):
        state = self.fresh(timeout=.4)
        (require_idle if disabled else require_resettable)(state)
        self.interlock()
        if disabled and not is_clear(state):
            raise RuntimeError("故障复发或未确认全部下使能，未继续清理控制器")
        return state

    def observe_preflight(self):
        deadline = time.monotonic() + 2.
        while self.upper_time == 0 and time.monotonic() < deadline:
            self.ros.spin_once(self.node, timeout_sec=.02)
        deadline = time.monotonic() + .5
        while time.monotonic() < deadline:
            self.healthy_stationary()

    def stop_navigation(self):
        self.report["phase"] = "stop_navigation"
        state = read_navigation_state()
        self.report["navigation_before"] = state
        if state["ActiveState"] in {"inactive", "failed"}:
            self.report["navigation_stopped"] = True
            return False
        require_managed_navigation(state)
        self.healthy_stationary()
        # Only this invocation's systemctl client is terminated on uncertainty;
        # no controller/worker process is killed and the request is not resent.
        process = subprocess.Popen(
            ["sudo", "-n", "systemctl", "stop", UNIT],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        deadline = time.monotonic() + 30.
        try:
            while process.poll() is None:
                if time.monotonic() >= deadline:
                    raise RuntimeError("导航停止请求超时；停止结果不确定，未重发")
                self.ros.spin_once(self.node, timeout_sec=.02)
                if self.read_error or time.monotonic() - self.received > .4:
                    raise RuntimeError("停止导航时硬件反馈中断；停止结果不确定，未重发")
                require_stationary(self.state)
                self.interlock()
            stdout, stderr = process.communicate(timeout=1.)
            if process.returncode:
                raise RuntimeError("导航服务停止未成功：" + (stderr or stdout).strip()[-400:])
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.communicate(timeout=1.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate(timeout=1.)
        after = read_navigation_state()
        self.report["navigation_after"] = after
        if after.get("ActiveState") not in {"inactive", "failed"}:
            raise RuntimeError("导航服务停止尚未确认，未继续恢复")
        self.report["navigation_stopped"] = True
        self.healthy_stationary()
        return True

    def pause_for_recovery(self):
        self.report["phase"] = "pause_odometry"
        self.healthy_stationary()
        self.pause_odometry()
        if not self.report.get("odometry_paused"):
            raise RuntimeError("未确认里程计已暂停；请检查底盘部署与里程计守护服务")

    def stop_motion_controllers(self):
        from controller_manager_msgs.srv import SwitchController
        self.report["phase"] = "stop_controllers"
        states = self.controller_states(allow_motion=True)
        active = sorted(name for name in MOTION_CONTROLLERS if states.get(name) == "active")
        self.healthy_stationary()
        if active:
            client = self.node.create_client(SwitchController, "/controller_manager/switch_controller")
            try:
                if not client.wait_for_service(timeout_sec=5.):
                    raise RuntimeError("控制器停用服务不可用")
                request = SwitchController.Request()
                request.activate_controllers = []
                request.deactivate_controllers = active
                request.strictness = 2
                request.activate_asap = False
                request.timeout.sec = 3
                self.healthy_stationary()
                result = self.wait(client.call_async(request), 5., monitor="stationary")
                if not result.ok:
                    raise RuntimeError("运动控制器停用失败；未发送故障复位")
            finally:
                self.node.destroy_client(client)
        after = self.controller_states()
        self.report["controllers_after_stop"] = after
        self.report["stopped_controllers"] = active
        self.healthy_stationary()
        return bool(active)

    def unload_temporary(self):
        from controller_manager_msgs.srv import UnloadController
        self.report["phase"] = "unload_controllers"
        removed = []
        self.report["unloaded_controllers"] = removed
        for name in TEMPORARY_CONTROLLERS:
            states = self.controller_states()
            self.healthy_stationary(disabled=True)
            if name not in states:
                continue
            client = self.node.create_client(UnloadController, "/controller_manager/unload_controller")
            try:
                if not client.wait_for_service(timeout_sec=5.):
                    raise RuntimeError("控制器卸载服务不可用：" + name)
                request = UnloadController.Request()
                request.name = name
                self.healthy_stationary(disabled=True)
                result = self.wait(client.call_async(request), 5., monitor=True)
                if not result.ok:
                    raise RuntimeError("遗留控制器卸载未成功：" + name)
            finally:
                self.node.destroy_client(client)
            if name in self.controller_states():
                raise RuntimeError("卸载返回后控制器仍存在：" + name)
            removed.append(name)
        return bool(removed)

    def run(self):
        self.report["phase"] = "preflight"
        self.report["before"] = self.fresh()
        try:
            self.report["controllers_before"] = self.controller_states(allow_motion=True)
            navigation = read_navigation_state()
            if navigation["ActiveState"] not in {"inactive", "failed"}:
                require_managed_navigation(navigation)
            self.report["navigation_before"] = navigation
            self.observe_preflight()
        except RuntimeError as exc:
            self.report["blocked_reason"] = str(exc)
            if self.execute:
                raise
            self.report.update(ok=True, ready_to_recover=False,
                               cleared_and_disabled=bool(self.state and is_clear(self.state)))
            return self.report
        self.report["ready_to_recover"] = True
        if not self.execute:
            self.report.update(ok=True, cleared_and_disabled=is_clear(self.state),
                               navigation_stopped=navigation["ActiveState"] in {"inactive", "failed"},
                               temporary_controllers_removed=not (
                                   set(self.report["controllers_before"]) & set(TEMPORARY_CONTROLLERS)))
            return self.report
        self.action("stop_navigation", self.stop_navigation)
        self.action("pause_odometry", self.pause_for_recovery)
        self.action("deactivate_motion_controllers", self.stop_motion_controllers)
        self.healthy_stationary()
        if faults_clear(self.state):
            self.report["skipped"] = "当前无故障，未发送复位"
        else:
            self.action("reseterror", self.reset_once)
        if not is_clear(self.fresh()):
            self.action("disable", self.disable_all)
        self.action("verify_disabled", self.verify_final)
        self.action("unload_temporary_controllers", self.unload_temporary)
        self.report["phase"] = "final_check"
        states = self.controller_states()
        self.report["controllers_after"] = states
        self.healthy_stationary(disabled=True)
        navigation = read_navigation_state()
        if navigation["ActiveState"] not in {"inactive", "failed"}:
            raise RuntimeError("恢复期间导航服务重新启动，未报告恢复成功")
        if set(states) & set(TEMPORARY_CONTROLLERS):
            raise RuntimeError("遗留控制器未全部卸载，未报告恢复成功")
        self.report.update(ok=True, phase="complete", cleared_and_disabled=True,
                           temporary_controllers_removed=True, navigation_stopped=True)
        return self.report


def main(execute=False):
    report = dict(part="base", mode="recover" if execute else "check", ok=False,
                  phase="preflight", send_attempted=False, enable_sent=False,
                  motion_sent=False, actions=[])
    session = lock = None
    initialized = False
    previous_handlers = {}

    def interrupted(signum, frame):
        raise KeyboardInterrupt("收到退出信号；已发出的恢复操作需只读复查，未自动重试")

    try:
        for signum in (signal.SIGTERM, signal.SIGHUP):
            previous_handlers[signum] = signal.signal(signum, interrupted)
        if execute:
            lock = (Path.home()/".fairino_keyboard.lock").open("a")
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError("其他底盘控制会话仍占用控制权；请正常结束原会话，未强制接管") from None
        import rclpy
        rclpy.init()
        initialized = True
        session = RecoverySession(execute)
        report = session.report
        session.run()
    except (Exception, KeyboardInterrupt) as exc:
        report["ok"] = False
        report["failure"] = str(exc) or "用户中断；已发出的操作需先只读复查，不自动重试"
    finally:
        if session and session.state:
            report["after"] = session.state
        cleanup_errors = []
        for label, callback in (
            ("关闭恢复节点", session.node.destroy_node if session else None),
            ("关闭ROS上下文", rclpy.shutdown if initialized else None),
            ("释放控制锁", lock.close if lock else None),
        ):
            if callback is not None:
                try:
                    callback()
                except (Exception, KeyboardInterrupt) as exc:
                    cleanup_errors.append(label + "：" + (str(exc) or "用户中断"))
        for signum, previous in previous_handlers.items():
            try:
                signal.signal(signum, previous)
            except (Exception, KeyboardInterrupt) as exc:
                cleanup_errors.append("恢复信号处理：" + (str(exc) or "用户中断"))
        if cleanup_errors:
            report.update(ok=False, cleanup_errors=cleanup_errors)
            report["failure"] = report.get("failure") or "恢复结束清理未确认：" + "；".join(cleanup_errors)
        print(PREFIX + json.dumps(report, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in {"check", "recover"}:
        raise SystemExit("Use the chassis recovery client with check/recover")
    raise SystemExit(main(sys.argv[1] == "recover"))
