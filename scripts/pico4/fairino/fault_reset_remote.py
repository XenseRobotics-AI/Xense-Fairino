"""Single-shot FAIRINO fault reset. Executed over SSH; no motion interface."""
import fcntl
import json
import math
import os
import sys
import time
from pathlib import Path

PREFIX = "FAULT_RESET_RESULT "
ALLOWED_CONTROLLERS = {
    "fairino_hardware_state_broadcaster", "fairino_hardware_command_controller",
    "joint_state_broadcaster",
}


def drive_state(word):
    """Decode CiA 402 states; bit 2 alone misreads 0x220F as enabled."""
    for mask, value, label in (
        (0x4F, 0x0F, "fault_reaction"), (0x4F, 0x08, "fault"),
        (0x6F, 0x27, "operation_enabled"), (0x6F, 0x07, "quick_stop"),
        (0x4F, 0x40, "switch_on_disabled"), (0x6F, 0x21, "ready"),
        (0x6F, 0x23, "switched_on"), (0x4F, 0x00, "not_ready"),
    ):
        if int(word) & mask == value:
            return label
    return "unknown"


def snapshot(msg, count):
    names = list(msg.joint_names)
    if len(names) != count or set(names) != {f"j{i}" for i in range(1, count + 1)}:
        raise RuntimeError("关节数量或名称与该控制板不符")
    arrays = {key: list(getattr(msg, key)) for key in
              ("stateword", "errorcode", "position", "velocity")}
    if any(len(values) != count for values in arrays.values()):
        raise RuntimeError("关节反馈不完整")
    if any(not math.isfinite(float(x)) for key in ("position", "velocity") for x in arrays[key]):
        raise RuntimeError("关节反馈含非有限数值")
    states = [drive_state(s) for s in arrays["stateword"]]
    return {
        "joint_names": names, **arrays, "drive_states": states,
        "enabled_axes": [n for n, s in zip(names, states) if s == "operation_enabled"],
        "error_hex": f"0x{int(msg.hexerrorcode_curerrcode) & 0xFFFFFFFF:08X}",
        "joint_errors": {n: int(e) for n, e in zip(names, arrays["errorcode"]) if e},
        "max_abs_velocity_rad_s": max(abs(float(v)) for v in arrays["velocity"]),
    }


def require_idle(state):
    if any(s in {"operation_enabled", "quick_stop", "unknown", "not_ready"}
           for s in state["drive_states"]):
        raise RuntimeError("电机仍使能、正在快停或驱动状态未就绪；先退出控制程序")
    if state["max_abs_velocity_rad_s"] > .005:
        raise RuntimeError("关节尚未静止")


def is_clear(state):
    return (state["error_hex"] == "0x00000000" and not state["joint_errors"]
            and all(s in {"switch_on_disabled", "ready", "switched_on"}
                    for s in state["drive_states"]))


def require_interlock(part, gpio=None, upper_error=None, upper_age=math.inf):
    if part == "upper":
        if gpio != {"56": "0", "57": "0"}:
            raise RuntimeError("外部急停输入未释放或不可读取")
    elif upper_error != 0 or upper_age > .4:
        raise RuntimeError("上半身急停/故障转发非零或已超时；先处理上半身")


class ResetSession:
    def __init__(self, part, execute):
        import rclpy
        from hardware_state_broadcaster_msgs.msg import HardwareState
        from std_msgs.msg import Int32
        self.ros = rclpy
        self.node = rclpy.create_node(f"fairino_fault_reset_{part}_{os.getpid()}")
        self.part, self.execute = part, execute
        self.state = None
        self.received = self.serial = self.upper_time = 0
        self.upper_error = None
        self.header_stamp = None
        self.read_error = None
        self.report = {"part": part, "mode": "reset" if execute else "check",
                       "command": "reseterror()", "send_attempted": False,
                       "enable_sent": False, "motion_sent": False, "ok": False}
        self.sub = self.node.create_subscription(HardwareState, "/hardware_states", self.on_hw, 1)
        if part == "base":
            self.upper_sub = self.node.create_subscription(Int32, "/upper_estop_error_code", self.on_upper, 1)

    def on_hw(self, msg):
        try:
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if stamp == self.header_stamp:
                return
            self.state = snapshot(msg, 20 if self.part == "upper" else 8)
            self.header_stamp = stamp
            self.received = time.monotonic()
            self.serial += 1
            self.read_error = None
        except Exception as exc:
            self.read_error = str(exc)

    def on_upper(self, msg):
        self.upper_error, self.upper_time = int(msg.data), time.monotonic()

    def fresh(self, timeout=5.):
        previous = self.serial
        deadline = time.monotonic() + timeout
        while self.serial == previous and time.monotonic() < deadline:
            self.ros.spin_once(self.node, timeout_sec=.02)
            if self.read_error:
                raise RuntimeError(self.read_error)
        if self.serial == previous or time.monotonic() - self.received > .4:
            raise RuntimeError("没有持续更新的硬件反馈")
        self.report["latest"] = self.state
        return self.state

    def interlock(self):
        if self.part == "upper":
            try:
                gpio = {str(i): Path(f"/sys/class/gpio/gpio{i}/value").read_text().strip() for i in (56, 57)}
            except OSError:
                gpio = None
            self.report["gpio"] = gpio
            require_interlock(self.part, gpio=gpio)
        else:
            age = time.monotonic() - self.upper_time
            self.report["upper_error"] = self.upper_error
            self.report["upper_feedback_age_s"] = age if self.upper_time else None
            require_interlock(self.part, upper_error=self.upper_error, upper_age=age)

    def wait(self, future, timeout, monitor=False):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.ros.spin_once(self.node, timeout_sec=.02)
            if monitor:
                if self.read_error or time.monotonic() - self.received > .4:
                    raise RuntimeError("等待复位结果时硬件反馈中断；结果不确定，未自动重试")
                require_idle(self.state)
                self.interlock()
        if not future.done():
            raise RuntimeError("ROS 请求超时；结果不确定，未自动重试")
        return future.result()

    def controllers(self):
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError("无法查询控制器状态")
        result = self.wait(client.call_async(ListControllers.Request()), 5.)
        active = [c.name for c in result.controller if c.state == "active"]
        self.report["active_controllers"] = active
        unexpected = set(active) - ALLOWED_CONTROLLERS
        if unexpected:
            raise RuntimeError("仍有运动控制器运行：" + ", ".join(sorted(unexpected)))
        if "fairino_hardware_command_controller" not in active:
            raise RuntimeError("厂家指令控制器未激活")

    def preflight(self):
        self.controllers()
        if self.part == "base":
            deadline = time.monotonic() + 2.
            while self.upper_time == 0 and time.monotonic() < deadline:
                self.ros.spin_once(self.node, timeout_sec=.02)
        # Observe for half a second, rather than accept one stationary sample.
        deadline = time.monotonic() + .5
        while time.monotonic() < deadline:
            require_idle(self.fresh())
            self.interlock()
        if int(self.state["error_hex"], 16) >> 24 == 0x0F:
            raise RuntimeError("FATAL 级别故障不支持 reseterror；需要排查底层原因")

    def run(self):
        self.report["before"] = self.fresh()
        try:
            self.preflight()
        except RuntimeError as exc:
            self.report["blocked_reason"] = str(exc)
            if self.execute:
                raise
            self.report.update(ok=True, ready_to_reset=False, cleared_and_disabled=is_clear(self.state))
            return self.report
        self.report["ready_to_reset"] = True
        if not self.execute or is_clear(self.state):
            self.report.update(ok=True, cleared_and_disabled=is_clear(self.state))
            if self.execute:
                self.report["skipped"] = "当前已无故障，无需发送复位"
            return self.report
        from rclpy.action import ActionClient
        from fairino_hardware_command_msgs.action import StringCommand
        action = ActionClient(self.node, StringCommand, "/fairino_hardware_command_controller/command")
        if not action.wait_for_server(timeout_sec=5.):
            raise RuntimeError("厂家复位 Action 不可用")
        self.controllers()
        require_idle(self.fresh())
        self.interlock()
        # This is the only command this program can issue. No retry loop.
        goal = StringCommand.Goal()
        goal.command = "reseterror()"
        self.report["send_attempted"] = True
        print("FAULT_RESET_EVENT " + json.dumps({"part": self.part, "send_attempted": True}), flush=True)
        accepted = self.wait(action.send_goal_async(goal), 5., monitor=True)
        self.report["accepted"] = bool(accepted.accepted)
        if not accepted.accepted:
            raise RuntimeError("厂家拒绝了复位请求")
        result = self.wait(accepted.get_result_async(), 12., monitor=True)
        self.report.update(action_status=result.status, command_result=result.result.error_code)
        deadline = time.monotonic() + 5.
        clean_since = None
        frames = 0
        while time.monotonic() < deadline:
            state = self.fresh(timeout=.4)
            require_idle(state)
            self.interlock()
            if is_clear(state):
                clean_since = time.monotonic() if clean_since is None else clean_since
            else:
                clean_since = None
            frames += 1
        self.report.update(after=self.state, verification_frames=frames,
                           cleared_and_disabled=clean_since is not None and time.monotonic() - clean_since >= 2.)
        if result.status != 4 or str(result.result.error_code).strip().lower() != "0xc000":
            raise RuntimeError("厂家复位命令未报告成功，请查看命令返回值及复查状态")
        if not self.report["cleared_and_disabled"]:
            raise RuntimeError("故障未稳定清除或已再次触发；未重试，需排查原因")
        self.report["ok"] = True
        return self.report


def main(part, execute=False):
    report = {"part": part, "ok": False, "send_attempted": False}
    session = lock = None
    initialized = False
    try:
        import rclpy
        if execute:
            filename = ".fairino_upper_identify.lock" if part == "upper" else ".fairino_keyboard.lock"
            lock = open(Path.home() / filename, "a")
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError("遥操或键控程序仍占用控制权，请先退出") from None
        rclpy.init()
        initialized = True
        session = ResetSession(part, execute)
        report = session.report
        session.run()
    except (Exception, KeyboardInterrupt) as exc:
        report["failure"] = str(exc) or "用户中断；若已发送复位，请先只读复查"
    finally:
        print(PREFIX + json.dumps(report, ensure_ascii=False, allow_nan=False), flush=True)
        if session:
            session.node.destroy_node()
        if initialized:
            rclpy.shutdown()
        if lock:
            lock.close()
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] not in {"upper", "base"} or sys.argv[2] not in {"check", "reset"}:
        raise SystemExit("This worker is launched by clear_errors.py")
    raise SystemExit(main(sys.argv[1], sys.argv[2] == "reset"))
