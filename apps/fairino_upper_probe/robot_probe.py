#!/usr/bin/env python3
"""Hold J1–J20 enabled for the session; jog one selected joint at a time."""
import argparse
import fcntl
import json
import math
import queue
import signal
import sys
import tempfile
import threading
import time
from pathlib import Path

from probe_core import (JOINTS, ALL_MASK, SPEED_DEG_S, StepCancelled, check_hardware, check_packet,
                        fresh_request, plan_step, plan_target, plan_request, snapshot)

CONTROLLER = "fairino_upper_identify"
PREFIX = "@UPPERPROBE "


class Probe:
    # Shared defaults are retained for the TCP and VR subclasses.
    feedback_speed_check = True
    path_tracking_check = True

    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("fairino_upper_identify_guard")
        self.hw = None
        self.hw_time = 0.0
        self.baseline = None
        self.packet = None
        self.received = 0.0
        self.input_lock = threading.Lock()
        self.exit = threading.Event()
        self.output = queue.Queue(maxsize=40)
        self.fault = ""
        self.phase = "读取上半身状态"
        self.ready = False
        self.selected = None
        self.enable_attempted = False
        self.enabling = False
        self.enable_deadline = 0.0
        self.session_enabled = False
        self.hold_positions = None
        self.pending_enable_goal = self.pending_enable_result = None
        self.owned = False
        self.loaded = False
        self.active = False
        self.pending_goal = None
        self.goal = None
        self.last_request = 0
        self.cancel_count = None
        self.last_status = 0.0
        self.clients = {}
        self.params_dir = None
        self.sub = self.node.create_subscription(HardwareState, "/hardware_states", self.on_hw, 1)
        self.command_client = ActionClient(self.node, StringCommand,
            "/fairino_hardware_command_controller/command")
        self.trajectory_client = ActionClient(self.node, FollowJointTrajectory,
            f"/{CONTROLLER}/follow_joint_trajectory")
        threading.Thread(target=self.write_output, daemon=True).start()
        threading.Thread(target=self.read_input, daemon=True).start()

    def write_output(self):
        while True:
            text = self.output.get()
            try:
                print(text, flush=True)
            except (OSError, ValueError):
                self.exit.set()
            finally:
                self.output.task_done()

    def emit(self, **data):
        try:
            self.output.put_nowait(PREFIX + json.dumps(data, ensure_ascii=False, allow_nan=False))
        except queue.Full:
            self.exit.set()

    def read_input(self):
        try:
            for line in sys.stdin:
                packet = json.loads(line)
                if not isinstance(packet, dict):
                    raise ValueError("无效输入")
                with self.input_lock:
                    self.packet, self.received = packet, time.monotonic()
                if packet.get("exit"):
                    self.exit.set()
        except Exception as exc:
            self.fault = str(exc)
        finally:
            self.exit.set()

    def on_hw(self, msg):
        try:
            self.hw = snapshot({k: getattr(msg, k) for k in msg.get_fields_and_field_types()})
            self.hw_time = time.monotonic()
        except Exception as exc:
            self.fault = str(exc)

    def gpio(self):
        try:
            return [int(Path(f"/sys/class/gpio/gpio{i}/value").read_text().strip()) for i in (56, 57)]
        except (OSError, ValueError):
            return None

    def report(self, force=False, **extra):
        now = time.monotonic()
        if force or now - self.last_status > 0.1:
            self.emit(stamp=now, phase=self.phase, ready=self.ready, hardware=self.hw,
                      baseline=self.baseline, selected=self.selected, last_request=self.last_request,
                      hw_age=now-self.hw_time, enabling=self.enabling,
                      speed_protection=self.feedback_speed_check and not self.enabling,
                      path_tracking_check=self.path_tracking_check, **extra)
            self.last_status = now

    def guard(self, link=True, cancel=True):
        if self.fault:
            raise RuntimeError(self.fault)
        if self.exit.is_set() and link:
            raise RuntimeError("SSH 已断开或用户退出")
        now = time.monotonic()
        if self.enabling and now > self.enable_deadline:
            raise RuntimeError("全部关节使能阶段超过 12 秒，停止启动")
        try:
            check_hardware(self.hw, now-self.hw_time, self.gpio(), self.selected,
                           self.baseline, allow_enabled=self.enable_attempted,
                           check_speed=self.feedback_speed_check and not self.enabling,
                           held_positions=self.hold_positions,
                           moving_joints=getattr(self, "moving_joints", None),
                           moving_speed_limit=getattr(self, "moving_speed_limit", 2.0),
                           held_speed_limit=getattr(self, "held_speed_limit", 0.5))
            if self.session_enabled and not all(self.hw["enabled"]):
                raise RuntimeError("有上半身关节退出使能，结束本次控制")
        except RuntimeError as exc:
            # Save the actual triggering frame, not just the next 10 Hz UI sample.
            self.emit(fault_snapshot={"reason": str(exc), "stamp": now,
                      "phase": self.phase, "enabling": self.enabling,
                      "selected": self.selected, "hardware": self.hw})
            raise
        if link:
            with self.input_lock:
                packet, received = self.packet, self.received
            check_packet(packet, received, now, allow_unfocused=getattr(self, "allow_unfocused", False))
            if cancel and self.cancel_count is not None and packet.get("cancel", 0) != self.cancel_count:
                raise StepCancelled("用户中止本次点动")
            if (cancel and self.cancel_count is not None and not self.enabling
                    and getattr(self, 'pause_on_unfocused', False) and not packet.get('focused')):
                raise StepCancelled('控制界面失焦，暂停轨迹并保持使能')

    def pump(self, seconds=0.02, guarded=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(0.01, max(0, end-time.monotonic())))
            self.report()
            if guarded:
                self.guard()

    def wait(self, future, timeout=5, guarded=True):
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end:
                raise RuntimeError("ROS 请求超时")
            self.pump(0.01, guarded)
        if guarded:
            self.guard()
        return future.result()

    def service(self, typ, path, request, guarded=True):
        if path not in self.clients:
            self.clients[path] = self.node.create_client(typ, path)
        client = self.clients[path]
        end = time.monotonic() + 3
        while not client.service_is_ready():
            if time.monotonic() > end:
                raise RuntimeError("服务不可用：" + path)
            self.pump(0.02, guarded)
        return self.wait(client.call_async(request), 6, guarded)

    def controllers(self, guarded=True):
        result = self.service(ListControllers, "/controller_manager/list_controllers",
                              ListControllers.Request(), guarded)
        return {c.name: c.state for c in result.controller}

    def command(self, text, guarded=True):
        allowed = {f"enable({ALL_MASK})", f"disable({ALL_MASK})",
                   f"switchmode(position,{ALL_MASK})"}
        if text not in allowed:
            raise RuntimeError("拒绝非本程序生命周期指令：" + text)
        end = time.monotonic() + 3
        while not self.command_client.server_is_ready():
            if time.monotonic() > end:
                raise RuntimeError("厂家指令服务器不可用")
            self.pump(0.02, guarded)
        goal = StringCommand.Goal(); goal.command = text
        pending = self.command_client.send_goal_async(goal)
        is_enable = text == f"enable({ALL_MASK})"
        if is_enable:
            self.pending_enable_goal = pending
        handle = self.wait(pending, 4, guarded)
        if not handle.accepted:
            if is_enable:
                self.pending_enable_goal = None
            raise RuntimeError("厂家拒绝指令：" + text)
        result_future = handle.get_result_async()
        if is_enable:
            self.pending_enable_result = result_future
        result = self.wait(result_future, 8, guarded)
        if is_enable:
            self.pending_enable_goal = self.pending_enable_result = None
        self.emit(action=text, code=result.result.error_code, status=result.status)
        if result.status != 4 or result.result.error_code.lower() != "0xc000":
            raise RuntimeError(f"{text} 返回 {result.result.error_code}")

    def switch(self, active, guarded=True):
        req = SwitchController.Request()
        req.activate_controllers = [CONTROLLER] if active else []
        req.deactivate_controllers = [] if active else [CONTROLLER]
        req.strictness = 2
        req.activate_asap = False
        req.timeout.sec = 3
        if not self.service(SwitchController, "/controller_manager/switch_controller", req, guarded).ok:
            raise RuntimeError("辨认控制器切换失败")
        self.active = active

    def stationary_disabled(self):
        self.guard()
        if any(self.hw["enabled"]) or max(abs(v) for v in self.hw["velocity"]) > 0.005:
            raise RuntimeError("要求上半身 20 轴均未使能且静止")

    def check_initial_target(self):
        for i in range(20):
            actual = self.hw["position"][i]
            command = self.hw["command_position"][i]
            difference = abs(command - actual)
            if difference > 0.001:
                raise RuntimeError(
                    f"J{i+1} 初始指令与实际位置不一致，拒绝使能："
                    f"目标 {math.degrees(command):+.4f}°，实测 {math.degrees(actual):+.4f}°，"
                    f"差 {math.degrees(difference):.4f}° > {math.degrees(0.001):.4f}°")

    def prepare(self):
        deadline = time.monotonic()+10
        while self.hw is None and time.monotonic() < deadline:
            if self.exit.is_set():
                raise RuntimeError("启动已取消")
            self.pump(0.02, False)
        self.guard(False)
        controls = self.controllers(False)
        if self.args.check:
            self.phase = "只读检查通过，未使能"
            self.report(True, checked=True, controllers=controls)
            return
        self.phase = "等待操作窗口"
        deadline = time.monotonic()+30
        while True:
            self.pump(0.02, False)
            if self.exit.is_set():
                raise RuntimeError("启动已取消")
            try:
                self.guard()
                break
            except RuntimeError:
                if time.monotonic() > deadline:
                    raise
        self.stationary_disabled()
        allowed = {"fairino_hardware_state_broadcaster", "fairino_hardware_command_controller",
                   "joint_state_broadcaster"}
        if any(s == "active" and n not in allowed for n, s in controls.items()):
            raise RuntimeError("原上半身运动控制器仍活动，请先停止原控制程序")
        if CONTROLLER in controls:
            raise RuntimeError("上次辨认控制器仍存在，请先检查其退出状态")
        self.baseline = self.hw["position"][:]
        self.phase = "准备二十轴位置保持接口（尚未使能）"
        self.owned = True
        self.params_dir = tempfile.TemporaryDirectory(prefix="fairino-upper-probe-")
        path = Path(self.params_dir.name)/"controller.yaml"
        parameters = {"joints": JOINTS, "command_interfaces": ["position"],
                      "state_interfaces": ["position"], "state_publish_rate": 50.0,
                      "action_monitor_rate": 50.0,
                      # A previous session's command can outlive motor disable.
                      # Seed the new hold from measured positions, never that stale target.
                      "set_last_command_interface_value_as_state_on_activation": False,
                      "open_loop_control": False}
        path.write_text(yaml.safe_dump({CONTROLLER: {"ros__parameters": parameters}}))
        req = SetParameters.Request()
        req.parameters = [Parameter(CONTROLLER+".type", value="joint_trajectory_controller/JointTrajectoryController").to_parameter_msg(),
                          Parameter(CONTROLLER+".params_file", value=[str(path)]).to_parameter_msg()]
        result = self.service(SetParameters, "/controller_manager/set_parameters", req)
        if len(result.results) != 2 or not all(r.successful for r in result.results):
            raise RuntimeError("控制器加载参数设置失败")
        self.loaded = True  # Track attempts, including a late successful response.
        req = LoadController.Request(); req.name = CONTROLLER
        if not self.service(LoadController, "/controller_manager/load_controller", req).ok:
            raise RuntimeError("加载辨认控制器失败")
        req = ConfigureController.Request(); req.name = CONTROLLER
        if not self.service(ConfigureController, "/controller_manager/configure_controller", req).ok:
            raise RuntimeError("配置辨认控制器失败")
        self.switch(True)
        self.pump(0.3)
        self.check_initial_target()
        end = time.monotonic()+3
        while not self.trajectory_client.server_is_ready():
            if time.monotonic() > end:
                raise RuntimeError("辨认轨迹 Action 未就绪")
            self.pump()
        self.stationary_disabled()
        self.hold_positions = self.hw["command_position"][:]
        if self.args.prepare_check:
            self.phase = "二十轴接口自检通过，未使能、未发送轨迹"
            self.report(True, checked=True, controllers=self.controllers())
            return
        self.command(f"switchmode(position,{ALL_MASK})")
        self.check_initial_target()
        with self.input_lock:
            self.cancel_count = self.packet.get("cancel", 0)
        self.enable_all()
        self.cancel_count = None
        self.phase = "全部 20 轴已使能并保持；选轴后点动或输入目标角度"
        self.ready = True
        self.report(True)

    def enable_all(self):
        speed_note = "此阶段暂停速度检查" if self.feedback_speed_check else "实测超速退出已关闭"
        self.phase = f"正在使能全部 J1–J20（{speed_note}）"
        self.enable_attempted = True
        self.enabling = True
        self.enable_deadline = time.monotonic()+12
        self.report(True)
        try:
            self.command(f"enable({ALL_MASK})")
            while not all(self.hw["enabled"]):
                self.pump()
            self.session_enabled = True
        finally:
            # Also restore on timeout, estop, cancellation or communication failure.
            self.enabling = False
        self.guard()
        self.report(True)

    def stationary_enabled(self):
        self.guard()
        if not self.session_enabled or not all(self.hw["enabled"]):
            raise RuntimeError("全部 20 轴使能未确认")
        if max(abs(v) for v in self.hw["velocity"]) > 0.005:
            raise RuntimeError("关节尚未静止，停止接受点动")

    def perform_step(self, plan):
        self.stationary_enabled()
        self.selected = plan["joint"]
        self.ready = False
        self.goal = self.pending_goal = None
        if plan.get("kind") == "target":
            # Re-read the start, but never turn an absolute target into a jog.
            plan = plan_target(self.hw, self.selected, plan["target_deg"], self.hold_positions,
                               speed_deg_s=plan["speed_deg_s"])
            movement = f"移动到 {plan['target_deg']:+.3f}°"
        else:
            direction = 1 if plan["target"][self.selected-1] > plan["start"][self.selected-1] else -1
            plan = plan_step(self.hw, self.selected, direction, self.hold_positions,
                             speed_deg_s=plan["speed_deg_s"])
            movement = "单步点动中"
        self.moving_speed_limit = max(2.0, plan["speed_deg_s"])
        self.phase = f"J{self.selected} {movement} · 最高速度 {plan['speed_deg_s']:.1f}°/s"
        self.report(True)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for positions, seconds in ((plan["start"], 0.1), (plan["target"], plan["duration"]+0.1)):
            p = JointTrajectoryPoint()
            p.positions = positions; p.velocities = [0.0]*20; p.accelerations = [0.0]*20
            p.time_from_start = Duration(seconds=seconds).to_msg()
            goal.trajectory.points.append(p)
        for name in JOINTS:
            if self.path_tracking_check:
                goal.path_tolerance.append(JointTolerance(name=name, position=0.004))
            else:
                # ROS JointTolerance: -1 explicitly erases a tolerance;
                # zero/omission would inherit the controller's defaults.
                goal.path_tolerance.append(JointTolerance(
                    name=name, position=-1.0, velocity=-1.0, acceleration=-1.0))
            goal.goal_tolerance.append(JointTolerance(name=name, position=0.001))
        goal.goal_time_tolerance.sec = 1
        self.emit(step=plan, baseline=self.baseline)
        self.pending_goal = self.trajectory_client.send_goal_async(goal)
        self.goal = self.wait(self.pending_goal, 3)
        if not self.goal.accepted:
            raise RuntimeError("点动轨迹被拒绝")
        result = self.wait(self.goal.get_result_async(), plan["duration"]+3)
        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError("轨迹执行失败：" + result.result.error_string)
        actual = self.hw["position"][self.selected-1]
        if abs(actual-plan["target"][self.selected-1]) > 0.0015:
            raise RuntimeError("选中轴实际角度未到位")
        self.hold_positions = plan["target"][:]

    def cancel_step(self):
        """Cancel the trajectory and keep all axes enabled in position hold."""
        self.ready = False
        self.cancel_count = None
        if self.goal is None and self.pending_goal is not None:
            self.goal = self.wait(self.pending_goal, 3)
        if self.goal is not None and self.goal.accepted:
            result_future = self.goal.get_result_async()
            self.wait(self.goal.cancel_goal_async(), 3)
            result = self.wait(result_future, 3)
            if result.status not in (4, 5):
                raise RuntimeError("中止轨迹未确认，结束控制")
        self.pump(0.15)
        # The controller switches to a hold target after cancellation. Do not
        # accumulate feedback noise into targets of other axes between steps.
        self.hold_positions = self.hw["command_position"][:]
        self.goal = self.pending_goal = None

    def run(self):
        self.prepare()
        if self.args.check or self.args.prepare_check:
            return
        while True:
            self.pump()
            with self.input_lock:
                packet = self.packet
            request = packet.get("request") if packet.get('focused') else None
            if not request or request.get("id", 0) <= self.last_request:
                continue
            self.last_request = request["id"]
            try:
                fresh_request(request, time.monotonic())
                plan = plan_request(self.hw, request, self.hold_positions)
            except ValueError as exc:
                self.phase = str(exc)
                self.report(True)
                continue
            self.cancel_count = packet.get("cancel", 0)
            try:
                self.perform_step(plan)
                done_text = "本步完成，全部关节继续使能并保持位置"
            except StepCancelled:
                self.phase = "正在中止轨迹，全部关节保持使能"
                self.cancel_step()
                done_text = "本步已中止，全部关节继续使能并保持位置"
            # Other failures propagate straight to cleanup: disable all axes.
            self.cancel_count = None
            self.goal = self.pending_goal = None
            self.selected = None
            self.stationary_enabled()
            self.phase = done_text
            with self.input_lock:
                queued = self.packet.get("request")
                if queued:
                    self.last_request = max(self.last_request, queued.get("id", 0))
            self.ready = True
            self.report(True)

    def settle_enable_request(self):
        """Account for an enable response that arrives after a guard aborted it."""
        if self.pending_enable_goal is None:
            return
        handle = self.wait(self.pending_enable_goal, 4, False)
        if handle.accepted:
            if self.pending_enable_result is None:
                self.pending_enable_result = handle.get_result_async()
            self.wait(self.pending_enable_result, 8, False)
        self.pending_enable_goal = self.pending_enable_result = None

    def confirm_disabled(self):
        deadline = time.monotonic()+3
        stable_since = None
        while time.monotonic() < deadline:
            self.pump(0.02, False)
            now = time.monotonic()
            if (self.hw is not None and now-self.hw_time < 0.4
                    and not any(self.hw["enabled"])):
                if stable_since is None:
                    stable_since = now
                if now-stable_since >= 0.3:
                    return True
            else:
                stable_since = None
        return False

    def cleanup(self):
        self.ready = False
        self.enabling = False
        self.session_enabled = False
        self.cancel_count = None
        if not self.owned:
            return True
        self.phase = "正在结束控制并下使能全部 J1–J20"
        self.report(True)
        ok = True
        # Cancellation must never prevent the disable attempt.
        try:
            if self.goal is None and self.pending_goal is not None and self.pending_goal.done():
                self.goal = self.pending_goal.result()
            if self.goal is not None and self.goal.accepted:
                self.goal.cancel_goal_async()
        except Exception as exc:
            self.emit(cleanup_warning="取消轨迹失败，仍继续下使能："+str(exc))
        if self.enable_attempted:
            pending_enable = self.pending_enable_goal is not None
            first_disable_ok = True
            try:
                self.command(f"disable({ALL_MASK})", False)
            except Exception as exc:
                first_disable_ok = False
                self.emit(cleanup_warning="首次下使能请求失败："+str(exc))
            enable_settled = True
            if pending_enable:
                try:
                    self.settle_enable_request()
                except Exception as exc:
                    enable_settled = False
                    self.emit(error="使能请求尚未结束，不能确认停止："+str(exc))
                # A late enable must not arrive AFTER the last disable command.
                try:
                    self.command(f"disable({ALL_MASK})", False)
                    first_disable_ok = True
                except Exception as exc:
                    first_disable_ok = False
                    self.emit(error="再次下使能失败："+str(exc))
            disabled = self.confirm_disabled() and enable_settled and first_disable_ok
        else:
            disabled = (self.hw is not None and time.monotonic()-self.hw_time < 0.4
                        and not any(self.hw["enabled"]))
        if disabled:
            self.enable_attempted = False
        else:
            ok = False
            self.emit(error="未确认全部下使能，请现场检查并使用物理急停")
        if disabled and self.loaded:
            try:
                controls = self.controllers(False)
                if controls.get(CONTROLLER) == "active":
                    self.switch(False, False)
                if CONTROLLER in controls:
                    req = UnloadController.Request(); req.name = CONTROLLER
                    if not self.service(UnloadController, "/controller_manager/unload_controller", req, False).ok:
                        raise RuntimeError("卸载辨认控制器失败")
            except Exception as exc:
                ok = False
                self.emit(error=str(exc))
        if self.params_dir:
            self.params_dir.cleanup()
        self.phase = "已退出，确认上半身全部未使能" if disabled else "未确认全部下使能，请现场检查"
        self.report(True, finished=True, disabled_confirmed=disabled)
        return ok and disabled


class JointProbe(Probe):
    """Manual joint adjustment, with the user-requested soft trips removed."""
    feedback_speed_check = False
    path_tracking_check = False
    allow_unfocused = True
    pause_on_unfocused = True


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--prepare-check", action="store_true")
    args = parser.parse_args()
    if not args.check:
        lock = open("/home/fairino/.fairino_upper_identify.lock", "a")
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    rclpy.init()
    probe = JointProbe(args)
    signal.signal(signal.SIGTERM, lambda *_: probe.exit.set())
    signal.signal(signal.SIGINT, lambda *_: probe.exit.set())
    code = 0
    try:
        probe.run()
    except Exception as exc:
        code = 1
        probe.emit(error=str(exc))
    finally:
        if not probe.cleanup():
            code = 1
        deadline = time.monotonic()+1
        while probe.output.unfinished_tasks and time.monotonic() < deadline:
            time.sleep(0.01)
        probe.node.destroy_node(); rclpy.shutdown()
    return code


if __name__ == "__main__":
    import rclpy
    import yaml
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.duration import Duration
    from hardware_state_broadcaster_msgs.msg import HardwareState
    from fairino_hardware_command_msgs.action import StringCommand
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTolerance
    from trajectory_msgs.msg import JointTrajectoryPoint
    from controller_manager_msgs.srv import (ListControllers, LoadController, ConfigureController,
                                             SwitchController, UnloadController)
    from rcl_interfaces.srv import SetParameters
    sys.exit(main())
