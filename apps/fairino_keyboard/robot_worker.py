#!/usr/bin/env python3
"""Runs through SSH on 192.168.58.3. ROS is imported only in main()."""
import argparse
import fcntl
import json
import math
import os
import queue
import signal
import sys
import tempfile
import threading
import time
from pathlib import Path

from control_core import (JOINTS, STEER, DRIVE, check_feedback, check_input,
                          input_twist, neutral_duration,
                          normalise_hardware, FEEDBACK_TIMEOUT, check_speeds,
                          check_speed_envelope, DEFAULT_LINEAR, DEFAULT_ANGULAR,
                          MAX_LINEAR, MAX_ANGULAR, DRIVE_COMMAND_LIMIT, chassis_parameters)

ANGLE = "fairino_keyboard_angle"
CHASSIS = "eight_dof_chassis_controller"
ODOM = "/eight_dof_chassis_controller/odom"
TOPIC = "/fairino_keyboard/cmd_vel"
PREFIX = "@KEYCTRL "
CONFIG = "/opt/fairino_AMR_project_B/reference_chassis_runtime/install/fairino_hardware/share/fairino_hardware/config/ros2_controllers.yaml"


class Worker:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("fairino_keyboard_guard")
        self.hw = None
        self.hw_time = self.upper_time = 0.0
        self.upper = None
        self.packet = None
        self.received = 0.0
        self.input_lock = threading.Lock()
        self.stop = threading.Event()
        self.output = queue.Queue(maxsize=40)
        self.output_thread = threading.Thread(target=self.write_output, daemon=True)
        self.output_thread.start()
        self.input_error = ""
        self.phase = "连接反馈"
        self.owned = False
        self.enable_attempted = False
        self.loaded = []
        self.param_dir = None
        self.trajectory = None
        self.last_status = 0.0
        self.commanded = (0.0, 0.0, 0.0)
        self.pub = None
        self.homing = False
        self.ready = False
        self.subs = [
            self.node.create_subscription(HardwareState, "/hardware_states", self.on_hw, 1),
            self.node.create_subscription(Int32, "/upper_estop_error_code", self.on_upper, 1),
        ]
        self.action = ActionClient(self.node, StringCommand,
                                   "/fairino_hardware_command_controller/command")
        self.follow = ActionClient(self.node, FollowJointTrajectory,
                                   f"/{ANGLE}/follow_joint_trajectory")
        self.clients = {}
        threading.Thread(target=self.read_input, daemon=True).start()

    def emit(self, **values):
        # A blocked SSH output pipe must never block the motor watchdog.
        line = PREFIX + json.dumps(values, ensure_ascii=False, allow_nan=False)
        try:
            self.output.put_nowait(line)
        except queue.Full:
            self.stop.set()

    def write_output(self):
        while True:
            line = self.output.get()
            try:
                print(line, flush=True)
            except (OSError, ValueError):
                self.stop.set()
            finally:
                self.output.task_done()

    def on_hw(self, msg):
        try:
            self.hw = normalise_hardware({k: getattr(msg, k) for k in msg.get_fields_and_field_types()})
            self.hw_time = time.monotonic()
        except (ValueError, KeyError, TypeError) as exc:
            self.input_error = str(exc)
            self.hw = None

    def on_upper(self, msg):
        self.upper, self.upper_time = msg.data, time.monotonic()

    def read_input(self):
        try:
            for line in sys.stdin:
                packet = json.loads(line)
                if not isinstance(packet, dict):
                    raise ValueError("无效的输入包")
                with self.input_lock:
                    self.packet, self.received = packet, time.monotonic()
                if packet.get("stop"):
                    self.stop.set()
        except Exception as exc:
            self.input_error = str(exc)
        finally:
            self.stop.set()

    def status(self, force=False, **extra):
        now = time.monotonic()
        if force or now - self.last_status >= 0.1:
            self.emit(phase=self.phase, ready=self.ready, stamp=now, hardware=self.hw,
                      upper_error=self.upper, hw_age=now-self.hw_time,
                      upper_age=now-self.upper_time, command=self.commanded,
                      linear_speed=self.args.linear, angular_speed=self.args.angular, **extra)
            self.last_status = now

    def guard(self, input_required=True):
        now = time.monotonic()
        if self.input_error:
            raise RuntimeError(self.input_error)
        check_feedback(self.hw, now-self.hw_time, self.upper, now-self.upper_time)
        if input_required:
            if self.stop.is_set():
                raise RuntimeError("用户停止或 SSH 输入已断开")
            with self.input_lock:
                keys = check_input(self.packet, self.received, now)
                speeds = check_speeds(self.packet.get("linear", self.args.linear),
                                      self.packet.get("angular", self.args.angular))
                if keys and speeds != (self.args.linear, self.args.angular):
                    raise RuntimeError("调速前请松开全部运动键并将摇杆回中")
                self.args.linear, self.args.angular = speeds
        else:
            keys = []
        if self.homing:
            if any(self.hw["enabled"][i] for i in DRIVE):
                raise RuntimeError("回正期间行走电机意外使能")
            if any(abs(self.hw["velocity"][i]) > 0.05 for i in DRIVE):
                raise RuntimeError("回正期间检测到行走轮运动")
        if self.ready and not all(self.hw["enabled"]):
            raise RuntimeError("有电机退出使能")
        return keys

    def pump(self, seconds=0.02, guarded=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(0.01, max(0, end-time.monotonic())))
            self.status()
            if guarded:
                self.guard()

    def future(self, future, timeout=5.0, guarded=True):
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= end:
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
        return self.future(client.call_async(request), 6, guarded)

    def controllers(self, guarded=True):
        result = self.service(ListControllers, "/controller_manager/list_controllers",
                              ListControllers.Request(), guarded)
        return {c.name: c.state for c in result.controller}

    def set_params(self, node, values, guarded=True):
        request = SetParameters.Request()
        request.parameters = [Parameter(k, value=v).to_parameter_msg() for k, v in values.items()]
        result = self.service(SetParameters, f"/{node}/set_parameters", request, guarded)
        failures = [r.reason for r in result.results if not r.successful]
        if len(result.results) != len(values) or failures:
            raise RuntimeError("参数设置失败：" + str(failures))

    def odometry(self, action, guarded=True):
        from std_srvs.srv import Trigger
        result = self.service(Trigger, '/fairino_odometry/' + action, Trigger.Request(), guarded)
        if not result.success:
            raise RuntimeError('里程计联动失败：' + result.message)

    def verify_chassis(self, params):
        from control_core import flatten_parameters
        from rcl_interfaces.srv import GetParameters
        expected = flatten_parameters(params)
        request = GetParameters.Request()
        request.names = list(expected)
        result = self.service(GetParameters, '/' + CHASSIS + '/get_parameters', request)
        fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value',
                  5: 'byte_array_value', 6: 'bool_array_value', 7: 'integer_array_value',
                  8: 'double_array_value', 9: 'string_array_value'}
        actual = [getattr(v, fields[v.type], None) if v.type in fields else None for v in result.values]
        actual = [list(v) if not isinstance(v, (int, float, str, bool, type(None))) else v for v in actual]
        if len(actual) != len(expected) or any(a != e for a, e in zip(actual, expected.values())):
            raise RuntimeError('厂家底盘实例参数与当前配置不同；完成一次配置迁移并重新启动机器人后再控制')

    def load(self, name, typ, params):
        # Generated controller parameters can be read-only after on_init().
        # Use the installed spawner's params_file mechanism BEFORE loading.
        if self.param_dir is None:
            self.param_dir = tempfile.TemporaryDirectory(prefix="fairino-keyboard-")
        param_file = Path(self.param_dir.name) / (name + ".yaml")
        param_file.write_text(yaml.safe_dump({name: {"ros__parameters": params}}))
        self.set_params("controller_manager", {name + ".type": typ,
                                               name + ".params_file": [str(param_file)]})
        request = LoadController.Request(); request.name = name
        self.loaded.append(name)
        if not self.service(LoadController, "/controller_manager/load_controller", request).ok:
            raise RuntimeError("加载控制器失败：" + name)
        request = ConfigureController.Request(); request.name = name
        if not self.service(ConfigureController, "/controller_manager/configure_controller", request).ok:
            raise RuntimeError("配置控制器失败：" + name)

    def switch(self, activate, deactivate, guarded=True):
        if not activate and not deactivate:
            return
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = 2
        request.activate_asap = False
        request.timeout.sec = 3
        if not self.service(SwitchController, "/controller_manager/switch_controller", request, guarded).ok:
            raise RuntimeError("控制器切换失败")

    def command(self, text, guarded=True):
        end = time.monotonic() + 3
        while not self.action.server_is_ready():
            if time.monotonic() > end:
                raise RuntimeError("厂家指令服务器不可用")
            self.pump(0.02, guarded)
        goal = StringCommand.Goal(); goal.command = text
        handle = self.future(self.action.send_goal_async(goal), 4, guarded)
        if not handle.accepted:
            raise RuntimeError("指令被拒绝：" + text)
        result = self.future(handle.get_result_async(), 8, guarded)
        self.emit(action=text, code=result.result.error_code, action_status=result.status)
        if result.status != 4 or result.result.error_code.lower() != "0xc000":
            raise RuntimeError(f"{text} 失败：{result.result.error_code}")

    def zero(self):
        self.commanded = (0.0, 0.0, 0.0)
        if self.pub is not None:
            self.pub.publish(Twist())

    def wait_condition(self, condition, timeout, explanation):
        end = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > end:
                raise RuntimeError(explanation)
            self.zero()
            self.pump(0.02)

    def disabled_stationary(self):
        self.guard()
        if any(self.hw["enabled"]) or max(abs(v) for v in self.hw["velocity"]) > 0.03:
            raise RuntimeError("启动要求底盘全部下使能且静止；拒绝接管正在工作的底盘")

    def check_private_topic(self):
        subscribers = self.node.get_subscriptions_info_by_topic(TOPIC)
        if not any(s.node_name == CHASSIS for s in subscribers):
            raise RuntimeError("独立速度话题未接入，拒绝使能")
        if any(s.node_name == CHASSIS for s in self.node.get_subscriptions_info_by_topic("/cmd_vel")):
            raise RuntimeError("仍订阅网页速度话题，拒绝使能")

    def wait_initial_feedback(self, timeout=15.0):
        """Allow DDS discovery before ownership; retain the live 0.4 s watchdog."""
        deadline = time.monotonic() + timeout
        while True:
            now = time.monotonic()
            if self.stop.is_set():
                raise RuntimeError("启动已取消或 SSH 输入已断开")
            if self.input_error:
                raise RuntimeError(self.input_error)
            missing = []
            if self.hw is None or now - self.hw_time > FEEDBACK_TIMEOUT:
                missing.append("底盘状态")
            if self.upper is None or now - self.upper_time > FEEDBACK_TIMEOUT:
                missing.append("上半身急停状态")
            if not missing:
                self.phase = "底盘及上半身反馈已连接"
                self.status(True)
                self.guard(False)
                return
            self.phase = "SSH 已连接，等待" + "、".join(missing)
            if now >= deadline:
                raise RuntimeError(self.phase + f"（{timeout:g} 秒内未收到新鲜反馈）")
            self.pump(0.02, False)

    def startup(self):
        self.wait_initial_feedback()
        config = yaml.safe_load(Path(CONFIG).read_text())
        if config["angle_controller"]["ros__parameters"]["joints"] != ["j1", "j3", "j5", "j7"]:
            raise RuntimeError("转向关节配置发生变化")
        check_speed_envelope(config["eight_dof_chassis_controller"]["ros__parameters"])
        controls = self.controllers(False)
        if self.args.check:
            self.phase = "只读检查通过（未使能、未回正）"
            self.status(True, checked=True, controllers=controls)
            return
        self.phase = "等待操作界面就绪并松开全部按键"
        deadline = time.monotonic() + 30
        while True:
            self.pump(0.02, False)
            if self.stop.is_set():
                raise RuntimeError("启动已取消")
            try:
                if not self.guard():
                    break
            except RuntimeError:
                pass
            if time.monotonic() > deadline:
                raise RuntimeError("等待操作窗口超时")
        self.disabled_stationary()
        allowed = {"fairino_hardware_state_broadcaster", "fairino_hardware_command_controller",
                   "joint_state_broadcaster", "angle_controller", "velocity_controller",
                   "eight_dof_chassis_controller", ANGLE, CHASSIS}
        if any(s == "active" and n not in allowed for n, s in controls.items()):
            raise RuntimeError("存在未知的活动控制器，拒绝接管")
        if ANGLE in controls or 'fairino_keyboard_chassis' in controls:
            raise RuntimeError("上次键控控制器尚未卸载，需要先检查上次退出状态")
        chassis_params = config[CHASSIS]["ros__parameters"]
        if chassis_params.get('cmd_vel_topic') != TOPIC or chassis_params.get('odom_topic') != ODOM:
            raise RuntimeError('尚未安装统一底盘配置；先运行 tools/install_control_stack.py --execute 并按提示重启')
        if chassis_params != chassis_parameters(chassis_params):
            raise RuntimeError('底盘低速保护参数与共享配置不符；请重新安装统一底盘配置')
        if CHASSIS in controls:
            self.verify_chassis(chassis_params)
        self.odometry('pause')
        self.phase = "接入厂家底盘实例，临时准备转向回正"
        self.owned = True
        self.switch([], [n for n in ("angle_controller", "velocity_controller", "eight_dof_chassis_controller")
                         if controls.get(n) == "active"])
        self.command("switchmode(position,85)")
        self.command("switchmode(velocity,170)")
        angle_params = config["angle_controller"]["ros__parameters"]
        self.load(ANGLE, "joint_trajectory_controller/JointTrajectoryController", angle_params)
        if CHASSIS not in controls:
            self.load(CHASSIS, "eight_dof_chassis_controller/EightDofChassisController", chassis_params)
        self.pub = self.node.create_publisher(Twist, TOPIC, 1)
        self.switch([ANGLE], [])
        self.pump(0.2)
        current = [self.hw["position"][i] for i in STEER]
        if any(abs(self.hw["command_position"][i] - self.hw["position"][i]) > 0.04 for i in STEER):
            raise RuntimeError("转向控制器初始目标与实测位置不一致，禁止使能")
        if self.args.prepare_check:
            self.switch([CHASSIS], [ANGLE])
            self.zero(); self.pump(0.5)
            self.check_private_topic()
            self.disabled_stationary()
            self.phase = "控制器接口自检通过（未使能、未回正）"
            self.status(True, checked=True, controllers=self.controllers())
            return
        self.homing = True
        self.phase = "四轮缓慢回中立位（行走电机关闭）"
        self.enable_attempted = True
        self.command("enable(85)")
        self.wait_condition(lambda: all(self.hw["enabled"][i] for i in STEER), 2, "转向电机使能未确认")
        end = time.monotonic() + 3
        while not self.follow.server_is_ready():
            if time.monotonic() > end:
                raise RuntimeError("转向轨迹服务不可用")
            self.pump()
        duration = neutral_duration(current)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [JOINTS[i] for i in STEER]
        for positions, seconds in ((current, 0.2), ([0.0]*4, duration + 0.2)):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0]*4
            point.accelerations = [0.0]*4
            point.time_from_start = Duration(seconds=seconds).to_msg()
            goal.trajectory.points.append(point)
        for joint in goal.trajectory.joint_names:
            goal.path_tolerance.append(JointTolerance(name=joint, position=0.12))
            goal.goal_tolerance.append(JointTolerance(name=joint, position=0.02))
        goal.goal_time_tolerance.sec = 2
        self.trajectory = self.future(self.follow.send_goal_async(goal), 3)
        if not self.trajectory.accepted:
            raise RuntimeError("回正轨迹被拒绝")
        result = self.future(self.trajectory.get_result_async(), duration + 4)
        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError("回正失败：" + str(result.result.error_string))
        if any(abs(self.hw["position"][i]) > 0.02 for i in STEER):
            raise RuntimeError("回正未达到实测角度容差")
        self.command("disable(85)")
        self.wait_condition(lambda: not any(self.hw["enabled"]), 2, "转向下使能未确认")
        self.homing = False
        self.disabled_stationary()
        self.phase = "回正完成，接入键盘控制"
        self.switch([CHASSIS], [ANGLE])
        self.zero(); self.pump(0.4)
        self.check_private_topic()
        if any(abs(self.hw["command_velocity"][i]) > 0.001 for i in DRIVE):
            raise RuntimeError("行走初始指令不是零")
        if any(abs(self.hw["command_position"][i] - self.hw["position"][i]) > 0.04 for i in STEER):
            raise RuntimeError("底盘控制器接管时转向目标不一致")
        self.command("enable()")
        self.wait_condition(lambda: all(self.hw["enabled"]), 2, "八个电机使能未确认")
        self.phase = "请松开全部按键"
        while self.guard():
            self.zero(); self.pump(0.02)
        self.ready = True
        self.odometry('resume')
        self.phase = "就绪：按住移动，松开停止"
        self.status(True)

    def run(self):
        self.startup()
        if self.args.check or self.args.prepare_check:
            return
        while True:
            self.pump(0.02)
            keys = self.guard()
            self.commanded = input_twist(keys, self.args.linear, self.args.angular)
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = self.commanded
            self.pub.publish(msg)

    def cleanup(self):
        if not self.owned:
            return True
        reporting_ok = True

        def report_cleanup(**values):
            # Even a broken diagnostic channel must not interrupt motor cleanup.
            nonlocal reporting_ok
            try:
                self.emit(**values)
            except Exception:
                reporting_ok = False

        self.ready = False
        self.phase = "停止并下使能"
        try:
            self.zero()
        except Exception as exc:
            report_cleanup(cleanup_warning='发送零速度失败，仍继续下使能：' + str(exc))
        # Do not wait for a cancellation result before attempting disable.
        try:
            if self.trajectory is not None and self.trajectory.accepted:
                self.trajectory.cancel_goal_async()
        except Exception as exc:
            report_cleanup(cleanup_warning='取消回正轨迹失败，仍继续下使能：' + str(exc))
        disable_ok = (not self.enable_attempted and self.hw is not None
                      and time.monotonic()-self.hw_time < 0.4 and not any(self.hw["enabled"]))
        if self.enable_attempted:
            try:
                self.command("disable()", guarded=False)
                self.pump(0.3, False)
                disable_ok = (self.hw is not None and time.monotonic()-self.hw_time < 0.4
                              and not any(self.hw["enabled"]))
                if not disable_ok:
                    report_cleanup(error="下使能未获反馈确认，请使用物理急停")
            except Exception as exc:
                report_cleanup(error="下使能失败，请使用物理急停：" + str(exc))
        cleanup_ok = True
        try:
            self.odometry('pause', guarded=False)
        except Exception as exc:
            cleanup_ok = False
            report_cleanup(error='里程计暂停未确认：' + str(exc))
        try:
            controls = self.controllers(False)
            self.switch([], [n for n in set(self.loaded + [CHASSIS]) if controls.get(n) == "active"], False)
            if disable_ok:
                for name in reversed(self.loaded):
                    if name == CHASSIS or name not in controls:
                        continue
                    request = UnloadController.Request(); request.name = name
                    if not self.service(UnloadController, "/controller_manager/unload_controller", request, False).ok:
                        cleanup_ok = False
                        report_cleanup(error="卸载控制器失败：" + name)
        except Exception as exc:
            cleanup_ok = False
            report_cleanup(error="控制器清理失败：" + str(exc))
        if self.param_dir is not None:
            try:
                self.param_dir.cleanup()
            except Exception as exc:
                cleanup_ok = False
                report_cleanup(error='临时控制配置清理失败：' + str(exc))
        self.phase = "已停止并确认下使能" if disable_ok else "停止异常，请检查物理急停"
        try:
            self.status(True, finished=True, disabled_confirmed=disable_ok,
                        cleanup_confirmed=cleanup_ok and reporting_ok)
        except Exception:
            reporting_ok = False
        return disable_ok and cleanup_ok and reporting_ok


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--prepare-check", action="store_true")
    parser.add_argument("--linear", type=float, default=DEFAULT_LINEAR)
    parser.add_argument("--angular", type=float, default=DEFAULT_ANGULAR)
    args = parser.parse_args()
    try:
        check_speeds(args.linear, args.angular)
    except RuntimeError as exc:
        parser.error(str(exc))
    # One live keyboard owner; kernel releases the lock even after a crash.
    if not args.check:
        lock = open("/home/fairino/.fairino_keyboard.lock", "a")
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    rclpy.init()
    worker = Worker(args)
    signal.signal(signal.SIGTERM, lambda *_: worker.stop.set())
    signal.signal(signal.SIGINT, lambda *_: worker.stop.set())
    code = 0
    try:
        worker.run()
    except Exception as exc:
        code = 1
        worker.emit(error=str(exc))
    finally:
        if not worker.cleanup():
            code = 1
        deadline = time.monotonic() + 1.0
        while worker.output.unfinished_tasks and time.monotonic() < deadline:
            time.sleep(0.01)
        worker.node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    import rclpy
    import yaml
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int32
    from hardware_state_broadcaster_msgs.msg import HardwareState
    from fairino_hardware_command_msgs.action import StringCommand
    from controller_manager_msgs.srv import (ListControllers, LoadController, ConfigureController,
                                             SwitchController, UnloadController)
    from rcl_interfaces.srv import SetParameters
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTolerance
    from trajectory_msgs.msg import JointTrajectoryPoint
    sys.exit(main())
