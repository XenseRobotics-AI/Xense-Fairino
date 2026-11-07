#!/usr/bin/env python3
"""Shared mapping-drive and navigation chassis executor, sent over SSH.

Legacy ROS names and the ownership lock are compatibility interfaces.
"""
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
import uuid
from pathlib import Path

try:
    from .motion_feedback import TOPIC as MOTION_TOPIC, STATUS_NAME, motion_values, wire_values
    from .control_core import (JOINTS, STEER, DRIVE, check_feedback, check_input,
                               key_twist, neutral_duration, normalise_hardware,
                               FEEDBACK_TIMEOUT, check_speeds, check_speed_envelope,
                               DEFAULT_LINEAR, DEFAULT_ANGULAR, chassis_parameters,
                               limit_planar_twist, flatten_parameters)
except ImportError:  # The SSH launcher loads control_core as an in-memory module.
    from motion_feedback import TOPIC as MOTION_TOPIC, STATUS_NAME, motion_values, wire_values
    from control_core import (JOINTS, STEER, DRIVE, check_feedback, check_input,
                              key_twist, neutral_duration, normalise_hardware,
                              FEEDBACK_TIMEOUT, check_speeds, check_speed_envelope,
                              DEFAULT_LINEAR, DEFAULT_ANGULAR, chassis_parameters,
                              limit_planar_twist, flatten_parameters)

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
        self.nav_command = (0., 0., 0.)
        self.nav_received = 0.
        self.nav_graph_checked = 0.
        self.motion_pub = None
        self.motion_session = uuid.uuid4().hex
        self.motion_sequence = 0
        self.motion_published = 0.
        self.subs = [
            self.node.create_subscription(HardwareState, "/hardware_states", self.on_hw, 1),
            self.node.create_subscription(Int32, "/upper_estop_error_code", self.on_upper, 1),
        ]
        if getattr(args, 'navigation', False):
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.motion_pub = self.node.create_publisher(DiagnosticArray, MOTION_TOPIC, qos)
            self.subs.append(self.node.create_subscription(
                Twist, '/fairino_navigation/cmd_vel', self.on_navigation, 1))
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

    def on_navigation(self, msg):
        try:
            self.nav_command = limit_planar_twist(
                [msg.linear.x, msg.linear.y, msg.linear.z,
                 msg.angular.x, msg.angular.y, msg.angular.z])
            self.nav_received = time.monotonic()
        except ValueError as exc:
            self.input_error = str(exc)

    def navigation_command(self):
        now = time.monotonic()
        if now-self.nav_graph_checked > .5:
            self.nav_graph_checked = now
            sources = self.node.get_publishers_info_by_topic('/fairino_navigation/cmd_vel')
            if sources and (len(sources) != 1 or sources[0].node_name != 'velocity_gate'
                            or sources[0].node_namespace != '/fairino_navigation'):
                raise RuntimeError('导航速度来源不唯一或不匹配')
        if now-self.nav_received > .3:
            return (0., 0., 0.)
        return self.nav_command

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

    def link_status(self, now):
        with self.input_lock:
            packet, received = self.packet or {}, self.received
        ack = packet.get('ack')
        return {
            'input_age': now-received if received else None,
            'ack_age': now-ack if isinstance(ack, (int, float)) and math.isfinite(ack) else None,
            'client_status_age': packet.get('client_status_age'),
            'client_send_gap': packet.get('client_send_gap'),
            'process_nice': os.getpriority(os.PRIO_PROCESS, 0),
        }

    def publish_motion_state(self, force=False):
        # A local, depth-one BEST_EFFORT publication cannot wait for a slow reader.
        if getattr(self, 'motion_pub', None) is None:
            return
        now = time.monotonic()
        if not force and now - self.motion_published < .05:
            return
        try:
            healthy = False
            if self.ready:
                try:
                    self.guard()
                    with self.input_lock:
                        healthy = bool((self.packet or {}).get('focused')) and not self.stop.is_set()
                except RuntimeError:
                    healthy = False
            message, values = motion_values(
                self.hw, self.commanded, healthy, max(0., now-self.hw_time, now-self.upper_time))
            report = DiagnosticArray()
            report.header.stamp = self.node.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name, status.hardware_id = STATUS_NAME, self.motion_session
            # ROS byte fields use one-byte values in Python (not uint8 ints).
            status.level = DiagnosticStatus.OK if values['ready'] else DiagnosticStatus.ERROR
            status.message = message
            self.motion_sequence += 1
            status.values = [KeyValue(key=key, value=value)
                             for key, value in wire_values(values, self.motion_sequence).items()]
            report.status = [status]
            self.motion_pub.publish(report)
            self.motion_published = now
        except Exception as exc:
            # Losing this feedback must not silently leave an enabled control loop.
            self.input_error = '底盘运动反馈发布失败：' + str(exc)
            self.stop.set()

    def status(self, force=False, **extra):
        self.publish_motion_state(force)
        now = time.monotonic()
        if force or now - self.last_status >= 0.1:
            self.emit(phase=self.phase, ready=self.ready, stamp=now, hardware=self.hw,
                      upper_error=self.upper, hw_age=now-self.hw_time,
                      upper_age=now-self.upper_time, command=self.commanded,
                      link=self.link_status(now),
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
                    raise RuntimeError("调速前请松开全部运动键")
                if getattr(self.args, 'navigation', False):
                    if keys:
                        raise RuntimeError('导航接管期间不接受键盘运动输入')
                    if speeds != (DEFAULT_LINEAR, DEFAULT_ANGULAR):
                        raise RuntimeError('导航模式固定使用当前底盘默认速度')
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
            self.param_dir = tempfile.TemporaryDirectory(prefix="fairino-slam-chassis-")
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
            if self.input_error:
                raise RuntimeError(self.input_error)
            if self.stop.is_set():
                raise RuntimeError("启动已取消或 SSH 输入已断开")
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
            if self.input_error:
                raise RuntimeError(self.input_error)
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
            raise RuntimeError('尚未安装统一底盘配置；先运行 fairino-slam deploy control --execute 并按提示重启')
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
        self.phase = "回正完成，接入导航速度" if getattr(self.args, "navigation", False) else "回正完成，接入键盘控制"
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
        self.phase = "导航执行就绪：等待明确的导航目标" if getattr(self.args, "navigation", False) else "就绪：按住移动，松开停止"
        self.status(True)

    def run(self):
        self.startup()
        if self.args.check or self.args.prepare_check:
            return
        while True:
            self.pump(0.02)
            keys = self.guard()
            if getattr(self.args, 'navigation', False):
                with self.input_lock:
                    focused = bool((self.packet or {}).get('focused'))
                self.commanded = self.navigation_command() if focused else (0., 0., 0.)
            else:
                self.commanded = key_twist(keys, self.args.linear, self.args.angular)
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = self.commanded
            self.pub.publish(msg)

    def cleanup(self):
        was_ready = self.ready
        self.ready = False
        self.publish_motion_state(force=True)
        if not self.owned:
            return True
        self.phase = "停止并下使能"
        self.zero()
        # Do not wait for a cancellation result before attempting disable.
        if self.trajectory is not None and self.trajectory.accepted:
            self.trajectory.cancel_goal_async()
        disable_ok = (not self.enable_attempted and self.hw is not None
                      and time.monotonic()-self.hw_time < 0.4 and not any(self.hw["enabled"]))
        if self.enable_attempted:
            try:
                self.command("disable()", guarded=False)
                self.pump(0.3, False)
                disable_ok = (self.hw is not None and time.monotonic()-self.hw_time < 0.4
                              and not any(self.hw["enabled"]))
                if not disable_ok:
                    self.emit(error="下使能未获反馈确认，请使用物理急停")
            except Exception as exc:
                self.emit(error="下使能失败，请使用物理急停：" + str(exc))
        cleanup_ok = True
        # A healthy, already-running chassis can keep measuring wheel odometry
        # after motor disable. Do not reset the odom epoch merely because the
        # desktop controller exits. Startup/homing failures still tear down.
        preserve_odometry = was_ready and disable_ok
        if preserve_odometry:
            try:
                now = time.monotonic()
                check_feedback(self.hw, now-self.hw_time, self.upper, now-self.upper_time)
            except RuntimeError:
                preserve_odometry = False
        controls = None
        try:
            controls = self.controllers(False)
            preserve_odometry = preserve_odometry and controls.get(CHASSIS) == "active"
        except Exception as exc:
            preserve_odometry = False
            cleanup_ok = False
            self.emit(error="读取退出时控制器状态失败：" + str(exc))
        if not preserve_odometry:
            try:
                self.odometry('pause', guarded=False)
            except Exception as exc:
                cleanup_ok = False
                self.emit(error='里程计暂停未确认：' + str(exc))
        try:
            controls = controls if controls is not None else self.controllers(False)
            deactivate = [n for n in set(self.loaded + [CHASSIS])
                          if controls.get(n) == "active" and not (preserve_odometry and n == CHASSIS)]
            self.switch([], deactivate, False)
            if disable_ok:
                for name in reversed(self.loaded):
                    if name == CHASSIS or name not in controls:
                        continue
                    request = UnloadController.Request(); request.name = name
                    if not self.service(UnloadController, "/controller_manager/unload_controller", request, False).ok:
                        cleanup_ok = False
                        self.emit(error="卸载控制器失败：" + name)
        except Exception as exc:
            cleanup_ok = False
            self.emit(error="控制器清理失败：" + str(exc))
        self.phase = "已停止并确认下使能" if disable_ok else "停止异常，请检查物理急停"
        self.status(True, finished=True, disabled_confirmed=disable_ok,
                    odometry_preserved=preserve_odometry)
        if self.param_dir is not None:
            self.param_dir.cleanup()
        return disable_ok and cleanup_ok


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--navigation", action="store_true", help="显式接入 Nav2；仍使用底盘独占锁、反馈和心跳检查")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--prepare-check", action="store_true")
    parser.add_argument("--linear", type=float, default=DEFAULT_LINEAR)
    parser.add_argument("--angular", type=float, default=DEFAULT_ANGULAR)
    args = parser.parse_args()
    try:
        check_speeds(args.linear, args.angular)
    except RuntimeError as exc:
        parser.error(str(exc))
    # Mapping and navigation share the installed chassis ownership lock.
    if not args.check:
        lock = open("/home/fairino/.fairino_keyboard.lock", "a")
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    load_ros(navigation=args.navigation)
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


def load_ros(navigation=False):
    global rclpy, yaml, ActionClient, Duration, Parameter, Twist, Int32
    global DiagnosticArray, DiagnosticStatus, KeyValue, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    global HardwareState, StringCommand, ListControllers, LoadController
    global ConfigureController, SwitchController, UnloadController, SetParameters
    global FollowJointTrajectory, JointTolerance, JointTrajectoryPoint
    import rclpy
    import yaml
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int32
    if navigation:
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from hardware_state_broadcaster_msgs.msg import HardwareState
    from fairino_hardware_command_msgs.action import StringCommand
    from controller_manager_msgs.srv import (ListControllers, LoadController, ConfigureController,
                                             SwitchController, UnloadController)
    from rcl_interfaces.srv import SetParameters
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTolerance
    from trajectory_msgs.msg import JointTrajectoryPoint


if __name__ == "__main__":
    sys.exit(main())
