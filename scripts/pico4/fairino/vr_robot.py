"""One remote 20-axis owner; complete verified joint-position targets."""
import argparse
import fcntl
import json
import math
import signal
import sys
import threading
import time
import robot_probe as rp
from tcp_robot import TCPProbe, load_ros
from tcp_jog import LEASE
from vr_core import validate, direct_positions, KIND
from vr_lift import LiftPlanner, lift_axis, DEADBAND
from vr_timing import Timing, Deadline, RateMeter, add_timing_arguments


class VRProbe(TCPProbe):
    allow_unfocused = False  # The keyboard's focus-pause opt-in is not a VR mode.
    # The old keyboard guard's 8 deg/s trip would otherwise remain a hidden
    # speed restriction. Body holding, estop, hardware faults and links still
    # use Probe.guard; the old TCP velocity-integrator guards do not apply.
    moving_speed_limit = math.inf
    held_speed_limit = 1.0  # deg/s; VR-only trip threshold for held joints.

    def __init__(self, args):
        self.input_event = threading.Event()
        self.lift = LiftPlanner(json.loads(args.body_config_json), args.lift_speed_mm_s) if getattr(args, 'body_lift', False) else None
        self.lift_axis_active = False
        self.lift_stop_deadline = None
        super().__init__(args)

    def read_input(self):
        try:
            for line in sys.stdin:
                packet = json.loads(line)
                if not isinstance(packet, dict):
                    raise ValueError("无效输入")
                packet["server_read_monotonic"] = time.monotonic()
                with self.input_lock:
                    self.packet, self.received = packet, time.monotonic()
                    self.input_event.set()
                if packet.get("exit"):
                    self.exit.set()
        except Exception as exc:
            self.fault = str(exc)
        finally:
            self.exit.set()
            self.input_event.set()

    def pump_input(self, seconds):
        if not hasattr(self, "input_event"):
            return self.pump(seconds)
        deadline = time.monotonic()+seconds
        # Service fresh ROS feedback/guards while waiting. Incoming targets
        # interrupt the idle poll instead of all waiting for the next 5 ms tick.
        while True:
            self.pump(min(.001, max(.0001, deadline-time.monotonic())))
            if self.input_event.is_set():
                self.input_event.clear()
                return
            if time.monotonic() >= deadline:
                return

    def guard(self, link=True, cancel=True):
        rp.Probe.guard(self, link, cancel)
        deadline = getattr(self, 'lift_stop_deadline', None)
        if deadline is not None:
            if max(abs(v) for v in self.hw['velocity'][17:20]) < .005:
                self.lift_stop_deadline = None
                self.moving_joints = list(range(1, 15)) if self.path is not None else None
            elif time.monotonic() > deadline:
                raise RuntimeError('身体摇杆回中后未确认停止')

    def on_hw(self, msg):
        super().on_hw(msg)
        now = time.monotonic()
        if not hasattr(self, "hardware_rate"):
            self.hardware_rate = RateMeter()
        self.hardware_rate.mark(now)
        trace = getattr(self, "command_trace", None)
        target = getattr(self, "trace_target", None)
        if (trace is not None and target is not None and self.hw is not None
                and "server_command_observed" not in trace
                and max(abs(a-b) for a, b in zip(self.hw["command_position"], target)) < 1e-7):
            # This confirms observed command feedback, NOT physical arrival.
            trace["server_command_observed"] = now

    def report(self, force=False, **extra):
        now = time.monotonic()
        timing = self.timing if hasattr(self, "timing") else Timing()
        if not hasattr(self, "feedback_clock"):
            self.feedback_clock = Deadline(timing.feedback_hz, now)
        if force or now >= self.feedback_clock.next:
            if now >= self.feedback_clock.next:
                self.feedback_clock.advance(now)
            rate = getattr(self, "accepted_rate", None)
            super().report(True, control_protocol=KIND, arm_speed_limit=None,
                held_speed_limit_deg_s=self.held_speed_limit,
                timing=timing.payload(), target_count=rate.total if rate else 0,
                target_rate_hz=rate.hz(now) if rate else 0.,
                hardware_observed_hz=self.hardware_rate.hz(now) if hasattr(self, "hardware_rate") else 0.,
                command_trace=getattr(self, "command_trace", None),
                last_target_age_ms=getattr(self, "last_target_age_ms", None),
                body=self.lift.model.state(self.hw['position']) if getattr(self, 'lift', None) and self.hw else None,
                body_lift_enabled=bool(getattr(self, 'lift', None)), **extra)

    def publish_positions(self, points, now):
        msg = rp.JointTrajectory()
        msg.joint_names = rp.JOINTS
        # Keep the default zero header stamp (start on receipt). A nonzero
        # timestamp with a single t=0 point can already be in the past when
        # JTC validates it and cause the whole command to be rejected.
        for seconds, positions in points:
            point = rp.JointTrajectoryPoint()
            point.positions = positions
            # One point at t=0: no initial-to-target interpolation in Humble JTC.
            # It holds this already-resampled IK position until the next target.
            point.velocities = []
            point.accelerations = []
            point.time_from_start = rp.Duration(seconds=seconds).to_msg()
            msg.points.append(point)
        self.publisher.publish(msg)
        self.path, self.path_time = points, now

    def stop_stream(self, reason):
        self.ready = False
        self.phase = reason+"；停止追踪并保持当前位置"
        self.report(True)
        if self.path is not None:
            now = time.monotonic()
            current = self.path[-1][1][:]
            if getattr(self, 'lift', None):
                self.hold_positions[17:20] = current[17:20]
            hold = direct_positions(current, {}, self.hold_positions)
            self.publish_positions(hold, now)
            self.pump(.05)
            deadline = time.monotonic()+3.
            while max(abs(v) for v in self.hw["velocity"]) > .005:
                if time.monotonic() > deadline:
                    raise RuntimeError("释放输入后未确认停止")
                self.pump()
            if max(abs(a-b) for a, b in zip(self.hw["command_position"], hold[-1][1])) > 1e-5:
                raise RuntimeError("控制器未确认停止保持目标")
            self.hold_positions[:14] = self.hw["command_position"][:14]
            if getattr(self, 'lift', None):
                self.hold_positions[17:20] = self.hw['command_position'][17:20]
                self.lift.reset()
        self.path = None
        self.moving_joints = None
        self.lift_axis_active = False
        self.lift_stop_deadline = None
        self.stationary_enabled()
        self.phase = reason+"；全部关节保持位置"
        self.ready = True
        self.report(True)

    def run(self):
        self.timing = Timing.from_args(self.args)
        self.accepted_rate = RateMeter()
        self.moving_joints = None
        self.path = None
        self.stream_identity = None
        self.prepare()
        if self.args.check or self.args.prepare_check:
            return
        self.publisher = self.node.create_publisher(rp.JointTrajectory,
            f"/{rp.CONTROLLER}/joint_trajectory", 1)
        deadline = time.monotonic()+3
        while not self.publisher.get_subscription_count():
            if time.monotonic() > deadline:
                raise RuntimeError("双臂轨迹接口未就绪")
            self.pump()
        self.phase = ('双臂就绪；Grip 跟随，右摇杆上下升降，回中保持' if getattr(self, 'lift', None)
                      else '双臂就绪；各自握住 Grip 移动，松开停止；J15–J20 保持')
        self.report(True)
        accepted = None
        blocked = False
        while True:
            self.pump_input(1/self.timing.receiver_hz)
            with self.input_lock:
                packet = self.packet.copy()
            request = packet.get("request")
            now = time.monotonic()
            if request is None:
                if self.path is not None:
                    self.stop_stream('Grip / 升降摇杆已释放或输入暂停' if getattr(self, 'lift', None)
                                     else 'Grip 已松开或输入暂停')
                accepted = None
                blocked = False
                continue
            if blocked:
                continue
            try:
                if not isinstance(request, dict):
                    raise ValueError("无效双臂请求")
                if request.get("id") != (accepted or {}).get("id"):
                    if type(request.get("id")) is not int or request["id"] <= self.last_request:
                        raise ValueError("已丢弃旧双臂请求")
                    work_started = time.monotonic()
                    targets = validate(self.hw, request, now)
                    now = time.monotonic()
                    validated = now
                    if self.path is None:
                        self.stationary_enabled()
                        self.moving_joints = list(range(1, 15))
                    current = self.hold_positions[:] if self.path is None else self.path[-1][1][:]
                    points = direct_positions(current, targets, self.hold_positions)
                    axis = lift_axis(request)
                    lift = getattr(self, 'lift', None)
                    if abs(axis) > DEADBAND and lift is None:
                        raise ValueError('本次遥操未启用身体升降')
                    if lift is not None:
                        points[-1][1][17:20] = lift.sample(axis, points[-1][1], now)[17:20]
                        active = abs(axis) > DEADBAND
                        if active:
                            self.lift_stop_deadline = None
                            self.moving_joints = list(range(1, 15))+[18, 19, 20]
                        elif getattr(self, 'lift_axis_active', False):
                            self.lift_stop_deadline = now+1.
                        self.lift_axis_active = active
                        self.hold_positions[17:20] = points[-1][1][17:20]
                    self.publish_positions(points, now)
                    self.command_trace = {"id": request["id"],
                        "client_send": packet.get("client_send_monotonic"),
                        "server_read": packet.get("server_read_monotonic"),
                        "server_work": work_started, "server_validated": validated,
                        "server_published": time.monotonic()}
                    self.trace_target = points[-1][1][:]
                    self.accepted_rate.mark(now)
                    self.last_target_age_ms = 1000*(now-request["created"])
                    accepted = request
                    self.last_request = request["id"]
                if now-accepted["created"] > LEASE:
                    raise ValueError("PICO 目标超过 0.25 秒未更新")
                self.phase = ('PICO 双臂 / 身体升降；右摇杆回中保持' if getattr(self, 'lift_axis_active', False)
                              else '双臂 PICO 位置控制中；松开对应 Grip 停止并保持')
                self.ready = True
            except ValueError as exc:
                self.emit(jog_rejected=str(exc))
                self.stop_stream(str(exc))
                accepted = None
                blocked = True


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--prepare-check", action="store_true")
    mode.add_argument("--link-check", action="store_true")
    parser.add_argument("--duration", type=float, default=10.)
    parser.add_argument("--observe-ros", action="store_true")
    parser.add_argument('--body-lift', action='store_true')
    parser.add_argument('--body-config-json')
    parser.add_argument('--lift-speed-mm-s', type=float, default=15.)
    add_timing_arguments(parser)
    args = parser.parse_args()
    try:
        Timing.from_args(args)
    except ValueError as exc:
        parser.error(str(exc))
    if args.link_check:
        # This branch runs before the ownership lock, ROS initialization or any
        # controller/enable operation. Its synthetic targets are never published.
        from vr_link_check import run_server
        return run_server(args)
    lock = open("/home/fairino/.fairino_upper_identify.lock", "a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    load_ros()
    rp.rclpy.init()
    probe = VRProbe(args)
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
            time.sleep(.01)
        probe.node.destroy_node()
        rp.rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
