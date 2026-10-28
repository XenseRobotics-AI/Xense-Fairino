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
from vr_core import validate, direct_positions, KIND, LIFT_MODE
from vr_lift import lift_axis, DEADBAND
from vr_lift_presets import PresetLiftPlanner
from vr_timing import Timing, Deadline, RateMeter, add_timing_arguments
from tool_frames import tool_config, validate_runtime

STOP_CONFIRM_TIMEOUT = 3.
STOP_SPEED_RAD_S = .005
HOLD_COMMAND_TOLERANCE_RAD = 1e-5
LIFT_SAMPLE_MAX_GAP_S = .05


class VRProbe(TCPProbe):
    allow_unfocused = False  # The keyboard's focus-pause opt-in is not a VR mode.
    # The old keyboard guard's 8 deg/s trip would otherwise remain a hidden
    # speed restriction. Body holding, estop, hardware faults and links still
    # use Probe.guard; the old TCP velocity-integrator guards do not apply.
    moving_speed_limit = math.inf
    held_speed_limit = 1.0  # deg/s; VR-only trip threshold for held joints.

    def __init__(self, args):
        self.tools = validate_runtime(json.loads(args.tools_json)) if getattr(args, 'tools_json', None) else tool_config()
        self.input_event = threading.Event()
        self.lift = PresetLiftPlanner(json.loads(args.body_config_json)) if getattr(args, 'body_lift', False) else None
        self.lift_axis_active = False
        self.lift_stop_deadline = None
        self.last_lift_epoch = 0
        self.last_lift_sample_time = None
        super().__init__(args)

    def kinematics_matches(self, proof):
        return (super().kinematics_matches(proof)
                and proof.get('tool_id', 'flange') == self.tools['id'])

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
            fresh = self.hw_time > self.lift_stop_published
            error = max(abs(a-b) for a, b in zip(self.hw['command_position'][17:20], self.lift_stop_target))
            speed = max(abs(v) for v in self.hw['velocity'][17:20])
            if fresh and error <= HOLD_COMMAND_TOLERANCE_RAD and speed <= STOP_SPEED_RAD_S:
                self.emit(body_lift_stop_confirmation=dict(result='confirmed', fresh_feedback=True,
                    command_error_rad=error, max_velocity_rad_s=speed,
                    wait_ms=1000*(time.monotonic()-self.lift_stop_published)))
                self.lift_stop_deadline = None
                self.moving_joints = list(range(1, 15)) if self.path is not None else None
            elif time.monotonic() > deadline:
                raise RuntimeError('身体预设轨迹结束后未确认停止：需要新反馈、最终指令确认及实际静止')

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
                body_lift_preset=self.lift.snapshot(now) if getattr(self, 'lift', None) else None,
                body_lift_epoch=getattr(self, 'last_lift_epoch', 0),
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

    def hold_feedback(self, target, published_at):
        now = time.monotonic()
        errors = [abs(a-b) for a, b in zip(self.hw["command_position"], target)]
        joint = max(range(20), key=errors.__getitem__)
        velocity_joint = max(range(20), key=lambda i: abs(self.hw["velocity"][i]))
        return dict(published_monotonic=published_at, hardware_monotonic=self.hw_time,
                    fresh_feedback=self.hw_time > published_at,
                    wait_ms=1000*(now-published_at), hardware_age_ms=1000*(now-self.hw_time),
                    command_error_joint=joint+1, max_command_error_rad=errors[joint],
                    target_position_rad=target[joint],
                    feedback_command_position_rad=self.hw["command_position"][joint],
                    velocity_joint=velocity_joint+1,
                    max_velocity_rad_s=abs(self.hw["velocity"][velocity_joint]))

    def confirm_hold(self, target, published_at):
        """Confirm both the held command and measured stop using NEW feedback.

        The 100 Hz status stream can repeat the same hardware frame. A fixed
        sleep cannot confirm that the controller has consumed the hold target.
        Every pump continues the existing fault, estop, enable and link guards.
        """
        try:
            while True:
                self.pump(.01)
                state = self.hold_feedback(target, published_at)
                if (state["fresh_feedback"]
                        and state["max_command_error_rad"] <= HOLD_COMMAND_TOLERANCE_RAD
                        and state["max_velocity_rad_s"] <= STOP_SPEED_RAD_S):
                    self.emit(stop_confirmation=dict(result="confirmed", **state))
                    return
                if time.monotonic()-published_at >= STOP_CONFIRM_TIMEOUT:
                    if not state["fresh_feedback"]:
                        reason = "停止保持确认超时：未收到保持指令发布后的新硬件反馈"
                    elif state["max_command_error_rad"] > HOLD_COMMAND_TOLERANCE_RAD:
                        reason = "控制器未确认停止保持目标"
                    else:
                        reason = "释放输入后未确认停止"
                    raise RuntimeError(
                        f'{reason}：J{state["command_error_joint"]} 指令差 '
                        f'{state["max_command_error_rad"]:.6g} rad；'
                        f'J{state["velocity_joint"]} 速度 {state["max_velocity_rad_s"]:.6g} rad/s；'
                        f'反馈年龄 {state["hardware_age_ms"]:.1f} ms')
        except RuntimeError as exc:
            self.emit(stop_confirmation=dict(result="failed", reason=str(exc),
                                              **self.hold_feedback(target, published_at)))
            raise

    def stop_stream(self, reason, preserve_lift_gesture=False):
        # Reset even if no position target has been published yet: a partially
        # completed one-second gesture must not survive pause/Home/disconnect.
        # Only normal arm release under a fresh preset heartbeat can preserve
        # an untriggered gesture while the arms' physical stop is confirmed.
        lift = getattr(self, 'lift', None)
        preserve_lift_gesture = bool(preserve_lift_gesture and lift and not lift.moving
                                    and not getattr(self, 'lift_axis_active', False)
                                    and getattr(self, 'lift_stop_deadline', None) is None)
        # Ordinary Grip release is not a session pause. Advertising not-ready
        # here would make the client cancel its lift session/gesture while this
        # receiver is synchronously confirming only the released arms' hold.
        self.ready = preserve_lift_gesture
        if lift and not preserve_lift_gesture:
            self.lift.reset()
        self.lift_stop_deadline = None
        self.lift_axis_active = False
        self.last_lift_sample_time = None
        self.phase = reason+"；停止追踪并保持当前位置"
        self.report(True)
        try:
            if self.path is not None:
                now = time.monotonic()
                current = self.path[-1][1][:]
                if lift:
                    self.hold_positions[17:20] = current[17:20]
                hold = direct_positions(current, {}, self.hold_positions)
                self.publish_positions(hold, now)
                self.confirm_hold(hold[-1][1], time.monotonic())
                self.hold_positions[:14] = self.hw["command_position"][:14]
                if lift:
                    self.hold_positions[17:20] = self.hw['command_position'][17:20]
            self.path = None
            self.moving_joints = None
            self.lift_axis_active = False
            self.lift_stop_deadline = None
            self.stationary_enabled()
        except Exception:
            self.ready = False
            if lift:
                lift.reset()
            self.report(True)
            raise
        self.phase = reason+"；全部关节保持位置"
        self.ready = True
        self.report(True)

    def run(self):
        try:
            self.run_stream()
        finally:
            # Hardware/link/exit guards can raise outside the request handler.
            # A stopped session must never retain an autonomous body trajectory.
            if getattr(self, 'lift', None):
                self.lift.reset()
            self.lift_axis_active = False
            self.lift_stop_deadline = None
            self.last_lift_sample_time = None

    def run_stream(self):
        self.timing = Timing.from_args(self.args)
        self.accepted_rate = RateMeter()
        self.moving_joints = None
        self.path = None
        self.lift_axis_active = False
        self.lift_stop_deadline = None
        self.last_lift_epoch = getattr(self, 'last_lift_epoch', 0)
        self.last_lift_sample_time = None
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
        self.phase = ('双臂就绪；右摇杆上/下持续 1 秒到高位/下位，触发后回中仍继续到位' if getattr(self, 'lift', None)
                      else '双臂就绪；各自握住 Grip 移动，松开停止；J15–J20 保持')
        self.report(True)
        accepted = None
        blocked = False
        while True:
            self.pump_input(1/self.timing.receiver_hz)
            with self.input_lock:
                packet = self.packet.copy()
            request = self.wrist_request(packet)
            now = time.monotonic()
            if request is None:
                if self.path is not None:
                    self.stop_stream('遥操输入暂停，取消预设升降' if getattr(self, 'lift', None)
                                     else 'Grip 已松开或输入暂停')
                elif getattr(self, 'lift', None):
                    self.lift.reset()
                accepted = None
                blocked = False
                continue
            if blocked:
                # The latest-packet mailbox can overwrite a single None pause.
                # Only a validated NEW lift session's neutral/no-arms packet
                # may rearm without that transient packet ever being observed.
                epoch = request.get('lift_epoch') if isinstance(request, dict) else None
                if not (isinstance(request, dict) and request.get('lift_mode') == LIFT_MODE
                        and type(epoch) is int and epoch > self.last_lift_epoch
                        and request.get('lift') == 0. and request.get('arms') == {}):
                    continue
            try:
                if not isinstance(request, dict):
                    raise ValueError("无效双臂请求")
                new_request = request.get("id") != (accepted or {}).get("id")
                if request.get('lift_mode') != (accepted or {}).get('lift_mode'):
                    new_request = True
                # Epoch transitions cannot hide behind a reused request ID.
                # They must pass both protocol validation and ID monotonicity.
                if request.get('lift_mode') == LIFT_MODE and request.get('lift_epoch') != self.last_lift_epoch:
                    new_request = True
                targets = {}
                work_started = time.monotonic()
                if new_request:
                    if type(request.get("id")) is not int or request["id"] <= self.last_request:
                        raise ValueError("已丢弃旧双臂请求")
                    targets = validate(self.hw, request, now, getattr(self, 'tools', None))
                    now = time.monotonic()
                    validated = now
                    if request.get('lift_mode') == LIFT_MODE and getattr(self, 'lift', None) is None:
                        raise ValueError('本次遥操未启用身体升降，拒绝预设升降心跳或目标')
                    if request.get('lift_mode') == LIFT_MODE and request['lift_epoch'] < self.last_lift_epoch:
                        raise ValueError('已丢弃旧身体升降会话 lift_epoch；请暂停后重新回中')
                    current = self.hold_positions[:] if self.path is None else self.path[-1][1][:]
                    for side, target in targets.items():
                        offset = 0 if side == "left" else 7
                        self.wrist.guard.arm_segment(
                            current[offset:offset+7], target, self.hw["velocity"][offset:offset+7],
                            label="左腕 J6/J7" if side == "left" else "右腕 J13/J14")
                        # Actual feedback is checked separately from the previous
                        # command. A stale reference cannot hide a wrist crossing.
                        self.wrist.guard.arm_segment(
                            self.hw["position"][offset:offset+7], target,
                            self.hw["velocity"][offset:offset+7])
                    accepted = request
                    self.last_request = request["id"]
                    self.accepted_rate.mark(now)
                # Check every receiver cycle BEFORE autonomous body sampling.
                # Duplicate IDs cannot renew the accepted request's lease.
                now = time.monotonic()
                if now-accepted["created"] > LEASE:
                    raise ValueError("PICO 目标超过 0.25 秒未更新")
                current = self.hold_positions[:] if self.path is None else self.path[-1][1][:]
                lift = getattr(self, 'lift', None)
                if new_request and lift is not None:
                    if request.get('lift_mode') == LIFT_MODE:
                        epoch = request['lift_epoch']
                        if epoch > self.last_lift_epoch:
                            # Invalidate old autonomous motion even if its None
                            # cancellation was replaced in the latest mailbox.
                            lift.reset()
                            self.last_lift_epoch = epoch
                        blocked = False  # Full validation above, never raw epoch alone.
                        axis = lift_axis(request)
                        # Do not queue another gesture while the last body's
                        # physical stop is still being confirmed. Neutral can rearm.
                        if self.lift_stop_deadline is None or abs(axis) <= DEADBAND:
                            lift.observe(axis, current, now, gesture_id=request.get('lift_gesture'))
                    else:
                        # An arm-only packet cannot silently keep a body plan alive.
                        lift.reset()
                was_moving = bool(lift and lift.moving)
                cancelled_lift = self.lift_axis_active and not was_moving
                if was_moving and self.lift_axis_active:
                    previous_sample = self.last_lift_sample_time
                    if previous_sample is None or now-previous_sample > LIFT_SAMPLE_MAX_GAP_S+1e-9:
                        # A fresh network packet cannot make up a local receiver
                        # stall. Hold the LAST published position, never jump
                        # forward to the absolute-time trajectory's overdue point.
                        raise ValueError('身体预设采样间隔超过 50 ms，已取消轨迹；请暂停并回中重新开始')
                # Releasing both Grips without a body trajectory still performs
                # the existing full hold confirmation, rather than streaming
                # empty heartbeat position targets forever.
                if not accepted['arms'] and not was_moving and not cancelled_lift and self.lift_stop_deadline is None:
                    if self.path is not None:
                        preserve_gesture = bool(lift and accepted.get('lift_mode') == LIFT_MODE
                                                and 0. <= now-accepted['created'] <= LEASE)
                        self.stop_stream('双 Grip 已释放，身体预设未运行',
                                         preserve_lift_gesture=preserve_gesture)
                        # Preserve a valid neutral rearm after full hold
                        # confirmation. Never rearm from an expired packet.
                        after_hold = time.monotonic()
                        if (lift and accepted.get('lift_mode') == LIFT_MODE
                                and abs(lift_axis(accepted)) <= DEADBAND
                                and 0. <= after_hold-accepted['created'] <= LEASE):
                            lift.observe(0., self.hold_positions, after_hold,
                                         gesture_id=accepted.get('lift_gesture'))
                    self.phase = '双臂保持；右摇杆持续 1 秒选择高位/下位' if lift else '双臂保持'
                    self.ready = True
                    continue
                if new_request or was_moving or cancelled_lift:
                    if self.path is None:
                        self.stationary_enabled()
                    points = direct_positions(current, targets, self.hold_positions)
                    if lift is not None:
                        points[-1][1][17:20] = lift.sample(points[-1][1], now)[17:20]
                        self.hold_positions[17:20] = points[-1][1][17:20]
                    self.moving_joints = list(range(1, 15))
                    if was_moving or cancelled_lift or self.lift_stop_deadline is not None:
                        self.moving_joints += [18, 19, 20]
                    self.publish_positions(points, now)
                    published = time.monotonic()
                    if was_moving:
                        # Keep the time at which this successfully published
                        # target was sampled, so a slow publish is detected too.
                        self.last_lift_sample_time = now
                    if (was_moving or cancelled_lift) and not lift.moving:
                        self.lift_stop_target = points[-1][1][17:20]
                        self.lift_stop_published = published
                        self.lift_stop_deadline = published+STOP_CONFIRM_TIMEOUT
                    self.lift_axis_active = bool(lift and lift.moving)
                    self.command_trace = {"id": accepted["id"],
                        "client_send": packet.get("client_send_monotonic"),
                        "server_read": packet.get("server_read_monotonic"),
                        "server_work": work_started, "server_validated": validated if new_request else None,
                        "server_published": published, "body_sample": was_moving}
                    self.trace_target = points[-1][1][:]
                self.last_target_age_ms = 1000*(now-accepted["created"])
                self.phase = ('PICO 双臂 / 身体预设升降；回中继续到位，暂停取消' if self.lift_axis_active
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
    parser.add_argument('--tools-json')
    add_timing_arguments(parser)
    args = parser.parse_args()
    try:
        Timing.from_args(args)
        args.tools = validate_runtime(json.loads(args.tools_json)) if args.tools_json else tool_config()
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
