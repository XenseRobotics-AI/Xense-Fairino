#!/usr/bin/env python3
"""PICO input -> local kinematic model. No robot connection or actuator API.

This is an ideal position model, not a dynamics/latency/collision simulator:
accepted joint targets become model positions directly. The local vendor IK or
the constrained numerical IK can be selected before starting; both share the
same input mapping. A deliberate demo-only switch can remove the wrist envelope
(using box-constrained IK for the native backend); physical control is separate.
"""
import argparse
import json
import math
import queue
import signal
import sys
import threading
import time

from paths import ROOT
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'src'))

from apps.gripper_control.gripper_core import TriggerMapping
from body_core import load_config
from tcp_core import ARMS, multiply, tcp_forward_matrix
from tool_frames import DEFAULT_CONFIG, for_arm, load_geometry, tool_config
from vr_cartesian import CartesianInput
from vr_controls import JoystickControls, sensitivity_step
from vr_core import validate
from vr_heading import heading_alignment
from vr_home import HOME_PRESETS, HomeMotion, load_home, resolve_home
from vr_input import POSE_FRAMES, Stream, controller_sides, rotation_mapping
from vr_lift import LiftInput
from vr_lift_presets import HIGH_PERCENT, PresetLiftPlanner
from vr_native_ik import NativeIK
from vr_native_input import NativeClutch
from vr_plan import request as vendor_request
from vr_timing import Deadline
from wrist_guard import WristGuard


class ModelKinematics:
    """Independent Python FK used to validate the native solver's TCP."""
    def __init__(self, tools):
        self.tools = tools

    def forward_tcp(self, q, side):
        return multiply(tcp_forward_matrix(q), for_arm(self.tools, side)), 0.


def wrist_state(q):
    collision, violation, clearance = {}, {}, {}
    for side, joints in ARMS.items():
        pair = [q[j-1] for j in joints[-2:]]
        raw = WristGuard.clearance(pair, margin=0.)
        protected = WristGuard.clearance(pair)
        collision[side] = raw < -1e-9
        violation[side] = protected < -1e-9
        clearance[side] = protected if math.isfinite(protected) else None
    return dict(wrist_collision=collision, wrist_guard_violation=violation,
                wrist_clearance_deg=clearance)


class DemoEngine:
    def __init__(self, *, mode='mirror_same_direction', sensitivity=.25, tools=None,
                 home_preset='default', freeze_buttons=False, body_lift=False,
                 grippers=False, max_openings=None, hand_mapping='auto',
                 rotation_mode='auto', interpolation_delay_ms=12., wrist_guard=True,
                 ik_backend='native'):
        if ik_backend not in ('native', 'vendor'):
            raise ValueError('模型 IK 须选择 native 或 vendor')
        if type(sensitivity) not in (int, float) or not math.isfinite(sensitivity) or not 0 < sensitivity <= 1.:
            raise ValueError('模型灵敏度须在 0–1 之间且大于零')
        self.mode, self.sensitivity = mode, sensitivity
        self.ik_backend = ik_backend
        self.hands = controller_sides(mode, hand_mapping)
        if ik_backend == 'vendor':
            # Pure local ctypes FK/IK, without any robot SDK/transport. Do not
            # construct the native solver or require a C++ compiler on this path.
            from vendor_kinematics import VendorKinematics
            self.kin = VendorKinematics(tools=tools or tool_config())
            self.native = None
        else:
            self.kin = ModelKinematics(tools or tool_config())
            self.native = NativeIK(self.kin.tools)
        self.unconstrained = None
        self.guard = WristGuard(wrist_guard)
        self.clutches = {side: NativeClutch(side, mode, rotation_mapping(mode, rotation_mode)) for side in ARMS}
        self.cartesian = CartesianInput(self.clutches, interpolation_delay_ms)
        self.joysticks = JoystickControls()
        self.home = load_home(resolve_home(home_preset))
        self.body_model = load_config()
        self.q = self.body_model.pose(dict(height=HIGH_PERCENT, forward=0., roll=0., yaw=0.), [0.]*20)
        for side, joints in ARMS.items():
            if ik_backend == 'vendor':
                self.kin.verify_tcp(self.home['target'][side], side)
            for j, value in zip(joints, self.home['target'][side]):
                self.q[j-1] = value
        self.freeze_buttons = freeze_buttons
        self.lift_input = LiftInput() if body_lift else None
        self.lift = PresetLiftPlanner(self.body_model.config) if body_lift else None
        self.last_lift_stamp = None
        self.trigger = TriggerMapping(max_openings) if grippers else None
        self.grippers = dict.fromkeys(ARMS, 1.)
        self.home_pending = False
        self.home_motion = None
        self.sequence = 0
        self.paused = True
        self.message = '模型已在起始位姿；等待 PICO，松开两侧 Grip 后重握'
        self.heading_yaw = None
        self.heading_release = False
        self.last_time = None
        self.partial = False
        self.wrist_limited = False
        self.sides = ()
        self.stopped = False

    def hardware(self):
        # No simulated inertial tail: this ideal kinematic model is settled at
        # the last accepted position, so it has no measured actuator velocity.
        return dict(joint_names=[f'j{i}' for i in range(1, 21)], position=self.q[:],
                    command_position=self.q[:], velocity=[0.]*20)

    def hold(self, message):
        self.paused = True
        self.home_pending = False
        self.home_motion = None
        self.cartesian.reset()
        self.joysticks.reset()
        if self.lift:
            self.lift.reset()
            self.lift_input.reset()
        if self.trigger:
            self.trigger.reset()
        self.sides = ()
        self.partial = False
        self.wrist_limited = False
        self.message = message

    def command(self, command):
        if command in ('stop', 'quit'):
            self.hold('模型演示已停止')
            self.stopped = True
        elif command == 'pause':
            self.hold('模型暂停；松开两侧 Grip 后重握')
        elif command in ('slower', 'faster'):
            self.sensitivity = sensitivity_step(self.sensitivity, 1 if command == 'faster' else -1)
            self.hold(f'灵敏度 {self.sensitivity:g}；松开两侧 Grip 后重握')
        elif command == 'home':
            if self.home_motion is not None or self.home_pending:
                self.hold('已中止模型 Home；松开两侧 Grip 后重握')
            else:
                self.hold('准备模型 Home；请松开两侧 Grip')
                self.home_pending = True
        elif command in ('guard_on', 'guard_off'):
            enabled = command == 'guard_on'
            if enabled and any(wrist_state(self.q)['wrist_guard_violation'].values()):
                self.hold('当前模型腕部超出保护范围；保持关闭，先回 Home 后再开启')
            else:
                self.guard = WristGuard(enabled)
                self.hold('模型腕部联合限位已'+('开启' if enabled else '关闭')+'；松开两侧 Grip 后重握')
        else:
            raise ValueError('未知模型控制命令：'+str(command))

    @staticmethod
    def released(frame):
        return all(frame[s]['grip'] <= .3 for s in ARMS)

    def _apply(self, packet):
        if packet is None:
            self.sides = ()
            self.partial = False
            self.wrist_limited = False
            return
        for side, entry in packet['arms'].items():
            for j, value in zip(ARMS[side], entry['target']):
                self.q[j-1] = value
        self.sides = tuple(packet['arms'])
        self.partial = any(a.get('native_ik', {}).get('success') is False for a in packet['arms'].values())
        self.wrist_limited = any(a.get('wrist_scale', 1.) < 1. for a in packet['arms'].values())

    def step(self, frame, now, command=None, input_error=None):
        if type(now) not in (int, float) or not math.isfinite(now):
            raise ValueError('模型运行时间无效')
        if self.last_time is not None and now < self.last_time:
            raise ValueError('模型运行时间倒退')
        gap = self.last_time is not None and now-self.last_time > .25
        self.last_time = now
        if command:
            self.command(command)
            return self.snapshot(now, frame)
        if self.stopped:
            return self.snapshot(now, frame)
        if frame is None:
            self.hold(input_error or '等待 PICO 新数据；模型保持当前姿态')
            return self.snapshot(now)
        received = frame.get('received_monotonic')
        if type(received) not in (int, float) or not math.isfinite(received) or not 0 <= now-received <= .25:
            self.hold('PICO 输入超过 0.25 秒或时间无效；松开两侧 Grip 后重握')
            return self.snapshot(now)
        if gap:
            self.hold('模型更新中断超过 0.25 秒；松开两侧 Grip 后重握')
            return self.snapshot(now, frame)
        try:
            controls = self.joysticks.update(frame, now)
            if controls['calibrate_heading']:
                self.heading_release = True
                if self.home_pending or self.home_motion is not None:
                    raise ValueError('已中止模型 Home；松开摇杆后重新校准正前方')
                alignment, self.heading_yaw = heading_alignment(frame, now)
                for clutch in self.clutches.values():
                    clutch.set_heading(alignment)
                self.hold(f'模型正前方已校准 {self.heading_yaw:+.1f}°；松开摇杆后重握 Grip')
                return self.snapshot(now, frame)
            if controls['sensitivity']:
                self.command('faster' if controls['sensitivity'] > 0 else 'slower')
                return self.snapshot(now, frame)
            if controls['home']:
                self.command('home')
            hw = self.hardware()
            if self.home_motion is not None and not self.released(frame):
                self.hold('Grip 已握下，中止模型 Home；松开后重握')
                return self.snapshot(now, frame)
            if self.home_pending or self.home_motion is not None:
                if self.home_pending and self.released(frame):
                    self.home_motion = HomeMotion(self.home, hw, now, self.guard)
                    self.home_pending = False
                if self.home_motion is not None:
                    if self.home_motion.complete(hw, now):
                        self.hold('模型 Home 已到位；松开 Grip 后重握')
                    else:
                        self.sequence += 1
                        self._apply(self.home_motion.request(self.kin, hw, now, self.sequence, now))
                        self.message = '模型正在回 Home；握任一 Grip 可中止'
                return self.snapshot(now, frame)
            if self.paused:
                if self.released(frame) and (not self.heading_release or
                    (not frame['left'].get('axis_click') and not frame['right'].get('axis_click')
                     and frame['right'].get('joystick_available') and abs(frame['right'].get('axis_y', 1.)) <= .2)):
                    self.paused = False
                    self.heading_release = False
                    self.message = '模型就绪；握住对应 Grip 移动'
                return self.snapshot(now, frame)
            goals = self.cartesian.goals(frame, hw, self.kin, self.hands, self.sensitivity,
                                         self.freeze_buttons, now)
            packet = None
            if goals:
                self.sequence += 1
                if self.ik_backend == 'vendor':
                    # Retain the real client's arm-angle reference, path checks
                    # and Cartesian retreat when the optional guard is enabled.
                    packet = vendor_request(self.kin, hw, goals, self.sequence, now,
                                            self.cartesian.references, self.guard)
                    validate(hw, packet, now, self.kin.tools)
                else:
                    solver = self.native
                    if not self.guard.enabled:
                        if self.unconstrained is None:
                            from vr_demo_ik import DemoBoxIK
                            self.unconstrained = DemoBoxIK(self.kin.tools)
                        solver = self.unconstrained
                    packet = solver.request(self.kin, hw, goals, self.sequence, now,
                                            self.cartesian.references, self.guard)
                self.cartesian.accepted_plan(packet, frame, self.hands)
            self._apply(packet)
            if self.lift:
                active = frame['right'].get('joystick_available') and not frame['right'].get('axis_click')
                axis = self.lift_input.value(frame, active)
                if not active or self.lift_input.interrupted:
                    self.lift.reset()
                else:
                    # Only actual SDK receive samples count toward the 1 s hold.
                    if received != self.last_lift_stamp:
                        self.lift.observe(axis, self.q, now, self.lift_input.gesture_id)
                        self.last_lift_stamp = received
                    self.q = self.lift.sample(self.q, now)
            if self.trigger:
                self.grippers.update(self.trigger.update(frame, self.hands, ARMS))
            self.message = '模型跟随' if goals else '模型保持；握 Grip 移动'
            if self.partial:
                self.message += '；目标暂不可达，显示当前可行解'
            elif self.wrist_limited:
                self.message += '；原厂 IK 结果受腕部保护限制，已舍弃越界位移'
        except (ValueError, RuntimeError) as exc:
            self.hold('模型已暂停：'+str(exc))
        return self.snapshot(now, frame)

    def snapshot(self, now, frame=None):
        phase = ('已停止' if self.stopped else '回 Home' if self.home_motion is not None else
                 '等待 Home' if self.home_pending else '暂停' if self.paused else '就绪')
        return dict(demo=True, hardware=self.hardware(), grippers=self.grippers.copy() if self.trigger else {},
                    phase=phase, message=self.message, paused=self.paused, sensitivity=self.sensitivity,
                    mode=self.mode, ik_requested_backend=self.ik_backend,
                    ik_backend=('demo_joint_box_only' if self.ik_backend == 'native' and not self.guard.enabled
                                else self.ik_backend),
                    ik_partial=self.partial, wrist_limited=self.wrist_limited,
                    sides='/'.join(self.sides) or '保持',
                    grips={s: frame[s]['grip'] for s in ARMS} if frame else {},
                    body=self.body_model.state(self.q),
                    body_lift_preset=self.lift.snapshot(now) if self.lift else None,
                    heading_calibrated=self.heading_yaw is not None, heading_yaw_deg=self.heading_yaw,
                    heading_message='未校准' if self.heading_yaw is None else f'已校准 {self.heading_yaw:+.1f}°',
                    wrist_guard_enabled=self.guard.enabled, wrist_guard=dict(enabled=self.guard.enabled),
                    **wrist_state(self.q))


class DemoCommands:
    """No authentication/credentials: stdin contains model commands only."""
    def __init__(self, stream):
        self.stream = stream
        self.queue = queue.Queue(maxsize=32)
        self.closed = threading.Event()
        self.reason = None

    def read(self):
        try:
            for line in self.stream:
                packet = json.loads(line)
                if not isinstance(packet, dict) or set(packet) != {'command'} or packet['command'] not in (
                        'pause', 'home', 'slower', 'faster', 'stop', 'quit', 'guard_on', 'guard_off'):
                    raise ValueError('无效的模型控制命令')
                self.queue.put_nowait(packet['command'])
                if packet['command'] in ('stop', 'quit'):
                    break
            self.reason = '模型控制台已关闭'
        except (ValueError, TypeError, OSError, queue.Full) as exc:
            self.reason = str(exc)
        finally:
            self.closed.set()

    def take(self):
        try:
            return self.queue.get_nowait()
        except queue.Empty:
            return None


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('需要有限正数')
    return number


def parser():
    result = argparse.ArgumentParser(description='PICO 控制本地模型，无机器人连接')
    result.add_argument('--gui-stdio', action='store_true')
    result.add_argument('--ik-backend', choices=('native', 'vendor'), default='native',
                        help='本地模型 IK：native 数值求解，vendor 原厂离线库；均不连接机器人')
    result.add_argument('--mode', choices=tuple(POSE_FRAMES), default='mirror_same_direction')
    result.add_argument('--sensitivity', type=positive, default=.25)
    result.add_argument('--rotation-mode', choices=('auto', *POSE_FRAMES), default='auto')
    result.add_argument('--hand-mapping', choices=('auto', 'same', 'crossed'), default='auto')
    result.add_argument('--command-hz', type=positive, default=100.)
    result.add_argument('--interpolation-delay-ms', type=float, default=12.)
    result.add_argument('--tcp-tool', choices=('flange', 'taccap'), default='flange')
    result.add_argument('--tool-config', default=str(DEFAULT_CONFIG))
    result.add_argument('--home-preset', choices=tuple(HOME_PRESETS), default='default')
    result.add_argument('--freeze-buttons', action='store_true')
    result.add_argument('--body-lift', action='store_true')
    result.add_argument('--grippers', action='store_true')
    result.add_argument('--disable-wrist-guard', '--no-wrist-guard', action='store_true')
    for side in ARMS:
        result.add_argument(f'--{side}-gripper-max-open-percent', type=float, default=100.)
    return result


def emit(state):
    print('WORKBENCH_DEMO '+json.dumps(state, ensure_ascii=False, allow_nan=False), flush=True)


def interrupted(*_):
    raise KeyboardInterrupt()


def main(argv=None):
    arguments = parser()
    args = arguments.parse_args(argv)
    if not 1. <= args.command_hz <= 100.:
        arguments.error('模型更新频率须为 1–100 Hz')
    commands = DemoCommands(sys.stdin) if args.gui_stdio else None
    if commands:
        threading.Thread(target=commands.read, daemon=True).start()
    stream = None
    engine = None
    old_term = signal.signal(signal.SIGTERM, interrupted)
    try:
        tools = tool_config(args.tcp_tool, load_geometry(args.tool_config) if args.tcp_tool == 'taccap' else None)
        engine = DemoEngine(mode=args.mode, sensitivity=args.sensitivity, tools=tools,
                            home_preset=args.home_preset, freeze_buttons=args.freeze_buttons,
                            body_lift=args.body_lift, grippers=args.grippers,
                            max_openings={s: getattr(args, s+'_gripper_max_open_percent')/100. for s in ARMS},
                            hand_mapping=args.hand_mapping, rotation_mode=args.rotation_mode,
                            interpolation_delay_ms=args.interpolation_delay_ms,
                            wrist_guard=not args.disable_wrist_guard, ik_backend=args.ik_backend)
        emit(engine.snapshot(time.monotonic()))
        # No hardware credentials or transport are constructed on this path.
        if commands and commands.closed.is_set():
            return 0
        stream = Stream()
        stream.open()
        cadence = Deadline(args.command_hz, time.monotonic())
        last_display = 0.
        print('模型演示：握 Grip 遥操；右摇杆按压回 Home；左摇杆按压校准正前方。', flush=True)
        while not engine.stopped:
            if commands and commands.closed.is_set():
                engine.command('stop')
                break
            command = commands.take() if commands else None
            frame, error = None, None
            try:
                frame = stream.read()
            except ValueError as exc:
                error = str(exc)
            now = time.monotonic()
            state = engine.step(frame, now, command, error)
            if now-last_display >= 1/30 or command:
                state['update_hz'] = args.command_hz
                emit(state)
                last_display = now
            time.sleep(cadence.advance(time.monotonic()))
        return 0
    except KeyboardInterrupt:
        return 0
    except (ValueError, RuntimeError, OSError) as exc:
        print('模型演示未启动或已结束：'+str(exc), flush=True)
        return 1
    finally:
        if stream:
            stream.close()
        if engine:
            engine.command('stop')
            try:
                emit(engine.snapshot(time.monotonic()))
            except (BrokenPipeError, OSError):
                pass
        signal.signal(signal.SIGTERM, old_term)


if __name__ == '__main__':
    raise SystemExit(main())
