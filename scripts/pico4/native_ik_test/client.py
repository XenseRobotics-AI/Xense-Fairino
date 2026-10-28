#!/usr/bin/env python3
"""PICO desired-TCP sender for the separate robot-side native IK experiment."""
import argparse
import contextlib
import json
import math
from pathlib import Path
import queue
import select
import signal
import sys
import termios
import time
import tty

ROOT = Path(__file__).resolve().parents[3]
for directory in (ROOT, ROOT/'src', ROOT/'scripts/pico4/fairino'):
    sys.path.insert(0, str(directory))
import paths  # sets up the existing, validated robot/model module paths
from tcp_core import ARMS, MODEL_ID
from tool_frames import DEFAULT_CONFIG, load_geometry, tool_config
from upper_keyboard import Connection
from vendor_kinematics import VendorKinematics
from tcp_keyboard import verify_both
from vr_input import Stream, Clutch, POSE_FRAMES, controller_sides, rotation_mapping
from vr_cartesian import CartesianInput
from vr_heading import heading_alignment
from vr_timing import Deadline
from vr_exit import ExitLog, connection_state
from scripts.pico4.native_ik_test.protocol import KIND
from scripts.pico4.native_ik_test.transport import remote_command
from scripts.pico4.native_ik_test.native_input import NativeClutch


@contextlib.contextmanager
def keyboard():
    old = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
    if old is not None:
        tty.setcbreak(sys.stdin)
    try:
        yield
    finally:
        if old is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def key():
    if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return ''


def wait_for_input(stream):
    deadline = time.monotonic()+30
    last = None
    while time.monotonic() < deadline:
        try:
            frame = stream.read()
            if (time.monotonic()-frame['received_monotonic'] <= .15
                    and all(frame[s]['grip'] <= .3 for s in ARMS)):
                return
            message = '启动前请松开左右 Grip，等待新鲜 PICO 数据。'
        except ValueError as exc:
            message = str(exc)
        if message != last:
            print(message, flush=True)
            last = message
        time.sleep(.02)
    raise RuntimeError('未确认新鲜双手柄数据及 Grip 已释放；没有启动机器人控制')


def parser():
    p = argparse.ArgumentParser(description='独立 PICO + C++ 联合限位 IK 实机测试；默认只读检查')
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='使能 J1–J20；从当前姿态握 Grip 遥操')
    mode.add_argument('--check', action='store_true', help='只读检查反馈、依赖、模型和起始腕部范围；默认')
    p.add_argument('--host', default='192.168.58.1')
    p.add_argument('--user', default='fairino')
    p.add_argument('--arm', choices=('left', 'right', 'both'), default='both')
    p.add_argument('--command-hz', type=float, default=100.)
    p.add_argument('--feedback-hz', type=float, default=100.)
    p.add_argument('--feedback-age-ms', type=float, default=60.)
    p.add_argument('--input-timeout-ms', type=float, default=150.)
    p.add_argument('--sensitivity', type=float, default=.25)
    p.add_argument('--interpolation-delay-ms', type=float, default=12.)
    p.add_argument('--mode', choices=tuple(POSE_FRAMES), default='first_person')
    p.add_argument('--rotation-mode', choices=('auto', *POSE_FRAMES), default='auto')
    p.add_argument('--hand-mapping', choices=('auto', 'same', 'crossed'), default='auto')
    p.add_argument('--freeze-buttons', action='store_true')
    p.add_argument('--tcp-tool', choices=('flange', 'taccap'), default='flange')
    p.add_argument('--tool-config', type=Path, default=DEFAULT_CONFIG)
    return p


def checked_args(argv=None):
    p = parser()
    args = p.parse_args(argv)
    limits = {'command_hz': (20, 200), 'feedback_hz': (20, 200),
              'feedback_age_ms': (10, 100),
              'input_timeout_ms': (50, 200), 'sensitivity': (.05, 1),
              'interpolation_delay_ms': (0, 100)}
    for name, (lo, hi) in limits.items():
        value = getattr(args, name)
        if not math.isfinite(value) or not lo <= value <= hi:
            p.error(f'--{name.replace("_", "-")} 范围 {lo}–{hi}')
    try:
        args.tools = tool_config(args.tcp_tool,
            load_geometry(args.tool_config) if args.tcp_tool == 'taccap' else None,
            require_confirmed=args.execute)
    except (ValueError, OSError) as exc:
        p.error(str(exc))
    return args


class InputTargets:
    """Latch after input faults; retain the full unsatisfied TCP while gripped."""
    def __init__(self, args):
        self.args = args
        self.hands = controller_sides(args.mode, args.hand_mapping)
        rotation = rotation_mapping(args.mode, args.rotation_mode)
        # Only selected arms apply the native input domain: movement of an
        # unselected controller must not trip the selected arm's rotation limit.
        self.clutches = {s: (NativeClutch if args.arm in ('both', s) else Clutch)(
            s, mode=args.mode, rotation_mode=rotation) for s in ARMS}
        self.cartesian = CartesianInput(self.clutches, args.interpolation_delay_ms)
        self.paused = True
        self.sequence = 0

    def pause(self):
        self.paused = True
        self.cartesian.reset()

    def request(self, frame, status, kin, now, wrist_ready=True):
        age = now-frame['received_monotonic']
        if not math.isfinite(age) or not 0 <= age <= self.args.input_timeout_ms/1000:
            self.pause()
            raise ValueError('PICO 采样超过有效时间，正在保持最后指令；松开两侧 Grip 后重握')
        hw = status['hardware']
        state = status.get('native_test', {})
        if not status.get('ready') or state.get('need_rearm') or not wrist_ready:
            self.pause()
            return None
        if self.paused:
            if (all(frame[s]['grip'] <= .3 for s in ARMS)
                    and max(abs(v) for v in hw['velocity']) <= .005):
                self.paused = False
            return None
        goals = self.cartesian.goals(frame, hw, kin, self.hands, self.args.sensitivity,
                                     self.args.freeze_buttons, now)
        goals = {s: t for s, t in goals.items() if self.args.arm in ('both', s)}
        if not goals:
            return None
        self.sequence += 1
        return dict(kind=KIND, model=MODEL_ID, tool_id=self.args.tools['id'], id=self.sequence,
                    # Server clock only; subtract source age instead of renewing
                    # the lease of a repeated old PICO sample.
                    created=status['stamp']-age,
                    goals={s: t.reshape(16).tolist() for s, t in goals.items()})

    def calibrate(self, frame, status, now):
        self.pause()
        if not status.get('ready') or max(abs(v) for v in status['hardware']['velocity']) > .005:
            raise ValueError('请等待机器人停稳后再校准正前方')
        alignment, yaw = heading_alignment(frame, now)
        for clutch in self.clutches.values():
            clutch.set_heading(alignment)
        return yaw


def operate(args, kin, stream, connection, actions):
    controls = InputTargets(args)
    cadence = Deadline(args.command_hz, time.monotonic())
    startup = time.monotonic()
    proof = None
    checked = False
    last_message = None
    last_display = 0.
    while True:
        now = time.monotonic()
        while True:
            try:
                event = connection.events.get_nowait()
            except queue.Empty:
                break
            if 'error' in event:
                raise RuntimeError(event['error'])
            if 'jog_rejected' in event:
                controls.pause()
                print('接收端暂停：'+event['jog_rejected'], flush=True)
            if event.get('native_event') == 'feedback_preflight':
                print(f"使能前反馈检查：{event['result']}，{event['unique_frames']} 个新硬件帧，"
                      f"最大年龄 {event['max_age_ms']:.1f} ms / 预算 {event['limit_ms']:.1f} ms", flush=True)
            if event.get('checked'):
                proof, details = verify_both(kin, event['hardware'])
                checked = True
                print('只读检查通过；未使能、未发送运动指令。', flush=True)
                print(json.dumps(details, ensure_ascii=False), flush=True)
        if connection.done.is_set():
            if not args.execute and checked:
                return
            raise RuntimeError('机器人控制通道已结束，查看会话日志')
        character = key() if args.execute else ''
        if character in ('q', 'Q', '\x1b'):
            args.exit_reason = 'keyboard_stop'
            return
        if character == ' ':
            controls.pause()
            print('用户暂停：保持最后指令；松开两侧 Grip、停稳后重握。', flush=True)
        status = connection.latest
        if now-startup > 90 and not status.get('hardware'):
            raise RuntimeError('等待机器人编译/反馈超过 90 秒')
        outgoing = None
        send_proof = None
        if status.get('hardware'):
            if now-connection.last_status > .2:
                raise RuntimeError('机器人状态回传超过 200 ms 未更新')
            if args.execute:
                frame = None
                decision_now = None
                try:
                    frame = stream.read()
                    if proof is None:
                        proof, details = verify_both(kin, status['hardware'])
                        actions.write(json.dumps({'event': 'vendor_startup_check', 'details': details})+'\n')
                    # A callback may deliver this frame after the loop began,
                    # and the startup model check may itself take time. Assess
                    # the immutable frame only after reading/checking it; the
                    # loop's earlier timestamp can falsely make fresh data
                    # appear to come from the future.
                    decision_now = time.monotonic()
                    if (all(frame[s]['grip'] <= .3 for s in ARMS)
                            and 0 <= decision_now-frame['received_monotonic'] <= args.input_timeout_ms/1000):
                        send_proof = proof
                    if character in ('c', 'C'):
                        yaw = controls.calibrate(frame, status, decision_now)
                        print(f'正前方已校准 {yaw:+.1f}°；松开 Grip 后重握。', flush=True)
                    outgoing = controls.request(frame, status, kin, decision_now,
                        connection.wrist.ready(status.get('wrist_guard', {})) and connection.wrist.released)
                    last_message = None
                except ValueError as exc:
                    controls.pause()
                    if str(exc) != last_message:
                        print(str(exc), flush=True)
                        assessed = time.monotonic() if decision_now is None else decision_now
                        received = frame.get('received_monotonic') if frame is not None else None
                        actions.write(json.dumps({'event': 'input_pause', 'local_time': time.time(),
                            'reason': str(exc), 'assessed_monotonic': assessed,
                            'frame_received_monotonic': received,
                            'frame_age_ms': None if received is None else 1000*(assessed-received),
                            'input_timeout_ms': args.input_timeout_ms,
                            'grip': {side: (frame or {}).get(side, {}).get('grip') for side in ARMS},
                            'client_paused': controls.paused, 'server_ready': status.get('ready'),
                            'server_need_rearm': status.get('native_test', {}).get('need_rearm'),
                            'server_mode': status.get('native_test', {}).get('mode')},
                            ensure_ascii=False)+'\n')
                        last_message = str(exc)
                if outgoing:
                    actions.write(json.dumps({'event': 'tcp_goal', 'local_time': time.time(),
                                              'request': outgoing}, allow_nan=False)+'\n')
            if now-last_display > 1:
                test = status.get('native_test', {})
                errors = ' / '.join(
                    f"{side} {value['position_error_mm']:.1f} mm / {value['orientation_error_deg']:.2f}°"
                    for side, value in (test.get('result') or {}).items()
                    if value.get('position_error_mm') is not None and value.get('orientation_error_deg') is not None)
                print(f"{status.get('phase', '')} | "
                      f"IK {test.get('solve_ms', '?')} ms | "
                      f"硬件反馈年龄 {1000*status.get('hw_age', 0):.1f} ms"
                      + (' | 手柄输入等待双侧 Grip 松开，停稳后重握'
                         if args.execute and controls.paused else '')
                      + (' | 目标残差 '+errors if errors else ''), flush=True)
                last_display = now
        # Compilation happens before the remote stdin reader exists. Sending
        # 100 Hz heartbeats then could fill SSH stdin and block local exit.
        if status.get('stamp') is not None:
            connection.send(request=outgoing, kinematics=send_proof)
        time.sleep(cadence.advance(time.monotonic()))


def drain_stop(connection, timeout=4.):
    """On local exit, hold the last published position and confirm measured stop."""
    if connection.latest.get('stamp') is None:
        # No remote feedback/proof stage: enabling could not have happened.
        # finish() can send one exit token without filling a compiler's stdin.
        return True
    started = time.monotonic()
    while not connection.done.is_set() and time.monotonic()-started < timeout:
        connection.send(request=None)
        state = connection.latest
        if (connection.last_status > started and state.get('native_test', {}).get('settled')
                and time.monotonic()-connection.last_status < .2):
            return True
        time.sleep(.02)
    return False


def main(argv=None):
    args = checked_args(argv)
    connection = stream = None
    code = 0
    logs = ROOT/'.local/pico4/native-ik-test-logs'
    logs.mkdir(parents=True, exist_ok=True)
    exit_log = ExitLog(logs)
    exit_log.write('started', execute=args.execute, host=args.host, arm=args.arm)
    termination = dict(reason='completed')
    def interrupted(signum, _):
        termination.update(reason='signal', signal=signal.Signals(signum).name)
        raise KeyboardInterrupt
    old_term = signal.signal(signal.SIGTERM, interrupted)
    try:
        kin = VendorKinematics(tools=args.tools)
        command = remote_command(args)
        if args.execute:
            stream = Stream()
            stream.open()
            wait_for_input(stream)
            print('独立 C++ IK 测试：使能 J1–J20；J15–J20 保持。从当前姿态开始，无自动 Home。', flush=True)
            print(f'完整目标联合限位 IK；已取消额外逐周期速度/加速度约束与慢速跟踪层。'
                  f'灵敏度 {args.sensitivity:g}，手臂 {args.arm}。', flush=True)
            hands = controller_sides(args.mode, args.hand_mapping)
            print(f'映射 {args.mode}；旋转 {rotation_mapping(args.mode, args.rotation_mode)}；'
                  + '，'.join(f'{hands[side]} 手柄 → {side} 机械臂'
                              for side in ARMS if args.arm in ('both', side))+'。', flush=True)
            print('Grip 跟随 / 松开保持最后指令；空格暂停，c 校准正前方，q / Esc / Ctrl+C 退出下使能。', flush=True)
            print('腕部联合限位始终开启；此入口只控制双臂。', flush=True)
        connection = Connection(args, worker_command=command, log_directory=logs)
        exit_log.write('connected', connection=connection_state(connection))
        print('会话日志：'+str(connection.log_path), flush=True)
        with connection.log_path.with_suffix('.actions.jsonl').open('a', buffering=1) as actions, keyboard():
            actions.write(json.dumps({'event': 'configuration', 'execute': args.execute,
                'parameters': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}},
                ensure_ascii=False, allow_nan=False)+'\n')
            operate(args, kin, stream, connection, actions)
            termination['reason'] = getattr(args, 'exit_reason', 'completed')
    except KeyboardInterrupt:
        if termination['reason'] != 'signal':
            termination['reason'] = 'keyboard_interrupt'
        print('正在停止并退出。', flush=True)
    except Exception as exc:
        termination = dict(reason='exception', error_type=type(exc).__name__, error=str(exc))
        print('停止：'+str(exc), file=sys.stderr, flush=True)
        code = 1
    finally:
        exit_log.write('stopping', **termination, connection=connection_state(connection))
        if connection is not None:
            try:
                if args.execute and not connection.done.is_set():
                    if not drain_stop(connection):
                        print('未取得实际停稳确认；继续执行下使能清理。', file=sys.stderr)
                connection.finish()
                if args.execute and not connection.latest.get('disabled_confirmed'):
                    print('未收到全部下使能确认，请现场检查。', file=sys.stderr)
                    code = 1
                elif args.execute:
                    print('已确认上半身全部下使能。', flush=True)
            except (Exception, KeyboardInterrupt) as exc:
                print('退出清理未完成：'+str(exc), file=sys.stderr)
                code = 1
        if stream is not None:
            try:
                stream.close()
            except Exception as exc:
                print('PICO 接收器清理异常：'+str(exc), file=sys.stderr)
                code = 1
        exit_log.write('cleanup_finished', first_reason=termination['reason'], return_code=code,
                       connection=connection_state(connection))
        print('退出日志：'+str(exit_log.path), flush=True)
        signal.signal(signal.SIGTERM, old_term)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
