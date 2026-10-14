#!/usr/bin/env python3
"""PICO -> vendor flange IK -> one guarded FAIRINO ROS trajectory stream."""
import argparse
import base64
import contextlib
import json
import math
import queue
import select
import shlex
import signal
import sys
import termios
import time
import tty
import zlib
from pathlib import Path

from paths import ROOT, TCP, SHARED
from upper_keyboard import Connection
from tcp_keyboard import verify_both
from tcp_core import ARMS
from vendor_kinematics import VendorKinematics
from vr_core import validate
from vr_input import Clutch, Stream, MAPPING_ID, POSE_FRAMES, controller_sides, rotation_mapping
from vr_controls import JoystickControls, sensitivity_step
from vr_home import DEFAULT_HOME, load_home, HomeMotion
from vr_timing import Timing, Deadline, RateMeter, add_timing_arguments
from mounts import tcp_in_chest
from vr_plan import request
from vr_cartesian import CartesianInput

HERE = Path(__file__).resolve().parent


def remote_command(args):
    bundle = {name: (SHARED/(name+".py")).read_text() for name in ("probe_core", "robot_probe")}
    bundle.update({name: (TCP/(name+".py")).read_text() for name in ("tcp_core", "tcp_frames", "tcp_jog", "tcp_robot")})
    bundle.update({name: (HERE/(name+".py")).read_text() for name in ("vr_timing", "vr_core", "vr_link_check", "vr_robot")})
    encoded = base64.b64encode(zlib.compress(json.dumps(bundle).encode())).decode()
    argv = ["vr_robot.py", *Timing.from_args(args).argv()]
    if getattr(args, "link_check", False):
        argv.extend(["--link-check", "--duration", str(args.duration)])
        if getattr(args, "observe_ros", False):
            argv.append("--observe-ros")
    elif not args.execute:
        argv.append("--prepare-check" if args.prepare_check else "--check")
    loader = ("import base64,zlib,json,sys,types;"
        f"bundle=json.loads(zlib.decompress(base64.b64decode({encoded!r})));\n"
        "for name in ('probe_core','robot_probe','tcp_core','tcp_frames','tcp_jog','tcp_robot','vr_timing','vr_core','vr_link_check'):\n"
        " m=types.ModuleType(name);sys.modules[name]=m;exec(compile(bundle[name],name+'.py','exec'),m.__dict__)\n"
        f"sys.argv={argv!r}\n"
        "exec(compile(bundle['vr_robot'],'vr_robot.py','exec'),{'__name__':'__main__'})")
    setup = "\n".join(["set -e", "source /opt/ros/humble/setup.bash",
        "source /opt/fairino_art_plugin/plugin/install/setup.bash",
        "export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
        "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/fairino/fastrtps_udp.xml",
        "echo UPPERPROBE_CONNECTED", "exec python3 -u -c "+shlex.quote(loader)])
    return "bash -c "+shlex.quote(setup)


@contextlib.contextmanager
def keyboard():
    old = None
    if sys.stdin.isatty():
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)
    try:
        yield
    finally:
        if old is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def key():
    if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return ""


def wait_for_input(stream, duration=30.):
    deadline = time.monotonic()+duration
    previous = None
    while time.monotonic() < deadline:
        try:
            frame = stream.read()
            if all(frame[s]["grip"] <= .3 for s in ARMS):
                return frame
            message = "启动前请松开左右 Grip。"
        except ValueError as exc:
            message = str(exc)
        if message != previous:
            print(message, flush=True)
            previous = message
        time.sleep(.02)
    raise RuntimeError("未确认新鲜双手柄数据及 Grip 已释放；没有启动机器人控制。")


def operate(args, kin, stream, connection):
    timing = Timing.from_args(args)
    cadence = Deadline(timing.command_hz, time.monotonic())
    targets_rate, feedback_rate = RateMeter(), RateMeter()
    control_mode = getattr(args, "mode", "first_person")
    rotation_mode = rotation_mapping(control_mode, getattr(args, "rotation_mode", "auto"))
    hands = controller_sides(control_mode, getattr(args, "hand_mapping", "auto"))
    freeze_buttons = getattr(args, "freeze_buttons", False)
    clutches = {side: Clutch(side, mode=control_mode, rotation_mode=rotation_mode) for side in ARMS}
    cartesian = CartesianInput(clutches, timing.interpolation_delay_ms)
    joysticks = JoystickControls()
    home = load_home(getattr(args, "home_file", DEFAULT_HOME)) if args.execute else None
    if home:
        for q in home["target"].values():
            kin.verify_tcp(q)
    home_pending = args.execute and not getattr(args, "no_auto_home", False)
    home_motion = None
    sensitivity = args.sensitivity
    paused = True
    proof = None
    sequence = 0
    outgoing = None
    last_sides = ()
    last_x = False
    last_message = None
    last_display = 0.
    last_input_stop = None
    stream_bad_since = None
    checked = False
    startup = time.monotonic()
    action_path = connection.log_path.with_suffix(".actions.jsonl")
    print("日志：", connection.log_path, flush=True)
    if args.execute:
        print("即将使能 J1–J20；" + ("先自动回双臂 Home，完成后握 Grip 遥操。" if home_pending else "保持当前姿态，握 Grip 遥操。"), flush=True)
        print("左摇杆上/下：灵敏度增/减一档；右摇杆按压：回 Home（再按中止）；左摇杆按压：暂停。", flush=True)
        print("Home 仅移动双臂；请松开两侧 Grip，途中握任一 Grip 或按空格可中止。", flush=True)
        if freeze_buttons:
            print("按住 A/X 冻结旋转（只平移）；按住 B/Y 冻结位移（只旋转）；同时按住冻结两者。", flush=True)
            print("按键仅影响该手柄控制的手臂；松开不补走；仍须握 Grip。+/- 调灵敏度，空格暂停。", flush=True)
        else:
            print("X 切换灵敏度 0.25/0.5/1；B/Y 或空格暂停。", flush=True)
        print("Esc/Ctrl+C 退出并全部下使能。", flush=True)
        labels = {"left": "左", "right": "右"}
        print(f"位移 {control_mode} / 旋转 {rotation_mode}；" + "，".join(
            f"{labels[hand]}手柄 → 机器人{labels[arm]}臂" for arm, hand in hands.items()), flush=True)
        print(f"TCP 插值 v3：位移线性 / 姿态 SLERP → IK；采样/下发 {timing.command_hz:g} Hz，缓冲 {timing.interpolation_delay_ms:g} ms。", flush=True)
        print(f"机器人单点保持，无额外关节追赶段；接收轮询 {timing.receiver_hz:g} Hz / 回传 {timing.feedback_hz:g} Hz。", flush=True)
        print(f"坐标映射 {MAPPING_ID}：双臂按厂家整机 URDF 对齐胸部，X 前 / Y 左 / Z 上。", flush=True)
        print("方向随胸部变化；腰部或折叠结构倾斜时，胸部轴不保证与地面对齐。", flush=True)
        print("保留模型关节限位、厂家逆解检查、急停和断流停止。", flush=True)
    with action_path.open("a", buffering=1) as log, keyboard():
        log.write(json.dumps({"local_time": time.time(), "event": "timing_config", **timing.payload()})+"\n")
        while True:
            now = time.monotonic()
            stop_reason = None
            while True:
                try:
                    event = connection.events.get_nowait()
                except queue.Empty:
                    break
                if "stamp" in event:
                    feedback_rate.mark(time.monotonic())
                if "error" in event:
                    raise RuntimeError(event["error"])
                if "cleanup_warning" in event:
                    print(event["cleanup_warning"], flush=True)
                if "jog_rejected" in event:
                    stop_reason = event["jog_rejected"]
                if event.get("checked"):
                    checked = True
                    proof, details = verify_both(kin, event["hardware"])
                    print("机器人检查通过，未使能、未移动。", flush=True)
                    print(json.dumps(details, ensure_ascii=False), flush=True)
            if connection.done.is_set():
                if not args.execute and checked:
                    return 0
                raise RuntimeError("机器人控制通道已结束，请查看日志中的最后状态")
            status = connection.latest
            hw = status.get("hardware")
            if hw is None:
                if now-startup > 12:
                    raise RuntimeError("等待机器人状态超时")
                time.sleep(.02)
                continue
            if not args.execute:
                # Controller unloading reports its final state after ROS calls;
                # no motion lease is running in either inspection mode.
                if now-startup > 60.:
                    raise RuntimeError("机器人接口检查未在 60 秒内结束")
                connection.send()
                time.sleep(.02)
                continue
            if now-connection.last_status > .25:
                raise RuntimeError("机器人状态反馈超过 0.25 秒未更新")
            character = key()
            if character in ("\x1b", "q", "Q"):
                return 0
            frame = None
            try:
                frame = stream.read()
                stream_bad_since = None
            except ValueError as exc:
                stop_reason = str(exc)
                joysticks.reset()
                if stream_bad_since is None:
                    stream_bad_since = now
                elif now-stream_bad_since > 3.:
                    raise RuntimeError("PICO 数据中断超过 3 秒，结束本次控制")
            if frame:
                controls = joysticks.update(frame)
                x = frame["left"]["primary"]
                if controls["sensitivity"]:
                    sensitivity = sensitivity_step(sensitivity, controls["sensitivity"])
                    stop_reason = f"灵敏度已改为 {sensitivity:g}，请松开两侧 Grip"
                elif (not freeze_buttons and x and not last_x) or character in ("+", "=", "-"):
                    levels = (.25, .5, 1.)
                    if character == "-":
                        sensitivity = next((v for v in reversed(levels) if v < sensitivity-1e-6), levels[-1])
                    else:
                        sensitivity = next((v for v in levels if v > sensitivity+1e-6), levels[0])
                    stop_reason = f"灵敏度已改为 {sensitivity:g}，请松开两侧 Grip"
                last_x = x
                if (not freeze_buttons and any(frame[s]["secondary"] for s in ARMS)) or character == " " or controls["pause"]:
                    stop_reason = "用户暂停，请松开两侧 Grip 后重握"
                if controls["home"] and not stop_reason:
                    if home_motion is not None or home_pending:
                        stop_reason = "用户中止 Home；松开 Grip 后可继续遥操"
                    else:
                        home_pending = True
                        paused, outgoing, last_sides = True, None, ()
                        cartesian.reset()
                        print("准备回 Home：松开两侧 Grip，等待双臂停止。", flush=True)
                        log.write(json.dumps({"local_time": time.time(), "event": "home_requested"})+"\n")
                if home_motion is not None and any(frame[s]["grip"] > .3 for s in ARMS):
                    stop_reason = "Grip 已握下，中止 Home；松开后可继续遥操"
            if proof is None and frame is not None:
                proof, _ = verify_both(kin, hw)
            if stop_reason:
                home_pending, home_motion = False, None
                paused = True
                outgoing = None
                last_sides = ()
                cartesian.reset()
                if stop_reason != last_message:
                    print(stop_reason, flush=True)
                    log.write(json.dumps({"local_time": time.time(), "event": "input_paused", "reason": stop_reason,
                        "input": {s: {k: v for k, v in frame[s].items() if k != "pose"} for s in ARMS} if frame else None}, ensure_ascii=False)+"\n")
                    last_message = stop_reason
            elif frame and status.get("ready") and (home_pending or home_motion is not None):
                try:
                    if home_pending:
                        outgoing, last_sides = None, ()
                        if (all(frame[s]["grip"] <= .3 for s in ARMS)
                                and max(abs(v) for v in hw["velocity"]) < .005):
                            home_motion = HomeMotion(home, hw, now)
                            home_pending = False
                            print(f"双臂回 Home，预计 {home_motion.duration:.1f} 秒；头腰保持。", flush=True)
                            log.write(json.dumps({"local_time": time.time(), "event": "home_started",
                                "duration": home_motion.duration, "joint_degrees": home["joint_degrees"]})+"\n")
                    if home_motion is not None:
                        if home_motion.complete(hw, now):
                            home_motion = None
                            paused, outgoing, last_sides = True, None, ()
                            print("双臂 Home 已到位；握 Grip 开始遥操。", flush=True)
                            log.write(json.dumps({"local_time": time.time(), "event": "home_completed"})+"\n")
                        else:
                            sequence += 1
                            outgoing = home_motion.request(kin, hw, now, sequence, status["stamp"])
                            last_sides = tuple(ARMS)
                            targets_rate.mark(now)
                            log.write(json.dumps({"local_time": time.time(), "event": "home_target", "request": outgoing})+"\n")
                except ValueError as exc:
                    home_pending, home_motion = False, None
                    paused, outgoing, last_sides = True, None, ()
                    print(f"Home 已中止：{exc}", flush=True)
                    log.write(json.dumps({"local_time": time.time(), "event": "home_aborted", "reason": str(exc)}, ensure_ascii=False)+"\n")
            elif frame and status.get("ready"):
                if paused:
                    if (all(frame[s]["grip"] <= .3 for s in ARMS)
                            and max(abs(x) for x in hw["velocity"][:14]) < .005):
                        paused = False
                        print(f"双臂就绪 · 灵敏度 {sensitivity:g}；握住对应 Grip 移动", flush=True)
                        last_message = None
                else:
                    try:
                        goals = cartesian.goals(frame, hw, kin, hands, sensitivity, freeze_buttons, now)
                        sides = tuple(goals)
                        if not goals:
                            outgoing = None
                            reason = ("grip_released" if all(frame[s]["grip"] <= .3 for s in ARMS)
                                      else "waiting_for_stationary_or_full_grip")
                            if reason != last_input_stop:
                                log.write(json.dumps({"local_time": time.time(), "event": reason,
                                    "input": {s: {k: v for k, v in frame[s].items() if k != "pose"} for s in ARMS}})+"\n")
                            last_input_stop = reason
                        else:
                            last_input_stop = None
                            sequence += 1
                            ik_started = time.monotonic()
                            outgoing = request(kin, hw, goals, sequence, status["stamp"], cartesian.references)
                            ik_ms = 1000*(time.monotonic()-ik_started)
                            validate(hw, outgoing, status["stamp"])
                            cartesian.accepted_plan(outgoing)
                            targets_rate.mark(now)
                            log.write(json.dumps({"local_time": time.time(), "plan_monotonic": now, "vr_stamp": frame["stamp"],
                                "sensitivity": sensitivity, "mapping": MAPPING_ID,
                                "mode": control_mode, "rotation_mode": rotation_mode, "controller_sides": hands,
                                "input": {s: {k: v for k, v in frame[s].items() if k != "pose"} for s in ARMS},
                                "robot_status_age_ms": 1000*(now-connection.last_status),
                                "compute_ms": 1000*(time.monotonic()-now),
                                "ik_ms": ik_ms, "cartesian_sampling": cartesian.buffer.last_metadata,
                                "pico_pc_sample_age_ms": 1000*(now-frame.get("received_monotonic", now)),
                                "freeze": {s: {"position": c.frozen_position, "rotation": c.frozen_rotation}
                                           for s, c in clutches.items()},
                                "tcp_chest_mm": {side: tcp_in_chest(side, goal).flatten().tolist()
                                                 for side, goal in goals.items()},
                                "request": outgoing}, allow_nan=False)+"\n")
                        last_sides = sides
                    except ValueError as exc:
                        paused = True
                        outgoing = None
                        last_sides = ()
                        cartesian.reset()
                        print(f"已停止：{exc}；请松开两侧 Grip 后重试", flush=True)
            else:
                paused = True
                outgoing = None
                cartesian.reset()
            connection.send(request=outgoing, kinematics=proof if frame else None)
            if now-last_display > 1.:
                sides = "/".join(last_sides) if outgoing else "保持"
                frozen = "; ".join(f"{s}:" + "/".join(name for name, active in
                    (("位移", c.frozen_position), ("旋转", c.frozen_rotation)) if active)
                    for s, c in clutches.items() if c.frozen_position or c.frozen_rotation)
                phase = "回 Home 中" if home_motion is not None else "等待回 Home" if home_pending else status.get('phase', '')
                rate_data = {"send_hz": targets_rate.hz(now) if outgoing else 0.,
                    "accepted_hz": status.get("target_rate_hz", 0.),
                    "feedback_hz": feedback_rate.hz(now), "missed_loop_slots": cadence.skipped,
                    "hardware_observed_hz": status.get("hardware_observed_hz", 0.)}
                log.write(json.dumps({"local_time": time.time(), "event": "rates", **rate_data})+"\n")
                print(f"{phase} | 灵敏度 {sensitivity:g} | {sides}"
                      + f" | 下发 {rate_data['send_hz']:.1f} / 接收 {rate_data['accepted_hz']:.1f} / 回传 {rate_data['feedback_hz']:.1f} Hz"
                      + f" / 硬件反馈 {rate_data['hardware_observed_hz']:.1f} Hz"
                      + (f" | 冻结 {frozen}" if frozen else ""), flush=True)
                last_display = now
            # Both target generation and heartbeat follow one absolute cadence.
            # A late iteration skips expired slots and samples the latest input.
            time.sleep(cadence.advance(time.monotonic()))


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("必须是有限正数")
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description="法奥双臂 PICO 相对遥操；默认只读检查")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="使能全部 20 轴，默认先回双臂 Home，然后握 Grip 遥操")
    mode.add_argument("--check", action="store_true", help="只读检查（默认）")
    mode.add_argument("--prepare-check", action="store_true", help="检查临时控制器接口，不使能、不运动")
    mode.add_argument("--link-check", action="store_true", help="通信压测：不启用 ROS 控制器、不使能、不运动")
    parser.add_argument("--duration", type=positive, default=10., help="通信压测持续秒数")
    parser.add_argument("--observe-ros", action="store_true", help="仅用于 --link-check：增加只读硬件状态订阅；不发布运动目标")
    add_timing_arguments(parser)
    parser.add_argument("--host", default="192.168.58.1")
    parser.add_argument("--user", default="fairino")
    parser.add_argument("--sensitivity", type=positive, default=.25, help="位移和旋转灵敏度")
    parser.add_argument("--mode", choices=tuple(POSE_FRAMES), default="first_person",
                        help="first_person 同向；mirror 面对面镜像；mirror_same_direction 面对面且保持房间方向")
    parser.add_argument("--rotation-mode", choices=("auto", *POSE_FRAMES), default="auto",
                        help="独立旋转映射；auto 在面对面模式使用镜像旋转，保持位移模式不变")
    parser.add_argument("--hand-mapping", choices=("auto", "same", "crossed"), default="auto",
                        help="手柄到手臂：auto 在面对面模式交换左右；same 同侧；crossed 交叉")
    parser.add_argument("--freeze-buttons", action="store_true",
                        help="A/X 按住冻结旋转，B/Y 按住冻结位移；灵敏度用左摇杆或 +/-，暂停用左摇杆按压或空格")
    parser.add_argument("--home-file", type=Path, default=DEFAULT_HOME, help="双臂 Home 七轴配置文件")
    parser.add_argument("--no-auto-home", action="store_true", help="本次启动保持姿态，不自动回 Home；右摇杆回 Home 仍可用")
    args = parser.parse_args(argv)
    if args.observe_ros and not args.link_check:
        parser.error("--observe-ros 只能与 --link-check 一起使用")
    try:
        Timing.from_args(args)
    except ValueError as exc:
        parser.error(str(exc))
    stream = connection = None
    code = 0
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        kin = VendorKinematics()
        if args.execute:
            stream = Stream()
            stream.open()
            wait_for_input(stream)
        logs = ROOT/".local/pico4/fairino-logs"
        logs.mkdir(parents=True, exist_ok=True)
        connection = Connection(args, worker_command=remote_command(args), log_directory=logs)
        if args.link_check:
            from vr_link_check import run_client
            code = run_client(args, kin, connection)
        else:
            code = operate(args, kin, stream, connection)
    except KeyboardInterrupt:
        print("正在结束通信压测。" if args.link_check else "正在退出并等待机器人确认下使能。", flush=True)
    except Exception as exc:
        print("停止："+str(exc), file=sys.stderr, flush=True)
        code = 1
    finally:
        if connection is not None:
            connection.finish()
            if args.execute:
                if connection.latest.get("disabled_confirmed"):
                    print("已确认上半身全部下使能。", flush=True)
                else:
                    print("未收到全部下使能确认，请现场检查。", file=sys.stderr, flush=True)
                    code = 1
        if stream is not None:
            stream.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
