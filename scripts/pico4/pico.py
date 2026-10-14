"""Local PICO setup/diagnostics. This module never constructs a robot driver."""
from __future__ import annotations

import argparse
import importlib
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
RUNTIME = ROOT / ".local/pico4"
ADB = RUNTIME / "platform-tools/adb"


def service_directory():
    matches = list((RUNTIME / "service").rglob("RoboticsServiceProcess"))
    if len(matches) != 1:
        raise RuntimeError("接收服务尚未准备好，请运行 run.sh setup。")
    return matches[0].parent


def service_ready():
    try:
        with socket.create_connection(("127.0.0.1", 60061), timeout=0.3):
            return True
    except OSError:
        return False


def service_environment(directory):
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = ":".join(str(directory / p) for p in (".", "lib", "SDK/x64"))
    env["QT_PLUGIN_PATH"] = str(directory / "plugins")
    env["QT_QML_PATH"] = str(directory / "qml")
    env["QML2_IMPORT_PATH"] = str(directory / "qml")
    env["XDG_DATA_HOME"] = str(RUNTIME / "data")
    env["QT_QPA_PLATFORM"] = "offscreen"
    return env


def valid_pose(pose):
    return (len(pose) == 7 and all(math.isfinite(x) for x in pose)
            and 0.9 < math.sqrt(sum(x*x for x in pose[3:])) < 1.1)


class Freshness:
    """Use local monotonic time; headset and PC wall clocks need not match."""
    def __init__(self):
        self.stamp = None
        self.changed = None

    def update(self, stamp, now):
        if stamp > 0 and stamp != self.stamp:
            self.stamp, self.changed = stamp, now
        return self.changed is not None and now - self.changed < 0.5


def check():
    ok = True
    print("环境检查（不会连接或使能机器人）")
    for module in ("numpy", "draccus", "spdlog", "xensevr_pc_service_sdk",
                   "lerobot.teleoperators.bi_pico4"):
        try:
            loaded = importlib.import_module(module)
            print(f"OK  {module}: {loaded.__file__}")
        except Exception as exc:
            ok = False
            print(f"缺少/不可加载  {module}: {exc}")
    try:
        directory = service_directory()
        result = subprocess.run(["ldd", str(directory / "RoboticsServiceProcess")],
                                env=service_environment(directory), capture_output=True,
                                text=True, timeout=15)
        missing = [line.strip() for line in result.stdout.splitlines() if "not found" in line]
        if result.returncode or missing:
            raise RuntimeError(f"PC Service 动态库未就绪：{missing or result.stderr}")
        print("OK  PC Service 及动态库:", directory)
    except RuntimeError as exc:
        ok = False
        print(exc)
    print("服务端口 60061:", "已监听" if service_ready() else "未启动（运行 run.sh service）")
    if ADB.exists():
        subprocess.run([str(ADB), "devices", "-l"], check=True, timeout=15)
    else:
        ok = False
        print("ADB 尚未安装。")
    subprocess.run(["ip", "-brief", "-4", "addr"], check=False)
    print("PICO 与电脑应在同一局域网；连接电脑的 Wi-Fi IP，勿填机器人 IP。")
    print("本包包含 VR 接收和法奥适配；尚未提供完整 LeRobot 数据集录制流程。")
    return 0 if ok else 1


def service():
    if service_ready():
        raise RuntimeError("端口 60061 已监听；请先使用 monitor 检查，不重复启动接收服务。")
    directory = service_directory()
    # This is the data service, not the optional Qt device-management window.
    env = service_environment(directory)
    print("启动 PC 接收服务；保持此终端运行，Ctrl+C 停止。", flush=True)
    os.chdir(directory)
    os.execve(str(directory / "RoboticsServiceProcess"), ["RoboticsServiceProcess"], env)


def build_preview(sdk, mode="first_person", sensitivity=0.25):
    import numpy as np
    from lerobot.teleoperators.bi_pico4 import BiPico4, BiPico4Config
    teleop = BiPico4(BiPico4Config(
        control_mode=mode, pos_sensitivity=sensitivity, ori_sensitivity=sensitivity,
        max_pos_velocity=0.05, max_rot_velocity=0.2,
    ))
    # Seed a fictitious origin, never a robot state. The SDK is already initialized.
    seed = np.array([0., 0., 0., 1., 0., 0., 0., 1.])
    teleop._xrt = sdk
    teleop.connect(left_tcp_pose_quat=seed, right_tcp_pose_quat=seed)
    return teleop


def monitor(args):
    if not service_ready():
        raise RuntimeError("PC Service 未启动。先在另一个终端运行 run.sh service。")
    import xensevr_pc_service_sdk as sdk
    clock = Freshness()
    start = last_print = time.monotonic()
    preview = None
    had_data = False
    initializing = True
    sdk.init()
    print("等待头显发送 Controller 数据；Ctrl+C 退出。", flush=True)
    if args.command == "preview":
        print("只预览相对目标，基准为虚拟零点；不连接机器人。按住 Grip 移动，松开保持。", flush=True)
    try:
        while args.duration <= 0 or time.monotonic() - start < args.duration:
            now = time.monotonic()
            left, right = sdk.get_left_controller_pose(), sdk.get_right_controller_pose()
            stamp = sdk.get_time_stamp_ns()
            fresh = clock.update(stamp, now)
            usable = fresh and valid_pose(left) and valid_pose(right)
            had_data |= usable
            action = None
            if usable and args.command == "preview":
                if initializing:
                    # Start/restart preview only after both grips are released.
                    if sdk.get_left_grip() <= 0.3 and sdk.get_right_grip() <= 0.3:
                        if preview is None:
                            preview = build_preview(sdk, args.mode, args.sensitivity)
                        else:
                            for arm in (preview._left_pico4, preview._right_pico4):
                                arm.resume_motion_after_grip_release()
                        initializing = False
                if preview is not None and not initializing:
                    action = preview.get_action()
            elif not usable:
                if not initializing and preview is not None:
                    preview.suspend_arm_motion("left")
                    preview.suspend_arm_motion("right")
                initializing = True
            if now - last_print >= 0.5:
                last_print = now
                if not usable:
                    print("等待/数据流超时：检查头显 WORKING、Controller Tracking 和 Send。", flush=True)
                elif args.command == "preview" and action is None:
                    print("请先松开左右 Grip，建立预览起点。", flush=True)
                elif action is not None:
                    values = []
                    for side in ("left", "right"):
                        pos = [round(action[f"{side}_tcp.{a}"] * 1000, 2) for a in "xyz"]
                        rot = [round(action[f"{side}_tcp.r{i}"], 3) for i in range(1, 7)]
                        values.append(f"{side}: 虚拟目标 mm={pos}  旋转6D={rot}")
                    print(" | ".join(values), flush=True)
                else:
                    print(f"数据流更新  L xyz(m)={[round(v,4) for v in left[:3]]} "
                          f"Grip={sdk.get_left_grip():.2f} Trigger={sdk.get_left_trigger():.2f} | "
                          f"R xyz(m)={[round(v,4) for v in right[:3]]} "
                          f"Grip={sdk.get_right_grip():.2f} Trigger={sdk.get_right_trigger():.2f}", flush=True)
            time.sleep(1 / 60)
    finally:
        # Disconnect clears BiPico4's destructor state, preventing a second SDK close.
        if preview is not None and preview.is_connected:
            preview.disconnect()
        else:
            sdk.close()
    if not had_data:
        print("本次未收到有效的双手柄数据。", flush=True)
        return 2
    return 0


def install_apk(args):
    if not ADB.exists():
        raise RuntimeError("请先运行 run.sh setup 安装 ADB。")
    result = subprocess.check_output([str(ADB), "devices"], text=True, timeout=15)
    devices = [line.split()[0] for line in result.splitlines()[1:] if line.endswith("\tdevice")]
    serial = args.serial
    if serial is None:
        if len(devices) != 1:
            print(result)
            raise RuntimeError("需要恰好一台已授权 USB 调试的头显，或用 --serial 指定。")
        serial = devices[0]
    if serial not in devices:
        raise RuntimeError("指定设备未连接或尚未在头显内允许 USB 调试。")
    base = [str(ADB), "-s", serial]
    model = subprocess.check_output(base + ["shell", "getprop", "ro.product.model"], text=True).strip()
    brand = subprocess.check_output(base + ["shell", "getprop", "ro.product.manufacturer"], text=True).strip()
    api = int(subprocess.check_output(base + ["shell", "getprop", "ro.build.version.sdk"], text=True).strip())
    if "pico" not in (model + " " + brand).lower():
        raise RuntimeError(f"设备 {model} / {brand} 未识别为 PICO，未安装 APK。")
    if api < 29:
        raise RuntimeError(f"头显 Android API {api} 低于已准备 APK 的最低要求 29。")
    variant = ("standard" if api >= 30 else "sdk29") if args.variant == "auto" else args.variant
    if variant == "standard" and api < 30:
        raise RuntimeError("标准 APK 需要 Android API 30；此头显请使用 --variant sdk29。")
    suffix = "-SDK29" if variant == "sdk29" else ""
    apk = RUNTIME / "downloads" / f"XRoboToolkit-PICO-1.1.1{suffix}.apk"
    print(f"PICO {model}，Android API {api}；安装 {apk.name}", flush=True)
    # No -r: do not overwrite an existing headset app/data silently.
    subprocess.run(base + ["install", "-g", str(apk)], check=True)


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("必须是有限正数")
    return result


def main():
    parser = argparse.ArgumentParser(description="PICO 接收与相对遥操检查（无机器人控制）")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("check", help="检查环境、ADB 和服务端口")
    sub.add_parser("service", help="前台启动 PC 接收服务")
    for name in ("monitor", "preview"):
        p = sub.add_parser(name, help="查看原始数据" if name == "monitor" else "预览现有 BiPico4 相对映射")
        p.add_argument("--duration", type=positive, default=0, help="运行秒数，默认持续运行")
        p.add_argument("--mode", choices=("first_person", "mirror", "mirror_same_direction"), default="first_person")
        p.add_argument("--sensitivity", type=positive, default=0.25)
    p = sub.add_parser("install-apk", help="向已连接的 PICO 安装官方 APK")
    p.add_argument("--serial")
    p.add_argument("--variant", choices=("auto", "sdk29", "standard"), default="auto")
    args = parser.parse_args()
    try:
        if args.command == "check":
            return check()
        if args.command == "service":
            return service()
        if args.command == "install-apk":
            return install_apk(args)
        return monitor(args)
    except KeyboardInterrupt:
        print("\n已停止。")
        return 130
    except (RuntimeError, ImportError, OSError, subprocess.SubprocessError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
