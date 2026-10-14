#!/usr/bin/env python3
"""Desktop key-down/key-up control; the robot-side watchdog travels over SSH."""
import argparse
import base64
import getpass
import json
import math
import queue
import shlex
import signal
import sys
import threading
import time
import zlib
from pathlib import Path

from control_core import (KEYS, STEER, key_twist, check_speeds,
                          DEFAULT_LINEAR, DEFAULT_ANGULAR, MIN_SPEED,
                          MAX_LINEAR, MAX_ANGULAR)

HERE = Path(__file__).resolve().parent
PREFIX = "@KEYCTRL "


def remote_command(args):
    bundle = json.dumps({name: (HERE / name).read_text() for name in
                         ("control_core.py", "robot_worker.py")}).encode()
    encoded = base64.b64encode(zlib.compress(bundle)).decode()
    argv = ["robot_worker.py", "--linear", str(args.linear), "--angular", str(args.angular)]
    if args.check:
        argv.append("--check")
    if args.prepare_check:
        argv.append("--prepare-check")
    loader = ("import base64,zlib,json,sys,types;"
              f"bundle=json.loads(zlib.decompress(base64.b64decode({encoded!r})));"
              "m=types.ModuleType('control_core');sys.modules['control_core']=m;"
              "exec(compile(bundle['control_core.py'],'control_core.py','exec'),m.__dict__);"
              f"sys.argv={argv!r};"
              "exec(compile(bundle['robot_worker.py'],'robot_worker.py','exec'),{'__name__':'__main__'})")
    root = "/opt/fairino_AMR_project_B/reference_chassis_runtime/install"
    lines = ["set -e", "source /opt/ros/humble/setup.bash"]
    for pkg in ("hardware_state_broadcaster_msgs", "fairino_hardware_command_msgs"):
        lines.append(f"source {root}/{pkg}/share/{pkg}/local_setup.bash")
    lines += ["export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
              "export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fairino_AMR_project_B/config/fastrtps_udp.xml",
              "echo KEYCTRL_CONNECTED", "exec python3 -u -c " + shlex.quote(loader)]
    return "bash -c " + shlex.quote("\n".join(lines))


class Connection:
    def __init__(self, args):
        import pexpect
        self.pexpect = pexpect
        self.events = queue.Queue()
        self.latest = {}
        self.last_status = 0.0
        self.write_lock = threading.Lock()
        self.speeds = check_speeds(args.linear, args.angular)
        self.done = threading.Event()
        self.log = None
        self.child = pexpect.spawn("ssh", ["-T", "-o", "ConnectTimeout=5",
            "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
            "-o", "StrictHostKeyChecking=accept-new", f"{args.user}@{args.host}",
            remote_command(args)], encoding="utf-8", codec_errors="replace",
            timeout=12, maxread=65536, echo=False)
        self.child.delaybeforesend = None
        try:
            while True:
                result = self.child.expect(["KEYCTRL_CONNECTED", r"(?i)password:", pexpect.EOF])
                if result == 0:
                    break
                if result == 1:
                    self.child.sendline(getpass.getpass(f"SSH 密码（{args.user}@{args.host}）："))
                else:
                    raise RuntimeError("SSH 连接失败：" + self.child.before[-1500:])
        except BaseException:
            self.child.close(force=True)
            raise
        logs = HERE / "logs"
        logs.mkdir(exist_ok=True)
        self.log_path = logs / (time.strftime("%Y%m%d-%H%M%S") + ".jsonl")
        self.log = self.log_path.open("a", buffering=1)
        threading.Thread(target=self.reader, daemon=True).start()

    def reader(self):
        try:
            while True:
                try:
                    line = self.child.readline()
                except self.pexpect.TIMEOUT:
                    continue
                if not line:
                    break
                line = line.rstrip()
                if line.startswith(PREFIX):
                    try:
                        data = json.loads(line[len(PREFIX):])
                    except ValueError:
                        continue
                    if "stamp" in data:
                        self.latest = data
                        self.last_status = time.monotonic()
                    self.events.put(data)
                    self.log.write(json.dumps({"local_time": time.time(), **data}, ensure_ascii=False) + "\n")
                elif line:
                    self.log.write(json.dumps({"ros_log": line}, ensure_ascii=False) + "\n")
        except (self.pexpect.EOF, OSError, ValueError):
            pass
        finally:
            self.done.set()
            self.events.put({"connection_closed": True})

    def send(self, keys=(), focused=True, stop=False, speeds=None):
        if self.done.is_set():
            return
        # No replay after a delayed feedback stream: stop is latched robot-side.
        stale = self.last_status != 0 and time.monotonic() - self.last_status > 0.5
        linear, angular = check_speeds(*(self.speeds if speeds is None else speeds))
        packet = {"keys": sorted(keys), "focused": bool(focused),
                  "stop": bool(stop or stale), "ack": self.latest.get("stamp", 0),
                  "linear": linear, "angular": angular}
        with self.write_lock:
            try:
                self.child.sendline(json.dumps(packet))
            except (OSError, self.pexpect.EOF):
                self.done.set()

    def finish(self):
        self.send(stop=True)
        # Keep SSH alive while the remote worker tries zero + disable + cleanup.
        if not self.done.wait(20):
            print("远端尚未确认停止。请检查机器人并使用物理急停。", file=sys.stderr)
        if self.done.is_set():
            self.child.close()
            if self.log:
                self.log.close()


class Window:
    def __init__(self, args, connection=None):
        import gi
        gi.require_version("Gtk", "3.0")
        from gi.repository import Gtk, Gdk, GLib
        self.Gtk, self.Gdk, self.GLib = Gtk, Gdk, GLib
        self.args, self.connection = args, connection
        self.keys = set()
        self.held = set()
        self.ready = args.demo
        self.stopped = False
        self.closing = False
        self.had_focus = False
        self.speed_pending = False
        self.last_phase = "演示模式，不连接机器人" if args.demo else "正在读取底盘状态"
        self.window = Gtk.Window(title="法奥底盘键控 · WASD / JL")
        self.window.set_default_size(700, 620)
        self.window.set_border_width(22)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=15)
        self.window.add(box)
        title = Gtk.Label()
        title.set_markup('<span size="20000" weight="bold">底盘键控</span>')
        box.pack_start(title, False, False, 0)
        help_text = ("W 前进     S 后退     A 左横移     D 右横移\n"
                     "J 车身左转     L 车身右转\n"
                     "支持组合键；方向以机器人自身为准。松开按键停止。")
        help_label = Gtk.Label(label=help_text)
        help_label.set_justify(Gtk.Justification.CENTER)
        box.pack_start(help_label, False, False, 0)
        speed_grid = Gtk.Grid(column_spacing=12, row_spacing=8)
        self.speed_widgets = []
        self.speed_adjustments = []
        self.speed_spins = []
        self.setting_speed = False
        for row, (label, value, maximum) in enumerate((
                ("平移速度（m/s）", args.linear, MAX_LINEAR),
                ("旋转速度（rad/s）", args.angular, MAX_ANGULAR))):
            adjustment = Gtk.Adjustment(value=value, lower=MIN_SPEED, upper=maximum,
                                        step_increment=0.01, page_increment=0.05)
            slider = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL, adjustment=adjustment)
            slider.set_draw_value(False)
            slider.set_hexpand(True)
            slider.set_can_focus(False)
            spin = Gtk.SpinButton(adjustment=adjustment, climb_rate=0.01, digits=2)
            spin.set_numeric(True)
            spin.set_update_policy(Gtk.SpinButtonUpdatePolicy.IF_VALID)
            spin.set_width_chars(5)
            spin.connect("activate", self.resume_keyboard)
            speed_grid.attach(Gtk.Label(label=label), 0, row, 1, 1)
            speed_grid.attach(slider, 1, row, 1, 1)
            speed_grid.attach(spin, 2, row, 1, 1)
            self.speed_widgets.extend((slider, spin))
            self.speed_adjustments.append(adjustment)
            self.speed_spins.append(spin)
        box.pack_start(speed_grid, False, False, 0)
        self.speed_hint = Gtk.Label()
        box.pack_start(self.speed_hint, False, False, 0)
        self.control_focus = Gtk.Button(label="调速后点这里，或按回车，继续键盘控制")
        self.control_focus.connect("clicked", self.resume_keyboard)
        box.pack_start(self.control_focus, False, False, 0)
        self.phase = Gtk.Label(label=self.last_phase)
        self.phase.set_line_wrap(True)
        box.pack_start(self.phase, False, False, 0)
        self.motion = Gtk.Label(label="vx 0.000 m/s     vy 0.000 m/s     转速 0.000 rad/s")
        box.pack_start(self.motion, False, False, 0)
        self.wheels = Gtk.Label(label="等待轮组反馈")
        self.wheels.set_justify(Gtk.Justification.LEFT)
        box.pack_start(self.wheels, False, False, 0)
        self.error = Gtk.Label(label="")
        self.error.set_line_wrap(True)
        box.pack_start(self.error, False, False, 0)
        stop_button = Gtk.Button(label="停止并下使能  ·  空格 / Esc")
        stop_button.set_can_focus(False)
        stop_button.connect("clicked", lambda *_: self.stop())
        box.pack_start(stop_button, False, False, 0)
        footer = Gtk.Label(label=("松开运动键后可调速；再次按键按新速度运动。\n"
                                "启动自动回正；失焦、断线或故障会停止本次控制。"))
        box.pack_start(footer, False, False, 0)
        for adjustment in self.speed_adjustments:
            adjustment.connect("value-changed", self.speed_changed)
        self.update_speed_hint()
        self.window.connect("key-press-event", self.key_press)
        self.window.connect("key-release-event", self.key_release)
        self.window.connect("focus-in-event", self.focus_in)
        self.window.connect("focus-out-event", self.focus_out)
        self.window.connect("delete-event", self.close)
        self.window.show_all()
        self.control_focus.grab_focus()
        self.window.present()
        GLib.timeout_add(50, self.tick)

    def update_speed_hint(self):
        self.speed_hint.set_text(
            f"平移 {self.args.linear * 100:.0f} 厘米/秒  ·  "
            f"旋转 {math.degrees(self.args.angular):.1f}°/秒"
            + ("  ·  等待机器人确认速度" if self.speed_pending else ""))

    def speed_changed(self, *_):
        if self.setting_speed:
            return
        if self.keys or self.held or self.stopped:
            self.setting_speed = True
            try:
                for adjustment, value in zip(self.speed_adjustments,
                                             (self.args.linear, self.args.angular)):
                    adjustment.set_value(value)
            finally:
                self.setting_speed = False
            return
        self.args.linear, self.args.angular = check_speeds(
            *(a.get_value() for a in self.speed_adjustments))
        self.speed_pending = self.connection is not None
        self.update_speed_hint()
        self.send()

    def resume_keyboard(self, *_):
        for spin in self.speed_spins:
            spin.update()
        self.keys.clear()
        self.control_focus.grab_focus()
        self.send()

    def key_name(self, event):
        return (self.Gdk.keyval_name(event.keyval) or "").lower()

    def key_press(self, _, event):
        key = self.key_name(event)
        if key in ("space", "escape"):
            self.stop()
            return True
        if key in KEYS:
            if isinstance(self.window.get_focus(), self.Gtk.Entry):
                return False
            first_press = key not in self.held
            self.held.add(key)
            if (first_press and self.ready and not self.stopped and not self.speed_pending
                    and self.window.is_active()):
                self.keys.add(key)
                self.send()
            return True
        return False

    def key_release(self, _, event):
        key = self.key_name(event)
        self.keys.discard(key)
        self.held.discard(key)
        if key in KEYS:
            self.send()
            return True
        return False

    def focus_in(self, *_):
        self.had_focus = True
        self.keys.clear()
        self.held.clear()
        return False

    def focus_out(self, *_):
        self.keys.clear()
        if self.had_focus:
            self.stop("窗口失去焦点，正在停止")
        return False

    def send(self):
        for widget in self.speed_widgets:
            widget.set_sensitive(not (self.keys or self.held or self.stopped or self.closing))
        if self.connection:
            self.connection.send(self.keys if self.ready and not self.speed_pending else (),
                                 focused=self.window.is_active(), stop=self.stopped,
                                 speeds=(self.args.linear, self.args.angular))

    def stop(self, text="正在停止并下使能"):
        self.keys.clear()
        self.stopped = True
        self.ready = False
        self.last_phase = text
        self.phase.set_text(text)
        self.send()

    def tick(self):
        if self.closing:
            return False
        if self.connection:
            while True:
                try:
                    data = self.connection.events.get_nowait()
                except queue.Empty:
                    break
                if "error" in data:
                    self.error.set_text(data["error"])
                    self.stop("停止：请查看错误信息")
                if "phase" in data:
                    if (data.get("linear_speed"), data.get("angular_speed")) == (self.args.linear, self.args.angular):
                        self.speed_pending = False
                        self.update_speed_hint()
                    was_ready = self.ready
                    self.ready = data.get("ready", False) and not self.stopped
                    if not was_ready or not self.ready:
                        self.keys.clear()
                    self.last_phase = data["phase"]
                    self.phase.set_text(self.last_phase)
                    hw = data.get("hardware")
                    if hw:
                        angle = [math.degrees(hw["position"][i]) for i in STEER]
                        # Display in physical front/rear order, not joint index order.
                        self.wheels.set_text(
                            f"左前 J5 {angle[2]:6.1f}°     右前 J1 {angle[0]:6.1f}°\n"
                            f"左后 J7 {angle[3]:6.1f}°     右后 J3 {angle[1]:6.1f}°\n"
                            f"使能 {sum(hw['enabled'])}/8     故障 0x{hw['error']:08X}")
                if data.get("connection_closed"):
                    self.ready = False
                    self.keys.clear()
                    self.stopped = True
                    if not self.connection.latest.get("disabled_confirmed"):
                        self.phase.set_text("连接已关闭；没有收到下使能确认，请检查机器人")
            self.send()
        vx, vy, wz = key_twist(self.keys, self.args.linear, self.args.angular)
        self.motion.set_text(f"vx {vx:+.3f} m/s     vy {vy:+.3f} m/s     转速 {wz:+.3f} rad/s")
        return True

    def close(self, *_):
        if self.closing:
            return True
        self.stop("正在停止，等待远端下使能结果")
        self.closing = True
        if self.connection:
            def finish():
                self.connection.finish()
                self.GLib.idle_add(self.Gtk.main_quit)
            threading.Thread(target=finish, daemon=True).start()
        else:
            self.Gtk.main_quit()
        return True

    def run(self):
        self.Gtk.main()


def main():
    parser = argparse.ArgumentParser(description="法奥四舵轮键控；正常启动会使转向轮回正，再开放键控。")
    parser.add_argument("--host", default="192.168.58.3")
    parser.add_argument("--user", default="fairino")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="仅读取状态，不使能、不运动")
    mode.add_argument("--prepare-check", action="store_true", help="临时配置并验证控制器，电机始终不使能")
    mode.add_argument("--demo", action="store_true", help="离线窗口，检查按键映射")
    parser.add_argument("--linear", type=float, default=DEFAULT_LINEAR, help="初始平移速度 m/s，范围 0.01–0.20")
    parser.add_argument("--angular", type=float, default=DEFAULT_ANGULAR, help="初始旋转速度 rad/s，范围 0.01–0.40")
    args = parser.parse_args()
    if args.host.startswith("-") or args.user.startswith("-") or "@" in args.host or "@" in args.user:
        parser.error("无效的 SSH 主机或用户名")
    try:
        check_speeds(args.linear, args.angular)
    except RuntimeError as exc:
        parser.error(str(exc))
    if args.demo:
        Window(args).run()
        return 0
    if args.check:
        print("只读检查，不使能、不回正。")
    elif args.prepare_check:
        print("临时配置并验证控制器，电机始终不使能；完成后卸载临时控制器。")
    else:
        print("启动后四个转向轮将缓慢回正，再使能底盘。请保持轮组周围空旷、充电线拔下，并让控制窗口保持焦点。")
    connection = Connection(args)
    print("记录：", connection.log_path)
    if args.check or args.prepare_check:
        passed = False
        failed = False
        deadline = time.monotonic() + 50
        while not connection.done.is_set() and time.monotonic() < deadline:
            connection.send()
            time.sleep(0.05)
        while not connection.events.empty():
            data = connection.events.get_nowait()
            if "error" in data:
                failed = True
                print("检查失败：", data["error"])
            if data.get("checked"):
                passed = True
                print(json.dumps(data, ensure_ascii=False, indent=2))
        connection.finish()
        return 0 if passed and not failed else 1
    try:
        window = Window(args, connection)
        signal.signal(signal.SIGINT, lambda *_: window.close())
        signal.signal(signal.SIGTERM, lambda *_: window.close())
        window.run()
    finally:
        if not connection.done.is_set():
            connection.finish()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, OSError) as exc:
        print("启动失败：", exc, file=sys.stderr)
        sys.exit(1)
