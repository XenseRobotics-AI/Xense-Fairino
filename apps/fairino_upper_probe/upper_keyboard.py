#!/usr/bin/env python3
"""GTK window for single-joint jogging of both arms, head and body."""
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
from probe_core import (JOINT_GROUPS, SPEED_DEG_S, MIN_SPEED_DEG_S, MAX_SPEED_DEG_S,
                        TARGET_SPEED_DEG_S, MAX_TARGET_SPEED_DEG_S, step_duration, target_duration)

HERE = Path(__file__).resolve().parent
PREFIX = "@UPPERPROBE "
GROUP_LABELS = {"left": "左臂 · J1–J7", "right": "右臂 · J8–J14",
                "body": "头部 / 躯干 · J15–J20"}


def remote_command(args):
    bundle = {n: (HERE/n).read_text() for n in ("probe_core.py", "robot_probe.py")}
    encoded = base64.b64encode(zlib.compress(json.dumps(bundle).encode())).decode()
    argv = ["robot_probe.py"]
    if args.check:
        argv.append("--check")
    if args.prepare_check:
        argv.append("--prepare-check")
    loader = ("import base64,zlib,json,sys,types;"
              f"bundle=json.loads(zlib.decompress(base64.b64decode({encoded!r})));"
              "m=types.ModuleType('probe_core');sys.modules['probe_core']=m;"
              "exec(compile(bundle['probe_core.py'],'probe_core.py','exec'),m.__dict__);"
              f"sys.argv={argv!r};"
              "exec(compile(bundle['robot_probe.py'],'robot_probe.py','exec'),{'__name__':'__main__'})")
    setup = "\n".join([
        "set -e", "source /opt/ros/humble/setup.bash",
        "source /opt/fairino_art_plugin/plugin/install/setup.bash",
        "export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
        "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/fairino/fastrtps_udp.xml",
        "echo UPPERPROBE_CONNECTED", "exec python3 -u -c " + shlex.quote(loader)])
    return "bash -c " + shlex.quote(setup)


class Connection:
    def __init__(self, args, worker_command=None, log_directory=None):
        import pexpect
        self.pexpect = pexpect
        self.events = queue.Queue()
        self.latest = {}
        self.last_status = 0.0
        self.done = threading.Event()
        self.write_lock = threading.Lock()
        self.child = pexpect.spawn("ssh", ["-T", "-o", "ConnectTimeout=5",
            "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
            "-o", "StrictHostKeyChecking=accept-new", f"{args.user}@{args.host}",
            remote_command(args) if worker_command is None else worker_command], encoding="utf-8", codec_errors="replace",
            echo=False, timeout=12, maxread=65536)
        self.child.delaybeforesend = None
        try:
            while True:
                match = self.child.expect(["UPPERPROBE_CONNECTED", r"(?i)password:", pexpect.EOF])
                if match == 0:
                    break
                if match == 1:
                    self.child.sendline(getpass.getpass(f"SSH 密码（{args.user}@{args.host}）："))
                else:
                    raise RuntimeError("SSH 连接失败："+self.child.before[-1200:])
        except BaseException:
            self.child.close(force=True)
            raise
        logs = HERE/"logs" if log_directory is None else Path(log_directory)
        logs.mkdir(exist_ok=True)
        self.log_path = logs/(time.strftime("%Y%m%d-%H%M%S")+".jsonl")
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
                    data = json.loads(line[len(PREFIX):])
                    data["client_received_monotonic"] = time.monotonic()
                    if "stamp" in data:
                        self.latest, self.last_status = data, data["client_received_monotonic"]
                    self.events.put(data)
                    self.log.write(json.dumps({"local_time": time.time(), **data}, ensure_ascii=False)+"\n")
                elif line:
                    self.log.write(json.dumps({"ros_log": line}, ensure_ascii=False)+"\n")
        except (OSError, ValueError, self.pexpect.EOF):
            pass
        finally:
            self.done.set()
            self.events.put({"connection_closed": True})

    def send(self, request=None, cancel=0, focused=True, exit=False, kinematics=None):
        if self.done.is_set():
            return
        stale = self.last_status != 0 and time.monotonic()-self.last_status > 0.5
        data = {"request": request, "cancel": cancel, "focused": focused,
                "exit": bool(exit or stale), "ack": self.latest.get("stamp", 0)}
        if kinematics is not None:
            data["kinematics"] = kinematics
        with self.write_lock:
            try:
                data["client_send_monotonic"] = time.monotonic()
                self.child.sendline(json.dumps(data))
            except (OSError, ValueError, self.pexpect.EOF):
                self.done.set()

    def finish(self):
        self.send(exit=True)
        if not self.done.wait(60):
            print("尚未收到下使能确认，请检查机器人并使用物理急停。", file=sys.stderr)
        if self.done.is_set():
            self.child.close()
            self.log.close()


class Window:
    def __init__(self, args, connection=None):
        import gi
        gi.require_version("Gtk", "3.0")
        from gi.repository import Gtk, Gdk, GLib
        self.Gtk, self.Gdk, self.GLib = Gtk, Gdk, GLib
        self.connection, self.args = connection, args
        self.group = getattr(args, "group", "body")
        self.selected = JOINT_GROUPS[self.group][0]
        self.sequence = 0
        self.request = None
        self.cancel_count = 0
        self.held = set()
        self.ready = args.demo
        self.pending = False
        self.stopped = self.closing = self.had_focus = False
        self.last_hw = None
        self.window = Gtk.Window(title="上半身关节调节 · 点动 / 目标角度")
        self.window.set_default_size(1000, 750)
        self.window.set_border_width(20)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        self.window.add(box)
        title = Gtk.Label()
        title.set_markup('<span size="18000" weight="bold">双臂与上半身关节调节</span>')
        box.pack_start(title, False, False, 0)
        help_label = Gtk.Label(label=("切换下方部位标签；数字 1–7 选择当前组关节   ·   ← 减小角度   → 增大角度\n"
            "方向键每次点动 0.5°；也可输入绝对目标角度执行。空格中止运动，Esc 退出。"))
        box.pack_start(help_label, False, False, 0)
        speed_row = Gtk.Box(spacing=10)
        speed_row.pack_start(Gtk.Label(label="点动最高速度（°/s）"), False, False, 0)
        self.speed = Gtk.SpinButton.new_with_range(MIN_SPEED_DEG_S, MAX_SPEED_DEG_S, 0.1)
        self.speed.set_digits(1)
        self.speed.set_width_chars(7)
        self.speed.set_max_width_chars(9)
        self.speed.set_numeric(True)
        self.speed.set_snap_to_ticks(True)
        self.speed.set_value(SPEED_DEG_S)
        self.speed.set_tooltip_text("至少 0.1°/s，无软件速度上限；调整从下一次点动生效。")
        speed_row.pack_start(self.speed, False, False, 0)
        self.speed_hint = Gtk.Label()
        speed_row.pack_start(self.speed_hint, False, False, 0)
        self.speed.connect("value-changed", self.speed_changed)
        self.speed_changed()
        box.pack_start(speed_row, False, False, 0)
        self.phase = Gtk.Label(label="离线演示，不连接机器人" if args.demo else "正在连接上半身")
        self.phase.set_line_wrap(True)
        box.pack_start(self.phase, False, False, 0)
        self.enable_status = Gtk.Label(label="使能状态：等待反馈" if not args.demo else "离线演示：不会使能")
        box.pack_start(self.enable_status, False, False, 0)
        self.notebook = Gtk.Notebook()
        self.notebook.set_can_focus(False)
        box.pack_start(self.notebook, True, True, 0)
        self.buttons, self.angles, self.deltas, self.notes = {}, {}, {}, {}
        previous = None
        saved = {}
        try:
            saved = json.loads((HERE/"joint_mapping.json").read_text()).get("joints", {})
        except (OSError, ValueError, TypeError):
            pass
        self.group_names = list(JOINT_GROUPS)
        for group, joints in JOINT_GROUPS.items():
            grid = Gtk.Grid(column_spacing=18, row_spacing=8)
            grid.set_border_width(10)
            self.notebook.append_page(grid, Gtk.Label(label=GROUP_LABELS[group]))
            for col, name in enumerate(["选择", "实际角度", "相对启动", "观察到的部位 / 正方向"]):
                grid.attach(Gtk.Label(label=name), col, 0, 1, 1)
            for row, joint in enumerate(joints, 1):
                button = Gtk.RadioButton.new_with_label_from_widget(previous, f"{row}  ·  J{joint}")
                previous = button
                button.set_can_focus(False)
                button.connect("toggled", self.select, joint)
                grid.attach(button, 0, row, 1, 1)
                self.buttons[joint] = button
                angle = Gtk.Label(label="—"); delta = Gtk.Label(label="—")
                grid.attach(angle, 1, row, 1, 1); grid.attach(delta, 2, row, 1, 1)
                self.angles[joint], self.deltas[joint] = angle, delta
                entry = Gtk.Entry()
                entry.set_placeholder_text("记录实际运动部位及角度增大时的方向")
                entry.set_text(saved.get(f"J{joint}", {}).get("observation", ""))
                entry.set_hexpand(True)
                entry.connect("focus-in-event", self.edit_note)
                grid.attach(entry, 3, row, 1, 1)
                self.notes[joint] = entry
        self.selected = JOINT_GROUPS[self.group][0]
        self.buttons[self.selected].set_active(True)
        target_row = Gtk.Box(spacing=10)
        self.target_joint = Gtk.Label(label=f"J{self.selected} 目标角度（°）")
        target_row.pack_start(self.target_joint, False, False, 0)
        self.target_angle = Gtk.Entry()
        self.target_angle.set_width_chars(12)
        self.target_angle.set_placeholder_text("例如 -70.0")
        self.target_angle.set_tooltip_text("控制器反馈坐标下的绝对角度；输入本身不发送运动。")
        target_row.pack_start(self.target_angle, True, True, 0)
        self.use_current = Gtk.Button(label="填入当前角度")
        self.use_current.set_can_focus(False)
        self.use_current.connect("clicked", self.copy_current_angle)
        target_row.pack_start(self.use_current, False, False, 0)
        target_row.pack_start(Gtk.Label(label="最高速度（°/s）"), False, False, 0)
        self.target_speed = Gtk.SpinButton.new_with_range(MIN_SPEED_DEG_S, MAX_TARGET_SPEED_DEG_S, 0.1)
        self.target_speed.set_digits(1)
        self.target_speed.set_width_chars(7)
        self.target_speed.set_max_width_chars(9)
        self.target_speed.set_numeric(True)
        self.target_speed.set_value(TARGET_SPEED_DEG_S)
        self.target_speed.set_tooltip_text("默认 3°/s，无软件速度上限；根据所填速度规划轨迹。")
        target_row.pack_start(self.target_speed, False, False, 0)
        self.move_button = Gtk.Button(label="执行到目标")
        self.move_button.set_can_focus(False)
        self.move_button.connect("clicked", self.move_to_target)
        target_row.pack_start(self.move_button, False, False, 0)
        box.pack_start(target_row, False, False, 0)
        self.target_hint = Gtk.Label()
        self.target_hint.set_line_wrap(True)
        box.pack_start(self.target_hint, False, False, 0)
        self.target_angle.connect("changed", self.target_changed)
        self.target_speed.connect("value-changed", self.target_changed)
        self.target_changed()
        self.motion_controls()
        controls = Gtk.Box(spacing=12)
        box.pack_start(controls, False, False, 0)
        self.control_area = Gtk.Button(label="点击这里恢复方向键控制")
        self.control_area.connect("clicked", lambda *_: self.control_area.grab_focus())
        controls.pack_start(self.control_area, True, True, 0)
        save = Gtk.Button(label="保存辨认记录")
        save.set_can_focus(False); save.connect("clicked", self.save_notes)
        controls.pack_start(save, False, False, 0)
        stop = Gtk.Button(label="中止运动，保持使能 · 空格")
        stop.set_can_focus(False); stop.connect("clicked", lambda *_: self.cancel())
        controls.pack_start(stop, False, False, 0)
        self.error = Gtk.Label(label="")
        self.error.set_line_wrap(True)
        box.pack_start(self.error, False, False, 0)
        box.pack_start(Gtk.Label(label="启动使能全部 J1–J20；一次只移动所选关节，其他 19 轴保持目标位置。\n"
            "速度上限、过程跟踪误差及实测超速退出已取消；退出、失焦或故障时全部下使能。"), False, False, 0)
        self.window.connect("key-press-event", self.key_press)
        self.window.connect("key-release-event", self.key_release)
        self.window.connect("focus-in-event", self.focus_in)
        self.window.connect("focus-out-event", self.focus_out)
        self.window.connect("delete-event", self.close)
        self.window.show_all()
        self.notebook.set_current_page(self.group_names.index(self.group))
        self.notebook.connect("switch-page", self.switch_group)
        self.control_area.grab_focus(); self.window.present()
        GLib.timeout_add(50, self.tick)

    def switch_group(self, notebook, page, page_number):
        group = self.group_names[page_number]
        if self.pending or group == self.group:
            return
        self.group = group
        self.buttons[JOINT_GROUPS[group][0]].set_active(True)
        self.control_area.grab_focus()

    def select(self, button, joint):
        if button.get_active():
            if self.pending and joint != self.selected:
                self.buttons[self.selected].set_active(True)
                return
            previous = self.selected
            self.selected = joint
            if hasattr(self, "target_angle"):
                self.target_joint.set_text(f"J{joint} 目标角度（°）")
                if previous != joint:
                    self.target_angle.set_text("")
                self.target_changed()

    def edit_note(self, *_):
        if self.pending or not self.ready:
            self.cancel()
        return False

    def speed_changed(self, *_):
        duration = step_duration(self.speed.get_value())+0.1
        self.speed_hint.set_text(f"每步约 {duration:.2f} 秒 · 下一次点动生效")

    def motion_controls(self):
        editable = not self.pending and not self.stopped
        self.speed.set_sensitive(editable)
        self.notebook.set_sensitive(not self.pending)
        for button in self.buttons.values():
            button.set_sensitive(not self.pending)
        for widget in (self.target_angle, self.target_speed, self.use_current):
            widget.set_sensitive(editable)
        self.move_button.set_sensitive(editable and self.ready)

    def target_changed(self, *_):
        current = (math.degrees(self.last_hw["position"][self.selected-1])
                   if self.last_hw else None)
        prefix = f"当前 {current:+.3f}°；" if current is not None else "等待实际角度；"
        try:
            target = float(self.target_angle.get_text().strip())
            if not math.isfinite(target):
                raise ValueError()
            if current is None:
                text = prefix + "输入的是绝对角度，点击执行才会移动。"
            else:
                duration = target_duration(target-current, self.target_speed.get_value()) + .1
                text = prefix + f"目标 {target:+.3f}°；变化 {target-current:+.3f}°；预计 {duration:.2f} 秒。"
        except (ValueError, OverflowError):
            text = prefix + "请输入有限的目标角度（°），可使用负数。"
        self.target_hint.set_text(text)

    def copy_current_angle(self, *_):
        if self.last_hw and not self.pending and not self.stopped:
            self.target_angle.set_text(f"{math.degrees(self.last_hw['position'][self.selected-1]):.4f}")
        elif not self.last_hw:
            self.error.set_text("尚无实际关节角度反馈")

    def move_to_target(self, *_):
        if not self.ready or self.pending or self.stopped:
            return
        if not self.args.demo and (self.last_hw is None
                or time.monotonic()-self.connection.last_status > .4):
            self.error.set_text("关节反馈缺失或过期，未发送目标")
            return
        try:
            target = float(self.target_angle.get_text().strip())
            if not math.isfinite(target):
                raise ValueError("目标角度必须是有限数值")
            self.target_speed.update()
            speed = self.target_speed.get_value()
            current = math.degrees(self.last_hw["position"][self.selected-1]) if self.last_hw else 0.
            target_duration(target-current, speed)
        except (ValueError, OverflowError) as exc:
            self.error.set_text("未发送目标：" + (str(exc) or "请输入有效角度"))
            return
        self.error.set_text("")
        self.sequence += 1
        self.request = {"id": self.sequence, "kind": "target", "joint": self.selected,
                        "target_deg": target, "speed_deg_s": speed,
                        "created": self.connection.latest.get("stamp", 0) if self.connection else time.monotonic()}
        if self.args.demo:
            self.phase.set_text(f"演示：J{self.selected} → {target:+.3f}° · 最高 {speed:.1f}°/s（未连接机器人）")
        else:
            self.pending = True
            self.phase.set_text(f"已请求 J{self.selected} → {target:+.3f}°，等待反馈")
        self.motion_controls()
        self.send()

    def key_press(self, _, event):
        key = (self.Gdk.keyval_name(event.keyval) or "").lower()
        if key == "escape":
            self.close(); return True
        if key == "space" and self.pending:
            self.cancel(); return True
        # Arrow keys / digits typed into observations must never command motion.
        if isinstance(self.window.get_focus(), self.Gtk.Entry):
            if key in ("left", "right"):
                self.held.add(key)
            return False
        if key == "space":
            self.cancel(); return True
        if key in "1234567" and len(key) == 1:
            if not self.pending:
                joints = JOINT_GROUPS[self.group]
                if int(key) <= len(joints):
                    self.buttons[joints[int(key)-1]].set_active(True)
            return True
        if key in ("left", "right"):
            fresh = key not in self.held
            self.held.add(key)
            if fresh and self.ready and not self.pending and not self.stopped and self.window.is_active():
                if len(self.held & {"left", "right"}) == 1:
                    self.step(-1 if key == "left" else 1)
            return True
        return False

    def key_release(self, _, event):
        key = (self.Gdk.keyval_name(event.keyval) or "").lower()
        self.held.discard(key)
        return key in ("left", "right")

    def step(self, direction):
        if not self.ready or self.pending or self.stopped:
            return
        self.speed.update()
        speed = self.speed.get_value()
        step_duration(speed)  # Validate before sending a motion request.
        self.sequence += 1
        self.request = {"id": self.sequence, "joint": self.selected, "direction": direction,
                        "speed_deg_s": speed,
                        "created": self.connection.latest.get("stamp", 0) if self.connection else time.monotonic()}
        if self.args.demo:
            self.phase.set_text(f"演示：J{self.selected} {'+' if direction > 0 else '-'}0.5° · {speed:.1f}°/s（未连接机器人）")
        else:
            self.pending = True
            self.phase.set_text(f"已请求 J{self.selected} 单步移动，等待反馈")
        self.motion_controls()
        self.send()

    def cancel(self):
        self.cancel_count += 1
        self.request = None
        self.phase.set_text("正在中止运动，保持使能；使能阶段按空格则取消启动" if not self.args.demo else "演示：已中止")
        self.send()

    def focus_in(self, *_):
        self.had_focus = True
        self.held.clear()
        return False

    def focus_out(self, *_):
        self.held.clear()
        if self.had_focus:
            self.stopped = True
            self.ready = False
            self.phase.set_text("窗口失焦，正在结束控制并下使能")
            self.send()
        return False

    def send(self):
        if self.connection:
            self.connection.send(self.request, self.cancel_count,
                                 focused=self.window.is_active(), exit=self.stopped)

    def save_notes(self, *_):
        path = HERE/"joint_mapping.json"
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError):
            data = {"joints": {}}
        data.update(robot="192.168.58.1", saved_at=time.strftime("%Y-%m-%d %H:%M:%S"))
        data.setdefault("joints", {})
        for joint, entry in self.notes.items():
            record = data["joints"].setdefault(f"J{joint}", {})
            record["observation"] = entry.get_text().strip()
            if self.last_hw:
                record["last_angle_deg"] = math.degrees(self.last_hw["position"][joint-1])
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2)+"\n")
        tmp.replace(path)
        self.error.set_text("记录已保存："+str(path))

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
                    self.stopped = True; self.ready = False
                if "phase" in data:
                    self.phase.set_text(data["phase"])
                    self.ready = data.get("ready", False) and not self.stopped
                    if data.get("last_request", 0) >= self.sequence and self.ready:
                        self.pending = False
                    hw = data.get("hardware")
                    baseline = data.get("baseline")
                    if hw:
                        self.last_hw = hw
                        protection = "实测超速退出开启" if data.get("speed_protection") else "实测超速退出关闭"
                        tracking = "过程跟踪误差检查开启" if data.get("path_tracking_check", True) else "过程跟踪误差检查关闭"
                        self.enable_status.set_text(f"上半身使能 {sum(hw['enabled'])}/20 · {protection} · {tracking}")
                        for joint in self.angles:
                            angle = math.degrees(hw["position"][joint-1])
                            enabled = " ON" if hw["enabled"][joint-1] else " OFF"
                            self.angles[joint].set_text(f"{angle:+.2f}°{enabled}")
                            if baseline:
                                delta = math.degrees(hw["position"][joint-1]-baseline[joint-1])
                                self.deltas[joint].set_text(f"{delta:+.2f}°")
                if data.get("connection_closed"):
                    self.stopped = True; self.ready = False
                    if not self.connection.latest.get("disabled_confirmed"):
                        self.error.set_text("连接已关闭，未收到最终下使能确认，请检查机器人")
            self.motion_controls()
            self.target_changed()
            self.send()
        return True

    def close(self, *_):
        if self.closing:
            return True
        self.closing = self.stopped = True
        self.ready = False
        self.phase.set_text("正在退出，等待下使能确认")
        self.send()
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
    parser = argparse.ArgumentParser(description="启动使能全部上半身 J1–J20；逐关节点动或以最高 3°/s 移动到绝对目标角度。")
    parser.add_argument("--host", default="192.168.58.1")
    parser.add_argument("--user", default="fairino")
    parser.add_argument("--group", choices=tuple(JOINT_GROUPS), default="body",
                        help="启动显示的部位：left 左臂、right 右臂、body 头部/躯干")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="只读真实状态")
    mode.add_argument("--prepare-check", action="store_true", help="电机关闭时验证控制器接口")
    mode.add_argument("--demo", action="store_true", help="离线演示窗口")
    args = parser.parse_args()
    if any(s.startswith("-") or "@" in s for s in (args.host, args.user)):
        parser.error("无效的 SSH 主机或用户名")
    if args.demo:
        Window(args).run(); return 0
    if args.check or args.prepare_check:
        print("检查模式：不使能、不发送点动轨迹。")
    else:
        print("连接上半身 .1；窗口就绪后会自动使能全部 J1–J20 并保持当前位置。")
        print("过程跟踪误差检查、实测超速退出已关闭；Esc/退出时全部下使能。")
        print("选轴后输入绝对目标角度，点击执行；速度默认 3°/s，无软件上限，空格中止并保持。")
    connection = Connection(args)
    print("日志：", connection.log_path)
    if args.check or args.prepare_check:
        passed = failed = False
        end = time.monotonic()+45
        while not connection.done.is_set() and time.monotonic() < end:
            connection.send(); time.sleep(0.05)
        while not connection.events.empty():
            data = connection.events.get_nowait()
            if "error" in data:
                failed = True; print("失败：", data["error"])
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
    except (OSError, RuntimeError) as exc:
        print("启动失败：", exc, file=sys.stderr)
        sys.exit(1)
