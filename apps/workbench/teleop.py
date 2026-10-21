"""Teleoperation panel; launches the same high-rate client as `run.sh vr`."""
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib
from apps.gripper_control.gripper_core import TELEOP_SPEED
from apps.fairino_tcp_probe.tool_frames import load_geometry, tool_config
from scripts.pico4.fairino.home_presets import HOME_PRESETS
from apps.fairino_upper_probe.wrist_guard_ui import GuardControls
from apps.fairino_keyboard.control_core import (DEFAULT_LINEAR, DEFAULT_ANGULAR,
                                                MIN_SPEED, MAX_LINEAR, MAX_ANGULAR)

ROOT = Path(__file__).resolve().parents[2]


class TeleopPanel(Gtk.Box):
    def __init__(self, credentials, acquire, release, demo=False):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        self.credentials, self.acquire, self.release = credentials, acquire, release
        self.demo = demo
        self.process = self.service = None
        self.events = queue.Queue()
        self.closed = False
        self.upper_lease = self.gripper_lease = self.base_lease = False
        self.tool_preview = None
        self.gripper_feedback = {}
        self.settings_scroll = Gtk.ScrolledWindow()
        self.settings_scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        self.settings_scroll.set_overlay_scrolling(False)
        self.settings_scroll.set_min_content_height(300)
        self.settings_scroll.set_propagate_natural_height(False)
        settings = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        settings.set_margin_end(8)
        self.settings_scroll.add(settings)
        self.pack_start(self.settings_scroll, True, True, 0)
        settings.pack_start(Gtk.Label(label='PICO 遥操 · 双臂 / 夹爪 / 身体 / 底盘', xalign=0), False, False, 0)
        text = ('右手柄 → 左臂，左手柄 → 右臂；握 Grip 跟随，松开保持。\n'
                '按住 A/X 冻结旋转、B/Y 冻结位移；双击 X 降低、双击 Y 提高灵敏度。\n'
                '左杆 XY 平移、右杆 X 旋转；右杆 Y 升降。左杆按下暂停，右杆按下回 Home。\n'
                '底盘与升降需分别勾选；450 Hz 手臂控制由独立进程执行。\n'
                '先启动接收服务，在头显选择电脑 IP 并打开 Controller / Send。')
        label = Gtk.Label(label=text, xalign=0)
        label.set_line_wrap(True)
        settings.pack_start(label, False, False, 0)
        self.wrist_controls = GuardControls(self.switch_wrist)
        settings.pack_start(self.wrist_controls, False, False, 0)
        self.home = Gtk.CheckButton(label='启动后先回双臂 Home（会运动）；不勾选则保持当前姿态')
        self.home.set_active(True)
        settings.pack_start(self.home, False, False, 0)
        home_row = Gtk.Box(spacing=8)
        home_row.pack_start(Gtk.Label(label='双臂起始位姿'), False, False, 0)
        self.home_preset = Gtk.ComboBoxText()
        for name, (label, _) in HOME_PRESETS.items():
            self.home_preset.append(name, label)
        self.home_preset.set_active_id('default')
        home_row.pack_start(self.home_preset, False, False, 0)
        home_row.pack_start(Gtk.Label(label='自动回 Home 和右摇杆回 Home 均使用此选择'), False, False, 0)
        settings.pack_start(home_row, False, False, 0)
        tool_row = Gtk.Box(spacing=8)
        tool_row.pack_start(Gtk.Label(label='遥操 TCP'), False, False, 0)
        self.tcp_tool = Gtk.ComboBoxText()
        self.tcp_tool.append('flange', '法兰中心')
        self.tcp_tool.append('taccap', '带夹爪 · 固定指尖中心')
        self.tcp_tool.set_active_id('flange')
        tool_row.pack_start(self.tcp_tool, False, False, 0)
        self.tool_settings = Gtk.Button(label='安装角度 / TCP 设置与预览')
        self.tool_settings.connect('clicked', self.configure_tools)
        tool_row.pack_start(self.tool_settings, False, False, 0)
        settings.pack_start(tool_row, False, False, 0)
        self.grippers = Gtk.CheckButton(label='同时使能 TacCap 夹爪遥操 · Trigger 连续控制开度')
        self.grippers.set_active(False)
        settings.pack_start(self.grippers, False, False, 0)
        self.tcp_tool.connect('changed', self.tool_changed)
        gripper_row = Gtk.Box(spacing=8)
        self.gripper_serials = {}
        for side, title in [('left', '左爪 SN'), ('right', '右爪 SN')]:
            gripper_row.pack_start(Gtk.Label(label=title), False, False, 0)
            entry = Gtk.Entry()
            entry.set_placeholder_text('留空按 SDK 左右识别')
            entry.set_width_chars(23)
            self.gripper_serials[side] = entry
            gripper_row.pack_start(entry, False, False, 0)
        settings.pack_start(gripper_row, False, False, 0)
        speed_row = Gtk.Box(spacing=8)
        speed_row.pack_start(Gtk.Label(label='夹爪速度（电机轴 rad/s）'), False, False, 0)
        self.gripper_speed = Gtk.SpinButton.new_with_range(.01, 1., .05)
        self.gripper_speed.set_digits(2)
        self.gripper_speed.set_value(TELEOP_SPEED)
        speed_row.pack_start(self.gripper_speed, False, False, 0)
        speed_row.pack_start(Gtk.Label(label='启动时生效；0.75 约为原速度的 3 倍，力矩不变'), False, False, 0)
        settings.pack_start(speed_row, False, False, 0)
        limits_row = Gtk.Box(spacing=8)
        self.gripper_limits = {}
        for side, title in [('left', '左爪最大开度 %'), ('right', '右爪最大开度 %')]:
            limits_row.pack_start(Gtk.Label(label=title), False, False, 0)
            value = Gtk.SpinButton.new_with_range(0., 100., 1.)
            value.set_digits(1)
            value.set_value(100.)
            self.gripper_limits[side] = value
            limits_row.pack_start(value, False, False, 0)
        settings.pack_start(limits_row, False, False, 0)
        gripper_help = Gtk.Label(label='控制某臂的手柄 Trigger → 对应夹爪；松开到设定上限、压到底闭合，中间线性映射。\n'
            '100% 为各爪标定全开；限制在启动时生效，反馈中显示对应电机转角（不是两指夹角）。\n'
            '首次启动与暂停后先保持，松开 Trigger 再按压开始；Grip 仍用于手臂跟随。', xalign=0)
        gripper_help.set_line_wrap(True)
        settings.pack_start(gripper_help, False, False, 0)
        self.body_lift = Gtk.CheckButton(label='启用身体竖直升降 · 右摇杆上下，回中保持（沿用高低位标定）')
        self.body_lift.set_active(False)
        settings.pack_start(self.body_lift, False, False, 0)
        self.chassis = Gtk.CheckButton(label='同时启动底盘 · 回正并使能')
        self.chassis.set_active(False)
        settings.pack_start(self.chassis, False, False, 0)
        chassis_row = Gtk.Box(spacing=8)
        self.chassis_linear = Gtk.SpinButton.new_with_range(MIN_SPEED, MAX_LINEAR, .01)
        self.chassis_angular = Gtk.SpinButton.new_with_range(MIN_SPEED, MAX_ANGULAR, .01)
        for title, widget, default in [('底盘平移上限（m/s）', self.chassis_linear, DEFAULT_LINEAR),
                                       ('旋转上限（rad/s）', self.chassis_angular, DEFAULT_ANGULAR)]:
            widget.set_digits(2)
            widget.set_value(default)
            widget.set_sensitive(False)
            chassis_row.pack_start(Gtk.Label(label=title), False, False, 0)
            chassis_row.pack_start(widget, False, False, 0)
        self.chassis.connect('toggled', lambda *_: self.update_options())
        settings.pack_start(chassis_row, False, False, 0)
        chassis_help = Gtk.Label(label='底盘速度按摇杆幅度连续映射；启动和暂停后先回中，回中保持使能。\n'
            '使用顶部底盘 IP 与共用 SSH 凭据；不能同时使用底盘键控。', xalign=0)
        chassis_help.set_line_wrap(True)
        settings.pack_start(chassis_help, False, False, 0)
        row = Gtk.Box(spacing=8)
        self.stop_button = None
        for title, callback in [('启动 PICO 接收服务', self.start_service), ('启动遥操 · 使能上半身', self.start),
                                ('停止遥操并下使能', lambda *_: self.command('stop'))]:
            button = Gtk.Button(label=title)
            button.connect('clicked', callback)
            if title == '停止遥操并下使能':
                self.stop_button = button
                button.set_sensitive(False)
            row.pack_start(button, False, False, 0)
        self.pack_start(row, False, False, 0)
        commands = Gtk.Box(spacing=8)
        self.command_buttons = []
        for command, title in [('pause', '暂停 / 保持'), ('home', '回 Home / 中止 Home'), ('slower', '降低灵敏度'), ('faster', '提高灵敏度')]:
            button = Gtk.Button(label=title)
            button.connect('clicked', lambda _, c=command: self.command(c))
            self.command_buttons.append(button)
            button.set_sensitive(False)
            commands.pack_start(button, False, False, 0)
        self.pack_start(commands, False, False, 0)
        self.status = Gtk.Label(label='遥操未启动', xalign=0)
        self.status.set_line_wrap(True)
        self.pack_start(self.status, False, False, 0)
        self.gripper_status = Gtk.Label(label='夹爪遥操未启用', xalign=0)
        self.gripper_status.set_line_wrap(True)
        self.pack_start(self.gripper_status, False, False, 0)
        self.chassis_status = Gtk.Label(label='底盘遥操未启用', xalign=0)
        self.chassis_status.set_line_wrap(True)
        self.pack_start(self.chassis_status, False, False, 0)
        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        scroll.set_min_content_height(80)
        scroll.set_propagate_natural_height(False)
        self.log = Gtk.TextView(editable=False, cursor_visible=False, wrap_mode=Gtk.WrapMode.WORD_CHAR)
        scroll.add(self.log)
        self.pack_start(scroll, False, True, 0)
        GLib.timeout_add(100, self.tick)

    def tool_changed(self, *_):
        if self.tcp_tool.get_active_id() == 'taccap':
            self.grippers.set_active(True)

    def switch_wrist(self, enabled):
        if self.process is not None and self.process.poll() is None:
            self.command('guard_on' if enabled else 'guard_off')
            self.wrist_controls.update({}, pending=True)
        else:
            self.wrist_controls.update({})

    def configure_tools(self, *_):
        from apps.workbench.tool_settings import ToolSettings
        parent = self.get_toplevel()
        parent = parent if isinstance(parent, Gtk.Window) else None
        try:
            ToolSettings(parent, self.tool_preview).edit()
        except (ValueError, OSError) as exc:
            self.append(str(exc))

    def append(self, message):
        buffer = self.log.get_buffer()
        buffer.insert(buffer.get_end_iter(), message + '\n')
        if buffer.get_line_count() > 250:
            buffer.delete(buffer.get_start_iter(), buffer.get_iter_at_line(50))
        self.log.scroll_to_iter(buffer.get_end_iter(), 0, False, 0, 0)

    def reader(self, process, name):
        try:
            for line in process.stdout:
                self.events.put((name, line.rstrip()))
        finally:
            process.wait()
            self.events.put((name + '_exit', process.returncode))

    def start_service(self, *_):
        if self.demo:
            self.append('离线演示：未启动接收服务')
            return
        if self.service and self.service.poll() is None:
            self.append('本窗口启动的接收服务正在运行')
            return
        try:
            self.service = subprocess.Popen(['bash', str(ROOT/'run.sh'), 'pico', 'service'], cwd=ROOT,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
            threading.Thread(target=self.reader, args=(self.service, 'service'), daemon=True).start()
        except OSError as exc:
            self.append(str(exc))

    def start(self, *_):
        if self.demo:
            self.append('离线演示：未连接、未使能')
            return
        if self.closed or (self.process and self.process.poll() is None):
            return
        if self.upper_lease or self.gripper_lease or self.base_lease:
            self.append('上一会话正在退出，等待控制占用释放后再启动')
            return
        try:
            mode = self.tcp_tool.get_active_id()
            tool_config(mode, load_geometry() if mode == 'taccap' else None, require_confirmed=True)
        except (ValueError, OSError) as exc:
            self.append(str(exc))
            return
        credentials = self.credentials('upper')
        if credentials is None:
            return
        host, user, password = credentials
        try:
            base_credentials = self.credentials('base') if self.chassis.get_active() else None
            if self.chassis.get_active():
                if base_credentials is None:
                    return
                if base_credentials[1:] != credentials[1:]:
                    raise ValueError('集成底盘遥操使用顶部共用 SSH 用户和密码；底盘与上半身凭据必须一致')
            parts = ['upper']
            if self.grippers.get_active():
                parts.append('gripper')
            if self.chassis.get_active():
                parts.append('base')
            for part in parts:
                if not self.acquire(part, '遥操'):
                    self.release_leases()
                    return
                setattr(self, part+'_lease', True)
            argv = ['bash', str(ROOT/'run.sh'), 'vr', '--gui-stdio', '--host', host, '--user', user]
            argv.extend(['--tcp-tool', self.tcp_tool.get_active_id()])
            argv.extend(['--home-preset', self.home_preset.get_active_id()])
            if not self.wrist_controls.toggle.get_active():
                argv.append('--no-wrist-guard')
            if not self.home.get_active():
                argv.append('--no-auto-home')
            if self.grippers.get_active():
                argv.append('--grippers')
                argv.extend(['--gripper-speed-rad-s', str(self.gripper_speed.get_value())])
                for side in ('left', 'right'):
                    argv.extend([f'--{side}-gripper-sn', self.gripper_serials[side].get_text().strip()])
                    argv.extend([f'--{side}-gripper-max-open-percent', str(self.gripper_limits[side].get_value())])
            if self.body_lift.get_active():
                argv.append('--body-lift')
            if self.chassis.get_active():
                argv.extend(['--chassis', '--chassis-host', base_credentials[0],
                             '--chassis-linear', str(self.chassis_linear.get_value()),
                             '--chassis-angular', str(self.chassis_angular.get_value())])
            self.process = subprocess.Popen(argv, cwd=ROOT, stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
            self.process.stdin.write(json.dumps({'password': password}) + '\n')
            self.process.stdin.flush()
            self.status.set_text('等待双手柄新鲜数据及 Grip 释放…')
            self.chassis_status.set_text('等待底盘回正并使能；请保持摇杆回中' if self.chassis.get_active()
                                         else '底盘遥操未启用')
            self.update_options()
            threading.Thread(target=self.reader, args=(self.process, 'vr'), daemon=True).start()
        except (OSError, ValueError, RuntimeError) as exc:
            try:
                if self.process and self.process.poll() is None:
                    self.process.terminate()
            except OSError as stop_error:
                self.append(f'遥操进程退出请求失败：{stop_error}')
            finally:
                self.release_leases()
            self.append(str(exc))

    def release_leases(self):
        for part in ('upper', 'gripper', 'base'):
            name = part+'_lease'
            if getattr(self, name):
                self.release(part, '遥操')
                setattr(self, name, False)

    def update_options(self):
        running = self.process is not None and self.process.poll() is None
        editable = not running and not self.closed
        for widget in (self.home, self.home_preset, self.grippers, self.tcp_tool,
                       self.tool_settings, self.body_lift, self.chassis, *self.gripper_serials.values()):
            widget.set_sensitive(editable)
        for widget in (self.gripper_speed, *self.gripper_limits.values()):
            widget.set_sensitive(editable and self.grippers.get_active())
        for widget in (self.chassis_linear, self.chassis_angular):
            widget.set_sensitive(editable and self.chassis.get_active())

    def command(self, command):
        if self.process is None or self.process.poll() is not None:
            return
        try:
            self.process.stdin.write(json.dumps({'command': command}) + '\n')
            self.process.stdin.flush()
            if command == 'stop':
                self.status.set_text('正在等待遥操退出并确认下使能…')
        except (OSError, ValueError):
            self.status.set_text('遥操命令管道已关闭，等待退出结果')

    def tick(self):
        while not self.events.empty():
            name, data = self.events.get_nowait()
            if name == 'vr_exit':
                self.gripper_feedback = {}
                self.release_leases()
                self.status.set_text(f'遥操已退出，返回码 {data}；下使能结果见下方日志')
                self.chassis_status.set_text('底盘遥操已结束；下使能结果见下方日志' if self.chassis.get_active()
                                             else '底盘遥操未启用')
                if self.process:
                    self.process.stdin.close()
            elif name == 'vr' and data.startswith('WORKBENCH_GRIPPERS '):
                try:
                    self.gripper_feedback = json.loads(data[len('WORKBENCH_GRIPPERS '):])
                except (ValueError, TypeError):
                    self.gripper_feedback = {}
            elif name == 'vr' and data.startswith('WORKBENCH_VR '):
                try:
                    state = json.loads(data[len('WORKBENCH_VR '):])
                    self.wrist_controls.update(state.get('wrist_guard', {}),
                                               state.get('wrist_pending', True))
                    if state.get('wrist_limited'):
                        self.wrist_controls.label.set_text(
                            self.wrist_controls.label.get_text()+' · 接近边界，已缩小目标；可向内收腕')
                    grips = state.get('grips', {})
                    self.status.set_text(f'{state["phase"]} · {state["sides"]} · 灵敏度 {state["sensitivity"]:g}\n'
                        + ('TCP：夹爪固定指尖中心 · ' if state.get('tcp_tool') == 'taccap' else 'TCP：法兰中心 · ')+
                        f'Grip 左 {grips.get("left", 0):.2f} / 右 {grips.get("right", 0):.2f} · '
                        f'使能 {state["enabled"]}/20 · 下发 {state["send_hz"]:.0f} / 接收 {state["accepted_hz"]:.0f} Hz')
                    grippers = state.get('grippers')
                    self.gripper_feedback = grippers or {}
                    if grippers:
                        units = grippers.get('devices', {})
                        values = []
                        for side, label in [('left', '左爪'), ('right', '右爪')]:
                            unit = units.get(side)
                            text = f'{label} {unit["position"]*100:.1f}% / {unit["state"]}' if unit else f'{label}未连接'
                            if unit and unit.get('open_span_rad'):
                                maximum = grippers.get('max_openings', {}).get(side, 1.)
                                text += f' · 映射上限 {maximum*100:g}% ≈ {math.degrees(maximum*unit["open_span_rad"]):.1f}° 电机转角'
                            values.append(text)
                        self.gripper_status.set_text(grippers.get('message', '')+
                            f' · 速度 {grippers.get("speed_rad_s", 0.):g} rad/s\n'+' · '.join(values))
                    else:
                        self.gripper_status.set_text('夹爪遥操未启用')
                    chassis = state.get('chassis')
                    if chassis:
                        vx, vy, yaw = chassis.get('command', (0., 0., 0.))
                        self.chassis_status.set_text(
                            f'底盘：{chassis.get("phase", "等待反馈")} · 使能 {chassis.get("enabled", 0)}/8\n'
                            f'指令 vx {vx:+.3f} / vy {vy:+.3f} m/s · 旋转 {yaw:+.3f} rad/s\n'
                            + (f'故障：{chassis["error"]}' if chassis.get('error') else chassis.get('message', '')))
                    else:
                        self.chassis_status.set_text('等待底盘状态' if self.base_lease else '底盘遥操未启用')
                    if state.get('body_lift_enabled'):
                        body = state.get('body')
                        self.gripper_status.set_text(self.gripper_status.get_text()+'\n'+(
                            f'身体 {body["height"]:.1f}% / 相对低位 {body["height_mm"]:.1f} mm · '
                            f'右摇杆 {state.get("lift_axis", 0.):+.2f}' if body else '等待身体高度反馈'))
                except (ValueError, KeyError, TypeError):
                    self.append('遥操状态格式异常')
            else:
                self.append(f'[{name}] {data}')
        running = self.process is not None and self.process.poll() is None
        self.update_options()
        self.stop_button.set_sensitive(running)
        self.wrist_controls.toggle.set_sensitive(not self.closed)
        for button in self.command_buttons:
            button.set_sensitive(running)
        return not self.closed or running

    def close(self):
        self.command('stop')
        self.closed = True
        if self.service and self.service.poll() is None:
            os.killpg(self.service.pid, signal.SIGTERM)

    def finished(self):
        return all(p is None or p.poll() is not None for p in (self.process, self.service))
