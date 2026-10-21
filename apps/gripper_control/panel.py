"""Reusable GTK gripper debug panel; no device access until Connect is clicked."""
import time

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib

from apps.gripper_control.client import Session
from apps.gripper_control.gripper_core import DEFAULT_SPEED, DEFAULT_TORQUE, SIDES


class GripperPanel(Gtk.Box):
    def __init__(self, credentials, acquire, release, demo=False):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        self.credentials, self.acquire, self.release, self.demo = credentials, acquire, release, demo
        self.session = None
        self.closing = self.owned = False
        self.note = Gtk.Label(label='TacCap / XTac 夹爪 · 扫描只读，使能后可独立调试开度与闭合。', xalign=0)
        self.pack_start(self.note, False, False, 0)
        row = Gtk.Box(spacing=8)
        self.serials = {}
        for side, title in [('left', '左爪 SN'), ('right', '右爪 SN')]:
            row.pack_start(Gtk.Label(label=title), False, False, 0)
            entry = Gtk.Entry()
            entry.set_placeholder_text('留空按 SDK 左右识别')
            entry.set_width_chars(23)
            row.pack_start(entry, False, False, 0)
            self.serials[side] = entry
        self.pack_start(row, False, False, 0)
        row = Gtk.Box(spacing=8)
        self.connect_button = Gtk.Button(label='扫描 / 连接（只读）')
        self.connect_button.connect('clicked', self.connect_device)
        row.pack_start(self.connect_button, False, False, 0)
        self.disconnect_button = Gtk.Button(label='断开夹爪 · 下使能')
        self.disconnect_button.connect('clicked', self.disconnect)
        self.disconnect_button.set_sensitive(False)
        row.pack_start(self.disconnect_button, False, False, 0)
        self.pack_start(row, False, False, 0)
        self.state = Gtk.Label(label='夹爪未连接', xalign=0)
        self.state.set_line_wrap(True)
        self.state.set_selectable(True)
        self.pack_start(self.state, False, False, 0)
        self.widgets = {}
        columns = Gtk.Box(spacing=15, homogeneous=True)
        for side, title in [('left', '机器人左夹爪'), ('right', '机器人右夹爪')]:
            frame = Gtk.Frame(label=title)
            box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
            box.set_border_width(12)
            frame.add(box)
            info = Gtk.Label(label='未连接', xalign=0)
            info.set_line_wrap(True)
            info.set_max_width_chars(48)
            info.set_size_request(-1, 110)
            info.set_selectable(True)
            box.pack_start(info, False, False, 0)
            value = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 0, 100, 1)
            value.set_digits(1)
            value.set_value(50)
            value.set_hexpand(True)
            box.pack_start(Gtk.Label(label='目标开度（0% 闭合 / 100% 张开）', xalign=0), False, False, 0)
            box.pack_start(value, False, False, 0)
            settings = Gtk.Box(spacing=8)
            torque = Gtk.SpinButton.new_with_range(.01, 5., .05)
            torque.set_digits(2); torque.set_value(DEFAULT_TORQUE)
            speed = Gtk.SpinButton.new_with_range(.01, 1., .01)
            speed.set_digits(2); speed.set_value(DEFAULT_SPEED)
            for label, widget in [('力矩 Nm', torque), ('速度 rad/s', speed)]:
                settings.pack_start(Gtk.Label(label=label), False, False, 0)
                settings.pack_start(widget, False, False, 0)
            box.pack_start(settings, False, False, 0)
            buttons = {}
            grid = Gtk.Grid(column_spacing=8, row_spacing=8)
            for index, (action, label) in enumerate([
                    ('enable', '使能 · 保持当前位置'), ('target', '执行目标开度'),
                    ('open', '张开'), ('close', '闭合'), ('hold', '暂停 / 保持'), ('disable', '下使能')]):
                button = Gtk.Button(label=label)
                button.connect('clicked', lambda _, s=side, a=action: self.action(s, a))
                grid.attach(button, index % 2, index // 2, 1, 1)
                buttons[action] = button
                button.set_sensitive(False)
            box.pack_start(grid, False, False, 0)
            diagnostic_expander = Gtk.Expander(label='诊断详情 · 固件 / 故障 / 力矩包络')
            diagnostics = Gtk.Label(label='连接后显示只读诊断', xalign=0)
            diagnostics.set_line_wrap(True)
            diagnostics.set_selectable(True)
            diagnostics.set_max_width_chars(48)
            diagnostic_expander.add(diagnostics)
            box.pack_start(diagnostic_expander, False, False, 0)
            columns.pack_start(frame, True, True, 0)
            self.widgets[side] = dict(info=info, target=value, torque=torque, speed=speed, buttons=buttons,
                                      diagnostics=diagnostics, diagnostic_expander=diagnostic_expander)
            for widget in (value, torque, speed):
                widget.set_sensitive(False)
        self.pack_start(columns, False, False, 0)
        self.pack_start(Gtk.Label(label='拖动滑条只修改目标，点击“执行目标开度”才运动。\n'
            '力矩是电机 Nm，不是夹持力 N；参数更改在下一次使能时生效。', xalign=0), False, False, 0)
        GLib.timeout_add(100, self.tick)

    def connect_device(self, *_):
        if self.demo:
            self.state.set_text('离线预览：不连接设备、不使能；控件布局始终可见。')
            return
        if self.session and not self.session.finished():
            return
        credentials = self.credentials('upper')
        if credentials is None or not self.acquire('gripper', '夹爪调试'):
            return
        self.owned = True
        self.session = Session(*credentials, left_sn=self.serials['left'].get_text().strip(),
                               right_sn=self.serials['right'].get_text().strip()).start()
        self.state.set_text('正在扫描上半身 USB 夹爪，仅查询身份与状态…')

    def action(self, side, action):
        if not self.session or self.session.finished():
            return
        widgets = self.widgets[side]
        fields = {}
        if action == 'enable':
            fields = dict(torque=widgets['torque'].get_value(), speed=widgets['speed'].get_value())
        if action in ('target', 'open', 'close'):
            value = {'open': 100., 'close': 0.}.get(action, widgets['target'].get_value())
            widgets['target'].set_value(value)
            action, fields = 'target', dict(opening=value/100.)
        self.session.command(action, side, **fields)

    def disconnect(self, *_):
        if self.session:
            self.session.close()

    def tick(self):
        state = self.session.snapshot() if self.session else {}
        running = self.session is not None and not self.session.finished()
        if self.owned and not running:
            self.release('gripper', '夹爪调试')
            self.owned = False
        fresh = (running and not state.get('error') and not self.session.stop.is_set()
                 and time.monotonic()-state.get('received', 0) < .5)
        self.connect_button.set_sensitive(not running and not self.closing)
        self.disconnect_button.set_sensitive(running and not self.session.stop.is_set())
        for entry in self.serials.values():
            entry.set_sensitive(not running and not self.closing)
        if self.session:
            message = state.get('error') or state.get('message')
            if not running:
                result = '已确认本会话夹爪下使能' if self.session.disabled_confirmed else '未收到下使能确认，请检查设备'
                message = (message+'；' if message else '')+result
            elif self.session.stop.is_set():
                message = (message+'；' if message else '')+'正在断开，等待夹爪下使能确认…'
            elif state.get('busy') and not message:
                action = state.get('pending_action') or '操作'
                message = f'正在等待夹爪{action}确认…'
            elif fresh and not message:
                blocked = []
                for side, title in [('left', '左夹爪'), ('right', '右夹爪')]:
                    issues = state.get('devices', {}).get(side, {}).get('diagnostics', {}).get('issues', [])
                    if issues:
                        blocked.append(title+'使能检查未通过：\n'+'\n'.join(issues))
                message = '\n'.join(blocked) if blocked else '连接就绪；未连接的一侧保持禁用。'
            if message:
                self.state.set_text(message)
        for side in SIDES:
            widgets = self.widgets[side]
            unit = state.get('devices', {}).get(side, {})
            connected = bool(fresh and unit.get('connected'))
            controlled = connected and unit.get('controlled', False)
            if unit:
                widgets['info'].set_text(f'SN {unit["sn"]}\n{unit["port"]}\n'
                    f'实测开度 {100*unit["position"]:.1f}% · {unit["state"]}\n'
                    f'使能 {"是" if unit["enabled"] else "否"} · 力矩 {unit["torque_nm"]:+.2f} Nm · '
                    f'温度 {unit["temperature_c"]:.1f}°C'+('' if fresh else '\n反馈未更新'))
                diagnostic = unit.get('diagnostics', {})
                issues = diagnostic.get('issues', [])
                if diagnostic:
                    widgets['diagnostics'].set_text('固件：'+diagnostic.get('firmware', '未知')+'\n'+(
                        '\n'.join(issues) if issues else '使能前故障与包络检查通过；使能时仍会重新读取复查。'))
                    widgets['diagnostic_expander'].set_label(
                        f'诊断详情 · {len(issues)} 项使能检查未通过' if issues else '诊断详情 · 固件 / 故障 / 力矩包络')
                else:
                    widgets['diagnostics'].set_text('诊断数据尚未更新，请重新连接夹爪。')
            else:
                widgets['info'].set_text(state.get('device_errors', {}).get(side, '未连接'))
            for key in ('target', 'torque', 'speed'):
                widgets[key].set_sensitive(connected and (key == 'target' or not unit.get('enabled')))
            busy = state.get('busy', False)
            for action, button in widgets['buttons'].items():
                sensitive = connected and not busy and not self.closing
                sensitive = sensitive and (not unit.get('enabled') if action == 'enable' else controlled)
                button.set_sensitive(bool(sensitive))
        return not self.closing or running

    def close(self):
        self.closing = True
        self.disconnect()

    def finished(self):
        return self.session is None or self.session.finished()
