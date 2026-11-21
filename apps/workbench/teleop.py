"""Teleoperation panel; launches the same high-rate client as `run.sh vr`."""
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib
from apps.gripper_control.gripper_core import TELEOP_SPEED, INSTALLATION_SERIALS
from apps.fairino_tcp_probe.tool_frames import load_geometry, tool_config
from scripts.pico4.fairino.vr_dxhand import validate_ports
from scripts.pico4.fairino.home_presets import HOME_PRESETS
from scripts.pico4.fairino.control_modes import MODE_LABELS, CHASSIS_DIRECTIONS
from apps.fairino_upper_probe.wrist_guard_ui import GuardControls
from apps.fairino_keyboard.control_core import (DEFAULT_LINEAR, DEFAULT_ANGULAR,
                                                MIN_SPEED, MAX_LINEAR, MAX_ANGULAR)

ROOT = Path(__file__).resolve().parents[2]
TELEOP_MODES = {
    'mirror_same_direction': '右手柄 → 左臂 / 左爪，左手柄 → 右臂 / 右爪',
    'first_person': '左手柄 → 左臂 / 左爪，右手柄 → 右臂 / 右爪',
}
IK_BACKENDS = {'native': '联合限位 IK', 'vendor': '原厂 IK', 'tron2': 'TRON2 QP IK',
               'fairino_qp': 'Fairino QP IK（手臂+躯干）'}
DEMO_IK_BACKENDS = {**IK_BACKENDS, 'tron2': 'TRON2 QP IK（Demo）'}
NATIVE_GUARD_MESSAGE = '联合限位 IK 要求腕部联合限位保护开启，不能关闭'
TRON_GUARD_MESSAGE = 'TRON2 QP IK 实机要求腕部联合限位及路径检查开启，不能关闭'


class TeleopPanel(Gtk.Box):
    def __init__(self, credentials, acquire, release, demo=False):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        self.credentials, self.acquire, self.release = credentials, acquire, release
        self.demo = demo
        self.process = self.service = None
        self.session_ik_backend = None
        self.session_target = None
        self.demo_state = None
        self.demo_received = 0.
        self.updating_wrist = False
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
        target_row = Gtk.Box(spacing=8)
        target_row.pack_start(Gtk.Label(label='运行目标'), False, False, 0)
        self.runtime_target = Gtk.ComboBoxText()
        if not demo:
            self.runtime_target.append('robot', '真机')
        self.runtime_target.append('model', '控制台模型（Demo）')
        self.runtime_target.set_active_id('model' if demo else 'robot')
        target_row.pack_start(self.runtime_target, False, False, 0)
        settings.pack_start(target_row, False, False, 0)
        self.target_help = Gtk.Label(xalign=0)
        self.target_help.set_line_wrap(True)
        settings.pack_start(self.target_help, False, False, 0)
        mode_row = Gtk.Box(spacing=8)
        mode_row.pack_start(Gtk.Label(label='遥操方式'), False, False, 0)
        self.teleop_mode = Gtk.ComboBoxText()
        for name in TELEOP_MODES:
            self.teleop_mode.append(name, MODE_LABELS[name])
        self.teleop_mode.set_active_id('mirror_same_direction')
        mode_row.pack_start(self.teleop_mode, False, False, 0)
        settings.pack_start(mode_row, False, False, 0)
        self.mode_help = Gtk.Label(xalign=0)
        self.mode_help.set_line_wrap(True)
        settings.pack_start(self.mode_help, False, False, 0)
        self.teleop_mode.connect('changed', self.mode_changed)
        self.mode_changed()
        ik_row = Gtk.Box(spacing=8)
        ik_row.pack_start(Gtk.Label(label='IK 解算器'), False, False, 0)
        self.ik_backend = Gtk.ComboBoxText()
        self.ik_options_model = demo
        for name, title in (DEMO_IK_BACKENDS if demo else IK_BACKENDS).items():
            self.ik_backend.append(name, title)
        self.ik_backend.set_active_id('native')
        self.ik_backend.set_tooltip_text(
            '联合限位 IK / TRON2 QP IK 默认 500 Hz，原厂 IK 默认 450 Hz；启动后不能切换。\n'
            '沿用原厂链路的 TCP 插值：位移线性、姿态 SLERP，默认缓冲 12 ms，再进行 IK 并直接发布关节目标。\n'
            '联合限位 IK 在求解中约束腕部，TRON2 在求解后检查腕部及路径；实机两者均要求保护开启。')
        ik_row.pack_start(self.ik_backend, False, False, 0)
        settings.pack_start(ik_row, False, False, 0)
        text = ('法兰和 TacCap：握 Grip 跟随，松开保持；Trigger 控制 TacCap 开度。\n'
                '按住 A/X 冻结旋转、B/Y 冻结位移；双击 X 降低、双击 Y 提高灵敏度。\n'
                '左杆 XY 平移、右杆 X 旋转；右杆 Y 升降。右杆按下回 Home；空格或「暂停 / 保持」暂停。\n'
                '左杆按下校准双臂正前方；正常转头不改变校准，底盘方向仍由所选遥操方式决定。\n'
                '校准前法兰和 TacCap 需松开两侧 Grip；两摇杆回中并松开右杆按键，确认双臂、身体和底盘停止；平视希望的正前方。\n'
                '底盘与升降需分别勾选；手臂控制由独立进程执行，下方显示实际频率。\n'
                '先启动接收服务，在头显选择电脑 IP 并打开 Head / Controller / Send。')
        label = Gtk.Label(label=text, xalign=0)
        label.set_line_wrap(True)
        settings.pack_start(label, False, False, 0)
        self.wrist_controls = GuardControls(self.switch_wrist)
        self.robot_wrist_tooltip = self.wrist_controls.get_tooltip_text()
        settings.pack_start(self.wrist_controls, False, False, 0)
        self.held_speed_check = Gtk.CheckButton(label='保持关节速度检测（1°/s）· 启动时生效')
        self.held_speed_check.set_active(True)
        self.held_speed_check.set_tooltip_text(
            '默认开启；取消勾选仅关闭当前被标记为保持不动的关节的反馈速度检测。\n'
            '保持目标偏差超过 0.3°、停止确认、硬件故障、急停、断流及腕部保护仍保留。\n'
            '仅用于真机；运行中不能切换，停止遥操后可修改。')
        settings.pack_start(self.held_speed_check, False, False, 0)
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
        tool_row.pack_start(Gtk.Label(label='末端'), False, False, 0)
        self.effector = Gtk.ComboBoxText()
        self.effector.append('flange', '法兰中心')
        self.effector.append('taccap', 'TacCap 夹爪 · 固定指尖中心')
        self.effector.append('dxhand', '达显灵巧手 · 法兰中心')
        self.effector.set_active_id('flange')
        tool_row.pack_start(self.effector, False, False, 0)
        self.tool_settings = Gtk.Button(label='安装角度 / TCP 设置与预览')
        self.tool_settings.connect('clicked', self.configure_tools)
        tool_row.pack_start(self.tool_settings, False, False, 0)
        settings.pack_start(tool_row, False, False, 0)
        self.dx_panel = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        self.dx_sides, self.dx_ports, self.dx_calibrations = {}, {}, {}
        for side, title in (('left', '左手'), ('right', '右手')):
            row = Gtk.Box(spacing=6)
            enabled = Gtk.CheckButton(label=title)
            enabled.set_active(side == 'right')
            port, calibration = Gtk.Entry(), Gtk.Entry()
            port.set_placeholder_text('USB · 留空自动识别')
            calibration.set_placeholder_text('Manus 校准文件（可选）')
            for widget in (enabled, port, calibration):
                row.pack_start(widget, False, False, 0)
            self.dx_sides[side], self.dx_ports[side], self.dx_calibrations[side] = enabled, port, calibration
            self.dx_panel.pack_start(row, False, False, 0)
        note = Gtk.Label(label='USB 留空自动识别左右手（按固件手别并记住设备序列号）。手需上电自动回零后再启动；就绪后双臂与手指自动跟随。'
                         '暂停后再按恢复。退出下使能时，手中物体可能掉落。', xalign=0)
        note.set_line_wrap(True)
        self.dx_panel.pack_start(note, False, False, 0)
        settings.pack_start(self.dx_panel, False, False, 0)
        self.dx_panel.show_all()
        self.dx_panel.set_no_show_all(True)
        self.effector.connect('changed', self.tool_changed)
        gripper_row = Gtk.Box(spacing=8)
        self.gripper_serials = {}
        for side, title in [('left', '左爪 SN'), ('right', '右爪 SN')]:
            gripper_row.pack_start(Gtk.Label(label=title), False, False, 0)
            entry = Gtk.Entry()
            entry.set_placeholder_text('留空按安装侧绑定')
            entry.set_tooltip_text('当前安装绑定：'+INSTALLATION_SERIALS[side]+'；填写 SN 可覆盖')
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
        speed_row.pack_start(Gtk.Label(label='启动时生效；夹持力矩自动取各爪最大持续值'), False, False, 0)
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
        self.taccap_rows = (gripper_row, speed_row, limits_row, gripper_help)
        for row in self.taccap_rows:
            row.show_all()
            row.set_no_show_all(True)
        self.body_lift = Gtk.CheckButton(label='启用身体预设升降 · 右杆长推1秒：上到100% / 下到66.7% · 30°/s')
        self.body_lift.set_active(False)
        self.body_lift.set_tooltip_text('上位为标定100%，下位为66.7%（原50%向上提高剩余行程的1/3）。到位不重复运动；触发后回中继续到位。\n'
                                       '暂停、Home或断流中止；完成后回中才能再次触发。\n'
                                       '若当前低于66.7%，下推也会升到66.7%；启动本身不会抬升。只改变J18–J20。')
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
        self.stop_button = self.start_button = None
        for title, callback in [('启动 PICO 接收服务', self.start_service), ('启动遥操 · 使能上半身', self.start),
                                ('停止遥操并下使能', lambda *_: self.command('stop'))]:
            button = Gtk.Button(label=title)
            button.connect('clicked', callback)
            if title == '停止遥操并下使能':
                self.stop_button = button
                button.set_sensitive(False)
            elif title == '启动遥操 · 使能上半身':
                self.start_button = button
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
        self.heading_status = Gtk.Label(label='双臂正前方未校准 · 沿用原有追踪方向', xalign=0)
        self.heading_status.set_line_wrap(True)
        self.heading_status.set_max_width_chars(100)
        self.pack_start(self.heading_status, False, False, 0)
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
        for enabled in self.dx_sides.values():
            enabled.connect('toggled', lambda *_: self.update_options())
        self.ik_backend.connect('changed', self.ik_changed)
        self.runtime_target.connect('changed', self.target_changed)
        self.ik_changed()
        self.tool_changed()
        self.target_changed()
        GLib.timeout_add(100, self.tick)

    def model_demo_selected(self):
        return (self.session_target or self.runtime_target.get_active_id()) == 'model'

    def update_ik_choices(self):
        model = self.model_demo_selected()
        if model == self.ik_options_model:
            return
        choices = DEMO_IK_BACKENDS if model else IK_BACKENDS
        selected = self.ik_backend.get_active_id()
        self.ik_options_model = model
        self.ik_backend.remove_all()
        for name, title in choices.items():
            self.ik_backend.append(name, title)
        self.ik_backend.set_active_id(selected if selected in choices else 'native')

    def target_changed(self, *_):
        model = self.model_demo_selected()
        self.update_ik_choices()
        if model:
            self.chassis.set_active(False)
        self.target_help.set_text(
            '模型演示：启动前可选择联合限位 IK、原厂 IK、TRON2 QP IK 或 Fairino QP IK；腕部联合限位可开关。\n'
            '每次从所选双臂 Home 和身体上位开始；可演示双臂、身体升降和夹爪，底盘暂不参与。'
            if model else '真机遥操：启动会连接并使能上半身；可按需启用夹爪、身体及底盘。')
        self.start_button.set_label('启动模型 Demo' if model else '启动遥操 · 使能上半身')
        self.stop_button.set_label('停止模型 Demo' if model else '停止遥操并下使能')
        self.home.set_label('模型从所选双臂 Home 开始（每次启动重置）' if model else
                            '启动后先回双臂 Home（会运动）；不勾选则保持当前姿态')
        if model:
            self.wrist_controls.set_tooltip_text(
                '模型中可关闭左腕 J6/J7、右腕 J13/J14 的联合限位。\n'
                '红色腕部表示进入标定的前臂与腕壳干涉区（近似），不代表全身或环境碰撞检测。\n'
                '切换会暂停；重新启用前需回到保护范围内，可先回 Home。真机联合限位 IK / TRON2 始终开启保护。')
        else:
            self.wrist_controls.set_tooltip_text(self.robot_wrist_tooltip)
        self.chassis_status.set_text('模型 Demo 暂不包含底盘运动' if model else '底盘遥操未启用')
        self.update_options()

    def model_snapshot(self):
        """Return only simulated feedback; callers must not mix live observers into it."""
        if not self.model_demo_selected():
            return None
        active = self.session_target == 'model' and self.process is not None and self.process.poll() is None
        fresh = bool(active and self.demo_state is not None and time.monotonic()-self.demo_received < .5)
        state = self.demo_state or {}
        return {'flange_only': self.effector.get_active_id() == 'dxhand', 'hardware': state.get('hardware') if fresh else None,
                'grippers': state.get('grippers', {}) if fresh else {},
                'wrist_collision': state.get('wrist_collision', {}),
                'fresh': fresh, 'active': active, 'phase': state.get('phase', '等待模型 Demo 启动')}

    def mode_changed(self, *_):
        mode = self.teleop_mode.get_active_id()
        value = TELEOP_MODES.get(mode)
        self.mode_help.set_text((value+'。\n'+CHASSIS_DIRECTIONS[mode]+
            '。\n启动后需先结束遥操才能切换方式。') if value else '请选择遥操方式')

    def tool_changed(self, *_):
        dx = self.effector.get_active_id() == 'dxhand'
        self.dx_panel.set_visible(dx)
        for row in self.taccap_rows:
            row.set_visible(self.effector.get_active_id() == 'taccap')
        self.command_buttons[0].set_label('暂停 / 恢复' if dx else '暂停 / 保持')
        self.gripper_status.set_text('达显手等待启动' if dx else '夹爪遥操未启用')
        self.update_options()

    def tcp_mode(self):
        mode = self.effector.get_active_id()
        return 'flange' if mode == 'dxhand' else mode

    def dxhand_args(self):
        argv, ports = ['--dxhand'], []
        for side, enabled in self.dx_sides.items():
            if enabled.get_active():
                port = self.dx_ports[side].get_text().strip() or 'auto'
                ports.append(port)
                argv.extend(['--dxhand-'+side, port])
                path = self.dx_calibrations[side].get_text().strip()
                if path:
                    argv.extend(['--dxhand-'+side+'-calibration', path])
        validate_ports(ports)
        return argv

    def dxhand_status(self, state):
        if state.get('event') == 'unconfirmed':
            self.gripper_status.set_text('达显退出回执缺失 · 物理下使能需现场核验')
            return
        phases = {'connecting': '连接中', 'home_reset': '回零复位', 'homing': '回零中',
                  'holding': '保持', 'tracking': '跟随', 'closed': '已退出'}
        values = []
        for side, h in state.get('hands', {}).items():
            value = ('左手' if side == 'left' else '右手') + ' ' + phases.get(h['phase'], h['phase'])
            if state.get('event') == 'stopped':
                value += ' · 下使能广播' + ('已发送' if h['driver_off_sent'] else '未发送')
            values.append(value)
        self.gripper_status.set_text('达显：' + (' / '.join(values) or '等待反馈')
                                     + (' · 物理下使能需现场核验' if state.get('event') == 'stopped' else ''))

    def native_ik_selected(self):
        return (self.session_ik_backend or self.ik_backend.get_active_id()) == 'native'

    def required_wrist_message(self):
        if self.model_demo_selected():
            return None
        backend = self.session_ik_backend or self.ik_backend.get_active_id()
        return {'native': NATIVE_GUARD_MESSAGE, 'tron2': TRON_GUARD_MESSAGE,
                'fairino_qp': 'Fairino QP IK 实机要求腕部联合限位及路径检查开启，不能关闭'}.get(backend)

    def update_wrist_option(self):
        required = self.required_wrist_message()
        self.ik_backend.set_tooltip_text(
            'Demo 的 IK 均在本机计算，不连接机器人；启动后不能切换。\n'
            'Fairino QP IK 联动双臂和躯干；回 Home 时先收臂，再回到本次会话的躯干起始姿态。\n'
            '原厂 IK 和 TRON2 QP IK 可配合腕部联合限位检查；关闭限位不切换这两种 IK。\n'
            '选择联合限位 IK 后关闭限位，则使用 Demo 专用无腕部联合约束的数值 IK。'
            if self.model_demo_selected() else
            '联合限位 IK / TRON2 QP IK 默认 500 Hz，原厂 IK 默认 450 Hz，Fairino QP IK 默认 200 Hz。\n'
            '沿用原厂链路的 TCP 插值：位移线性、姿态 SLERP，默认缓冲 12 ms，再进行 IK 并直接发布关节目标。\n'
            '联合限位 IK 在求解中约束腕部；TRON2 在求解后检查腕部及路径，必要时缩小目标重新求解。\n'
            '联合限位 IK、TRON2 和 Fairino QP 在实机上均要求腕部保护开启。')
        if required and not self.wrist_controls.toggle.get_active():
            self.updating_wrist = True
            try:
                self.wrist_controls.toggle.set_active(True)
            finally:
                self.updating_wrist = False
        self.wrist_controls.toggle.set_sensitive(not self.closed and not required)
        self.wrist_controls.toggle.set_label('腕部联合限位 · 模型 Demo 可启用 / 停用' if self.model_demo_selected()
                                            else '腕部联合限位 · 当前 IK 要求开启' if required
                                            else '腕部联合限位 · 运行中可启用 / 停用')

    def ik_changed(self, *_):
        self.update_options()
        message = self.required_wrist_message()
        self.wrist_controls.update({'message': message} if message else {})

    def switch_wrist(self, enabled):
        if self.updating_wrist:
            return
        message = self.required_wrist_message()
        if message:
            self.update_wrist_option()
            self.wrist_controls.update({'message': message})
            return
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
        if self.closed:
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
        if self.closed or (self.process and self.process.poll() is None):
            return
        if self.session_target or self.upper_lease or self.gripper_lease or self.base_lease:
            self.append('上一会话正在退出，等待控制占用释放后再启动')
            return
        try:
            if self.teleop_mode.get_active_id() not in TELEOP_MODES:
                raise ValueError('请选择有效的遥操方式')
            backend = self.ik_backend.get_active_id()
            choices = DEMO_IK_BACKENDS if self.model_demo_selected() else IK_BACKENDS
            if backend not in choices:
                raise ValueError('请选择有效的 IK 解算器')
            self.update_wrist_option()
            mode = self.tcp_mode()
            dxhand = self.effector.get_active_id() == 'dxhand'
            dx_args = self.dxhand_args() if dxhand and not self.model_demo_selected() else []
            tool_config(mode, load_geometry() if mode == 'taccap' else None,
                        require_confirmed=not self.model_demo_selected())
        except (ValueError, OSError) as exc:
            self.append(str(exc))
            return
        if self.model_demo_selected():
            self.start_model_demo()
            return
        if self.demo:
            self.append('离线控制台仅支持模型 Demo')
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
            if self.effector.get_active_id() == 'taccap':
                parts.append('gripper')
            if self.chassis.get_active():
                parts.append('base')
            for part in parts:
                if not self.acquire(part, '遥操'):
                    self.release_leases()
                    return
                setattr(self, part+'_lease', True)
            argv = ['bash', str(ROOT/'run.sh'), 'vr', '--gui-stdio', '--host', host, '--user', user]
            argv.extend(['--mode', self.teleop_mode.get_active_id(), '--rotation-mode', 'auto',
                         '--hand-mapping', 'auto'])
            argv.extend(['--ik-backend', self.ik_backend.get_active_id()])
            argv.extend(['--tcp-tool', mode, *dx_args])
            argv.extend(['--home-preset', self.home_preset.get_active_id()])
            if not self.wrist_controls.toggle.get_active():
                argv.append('--no-wrist-guard')
            if not self.held_speed_check.get_active():
                argv.append('--no-held-speed-check')
            if not self.home.get_active():
                argv.append('--no-auto-home')
            if self.effector.get_active_id() == 'taccap':
                argv.append('--grippers')
                argv.extend(['--gripper-speed-rad-s', str(self.gripper_speed.get_value())])
                for side in ('left', 'right'):
                    argv.extend([f'--{side}-gripper-sn', self.gripper_serials[side].get_text().strip()])
                    argv.extend([f'--{side}-gripper-max-open-percent', str(self.gripper_limits[side].get_value())])
            if backend != 'fairino_qp' and self.body_lift.get_active():
                argv.append('--body-lift')
            if self.chassis.get_active():
                argv.extend(['--chassis', '--chassis-host', base_credentials[0],
                             '--chassis-linear', str(self.chassis_linear.get_value()),
                             '--chassis-angular', str(self.chassis_angular.get_value())])
            self.process = subprocess.Popen(argv, cwd=ROOT, stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
            self.session_ik_backend = self.ik_backend.get_active_id()
            self.session_target = 'robot'
            self.process.stdin.write(json.dumps({'password': password}) + '\n')
            self.process.stdin.flush()
            self.status.set_text('等待双手柄数据与达显手回零…' if dxhand else '等待双手柄新鲜数据及 Grip 释放…')
            self.heading_status.set_text('双臂正前方未校准 · 沿用原有追踪方向')
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
                self.session_target = self.session_ik_backend = None
            self.append(str(exc))

    def start_model_demo(self):
        """Separate local process: never request credentials or acquire robot devices."""
        argv = [str(ROOT/'.local/pico4/env/bin/python'), '-u',
                str(ROOT/'scripts/pico4/fairino/vr_demo.py'), '--gui-stdio',
                '--ik-backend', self.ik_backend.get_active_id(),
                '--mode', self.teleop_mode.get_active_id(), '--freeze-buttons',
                '--tcp-tool', self.tcp_mode(),
                '--home-preset', self.home_preset.get_active_id()]
        if not self.wrist_controls.toggle.get_active():
            argv.append('--disable-wrist-guard')
        if self.ik_backend.get_active_id() != 'fairino_qp' and self.body_lift.get_active():
            argv.append('--body-lift')
        if self.effector.get_active_id() == 'dxhand':
            argv.append('--dxhand')
        if self.effector.get_active_id() == 'taccap':
            argv.append('--grippers')
            for side in ('left', 'right'):
                argv.extend([f'--{side}-gripper-max-open-percent', str(self.gripper_limits[side].get_value())])
        try:
            self.process = subprocess.Popen(argv, cwd=ROOT, stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
            self.session_target = 'model'
            self.session_ik_backend = self.ik_backend.get_active_id()
            self.demo_state, self.demo_received = None, 0.
            self.gripper_feedback = {}
            self.status.set_text('模型 Demo 启动中 · 等待 PICO 手柄数据；机器人未连接')
            self.heading_status.set_text('双臂正前方未校准 · 沿用原有追踪方向')
            self.chassis_status.set_text('模型 Demo 暂不包含底盘运动')
            self.update_options()
            threading.Thread(target=self.reader, args=(self.process, 'demo'), daemon=True).start()
        except (OSError, ValueError, RuntimeError) as exc:
            if self.process and self.process.poll() is None:
                self.process.terminate()
            self.session_target = self.session_ik_backend = None
            self.append(f'模型 Demo 启动失败：{exc}')

    def release_leases(self):
        for part in ('upper', 'gripper', 'base'):
            name = part+'_lease'
            if getattr(self, name):
                self.release(part, '遥操')
                setattr(self, name, False)

    def update_options(self):
        self.update_ik_choices()
        running = self.process is not None and self.process.poll() is None
        editable = not running and self.session_target is None and not self.closed
        model = self.model_demo_selected()
        self.runtime_target.set_sensitive(editable and not self.demo)
        for widget in (self.teleop_mode, self.ik_backend, self.home, self.home_preset, self.effector,
                       self.tool_settings, self.body_lift, self.chassis, *self.gripper_serials.values()):
            widget.set_sensitive(editable)
        for widget in (self.home, self.chassis, *self.gripper_serials.values()):
            widget.set_sensitive(editable and not model)
        whole_body = self.ik_backend.get_active_id() == 'fairino_qp'
        if whole_body:
            self.body_lift.set_active(False)
        self.body_lift.set_sensitive(editable and not whole_body)
        self.dx_panel.set_sensitive(editable and not model)
        for side, enabled in self.dx_sides.items():
            for widget in (self.dx_ports[side], self.dx_calibrations[side]):
                widget.set_sensitive(enabled.get_active())
        self.held_speed_check.set_sensitive(editable and not model)
        for widget in (self.gripper_speed, *self.gripper_limits.values()):
            widget.set_sensitive(editable and self.effector.get_active_id() == 'taccap')
        self.gripper_speed.set_sensitive(editable and self.effector.get_active_id() == 'taccap' and not model)
        for widget in (self.chassis_linear, self.chassis_angular):
            widget.set_sensitive(editable and self.chassis.get_active() and not model)
        self.start_button.set_sensitive(editable)
        self.update_wrist_option()

    def command(self, command):
        if self.process is None or self.process.poll() is not None:
            return
        try:
            self.process.stdin.write(json.dumps({'command': command}) + '\n')
            self.process.stdin.flush()
            if command == 'stop':
                self.status.set_text('正在停止模型 Demo…' if self.model_demo_selected() else
                                     '正在等待遥操退出并确认下使能…')
        except (OSError, ValueError):
            self.status.set_text('遥操命令管道已关闭，等待退出结果')

    def receive_model_state(self, data):
        state = json.loads(data)
        hardware = state['hardware']
        names, positions = hardware['joint_names'], hardware['position']
        if names != [f'j{i}' for i in range(1, 21)] or len(positions) != 20:
            raise ValueError('模型关节格式错误')
        if not all(type(v) in (int, float) and math.isfinite(v) for v in positions):
            raise ValueError('模型关节角无效')
        grippers = state.get('grippers', {})
        if (not isinstance(grippers, dict) or any(side not in ('left', 'right') for side in grippers)
                or not all(type(v) in (int, float) and math.isfinite(v) and 0. <= v <= 1.
                           for v in grippers.values())):
            raise ValueError('模型夹爪开度无效')
        collisions = state.get('wrist_collision', {})
        if (not isinstance(collisions, dict) or any(side not in ('left', 'right') for side in collisions)
                or not all(type(v) is bool for v in collisions.values())):
            raise ValueError('模型干涉状态无效')
        sensitivity = state.get('sensitivity', .25)
        if type(sensitivity) not in (int, float) or not math.isfinite(sensitivity):
            raise ValueError('模型灵敏度无效')
        self.demo_state, self.demo_received = state, time.monotonic()
        backend = state.get('ik_backend')
        enabled = state.get('wrist_guard_enabled')
        if backend == 'vendor':
            ik_name = ('原厂 IK + 腕部联合限位检查' if enabled is True else
                       '原厂 IK（联合限位关闭）' if enabled is False else
                       '原厂 IK（联合限位状态未报告）')
        elif backend == 'native':
            ik_name = '联合限位 IK'
        elif backend == 'demo_joint_box_only':
            ik_name = '本地数值 IK（联合限位关闭）'
        elif backend in ('tron2', 'fairino_qp'):
            name = 'TRON2 QP IK' if backend == 'tron2' else DEMO_IK_BACKENDS[backend]
            ik_name = name + (' + 腕部联合限位检查' if enabled is True else
                              '（联合限位关闭）' if enabled is False else '（联合限位状态未报告）')
        else:
            ik_name = '未报告'
        text = (f'模型 Demo · {state.get("phase", "跟随")} · 灵敏度 {sensitivity:g} · 仅显示模拟关节'
                + '\nIK：'+ik_name)
        if state.get('message'):
            text += '\n'+str(state['message'])
        if state.get('ik_partial'):
            text += '\n当前为部分可行解，完整手柄目标仍保留'
        if backend == 'vendor' and state.get('wrist_limited'):
            text += '\n腕部联合限位已缩小目标；可向内收腕'
        self.status.set_text(text)
        heading = '双臂正前方已校准' if state.get('heading_calibrated') else '双臂正前方未校准 · 沿用原有追踪方向'
        yaw = state.get('heading_yaw_deg')
        if state.get('heading_calibrated') and type(yaw) in (int, float) and math.isfinite(yaw):
            heading += f' · 水平朝向 {yaw:+.1f}°'
        if state.get('heading_message'):
            heading += '\n'+str(state['heading_message'])
        self.heading_status.set_text(heading)
        self.gripper_status.set_text('模型夹爪：'+' · '.join(
            f'{label} {grippers[side]*100:.1f}%' for side, label in [('left', '左'), ('right', '右')]
            if side in grippers) if grippers else '达显模型 · 仅显示法兰' if state.get('dxhand') else '模型夹爪演示未启用')
        if type(enabled) is bool:
            self.updating_wrist = True
            try:
                self.wrist_controls.toggle.set_active(enabled)
            finally:
                self.updating_wrist = False
            self.wrist_controls.update({'message': '模型联合限位已启用' if enabled else
                                       '模型联合限位已停用；进入标定干涉区时腕部显示红色'})

    def tick(self):
        while not self.events.empty():
            name, data = self.events.get_nowait()
            if name == 'demo_exit':
                if self.session_target != 'model':
                    continue
                self.session_target = self.session_ik_backend = None
                self.status.set_text(f'模型 Demo 已停止，返回码 {data}；保留最后模型姿态')
                self.heading_status.set_text('方向校准仅对本次演示有效；下次启动需重新校准')
                if self.process:
                    self.process.stdin.close()
            elif name == 'demo' and data.startswith('WORKBENCH_DEMO '):
                if self.session_target != 'model':
                    continue
                try:
                    self.receive_model_state(data[len('WORKBENCH_DEMO '):])
                except (ValueError, KeyError, TypeError, AttributeError):
                    self.append('模型 Demo 状态格式异常；保留最后姿态')
            elif name in ('vr', 'vr_exit') and self.session_target == 'model':
                continue
            elif name == 'vr_exit':
                self.session_ik_backend = None
                self.session_target = None
                self.gripper_feedback = {}
                self.release_leases()
                self.status.set_text(f'遥操已退出，返回码 {data}；下使能结果见下方日志')
                self.heading_status.set_text('方向校准仅对本次遥操有效；下次启动需重新校准')
                self.chassis_status.set_text('底盘遥操已结束；下使能结果见下方日志' if self.chassis.get_active()
                                             else '底盘遥操未启用')
                if self.process:
                    self.process.stdin.close()
            elif name == 'vr' and data.startswith('WORKBENCH_DXHAND '):
                try:
                    self.dxhand_status(json.loads(data[len('WORKBENCH_DXHAND '):]))
                except (ValueError, KeyError, TypeError):
                    self.append('达显退出回执格式异常；物理下使能需现场核验')
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
                    if state.get('wrist_limited') and state.get('ik_backend') != 'native':
                        self.wrist_controls.label.set_text(
                            self.wrist_controls.label.get_text()+' · 接近边界，已缩小目标；可向内收腕')
                    grips = state.get('grips', {})
                    ik_name = IK_BACKENDS.get(state.get('ik_backend'), '未报告')
                    self.status.set_text(f'{state["phase"]} · {state["sides"]} · 灵敏度 {state["sensitivity"]:g}\n'
                        + f'IK：{ik_name} · '
                        + ('TCP：夹爪固定指尖中心 · ' if state.get('tcp_tool') == 'taccap' else 'TCP：法兰中心 · ')+
                        f'Grip 左 {grips.get("left", 0):.2f} / 右 {grips.get("right", 0):.2f} · '
                        f'使能 {state["enabled"]}/20 · IK 目标下发 {state["send_hz"]:.0f} / 接收 {state["accepted_hz"]:.0f} Hz')
                    held_check = state.get('held_speed_check_enabled')
                    self.status.set_text(self.status.get_text()+'\n保持关节速度检测（1°/s）：'+
                        ('已开启' if held_check is True else '已关闭' if held_check is False else '等待接收端确认'))
                    if state.get('ik_backend') == 'native' and state.get('ik_partial'):
                        self.status.set_text(self.status.get_text()+'\n当前为部分可行解，完整手柄目标仍保留')
                    elif state.get('ik_backend') in ('tron2', 'fairino_qp') and state.get('ik_partial'):
                        self.status.set_text(self.status.get_text()+(
                            '\n腕部保护已缩小目标，越界手柄位移已舍弃' if state.get('wrist_limited') else
                            '\n' + ('TRON2' if state['ik_backend'] == 'tron2' else 'Fairino QP')
                            + ' 当前未完全收敛，完整手柄目标仍保留'))
                    if state.get('receiver_phase') and state['receiver_phase'] != state['phase']:
                        self.status.set_text(self.status.get_text()+'\n接收端：'+str(state['receiver_phase']))
                    heading = '双臂正前方已校准' if state.get('heading_calibrated') else '双臂正前方未校准 · 沿用原有追踪方向'
                    heading_yaw = state.get('heading_yaw_deg')
                    if (state.get('heading_calibrated') and type(heading_yaw) in (int, float)
                            and math.isfinite(heading_yaw)):
                        heading += f' · 水平朝向 {heading_yaw:+.1f}°'
                    if state.get('heading_message'):
                        heading += '\n' + str(state['heading_message'])
                    self.heading_status.set_text(heading)
                    grippers = state.get('grippers')
                    self.gripper_feedback = grippers or {}
                    if grippers:
                        units = grippers.get('devices', {})
                        values = []
                        for side, label in [('left', '左爪'), ('right', '右爪')]:
                            unit = units.get(side)
                            text = f'{label} {unit["position"]*100:.1f}% / {unit["state"]}' if unit else f'{label}未连接'
                            if unit and unit.get('controlled') and 'grasp_torque_nm' in unit:
                                text += f' · 夹持设定 {unit["grasp_torque_nm"]:.2f} Nm（最大持续）'
                            if unit and unit.get('open_span_rad'):
                                maximum = grippers.get('max_openings', {}).get(side, 1.)
                                text += f' · 映射上限 {maximum*100:g}% ≈ {math.degrees(maximum*unit["open_span_rad"]):.1f}° 电机转角'
                            values.append(text)
                        self.gripper_status.set_text(grippers.get('message', '')+
                            f' · 速度 {grippers.get("speed_rad_s", 0.):g} rad/s\n'+' · '.join(values))
                    elif state.get('dxhand') is not None:
                        self.dxhand_status(state['dxhand'])
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
                        preset = state.get('body_lift_preset') or {}
                        self.gripper_status.set_text(self.gripper_status.get_text()+'\n'+(
                            f'身体 {body["height"]:.1f}% / 相对低位 {body["height_mm"]:.1f} mm · '
                            + preset.get('message', '长推1秒：上到100% / 下到66.7% · 30°/s')
                            if body else '等待身体高度反馈'))
                except (ValueError, KeyError, TypeError):
                    self.append('遥操状态格式异常')
            else:
                self.append(f'[{name}] {data}')
        running = self.process is not None and self.process.poll() is None
        self.update_options()
        self.stop_button.set_sensitive(running)
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
