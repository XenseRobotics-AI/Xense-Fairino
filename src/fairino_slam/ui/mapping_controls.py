"""Mapping chassis controls, reusing the original guarded keyboard window."""
import math
import queue
import threading
from types import SimpleNamespace

from fairino_slam.chassis.control_core import (
    KEYS, STEER, key_twist, check_speeds, DEFAULT_LINEAR, DEFAULT_ANGULAR,
    MIN_SPEED, MAX_LINEAR, MAX_ANGULAR,
)
from fairino_slam.chassis.transport import Connection
from fairino_slam.paths import LOG_ROOT


class Window:
    def __init__(self, args, connection=None, window_factory=None, on_closed=None, compact=False, start_timer=True):
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
        self.input_suspended = False
        self.speed_pending = False
        self.last_phase = "演示模式，不连接机器人" if args.demo else "正在读取底盘状态"
        self.ui_closed = on_closed or Gtk.main_quit
        self.window = (window_factory or Gtk.Window)(title="法奥底盘键控 · WASD / JL")
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
                                "切窗暂停移动并保持使能；切回后松键重按。断线或故障会结束控制。"))
        box.pack_start(footer, False, False, 0)
        if compact:
            # Reuse the same controls, events and transport inside the mapping panel.
            # Only their arrangement changes; motion and stop semantics stay identical.
            for child in box.get_children():
                box.remove(child)
            title.destroy()
            self.window.set_border_width(4)
            box.set_spacing(4)
            help_label.set_text('W/S 前后  ·  A/D 横移  ·  J/L 旋转  ·  松键停止，保持使能')
            box.pack_start(help_label, False, False, 0)
            columns = Gtk.Box(spacing=12)
            for widgets in ((speed_grid, self.speed_hint, self.control_focus),
                            (self.phase, self.motion, self.error, stop_button)):
                column = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
                for widget in widgets:
                    column.pack_start(widget, False, False, 0)
                columns.pack_start(column, True, True, 0)
            box.pack_start(columns, False, False, 0)
            details = Gtk.Expander(label='轮组反馈与操作说明')
            detail_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
            detail_box.pack_start(self.wheels, False, False, 0)
            detail_box.pack_start(footer, False, False, 0)
            details.add(detail_box)
            box.pack_start(details, False, False, 0)
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
        self.timer = GLib.timeout_add(50, self.tick) if start_timer else None

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
                    and not self.input_suspended and self.window.is_active()):
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
        self.input_suspended = False
        self.keys.clear()
        # Preserve held keys until a release: autorepeat cannot restart motion.
        return False

    def focus_out(self, *_):
        self.keys.clear()
        self.input_suspended = True
        self.phase.set_text('已暂停移动，保持使能；切回后松开方向键再按')
        self.send()
        return False

    def send(self):
        for widget in self.speed_widgets:
            widget.set_sensitive(not (self.keys or self.held or self.stopped or self.closing))
        if self.connection:
            focused = self.window.is_active() and not self.input_suspended
            self.connection.send(self.keys if self.ready and not self.speed_pending and focused else (),
                                 focused=focused, stop=self.stopped,
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
                self.GLib.idle_add(self.ui_closed)
            threading.Thread(target=finish, daemon=True).start()
        else:
            self.ui_closed()
        return True

    def run(self):
        self.Gtk.main()

