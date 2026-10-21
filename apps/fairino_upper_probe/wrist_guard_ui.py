"""Thin GTK adapter. All decisions and protocol live in wrist_guard.py."""
import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

if __package__:
    from .wrist_guard import VERSION, MARGIN_DEG
else:
    from wrist_guard import VERSION, MARGIN_DEG


class GuardControls(Gtk.Box):
    def __init__(self, on_change):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=3)
        self.toggle = Gtk.CheckButton(label="腕部联合限位 · 运行中可启用 / 停用")
        self.toggle.set_active(True)
        self.toggle.connect("toggled", lambda button: on_change(button.get_active()))
        self.pack_start(self.toggle, False, False, 0)
        self.label = Gtk.Label(xalign=0)
        self.label.set_line_wrap(True)
        self.pack_start(self.label, False, False, 0)
        self.set_tooltip_text(
            "左腕 J6/J7、右腕 J13/J14 的前臂与腕壳干涉保护。\n"
            "切换先暂停并等待静止；保持使能，重新操作才继续。\n"
            f"两个角度方向各留 {MARGIN_DEG:g}° 模型余量；不覆盖全身 / 环境碰撞。")
        self.update({})

    def update(self, status, pending=False):
        state = status or {}
        active = state.get("policy")
        text = state.get("error") or ("切换中：暂停并等待双端确认" if pending else state.get("message"))
        if not text:
            text = "默认启用，连接后确认" if self.toggle.get_active() else "启动时停用此模块"
        self.label.set_text(f"{text} · {VERSION} · 双轴余量 {MARGIN_DEG:g}°"
                            + (f" · 配置 #{active['epoch']}" if active else ""))
