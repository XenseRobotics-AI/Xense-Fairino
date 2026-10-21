"""Editable CAD mounting shared by desktop visuals and PICO kinematics."""
import json

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk

from apps.fairino_tcp_probe.tool_frames import (DEFAULT_CONFIG, load_geometry, validate_geometry,
                                               flange_from_tip)


class ToolSettings(Gtk.Dialog):
    def __init__(self, parent=None, preview=None, path=DEFAULT_CONFIG):
        super().__init__(title='夹爪安装与固定 TCP', transient_for=parent, modal=True)
        self.path, self.preview = path, preview
        self.original = load_geometry(path)
        self.set_default_size(640, 420)
        self.add_button('取消', Gtk.ResponseType.CANCEL)
        self.add_button('保存配置', Gtk.ResponseType.OK)
        box = self.get_content_area()
        box.set_border_width(16)
        box.set_spacing(10)
        text = ('转接件厚 13 mm；夹爪定位凸台按 CAD 配合。调整时右侧模型实时预览。\n'
                '安装角度 0°：夹爪开合方向沿手腕局部 Y 轴；正角度绕法兰向外轴。\n'
                '当前 STEP 为张开姿态。固定 TCP 不随夹爪开合改变；实装角度和工具点需现场核对。')
        label = Gtk.Label(label=text, xalign=0)
        label.set_line_wrap(True)
        box.pack_start(label, False, False, 0)
        self.rows = {}
        self.tip = []
        for side, title in [('left', '左臂'), ('right', '右臂')]:
            row = Gtk.Box(spacing=8)
            installed = Gtk.CheckButton(label=title+'装有夹爪')
            installed.set_active(self.original['arms'][side]['installed'])
            angle = Gtk.SpinButton.new_with_range(-180., 180., 1.)
            angle.set_digits(1)
            angle.set_value(self.original['arms'][side]['roll_deg'])
            confirmed = Gtk.CheckButton(label='已核对实装角度及 TCP')
            confirmed.set_active(self.original['arms'][side]['confirmed'])
            for widget in (installed, Gtk.Label(label='安装角度 °'), angle, confirmed):
                row.pack_start(widget, False, False, 0)
            box.pack_start(row, False, False, 0)
            self.rows[side] = dict(installed=installed, angle=angle, confirmed=confirmed)
        tip_row = Gtk.Box(spacing=8)
        tip_row.pack_start(Gtk.Label(label='固定指尖中心 · 原始 CAD 坐标 mm'), False, False, 0)
        for axis, value, limits in zip('XYZ', self.original['tip_cad_mm'], [(-25, 25), (-20, 60), (-230, -80)]):
            tip_row.pack_start(Gtk.Label(label=axis), False, False, 0)
            spin = Gtk.SpinButton.new_with_range(*limits, .1)
            spin.set_digits(3)
            spin.set_value(value)
            self.tip.append(spin)
            tip_row.pack_start(spin, False, False, 0)
        box.pack_start(tip_row, False, False, 0)
        self.details = Gtk.Label(xalign=0)
        self.details.set_line_wrap(True)
        box.pack_start(self.details, False, False, 0)
        for side, row in self.rows.items():
            row['angle'].connect('value-changed', self.changed, side)
            row['installed'].connect('toggled', self.changed, side)
            row['confirmed'].connect('toggled', self.refresh)
        for spin in self.tip:
            spin.connect('value-changed', self.changed, None)
        self.refresh()
        self.show_all()

    def data(self):
        data = json.loads(json.dumps(self.original))
        data['tip_cad_mm'] = [spin.get_value() for spin in self.tip]
        data['arms'] = {side: dict(installed=row['installed'].get_active(),
            roll_deg=row['angle'].get_value(), confirmed=row['confirmed'].get_active())
            for side, row in self.rows.items()}
        return validate_geometry(data)

    def changed(self, _widget, side):
        for name, row in self.rows.items():
            if side is None or side == name:
                row['confirmed'].set_active(False)
        self.refresh()

    def refresh(self, *_):
        data = self.data()
        values = []
        for side, title in [('left', '左'), ('right', '右')]:
            offset = flange_from_tip(data, side)
            xyz = ' / '.join(f'{offset[i][3]:.2f}' for i in range(3))
            values.append(f'{title}爪：法兰到 TCP 的 X / Y / Z 偏置 {xyz} mm')
        self.details.set_text('\n'.join(values)+'\n未核对时可保存预览，但不会用于实机夹爪 TCP 遥操。')
        if self.preview:
            self.preview(data)

    def edit(self):
        try:
            if self.run() == Gtk.ResponseType.OK:
                data = self.data()
                temporary = self.path.with_suffix('.json.tmp')
                temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
                temporary.replace(self.path)
                return data
            return None
        finally:
            if self.preview:
                self.preview(load_geometry(self.path))
            self.destroy()
