"""GTK view shared by the standalone viewer and the workbench."""
import datetime
import math
from html import escape
import threading
import time

import gi
from apps.workbench import gtk_support
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib
from matplotlib.backends.backend_gtk3agg import FigureCanvasGTK3Agg
from matplotlib.backends.backend_gtk3 import NavigationToolbar2GTK3

from .viewer import MapPlot, health, matching_health, map_action, save_snapshot, setup_plotting


class MapPanel(Gtk.Box):
    def __init__(self, stream, output, start_stream=True, compact=False, acquire=None, release=None):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        from .style import install_style
        install_style()
        self.set_name('map-studio')
        setup_plotting()
        self.stream, self.output = stream, output
        self.demo = bool(getattr(stream.args, 'demo', False))
        self.closed = self.busy = False
        self.displayed = {}
        self.pick_pose = False
        self.pick_goal = False
        self.pose_start = None
        self.pick_context = None
        self.base_controls = None
        self.buttons, self.layer_buttons = {}, {}

        heading = Gtk.Box(spacing=10)
        title = Gtk.Label(xalign=0)
        title.set_markup('<span size="15000" weight="bold">地图与导航</span>')
        heading.pack_start(title, False, False, 0)
        self.mode = Gtk.Label(label='等待连接', xalign=0)
        self.mode.set_line_wrap(True)
        self.mode.set_max_width_chars(70)
        heading.pack_start(self.mode, True, True, 0)
        if self.demo:
            self.scenario_choice = Gtk.ComboBoxText()
            for key, label in [('tracking', '离线演示 · 巡航'), ('candidate', '离线演示 · 定位候选'),
                               ('stale', '离线演示 · 反馈中断'), ('empty', '离线演示 · 无地图')]:
                self.scenario_choice.append(key, label)
            self.scenario_choice.set_active_id('tracking')
            self.scenario_choice.connect('changed', self.change_scenario)
            heading.pack_end(self.scenario_choice, False, False, 0)
        self.pack_start(heading, False, False, 0)

        metrics = Gtk.Box(spacing=8)
        self.metrics = {}
        for name, label in [('pose', '地图位姿 · TF'), ('localization', 'AMCL 定位'),
                            ('particles', '粒子云'), ('scans', '双雷达')]:
            card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
            card.get_style_context().add_class('map-metric')
            caption = Gtk.Label(label=label, xalign=0)
            caption.get_style_context().add_class('map-caption')
            value = Gtk.Label(label='等待数据', xalign=0)
            value.set_ellipsize(3)
            value.set_max_width_chars(40)
            card.pack_start(caption, False, False, 0)
            card.pack_start(value, False, False, 0)
            metrics.pack_start(card, True, True, 0)
            self.metrics[name] = value
        self.pack_start(metrics, False, False, 0)

        body = Gtk.Paned(orientation=Gtk.Orientation.HORIZONTAL)
        self.body_pane = body
        self.body_width = 0
        body.connect('size-allocate', self.allocate_map_width)
        self.pack_start(body, True, True, 0)
        map_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        body.pack1(map_box, resize=True, shrink=True)
        bar = Gtk.Box(spacing=6)
        self.follow = Gtk.CheckButton(label='跟随机器人')
        self.freeze = Gtk.CheckButton(label='冻结画面')
        self.freeze.set_tooltip_text('只冻结显示，不暂停建图或导航')
        for title, callback in [('全图', self.fit), ('−', lambda *_: self.zoom(1.3)),
                                ('＋', lambda *_: self.zoom(1/1.3)), ('保存截图', self.save)]:
            button = Gtk.Button(label=title)
            button.connect('clicked', callback)
            bar.pack_start(button, False, False, 0)
        bar.pack_start(self.follow, False, False, 0)
        bar.pack_end(self.freeze, False, False, 0)
        map_box.pack_start(bar, False, False, 0)
        self.plot = MapPlot()
        self.plot.show_status_in_plot = False
        self.canvas = FigureCanvasGTK3Agg(self.plot.figure)
        self.canvas.set_size_request(480, 200 if compact else 300)
        self.canvas.set_hexpand(True)
        self.canvas.set_vexpand(True)
        self.canvas.mpl_connect('button_press_event', self.pose_press)
        self.canvas.mpl_connect('button_release_event', self.pose_release)
        self.canvas.mpl_connect('motion_notify_event', self.pointer_move)
        self.canvas.mpl_connect('key_press_event', self.pick_key)
        self.canvas.mpl_connect('scroll_event', self.scroll_zoom)
        map_box.pack_start(self.canvas, True, True, 0)
        bottom = Gtk.Box(spacing=6)
        self.toolbar = NavigationToolbar2GTK3(self.canvas)
        self.toolbar.set_show_arrow(False)
        self.toolbar.set_style(Gtk.ToolbarStyle.ICONS)
        bottom.pack_start(self.toolbar, False, False, 0)
        self.cursor_label = Gtk.Label(label='地图坐标 / m · 滚轮缩放', xalign=1)
        self.cursor_label.get_style_context().add_class('map-caption')
        bottom.pack_end(self.cursor_label, True, True, 0)
        map_box.pack_start(bottom, False, False, 0)

        self.side_notebook = Gtk.Notebook()
        self.side_notebook.set_size_request(340, -1)
        self.side_notebook.set_hexpand(False)
        body.pack2(self.side_notebook, resize=False, shrink=False)
        body.set_position(900 if compact else 800)
        mapping = self.side_page('建图')
        self.section(mapping, '按需创建地图', '默认只查看地图。新建图后，使用下方底盘键控低速移动。')
        for action, title in [('start', '准备新建图'), ('pause', '暂停建图'),
                              ('resume', '继续建图'), ('save', '另存地图'), ('view', '结束建图 / 返回底图')]:
            self.action_button(mapping, action, title)
        self.section(mapping, '采集状态')
        self.counters = self.wrapped_label()
        mapping.pack_start(self.counters, False, False, 0)
        self.mapping_status = self.wrapped_label()
        mapping.pack_start(self.mapping_status, False, False, 8)

        # One workflow owns one chassis session; keep stop controls outside the scroll.
        workflow = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        workflow.set_name('map-workflow')
        self.workflow_scroll = Gtk.ScrolledWindow()
        self.workflow_scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        workflow.pack_start(self.workflow_scroll, True, True, 0)
        localization = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.workflow_scroll.add(localization)
        self.workflow_page = self.side_notebook.append_page(workflow, Gtk.Label(label='定位与导航'))
        localization.set_spacing(3)
        localization.set_border_width(8)
        self.map_choice = Gtk.ComboBoxText()
        self.map_choice.set_hexpand(True)
        self.map_choice.set_tooltip_text('选择与当前环境相符的地图，再点击选用；保存地图不会自动选用。')
        for cell in self.map_choice.get_cells():
            cell.set_property('ellipsize', 3)
            cell.set_property('max-width-chars', 13)
        row = Gtk.Box(spacing=6)
        row.pack_start(Gtk.Label(label='地图', xalign=0), False, False, 0)
        row.pack_start(self.map_choice, True, True, 0)
        self.action_button(row, 'maps', '刷新')
        self.action_button(row, 'use', '选用')
        localization.pack_start(row, False, False, 0)
        self.prepare_drive = Gtk.Button(label='① 准备底盘 · 回正并使能')
        self.prepare_drive.set_tooltip_text('定位与导航共用此连接；会回正轮组并使能，无需启动手动键控。')
        self.prepare_drive.get_style_context().add_class('suggested-action')
        self.prepare_drive.connect('clicked', lambda _: self.navigation.control('drive'))
        localization.pack_start(self.prepare_drive, False, False, 0)
        self.drive_status = self.wrapped_label()
        self.drive_status.set_lines(1)
        self.drive_status.set_ellipsize(3)
        self.drive_status.get_style_context().add_class('map-caption')
        localization.pack_start(self.drive_status, False, False, 0)
        self.section(localization, '②  2D 位姿估计 · AMCL').set_margin_top(2)
        row = Gtk.Box(spacing=6, homogeneous=True)
        self.action_button(row, 'localize', '启动 AMCL 定位', horizontal=True)
        self.action_button(row, 'stop-localization', '停止定位', horizontal=True)
        localization.pack_start(row, False, False, 0)
        row = Gtk.Box(spacing=6, homogeneous=True)
        self.action_button(row, 'initial-pose', '地图上设置初始位姿', horizontal=True)
        self.buttons['initial-pose'].set_tooltip_text('按住机器人实际位置，拖向正前方；橙色箭头预览，松开后确认发送。')
        self.pose_coordinates = Gtk.Button(label='输入坐标…')
        self.pose_coordinates.set_tooltip_text('输入机器人初始位置与朝向，供 AMCL 定位使用。')
        self.pose_coordinates.connect('clicked', lambda _: self.pose_dialog())
        row.pack_start(self.pose_coordinates, True, True, 0)
        localization.pack_start(row, False, False, 0)
        self.match_status = self.wrapped_label()

        from apps.navigation.panel import NavigationPanel
        self.section(localization, '③  导航').set_margin_top(2)
        self.navigation = NavigationPanel(self, acquire, release, integrated=True)
        self.navigation.details_box.pack_start(self.match_status, False, False, 4)
        localization.pack_start(self.navigation, False, False, 0)
        # Stop/cancel stays visible even when controls above need scrolling.
        footer = self.navigation.footer
        self.navigation.remove(footer)
        footer.set_border_width(10)
        workflow.pack_end(footer, False, False, 0)
        self.end_drive = self.navigation.buttons['end']
        display = self.side_page('图层')
        self.section(display, '地图叠加', '轮廓为真实尺寸的底盘；不包含伸出的双臂。轨迹来自 TF 位姿估计。')
        for name, title, active in [('particles', 'AMCL 粒子云', True), ('uncertainty', '定位不确定度', True),
                                     ('scans', '前后雷达扫描', True), ('footprints', 'Nav2 局部 / 全局碰撞轮廓', True), ('trail', '机器人运动轨迹', True),
                                     ('paths', '导航路径与目标', True), ('local_radar', '本体雷达小窗', False)]:
            check = Gtk.CheckButton(label=title)
            check.set_active(active)
            check.connect('toggled', lambda button, n=name: self.layer_changed(n, button.get_active()))
            display.pack_start(check, False, False, 0)
            self.layer_buttons[name] = check
            self.plot.set_layer(name, active)
        clear = Gtk.Button(label='清除显示轨迹')
        clear.connect('clicked', self.clear_trail)
        display.pack_start(clear, False, False, 8)
        self.section(display, '读图提示', '紫色：AMCL 粒子\n青色：机器人与经过轨迹\n蓝色实线：控制器当前路径\n橙色：局部预测轨迹\n紫色虚线：预览路径\n红色叉号：导航目标\n断流后轮廓变灰，表示最后位置。')
        self.overlay_status = self.wrapped_label()
        display.pack_start(self.overlay_status, False, False, 0)
        # GTK ignores selection while notebook pages are still invisible.
        workflow.show_all()
        self.side_notebook.set_current_page(self.workflow_page)

        self.base_expander = Gtk.Expander(label='手动底盘控制 · 展开键控与速度设置')
        self.base_expander.set_no_show_all(True)
        self.base_scroll = Gtk.ScrolledWindow()
        self.base_scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        self.base_scroll.set_min_content_height(175)
        self.base_scroll.set_max_content_height(180)
        self.base_scroll.set_propagate_natural_height(True)
        self.base_expander.add(self.base_scroll)
        self.pack_start(self.base_expander, False, False, 0)
        self.message = Gtk.Label(xalign=0)
        self.message.set_line_wrap(True)
        self.message.set_max_width_chars(120)
        self.message.get_style_context().add_class('map-notice')
        self.pack_start(self.message, False, False, 0)
        if self.demo:
            self.message.set_text('离线演示 · 地图、粒子与运动均为模拟数据，不连接或控制机器人。')
        if start_stream and not self.demo:
            stream.thread.start()
        self.tick()
        self.timer = GLib.timeout_add(250, self.tick)

    @staticmethod
    def wrapped_label(text=''):
        label = Gtk.Label(label=text, xalign=0)
        label.set_line_wrap(True)
        label.set_max_width_chars(32)
        return label

    def allocate_map_width(self, widget, allocation):
        if allocation.width != self.body_width:
            self.body_width = allocation.width
            widget.set_position(max(480, allocation.width-352))

    def side_page(self, title):
        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        scroll.set_propagate_natural_width(False)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        box.set_border_width(14)
        scroll.add(box)
        self.side_notebook.append_page(scroll, Gtk.Label(label=title))
        return box

    def section(self, box, title, text=None):
        label = self.wrapped_label()
        label.set_markup('<b>'+escape(title)+'</b>')
        label.set_margin_top(8)
        box.pack_start(label, False, False, 0)
        if text:
            caption = self.wrapped_label(text)
            caption.get_style_context().add_class('map-caption')
            box.pack_start(caption, False, False, 0)
        return label

    def action_button(self, box, action, title, horizontal=False):
        button = Gtk.Button(label=title)
        button.connect('clicked', lambda _: self.control(action))
        box.pack_start(button, horizontal, horizontal, 0)
        self.buttons[action] = button
        if action in ('start', 'localize', 'initial-pose'):
            button.get_style_context().add_class('suggested-action')
        return button

    def attach_base_controls(self, widget):
        parent = widget.get_parent()
        if parent is not None:
            parent.remove(widget)
        self.detach_base_controls()
        self.base_controls = widget
        widget.set_name('map-manual-controls')
        self.base_scroll.add(widget)
        self.base_expander.show()
        self.base_scroll.show_all()

    def detach_base_controls(self):
        widget = self.base_controls
        if widget is not None:
            widget.get_parent().remove(widget)
            viewport = self.base_scroll.get_child()
            if viewport is not None:
                self.base_scroll.remove(viewport)
            self.base_controls = None
        self.base_expander.hide()
        return widget

    def change_scenario(self, combo):
        self.stream.scenario = combo.get_active_id()
        self.cancel_pick()
        self.plot.clear_trail()
        self.tick()

    def layer_changed(self, name, enabled):
        self.plot.set_layer(name, enabled)
        self.canvas.draw_idle()

    def clear_trail(self, *_):
        self.plot.clear_trail()
        self.canvas.draw_idle()

    def zoom(self, factor):
        self.follow.set_active(False)
        self.plot.zoom(1/factor)
        self.canvas.draw_idle()

    def scroll_zoom(self, event):
        if event.inaxes == self.plot.ax and not self.pick_pose and not self.pick_goal:
            self.zoom(1/1.2 if event.button == 'up' else 1.2)

    def pointer_move(self, event):
        if self.valid_map_point(event):
            self.cursor_label.set_text(f'X {event.xdata: .2f}   Y {event.ydata: .2f} m')
            if self.pose_start is not None:
                if not self.pick_state_valid(self.pick_context):
                    self.cancel_pick()
                    return
                self.plot.set_pose_preview(self.pose_start, (event.xdata, event.ydata),
                                           kind='goal' if self.pick_goal else 'initial')
                self.canvas.draw_idle()

    def valid_map_point(self, event):
        return (event.inaxes == self.plot.ax and
                all(isinstance(v, (int, float)) and math.isfinite(v)
                    for v in (event.xdata, event.ydata)))

    def cancel_pick(self):
        self.pick_pose = self.pick_goal = False
        self.pose_start = self.pick_context = None
        self.plot.clear_pose_preview()
        self.canvas.draw_idle()

    def pick_key(self, event):
        if event.key == 'escape' and (self.pick_pose or self.pick_goal or self.pose_start is not None):
            self.cancel_pick()
            self.message.set_text('已取消地图选点，未发送位姿或运动目标。')

    def begin_initial_pose(self):
        self.cancel_pick()
        if not (self.stream.snapshot().get('frame', {}).get('localization') or {}).get('active'):
            self.message.set_text('请先启动 AMCL 定位，再设置初始位姿。')
            return
        self.freeze.set_active(False)
        self.follow.set_active(False)
        if self.toolbar.mode:
            self.message.set_text('请先关闭工具栏的平移/缩放，再设置初始位姿。')
            return
        self.tick()
        self.canvas.draw()
        if not self.pick_state_valid():
            return
        self.pose_session = (self.displayed.get('frame', {}).get('localization') or {}).get('id')
        self.pick_pose = True
        self.message.set_text('按住机器人当前位置，拖向正前方；橙色箭头实时预览，松开后核对并发送。右键 / Esc 取消。')

    def pick_navigation_goal(self):
        if self.busy or self.navigation.active() or self.navigation.busy:
            return
        if not (self.stream.snapshot().get('frame', {}).get('localization') or {}).get('ready'):
            self.message.set_text('请先等待定位就绪，再在地图上选择目标。')
            return
        self.cancel_pick()
        self.pick_goal = True
        self.freeze.set_active(False)
        self.follow.set_active(False)
        if self.toolbar.mode:
            self.message.set_text('请先关闭工具栏的平移/缩放，再选择目标。')
            self.pick_goal = False
            return
        self.tick()
        self.canvas.draw()
        if not self.pick_state_valid():
            self.pick_goal = False
            return
        self.message.set_text('在地图上按住目标位置，拖向目标朝向再松开；只填入目标，不会开始运动。')

    def pick_state_valid(self, expected=None):
        from .map_rendering import session_key
        state = self.stream.snapshot()
        now = time.monotonic()
        if (self.freeze.get_active() or 'error' in state or
                now-state.get('received', 0) >= 2 or not self.displayed or
                now-self.displayed.get('received', 0) >= 2):
            self.message.set_text('地图反馈已过期或画面已冻结，请恢复实时显示后重新选择。')
            return False
        token = session_key(state.get('frame', {}))
        if (token != session_key(self.displayed.get('frame', {})) or
                expected is not None and token != expected):
            self.message.set_text('地图或定位会话已变化，请等待画面刷新后重新选择。')
            return False
        if self.busy or self.navigation.locked():
            self.message.set_text('地图或导航操作正在进行，未修改目标。')
            return False
        return True

    def pose_press(self, event):
        if event.button == 3 and (self.pick_pose or self.pick_goal):
            self.cancel_pick()
            self.message.set_text('已取消地图选点，未发送位姿或运动目标。')
            return
        if (self.pick_pose or self.pick_goal) and not self.busy and event.inaxes == self.plot.ax and event.button == 1 and not self.toolbar.mode:
            self.pose_start = None
            if not self.pick_state_valid() or not self.valid_map_point(event):
                return
            self.pose_start = (event.xdata, event.ydata)
            from .map_rendering import session_key
            self.pick_context = session_key(self.displayed.get('frame', {}))
            self.plot.set_pose_preview(self.pose_start, kind='goal' if self.pick_goal else 'initial')
            self.canvas.grab_focus()
            self.canvas.draw_idle()

    def pose_release(self, event):
        start, self.pose_start = self.pose_start, None
        if not (self.pick_pose or self.pick_goal) or start is None:
            return
        if event.button != 1 or not self.valid_map_point(event):
            self.plot.clear_pose_preview()
            self.canvas.draw_idle()
            self.message.set_text('未完成选点：请在地图内按住并拖出方向，再松开。')
            return
        if not self.pick_state_valid(self.pick_context):
            self.cancel_pick()
            return
        dx, dy = event.xdata-start[0], event.ydata-start[1]
        if math.hypot(dx, dy) < .05:
            self.plot.clear_pose_preview()
            self.canvas.draw_idle()
            self.message.set_text('请拖长一点以确定朝向（地图上至少 0.05 m）；箭头长度不代表机器人要移动的距离。')
            return
        self.plot.set_pose_preview(start, (event.xdata, event.ydata),
                                   kind='goal' if self.pick_goal else 'initial')
        self.canvas.draw_idle()
        if self.pick_goal:
            self.pick_goal = False
            if not self.navigation.set_picked_target(start[0], start[1], math.atan2(dy, dx)):
                self.cancel_pick()
                self.message.set_text('导航状态已变化，目标未修改，请重新选择。')
                return
            self.side_notebook.set_current_page(self.workflow_page)
            self.message.set_text('目标已填入；请核对朝向、预览路径，再明确开始导航。')
            self.cancel_pick()
        else:
            self.pick_pose = False
            self.pose_dialog((start[0], start[1], math.atan2(dy, dx)))

    def pose_dialog(self, suggested=None):
        if suggested is None:
            self.cancel_pick()
            if not self.pick_state_valid() or not (self.stream.snapshot().get('frame', {}).get('localization') or {}).get('active'):
                return
            self.pose_session = (self.stream.snapshot().get('frame', {}).get('localization') or {}).get('id')
            from .map_rendering import session_key
            self.pick_context = session_key(self.displayed.get('frame', {}))
        dialog_context, dialog_session = self.pick_context, self.pose_session
        dialog = Gtk.Dialog(title='设置 map 中的初始位姿', transient_for=self.get_toplevel(), modal=True)
        dialog.add_buttons('取消', Gtk.ResponseType.CANCEL, '地图拖拽', Gtk.ResponseType.APPLY,
                           '发送初始位姿', Gtk.ResponseType.OK)
        box = dialog.get_content_area()
        box.pack_start(Gtk.Label(label='给出大致位置与朝向，AMCL 会继续估计；无需精确坐标。\nX/Y 单位米；朝向单位度，地图 +X 为 0°，逆时针为正。'), False, False, 8)
        entries = []
        for index, label in enumerate(('X', 'Y', '朝向')):
            row = Gtk.Box(spacing=8)
            row.pack_start(Gtk.Label(label=label), False, False, 0)
            entry = Gtk.Entry()
            if suggested is not None:
                value = math.degrees(suggested[index]) if index == 2 else suggested[index]
                entry.set_text(f'{value:.4f}')
            row.pack_start(entry, True, True, 0)
            box.pack_start(row, False, False, 4)
            entries.append(entry)
        dialog.show_all()
        response = dialog.run()
        values = [entry.get_text() for entry in entries]
        dialog.destroy()
        if response == Gtk.ResponseType.APPLY:
            self.begin_initial_pose()
        elif response == Gtk.ResponseType.OK:
            valid = self.pick_state_valid(dialog_context)
            self.cancel_pick()
            if not valid:
                return
            try:
                x, y, heading = map(float, values)
                if not all(math.isfinite(v) for v in (x, y, heading)):
                    raise ValueError()
            except ValueError:
                self.message.set_text('初始位姿未发送：请填写三个有限数值。')
                return
            self.control('initial-pose', {'x': x, 'y': y, 'yaw': math.radians(heading), 'session_id': dialog_session})
        else:
            self.cancel_pick()
            self.message.set_text('初始位姿未发送。')

    def control(self, action, pose=None):
        if self.busy or self.closed or self.navigation.locked():
            return
        if self.demo:
            self.message.set_text('离线演示不发送建图或定位指令；可切换上方场景、查看图层和选择导航目标。')
            return
        name = None
        if action == 'initial-pose' and pose is None:
            self.begin_initial_pose()
            return
        if action == 'localize' and not self.localization_inputs_ready():
            self.message.set_text('定位需要持续的里程计和雷达。请先点击“定位与导航”上方的“① 准备底盘”，等待底盘与数据就绪。')
            return
        self.cancel_pick()
        if action == 'use':
            name = self.map_choice.get_active_text()
            if not name:
                self.message.set_text('地图库为空或尚未选择；请先另存地图，再刷新列表。')
                return
        if action == 'save':
            dialog = Gtk.Dialog(title='另存地图', transient_for=self.get_toplevel(), modal=True)
            dialog.add_buttons('取消', Gtk.ResponseType.CANCEL, '保存', Gtk.ResponseType.OK)
            entry = Gtk.Entry(text=datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
            dialog.get_content_area().pack_start(Gtk.Label(label='新地图名：中文、英文、数字、下划线或短横线（1–80 字）'), False, False, 8)
            dialog.get_content_area().pack_start(entry, False, False, 8)
            dialog.show_all()
            response = dialog.run()
            name = entry.get_text().strip()
            dialog.destroy()
            if response != Gtk.ResponseType.OK or not name:
                return
        self.busy = True
        self.message.set_text('等待健康的里程计与雷达（最多 90 秒）…' if action in ('start', 'resume', 'localize') else '正在处理地图…')

        def work():
            result = {}
            try:
                result = map_action(action, self.stream.args.host, self.stream.args.user,
                                    self.stream.control_password, name=name,
                                    pose=pose, wait=90 if action in ('start', 'resume', 'localize') else 0)
                message = result.get('message', '完成')
                for key in ('saved_directory', 'archived_directory'):
                    if result.get(key):
                        message += '；机器人目录：' + result[key]
            except Exception as exc:
                message = '操作未完成：' + str(exc)
            GLib.idle_add(self.completed, message, result)
        threading.Thread(target=work, daemon=True).start()

    def completed(self, message, result=None):
        self.busy = False
        if not self.closed:
            self.message.set_text(message)
            if result is not None and 'maps' in result:
                previous = self.map_choice.get_active_text()
                self.map_choice.remove_all()
                names = result['maps']
                for name in names:
                    self.map_choice.append_text(name)
                if previous in names:
                    self.map_choice.set_active(names.index(previous))
                self.message.set_text('地图库为空，请先建图并另存。' if not names
                                      else '请选择地图，再点击“选用地图”；刷新列表不会切换底图。')
        return False

    def fit(self, *_):
        self.follow.set_active(False)
        self.plot.fit()
        self.canvas.draw_idle()

    def save(self, *_):
        try:
            self.message.set_text('截图和数据：' + str(save_snapshot(self.displayed, self.plot, self.output)))
        except Exception as exc:
            self.message.set_text('截图失败：' + str(exc))

    def localization_inputs_ready(self, state=None):
        """Reject missing inputs before starting the blocking remote operation."""
        state = self.stream.snapshot() if state is None else state
        lag = time.monotonic()-state.get('received', 0)
        if 'error' in state or lag >= 2:
            return False
        frame = state.get('frame', {})
        if self.navigation.drive is not None and not self.navigation.drive.ready:
            return False
        source = frame.get('odometry_source') or frame.get('ekf_source') or '/eight_dof_chassis_controller/odom'
        for topic in (source, '/odom', '/scan'):
            age = (frame.get('topics', {}).get(topic) or {}).get('age')
            if not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0 or age+max(0, lag) > .5:
                return False
        return True

    def tick(self):
        if self.closed:
            return False
        state = self.stream.snapshot()
        frame = state.get('frame', {})
        mode, paused = frame.get('map_mode'), frame.get('mapping_paused')
        mapping = mode == 'mapping'
        fresh = 'error' not in state and time.monotonic() - state.get('received', 0) < 2
        allowed = {'start': mode in ('saved', 'empty'), 'pause': mapping and not paused,
                   'resume': mapping and paused, 'save': mapping, 'view': mapping,
                   'maps': True, 'use': not mapping and bool(self.map_choice.get_active_text()),
                   'localize': mode == 'saved' and bool(frame.get('saved_map_source')) and not (frame.get('localization') or {}).get('active') and self.localization_inputs_ready(state),
                   'stop-localization': bool((frame.get('localization') or {}).get('active')),
                   'initial-pose': mode == 'saved' and bool((frame.get('localization') or {}).get('active'))}
        if self.pick_context is not None:
            from .map_rendering import session_key
            if (not fresh or self.freeze.get_active() or not allowed['initial-pose']
                    or self.toolbar.mode or session_key(frame) != self.pick_context
                    or self.busy or self.navigation.locked()):
                self.cancel_pick()
                self.message.set_text('地图、定位或操作状态已变化，已取消选点；未发送位姿或目标。')
        elif self.pick_pose and not allowed['initial-pose']:
            self.cancel_pick()
        for action, button in self.buttons.items():
            button.set_sensitive(bool(allowed[action] and fresh and not self.busy and not self.navigation.locked() and not self.demo))
        self.pose_coordinates.set_sensitive(self.buttons['initial-pose'].get_sensitive())
        self.prepare_drive.set_sensitive(bool(fresh and mode == 'saved' and frame.get('saved_map_source')
                                             and self.navigation.preparation_available()))
        self.drive_status.set_text(self.navigation.preparation_status(frame))
        self.drive_status.set_tooltip_text(self.drive_status.get_text())
        self.buttons['localize'].set_tooltip_text('需要持续更新的原始里程计、/odom 和雷达；可先用上方按钮准备底盘。')
        message, color = health(state)
        if self.freeze.get_active():
            message, color = '画面已冻结 · 不暂停建图或导航；取消冻结可恢复实时显示', '#a6660b'
        elif self.demo:
            message = '离线模拟 · '+message
        self.mode.set_markup(f'<span foreground="{color}" weight="bold">{escape(message)}</span>')
        pose = frame.get('pose') if fresh else None
        self.metrics['pose'].set_text(f'X {pose[0]:.2f}  Y {pose[1]:.2f} m  ·  {math.degrees(pose[2]):.1f}°'
                                     if pose else '暂无有效位置' if fresh else '反馈中断 · 保留最后位置')
        loc = frame.get('localization') or {}
        labels = {'tracking': '定位就绪', 'initializing': '正在初始化', 'uncertain': '候选 · 未收敛',
                  'waiting': '等待初始位姿', 'stopped': '未启动', 'stale': '定位更新滞后'}
        self.metrics['localization'].set_text(labels.get(loc.get('state'), '等待数据') if fresh else '状态已过期')
        particles = frame.get('particles') or {}
        self.metrics['particles'].set_text(f"{particles.get('count', 0)} 粒子 · 显示 {len(particles.get('points', []))}"
                                          if fresh and particles.get('valid') else '暂无有效粒子云')
        scan_hz = [frame.get('topics', {}).get(t, {}).get('hz', 0) for t in ('/front/scan', '/back/scan')]
        self.metrics['scans'].set_text(f'前 {scan_hz[0]:.1f} Hz / 后 {scan_hz[1]:.1f} Hz' if fresh else '数据已过期')
        overlays = ['AMCL：'+particles.get('reason', '等待 /particle_cloud')]
        if particles.get('age') is not None:
            overlays.append(f"粒子上次更新 {particles['age']:.1f} 秒前")
        for key, title in [('global', '控制器路径'), ('local', '局部预测')]:
            data = (frame.get('navigation_paths') or {}).get(key) or {}
            overlays.append(title+'：'+data.get('reason', '等待数据'))
        self.overlay_status.set_text('\n'.join(overlays))
        values = []
        source = frame.get('odometry_source') or frame.get('ekf_source') or '/eight_dof_chassis_controller/odom'
        output_label = '建图 odom（原始直通）' if frame.get('odometry_mode') == 'raw' else '建图 odom（EKF）'
        lag = max(0, time.monotonic() - state.get('received', time.monotonic()))
        for topic, label in [('/front/scan', '前雷达'), ('/back/scan', '后雷达'), (source, '原始 odom'), ('/odom', output_label)]:
            item = frame.get('topics', {}).get(topic, {})
            age = item.get('age')
            value = '等待' if age is None else f'{item.get("hz", 0):.1f} Hz / {age + lag:.1f} 秒前'
            values.append(label + '：' + value)
        self.counters.set_text('\n'.join(values))
        if mapping:
            text, color = matching_health(frame, lag)
            count = (frame.get('scan_match') or {}).get('count_observed', 0)
            markup = f'<span foreground="{color}">{escape(text)} · 本次连接观察到 {count} 次匹配</span>'
            self.mapping_status.set_markup(markup)
            self.match_status.set_markup(markup)
        else:
            self.mapping_status.set_text('尚未启动建图；已有地图保持不变。')
            loc = frame.get('localization') or {}
            details = loc.get('reason', '')
            if not loc.get('active') and loc.get('id') and details:
                details = '上次定位停止原因：'+details
            if loc.get('estimate_visible') and loc.get('pose'):
                x, y, yaw = loc['pose']
                details += f" · AMCL 估计 X={x:.3f} m，Y={y:.3f} m，朝向={math.degrees(yaw):.1f}°"
                details += f" · 本次连接已观察到 {loc.get('matches', 0)} 次位姿更新"
            if 'xy_std' in loc:
                details += f" · 估计不确定度（非实际误差）：位置 σ={loc['xy_std']:.3f} m / 朝向 σ={math.degrees(loc['yaw_std']):.1f}°"
            if frame.get('saved_map_source'):
                details += ' · 底图：' + frame['saved_map_source']
            self.match_status.set_text(details)
        if not self.freeze.get_active():
            self.displayed = state
            self.plot.update(state, self.follow.get_active())
            self.canvas.draw_idle()
        return True

    def close(self):
        if self.busy:
            self.message.set_text('地图操作正在进行，请等待完成。')
            return False
        if not self.navigation.close():
            self.message.set_text('正在结束导航并等待下使能，请等待完成后关闭。')
            return False
        self.closed = True
        self.stream.stop.set()
        return True


def window(stream, output):
    import signal
    root = Gtk.Window(title='法奥地图 · 默认查看，按需建图')
    root.set_default_size(1200, 850)
    root.set_border_width(12)
    panel = MapPanel(stream, output)
    root.add(panel)

    def close(*_):
        if panel.close():
            root.hide()
            GLib.timeout_add(100, finish)
        return True

    def finish():
        if stream.thread.is_alive():
            return True
        root.destroy()
        Gtk.main_quit()
        return False

    root.connect('delete-event', close)
    signal.signal(signal.SIGINT, close)
    signal.signal(signal.SIGTERM, close)
    root.show_all()
    Gtk.main()
