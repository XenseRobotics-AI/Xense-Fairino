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
        setup_plotting()
        self.stream, self.output = stream, output
        self.closed = self.busy = False
        self.displayed = {}
        self.pick_pose = False
        self.pose_start = None
        self.mode = Gtk.Label(label='地图 · 默认查看，按需建图', xalign=0)
        self.mode.set_line_wrap(True)
        self.mode.set_max_width_chars(100)
        self.pack_start(self.mode, False, False, 0)
        actions = Gtk.Box(spacing=6)
        self.buttons = {}
        for action, title in [('start', '准备新建图'), ('pause', '暂停建图'),
                              ('resume', '继续建图'), ('save', '另存地图'), ('view', '结束建图 / 返回底图')]:
            button = Gtk.Button(label=title)
            button.connect('clicked', lambda _, a=action: self.control(a))
            actions.pack_start(button, False, False, 0)
            self.buttons[action] = button
        self.pack_start(actions, False, False, 0)
        localization_bar = Gtk.Box(spacing=6)
        self.map_choice = Gtk.ComboBoxText()
        self.map_choice.set_tooltip_text('先刷新地图列表，再明确选用地图；不会自动加载旧地图')
        localization_bar.pack_start(self.map_choice, True, True, 0)
        for action, title in [('maps', '刷新地图库'), ('use', '选用地图'),
                              ('localize', '启动定位'), ('stop-localization', '停止定位'),
                              ('initial-pose', '设置初始位姿')]:
            button = Gtk.Button(label=title)
            button.connect('clicked', lambda _, a=action: self.control(a))
            localization_bar.pack_start(button, False, False, 0)
            self.buttons[action] = button
        self.pack_start(localization_bar, False, False, 0)
        self.message = Gtk.Label(xalign=0)
        self.message.set_line_wrap(True)
        self.pack_start(self.message, False, False, 0)
        self.counters = Gtk.Label(xalign=0)
        self.counters.set_line_wrap(True)
        self.pack_start(self.counters, False, False, 0)
        self.match_status = Gtk.Label(xalign=0)
        self.match_status.set_line_wrap(True)
        self.pack_start(self.match_status, False, False, 0)
        bar = Gtk.Box(spacing=8)
        self.follow = Gtk.CheckButton(label='跟随机器人')
        self.freeze = Gtk.CheckButton(label='冻结显示（不暂停建图）')
        for title, callback in [('全图', self.fit), ('保存截图', self.save)]:
            button = Gtk.Button(label=title)
            button.connect('clicked', callback)
            bar.pack_start(button, False, False, 0)
        bar.pack_start(self.follow, False, False, 0)
        bar.pack_start(self.freeze, False, False, 0)
        self.pack_start(bar, False, False, 0)
        self.plot = MapPlot()
        self.plot.show_status_in_plot = not compact
        self.canvas = FigureCanvasGTK3Agg(self.plot.figure)
        self.canvas.set_size_request(640, 240 if compact else 420)
        if compact:
            self.plot.figure.subplots_adjust(top=.86, bottom=.16)
        self.canvas.mpl_connect('button_press_event', self.pose_press)
        self.canvas.mpl_connect('button_release_event', self.pose_release)
        self.pack_start(self.canvas, True, True, 0)
        self.toolbar = NavigationToolbar2GTK3(self.canvas)
        self.pack_start(self.toolbar, False, False, 0)
        from apps.navigation.panel import NavigationPanel
        self.navigation = NavigationPanel(self, acquire, release)
        self.pack_start(self.navigation, False, False, 0)
        if start_stream:
            stream.thread.start()
        self.tick()
        GLib.timeout_add(300, self.tick)

    def pose_press(self, event):
        if self.pick_pose and not self.busy and event.inaxes == self.plot.ax and event.button == 1 and not self.toolbar.mode:
            self.pose_start = (event.xdata, event.ydata)

    def pose_release(self, event):
        start, self.pose_start = self.pose_start, None
        if not self.pick_pose or start is None or event.inaxes != self.plot.ax or event.button != 1:
            return
        dx, dy = event.xdata-start[0], event.ydata-start[1]
        if math.hypot(dx, dy) < .05:
            self.message.set_text('请从机器人所在位置拖向机器人正前方，拖动距离至少 5 厘米。')
            return
        self.pick_pose = False
        self.pose_dialog((start[0], start[1], math.atan2(dy, dx)))

    def pose_dialog(self, suggested=None):
        if suggested is None:
            self.pose_session = (self.stream.snapshot().get('frame', {}).get('localization') or {}).get('id')
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
            if self.freeze.get_active():
                self.freeze.set_active(False)
            self.follow.set_active(False)
            if self.toolbar.mode:
                self.message.set_text('请先关闭工具栏的平移/缩放，再设置初始位姿。')
                return
            self.pick_pose = True
            self.message.set_text('在左侧地图从机器人当前位置拖向正前方，松开后核对数值再发送。')
        elif response == Gtk.ResponseType.OK:
            try:
                x, y, heading = map(float, values)
                if not all(math.isfinite(v) for v in (x, y, heading)):
                    raise ValueError()
            except ValueError:
                self.message.set_text('初始位姿未发送：请填写三个有限数值。')
                return
            self.control('initial-pose', {'x': x, 'y': y, 'yaw': math.radians(heading), 'session_id': self.pose_session})

    def control(self, action, pose=None):
        if self.busy or self.closed or self.navigation.locked():
            return
        name = None
        if action == 'initial-pose' and pose is None:
            self.pose_dialog()
            return
        self.pick_pose = False
        if action == 'use':
            name = self.map_choice.get_active_text()
            if not name:
                self.message.set_text('地图库为空或尚未选择；请先另存地图，再刷新列表。')
                return
        if action == 'save':
            dialog = Gtk.Dialog(title='另存地图', transient_for=self.get_toplevel(), modal=True)
            dialog.add_buttons('取消', Gtk.ResponseType.CANCEL, '保存', Gtk.ResponseType.OK)
            entry = Gtk.Entry(text=datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
            dialog.get_content_area().pack_start(Gtk.Label(label='新地图名：英文、数字、下划线或短横线'), False, False, 8)
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
                   'localize': mode == 'saved' and bool(frame.get('saved_map_source')) and not (frame.get('localization') or {}).get('active'),
                   'stop-localization': bool((frame.get('localization') or {}).get('active')),
                   'initial-pose': mode == 'saved' and bool((frame.get('localization') or {}).get('active'))}
        if not allowed['initial-pose']:
            self.pick_pose = False
            self.pose_start = None
        for action, button in self.buttons.items():
            button.set_sensitive(bool(allowed[action] and fresh and not self.busy and not self.navigation.locked()))
        message, color = health(state)
        self.mode.set_markup(f'<span foreground="{color}" weight="bold">{escape(message)}</span>')
        values = []
        source = frame.get('odometry_source') or frame.get('ekf_source') or '/eight_dof_chassis_controller/odom'
        output_label = '建图 odom（原始直通）' if frame.get('odometry_mode') == 'raw' else '建图 odom（EKF）'
        lag = max(0, time.monotonic() - state.get('received', time.monotonic()))
        for topic, label in [('/front/scan', '前雷达'), ('/back/scan', '后雷达'), (source, '原始 odom'), ('/odom', output_label)]:
            item = frame.get('topics', {}).get(topic, {})
            age = item.get('age')
            value = '等待' if age is None else f'{item.get("hz", 0):.1f} Hz / {age + lag:.1f} 秒前'
            values.append(label + '：' + value)
        self.counters.set_text('  |  '.join(values))
        if mapping:
            text, color = matching_health(frame, lag)
            count = (frame.get('scan_match') or {}).get('count_observed', 0)
            self.match_status.set_markup(f'<span foreground="{color}">{escape(text)} · 本次连接观察到 {count} 次匹配</span>')
        else:
            loc = frame.get('localization') or {}
            details = loc.get('reason', '')
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
