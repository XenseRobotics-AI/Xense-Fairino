"""GTK view shared by the standalone viewer and the workbench."""
import datetime
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
    def __init__(self, stream, output, start_stream=True, compact=False):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        setup_plotting()
        self.stream, self.output = stream, output
        self.closed = self.busy = False
        self.displayed = {}
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
        self.pack_start(self.canvas, True, True, 0)
        self.pack_start(NavigationToolbar2GTK3(self.canvas), False, False, 0)
        if start_stream:
            stream.thread.start()
        self.tick()
        GLib.timeout_add(300, self.tick)

    def control(self, action):
        if self.busy or self.closed:
            return
        name = None
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
        self.message.set_text('等待健康的里程计与雷达（最多 90 秒）…' if action in ('start', 'resume') else '正在处理地图…')

        def work():
            try:
                result = map_action(action, self.stream.args.host, self.stream.args.user,
                                    self.stream.control_password, name=name,
                                    wait=90 if action in ('start', 'resume') else 0)
                message = result.get('message', '完成')
                for key in ('saved_directory', 'archived_directory'):
                    if result.get(key):
                        message += '；机器人目录：' + result[key]
            except Exception as exc:
                message = '操作未完成：' + str(exc)
            GLib.idle_add(self.completed, message)
        threading.Thread(target=work, daemon=True).start()

    def completed(self, message):
        self.busy = False
        if not self.closed:
            self.message.set_text(message)
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
                   'resume': mapping and paused, 'save': mapping, 'view': mapping}
        for action, button in self.buttons.items():
            button.set_sensitive(bool(allowed[action] and fresh and not self.busy))
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
            self.match_status.set_text('')
        if not self.freeze.get_active():
            self.displayed = state
            self.plot.update(state, self.follow.get_active())
            self.canvas.draw_idle()
        return True

    def close(self):
        if self.busy:
            self.message.set_text('地图操作正在进行，请等待完成。')
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
