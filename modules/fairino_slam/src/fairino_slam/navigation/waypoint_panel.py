"""Named-pose controls; persistence runs off the GUI heartbeat thread."""
import math
import queue
import threading

from gi.repository import Gtk

from .waypoints import WaypointStore, capture, fresh_frame, map_binding


class WaypointPanel(Gtk.Box):
    def __init__(self, navigation, store=None):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        self.navigation = navigation
        self.store = store or WaypointStore()
        self.binding = None
        self.records = []
        self.pending = False
        self.loaded_binding = None
        self.events = queue.Queue()
        row = Gtk.Box(spacing=6)
        row.pack_start(Gtk.Label(label='位置点名称', xalign=0), False, False, 0)
        self.name = Gtk.Entry()
        self.name.set_width_chars(12)
        self.name.set_placeholder_text('例如：取料位')
        row.pack_start(self.name, True, True, 0)
        self.save_button = Gtk.Button(label='保存当前位置与朝向')
        self.save_button.connect('clicked', self.save)
        row.pack_start(self.save_button, False, False, 0)
        self.pack_start(row, False, False, 0)
        row = Gtk.Box(spacing=6)
        self.choice = Gtk.ComboBoxText()
        row.pack_start(self.choice, True, True, 0)
        self.use_button = Gtk.Button(label='设为导航目标')
        self.use_button.connect('clicked', self.use)
        row.pack_start(self.use_button, False, False, 0)
        self.refresh_button = Gtk.Button(label='刷新位置点')
        self.refresh_button.connect('clicked', lambda *_: self.refresh())
        row.pack_start(self.refresh_button, False, False, 0)
        self.delete_button = Gtk.Button(label='删除点')
        self.delete_button.connect('clicked', self.delete)
        row.pack_start(self.delete_button, False, False, 0)
        self.pack_start(row, False, False, 0)
        self.info = Gtk.Label(label='位置点按机器人和地图保存；选择后点击“开始导航”执行。', xalign=0)
        self.info.set_line_wrap(True)
        self.pack_start(self.info, False, False, 0)

    def current(self):
        nav = self.navigation
        snapshot = nav.map_panel.stream.snapshot()
        host, user, _ = nav.credentials()
        frame = fresh_frame(snapshot)
        return snapshot, frame, map_binding(frame, host, user)

    def work(self, operation, message='', selected=None):
        if self.pending:
            return
        self.pending = True
        binding = self.binding
        def run():
            try:
                records = operation()
                self.events.put((binding, records, message, selected, None))
            except Exception as exc:
                self.events.put((binding, [], '', None, str(exc)))
        threading.Thread(target=run, daemon=True).start()

    def refresh(self):
        if self.binding is not None:
            binding = self.binding
            self.work(lambda: self.store.list(binding))

    def selected(self):
        key = self.choice.get_active_id()
        return next((p for p in self.records if p['id'] == key), None)

    def available(self):
        nav = self.navigation
        return not self.pending and not nav.locked() and not nav.map_panel.busy and not nav.closed

    def save(self, *_):
        if not self.available():
            return
        try:
            snapshot, _, binding = self.current()
            if binding != self.binding:
                raise ValueError('地图已切换，请等待位置点列表刷新')
            host, user, _ = self.navigation.credentials()
            point = capture(self.name.get_text(), snapshot, host, user)
            self.work(lambda: self.store.save(point), '已保存“'+point['name']+'”的位置与朝向', point['id'])
        except ValueError as exc:
            self.info.set_text(str(exc))

    def use(self, *_):
        if not self.available():
            return
        try:
            _, frame, binding = self.current()
            point = self.selected()
            if point is None or point['map'] != binding:
                raise ValueError('请先选择当前地图的已保存点')
            self.navigation.use_waypoint(point, frame)
            self.info.set_text('已选“'+point['name']+'”为目标；到达后对齐保存朝向。点击“开始导航”执行。')
        except ValueError as exc:
            self.info.set_text(str(exc))

    def delete(self, *_):
        if not self.available():
            return
        point = self.selected()
        if point is None:
            return
        dialog = Gtk.MessageDialog(transient_for=self.get_toplevel(), modal=True,
            message_type=Gtk.MessageType.QUESTION, buttons=Gtk.ButtonsType.OK_CANCEL,
            text='删除已保存点“'+point['name']+'”？')
        try:
            if dialog.run() != Gtk.ResponseType.OK:
                return
        finally:
            dialog.destroy()
        # A nested dialog loop can process a map/session change.
        try:
            _, _, binding = self.current()
            if binding != point['map'] or not self.available():
                raise ValueError('当前状态已变化，未删除')
            self.work(lambda: self.store.delete(point['id'], binding), '已删除“'+point['name']+'”')
        except ValueError as exc:
            self.info.set_text(str(exc))

    def tick(self):
        while not self.events.empty():
            binding, records, message, selected, error = self.events.get_nowait()
            self.pending = False
            if binding != self.binding:
                continue
            self.loaded_binding = binding
            self.records = records
            active = selected or self.choice.get_active_id()
            self.choice.remove_all()
            for p in records:
                pose = p['pose']
                label = f"{p['name']}  ({pose['x']:.2f}, {pose['y']:.2f}) m · {math.degrees(pose['yaw']):.1f}°"
                self.choice.append(p['id'], label)
            if not self.choice.set_active_id(active or '') and records:
                self.choice.set_active(0)
            self.info.set_text(('位置点操作失败：'+error) if error else message or
                               f'当前地图已保存 {len(records)} 个点；选择后设为目标。')
        try:
            _, frame, binding = self.current()
        except ValueError:
            binding, frame = None, {}
        if binding != self.binding:
            self.binding = binding
            self.loaded_binding = None
            self.records = []
            self.choice.remove_all()
            self.info.set_text('等待当前地图的位置点…' if binding else '请连接并选用地图后管理位置点。')
        # A former map's disk task may have finished after the map changed.
        if binding is not None and not self.pending and self.loaded_binding != binding:
            self.refresh()
        free = self.available() and binding is not None
        loc = frame.get('localization') or {}
        self.save_button.set_sensitive(bool(free and loc.get('ready') and frame.get('pose')))
        self.use_button.set_sensitive(bool(free and self.records and loc.get('active')))
        self.delete_button.set_sensitive(bool(free and self.records))
        self.refresh_button.set_sensitive(bool(free))
        self.choice.set_sensitive(bool(free))
        self.name.set_sensitive(bool(free))
