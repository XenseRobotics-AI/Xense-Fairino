"""Workbench navigation buttons. Uses the same remote API as the CLI."""
import math
import threading
import time
import queue

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib

from .desktop import Target, DriveSession, active_goal, context, execute


class NavigationPanel(Gtk.Box):
    def __init__(self, map_panel, acquire=None, release=None, integrated=False):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        self.integrated = integrated
        self.map_panel = map_panel
        self.acquire = acquire or (lambda *_: True)
        self.release = release or (lambda *_: None)
        self.target = Target()
        self.drive = None
        self.drive_message = '导航底盘未接入'
        self.lease = False
        self.busy = self.polling = self.closed = self.shutdown = False
        self.state, self.received = {}, 0.
        self.events = queue.Queue()
        self.command_lock = threading.Lock()
        self.next_poll = 0.
        self.generation = 0
        self.owned = False
        self.uncertain_goal = False
        self.reported_failure = None
        self.buttons = {}
        self.set_border_width(4)
        for action, title in [('drive', '① 准备底盘 · 回正并使能'),
                              ('start', '启动导航')]:
            button = Gtk.Button(label=title)
            button.connect('clicked', lambda _, a=action: self.control(a))
            if not (integrated and action == 'drive'):
                self.pack_start(button, False, False, 0)
            self.buttons[action] = button
        self.target_coordinates = Gtk.Expander(label='目标坐标 · 未设置')
        self.target_coordinates.set_expanded(not integrated)
        grid = Gtk.Grid(column_spacing=8, row_spacing=6)
        self.entries = []
        for index, label in enumerate(('X / m', 'Y / m', '朝向 / °')):
            grid.attach(Gtk.Label(label=label, xalign=0), 0, index, 1, 1)
            entry = Gtk.Entry()
            entry.set_width_chars(9)
            entry.set_hexpand(True)
            entry.set_placeholder_text('0' if index == 2 else '')
            if index == 2:
                entry.set_text('0')
            entry.connect('changed', self.edit_target)
            grid.attach(entry, 1, index, 1, 1)
            self.entries.append(entry)
        self.target_coordinates.add(grid)
        if not integrated:
            self.pack_start(self.target_coordinates, False, False, 0)
        target_actions = Gtk.Box(spacing=6, homogeneous=True) if integrated else self
        if integrated:
            self.pack_start(target_actions, False, False, 0)
        for action, title in [('pick', '在地图上选择目标'), ('plan', '预览路径')]:
            button = Gtk.Button(label=title)
            button.connect('clicked', lambda _, a=action: self.control(a))
            target_actions.pack_start(button, integrated, integrated, 0)
            self.buttons[action] = button
        if integrated:
            self.pack_start(self.target_coordinates, False, False, 0)
        self.footer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        row = Gtk.Box(spacing=6, homogeneous=True)
        for action, title in [('goal', '开始导航'), ('cancel', '取消目标')]:
            button = Gtk.Button(label=title)
            button.connect('clicked', lambda _, a=action: self.control(a))
            button.get_style_context().add_class('suggested-action' if action == 'goal' else 'destructive-action')
            row.pack_start(button, True, True, 0)
            self.buttons[action] = button
        self.footer.pack_start(row, False, False, 4)
        button = Gtk.Button(label='结束导航并下使能')
        button.get_style_context().add_class('destructive-action')
        button.connect('clicked', lambda _: self.control('end'))
        self.buttons['end'] = button
        self.footer.pack_start(button, False, False, 0)
        self.pack_start(self.footer, False, False, 0)
        hint = Gtk.Label(label='0° 沿地图 +X。选点或输入仅设置目标，点击“开始导航”才执行。', xalign=0)
        hint.set_line_wrap(True)
        hint.set_max_width_chars(32)
        details = self
        if integrated:
            expander = Gtk.Expander(label='定位 / 导航详情与操作提示')
            self.details_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
            expander.add(self.details_box)
            self.pack_start(expander, False, False, 0)
            details = self.details_box
        details.pack_start(hint, False, False, 4)
        self.status = Gtk.Label(label='导航未接入；先选择地图。', xalign=0)
        self.status.set_line_wrap(True)
        self.status.set_max_width_chars(32)
        if integrated:
            self.footer.pack_start(self.status, False, False, 0)
            self.footer.reorder_child(self.status, 0)
        else:
            details.pack_start(self.status, False, False, 0)
        self.message = Gtk.Label(xalign=0)
        self.message.set_line_wrap(True)
        self.message.set_max_width_chars(32)
        details.pack_start(self.message, False, False, 0)
        if integrated:
            self.set_border_width(0)
            self.status.set_lines(1)
            self.status.set_ellipsize(3)
            self.message.set_lines(2)
            self.message.set_ellipsize(3)
            for label in (self.status, self.message):
                label.connect('notify::label', lambda label, _: label.set_tooltip_text(label.get_text()))
            # Action results remain visible even when the detail expander is closed.
            self.message.connect('notify::label', lambda label, _: self.map_panel.message.set_text(label.get_text()))
        # Hardware actions are unavailable from the very first demo frame.
        if self.is_demo():
            self.tick()
        GLib.timeout_add(100, self.tick)

    def is_demo(self):
        return getattr(self.map_panel.stream.args, 'demo', False) is True

    def credentials(self):
        stream = self.map_panel.stream
        return stream.args.host, stream.args.user, stream.control_password

    def frame(self):
        return self.map_panel.stream.snapshot().get('frame', {})

    def fresh_map(self):
        state = self.map_panel.stream.snapshot()
        return 'error' not in state and time.monotonic()-state.get('received', 0) < 2

    def active(self):
        return (active_goal(self.state) and not self.state.get('stale') or self.uncertain_goal or
                self.target.goal_id is not None and self.target.label in ('已接收', '执行中', '取消中'))

    def locked(self):
        # Prevent map/pose mutations while navigation may own a live goal.
        return self.busy or self.active() or self.shutdown

    def preparation_available(self):
        """One guarded startup entry, shared by localization and navigation."""
        return bool(not self.is_demo() and not self.closed and not self.busy
                    and not self.shutdown and not self.map_panel.busy
                    and self.drive is None and not self.active())

    def preparation_status(self, frame=None):
        """Explain current motor readiness without reusing a past AMCL error."""
        if self.is_demo():
            return '离线演示，不连接或使能底盘。'
        if self.shutdown:
            return '正在结束导航，等待底盘停止并下使能。'
        if self.drive is None:
            return self.drive_message+'；定位与导航共用底盘，请先准备底盘。'
        if not self.drive.ready:
            return self.drive.message
        if not self.fresh_map():
            return '底盘控制已就绪；地图 / 定位反馈未更新，请等待连接恢复。'
        frame = self.frame() if frame is None else frame
        loc = frame.get('localization') or {}
        if not loc.get('active'):
            return '底盘已就绪，请在第②步启动 AMCL 并设置初始位姿。'
        if not loc.get('ready'):
            return '底盘已就绪；请在第②步设置初始位姿并等待定位就绪。'
        return '底盘与定位已就绪；可在第③步启动导航并选择目标。'

    def draw_target(self):
        if hasattr(self, 'target_coordinates'):
            pose = self.target.pose
            self.target_coordinates.set_label(
                f"目标坐标 · {pose['x']:.2f}, {pose['y']:.2f} m / {math.degrees(pose['yaw']):.1f}°"
                if pose else '目标坐标 · 未设置（可展开输入）')
        self.map_panel.plot.set_navigation(self.target.pose, self.target.points, self.target.label)
        self.map_panel.canvas.draw_idle()

    def edit_target(self, *_):
        if len(self.entries) != 3 or self.busy or self.active():
            return
        try:
            self.target.set([entry.get_text() for entry in self.entries], self.frame())
            self.message.set_text('目标已标为红色 ❌；可先预览路径，再开始导航。')
            # Include an off-screen target in the map view.
            self.map_panel.follow.set_active(False)
            self.map_panel.plot.set_navigation(self.target.pose, [], self.target.label)
            self.map_panel.plot.fit()
        except ValueError:
            self.target.clear()
            self.message.set_text('请填写完整目标坐标，并先选用地图、启动定位。')
        self.draw_target()

    def set_picked_target(self, x, y, yaw):
        if self.locked() or self.map_panel.busy:
            return False
        values = (x, y, yaw)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            self.message.set_text('地图目标无效，请重新选择。')
            return False
        for entry, value in zip(self.entries, (x, y, math.degrees(yaw))):
            entry.set_text(f'{value:.4f}')
        return self.target.pose is not None

    def submit(self, action, pose=None):
        if self.is_demo():
            self.message.set_text('离线演示，不发送导航指令。')
            return
        self.busy = True
        self.generation += 1
        self.message.set_text({'plan': '正在计算路径…', 'goal': '正在发送导航目标…',
                               'start': '正在激活已准备好的导航节点…', 'cancel': '正在取消目标…'}.get(action, '处理中…'))
        creds = self.credentials()
        binding = self.target.context
        def work():
            try:
                with self.command_lock:
                    result = execute(action, *creds, pose=pose)
                self.events.put(('result', (action, binding, result, None)))
            except Exception as exc:
                self.events.put(('result', (action, binding, {}, str(exc))))
        threading.Thread(target=work, daemon=True).start()

    def control(self, action):
        if self.closed or self.shutdown:
            return
        if self.is_demo():
            if action == 'pick':
                self.map_panel.pick_navigation_goal()
            elif action == 'plan':
                try:
                    pose = self.target.request(self.frame())
                    self.target.points = self.map_panel.stream.preview_path(pose)
                    self.target.label = '离线演示路径'
                    self.message.set_text('离线演示路径，仅展示示例路线，不会执行。')
                    self.draw_target()
                except (ValueError, AttributeError) as exc:
                    self.message.set_text('无法预览离线目标：'+str(exc))
            else:
                self.message.set_text('离线演示，不连接、使能或移动机器人。')
            return
        if action == 'end':
            self.end()
            return
        if self.busy:
            return
        if action == 'pick':
            if not self.active() and not self.map_panel.busy and context(self.frame()):
                self.map_panel.pick_navigation_goal()
            return
        if action == 'drive':
            # Shared entry; do not rely on a disabled widget for safety.
            if not self.preparation_available():
                self.message.set_text(self.preparation_status())
                return
            if not self.acquire('base', '导航'):
                return
            self.lease = True
            self.drive = DriveSession(*self.credentials())
            self.drive.thread.start()
        elif action in ('plan', 'goal'):
            try:
                if not self.fresh_map():
                    raise ValueError('地图反馈已过期，请等待连接恢复')
                pose = self.target.request(self.frame())
                if action == 'goal' and (not self.drive or not self.drive.ready):
                    raise ValueError('请先接入导航底盘并等待就绪')
                if self.active():
                    raise ValueError('请先取消当前目标')
                if action == 'goal':
                    self.uncertain_goal = True
                    self.target.label = '发送中'
                    self.draw_target()
                self.submit(action, pose)
            except ValueError as exc:
                self.message.set_text(str(exc))
                self.draw_target()
        else:
            if action == 'start':
                if not self.drive or not self.drive.ready:
                    self.message.set_text('请先结束手动底盘控制，再点击上方“① 准备底盘”并等待就绪。')
                    return
                if not self.fresh_map() or not (self.frame().get('localization') or {}).get('ready'):
                    self.message.set_text('请保持定位开启，等待定位就绪后再启动导航。')
                    return
                self.owned = True
            self.submit(action)

    def end(self):
        if self.is_demo():
            self.message.set_text('离线演示，不发送导航指令。')
            return
        if self.shutdown:
            return
        self.shutdown = True
        self.generation += 1
        if self.drive:
            self.drive.stop()
        self.message.set_text('正在停止导航并等待底盘下使能…')
        creds = self.credentials()
        def work():
            errors = []
            # Run after any outstanding goal request; never stop before a late send.
            with self.command_lock:
                for action in ('cancel', 'stop'):
                    try:
                        execute(action, *creds)
                    except Exception as exc:
                        errors.append(action+': '+str(exc))
            self.events.put(('ended', (errors, any(e.startswith('stop:') for e in errors))))
        threading.Thread(target=work, daemon=True).start()

    def poll(self):
        if self.is_demo():
            return
        self.polling = True
        generation = self.generation
        creds = self.credentials()
        def work():
            try:
                value = execute('status', *creds)
                self.events.put(('status', (generation, value, None)))
            except Exception as exc:
                self.events.put(('status', (generation, {}, str(exc))))
        threading.Thread(target=work, daemon=True).start()

    def tick(self):
        if self.closed:
            return False
        if self.is_demo():
            frame = self.frame()
            if self.target.reconcile(frame):
                self.draw_target()
            for name, button in self.buttons.items():
                button.set_sensitive(bool(name in ('pick', 'plan') and context(frame)
                                          and (name == 'pick' or self.target.pose)))
            self.status.set_text('离线演示 · 目标：'+(self.target.label or '未设置'))
            return True
        now = time.monotonic()
        if self.drive:
            self.drive.pulse()
            if self.drive.done.is_set():
                self.drive_message = self.drive.message
                self.message.set_text(self.drive_message)
                self.drive = None
                if self.lease:
                    self.release('base', '导航')
                    self.lease = False
        new_failure = None
        while not self.events.empty():
            kind, value = self.events.get_nowait()
            if kind == 'status':
                self.polling = False
                generation, state, error = value
                if generation != self.generation or self.busy or self.shutdown:
                    continue
                self.state = state
                self.received = now if not error else 0.
                if error:
                    self.status.set_text('导航状态不可用：'+error[-250:])
                if not error and not self.state.get('stale') and not self.busy:
                    self.uncertain_goal = False
                last = self.state.get('last_goal') or {}
                if not error and not self.state.get('stale') and self.target.goal_id and last.get('id') == self.target.goal_id:
                    self.target.label = {1: '已接收', 2: '执行中', 3: '取消中',
                                         4: '已到达', 5: '已取消', 6: '未完成'}.get(last.get('status'), '状态未知')
                    failure = self.state.get('failure') or {}
                    if failure.get('goal') == self.target.goal_id:
                        key = (failure['goal'], failure.get('reason'))
                        if key != getattr(self, 'reported_failure', None):
                            self.reported_failure = key
                            new_failure = '导航保护停止：'+failure.get('reason', '请查看导航记录')
                    if failure.get('goal') == self.target.goal_id and last.get('status') in (5, 6):
                        self.target.label = '保护停止'
                    self.draw_target()
            elif kind == 'result':
                self.busy = False
                action, binding, result, error = value
                self.received = 0.  # Require a new status after each operation.
                self.next_poll = 0.
                self.message.set_text('操作未完成：'+error if error else result.get('message', '完成'))
                if action == 'goal' and error:
                    self.target.label = '结果待核对'
                    # No retry until fresh status or explicit cancel.
                if binding == self.target.context:
                    if action == 'plan' and not error:
                        self.target.points = result.get('points', [])
                    if action == 'goal' and not error:
                        self.target.goal_id = result['goal_id']
                        self.target.label = '已接收'
                    if action == 'cancel' and not error:
                        self.target.label = '取消中'
                    self.draw_target()
            elif kind == 'ended':
                self.shutdown = False
                errors, stop_failed = value
                self.owned = stop_failed
                self.state, self.received = {}, 0.
                self.uncertain_goal = stop_failed
                self.target.points = []
                self.target.label = '停止结果待核对' if stop_failed else '已结束'
                self.draw_target()
                self.message.set_text('导航已停止；底盘下使能结果见状态。' if not errors
                                      else '停止反馈：'+'；'.join(errors))
        frame = self.frame()
        if self.target.reconcile(frame):
            self.draw_target()
            self.message.set_text('地图或定位会话已变化，已清除旧目标和预览路径。')
        if new_failure:
            # Preserve the actual stop cause even if that same input failure
            # invalidates the localization session and clears the target.
            self.message.set_text(new_failure)
        fresh = now-self.received < 3 and not self.state.get('stale')
        ready = fresh and self.state.get('localization_ready') and self.state.get('controller_ready')
        if self.received:
            drive = self.preparation_status(frame)
            nav = ('状态过期 / 导航未启动' if not fresh else '就绪' if ready
                   else self.state.get('localization_reason', '等待启动 / 就绪'))
            loc = frame.get('localization') or {}
            if fresh and self.fresh_map() and not loc.get('active'):
                nav = '定位未启动'
                if loc.get('id') and loc.get('reason'):
                    nav += '；上次定位停止原因：'+str(loc['reason'])
            failure = self.state.get('failure') or {}
            if fresh and failure.get('goal') == self.target.goal_id and self.target.goal_id:
                nav = failure.get('reason', '保护停止')
            prefix = '' if getattr(self, 'integrated', False) else drive+' · '
            self.status.set_text(prefix+'导航：'+str(nav)+' · 目标：'+(self.target.label or '未设置'))
        elif self.drive:
            self.status.set_text(self.preparation_status(frame)+' · 等待导航状态')
        free = not self.busy and not self.shutdown and not self.map_panel.busy
        self.buttons['drive'].set_sensitive(self.preparation_available())
        self.buttons['start'].set_sensitive(bool(free and self.drive and self.drive.ready and context(frame) and self.fresh_map()
                                                       and (frame.get('localization') or {}).get('ready') and not self.active()))
        self.buttons['end'].set_sensitive(bool(not self.shutdown and (self.drive or self.owned or self.active() or self.state.get('running'))))
        self.buttons['cancel'].set_sensitive(free and (self.active() or self.state.get('running', False)))
        for name in ('plan', 'goal'):
            self.buttons[name].set_sensitive(bool(free and ready and self.fresh_map() and self.target.pose
                                                 and not self.active()
                                                 and (name == 'plan' or self.drive and self.drive.ready)))
        if 'pick' in self.buttons:
            self.buttons['pick'].set_sensitive(bool(free and not self.active() and self.fresh_map() and context(frame)))
        for entry in self.entries:
            entry.set_sensitive(free and not self.active())
        if now >= self.next_poll and not self.polling and not self.busy and not self.shutdown:
            self.next_poll = now+1.
            self.poll()
        return True

    def close(self):
        if self.is_demo():
            self.closed = True
            return True
        if self.busy or self.shutdown or self.drive or self.owned or self.active():
            waiting_drive = self.drive and self.drive.stop_event.is_set() and not self.busy and not self.owned and not self.active()
            if not self.shutdown and not waiting_drive:
                self.end()
            return False
        self.closed = True
        return True
