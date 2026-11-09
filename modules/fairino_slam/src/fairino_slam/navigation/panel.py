"""Standalone navigation panel using the same remote API as the CLI."""
import threading
import math
import time
import queue

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib

from .desktop import Target, DriveSession, active_goal, context, execute, navigation_status
from .client import configured_heading_mode
from .recovery import RecoverySession


class NavigationPanel(Gtk.Box):
    RECOVERY_OWNER = '恢复接入'

    def __init__(self, map_panel, acquire=None, release=None):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        self.map_panel = map_panel
        self.acquire = acquire or (lambda *_: True)
        self.release = release or (lambda *_: None)
        self.target = Target(configured_heading_mode())
        self.drive = None
        self.drive_message = '导航底盘未接入'
        self.lease = False
        self.busy = self.polling = self.closed = self.shutdown = False
        self.state, self.received = {}, 0.
        self.events = queue.Queue()
        self.command_lock = threading.Lock()
        self.recovery = None
        self.recovery_lease = self.recovery_reserved = False
        self.next_poll = 0.
        self.generation = 0
        self.owned = False
        self.uncertain_goal = False
        self.buttons = {}
        self.setting_waypoint = False
        row = Gtk.Box(spacing=6)
        for action, title in [('drive', '① 接入导航底盘 · 回正使能'),
                              ('start', '② 启动导航'), ('end', '结束导航并下使能'),
                              ('recover', '恢复底盘接入（不使能）')]:
            button = Gtk.Button(label=title)
            button.connect('clicked', lambda _, a=action: self.control(a))
            row.pack_start(button, False, False, 0)
            self.buttons[action] = button
        self.pack_start(row, False, False, 0)
        recovery_hint = Gtk.Label(
            label='恢复会结束当前底盘控制并暂停里程计；建图时请先保存地图。'
                  '完成后需重新接入并按提示初始化定位，不会自动使能。',
            xalign=0)
        recovery_hint.set_line_wrap(True)
        self.pack_start(recovery_hint, False, False, 0)
        row = Gtk.Box(spacing=6)
        self.entries = []
        for label in ('X / m', 'Y / m', '朝向 / °'):
            row.pack_start(Gtk.Label(label=label), False, False, 0)
            entry = Gtk.Entry()
            entry.set_width_chars(7)
            entry.set_placeholder_text('必填' if label == '朝向 / °' else '')
            entry.connect('changed', self.edit_target)
            row.pack_start(entry, False, False, 0)
            self.entries.append(entry)
        for action, title in [('plan', '预览路径'), ('goal', '③ 开始导航'), ('cancel', '取消目标')]:
            button = Gtk.Button(label=title)
            button.connect('clicked', lambda _, a=action: self.control(a))
            row.pack_start(button, False, False, 0)
            self.buttons[action] = button
        self.pack_start(row, False, False, 0)
        heading_hint = ('行进时保持启动朝向，到达位置后对齐目标朝向。' if self.target.heading_mode == 'hold_initial'
                        else '行进时朝向沿路径调整，接近终点时转向目标的 map 绝对朝向（0° 沿 +X，90° 沿 +Y）。')
        hint = Gtk.Label(label='目标位置为 map 绝对坐标；'+heading_hint+'红色 ❌ 为目标，输入后不会自动移动。', xalign=0)
        hint.set_line_wrap(True)
        self.pack_start(hint, False, False, 0)
        from .waypoint_panel import WaypointPanel
        self.waypoint_panel = WaypointPanel(self)
        self.pack_start(self.waypoint_panel, False, False, 0)
        self.status = Gtk.Label(label='导航未接入；先选择地图。', xalign=0)
        self.status.set_line_wrap(True)
        self.pack_start(self.status, False, False, 0)
        self.message = Gtk.Label(xalign=0)
        self.message.set_line_wrap(True)
        self.pack_start(self.message, False, False, 0)
        GLib.timeout_add(100, self.tick)

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
        return self.busy or self.active() or self.shutdown or self.recovery is not None

    def draw_target(self):
        self.map_panel.plot.set_navigation(self.target.pose, self.target.points, self.target.label)
        self.map_panel.canvas.draw_idle()

    def edit_target(self, *_):
        if len(self.entries) != 3 or self.busy or self.active() or self.setting_waypoint:
            return
        try:
            self.target.set([entry.get_text() for entry in self.entries], self.frame())
            self.message.set_text('目标已标为红色 ❌；可先预览路径，再开始导航。')
            # Include an off-screen target in the map view.
            self.map_panel.follow.set_active(False)
            self.map_panel.plot.set_navigation(self.target.pose, [], self.target.label)
            self.map_panel.plot.fit()
        except ValueError as exc:
            self.target.clear()
            self.message.set_text(str(exc))
        self.draw_target()

    def use_waypoint(self, point, frame):
        if self.locked() or self.map_panel.busy:
            raise ValueError('当前导航正在执行操作，请结束后再选择位置点')
        self.target.set_saved(point, frame)
        self.setting_waypoint = True
        try:
            for entry, key in zip(self.entries[:2], ('x', 'y')):
                entry.set_text(format(self.target.pose[key], '.15g'))
            self.entries[2].set_text(format(math.degrees(self.target.pose['yaw']), '.10g'))
        finally:
            self.setting_waypoint = False
        self.map_panel.follow.set_active(False)
        self.draw_target()
        self.map_panel.plot.fit()
        self.message.set_text('已设置保存点“'+point['name']+'”；可预览路径，再开始导航。')

    def submit(self, action, pose=None):
        self.busy = True
        self.generation += 1
        generation = self.generation
        self.message.set_text({'plan': '正在计算路径…', 'goal': '正在发送导航目标…',
                               'start': '正在激活已准备好的导航节点…', 'cancel': '正在取消目标…'}.get(action, '处理中…'))
        creds = self.credentials()
        binding = self.target.context
        def work():
            try:
                with self.command_lock:
                    # A recovery may supersede work queued before this lock was acquired.
                    if generation != self.generation:
                        return
                    result = execute(action, *creds, pose=pose)
                self.events.put(('result', (generation, action, binding, result, None)))
            except Exception as exc:
                self.events.put(('result', (generation, action, binding, {}, str(exc))))
        threading.Thread(target=work, daemon=True).start()

    def control(self, action):
        if self.closed:
            return
        if action == 'recover':
            self.begin_recovery()
            return
        if self.recovery is not None or self.shutdown:
            return
        if action == 'end':
            self.end()
            return
        if self.busy:
            return
        if action == 'drive':
            if self.drive or not self.acquire('base', '导航'):
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
                    self.message.set_text('请先结束手动底盘控制，再点击①接入导航底盘并等待就绪。')
                    return
                if not self.fresh_map() or not (self.frame().get('localization') or {}).get('ready'):
                    self.message.set_text('请保持定位开启，等待定位就绪后再启动导航。')
                    return
                self.owned = True
            self.submit(action)

    def begin_recovery(self):
        if self.closed or self.recovery is not None:
            return
        if self.map_panel.busy:
            self.message.set_text('地图操作正在进行，请等待完成后恢复底盘接入。')
            return
        ownership = self.map_panel.chassis_ownership
        if not ownership.reserve(self.RECOVERY_OWNER):
            self.message.set_text('已有恢复操作正在进行，请等待完成。')
            return
        self.recovery_reserved = True
        self.generation += 1
        self.busy, self.shutdown = True, False
        self.state, self.received = {}, 0.
        self.uncertain_goal = True
        self.target.clear()
        self.target.label = '恢复中；旧目标已清除'
        self.draw_target()
        drive = self.drive
        mapping = self.map_panel.mapping_control
        if drive is not None:
            drive.stop()
        mapping.stop()
        self.message.set_text('正在结束旧控制会话；恢复完成后需手动重新接入，不会自动行走。')
        try:
            self.recovery = RecoverySession(
                *self.credentials(), command_lock=self.command_lock,
                local_ready=lambda: (drive is None or drive.done.is_set()) and not mapping.busy)
            self.recovery.thread.start()
        except Exception as exc:
            self.recovery = None
            self.busy = False
            ownership.unreserve(self.RECOVERY_OWNER)
            self.recovery_reserved = False
            self.message.set_text('恢复未开始：'+str(exc))

    def tick_recovery(self):
        task = self.recovery
        if task is None:
            return
        if task.ready_to_acquire.is_set() and not task.done.is_set() and not task.authorized.is_set():
            if self.acquire('base', self.RECOVERY_OWNER):
                self.recovery_lease = True
                task.authorize()
            else:
                task.reject('底盘仍由其他会话占用，未发送恢复请求')
        if not task.done.is_set():
            self.message.set_text(task.message)
            return
        if self.recovery_lease:
            self.release('base', self.RECOVERY_OWNER)
            self.recovery_lease = False
        if self.recovery_reserved:
            self.map_panel.chassis_ownership.unreserve(self.RECOVERY_OWNER)
            self.recovery_reserved = False
        success = task.error is None and bool(task.report and task.report.get('ok'))
        self.recovery = None
        self.busy = self.shutdown = False
        self.owned = self.uncertain_goal = not success
        self.state, self.received, self.next_poll = {}, 0., 0.
        self.target.clear()
        self.target.label = '已恢复；请重新设置目标' if success else '恢复未完成；旧目标已清除'
        self.draw_target()
        if success:
            self.drive_message = '底盘已恢复接入条件，当前下使能'
        lines = []
        if task.report:
            from fairino_slam.maintenance.recovery import report_lines
            lines = report_lines(task.report)
        text = ('恢复完成：请重新接入底盘，并按提示初始化定位；不会自动继续旧目标。' if success
                else '恢复未完成：'+str(task.error or '最终检查未通过'))
        if lines:
            text += '\n'+'\n'.join(lines)
        self.message.set_text(text)

    def end(self):
        if self.recovery is not None or self.shutdown:
            return
        self.shutdown = True
        self.generation += 1
        generation = self.generation
        if self.drive:
            self.drive.stop()
        self.message.set_text('正在停止导航并等待底盘下使能…')
        creds = self.credentials()
        def work():
            errors = []
            # Run after any outstanding goal request; never stop before a late send.
            with self.command_lock:
                if generation != self.generation:
                    return
                for action in ('cancel', 'stop'):
                    try:
                        execute(action, *creds)
                    except Exception as exc:
                        errors.append(action+': '+str(exc))
            self.events.put(('ended', (generation, errors, any(e.startswith('stop:') for e in errors))))
        threading.Thread(target=work, daemon=True).start()

    def poll(self):
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
                    if failure.get('goal') == self.target.goal_id and last.get('status') in (5, 6):
                        self.target.label = '保护停止'
                    self.draw_target()
            elif kind == 'result':
                generation, action, binding, result, error = value
                if generation != self.generation or self.recovery is not None:
                    continue
                self.busy = False
                self.received = 0.  # Require a new status after each operation.
                self.next_poll = 0.
                self.message.set_text('操作未完成：'+error if error else result.get('message', '完成'))
                if action == 'goal' and error:
                    self.target.label = '结果待核对'
                    # No retry until fresh status or explicit cancel.
                if binding == self.target.context:
                    if action in ('plan', 'goal') and not error:
                        self.target.accept_heading(result)
                    if action == 'plan' and not error:
                        self.target.points = result.get('points', [])
                    if action == 'goal' and not error:
                        self.target.goal_id = result['goal_id']
                        self.target.label = '已接收'
                    if action == 'cancel' and not error:
                        self.target.label = '取消中'
                    self.draw_target()
            elif kind == 'ended':
                generation, errors, stop_failed = value
                if generation != self.generation or self.recovery is not None:
                    continue
                self.busy = self.shutdown = False
                self.owned = stop_failed
                self.state, self.received = {}, 0.
                self.uncertain_goal = stop_failed
                self.target.points = []
                self.target.label = '停止结果待核对' if stop_failed else '已结束'
                self.draw_target()
                self.message.set_text('导航已停止；底盘下使能结果见状态。' if not errors
                                      else '停止反馈：'+'；'.join(errors))
        self.tick_recovery()
        self.waypoint_panel.tick()
        frame = self.frame()
        if self.target.reconcile(frame):
            self.draw_target()
            self.message.set_text('地图或定位会话已变化，已清除旧目标和预览路径。')
        fresh = now-self.received < 3 and not self.state.get('stale')
        ready = fresh and self.state.get('localization_ready') and self.state.get('controller_ready')
        if self.received:
            drive = self.drive.message if self.drive else self.drive_message
            nav = ('状态过期 / 导航未启动' if not fresh
                   else navigation_status(self.state, self.target.goal_id,
                                          connecting=bool(self.drive and not self.drive.ready
                                                          and not self.drive.stop_event.is_set())))
            self.status.set_text(drive+' · 导航：'+str(nav)+' · 目标：'+(self.target.label or '未设置'))
        elif self.drive:
            self.status.set_text(self.drive.message+' · 等待导航状态')
        free = not self.busy and not self.shutdown and self.recovery is None and not self.map_panel.busy
        self.buttons['recover'].set_sensitive(self.recovery is None and not self.map_panel.busy)
        self.buttons['drive'].set_sensitive(free and self.drive is None and not self.active())
        self.buttons['start'].set_sensitive(bool(free and self.drive and self.drive.ready and fresh
                                                       and self.state.get('chassis_motion', {}).get('ready')
                                                       and context(frame) and self.fresh_map()
                                                       and (frame.get('localization') or {}).get('ready') and not self.active()))
        self.buttons['end'].set_sensitive(bool(self.recovery is None and not self.shutdown and (self.drive or self.owned or self.active() or self.state.get('running'))))
        self.buttons['cancel'].set_sensitive(free and (self.active() or self.state.get('running', False)))
        for name in ('plan', 'goal'):
            self.buttons[name].set_sensitive(bool(free and ready and self.fresh_map() and self.target.pose
                                                 and not self.active()
                                                 and (name == 'plan' or self.drive and self.drive.ready)))
        for index, entry in enumerate(self.entries):
            entry.set_sensitive(free and not self.active())
        if now >= self.next_poll and not self.polling and not self.busy and not self.shutdown:
            self.next_poll = now+1.
            self.poll()
        return True

    def close(self):
        if self.recovery is not None:
            self.message.set_text('正在恢复底盘接入，请等待结果后关闭；尚未释放恢复占用。')
            return False
        if self.busy or self.shutdown or self.drive or self.owned or self.active():
            waiting_drive = self.drive and self.drive.stop_event.is_set() and not self.busy and not self.owned and not self.active()
            if not self.shutdown and not waiting_drive:
                self.end()
            return False
        self.closed = True
        return True
