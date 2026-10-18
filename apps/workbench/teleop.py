"""Teleoperation panel; launches the same high-rate client as `run.sh vr`."""
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib

ROOT = Path(__file__).resolve().parents[2]


class TeleopPanel(Gtk.Box):
    def __init__(self, credentials, acquire, release, demo=False):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        self.credentials, self.acquire, self.release = credentials, acquire, release
        self.demo = demo
        self.process = self.service = None
        self.events = queue.Queue()
        self.closed = False
        self.pack_start(Gtk.Label(label='PICO 双臂遥操', xalign=0), False, False, 0)
        text = ('右手柄 → 左臂，左手柄 → 右臂；握 Grip 跟随，松开保持。\n'
                'A/X 冻结旋转，B/Y 冻结位移；450 Hz 控制由独立进程执行。\n'
                '先启动接收服务，在头显选择电脑 IP 并打开 Controller / Send。')
        label = Gtk.Label(label=text, xalign=0)
        label.set_line_wrap(True)
        self.pack_start(label, False, False, 0)
        self.home = Gtk.CheckButton(label='启动后先回双臂 Home（会运动）；不勾选则保持当前姿态')
        self.home.set_active(True)
        self.pack_start(self.home, False, False, 0)
        row = Gtk.Box(spacing=8)
        self.stop_button = None
        for title, callback in [('启动 PICO 接收服务', self.start_service), ('启动遥操 · 使能上半身', self.start),
                                ('停止遥操并下使能', lambda *_: self.command('stop'))]:
            button = Gtk.Button(label=title)
            button.connect('clicked', callback)
            if title == '停止遥操并下使能':
                self.stop_button = button
                button.set_sensitive(False)
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
        scroll = Gtk.ScrolledWindow()
        self.log = Gtk.TextView(editable=False, cursor_visible=False, wrap_mode=Gtk.WrapMode.WORD_CHAR)
        scroll.add(self.log)
        self.pack_start(scroll, True, True, 0)
        GLib.timeout_add(100, self.tick)

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
        if self.demo:
            self.append('离线演示：未启动接收服务')
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
        if self.demo:
            self.append('离线演示：未连接、未使能')
            return
        if self.process and self.process.poll() is None:
            return
        credentials = self.credentials('upper')
        if credentials is None or not self.acquire('upper', '遥操'):
            return
        host, user, password = credentials
        try:
            argv = ['bash', str(ROOT/'run.sh'), 'vr', '--gui-stdio', '--host', host, '--user', user]
            if not self.home.get_active():
                argv.append('--no-auto-home')
            self.process = subprocess.Popen(argv, cwd=ROOT, stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
            self.process.stdin.write(json.dumps({'password': password}) + '\n')
            self.process.stdin.flush()
            self.status.set_text('等待双手柄新鲜数据及 Grip 释放…')
            self.home.set_sensitive(False)
            threading.Thread(target=self.reader, args=(self.process, 'vr'), daemon=True).start()
        except (OSError, ValueError) as exc:
            if self.process and self.process.poll() is None:
                self.process.terminate()
            self.release('upper', '遥操')
            self.append(str(exc))

    def command(self, command):
        if self.process is None or self.process.poll() is not None:
            return
        try:
            self.process.stdin.write(json.dumps({'command': command}) + '\n')
            self.process.stdin.flush()
            if command == 'stop':
                self.status.set_text('正在等待遥操退出并确认下使能…')
        except (OSError, ValueError):
            self.status.set_text('遥操命令管道已关闭，等待退出结果')

    def tick(self):
        while not self.events.empty():
            name, data = self.events.get_nowait()
            if name == 'vr_exit':
                self.release('upper', '遥操')
                self.home.set_sensitive(True)
                self.status.set_text(f'遥操已退出，返回码 {data}；下使能结果见下方日志')
                if self.process:
                    self.process.stdin.close()
            elif name == 'vr' and data.startswith('WORKBENCH_VR '):
                try:
                    state = json.loads(data[len('WORKBENCH_VR '):])
                    grips = state.get('grips', {})
                    self.status.set_text(f'{state["phase"]} · {state["sides"]} · 灵敏度 {state["sensitivity"]:g}\n'
                        f'Grip 左 {grips.get("left", 0):.2f} / 右 {grips.get("right", 0):.2f} · '
                        f'使能 {state["enabled"]}/20 · 下发 {state["send_hz"]:.0f} / 接收 {state["accepted_hz"]:.0f} Hz')
                except (ValueError, KeyError, TypeError):
                    self.append('遥操状态格式异常')
            else:
                self.append(f'[{name}] {data}')
        running = self.process is not None and self.process.poll() is None
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
