"""Passive feedback session, independent from motion ownership."""
import json
from pathlib import Path
import shlex
import threading
import time

import pexpect
from tools.ros_environment import feedback_environment


class Feedback:
    def __init__(self, host, user, password, part):
        self.host, self.user, self.password, self.part = host, user, password, part
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest = {}
        self.thread = threading.Thread(target=self.run, daemon=True)

    def snapshot(self):
        with self.lock:
            return dict(self.latest)

    def run(self):
        child = None
        try:
            command = feedback_environment(self.part) + 'exec python3 -u -c ' + shlex.quote(
                Path(__file__).with_name('telemetry_remote.py').read_text())
            child = pexpect.spawn('ssh', ['-tt', '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=2',
                '-o', 'ServerAliveCountMax=2', '-o', 'StrictHostKeyChecking=accept-new',
                self.user+'@'+self.host, 'bash -c '+shlex.quote(command)],
                encoding='utf-8', echo=False, timeout=1, maxread=65536)
            asked = False
            tail = ''
            started = time.monotonic()
            while not self.stop.is_set():
                match = child.expect([r'(?i)password:', r'\r?\n', pexpect.EOF, pexpect.TIMEOUT])
                if match == 0:
                    if asked:
                        raise RuntimeError('SSH 密码认证失败')
                    child.sendline(self.password)
                    self.password = None
                    asked = True
                elif match == 1:
                    line = child.before.strip()
                    if line.startswith('WORKBENCH_FEEDBACK '):
                        value = json.loads(line[len('WORKBENCH_FEEDBACK '):])
                        value['received'] = time.monotonic()
                        with self.lock:
                            self.latest = value
                    elif line:
                        tail = (tail+'\n'+line)[-500:]
                elif match == 2:
                    raise RuntimeError('反馈连接结束：' + tail)
                if time.monotonic()-started > 15 and not self.snapshot():
                    raise RuntimeError('未收到反馈：' + tail)
        except Exception as exc:
            with self.lock:
                self.latest = {**self.latest, 'error': str(exc)}
        finally:
            self.password = None
            if child is not None:
                if child.isalive():
                    child.sendcontrol('c')
                    try:
                        child.expect(pexpect.EOF, timeout=2)
                    except (pexpect.TIMEOUT, pexpect.EOF):
                        pass
                child.close(force=True)


def fresh_hardware(state, max_age=.5):
    age = state.get('age')
    if 'error' in state or age is None or age + time.monotonic()-state.get('received', 0) > max_age:
        return None
    return state.get('hardware') or None
