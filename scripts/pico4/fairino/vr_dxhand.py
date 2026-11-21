"""Local DXHand service client. Only the arm control loop renews its heartbeat."""
import json
import os
from pathlib import Path
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[3]

SIDES = ('left', 'right')


def validate_ports(ports):
    if not ports:
        raise ValueError('请指定 --dxhand-left USB 或 --dxhand-right USB（可用 auto 自动识别）')
    if len(ports) == 2 and 'auto' not in ports and os.path.realpath(ports[0]) == os.path.realpath(ports[1]):
        raise ValueError('两只达显手需要分别指定不同的 USB 设备路径')


def service_command(args):
    if args.tcp_tool != 'flange':
        raise ValueError('达显灵巧手使用法兰 TCP')
    argv = [str(ROOT / '.local/dxhand/env/bin/python'), '-B',
            str(ROOT / 'scripts/dxhand/dxhand_service.py')]
    ports = []
    for side in SIDES:
        port = getattr(args, 'dxhand_' + side)
        calibration = getattr(args, 'dxhand_' + side + '_calibration')
        if port:
            ports.append(port)
            argv.extend(['--' + side, port])
            if calibration:
                argv.extend(['--' + side + '-calibration', calibration])
        elif calibration:
            raise ValueError(f'{side} 校准文件需要启用对应灵巧手')
    validate_ports(ports)
    return argv


class DxHandTeleop:
    def __init__(self, args):
        self.argv = service_command(args)
        self.sides = {s for s in SIDES if getattr(args, 'dxhand_' + s)}
        self.proc = None
        self.reader = None
        self.state = {}
        self.read_error = None
        self.received = 0.
        self.follow = False
        self.last_beat = 0.
        self.closed = False

    def start(self):
        self.proc = subprocess.Popen(self.argv, cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     bufsize=0, start_new_session=True)
        os.set_blocking(self.proc.stdin.fileno(), False)
        self.received = time.monotonic()
        self.reader = threading.Thread(target=self._read, name='DXHand status', daemon=True)
        self.reader.start()

    def _read(self):
        try:
            for line in self.proc.stdout:
                state = json.loads(line)
                hands = state['hands']
                if (state['event'] not in ('status', 'stopped') or type(state['ready']) is not bool
                        or not isinstance(hands, dict) or hands.keys() != self.sides
                        or any(not isinstance(h, dict) or not isinstance(h.get('phase'), str)
                               or type(h.get('driver_off_sent')) is not bool
                               or h.get('disable_confirmed') is not False for h in hands.values())):
                    raise ValueError('无效的达显状态')
                self.received = time.monotonic()
                self.state = state
        except (ValueError, TypeError, KeyError, OSError) as exc:
            self.read_error = str(exc)
        finally:
            self.proc.stdout.close()
            if self.state.get('event') != 'stopped' and not self.read_error:
                self.read_error = '达显服务关闭状态管道，未收到退出回执'

    def snapshot(self):
        return self.state.copy()

    @property
    def ready(self):
        if self.read_error:
            raise RuntimeError(self.read_error)
        state = self.state
        if state.get('event') == 'stopped':
            raise RuntimeError(f'达显服务已退出：{state.get("errors") or state.get("reason")}')
        if state.get('stopping'):
            raise RuntimeError(f'达显服务正在退出：{state.get("errors") or state.get("reason")}')
        if time.monotonic() - self.received > (.5 if state else 30.):
            raise RuntimeError('达显服务状态超时')
        return state.get('ready', False)

    def _send(self, packet):
        # One atomic write (< PIPE_BUF). Backpressure is a session failure.
        os.write(self.proc.stdin.fileno(), json.dumps(packet).encode('ascii') + b'\n')

    def update(self, frame, hands, active):
        now = time.monotonic()
        enabled = bool(self.ready and active and frame and 0 <= now-frame['received_monotonic'] <= .25)
        if enabled != self.follow:
            self._send(dict(cmd='follow', enabled=enabled))
            self.follow = enabled
        if self.state and now - self.last_beat >= .04:
            self._send(dict(cmd='heartbeat', sent_ns=time.monotonic_ns()))
            self.last_beat = now

    def close(self):
        if self.proc is None or self.closed:
            return
        self.closed = True
        try:
            self._send(dict(cmd='stop'))
        except (BrokenPipeError, BlockingIOError):
            pass
        finally:
            self.proc.stdin.close()  # EOF also requests hold and driver-off.

    def finish(self):
        if self.proc is None:
            return False
        self.close()
        try:
            code = self.proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            self.proc.terminate()  # SIGTERM follows the service's normal shutdown path.
            code = self.proc.wait(timeout=5)
        if self.reader and self.reader.ident is not None:
            self.reader.join(timeout=1)
        state = self.state
        return bool(code == 0 and state.get('event') == 'stopped' and not state.get('errors')
                    and state.get('hands') and all(h.get('driver_off_sent') is True
                                                  for h in state['hands'].values()))
