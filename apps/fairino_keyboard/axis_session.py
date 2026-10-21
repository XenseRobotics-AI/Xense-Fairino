"""Latest-sample continuous input for the shared, guarded chassis worker."""
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

from apps.fairino_keyboard.control_core import check_axes, check_speeds

INPUT_TIMEOUT = .2
PERIOD = .02
ZERO = (0., 0., 0.)


class AxisSession:
    """SSH stays off the caller's control loop; stale input requires recentering."""
    def __init__(self, host, user, password, linear=.06, angular=.10, connection_factory=None):
        linear, angular = check_speeds(linear, angular)
        self.args = SimpleNamespace(host=host, user=user, linear=linear, angular=angular,
                                    check=False, prepare_check=False)
        self.password, self.connection_factory = password, connection_factory
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.latest = {}
        self.failure = ''
        self.disabled_confirmed = False
        self.cleanup_confirmed = False
        self.axes, self.source_time, self.active = ZERO, 0., False
        self.armed, self.neutral_floor = False, 0.
        self.sent_axes = ZERO

    def start(self):
        self.thread.start()
        return self

    def set_axes(self, axes=ZERO, source_time=None, active=False):
        values = tuple(check_axes(axes))
        valid_time = type(source_time) in (int, float) and math.isfinite(source_time)
        with self.lock:
            if self.stop.is_set():
                return
            if not active or not valid_time or source_time < self.source_time:
                self.active = self.armed = False
                self.neutral_floor = max(self.neutral_floor, self.source_time)
                self.axes = ZERO
                return
            self.axes, self.source_time, self.active = values, float(source_time), True

    def command_at(self, now, ready):
        """Called by the I/O thread; testable without hardware or a running clock."""
        with self.lock:
            if (not ready or not self.active or self.stop.is_set()
                    or not 0. <= now-self.source_time <= INPUT_TIMEOUT):
                self.armed = False
                self.neutral_floor = max(self.neutral_floor, self.source_time)
                self.sent_axes = ZERO
                return ZERO
            if not self.armed:
                if self.source_time > self.neutral_floor and not any(self.axes):
                    self.armed = True
                self.sent_axes = ZERO
                return ZERO
            self.sent_axes = self.axes
            return self.sent_axes

    def snapshot(self):
        with self.lock:
            return {**self.latest, 'error': self.failure, 'armed': self.armed,
                    'axes': self.sent_axes, 'disabled_confirmed': self.disabled_confirmed,
                    'cleanup_confirmed': self.cleanup_confirmed}

    def run(self):
        connection = None
        try:
            factory = self.connection_factory
            if factory is None:
                sys.path.insert(0, str(Path(__file__).resolve().parent))
                from keyboard_control import Connection
                factory = Connection
            connection = factory(self.args, password=self.password)
            self.password = None
            started = time.monotonic()
            while not self.stop.wait(PERIOD):
                while not connection.events.empty():
                    event = connection.events.get_nowait()
                    if event.get('error'):
                        raise RuntimeError(event['error'])
                if connection.done.is_set():
                    raise RuntimeError('底盘控制连接已结束，请查看 apps/fairino_keyboard/logs')
                now = time.monotonic()
                state = connection.latest
                if state:
                    if now-connection.last_status > .5:
                        raise RuntimeError('底盘状态反馈超过 0.5 秒未更新')
                    with self.lock:
                        self.latest = {**state, 'received': connection.last_status}
                elif now-started > 20.:
                    raise RuntimeError('等待底盘状态超时')
                connection.send(axes=self.command_at(now, bool(state.get('ready'))), focused=True)
        except Exception as exc:
            self.failure = str(exc)
        finally:
            self.password = None
            self.stop.set()
            self.command_at(time.monotonic(), False)
            if connection is not None:
                try:
                    connection.finish()
                    self.disabled_confirmed = bool(connection.latest.get('disabled_confirmed'))
                    self.cleanup_confirmed = bool(connection.latest.get('cleanup_confirmed',
                                                                         self.disabled_confirmed))
                    with self.lock:
                        self.latest = {**connection.latest, 'ready': False,
                                       'received': connection.last_status}
                    while not connection.events.empty():
                        event = connection.events.get_nowait()
                        error = event.get('error')
                        # finish intentionally sets the worker's stop latch.
                        if error and error not in ('用户停止或 SSH 输入已断开', '已停止：用户停止'):
                            self.failure = self.failure or error
                except Exception as exc:
                    self.failure = self.failure or str(exc)

    def close(self):
        self.stop.set()

    def finished(self):
        return not self.thread.is_alive()

    def finish(self, timeout=25.):
        self.close()
        if self.thread.ident is not None:
            self.thread.join(timeout)
        return self.finished() and self.disabled_confirmed and self.cleanup_confirmed
