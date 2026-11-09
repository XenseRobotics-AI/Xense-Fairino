"""Embedded mapping chassis lifecycle; movement stays in the preserved Window."""
import queue
import threading
from types import SimpleNamespace

from fairino_slam.chassis.control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR
from fairino_slam.chassis.transport import Connection
from fairino_slam.paths import LOG_ROOT


class ChassisOwnership:
    """One local owner, also respecting an optional enclosing application's lock."""
    def __init__(self, acquire=None, release=None):
        self.owner = None
        self.reserved_for = None
        self._acquire = acquire or (lambda *_: True)
        self._release = release or (lambda *_: None)

    def reserve(self, owner):
        """Block new sessions while the existing owner is finishing safely."""
        if self.reserved_for is not None:
            return False
        self.reserved_for = owner
        return True

    def unreserve(self, owner):
        if self.reserved_for == owner:
            self.reserved_for = None

    def acquire(self, part, owner):
        if (self.reserved_for is not None and self.reserved_for != owner
                or self.owner is not None or not self._acquire(part, owner)):
            return False
        self.owner = owner
        return True

    def release(self, part, owner):
        if self.owner == owner:
            self._release(part, owner)
            self.owner = None


class MappingSession:
    """Connect asynchronously; never release ownership while a worker is alive."""
    OWNER = '建图键控'

    def __init__(self, panel, ownership, connection_factory=Connection):
        self.panel, self.ownership = panel, ownership
        self.factory = connection_factory
        self.events = queue.Queue()
        self.connection = self.control = self.host = None
        self.starting = self.leased = self.stopping = self.finishing = False
        self.args = None
        self.bound_window = None

    @property
    def busy(self):
        return self.starting or self.connection is not None or self.leased

    def start(self):
        if self.busy:
            return
        if not self.ownership.acquire('base', self.OWNER):
            self.panel.mapping_status.set_text('底盘正由导航占用，请先结束导航并等待下使能。')
            return
        self.leased, self.starting = True, True
        self.stopping = self.finishing = False
        source = self.panel.stream
        self.args = SimpleNamespace(
            host=source.args.host, user=source.args.user, demo=False,
            check=False, prepare_check=False, navigation=False,
            linear=DEFAULT_LINEAR, angular=DEFAULT_ANGULAR,
            log_directory=str(LOG_ROOT/'mapping/chassis'))
        self.panel.mapping_status.set_text('正在连接建图底盘；随后回正并使能。')
        password = source.control_password

        def connect():
            try:
                connection = self.factory(self.args, password=password)
                self.events.put(('connected', connection))
            except Exception as exc:
                self.events.put(('error', str(exc)))
        threading.Thread(target=connect, daemon=True).start()

    def bind_window(self, window):
        if self.bound_window is window:
            return
        self.bound_window = window
        for name in ('key-press-event', 'key-release-event', 'focus-in-event', 'focus-out-event'):
            window.connect(name, lambda _, event, signal=name: self.dispatch(signal, event))

    def dispatch(self, signal, event):
        if self.host is None:
            return False
        return self.host.dispatch(signal, event)

    def _attach(self):
        from .embedding import PanelHost
        from .mapping_controls import Window
        owner = self.panel.get_toplevel()
        self.bind_window(owner)
        self.host = PanelHost(owner)
        self.host.set_sensitive(False)
        self.control = Window(
            self.args, connection=self.connection, compact=True,
            window_factory=lambda **_: self.host,
            on_closed=lambda: self.events.put(('closed', None)))
        self.panel.mapping_area.pack_start(self.host, False, False, 0)
        self.panel.mapping_area.show_all()

    def stop(self):
        if not self.busy:
            return
        self.stopping = True
        self.panel.mapping_status.set_text('正在结束建图底盘，等待远端退出与下使能反馈…')
        if self.connection is None or self.finishing:
            return
        self.finishing = True
        if self.control is not None and not self.control.closing:
            self.host.input_enabled = False
            self.control.close()
        else:
            def finish():
                self.connection.finish()
                self.events.put(('closed', None))
            threading.Thread(target=finish, daemon=True).start()

    def _release(self):
        if self.host is not None:
            self.host.destroy()
        self.host = self.control = self.connection = None
        self.starting = self.stopping = self.finishing = False
        if self.leased:
            self.ownership.release('base', self.OWNER)
            self.leased = False

    def tick(self):
        while not self.events.empty():
            kind, value = self.events.get_nowait()
            if kind == 'error':
                self.panel.mapping_status.set_text('建图底盘连接失败：'+value)
                self._release()
            elif kind == 'connected':
                self.starting, self.connection = False, value
                if self.stopping or self.panel.closed:
                    self.stop()
                else:
                    try:
                        self._attach()
                    except Exception as exc:
                        self.panel.mapping_status.set_text('建图控制面板加载失败：'+str(exc))
                        self.stop()
            elif kind == 'closed':
                # Connection.finish may time out: retain the lease and continue
                # waiting for a real process exit rather than handing over motors.
                if self.connection and self.connection.done.is_set():
                    confirmed = self.connection.latest.get('disabled_confirmed')
                    error = getattr(self.connection, 'last_error', '')
                    text = ('建图底盘已结束并确认下使能' if confirmed
                            else '建图底盘连接已结束，未确认下使能；请检查机器人反馈')
                    self.panel.mapping_status.set_text((error+'；' if error else '')+text)
                    self._release()
                else:
                    self.finishing = False
        if self.connection is not None:
            if self.stopping:
                self.stop()
            elif self.connection.done.is_set():
                self.stop()
            elif self.control is not None:
                ready = self.control.ready and not self.control.stopped
                self.host.set_sensitive(ready)
                self.host.input_enabled = ready
                if self.control.stopped:
                    self.stop()
                else:
                    self.panel.mapping_status.set_text(self.control.last_phase)
        self.panel.mapping_start.set_sensitive(
            not self.busy and self.ownership.owner is None and self.ownership.reserved_for is None)
        self.panel.mapping_stop.set_sensitive(self.busy and not self.stopping)

    def close(self):
        if self.busy:
            self.stop()
            return False
        return True
