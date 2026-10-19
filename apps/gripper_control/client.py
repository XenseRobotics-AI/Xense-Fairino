"""Asynchronous SSH adapter, shared by the standalone UI and PICO client."""
import base64
import json
from pathlib import Path
import queue
import shlex
import sys
import threading
import time
from types import SimpleNamespace
import zlib

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def remote_command(left_sn='', right_sn='', sdk_python='/home/fairino/umi/.venv/bin/python'):
    bundle = {name: (HERE/(name+'.py')).read_text() for name in ('gripper_core', 'worker')}
    encoded = base64.b64encode(zlib.compress(json.dumps(bundle).encode())).decode()
    argv = ['worker.py', '--left-sn', left_sn, '--right-sn', right_sn]
    loader = ("import base64,zlib,json,sys,types;"
              f"bundle=json.loads(zlib.decompress(base64.b64decode({encoded!r})));"
              "m=types.ModuleType('gripper_core');sys.modules['gripper_core']=m;"
              "exec(compile(bundle['gripper_core'],'gripper_core.py','exec'),m.__dict__);"
              f"sys.argv={argv!r};"
              "exec(compile(bundle['worker'],'worker.py','exec'),{'__name__':'__main__'})")
    setup = 'echo UPPERPROBE_CONNECTED\nexec '+shlex.quote(sdk_python)+' -u -c '+shlex.quote(loader)
    return 'bash -c '+shlex.quote(setup)


class Session:
    """All network I/O lives here, outside GTK and the high-rate arm loop."""
    def __init__(self, host, user, password, left_sn='', right_sn='', connection_factory=None):
        self.args = SimpleNamespace(host=host, user=user)
        self.password = password
        self.left_sn, self.right_sn = left_sn, right_sn
        self.connection_factory = connection_factory
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.commands = queue.Queue(maxsize=16)
        self.latest = {}
        self.failure = self.message = ''
        self.busy = False
        self.targets, self.target_time = None, 0.
        self.disabled_confirmed = False

    def start(self):
        self.thread.start()
        return self

    def snapshot(self):
        with self.lock:
            return {**self.latest, 'error': self.failure, 'message': self.message, 'busy': self.busy}

    def command(self, action, side='both', **fields):
        if self.stop.is_set():
            return
        with self.lock:
            self.targets = None
            self.busy = True
            self.message = ''
        self.commands.put_nowait(dict(action=action, side=side, **fields))

    def set_targets(self, values):
        with self.lock:
            self.targets = dict(values) if values else None
            self.target_time = time.monotonic()

    def run(self):
        connection = None
        try:
            factory = self.connection_factory
            if factory is None:
                sys.path.insert(0, str(ROOT/'apps/fairino_upper_probe'))
                from upper_keyboard import Connection
                factory = Connection
            log_dir = ROOT/'logs/gripper'
            log_dir.mkdir(parents=True, exist_ok=True)
            connection = factory(self.args, worker_command=remote_command(self.left_sn, self.right_sn),
                                 log_directory=log_dir, password=self.password, feedback_timeout=3.)
            self.password = None
            sequence, pending, streaming = 0, None, False
            started = time.monotonic()
            while not self.stop.wait(.04):
                now = time.monotonic()
                while not connection.events.empty():
                    event = connection.events.get_nowait()
                    if event.get('error'):
                        raise RuntimeError(event['error'])
                    if 'disabled_confirmed' in event:
                        self.disabled_confirmed = event['disabled_confirmed']
                    if event.get('rejected') or event.get('stream_paused'):
                        self.message = event.get('rejected') or event['stream_paused']
                if connection.done.is_set():
                    raise RuntimeError('夹爪连接已结束，详情见 logs/gripper')
                state = connection.latest
                if not state:
                    if now-started > 30.:
                        raise RuntimeError('等待夹爪扫描结果超时')
                    connection.send()
                    continue
                timeout = 3. if pending and pending.get('action') in ('enable', 'disable') else .5
                if now-connection.last_status > timeout:
                    raise RuntimeError(f'夹爪反馈超过 {timeout:g} 秒未更新')
                with self.lock:
                    self.latest = {**state, 'received': connection.last_status}
                    values, source_time = self.targets, self.target_time
                if pending and state.get('last_request', 0) >= pending['id']:
                    pending = None
                    with self.lock:
                        self.busy = not self.commands.empty()
                if pending is None:
                    try:
                        command = self.commands.get_nowait()
                    except queue.Empty:
                        command = None
                    if command is not None:
                        sequence += 1
                        pending = {**command, 'id': sequence, 'created': state['stamp']}
                        streaming = False
                outgoing = pending
                if pending is None:
                    if values is not None and now-source_time <= .2:
                        sequence += 1
                        outgoing = dict(action='stream', targets=values, id=sequence, created=state['stamp'])
                        streaming = True
                    elif streaming:
                        sequence += 1
                        outgoing = dict(action='hold', side='both', id=sequence, created=state['stamp'])
                        pending, streaming = outgoing, False
                connection.send(request=outgoing)
        except Exception as exc:
            self.failure = str(exc)
        finally:
            self.password = None
            if connection is not None:
                connection.finish()
                # The final cleanup event does not have a periodic status stamp.
                while not connection.events.empty():
                    event = connection.events.get_nowait()
                    if 'disabled_confirmed' in event:
                        self.disabled_confirmed = event['disabled_confirmed']
                    if event.get('cleanup_errors'):
                        self.failure = str(event['cleanup_errors'])

    def close(self):
        self.set_targets(None)
        self.stop.set()

    def finished(self):
        return not self.thread.is_alive()
