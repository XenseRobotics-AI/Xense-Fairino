"""Small UI adapter; the existing VR client owns timing and robot cleanup."""
import json
import os
import queue
import signal
import sys
import threading
import time

KEYS = {'pause': ' ', 'home': 'h', 'slower': '-', 'faster': '+',
        'guard_on': 'g', 'guard_off': 'G'}


class GuiInput:
    def __init__(self, stream=None):
        self.stream = stream or sys.stdin
        self.commands = queue.Queue(maxsize=32)
        self.exit_reason = None
        self.exit_monotonic = None

    def authenticate(self):
        packet = json.loads(self.stream.readline())
        if set(packet) != {'password'} or not isinstance(packet['password'], str):
            raise ValueError('GUI 连接初始化无效')
        threading.Thread(target=self.read, daemon=True).start()
        return packet['password']

    def read(self):
        reason = 'parent_pipe_closed'
        try:
            for line in self.stream:
                packet = json.loads(line)
                if set(packet) != {'command'}:
                    reason = 'gui_input_invalid'
                    break
                command = packet['command']
                if command == 'stop':
                    reason = 'gui_stop'
                    break
                if command not in KEYS:
                    reason = 'gui_input_invalid'
                    break
                self.commands.put_nowait(KEYS[command])
        except (ValueError, TypeError):
            reason = 'gui_input_invalid'
        except queue.Full:
            reason = 'gui_command_queue_full'
        except OSError:
            reason = 'parent_pipe_error'
        self.exit_reason, self.exit_monotonic = reason, time.monotonic()
        # Includes a closed parent pipe. Let vr_client's finally disable axes.
        os.kill(os.getpid(), signal.SIGTERM)

    def key(self):
        try:
            return self.commands.get_nowait()
        except queue.Empty:
            return ''


def status(**values):
    print('WORKBENCH_VR ' + json.dumps(values, ensure_ascii=False, allow_nan=False), flush=True)


def gripper_status(state):
    # Small measured-state packet, independent from the one-second text log.
    fields = ('position', 'age_ms', 'connected', 'open_span_rad')
    value = dict(received=state.get('received'), error=state.get('error'),
                 devices={s: {k: unit[k] for k in fields if k in unit}
                          for s, unit in state.get('devices', {}).items()})
    print('WORKBENCH_GRIPPERS ' + json.dumps(value, allow_nan=False), flush=True)
