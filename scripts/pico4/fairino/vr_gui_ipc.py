"""Small UI adapter; the existing VR client owns timing and robot cleanup."""
import json
import os
import queue
import signal
import sys
import threading

KEYS = {'pause': ' ', 'home': 'h', 'slower': '-', 'faster': '+'}


class GuiInput:
    def __init__(self, stream=None):
        self.stream = stream or sys.stdin
        self.commands = queue.Queue(maxsize=32)

    def authenticate(self):
        packet = json.loads(self.stream.readline())
        if set(packet) != {'password'} or not isinstance(packet['password'], str):
            raise ValueError('GUI 连接初始化无效')
        threading.Thread(target=self.read, daemon=True).start()
        return packet['password']

    def read(self):
        try:
            for line in self.stream:
                packet = json.loads(line)
                if set(packet) != {'command'}:
                    break
                command = packet['command']
                if command == 'stop':
                    break
                if command not in KEYS:
                    break
                self.commands.put_nowait(KEYS[command])
        except (ValueError, TypeError, queue.Full):
            pass
        # Includes a closed parent pipe. Let vr_client's finally disable axes.
        os.kill(os.getpid(), signal.SIGTERM)

    def key(self):
        try:
            return self.commands.get_nowait()
        except queue.Empty:
            return ''


def status(**values):
    print('WORKBENCH_VR ' + json.dumps(values, ensure_ascii=False, allow_nan=False), flush=True)
