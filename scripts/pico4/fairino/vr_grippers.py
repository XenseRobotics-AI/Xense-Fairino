"""Opt-in PICO Trigger -> shared TacCap Session. Arm loop never performs I/O."""
import sys

from paths import ROOT
sys.path.insert(0, str(ROOT))
from apps.gripper_control.client import Session
from apps.gripper_control.gripper_core import TriggerMapping, DEFAULT_TORQUE, DEFAULT_SPEED


class GripperTeleop:
    def __init__(self, args, password, session_factory=Session):
        self.session = session_factory(args.host, args.user, password,
            left_sn=getattr(args, 'left_gripper_sn', ''), right_sn=getattr(args, 'right_gripper_sn', ''))
        self.mapping = TriggerMapping()
        self.enable_requested = False
        self.note = '等待夹爪连接'

    def start(self):
        self.session.start()
        return self

    def update(self, frame, hands, active):
        state = self.session.snapshot()
        if state.get('error'):
            raise RuntimeError('夹爪控制：'+state['error'])
        devices = state.get('devices', {})
        connected = [side for side, unit in devices.items() if unit.get('connected')]
        if state.get('message'):
            raise RuntimeError('夹爪控制：'+state['message'])
        if not connected:
            self.note = '未连接夹爪（双臂可独立遥操）' if state.get('ready') else '等待夹爪扫描'
            if state.get('device_errors'):
                self.note += '；'+str(state['device_errors'])
            return
        if not active or frame is None:
            self.mapping.reset()
            self.session.set_targets(None)
            self.note = '夹爪保持；松开 Trigger 后重新按压继续'
            return
        if not self.enable_requested:
            if not all(frame[hands[s]].get('trigger_available', False)
                       and frame[hands[s]]['trigger'] <= .02 for s in connected):
                self.note = '请松开夹爪对应 Trigger，等待有效扳机数据'
                return
            self.session.command('enable', 'both', torque=DEFAULT_TORQUE, speed=DEFAULT_SPEED)
            self.enable_requested = True
            self.note = '夹爪使能并保持当前位置'
            return
        if state.get('busy') or not all(devices[s].get('controlled') for s in connected):
            return
        try:
            targets = self.mapping.update(frame, hands, connected)
        except ValueError as exc:
            self.mapping.reset()
            self.session.set_targets(None)
            self.note = str(exc)
            return
        self.session.set_targets(targets)
        self.note = 'Trigger 连续开度：松开张开，压到底闭合' if targets else '保持当前开度，按 Trigger 开始'

    def snapshot(self):
        state = self.session.snapshot()
        return dict(message=self.note, devices=state.get('devices', {}))

    def close(self):
        self.session.close()

    def finish(self):
        self.close()
        self.session.thread.join(15.)
        return self.session.finished() and self.session.disabled_confirmed and not self.session.failure
