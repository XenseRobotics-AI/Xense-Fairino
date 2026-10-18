"""Robot-side SDK owner. No ROS, firmware writes, calibration or implicit reset."""
import argparse
import fcntl
import json
from pathlib import Path
import signal
import sys
import threading
import time

from gripper_core import (DEFAULT_SPEED, DEFAULT_TORQUE, LEASE, SIDES, fresh_request,
                          healthy_sample, number, opening, select_devices)

PREFIX = '@UPPERPROBE '


def emit(**values):
    print(PREFIX + json.dumps(values, ensure_ascii=False, allow_nan=False), flush=True)


class Device:
    def __init__(self, sdk, endpoint):
        self.sdk, self.endpoint = sdk, endpoint
        self.controller = None
        self.enable_deadline = 0.
        self.gripper = sdk.FollowerGripper(mcu_device=endpoint['port'], ack_timeout_ms=200,
                                           max_retries=1, open_cameras=False)
        try:
            if self.gripper.firmware_sn != endpoint['sn']:
                raise RuntimeError('夹爪身份在扫描后发生变化')
            self.gripper.position_map()  # Valid calibrated conversion, no config writes.
            self.spec = self.gripper.motor.get_spec()
            self.audit = self.gripper.audit_envelope()
            self.fault = self.gripper.motor.fault_report()
            self.last = self.read_idle()
        except BaseException:
            self.gripper.close()
            raise

    def read_idle(self):
        sample = self.gripper.motor.read_status(timeout_ms=150)
        return dict(position=self.gripper.rad_to_pos(sample.actual_pos),
                    velocity_rad_s=sample.actual_vel, torque_nm=sample.actual_torque,
                    temperature_c=sample.motor_temp_c, enabled=bool(sample.status & 1),
                    status=int(sample.status), state='未使能' if not sample.status & 1 else '已使能（未接管）',
                    controlled=False, age_ms=0.)

    def enable(self, torque, speed):
        if self.controller is not None:
            raise ValueError('夹爪已由本会话使能；修改参数前先下使能')
        sample = self.read_idle()
        healthy_sample(sample['position'], sample['velocity_rad_s'], sample['torque_nm'],
                       sample['temperature_c'], sample['status'])
        if sample['enabled']:
            raise ValueError('夹爪已有使能，拒绝接管其他会话')
        fault = self.gripper.motor.fault_report()
        if (fault.report_flags & 0x0C != 0x0C or fault.motor_fault_code
                or fault.motor_latched_fault_code or fault.firmware_fault_code
                or fault.firmware_latched_fault_code not in (0, 0x0303)):
            raise ValueError('夹爪当前故障或故障诊断无效；请先独立排查，不自动清错')
        # 0x0303 is recorded host-deadline history, as reviewed in umi/xtac_grip.py.
        self.audit = self.gripper.audit_envelope()
        if not self.audit.ok or self.audit.effective is None:
            raise ValueError('夹爪固件力矩包络未通过检查：'+self.audit.detail)
        limit = min(self.spec.stall_cont_torque_nm, self.audit.effective.cont_torque_nm)
        number(torque, .01, limit, '夹持力矩 Nm')
        number(speed, .01, min(1., self.spec.v_max_rad_s), '夹爪速度 rad/s')
        cfg = self.sdk.ForcePositionConfig.for_spec(self.spec)
        cfg.grasp_torque_nm = torque
        cfg.close_speed_radps = speed
        cfg.status_timeout_ms = 200
        cfg.motor_stream_hz = 100
        self.controller = self.sdk.ForcePositionController(self.gripper, cfg)
        try:
            self.controller.start()  # Seeds a measured-position hold BEFORE enable.
            self.enable_deadline = time.monotonic()+.4
            self.gripper.motor.enable()
        except BaseException:
            self.disable()
            raise

    def target(self, value):
        if self.controller is None:
            raise ValueError('请先显式使能该夹爪')
        self.controller.set_target(opening(value))

    def hold(self):
        if self.controller is not None:
            self.controller.hold_position()

    def snapshot(self):
        if self.controller is None:
            return self.last
        s = self.controller.snapshot()
        o = s.observation
        if s.fault_reason or not o.valid or o.age_ms > 250:
            raise RuntimeError(s.fault_reason or '夹爪状态流无效 / 超过 250 ms')
        healthy_sample(o.position, o.velocity, o.torque, o.motor_temp_c, o.status)
        if not o.status & 1 and time.monotonic() > self.enable_deadline:
            raise RuntimeError('夹爪意外退出使能')
        self.last = dict(position=o.position, velocity_rad_s=o.velocity, torque_nm=o.torque,
                         temperature_c=o.motor_temp_c, enabled=bool(o.status & 1), status=int(o.status),
                         target=s.target_position, state=s.state.name, controlled=True,
                         age_ms=o.age_ms, grasp_torque_nm=s.grasp_torque_nm)
        return self.last

    def disable(self):
        controller, self.controller = self.controller, None
        if controller is None:
            return  # Never disable a motor we did not enable.
        try:
            controller.stop()  # Zero torque + disable, including after a fault.
        finally:
            self.gripper.motor.disable()
        self.last = self.read_idle()
        if self.last['enabled']:
            raise RuntimeError('未确认夹爪下使能')

    def close(self):
        try:
            self.disable()
        finally:
            self.gripper.close()


class Worker:
    def __init__(self, sdk, assignments=None):
        self.sdk, self.assignments = sdk, assignments or {}
        self.devices, self.errors = {}, {}
        self.packet = {}
        self.lock = threading.Lock()
        self.exit = threading.Event()
        self.received = time.monotonic()
        self.last_request = 0
        self.streaming_at = None

    def read(self):
        try:
            for line in sys.stdin:
                packet = json.loads(line)
                if not isinstance(packet, dict):
                    break
                with self.lock:
                    self.packet, self.received = packet, time.monotonic()
                if packet.get('exit'):
                    break
        finally:
            self.exit.set()

    def connect(self):
        self.endpoints = [dict(side=ep.side.name, role=ep.role.name, sn=ep.firmware_sn,
                              port=ep.mcu_device, in_use=ep.in_use)
                          for ep in self.sdk.scan_grippers()]
        selected = select_devices(self.endpoints, self.assignments)
        for side, ep in selected.items():
            try:
                self.devices[side] = Device(self.sdk, ep)
            except Exception as exc:
                self.errors[side] = str(exc)

    def command(self, request, now):
        fresh_request(request, now)
        action = request.get('action')
        if action == 'stream':
            values = request.get('targets')
            if not isinstance(values, dict) or not set(values) <= set(self.devices):
                raise ValueError('夹爪连续目标包含未连接的设备')
            values = {side: opening(value) for side, value in values.items()}
            if any(self.devices[side].controller is None for side in values):
                raise ValueError('连续目标中的夹爪尚未由本会话使能')
            for side, value in values.items():
                self.devices[side].target(value)
            self.streaming_at = now
            return
        sides = list(self.devices) if request.get('side') == 'both' else [request.get('side')]
        if not sides or any(side not in self.devices for side in sides):
            raise ValueError('目标夹爪未连接')
        for side in sides:
            device = self.devices[side]
            if action == 'enable':
                device.enable(number(request.get('torque', DEFAULT_TORQUE), .01, 5., '夹持力矩'),
                              number(request.get('speed', DEFAULT_SPEED), .01, 1., '速度'))
            elif action == 'target':
                device.target(request.get('opening'))
            elif action == 'hold':
                device.hold()
            elif action == 'disable':
                device.disable()
            else:
                raise ValueError('未知夹爪操作')
        self.streaming_at = None

    def report(self):
        states = {}
        for side, device in self.devices.items():
            states[side] = dict(connected=True, sn=device.endpoint['sn'], port=device.endpoint['port'],
                                **device.snapshot())
        emit(stamp=time.monotonic(), ready=True, devices=states, device_errors=self.errors,
             discovered=self.endpoints, last_request=self.last_request)

    def run(self):
        threading.Thread(target=self.read, daemon=True).start()
        self.connect()
        self.report()
        last_report, idle_read = time.monotonic(), time.monotonic()
        first_report = last_report
        ack_grace = first_report+.6
        while not self.exit.wait(.02):
            now = time.monotonic()
            with self.lock:
                packet, received = self.packet.copy(), self.received
            if now-first_report < .6 and not packet.get('ack'):
                continue  # First status/ack round trip after a potentially slow scan.
            if (now-received > .6 or packet.get('exit')
                    or (now >= ack_grace and not 0 <= now-packet.get('ack', 0.) <= .6)):
                raise RuntimeError('夹爪连接或反馈确认超时，退出并下使能')
            request = packet.get('request')
            if request and request.get('id', 0) > self.last_request:
                self.last_request = request['id']
                try:
                    self.command(request, now)
                except (ValueError, KeyError, TypeError) as exc:
                    for device in self.devices.values():
                        device.hold()
                    self.streaming_at = None
                    emit(rejected=str(exc), request_id=self.last_request)
                self.report()
                last_report = time.monotonic()
                ack_grace = last_report+.6
            if self.streaming_at is not None and now-self.streaming_at > LEASE:
                for device in self.devices.values():
                    device.hold()
                self.streaming_at = None
                emit(stream_paused='夹爪目标断流，保持当前位置')
            for device in self.devices.values():
                device.snapshot()
            if now-idle_read >= 1.:
                for device in self.devices.values():
                    if device.controller is None:
                        device.last = device.read_idle()
                idle_read = now
            if now-last_report >= .1:
                self.report()
                last_report = now

    def close(self):
        errors = {}
        for side, device in self.devices.items():
            try:
                device.close()
            except Exception as exc:
                errors[side] = str(exc)
        confirmed = not errors and all(not d.last.get('enabled') for d in self.devices.values())
        emit(disabled_confirmed=confirmed, cleanup_errors=errors)
        return confirmed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--left-sn', default='')
    parser.add_argument('--right-sn', default='')
    args = parser.parse_args()
    lock = (Path.home()/'.fairino_xtac_gripper.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    import xense.taccap as sdk
    sdk.log.set_level('warn')
    worker = Worker(sdk, dict(left=args.left_sn, right=args.right_sn))
    signal.signal(signal.SIGTERM, lambda *_: worker.exit.set())
    signal.signal(signal.SIGINT, lambda *_: worker.exit.set())
    code = 0
    try:
        worker.run()
    except Exception as exc:
        emit(error=str(exc))
        code = 1
    finally:
        if not worker.close():
            code = 1
    return code


if __name__ == '__main__':
    raise SystemExit(main())
