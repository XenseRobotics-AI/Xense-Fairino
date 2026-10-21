"""Robot-side SDK owner. No ROS, firmware writes, calibration or implicit reset."""
import argparse
import fcntl
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

from gripper_core import (DEFAULT_SPEED, DEFAULT_TORQUE, LEASE, SIDES, fresh_request,
                          healthy_sample, number, opening, select_devices, diagnostic_snapshot,
                          TARGET_PERIOD)

PREFIX = '@UPPERPROBE '
LINK_TIMEOUT = .6


def emit(**values):
    print(PREFIX + json.dumps(values, ensure_ascii=False, allow_nan=False), flush=True)


def client_exit_reason(packet):
    value = packet.get('exit_reason')
    return value if value in ('client_exit', 'feedback_timeout') else 'unspecified'


class LinkFailure(RuntimeError):
    def __init__(self, reason, message, diagnostics):
        super().__init__(message+'，退出并下使能')
        self.reason, self.diagnostics = reason, diagnostics


def check_link(packet, received, now, first_report, ack_grace):
    """All timestamps here belong to the robot's monotonic clock."""
    ack = packet.get('ack')
    valid_ack = isinstance(ack, (int, float)) and not isinstance(ack, bool) and math.isfinite(ack)
    diagnostics = dict(server_monotonic=now, received_monotonic=received,
                       heartbeat_age_s=now-received, timeout_s=LINK_TIMEOUT,
                       feedback_ack_monotonic=ack if valid_ack else None,
                       feedback_ack_age_s=now-ack if valid_ack else None,
                       ack_grace_remaining_s=max(0., ack_grace-now))
    # An exit packet must not be misreported as a timeout, even if its ACK is old.
    if packet.get('exit'):
        return dict(reason='exit_request', client_exit_reason=client_exit_reason(packet), **diagnostics)
    if now-first_report < LINK_TIMEOUT and not ack:
        return None
    if now-received > LINK_TIMEOUT:
        raise LinkFailure('heartbeat_timeout', '夹爪上位机心跳超时（超过 0.6 秒未更新）', diagnostics)
    if now >= ack_grace:
        if not valid_ack or ack <= 0 or ack > now:
            raise LinkFailure('feedback_ack_invalid', '夹爪反馈确认 ACK 无效或时间戳超前', diagnostics)
        if now-ack > LINK_TIMEOUT:
            raise LinkFailure('feedback_ack_timeout', '夹爪反馈确认 ACK 超时（超过 0.6 秒未更新）', diagnostics)
    return None


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
            mapping = self.gripper.position_map()  # Cached calibrated conversion, no config writes.
            self.open_span_rad = float(mapping.max_open_rad - mapping.min_open_rad)
            if not math.isfinite(self.open_span_rad) or self.open_span_rad <= 0:
                raise RuntimeError('夹爪标定行程无效')
            self.spec = self.gripper.motor.get_spec()
            self.firmware = str(self.gripper.firmware_version)
            self.audit = self.gripper.audit_envelope()
            self.fault = self.gripper.motor.fault_report()
            self.last = self.read_idle()
        except BaseException:
            self.gripper.close()
            raise

    def read_idle(self):
        sample = self.gripper.motor.read_status(timeout_ms=150)
        self.sampled_at = time.monotonic()
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
        self.fault = self.gripper.motor.fault_report()
        # 0x0303 is recorded host-deadline history, as reviewed in umi/xtac_grip.py.
        self.audit = self.gripper.audit_envelope()
        issues = self.diagnostics()['issues']
        if issues:
            raise ValueError('夹爪使能检查未通过（未发送使能）：\n'+'\n'.join(issues))
        limit = min(self.spec.stall_cont_torque_nm, self.audit.effective.cont_torque_nm)
        number(torque, .01, limit, '夹持力矩 Nm')
        number(speed, .01, min(1., self.spec.v_max_rad_s), '夹爪速度 rad/s')
        cfg = self.sdk.ForcePositionConfig.for_spec(self.spec)
        cfg.grasp_torque_nm = torque
        cfg.close_preload_nm = min(cfg.close_preload_nm, torque)
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

    def diagnostics(self):
        return diagnostic_snapshot(self.fault, self.audit, self.firmware)

    def hold(self):
        if self.controller is not None:
            self.controller.hold_position()

    def snapshot(self):
        if self.controller is None:
            return dict(self.last, age_ms=(time.monotonic()-self.sampled_at)*1000,
                        open_span_rad=self.open_span_rad,
                        opening_angle_rad=self.last['position']*self.open_span_rad)
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
                         age_ms=o.age_ms, grasp_torque_nm=s.grasp_torque_nm,
                         open_span_rad=self.open_span_rad, opening_angle_rad=o.position*self.open_span_rad)
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
        # Signal handlers can request exit while the main thread owns this lock.
        self.lock = threading.RLock()
        self.exit = threading.Event()
        self.received = time.monotonic()
        self.last_request = 0
        self.streaming_at = None
        self.exit_detail = None

    def request_exit(self, reason, **details):
        with self.lock:
            if self.exit_detail is None:
                self.exit_detail = dict(reason=reason, server_monotonic=time.monotonic(), **details)
        self.exit.set()

    def read(self):
        try:
            for line in sys.stdin:
                packet = json.loads(line)
                if not isinstance(packet, dict):
                    self.request_exit('input_invalid')
                    break
                with self.lock:
                    self.packet, self.received = packet, time.monotonic()
                if packet.get('exit'):
                    self.request_exit('exit_request', client_exit_reason=client_exit_reason(packet))
                    break
        except (ValueError, TypeError):
            self.request_exit('input_invalid')
        except OSError as exc:
            self.request_exit('input_read_error', error=str(exc))
        finally:
            self.request_exit('stdin_closed')

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
                                diagnostics=device.diagnostics(), **device.snapshot())
        emit(stamp=time.monotonic(), ready=True, devices=states, device_errors=self.errors,
             discovered=self.endpoints, last_request=self.last_request)

    def run(self):
        try:
            self._run()
        except LinkFailure as exc:
            self.exit_detail = dict(reason=exc.reason, **exc.diagnostics)
            raise
        except Exception as exc:
            self.exit_detail = dict(reason='worker_error', server_monotonic=time.monotonic(),
                                    error_type=type(exc).__name__, error=str(exc))
            raise
        finally:
            emit(exit_reason=self.exit_detail or dict(
                reason='stop_event', server_monotonic=time.monotonic()))

    def _run(self):
        threading.Thread(target=self.read, daemon=True).start()
        self.connect()
        self.report()
        last_report, idle_read = time.monotonic(), time.monotonic()
        first_report = last_report
        ack_grace = first_report+LINK_TIMEOUT
        while not self.exit.wait(TARGET_PERIOD):
            now = time.monotonic()
            with self.lock:
                packet, received = self.packet.copy(), self.received
            exit_detail = check_link(packet, received, now, first_report, ack_grace)
            if exit_detail:
                self.exit_detail = exit_detail
                break
            if now-first_report < LINK_TIMEOUT and not packet.get('ack'):
                continue  # First status/ack round trip after a potentially slow scan.
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
                ack_grace = last_report+LINK_TIMEOUT
            if self.streaming_at is not None and now-self.streaming_at > LEASE:
                for device in self.devices.values():
                    device.hold()
                self.streaming_at = None
                emit(stream_paused='夹爪目标断流，保持当前位置')
            for device in self.devices.values():
                device.snapshot()
            if now-idle_read >= .1:
                for device in self.devices.values():
                    if device.controller is None:
                        device.last = device.read_idle()
                idle_read = now
            if now-last_report >= .05:
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
    signal.signal(signal.SIGTERM, lambda *_: worker.request_exit('signal', signal='SIGTERM'))
    signal.signal(signal.SIGINT, lambda *_: worker.request_exit('signal', signal='SIGINT'))
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
