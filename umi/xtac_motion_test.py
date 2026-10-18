"""Bounded, supervised, empty-jaw smoke test for the identified EL05 follower.

Uses the official v0.4.3 wire definitions, not the Linux-only native SDK.
No flash/configuration writes. Requires --execute; otherwise checks only.
Not a replacement for the official ForcePositionController in production.
"""
import argparse
import json
import math
import struct
import time
from pathlib import Path

from xtac_probe import decode_config, decode_fault, decode_status, decode_control_stats, pack
from xtac_upgrade import OperationLink

MAX_COMMAND_TORQUE = 0.35
KP, KD = 3.0, 0.7
RAMP_SPEED = 0.15  # shaft rad/s


def normalized(raw, cfg):
    direction = -1 if cfg['reverse'] else 1
    return (raw * direction - cfg['min_open_rad']) / (cfg['max_open_rad'] - cfg['min_open_rad'])


def to_rad(position, cfg):
    direction = -1 if cfg['reverse'] else 1
    return direction * (cfg['min_open_rad'] + position * (cfg['max_open_rad'] - cfg['min_open_rad']))


def bounded_target(actual, reference, cfg):
    delta = MAX_COMMAND_TORQUE / KP
    target = max(actual - delta, min(actual + delta, reference))
    limits = sorted((to_rad(0, cfg), to_rad(1, cfg)))
    return max(limits[0], min(limits[1], target))


def check_sample(sample, cfg, max_torque=0.65, max_speed=0.6):
    if not all(math.isfinite(v) for v in sample.values()):
        raise RuntimeError('Non-finite motor state')
    if sample['status_bits'] & ~1:
        raise RuntimeError(f'Motor fault/status bits: {sample["status_bits"]:#x}')
    if not 5 <= sample['temperature_c'] < 45:
        raise RuntimeError('Invalid or excessive motor temperature')
    # None disables only that host-side magnitude trip; data validity, motor
    # faults, temperature and calibrated travel are still checked.
    if ((max_torque is not None and abs(sample['torque_nm']) > max_torque)
            or (max_speed is not None and abs(sample['velocity_rad_s']) > max_speed)):
        raise RuntimeError('Smoke-test torque or speed threshold exceeded')
    if not -0.03 <= normalized(sample['position_rad'], cfg) <= 1.03:
        raise RuntimeError('Position outside calibrated travel')


def preflight(link, expected_sn, allow_recorded_deadline=False, max_torque=0.65, max_speed=0.6):
    sn = link.read(4).split(b'\0')[0].decode('ascii')
    version = tuple(link.read(2))
    if sn != expected_sn or not sn.endswith('s') or version[:3] != (1, 2, 16):
        raise RuntimeError('Exact follower identity / reviewed firmware check failed')
    model = link.read(0x59)
    if model[0] != 0 or not model[9]:
        raise RuntimeError('This smoke test is restricted to a recorded EL05 motor')
    cfg = decode_config(link.read(0x67))
    if cfg['magic'] != '0x47525052' or cfg['version'] != 1 or not cfg['flags'] & 1:
        raise RuntimeError('Invalid calibration record')
    if not 0.1 < cfg['max_open_rad'] - cfg['min_open_rad'] < 2:
        raise RuntimeError('Unexpected calibrated travel')
    if int(cfg['envelope']['flags'], 16) & 0xF003 != 0x2003:
        raise RuntimeError('Firmware safety envelope is not enabled with layout 2')
    if not 0 < cfg['envelope']['continuous_nm'] <= cfg['envelope']['peak_nm'] <= 2:
        raise RuntimeError('Firmware torque envelope is not suitable for this test')
    home = link.read(0x57)
    if home[1] != 12 or home[2] != 0 or home[3] & 1:
        raise RuntimeError('Power-on calibration has not completed successfully')
    fault = decode_fault(link.read(0x52))
    allowed_history = (0, 0x303) if allow_recorded_deadline else (0,)
    if (not fault['report_flags'] & 4
            or any(fault[k] for k in ('motor', 'motor_latched', 'firmware'))
            or fault['firmware_latched'] not in allowed_history):
        raise RuntimeError(f'Fault diagnostic is not clear: {fault}')
    sample = decode_status(link.read(0x50))
    check_sample(sample, cfg, max_torque=max_torque, max_speed=max_speed)
    if sample['status_bits'] & 1:
        raise RuntimeError('Motor is already enabled; refusing to take over')
    stats = decode_control_stats(link.read(0x51))
    if stats['running'] or stats['error_count'] or stats['timeout_count'] or stats['last_error']:
        raise RuntimeError(f'Control task is not idle and error-free: {stats}')
    return dict(serial_number=sn, firmware=list(version), config=cfg, status=sample, fault=fault,
                control_before=stats, recorded_deadline_accepted=bool(fault['firmware_latched']))


def submit(link, target, kp=KP, kd=KD):
    seq = link.seq
    link.seq = (seq + 1) & 255
    # Zero feedforward torque/velocity; spring error is bounded by caller.
    link.serial.write(pack(seq, 0x43, struct.pack('<5f', target, kp, kd, 0, 0), frame_type=1))


def next_status(link):
    deadline = time.monotonic() + 0.15
    while True:
        _, _, kind, cmd, payload = link.frame(deadline)
        if kind == 3 and cmd == 0x50:
            return decode_status(payload)


def run(link, report, targets):
    cfg = report['config']
    rows = report['samples'] = []
    report['steps'] = []
    armed = False
    streaming = False
    try:
        link.request(0x21)
        streaming = True
        link.request(0x20, struct.pack('<HB4HB', 0x10, 0, 0, 0, 0, 100, 1))
        first = next_status(link)
        check_sample(first, cfg)
        reference = first['position_rad']
        # Seed a current-position target before enable, matching the SDK's
        # controller.start(); motor.enable() ordering. Never resume an old target.
        armed = True
        submit(link, reference)
        link.request(0x30)
        submit(link, reference)
        start_all = time.monotonic()
        for goal in targets:
            goal_rad = to_rad(goal, cfg)
            begin = time.monotonic()
            last = begin
            arrival_since = None
            start_position = normalized(reference, cfg)
            budget = abs(reference - goal_rad) / RAMP_SPEED + 5.0
            while True:
                sample = next_status(link)
                now = time.monotonic()
                check_sample(sample, cfg)
                if now - last > 0.1:
                    raise RuntimeError('Control gap exceeded 100 ms')
                dt = min(now - last, 0.025)
                last = now
                error = goal_rad - reference
                reference += max(-RAMP_SPEED * dt, min(RAMP_SPEED * dt, error))
                command = bounded_target(sample['position_rad'], reference, cfg)
                submit(link, command)
                position = normalized(sample['position_rad'], cfg)
                rows.append(dict(t=round(now-start_all, 5), goal=goal, normalized=position,
                                 command_rad=command, **sample))
                arrived = abs(sample['position_rad']-goal_rad) < 0.035 and abs(reference-goal_rad) < 0.005
                arrival_since = (arrival_since or now) if arrived else None
                if arrival_since is not None and now-arrival_since >= 0.3:
                    step = dict(target=goal, actual=position, elapsed_s=round(now-begin, 3),
                                travel=abs(position-start_position))
                    report['steps'].append(step)
                    print(json.dumps(step), flush=True)
                    break
                if now-begin > budget:
                    raise RuntimeError(f'Did not reach {goal:.2f} in {budget:.1f}s; actual={position:.3f}')
        report['passed'] = True
    finally:
        report['cleanup'] = {}
        # Always remove spring torque and request disable before stopping data.
        if armed:
            try:
                submit(link, 0, kp=0, kd=0)
            except Exception as error:
                report['cleanup']['zero_torque_error'] = str(error)
            try:
                link.request(0x31, timeout=0.5, retries=1)
                report['cleanup']['disable_ack'] = True
            except Exception as error:
                report['cleanup']['disable_error'] = str(error)
                report['passed'] = False
                print('DISABLE NOT CONFIRMED: disconnect 24V immediately.', flush=True)
        if streaming:
            try:
                link.request(0x21)
                report['cleanup']['stop_stream_ack'] = True
            except Exception as error:
                report['cleanup']['stop_stream_error'] = str(error)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--expected-sn', required=True)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--extended', action='store_true')
    ap.add_argument('--output', type=Path, default=Path('output/motion-test.json'))
    args = ap.parse_args()
    link = OperationLink(args.port, {0x20, 0x21, 0x30, 0x31})
    report = dict(passed=False, executed=args.execute)
    try:
        report.update(preflight(link, args.expected_sn))
        print(json.dumps(report, indent=2), flush=True)
        if args.execute:
            targets = [0.85, 0.95]
            if args.extended:
                targets += [0.15, 0.9, 0.15, 0.9]
            run(link, report, targets)
            report['final_status'] = decode_status(link.read(0x50))
            report['final_fault'] = decode_fault(link.read(0x52))
            report['control_after'] = decode_control_stats(link.read(0x51))
            if report['final_status']['status_bits'] or any(report['final_fault'][k] for k in ('motor','motor_latched','firmware','firmware_latched')):
                report['passed'] = False
                raise RuntimeError('Final disabled/fault-free verification failed')
    except BaseException as error:
        report['error'] = str(error)
        report['passed'] = False
        raise
    finally:
        link.close()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        summary = {k:v for k,v in report.items() if k not in ('samples','config','status','fault')}
        print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
    main()
