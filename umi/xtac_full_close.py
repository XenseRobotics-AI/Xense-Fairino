"""One supervised closure, then a brief maximum CONTINUOUS grip and disable.

No flash writes. The 6 Nm motor peak is never used as a grip command.
An explicitly accepted historical ControlDeadline is retained in the report;
current faults, motor fault history, and other firmware history remain blockers.
"""
import argparse
import json
import math
import struct
import time
from pathlib import Path

from xtac_probe import pack, decode_status, decode_fault, decode_control_stats
from xtac_upgrade import OperationLink
from xtac_motion_test import preflight, check_sample, next_status, normalized, to_rad

KP, KD = 3.0, 0.7
APPROACH_BUDGET = 0.35
SPEED = 0.15
PRELOAD = 0.25
CLOSED_TOLERANCE = 0.003


def approach_command(actual, reference, cfg, preload, budget=APPROACH_BUDGET):
    if not 0 <= preload <= budget <= 1.1:
        raise ValueError('Invalid preload / continuous torque budget')
    direction = 1.0 if cfg['reverse'] else -1.0
    delta = (budget - preload) / KP
    target = max(actual - delta, min(actual + delta, reference))
    limits = sorted((to_rad(0, cfg), to_rad(1, cfg)))
    target = max(limits[0], min(limits[1], target))
    return target, KP, KD, direction * preload


def submit(link, target, kp, kd, ff):
    if not all(math.isfinite(v) for v in (target, kp, kd, ff)) or abs(ff) > 1.100001:
        raise ValueError('Invalid command')
    seq = link.seq
    link.seq = (seq + 1) & 255
    link.serial.write(pack(seq, 0x43, struct.pack('<5f', target, kp, kd, ff, 0), frame_type=1))


def run(link, report):
    cfg = report['config']
    limit = report['hold_limit_nm']
    direction = 1.0 if cfg['reverse'] else -1.0
    goal = to_rad(0, cfg)
    rows = report['samples'] = []
    report['motion_completed'] = False
    streaming = armed = False
    try:
        link.request(0x21)
        streaming = True
        link.request(0x20, struct.pack('<HB4HB', 0x10, 0, 0, 0, 0, 100, 1))
        initial = next_status(link)
        check_sample(initial, cfg)
        reference = initial['position_rad']
        armed = True
        submit(link, reference, KP, KD, 0)
        link.request(0x30)
        submit(link, reference, KP, KD, 0)
        start = last = time.monotonic()
        deadline = start + abs(reference - goal) / SPEED + 6.0
        phase = 'approach'
        preload = 0.0
        stable_since = hold_since = None
        force = PRELOAD
        while True:
            sample = next_status(link)
            now = time.monotonic()
            gap = now - last
            last = now
            if gap > 0.1:
                raise RuntimeError('Control gap exceeded 100 ms')
            dt = min(gap, 0.025)
            check_sample(sample, cfg, max_torque=1.25, max_speed=0.6 if phase == 'approach' else 0.12)
            if not sample['status_bits'] & 1 and now - start > 0.1:
                raise RuntimeError('Motor unexpectedly disabled')
            actual = sample['position_rad']
            opening = normalized(actual, cfg)
            if direction * (actual - goal) > 0.015:
                raise RuntimeError('Closed-end deflection exceeded 0.015 rad')
            if phase == 'approach':
                error = goal - reference
                reference += max(-SPEED * dt, min(SPEED * dt, error))
                closed = abs(actual-goal) <= CLOSED_TOLERANCE and abs(sample['velocity_rad_s']) < 0.05
                # This device's calibrated zero is a loaded pad position.
                # Only in the final 12% of travel, after the slow reference
                # reaches zero, ramp preload within the user's continuous
                # grip budget. Freeze at zero long enough to verify contact.
                if abs(reference-goal) < 1e-6 and opening < 0.12 and not closed:
                    preload = min(limit, preload + 0.25 * dt)
                budget = min(limit, APPROACH_BUDGET + preload)
                command = approach_command(actual, reference, cfg, preload, budget)
                stable_since = (stable_since or now) if closed else None
                if stable_since is not None and now - stable_since >= 0.3:
                    report['closed_observed'] = dict(t=now-start, normalized=opening, **sample)
                    print(json.dumps({'closed_observed':report['closed_observed']}), flush=True)
                    phase = 'force_ramp'
                    force = preload
                    deadline = now + 5.0
                elif now > deadline:
                    raise RuntimeError(f'Closure did not reach calibrated zero; opening={opening:.5f}')
            else:
                if abs(actual-goal) > 0.015:
                    raise RuntimeError('Contact position lost during grip')
                force = min(limit, force + 0.5 * dt)
                # Only use pure torque after contact at calibrated zero is
                # observed. End after 0.5 s at the continuous (not peak) limit.
                command = (goal, 0.0, 0.0, direction * force)
                if force >= limit:
                    phase = 'max_continuous_hold'
                    hold_since = hold_since or now
                if now > deadline:
                    raise RuntimeError('Grip ramp exceeded time budget')
            submit(link, *command)
            rows.append(dict(t=round(now-start, 5), phase=phase, normalized=opening,
                             command_pos=command[0], command_kp=command[1], command_kd=command[2],
                             command_ff_nm=command[3], **sample))
            if hold_since is not None and now - hold_since >= 0.5:
                report['motion_completed'] = True
                report['max_hold_command_nm'] = limit
                report['hold_duration_s'] = now - hold_since
                report['closed_at_end'] = dict(normalized=opening, **sample)
                return
    finally:
        cleanup = report['cleanup'] = {}
        if armed:
            try:
                submit(link, 0, 0, 0, 0)
            except Exception as error:
                cleanup['zero_torque_error'] = str(error)
            try:
                link.request(0x31, timeout=0.5, retries=1)
                cleanup['disable_ack'] = True
            except Exception as error:
                cleanup['disable_error'] = str(error)
                print('DISABLE NOT CONFIRMED: disconnect 24V immediately.', flush=True)
        if streaming:
            try:
                link.request(0x21)
                cleanup['stop_stream_ack'] = True
            except Exception as error:
                cleanup['stop_stream_error'] = str(error)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--expected-sn', required=True)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--allow-recorded-deadline', action='store_true')
    ap.add_argument('--output', type=Path, default=Path('output/full-close.json'))
    args = ap.parse_args()
    report = dict(executed=args.execute, passed=False)
    link = OperationLink(args.port, {0x20, 0x21, 0x30, 0x31})
    try:
        report.update(preflight(link, args.expected_sn, args.allow_recorded_deadline))
        spec = link.read(0x56)
        continuous = struct.unpack_from('<f', spec, 32)[0]
        if abs(continuous-1.1) > 1e-5:
            raise RuntimeError('Unexpected continuous torque rating')
        report['hold_limit_nm'] = min(1.1, continuous, report['config']['envelope']['continuous_nm'])
        print(json.dumps({k:v for k,v in report.items() if k != 'config'}, indent=2), flush=True)
        if args.execute:
            run(link, report)
    except BaseException as error:
        report['error'] = str(error)
        raise
    finally:
        try:
            report['final_status'] = decode_status(link.read(0x50))
            report['final_fault'] = decode_fault(link.read(0x52))
            report['control_after'] = decode_control_stats(link.read(0x51))
            report['passed'] = bool(report.get('motion_completed') and report.get('cleanup', {}).get('disable_ack')
                and report['final_status']['status_bits'] == 0
                and not any(report['final_fault'][k] for k in ('motor','motor_latched','firmware','firmware_latched')))
        except Exception as error:
            report['final_verification_error'] = str(error)
        finally:
            link.close()
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
            print(json.dumps({k:v for k,v in report.items() if k not in ('samples','config')}, indent=2), flush=True)


if __name__ == '__main__':
    main()
