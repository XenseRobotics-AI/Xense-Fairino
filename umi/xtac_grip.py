# /// script
# requires-python = ">=3.12"
# dependencies = ["pyserial==3.5"]
# ///
"""Close/grip the identified EL05 at 1.1 Nm, with velocity damping.

Usage: uv run xtac_grip.py --execute --hold 2
The torque is the command setting, not a claim of measured output torque.
An object stopping travel is a successful grip; calibrated zero is not required.
No firmware/configuration writes. Known historical 0x303 is logged; current
faults still block motion. Ctrl+C requests zero torque and motor disable.
Host torque/speed magnitude trips are disabled; the firmware envelope remains.
"""
import argparse
from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import struct
import time

from xtac_probe import decode_status, decode_fault, decode_control_stats
from xtac_upgrade import OperationLink
from xtac_motion_test import preflight, check_sample, next_status, normalized, to_rad
from xtac_full_close import submit

TORQUE = 1.1
DAMPING = 4.0  # unloaded steady speed approximately 1.1/4 = 0.275 rad/s
SN = 'TCGU01A28Z0015s'


class GripDetector:
    def __init__(self, hold_seconds):
        self.hold_seconds = hold_seconds
        self.window = deque()
        self.hold_since = None

    def update(self, sample, elapsed):
        self.window.append((elapsed, sample['position_rad']))
        while len(self.window) > 1 and self.window[1][0] <= elapsed - 0.35:
            self.window.popleft()
        positions = [p for _, p in self.window]
        contact = (elapsed >= 0.5 and elapsed-self.window[0][0] >= 0.3
                   and max(positions)-min(positions) <= 0.003
                   and abs(sample['velocity_rad_s']) < 0.05
                   and abs(sample['torque_nm']) >= 0.10)
        self.hold_since = (elapsed if self.hold_since is None else self.hold_since) if contact else None
        done = self.hold_since is not None and elapsed-self.hold_since >= self.hold_seconds
        return contact, done


def grip(link, report, hold):
    cfg = report['config']
    direction = 1.0 if cfg['reverse'] else -1.0
    detector = GripDetector(hold)
    rows = report['samples'] = []
    armed = streaming = False
    try:
        link.request(0x21)
        streaming = True
        link.request(0x20, struct.pack('<HB4HB', 0x10, 0, 0, 0, 0, 100, 1))
        initial = next_status(link)
        check_sample(initial, cfg, max_torque=None, max_speed=None)
        armed = True
        submit(link, initial['position_rad'], 0, DAMPING, 0)
        link.request(0x30)
        start = last = time.monotonic()
        previous_contact = False
        while True:
            sample = next_status(link)
            now = time.monotonic()
            if now-last > 0.1:
                raise RuntimeError('Feedback gap exceeded 100 ms')
            last = now
            elapsed = now-start
            check_sample(sample, cfg, max_torque=None, max_speed=None)
            if elapsed > 0.1 and not sample['status_bits'] & 1:
                raise RuntimeError('Motor unexpectedly disabled')
            if direction*(sample['position_rad']-to_rad(0,cfg)) > 0.015:
                raise RuntimeError('Closed-end deflection limit reached')
            if elapsed > 30+hold:
                raise RuntimeError('No stable grip within time budget')
            # Same 1.1 Nm feedforward throughout travel AND contact. Local
            # velocity damping reduces net torque while the jaw is moving.
            submit(link, sample['position_rad'], 0, DAMPING, direction*TORQUE)
            contact, done = detector.update(sample, elapsed)
            rows.append(dict(t=round(elapsed,5), normalized=normalized(sample['position_rad'],cfg),
                             contact=contact, command_ff_nm=direction*TORQUE, **sample))
            if contact and not previous_contact:
                print(f'Grip detected at {rows[-1]["normalized"]:.1%} opening; holding {hold:g} s.', flush=True)
            previous_contact = contact
            if done:
                report['grip_completed'] = True
                report['grip_result'] = rows[-1]
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
    ap.add_argument('--port', default='COM11')
    ap.add_argument('--hold', type=float, default=2.0, help='Seconds to hold a stable grip (0..300)')
    ap.add_argument('--execute', action='store_true', help='Actually move; omitted means read-only preflight')
    args = ap.parse_args()
    if not math.isfinite(args.hold) or not 0 <= args.hold <= 300:
        ap.error('--hold must be between 0 and 300 seconds')
    report = dict(executed=args.execute, torque_setting_nm=TORQUE, damping=DAMPING,
                  hold_seconds=args.hold, grip_completed=False,
                  host_torque_stop_nm=None, host_speed_stop_rad_s=None)
    output = Path(__file__).resolve().parent/'output'/f'grip-{datetime.now():%Y%m%d-%H%M%S-%f}.json'
    link = None
    code = 0
    try:
        link = OperationLink(args.port, {0x20,0x21,0x30,0x31})
        report.update(preflight(link, SN, allow_recorded_deadline=True,
                                max_torque=None, max_speed=None))
        spec = link.read(0x56)
        kd_max, rated, continuous = struct.unpack_from('<3f', spec, 24)
        envelope = report['config']['envelope']
        if not (kd_max >= DAMPING and rated >= TORQUE and continuous >= TORQUE-1e-5
                and envelope['continuous_nm'] >= TORQUE-1e-5):
            raise RuntimeError('Device specifications/envelope do not support these settings')
        if report['recorded_deadline_accepted']:
            print('Existing historical 0x0303 recorded; current faults are clear.', flush=True)
        if args.execute:
            print(f'Closing: 1.1 Nm setting, velocity damping; hold {args.hold:g} s. Ctrl+C stops.', flush=True)
            grip(link, report, args.hold)
        else:
            print('Preflight OK. No motion. Add --execute to grip.', flush=True)
    except KeyboardInterrupt:
        report['error'] = 'Interrupted by user'
        code = 130
    except Exception as error:
        report['error'] = str(error)
        print(f'ERROR: {error}', flush=True)
        code = 1
    finally:
        if link is not None:
            try:
                report['final_status'] = decode_status(link.read(0x50))
                report['final_fault'] = decode_fault(link.read(0x52))
                report['control_after'] = decode_control_stats(link.read(0x51))
                if report['final_status']['status_bits']:
                    code = 1
                    print('Motor not confirmed disabled: disconnect 24V.', flush=True)
                if report['final_fault']['firmware_latched']:
                    print(f'Firmware history: {report["final_fault"]["firmware_latched"]:#x}; see log.', flush=True)
                if any(report['final_fault'][k] for k in ('motor','motor_latched','firmware')):
                    code = 1
            except Exception as error:
                report['final_verification_error'] = str(error)
                code = 1
            finally:
                link.close()
        if args.execute and not report.get('cleanup',{}).get('disable_ack'):
            code = 1
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        print(f'Log: {output}', flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
