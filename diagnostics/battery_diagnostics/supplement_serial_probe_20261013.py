#!/usr/bin/env python3
"""Bounded IMU/BMS test; no motor, GPIO, BMS configuration or power commands."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import select
import signal
import struct
import termios
import time
import tty


def crc(data):
    value = 0xffff
    for b in data:
        value ^= b
        for _ in range(8):
            value = (value >> 1) ^ 0xa001 if value & 1 else value >> 1
    return value.to_bytes(2, 'little')


def inspect(data):
    imu = {}; bms = []; acks = []
    i = 0
    while i + 4 < len(data):
        if data[i] == 0x68 and data[i + 1] in (5, 13, 31):
            size = data[i + 1] + 1
            frame = data[i:i + size]
            if len(frame) == size and sum(frame[1:-1]) % 256 == frame[-1]:
                key = frame[:4].hex()
                imu[key] = imu.get(key, 0) + 1
                if size == 6: acks.append(frame.hex())
                i += size
                continue
        if data[i] in (1, 2) and data[i + 1] in (3, 0x83):
            size = 5 if data[i + 1] == 0x83 else 95 if data[i + 2] == 90 else 0
            if size:
                frame = data[i:i + size]
                if len(frame) == size and crc(frame[:-2]) == frame[-2:]:
                    item = {'address': frame[0], 'function': frame[1], 'hex': frame.hex()}
                    if size == 95:
                        v, a, soc, cap = struct.unpack_from('>IiHI', frame, 3)
                        item.update(voltage_V=v/1000, current_A=a/1000, soc_percent=soc, capacity_Ah=cap/1000)
                        item['plausible'] = 30000 <= v <= 60000 and 0 <= soc <= 100 and 1000 <= cap <= 300000
                    bms.append(item)
                    i += size
                    continue
        i += 1
    return {'bytes': len(data), 'imu_frames': imu, 'imu_acknowledgements': acks, 'modbus_frames': bms, 'raw_hex': data.hex()}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--query-mode', action='store_true')
    p.add_argument('--alternate-baud', action='store_true')
    args = p.parse_args()
    if args.alternate_baud and not args.query_mode:
        p.error('--alternate-baud requires --query-mode')
    device = '/dev/ttyS7'
    owners = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdecimal() or int(proc.name) == os.getpid(): continue
        try:
            for f in (proc / 'fd').iterdir():
                try:
                    if os.readlink(f) == device: owners.append(int(proc.name)); break
                except OSError: pass
        except OSError: pass
    if owners: raise RuntimeError('Serial device already owned by ' + str(owners))
    def interrupted(*_): raise InterruptedError('test interrupted')
    for sig in (signal.SIGALRM, signal.SIGHUP, signal.SIGTERM, signal.SIGINT): signal.signal(sig, interrupted)
    signal.alarm(40)
    result = {'time':time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'device':device, 'baud':115200, 'format':'8N1', 'query_mode_requested':args.query_mode, 'alternate_baud':args.alternate_baud, 'phases':[], 'tx':[]}
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = termios.tcgetattr(fd)
    last_rx = time.monotonic()
    mode_attempted = False
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        tty.setraw(fd, termios.TCSANOW)
        st = termios.tcgetattr(fd)
        st[2] &= ~(termios.CSIZE | termios.PARENB | termios.PARODD | termios.CSTOPB | termios.CRTSCTS)
        st[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
        st[4] = st[5] = termios.B115200
        st[6][termios.VMIN] = st[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, st)
        def receive(seconds, quiet=False):
            nonlocal last_rx
            data = bytearray(); end = time.monotonic() + seconds
            while time.monotonic() < end:
                ready, _, _ = select.select([fd], [], [], min(.001 if quiet else .02, max(0,end-time.monotonic())))
                if ready:
                    try:
                        chunk = os.read(fd, 4096)
                        if chunk: data.extend(chunk); last_rx = time.monotonic()
                    except BlockingIOError: pass
                if quiet and time.monotonic() - last_rx >= .003: break
                if len(data) > 100000: raise RuntimeError('Excessive serial data')
            return bytes(data)
        def phase(name, data):
            entry = {'name': name, **inspect(data)}
            result['phases'].append(entry)
        def send(raw):
            allowed = {bytes.fromhex('6805000c0011'), bytes.fromhex('6804000408'), bytes.fromhex('6805000c0617')}
            for addr in (1,2):
                body=struct.pack('>BBHH',addr,3,0,45); allowed.add(body+crc(body))
            if raw not in allowed: raise ValueError('Command not allowlisted')
            if os.write(fd,raw) != len(raw): raise RuntimeError('Incomplete write')
            termios.tcdrain(fd)
            result['tx'].append({'hex':raw.hex(),'monotonic':time.monotonic()})
        phase('passive_before', receive(2.0))
        if args.query_mode:
            # RION IMU406 protocol: mode 00 = query, 06 = 100 Hz automatic.
            # Confirm the observed starting stream is 100 Hz before touching its mode.
            baseline = result['phases'][-1]['imu_frames'].get('681f0084', 0)
            if not 190 <= baseline <= 210:
                raise RuntimeError('No verified 100 Hz baseline; refuse mode change')
            for attempt in range(3):
                phase('before_mode_switch_'+str(attempt),receive(.4,quiet=True))
                mode_attempted = True
                send(bytes.fromhex('6805000c0011'))
                phase('mode_switch_'+str(attempt),receive(.4))
            phase('passive_after_mode_switch',receive(1.0))
        if args.alternate_baud:
            if result['phases'][-1]['bytes'] != 0:
                raise RuntimeError('IMU did not become quiet; refuse alternate baud test')
            st=termios.tcgetattr(fd); st[4]=st[5]=termios.B9600
            termios.tcsetattr(fd,termios.TCSANOW,st)
        for attempt in range(3):
            for addr in (1,2):
                phase('before_bms_'+str(addr)+'_'+str(attempt),receive(.4,quiet=True))
                body=struct.pack('>BBHH',addr,3,0,45)
                send(body+crc(body))
                phase('bms_'+str(addr)+'_'+str(attempt),receive(.75 if args.alternate_baud else .5))
        if args.query_mode:
            st=termios.tcgetattr(fd); st[4]=st[5]=termios.B115200
            termios.tcsetattr(fd,termios.TCSANOW,st)
            send(bytes.fromhex('6804000408'))
            phase('imu_single_query',receive(.5))
        phase('passive_after',receive(2.0))
    except BaseException as exc:
        result['error'] = repr(exc)
        raise
    finally:
        try:
            if mode_attempted:
                st=termios.tcgetattr(fd); st[4]=st[5]=termios.B115200
                termios.tcsetattr(fd,termios.TCSANOW,st)
                send(bytes.fromhex('6805000c0617'))
                phase('restore_auto100_ack',receive(.4))
                phase('restore_auto100_stream',receive(2.0))
                count = result['phases'][-1]['imu_frames'].get('681f0084', 0)
                result['imu_100hz_restored'] = 190 <= count <= 210
        finally:
            try:
                termios.tcsetattr(fd,termios.TCSANOW,original)
                result['termios_restored'] = termios.tcgetattr(fd) == original
            finally:
                os.close(fd); signal.alarm(0)
                print(json.dumps(result),flush=True)


if __name__ == '__main__': main()
