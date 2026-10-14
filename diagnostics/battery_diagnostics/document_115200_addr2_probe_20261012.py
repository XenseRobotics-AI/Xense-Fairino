#!/usr/bin/env python3
"""Bounded function-03 BMS reads; restore termios, never issue power commands."""
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
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0xa001 if value & 1 else value >> 1
    return value.to_bytes(2, 'little')


def read_request(address):
    body = struct.pack('>BBHH', address, 3, 0, 45)
    return body + crc(body)


def frames_from(data):
    frames = []
    for offset in range(max(0, len(data) - 4)):
        if data[offset] not in (1, 2) or data[offset + 1] not in (3, 0x83):
            continue
        length = 5 if data[offset + 1] == 0x83 else data[offset + 2] + 5
        frame = bytes(data[offset:offset + length])
        if len(frame) != length or crc(frame[:-2]) != frame[-2:]:
            continue
        decoded = {'hex': frame.hex(), 'address': frame[0], 'function': frame[1]}
        if frame[1] == 0x83:
            decoded['exception_code'] = frame[2]
        elif frame[2] >= 14:
            payload = frame[3:-2]
            voltage, current, soc, capacity = struct.unpack_from('>IiHI', payload)
            decoded['documented_prefix'] = {
                'voltage_V': voltage / 1000,
                'current_A': current / 1000,
                'soc_percent': soc,
                'system_capacity_Ah': capacity / 1000,
            }
        frames.append(decoded)
    return frames


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', default='/dev/ttyS7',
                        choices=['/dev/ttyS7', '/dev/ttyS0', '/dev/ttyS3', '/dev/ttyS6'])
    args = parser.parse_args()
    owners = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdecimal() or int(proc.name) == os.getpid():
            continue
        try:
            for descriptor in (proc / 'fd').iterdir():
                try:
                    if os.readlink(descriptor) == args.device:
                        owners.append(int(proc.name))
                        break
                except OSError:
                    continue
        except OSError:
            continue
    if owners:
        raise RuntimeError(f'Port already in use: {owners}')

    def interrupted(*_):
        raise InterruptedError('Probe interrupted or deadline exceeded')

    for signum in (signal.SIGTERM, signal.SIGINT, signal.SIGALRM):
        signal.signal(signum, interrupted)
    report = {'time': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'device': args.device,
              'query_type': 'Modbus function 03 only, register 0, count 45',
              'trials': [], 'settings_restored': False}
    fd = os.open(args.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = termios.tcgetattr(fd)
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        signal.alarm(30)
        tty.setraw(fd, termios.TCSANOW)
        for baud, address in [(115200, 2)]:
            settings = termios.tcgetattr(fd)
            settings[2] &= ~(termios.PARENB | termios.PARODD | termios.CSTOPB |
                             termios.CSIZE | termios.CRTSCTS)
            settings[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
            settings[4] = settings[5] = getattr(termios, 'B' + str(baud))
            settings[6][termios.VMIN] = 0
            settings[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, settings)
            received = bytearray()

            def collect(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    ready, _, _ = select.select([fd], [], [], min(.05, max(0, end - time.monotonic())))
                    if ready:
                        try:
                            chunk = os.read(fd, 4096)
                            received.extend(chunk)
                            if not chunk:
                                time.sleep(.005)
                        except BlockingIOError:
                            pass
                    if len(received) > 65536:
                        raise RuntimeError('Unexpected continuous serial stream')

            collect(.2)
            trial = {'baud': baud, 'format': '8N1', 'address': address,
                     'passive_bytes': len(received), 'passive_hex': bytes(received).hex(), 'attempts': []}
            report['trials'].append(trial)
            query = read_request(address)
            assert query[1] == 3 and len(query) == 8
            for _ in range(3):
                start = len(received)
                written = os.write(fd, query)
                if written != len(query):
                    raise RuntimeError('Short serial write')
                collect(1.0)
                trial['attempts'].append({'tx_hex': query.hex(),
                                          'rx_hex': bytes(received[start:]).hex()})
                frames = frames_from(received)
                if any('documented_prefix' in f for f in frames):
                    break
            trial['total_rx_bytes'] = len(received)
            trial['valid_frames'] = frames_from(received)
            if any('documented_prefix' in f for f in trial['valid_frames']):
                break
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        signal.alarm(0)
        try:
            termios.tcsetattr(fd, termios.TCSANOW, original)
            report['settings_restored'] = termios.tcgetattr(fd) == original
        except Exception as exc:
            report['restoration_error'] = repr(exc)
        finally:
            os.close(fd)
    print(json.dumps(report))


if __name__ == '__main__':
    assert read_request(2).hex() == '02030000002d85e4'
    assert read_request(1).hex() == '01030000002d85d7'
    main()
