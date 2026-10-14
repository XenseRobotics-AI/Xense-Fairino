#!/usr/bin/env python3
"""Finite BMS discovery. Function 03 only; no GPIO, relay or power writes."""
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


def emit(**record):
    print(json.dumps(record, ensure_ascii=False), flush=True)


def crc(data):
    result = 0xffff
    for byte in data:
        result ^= byte
        for _ in range(8):
            result = (result >> 1) ^ 0xa001 if result & 1 else result >> 1
    return result.to_bytes(2, 'little')


def request(address):
    # Address 03 is documented as a vendor broadcast; 00 is standard broadcast.
    if not 1 <= address <= 247 or address == 3:
        raise ValueError('Not an allowed unicast address')
    body = struct.pack('>BBHH', address, 3, 0, 45)
    return body + crc(body)


def parse_frames(data):
    frames = []
    for i in range(max(0, len(data) - 4)):
        if not 1 <= data[i] <= 247 or data[i + 1] not in (3, 0x83):
            continue
        size = 5 if data[i + 1] == 0x83 else 5 + data[i + 2]
        raw = bytes(data[i:i + size])
        if len(raw) != size or crc(raw[:-2]) != raw[-2:]:
            continue
        frame = {'address': raw[0], 'function': raw[1], 'hex': raw.hex()}
        if raw[1] == 0x83:
            frame['exception_code'] = raw[2]
        elif raw[2] >= 14:
            v, current, soc, capacity = struct.unpack_from('>IiHI', raw, 3)
            frame['prefix'] = {'voltage_V': v / 1000, 'current_A': current / 1000,
                               'soc_percent': soc, 'capacity_Ah': capacity / 1000}
            frame['plausible_48V_pack'] = 30000 <= v <= 60000 and 0 <= soc <= 100 and 1000 <= capacity <= 300000
        frames.append(frame)
    return frames


def port_owners(device):
    result = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdecimal() or int(proc.name) == os.getpid():
            continue
        try:
            for fd in (proc / 'fd').iterdir():
                try:
                    if os.readlink(fd) == device:
                        result.append(int(proc.name))
                        break
                except OSError:
                    pass
        except OSError:
            pass
    return result


def scan_port(device, configurations):
    owners = port_owners(device)
    if owners:
        emit(event='skip_busy', device=device, owners=owners)
        return False
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = termios.tcgetattr(fd)
    emit(event='port_start', device=device, original_flags=original[:6])
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        tty.setraw(fd, termios.TCSANOW)
        for baud, parity, addresses in configurations:
            settings = termios.tcgetattr(fd)
            settings[2] &= ~(termios.CSIZE | termios.PARENB | termios.PARODD | termios.CSTOPB | termios.CRTSCTS)
            settings[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
            if parity in ('E', 'O'):
                settings[2] |= termios.PARENB
            if parity == 'O':
                settings[2] |= termios.PARODD
            settings[4] = settings[5] = getattr(termios, 'B' + str(baud))
            settings[6][termios.VMIN] = settings[6][termios.VTIME] = 0
            try:
                termios.tcsetattr(fd, termios.TCSANOW, settings)
            except termios.error as exc:
                emit(event='config_error', device=device, baud=baud, parity=parity, error=str(exc))
                continue
            actual = termios.tcgetattr(fd)
            active_parity = 'N' if not actual[2] & termios.PARENB else ('O' if actual[2] & termios.PARODD else 'E')
            if active_parity != parity:
                emit(event='config_mismatch', device=device, baud=baud, requested_parity=parity, actual_parity=active_parity)
                continue
            received = bytearray()

            def collect(duration):
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    ready, _, _ = select.select([fd], [], [], min(.03, max(0, end - time.monotonic())))
                    if ready:
                        try:
                            chunk = os.read(fd, 4096)
                            received.extend(chunk)
                            if not chunk:
                                time.sleep(.005)
                        except BlockingIOError:
                            pass
                    if len(received) > 16384:
                        raise RuntimeError('Unexpected continuous serial stream; stop active queries')

            collect(.15)
            passive = len(received)
            if passive > 16:
                emit(event='passive_stream', device=device, baud=baud, parity=parity, rx_hex=received.hex())
                return True
            attempted = []
            # Includes the complete 95-byte reply at the selected baud, plus response delay.
            response_window = max(.20, 95 * 11 / baud + .12)
            for address in addresses:
                query = request(address)
                if os.write(fd, query) != len(query):
                    raise RuntimeError('Incomplete UART write')
                attempted.append(address)
                collect(response_window)
                frames = parse_frames(received)
                if frames:
                    emit(event='response_found', device=device, baud=baud, parity=parity,
                         queried_address=address, attempted_addresses=attempted,
                         frames=frames, rx_hex=received.hex())
                    return True
                if len(attempted) % 32 == 0:
                    emit(event='progress', device=device, baud=baud, parity=parity,
                         queries=len(attempted), last_address=address, rx_bytes=len(received))
            emit(event='config_complete', device=device, baud=baud, parity=parity,
                 attempted_addresses=attempted, passive_bytes=passive,
                 rx_bytes=len(received), rx_hex=received.hex())
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSANOW, original)
            restored = termios.tcgetattr(fd) == original
            emit(event='port_restored', device=device, success=restored)
            if not restored:
                raise RuntimeError('Serial settings restoration verification failed')
        finally:
            os.close(fd)
    return False


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--mode', choices=['common', 'addresses'], required=True)
    p.add_argument('--devices', nargs='+', default=['/dev/ttyS7'])
    args = p.parse_args()
    allowed = {'/dev/ttyS0', '/dev/ttyS3', '/dev/ttyS6', '/dev/ttyS7'}
    if not set(args.devices) <= allowed:
        raise ValueError('Refuse unreviewed UART; Bluetooth and console must be excluded')

    def interrupted(*_):
        raise InterruptedError('Time limit or signal received')

    for sig in (signal.SIGALRM, signal.SIGHUP, signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, interrupted)
    signal.alarm(300)
    emit(event='start', pid=os.getpid(), mode=args.mode, devices=args.devices,
         time=time.strftime('%Y-%m-%dT%H:%M:%S%z'), maximum_seconds=300,
         serial_counters=Path('/proc/tty/driver/serial').read_text())
    if args.mode == 'common':
        configurations = [(b, 'N', [2, 1]) for b in [115200, 9600, 19200, 38400, 57600, 4800, 2400, 1200]]
        configurations += [(b, parity, [2, 1]) for b in [115200, 9600, 19200] for parity in ['E', 'O']]
    else:
        addresses = [a for a in range(1, 248) if a != 3]
        configurations = [(115200, 'N', addresses), (9600, 'N', addresses)]
    found = False
    try:
        for device in args.devices:
            if scan_port(device, configurations):
                found = True
                break
        emit(event='complete', found=found, serial_counters=Path('/proc/tty/driver/serial').read_text())
    except Exception as exc:
        emit(event='error', error=repr(exc))
        raise
    finally:
        signal.alarm(0)


if __name__ == '__main__':
    main()
