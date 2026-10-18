"""Portable, read-only XTac MCU diagnostics (Windows / Linux, including ARM64).

Protocol: official TacCap-Gripper v0.4.3, cpp/{include,src}/taccap/bus
and cpp/include/taccap/protocol/{commands,payloads}.hpp.
No firmware, calibration, reset, stream-control or motor commands are sent.
"""
import argparse
import json
import platform
import struct
import time
from pathlib import Path

import serial
from serial.tools import list_ports

READS = {0x02, 0x04, 0x06, 0x34, 0x50, 0x51, 0x52, 0x54, 0x56, 0x57, 0x59, 0x67}


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def pack(seq, cmd, payload=b'', frame_type=0):
    plain = struct.pack('<5BH', 0xAA, 2, seq, frame_type, cmd, len(payload)) + payload
    body = plain[1:] + struct.pack('<H', crc16(plain))
    wire = bytearray([0xAA])
    for value in body:
        wire.extend((0x7D, value ^ 0x20) if value in (0xAA, 0x55, 0x7D) else (value,))
    return bytes(wire + b'\x55')


def unpack(wire):
    if not wire.startswith(b'\xAA') or not wire.endswith(b'\x55'):
        raise ValueError('Bad frame delimiters')
    body = bytearray()
    escaped = False
    for value in wire[1:-1]:
        if escaped:
            body.append(value ^ 0x20)
            escaped = False
        elif value == 0x7D:
            escaped = True
        else:
            body.append(value)
    if escaped or len(body) < 8:
        raise ValueError('Incomplete frame')
    addr, seq, kind, cmd, length = struct.unpack_from('<4BH', body)
    if len(body) != length + 8 or crc16(b'\xAA' + body[:-2]) != struct.unpack('<H', body[-2:])[0]:
        raise ValueError('Frame length / CRC mismatch')
    return addr, seq, kind, cmd, bytes(body[6:-2])


class Link:
    def __init__(self, port):
        self.serial = serial.Serial(port=None, baudrate=3000000, timeout=0.02, write_timeout=0.2)
        self.serial.dtr = False
        self.serial.rts = False
        if platform.system() != 'Windows':
            self.serial.exclusive = True
        self.serial.port = port
        self.serial.open()
        self.seq = 1
        self.buffer = bytearray()

    def close(self):
        self.serial.close()

    def frame(self, deadline):
        while time.monotonic() < deadline:
            if 0xAA in self.buffer:
                del self.buffer[:self.buffer.index(0xAA)]
                end = self.buffer.find(0x55, 1)
                if end >= 0:
                    wire = bytes(self.buffer[:end + 1])
                    del self.buffer[:end + 1]
                    try:
                        return unpack(wire)
                    except ValueError:
                        continue
            else:
                self.buffer.clear()
            self.buffer.extend(self.serial.read(max(1, min(self.serial.in_waiting, 4096))))
            if len(self.buffer) > 65536:
                raise RuntimeError('Receive buffer exceeded limit')
        raise TimeoutError('No matching reply within deadline')

    def read(self, cmd):
        if cmd not in READS:
            raise ValueError('Command is not in the read-only allowlist')
        seq = self.seq
        self.seq = (seq + 1) & 255
        self.serial.write(pack(seq, cmd))
        deadline = time.monotonic() + 1.0
        while True:
            _, received_seq, kind, echoed_cmd, payload = self.frame(deadline)
            if received_seq != seq or kind != 2:
                continue
            if echoed_cmd == 0:
                raise RuntimeError('Plain ACK/NACK without query result: ' + payload.hex())
            if echoed_cmd == cmd:
                if len(payload) == 1 and cmd not in (0x06, 0x34):
                    raise RuntimeError('Short reply/error: ' + payload.hex())
                return payload


def decode_status(data):
    values = struct.unpack_from('<4fH', data)
    return dict(zip(('position_rad', 'velocity_rad_s', 'torque_nm', 'temperature_c', 'status_bits'), values))


def decode_config(data):
    magic, version, flags, max_rad, min_rad = struct.unpack_from('<IHHff', data)
    continuous, peak, derate, stop, envelope_flags = struct.unpack_from('<ffBB4xH', data, 16)
    return dict(magic=hex(magic), version=version, flags=flags, max_open_rad=max_rad,
                min_open_rad=min_rad, reverse=bool(flags & 2), envelope=dict(
                    continuous_nm=continuous, peak_nm=peak, derate_c=derate,
                    stop_c=stop, flags=hex(envelope_flags)))


def decode_fault(data):
    return dict(report_flags=data[1], source_mask=data[2], protocol=data[3],
                motor=struct.unpack_from('<I', data, 12)[0],
                motor_latched=struct.unpack_from('<I', data, 16)[0],
                firmware=struct.unpack_from('<I', data, 24)[0],
                firmware_latched=struct.unpack_from('<I', data, 28)[0],
                status_timestamp_ms=struct.unpack_from('<I', data, 56)[0], stop_reason=data[60])


def decode_control_stats(data):
    fields = ('running', 'mode', 'target_hz', 'period_ms', 'sample_ms', 'actual_hz',
              'target_seq', 'applied_seq', 'loop_count', 'error_count',
              'deadline_miss_count', 'timeout_count', 'last_error', 'target_age_ms',
              'reserved', 'target_update_hz')
    return dict(zip(fields, struct.unpack('<BBHHHf6IiHHf', data)))


def inspect(link):
    raw = {}
    for cmd in (0x02, 0x04, 0x06, 0x34, 0x59, 0x56, 0x67, 0x50, 0x52, 0x57, 0x54):
        raw[f'{cmd:02x}'] = link.read(cmd).hex()
    get = lambda key: bytes.fromhex(raw[key])
    model = get('59')
    report = dict(host=dict(system=platform.system(), architecture=platform.machine(), python=platform.python_version()),
                  firmware='.'.join(map(str, get('02')[:3])), serial_number=get('04').split(b'\0')[0].decode('ascii'),
                  side=get('06')[0], can_id=get('34')[0],
                  motor=dict(id=model[0], name=model[1:9].split(b'\0')[0].decode('ascii'),
                             from_flash=bool(model[9]), torque_range_nm=struct.unpack_from('<f', model, 10)[0],
                             autodetected=bool(model[18])),
                  config=decode_config(get('67')), status=decode_status(get('50')),
                  fault=decode_fault(get('52')), homing=dict(state=get('57')[1], failure=get('57')[2], flags=get('57')[3]), raw=raw)
    before = report['fault']['status_timestamp_ms']
    start = time.monotonic()
    time.sleep(0.5)
    after = decode_fault(link.read(0x52))['status_timestamp_ms']
    report['freshness'] = dict(wall_ms=round((time.monotonic()-start)*1000), status_advance_ms=(after-before) & 0xFFFFFFFF)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    if args.list:
        print(json.dumps([dict(port=p.device, description=p.description, hwid=p.hwid)
                          for p in list_ports.comports() if p.vid == 0x1A86 and p.pid == 0x55D2], indent=2))
        return
    if not args.port:
        parser.error('--port is required; --list does not open devices')
    link = Link(args.port)
    try:
        report = inspect(link)
    finally:
        link.close()
    text = json.dumps(report, indent=2, allow_nan=False)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
