"""Update this follower with the official v0.4.3 release image.

Windows transport adapter for the official OtaSession sequence in
vendor/TacCap-Gripper/cpp/src/ota.cpp. Dry-run unless --execute is present.
Checks exact SN/role, immutable release image size/CRC, and device CRC verification.
"""
import argparse
import hashlib
import json
import struct
import time
import zlib
from pathlib import Path

from xtac_probe import Link, pack, decode_status

ROOT = Path(__file__).resolve().parent
FW = ROOT / 'vendor/TacCap-Gripper/firmware'


def checked_image():
    record = json.loads((FW / 'manifest.json').read_text(encoding='utf-8'))['images']['slave']
    if (record['file'], record['version'], record['sn_suffix']) != ('tc-gu-01-slave-1.2.16.bin', '1.2.16', 's'):
        raise ValueError('Release identity differs from the reviewed follower image')
    image = (FW / record['file']).read_bytes()
    crc = zlib.crc32(image)
    if len(image) != 167544 or crc != 0x4A8CD360:
        raise ValueError('Official release image size/CRC mismatch')
    if len(image) != record['size_bytes'] or crc != int(record['crc32'], 16):
        raise ValueError('Manifest mismatch')
    return image, dict(record, sha256=hashlib.sha256(image).hexdigest())


class OperationLink(Link):
    def __init__(self, port, allowed):
        super().__init__(port)
        self.allowed = frozenset(allowed)

    def request(self, cmd, payload=b'', timeout=1.0, retries=0):
        if cmd not in self.allowed:
            raise ValueError('Operation not allowed in this session')
        seq = self.seq
        self.seq = (seq + 1) & 255
        wire = pack(seq, cmd, payload)
        for attempt in range(retries + 1):
            self.serial.write(wire)
            deadline = time.monotonic() + timeout
            try:
                while True:
                    _, received_seq, kind, echoed_cmd, data = self.frame(deadline)
                    if received_seq != seq or kind != 2:
                        continue
                    if echoed_cmd == 0:
                        if data != b'\0':
                            raise RuntimeError(f'NACK cmd=0x{cmd:02x}: {data.hex()}')
                        return b''
                    if echoed_cmd == cmd:
                        if len(data) == 1 and data != b'\0':
                            raise RuntimeError(f'Error cmd=0x{cmd:02x}: {data.hex()}')
                        return data
            except TimeoutError:
                if attempt == retries:
                    raise
                # Reuse the complete frame including sequence. New-sequence OTA
                # retries are not idempotent (official OtaSession uses SameSeq).


def ota_status(link):
    data = link.request(0x75)
    if len(data) != 8:
        raise RuntimeError('Unexpected OTA status length')
    state, error, written, progress = struct.unpack('<BBIH', data)
    return dict(state=state, error=error, bytes_written=written, progress_per_mille=progress)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--expected-sn', required=True)
    ap.add_argument('--execute', action='store_true')
    args = ap.parse_args()
    image, record = checked_image()
    report = dict(image=record, executed=False, events=[])
    # Dry-run opens only the read-only Link. Writing operations get a separate
    # session only after physical identity and file integrity have passed.
    link = Link(args.port)
    try:
        sn = link.read(0x04).split(b'\0')[0].decode('ascii')
        before = tuple(link.read(0x02))
        status = decode_status(link.read(0x50))
    finally:
        link.close()
    if sn != args.expected_sn or not sn.endswith('s'):
        raise ValueError(f'Exact follower identity check failed: {sn!r}')
    if status['status_bits'] or abs(status['velocity_rad_s']) > 0.1:
        raise RuntimeError(f'Motor must be idle, disabled and fault-free: {status}')
    report.update(serial_number=sn, before_version=list(before), status=status)
    print(json.dumps(report, indent=2), flush=True)
    if not args.execute:
        print('DRY RUN: image and identity verified; firmware unchanged.')
        return
    if before[:3] >= (1, 2, 16):
        raise RuntimeError('Already at target/newer version; refusing unnecessary flash')
    report['executed'] = True
    link = OperationLink(args.port, {0x70, 0x71, 0x72, 0x73, 0x74, 0x75})
    applying = False
    started = False
    try:
        initial = ota_status(link)
        if initial['state'] != 0 or initial['error']:
            raise RuntimeError(f'OTA session is not idle: {initial}')
        started = True
        link.request(0x70, struct.pack('<II4B', len(image), zlib.crc32(image), 1, 2, 16, 0), timeout=5.0, retries=1)
        report['events'].append('started')
        for offset in range(0, len(image), 1024):
            block = image[offset:offset + 1024]
            link.request(0x71, struct.pack('<IH', offset, len(block)) + block, timeout=1.0, retries=1)
            if offset % (32 * 1024) == 0 or offset + len(block) == len(image):
                print(f'Written {offset + len(block)}/{len(image)} bytes', flush=True)
        link.request(0x72, timeout=2.0, retries=1)
        verified = ota_status(link)
        if verified['state'] != 3 or verified['error'] or verified['bytes_written'] != len(image):
            raise RuntimeError(f'Device CRC/length verification failed: {verified}')
        report['device_verification'] = verified
        report['events'].append('device_crc_verified')
        print('Device CRC verification passed; applying verified image.', flush=True)
        applying = True
        try:
            link.request(0x73, timeout=2.0)
            report['events'].append('apply_acknowledged')
        except TimeoutError:
            report['events'].append('apply_ack_missing_must_read_version')
    except BaseException as error:
        report['error'] = str(error)
        if started and not applying:
            try:
                link.request(0x74, timeout=0.5)
                report['events'].append('aborted_before_apply')
            except Exception as abort_error:
                report['abort_error'] = str(abort_error)
        raise
    finally:
        link.close()
        (ROOT / 'output').mkdir(exist_ok=True)
        (ROOT / 'output/firmware-update.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print('Apply sent. Read back firmware version, then disconnect 24V for ~2 seconds and reconnect.')


if __name__ == '__main__':
    main()
