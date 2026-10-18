import unittest
import struct
from unittest.mock import patch

from xtac_probe import crc16, pack, unpack, decode_config
from xtac_upgrade import OperationLink, checked_image
from xtac_motion_test import bounded_target, check_sample, to_rad, normalized, MAX_COMMAND_TORQUE, KP, preflight
from xtac_full_close import approach_command, run as run_full_close


class WireTests(unittest.TestCase):
    def test_captured_version_reply(self):
        self.assertEqual(unpack(bytes.fromhex('aa02010202040001020a008c6555')),
                         (2, 1, 2, 2, bytes([1, 2, 10, 0])))

    def test_crc_reference(self):
        self.assertEqual(crc16(b'123456789'), 0x4B37)
        self.assertEqual(pack(1, 2).hex(), 'aa0201000200008ce855')

    def test_escape_and_damage_rejection(self):
        payload = bytes(range(256)) * 4
        wire = pack(0xAA, 0x71, payload)
        self.assertEqual(unpack(wire)[4], payload)
        damaged = bytearray(wire)
        damaged[8] ^= 1
        with self.assertRaises(ValueError):
            unpack(damaged)

    def test_captured_config_offsets(self):
        config = decode_config(bytes.fromhex('52505247010003002a359b3f00000000cdcc8c3f6666e63f0000000000000320'))
        self.assertEqual(config['envelope']['flags'], '0x2003')
        self.assertAlmostEqual(config['envelope']['continuous_nm'], 1.1)

    def test_ota_retry_reuses_sequence(self):
        class FakeSerial:
            def __init__(self): self.writes = []
            def write(self, value): self.writes.append(value)
        obj = object.__new__(OperationLink)
        obj.serial, obj.allowed, obj.seq = FakeSerial(), {0x70}, 7
        replies = [TimeoutError(), (2, 7, 2, 0x70, b'\0')]
        def fake_frame(_deadline):
            result = replies.pop(0)
            if isinstance(result, Exception): raise result
            return result
        obj.frame = fake_frame
        self.assertEqual(obj.request(0x70, b'test', retries=1), b'\0')
        self.assertEqual(obj.serial.writes[0], obj.serial.writes[1])
        self.assertEqual(obj.seq, 8)

    def test_nack_is_not_retried(self):
        obj = object.__new__(OperationLink)
        obj.allowed, obj.seq = {0x70}, 7
        class FakeSerial:
            def write(self, _value): pass
        obj.serial = FakeSerial()
        obj.frame = lambda _deadline: (2, 7, 2, 0, b'\x50')
        with self.assertRaises(RuntimeError):
            obj.request(0x70, retries=1)

    def test_reviewed_release_image(self):
        image, record = checked_image()
        self.assertEqual(len(image), 167544)
        self.assertEqual(record['version'], '1.2.16')

    def test_motion_bounds_both_directions(self):
        for reverse in (False, True):
            cfg = dict(reverse=reverse, min_open_rad=0.03, max_open_rad=1.2)
            for p in (0, 0.1, 0.5, 0.9, 1):
                actual = to_rad(p, cfg)
                self.assertAlmostEqual(normalized(actual, cfg), p)
                for reference in (-100, -1, 0, 1, 100):
                    target = bounded_target(actual, reference, cfg)
                    self.assertLessEqual(abs(target-actual)*KP, MAX_COMMAND_TORQUE+1e-9)
                    self.assertGreaterEqual(normalized(target,cfg), -1e-9)
                    self.assertLessEqual(normalized(target,cfg), 1+1e-9)

    def test_motion_fault_and_echo_rejection(self):
        cfg = dict(reverse=True, min_open_rad=0, max_open_rad=1.2)
        healthy = dict(position_rad=-0.6, velocity_rad_s=0.1, torque_nm=0.2, temperature_c=28., status_bits=1)
        check_sample(healthy, cfg)
        for key,value in [('torque_nm',1),('velocity_rad_s',1),('temperature_c',0),('status_bits',3),('position_rad',float('nan'))]:
            with self.assertRaises(RuntimeError):
                check_sample(dict(healthy, **{key:value}),cfg)

    def test_full_close_preload_is_inside_approach_budget(self):
        for reverse in (False, True):
            cfg = dict(reverse=reverse, min_open_rad=0, max_open_rad=1.2)
            for opening in (0, 0.001, 0.05, 0.5, 1):
                actual = to_rad(opening, cfg)
                for budget in (0.35, 0.6, 1.1):
                    for preload in (0, 0.1, budget):
                        target, kp, kd, ff = approach_command(actual, to_rad(0,cfg), cfg, preload,budget)
                        self.assertLessEqual(abs(kp*(target-actual))+abs(ff), budget+1e-6)
                        self.assertGreaterEqual(normalized(target,cfg), -1e-6)
                        self.assertEqual(ff, preload if reverse else -preload)
        with self.assertRaises(ValueError):
            approach_command(-0.1,0,cfg,1.2,1.2)

    def test_only_recorded_deadline_can_be_explicitly_accepted(self):
        class FakeLink:
            def __init__(self, current=0, history=0, motor=0):
                fault=bytearray(64);fault[1]=0x0d
                struct.pack_into('<I',fault,12,motor)
                struct.pack_into('<II',fault,24,current,history)
                self.data={2:bytes([1,2,16,0]),4:b'TCGU01A28Z0015s',
                    0x59:b'\0EL05\0\0\0\0\x01',
                    0x67:bytes.fromhex('52505247010003002a359b3f00000000cdcc8c3f6666e63f0000000000000320'),
                    0x57:bytes([1,12,0,0]),0x52:bytes(fault),
                    0x50:struct.pack('<4fH',-0.5,0,0,30,0),0x51:bytes(48)}
            def read(self,cmd):return self.data[cmd]
        sn='TCGU01A28Z0015s'
        with self.assertRaises(RuntimeError):
            preflight(FakeLink(history=0x303),sn)
        self.assertTrue(preflight(FakeLink(history=0x303),sn,True)['recorded_deadline_accepted'])
        for kwargs in ({'current':0x303},{'history':0x304},{'motor':1}):
            with self.assertRaises(RuntimeError):
                preflight(FakeLink(**kwargs),sn,True)

    def test_full_close_disables_on_lost_feedback(self):
        class FakeLink:
            def __init__(self):
                self.seq=1;self.requests=[];self.writes=[];self.serial=self
            def write(self,value):self.writes.append(value)
            def request(self,cmd,*args,**kwargs):self.requests.append(cmd);return b'\0'
        link=FakeLink()
        report={'config':dict(reverse=True,min_open_rad=0,max_open_rad=1.2),'hold_limit_nm':1.1}
        first=dict(position_rad=-0.5,velocity_rad_s=0,torque_nm=0,temperature_c=30,status_bits=0)
        with patch('xtac_full_close.next_status',side_effect=[first,TimeoutError('lost feedback')]):
            with self.assertRaises(TimeoutError):run_full_close(link,report)
        self.assertEqual(link.requests[-2:],[0x31,0x21])
        self.assertTrue(report['cleanup']['disable_ack'])
        self.assertEqual(struct.unpack('<5f',unpack(link.writes[-1])[4]),(0,0,0,0,0))


if __name__ == '__main__':
    unittest.main()
