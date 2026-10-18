import unittest
from unittest.mock import patch

from xtac_grip import GripDetector, grip
from xtac_motion_test import check_sample


def sample(position=-0.4, velocity=0, torque=0.7, enabled=1):
    return dict(position_rad=position, velocity_rad_s=velocity, torque_nm=torque,
                temperature_c=30, status_bits=enabled)


class GripTests(unittest.TestCase):
    def test_disabled_magnitude_trips_preserve_other_checks(self):
        cfg=dict(reverse=True,min_open_rad=0,max_open_rad=1.2)
        high=sample(velocity=1.0,torque=1.4)
        check_sample(high,cfg,max_torque=None,max_speed=None)
        with self.assertRaises(RuntimeError):check_sample(high,cfg)
        for changes in ({'status_bits':3},{'temperature_c':46},
                        {'position_rad':-2},{'velocity_rad_s':float('nan')}):
            with self.assertRaises(RuntimeError):
                check_sample(dict(high,**changes),cfg,max_torque=None,max_speed=None)

    def test_object_contact_completes_without_reaching_zero(self):
        detector=GripDetector(2)
        for i in range(260):
            contact,done=detector.update(sample(),i/100)
        self.assertTrue(contact)
        self.assertTrue(done)

    def test_motion_and_absent_force_are_not_contact(self):
        for moving in (False,True):
            detector=GripDetector(0.1)
            for i in range(200):
                s=sample(position=-0.8+i*0.002,velocity=0.2) if moving else sample(torque=0)
                contact,done=detector.update(s,i/100)
                self.assertFalse(contact)
                self.assertFalse(done)

    def test_slip_restarts_hold_timer(self):
        detector=GripDetector(2)
        for i in range(100):detector.update(sample(),i/100)
        self.assertIsNotNone(detector.hold_since)
        self.assertEqual(detector.update(sample(position=-0.35,velocity=0.2),1),(False,False))
        self.assertIsNone(detector.hold_since)

    def test_lost_feedback_requests_disable(self):
        class FakeLink:
            def __init__(self):self.seq=1;self.serial=self;self.requests=[]
            def write(self,_value):pass
            def request(self,cmd,*args,**kwargs):self.requests.append(cmd)
        link=FakeLink()
        report={'config':dict(reverse=True,min_open_rad=0,max_open_rad=1.2)}
        with patch('xtac_grip.next_status',side_effect=[sample(torque=0,enabled=0),TimeoutError('lost')]):
            with self.assertRaises(TimeoutError):grip(link,report,2)
        self.assertEqual(link.requests[-2:],[0x31,0x21])
        self.assertTrue(report['cleanup']['disable_ack'])


if __name__=='__main__':unittest.main()
