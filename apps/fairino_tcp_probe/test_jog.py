import copy
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from tcp_jog import *
from tcp_core import ARMS,tcp_forward_matrix,pose_error
from tcp_frames import tcp_in_chest
from vendor_kinematics import VendorKinematics
from test_tcp import hardware,probe
import robot_probe as rp
from test_probe import STUBS


class ContinuousMathTests(unittest.TestCase):
    def setUp(self):self.kin=VendorKinematics();self.hw=hardware()

    def request(self,arm='left',axis=0,direction=1):
        r=make_request(self.kin,self.hw,arm,axis,direction,8,2,4)
        r.update(id=1,created=10.)
        return r

    def test_24_directions_integrate_continuously_and_hold_other_axes(self):
        for arm,indices in ARMS.items():
            for axis in range(6):
                for direction in (-1,1):
                    hw=hardware();held=hw['position'][:];path=None
                    before=tcp_in_chest(arm,tcp_forward_matrix([held[j-1] for j in indices]));speeds=[]
                    for tick in range(100):
                        q,v=sample(path,PERIOD) if path else (held[:],[0.]*20)
                        hw['position']=q;hw['command_position']=q[:]
                        if tick%5==0:
                            r=make_request(self.kin,hw,arm,axis,direction,8,2,4)
                            r.update(id=tick+1,created=tick*PERIOD)
                            desired=validate_velocity(hw,r,tick*PERIOD)
                        path=trajectory(q,v,desired,indices,held)
                        speeds.append(max(abs(math.degrees(x)) for x in v))
                        for j in range(1,21):
                            if j not in indices:
                                for _,p,vel in path:self.assertEqual(p[j-1],held[j-1]);self.assertEqual(vel[j-1],0.)
                        for t in (0,.02,.04,.08,.1,.2,.28):
                            p,vel=sample(path,t)
                            self.assertLessEqual(max(abs(math.degrees(x)) for x in vel),MAX_SPEED+1e-7)
                    self.assertGreater(min(speeds[20:]),.1) # no periodic stop between requests
                    after=tcp_in_chest(arm,tcp_forward_matrix([q[j-1] for j in indices]))
                    mm,deg=pose_error(before,after)
                    self.assertGreater(mm if axis<3 else deg,1.)
                    if axis<3:self.assertGreater(direction*(after[axis][3]-before[axis][3]),1.)

    def test_fourfold_limits_and_receiver_rejects_tampering_stale_and_unknown(self):
        r=self.request()
        for speed in (.1,2.,4.):
            v=validate_velocity(self.hw,{**r,'joint_speed':speed},10.)
            self.assertLessEqual(max(abs(math.degrees(x)) for x in v),speed+1e-9)
        for change in ({'kind':'tcp_step'},{'model':'x'},{'id':True},{'created':9.7},
                       {'created':11},{'created':float('nan')},{'joint_speed':4.1},
                       {'joint_speed':True},{'rate':8.1},{'axis':True},{'direction':0},
                       {'arm':'body'},{'target':[float('nan')]*7}):
            with self.subTest(change=change),self.assertRaises(ValueError):
                validate_velocity(self.hw,{**r,**change},10.)
        r['target'][1]+=math.radians(.1)
        with self.assertRaisesRegex(ValueError,'正解'):validate_velocity(self.hw,r,10.)

    def test_finite_tail_and_release_brake_stop_without_new_messages(self):
        q=self.hw['position'];v=[math.radians(4.)]+[0.]*19
        path=trajectory(q,v,v,ARMS['left'],q)
        final,stopped=sample(path,100)
        self.assertEqual(stopped,[0.]*20)
        self.assertLessEqual(math.degrees(final[0]-q[0]),1.13)
        stop=braking(q,v)
        self.assertAlmostEqual(stop[-1][0],.2)
        self.assertAlmostEqual(math.degrees(stop[-1][1][0]-q[0]),.4)
        for tick in range(21):
            _,vel=sample(stop,tick*.01)
            self.assertLessEqual(abs(vel[0]),abs(v[0])+1e-9)
        self.assertEqual(sample(stop,1.)[1],[0.]*20)

    def test_reversal_has_no_velocity_overshoot_and_stopping_tail_respects_limits(self):
        q=self.hw['position'][:];v=[math.radians(4.)]+[0.]*19;d=[-v[0]]+[0.]*19
        path=trajectory(q,v,d,ARMS['left'],q)
        for i in range(281):
            _,vel=sample(path,i*.001)
            self.assertLessEqual(abs(vel[0]),abs(v[0])+1e-9)
        q[0]=math.radians(174.9)
        with self.assertRaisesRegex(ValueError,'限位'):trajectory(q,v,v,ARMS['left'],q)


class EndSimulation(Exception):pass


class RobotLoopTests(unittest.TestCase):
    def test_active_arm_speed_margin_does_not_relax_held_body_or_tracking(self):
        p=probe();rp.Probe.enable_all(p);p.moving_joints=ARMS['left']
        p.hw['velocity'][0]=math.radians(4.);p.guard()
        p.hw['velocity'][0]=math.radians(8.1)
        with self.assertRaisesRegex(RuntimeError,'J1 速度'):p.guard()
        p.hw['velocity'][0]=0.;p.hw['velocity'][17]=math.radians(.6)
        with self.assertRaisesRegex(RuntimeError,'J18 速度'):p.guard()
        p.hw['velocity'][17]=0.;p.hw['position'][0]+=.0041
        with self.assertRaisesRegex(RuntimeError,'跟踪误差'):p.guard()

    def simulate(self,mode,axis=0):
        p=probe();kin=VendorKinematics();clock=[10.];steps=[0];published=[];stream_request=[None]
        p.node=NS(get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:clock[0])),
                  create_publisher=lambda *a:NS(get_subscription_count=lambda:1,publish=published.append))
        def prepare():
            p.hw_time=p.received=clock[0];p.packet['ack']=clock[0]
            rp.Probe.enable_all(p);p.ready=True
        p.prepare=prepare
        def pump(seconds=.02,guarded=True):
            clock[0]+=seconds;steps[0]+=1
            if getattr(p,'path',None) is not None:
                q,v=sample(p.path,clock[0]-p.path_time)
                p.hw['position']=q[:];p.hw['command_position']=q[:];p.hw['velocity']=v[:]
            p.hw_time=p.received=clock[0];p.packet['ack']=clock[0]
            if steps[0]>65:raise EndSimulation()
            if steps[0]<22:
                if steps[0]%4==1 or stream_request[0] is None:
                    r=make_request(kin,p.hw,'left',axis,1,8,2,4,
                        rotation_anchor=(stream_request[0] or {}).get('rotation_anchor'))
                    r.update(id=steps[0],created=clock[0]);stream_request[0]=r
                p.packet['request']=copy.deepcopy(stream_request[0])
            elif mode=='release':p.packet['request']=None
            elif mode=='unfocused':p.packet['focused']=False # deliberately retain queued motion
            elif mode=='stale':pass # heartbeat continues; old IK lease must still expire
            elif mode=='invalid':
                r=copy.deepcopy(stream_request[0]);r.update(id=1000,created=clock[0],target=[0.]*7)
                p.packet['request']=r
            elif mode=='estop':p.gpio=lambda:[1,1]
            if guarded:p.guard()
        p.pump=pump
        stubs={**STUBS,'JointTrajectory':lambda:NS(joint_names=[],points=[],header=NS(stamp=None))}
        with patch.dict(rp.__dict__,stubs),patch('tcp_robot.time.monotonic',lambda:clock[0]):
            if mode=='estop':
                with self.assertRaisesRegex(RuntimeError,'急停'):p.run()
                self.assertTrue(p.cleanup());self.assertFalse(any(p.hw['enabled']))
            else:
                with self.assertRaises(EndSimulation):p.run()
                self.assertIsNone(p.path);self.assertIsNone(p.moving_joints)
                self.assertTrue(all(p.hw['enabled']))
                self.assertEqual(p.hw['velocity'],[0.]*20)
            self.assertGreater(len(published),10)
            for msg in published:
                self.assertEqual(msg.joint_names,rp.JOINTS)
                self.assertEqual(msg.points[-1].velocities,[0.]*20)
                for point in msg.points:self.assertEqual(point.positions[7:],p.baseline[7:])
            return p,published

    def test_release_stops_stream_and_keeps_all_enabled(self):self.simulate('release')
    def test_focus_loss_stops_queued_motion_and_keeps_session_enabled(self):
        p,_=self.simulate('unfocused')
        self.assertIsNotNone(p.packet['request'])
        self.assertFalse(any('jog_rejected' in e or 'error' in e for e in p.events))
        self.assertTrue(p.ready)
    def test_old_request_expires_despite_live_heartbeat(self):
        p,_=self.simulate('stale')
        self.assertTrue(any('jog_rejected' in e for e in p.events))
    def test_invalid_ik_stops_and_latches_until_release(self):self.simulate('invalid')
    def test_estop_still_disables_all_axes(self):self.simulate('estop')

    def test_held_flange_rotation_runs_through_receiver_and_stops(self):self.simulate('release',axis=4)


if __name__=='__main__':unittest.main()
