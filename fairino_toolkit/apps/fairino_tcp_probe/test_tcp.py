import copy
import json
import math
import queue
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'fairino_upper_probe'))
import robot_probe as rp
from probe_core import ALL_MASK,snapshot,JOINTS,check_hardware
from test_probe import fake_probe,STUBS,Future,goal
from tcp_core import *
from vendor_kinematics import VendorKinematics
from tcp_robot import TCPProbe
from tcp_keyboard import Window,verify_both

POSE_DEG=[93.0505,89.7396,-89.9978,104.7921,.0017,-.0085,-.0085,
          -85.9925,89.678,90.0122,105.1225,.025,-.0117,-.0089,
          10.62,-.05,-.09,-48.88,91.22,-43.12]


def hardware():
    q=list(map(math.radians,POSE_DEG))
    return snapshot({'joint_names':JOINTS,'position':q,'command_position':q[:],
        'velocity':[0.]*20,'stateword':[0x2240]*20,'errorcode':[0]*20,'hexerrorcode_curerrcode':0})


def request(kin,hw,arm='left',axis=0,direction=1,step=1.):
    q=[hw['position'][j-1] for j in ARMS[arm]]
    return {'id':1,'created':time.monotonic(),'model':MODEL_ID,'kind':'tcp_step','arm':arm,
            'axis':axis,'direction':direction,'step':step,'reference':q,
            'target':kin.step(q,axis,direction,step),'joint_speed':.5}


class MathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.kin=VendorKinematics()

    def test_real_vendor_forward_inverse_roundtrip_for_both_arms(self):
        proof,details=verify_both(self.kin,hardware())
        self.assertEqual(proof['model'],MODEL_ID)
        for d in details.values():self.assertLess(d['roundtrip_deg'],.0001)

    def test_all_twenty_four_cartesian_directions_hold_other_thirteen_axes(self):
        hw=hardware();held=hw['position'][:]
        for arm,joints in ARMS.items():
            for axis in range(6):
                for direction in (-1,1):
                    r=request(self.kin,hw,arm,axis,direction,1. if axis<3 else .2)
                    p=validate_plan(hw,held,r)
                    for i in range(20):
                        if i+1 not in joints:
                            self.assertEqual(p['start'][i],held[i]);self.assertEqual(p['target'][i],held[i])
                    expected=target_matrix(tcp_forward_matrix(r['reference']),axis,direction,r['step'])
                    mm,deg=pose_error(expected,self.kin.forward_tcp(r['target'])[0])
                    self.assertLess(mm,.001);self.assertLess(deg,.001)

    def test_rotations_keep_tcp_point_fixed_and_are_about_base_axes(self):
        t=tcp_forward_matrix(hardware()['position'][:7])
        for axis in range(3,6):
            target=target_matrix(t,axis,1,.2)
            self.assertEqual([r[3] for r in t],[r[3] for r in target])
            self.assertAlmostEqual(pose_error(t,target)[1],.2,places=5)

    def test_pose_display_conversion_roundtrip_including_euler_singularities(self):
        for p in ([1,2,3,10,20,30],[1,2,3,10,90,30],[1,2,3,10,-90,30]):
            t=pose_matrix(p);mm,deg=pose_error(t,pose_matrix(matrix_pose(t)))
            self.assertLess(mm,1e-9);self.assertLess(deg,1e-5)

    def test_invalid_packets_are_rejected_before_trajectory(self):
        hw=hardware();r=request(self.kin,hw)
        mutations=[{'model':'wrong'},{'kind':'joint'},{'arm':'body'},{'axis':6},
            {'axis':True},{'direction':True},{'step':10},{'step':float('nan')},
            {'target':[float('nan')]*7},{'target':[0.]*20},{'reference':[0.]*7},
            {'joint_speed':5},{'joint_speed':True}]
        for change in mutations:
            with self.subTest(change=change),self.assertRaises(ValueError):
                validate_plan(hw,hw['position'],{**r,**change})

    def test_modified_ik_target_is_rejected_by_independent_robot_fk(self):
        hw=hardware();r=request(self.kin,hw);r['target'][0]+=math.radians(.1)
        with self.assertRaisesRegex(ValueError,'独立正解'):validate_plan(hw,hw['position'],r)

    def test_large_joint_jump_and_model_joint_limits_rejected(self):
        hw=hardware();r=request(self.kin,hw)
        r['target'][0]+=math.radians(3)
        with self.assertRaisesRegex(ValueError,'超过 2'):validate_plan(hw,hw['position'],r)
        r=request(self.kin,hw);r['target'][3]=math.radians(111)
        with self.assertRaisesRegex(ValueError,'模型限制'):validate_plan(hw,hw['position'],r)

    def test_sender_duration_and_other_arm_overrides_have_no_effect(self):
        hw=hardware();r=request(self.kin,hw);r.update(duration=.001,joints=list(range(1,21)),full_target=[99.]*20)
        p=validate_plan(hw,hw['position'],r)
        self.assertEqual(p['joints'],list(range(1,8)))
        self.assertEqual(p['target'][7:],hw['position'][7:])
        peak=max(abs(math.degrees(a-b)) for a,b in zip(p['start'],p['target']))*1.875/p['duration']
        self.assertLessEqual(peak,.5+1e-9)

    def test_held_axes_guard_and_speed_guard_remain_active_during_arm_trajectory(self):
        hw=hardware();held=hw['position'][:];hw['position'][0]+=math.radians(.5)
        hw['position'][6]+=math.radians(.5);hw['velocity'][6]=math.radians(1.)
        opts={'held_positions':held,'moving_joints':ARMS['left']}
        check_hardware(hw,0,[0,0],**opts)
        for other in (8,14,15,16,17,18,19,20):
            hw['position'][other-1]+=math.radians(.31)
            with self.assertRaisesRegex(RuntimeError,f'J{other} 偏离'):check_hardware(hw,0,[0,0],**opts)
            hw['position'][other-1]=held[other-1]
        hw['velocity'][6]=math.radians(2.1)
        with self.assertRaisesRegex(RuntimeError,'J7 速度'):check_hardware(hw,0,[0,0],**opts)


def probe():
    p=fake_probe();p.__class__=TCPProbe;p.moving_joints=None
    p.hw=hardware();p.baseline=p.hw['position'][:];p.hold_positions=p.hw['position'][:]
    return p


class LifecycleTests(unittest.TestCase):
    def test_enable_waits_for_verified_reference_then_uses_existing_lifecycle(self):
        p=probe();p.packet['kinematics']={'model':MODEL_ID,'reference':p.hw['position'][:14]}
        p.enable_all()
        self.assertEqual(p.commands,[f'enable({ALL_MASK})']);self.assertTrue(p.session_enabled)

    def test_bad_kinematics_proof_never_reaches_enable(self):
        for proof in (None,{'model':'wrong','reference':[0.]*14},{'model':MODEL_ID,'reference':[0.]*14}):
            p=probe();p.packet['kinematics']=proof
            tick=[0]
            def pump(*_):tick[0]+=11
            p.pump=pump
            with patch('tcp_robot.time.monotonic',lambda:tick[0]),self.assertRaisesRegex(RuntimeError,'拒绝使能'):
                p.enable_all()
            self.assertEqual(p.commands,[])

    def test_tcp_trajectory_holds_body_and_cleanup_disables_all(self):
        kin=VendorKinematics()
        for arm in ARMS:
            p=probe();rp.Probe.enable_all(p);r=request(kin,p.hw,arm)
            def send(g):
                p.sent.append(g)
                p.hw['position']=g.trajectory.points[-1].positions[:]
                p.hw['command_position']=p.hw['position'][:]
                p.guard()
                return Future(NS(accepted=True,get_result_async=lambda:Future(NS(status=4,result=NS(error_code=0,error_string=''))),cancel_goal_async=lambda:Future(NS())))
            p.trajectory_client=NS(send_goal_async=send)
            with patch.dict(rp.__dict__,STUBS):
                p.perform_tcp_step(r)
                p.moving_joints=None;p.guard()
                for point in p.sent[0].trajectory.points:self.assertEqual(point.positions[14:],p.baseline[14:])
                self.assertEqual(p.commands,[f'enable({ALL_MASK})'])
                self.assertTrue(p.cleanup())
            self.assertEqual(p.commands[-1],f'disable({ALL_MASK})')

    def test_cancel_keeps_all_enabled_and_fault_cleanup_disables_all(self):
        p=probe();rp.Probe.enable_all(p);p.moving_joints=ARMS['left']
        p.goal=NS(accepted=True,get_result_async=lambda:Future(NS(status=5)),cancel_goal_async=lambda:Future(NS()))
        p.cancel_step();self.assertTrue(all(p.hw['enabled']))
        p.gpio=lambda:[1,1]
        with self.assertRaises(RuntimeError):p.guard()
        with patch.dict(rp.__dict__,STUBS):self.assertTrue(p.cleanup())
        self.assertFalse(any(p.hw['enabled']))


try:
    import gi
    gi.require_version('Gtk','3.0')
    from gi.repository import Gtk,Gdk
    GTK=Gtk.init_check()[0]
except ImportError:GTK=False


@unittest.skipUnless(GTK,'GTK/display unavailable')
class GuiTests(unittest.TestCase):
    def setUp(self):
        class HiddenWindow(Gtk.OffscreenWindow):
            def present(self):pass
            def is_active(self):return True
        with patch.object(Gtk,'Window',HiddenWindow):
            self.w=Window(NS(demo=True,arm='left'),VendorKinematics())
        self.w.update_hardware(hardware());self.w.ready=True;self.w.refresh_controls()
        while Gtk.events_pending():Gtk.main_iteration_do(False)

    def tearDown(self):
        self.w.closing=True;self.w.window.destroy()

    def test_all_twelve_keys_hold_without_repeat_and_release_clears_request(self):
        from tcp_jog import KEYS
        w=self.w
        for arm in ARMS:
            w.arm_buttons[arm].set_active(True)
            for key,(axis,direction) in KEYS.items():
                w.press(key)
                self.assertEqual(w.request['axis'],axis)
                self.assertEqual(w.request['direction'],direction)
                self.assertEqual(w.request['arm'],arm)
                before=w.sequence;w.press(key)
                self.assertEqual(w.sequence,before)
                w.last_plan=0;w.tick()
                self.assertGreater(w.sequence,before)
                w.release(key);self.assertIsNone(w.request)
                self.assertIsNone(w.active_key)

    def test_multikey_and_space_require_all_keys_released_before_restart(self):
        w=self.w;w.press('w');w.press('a')
        self.assertIsNone(w.request);self.assertTrue(w.input_blocked)
        w.release('a');w.press('w');self.assertIsNone(w.request)
        w.release('w');w.press('s');self.assertIsNotNone(w.request)
        w.cancel();w.press('s');self.assertIsNone(w.request)
        w.release('s');w.press('s');self.assertIsNotNone(w.request)
        self.assertTrue(all(not b.get_sensitive() for b in w.arm_buttons.values()))
        self.assertFalse(w.speed.get_sensitive())

    def test_startup_stale_feedback_editing_and_focus_cannot_start_motion(self):
        w=self.w;w.proof=None;w.press('w');self.assertIsNone(w.request)
        w.update_hardware(hardware());w.press('w');self.assertIsNone(w.request)
        w.release('w');w.translation.grab_focus()
        self.assertFalse(w.key_press(None,NS(keyval=Gdk.KEY_w)))
        self.assertIsNone(w.request);w.control_area.grab_focus()
        w.args.demo=False;sent=[]
        w.connection=NS(latest={'stamp':time.monotonic()},events=queue.Queue(),send=lambda *a,**kw:sent.append((a,kw)))
        w.last_status-=1;w.press('w');self.assertIsNone(w.request)
        w.release('w');w.update_hardware(hardware())
        w.connection.events.put({'phase':'保持','ready':True,'jogging':False,'stamp':w.stop_stamp+.01})
        w.tick();w.press('w');self.assertIsNotNone(w.request)
        w.had_focus=True;w.focus_out();self.assertFalse(sent[-1][1]['exit'])
        self.assertFalse(sent[-1][1]['focused']);self.assertIsNone(sent[-1][0][0])
        self.assertIsNone(w.request);self.assertFalse(w.stopped)
        self.assertTrue(w.input_suspended);self.assertTrue(w.awaiting_stop)
        w.connection.events.put({'phase':'保持','ready':True,'jogging':False,'stamp':w.stop_stamp+.01})
        w.tick();self.assertFalse(w.awaiting_stop)
        w.focus_in();w.tick();self.assertIsNone(w.request)
        w.press('w');self.assertIsNone(w.request) # old key/autorepeat must not resume
        w.release('w');w.press('w');self.assertIsNotNone(w.request)

    def test_inline_arm_selection_keeps_session_and_does_not_issue_motion(self):
        w=self.w;sent=[]
        w.connection=NS(send=lambda *a,**kw:sent.append((a,kw)))
        for arm in ('right','left'):
            w.arm_buttons[arm].set_active(True);w.send()
            self.assertEqual(w.arm,arm);self.assertTrue(w.ready)
            self.assertFalse(w.stopped);self.assertIsNone(w.request)
            self.assertFalse(sent[-1][1]['exit'])
            self.assertIsNone(sent[-1][0][0])

    def test_focus_loss_during_startup_does_not_cancel_enabling(self):
        w=self.w;sent=[];w.ready=False;w.had_focus=True
        w.connection=NS(send=lambda *a,**kw:sent.append((a,kw)))
        epoch=w.cancel_count;w.focus_out();w.send()
        self.assertEqual(w.cancel_count,epoch)
        self.assertFalse(sent[-1][1]['exit']);self.assertFalse(sent[-1][1]['focused'])
        w.focus_in();w.press('w');self.assertIsNone(w.request)

    def test_inactive_window_never_sends_motion_even_before_focus_event(self):
        w=self.w;w.press('w');self.assertIsNotNone(w.request);sent=[]
        w.connection=NS(send=lambda *a,**kw:sent.append((a,kw)))
        with patch.object(w.window,'is_active',return_value=False):w.send()
        self.assertIsNone(sent[-1][0][0]);self.assertFalse(sent[-1][1]['exit'])
        self.assertFalse(sent[-1][1]['focused'])

    def test_x11_autorepeat_release_press_pair_does_not_stop_or_duplicate(self):
        w=self.w;event=NS(keyval=Gdk.KEY_w)
        w.key_press(None,event);seq=w.sequence
        w.key_release(None,event);self.assertIn('w',w.releases)
        w.key_press(None,event);self.assertNotIn('w',w.releases)
        self.assertEqual(w.sequence,seq);self.assertIsNotNone(w.request)
        w.release('w');self.assertIsNone(w.request)

    def test_real_key_release_timer_clears_active_motion(self):
        w=self.w;event=NS(keyval=Gdk.KEY_w)
        w.key_press(None,event);w.key_release(None,event)
        deadline=time.monotonic()+.1
        while w.releases and time.monotonic()<deadline:
            while Gtk.events_pending():Gtk.main_iteration_do(False)
            time.sleep(.001)
        self.assertIsNone(w.request);self.assertIsNone(w.active_key)
        self.assertFalse(w.held)


if __name__=='__main__':unittest.main()
