"""Regression tests for the flange point, including the real vendor library."""
import json
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

from tcp_core import (ARMS,FLANGE_OFFSET_MM,MODEL_ID,forward_matrix,tcp_forward_matrix,
                      wrist_to_flange,flange_to_wrist,pose_matrix,pose_error,target_matrix)
from tcp_jog import make_request,validate_velocity,trajectory,sample,braking,PERIOD
from tcp_frames import CHEST_FRAME_ID,tcp_in_chest
from vendor_kinematics import VendorKinematics
from tcp_keyboard import verify_both,Window
from test_tcp import hardware,GTK,Gtk
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).parent/'analysis'))
from model_audit import URDF,pose_point,distance


class FlangeTests(unittest.TestCase):
    def setUp(self):self.kin=VendorKinematics()

    def test_offset_is_rotated_with_wrist_and_inverse_applied_once(self):
        self.assertEqual(FLANGE_OFFSET_MM,(-106.,0.,0.))
        for pose in ([0,0,0,0,0,0],[25,30,50,0,0,90],[25,30,50,30,40,50]):
            wrist=pose_matrix(pose);flange=wrist_to_flange(wrist)
            for i in range(3):self.assertAlmostEqual(flange[i][3]-wrist[i][3],-106*wrist[i][0])
            self.assertLess(pose_error(wrist,flange_to_wrist(flange))[0],1e-10)
        flange=wrist_to_flange(pose_matrix([0,0,0,0,0,90]))
        self.assertAlmostEqual(flange[0][3],0.);self.assertAlmostEqual(flange[1][3],-106.)

    def test_raw_vendor_interface_remains_wrist_and_tcp_interface_uses_flange(self):
        for arm,indices in ARMS.items():
            q=[hardware()['position'][j-1] for j in indices]
            raw,angle=self.kin.forward(q);tcp,_=self.kin.forward_tcp(q)
            self.assertAlmostEqual(pose_error(raw,tcp)[0],106.)
            self.assertLess(pose_error(tcp,tcp_forward_matrix(q))[0],1e-8)
            restored=self.kin.inverse_tcp(tcp,angle,q)
            self.assertLess(max(abs(a-b) for a,b in zip(restored,q)),1e-7)

    def test_rotations_hold_flange_in_independent_urdf_and_compensate_wrist(self):
        for arm,indices in ARMS.items():
            model=URDF(arm);hw=hardware();q=[hw['position'][j-1] for j in indices]
            p0=pose_point(model.fk(q),[-106,0,0]);wrist0=forward_matrix(q)
            wrist_moves=[]
            for axis in (3,4,5):
                for direction in (-1,1):
                    r=make_request(self.kin,hw,arm,axis,direction,8,2,4)
                    p1=pose_point(model.fk(r['target']),[-106,0,0])
                    self.assertLess(distance(p0,p1),.005)
                    wrist_moves.append(pose_error(wrist0,forward_matrix(r['target']))[0])
            self.assertGreater(max(wrist_moves),.2)

    def test_receiver_rejects_old_wrist_target_even_with_new_model_label(self):
        hw=hardware();r=make_request(self.kin,hw,'left',4,1,8,2,4)
        raw,angle=self.kin.forward(r['reference'])
        r['target']=self.kin.inverse(target_matrix(raw,4,1,r['step']),angle,r['reference'])
        r.update(id=1,created=10.)
        with self.assertRaisesRegex(ValueError,'正解'):validate_velocity(hw,r,10.)
        r=make_request(self.kin,hw,'left',4,1,8,2,4)
        r.update(id=1,created=10.,model='ART7-R7-20260615-zero-tool-v1')
        with self.assertRaisesRegex(ValueError,'模型'):validate_velocity(hw,r,10.)

    def test_startup_check_reports_flange_and_version(self):
        proof,details=verify_both(self.kin,hardware())
        self.assertEqual(proof['model'],MODEL_ID)
        self.assertIn('flange',MODEL_ID)
        for arm,indices in ARMS.items():
            self.assertEqual(details[arm]['tcp_definition'],'flange_face_center')
            self.assertEqual(details[arm]['coordinate_frame'],CHEST_FRAME_ID)
            expected=tcp_in_chest(arm,tcp_forward_matrix([hardware()['position'][j-1] for j in indices]))
            for i in range(3):self.assertAlmostEqual(details[arm]['pose_mm_deg'][i],expected[i][3])

    def test_rotation_anchor_cannot_jump_or_change_during_hold(self):
        hw=hardware();r=make_request(self.kin,hw,'left',4,1,8,2,4);r.update(id=1,created=10.)
        with self.assertRaisesRegex(ValueError,'保持点'):
            validate_velocity(hw,{**r,'rotation_anchor':[0.,0.,0.]},10.)
        shifted=r['rotation_anchor'][:];shifted[0]+=.01
        altered=make_request(self.kin,hw,'left',4,1,8,2,4,rotation_anchor=shifted)
        altered.update(id=2,created=10.)
        with self.assertRaisesRegex(ValueError,'不能更改'):
            validate_velocity(hw,altered,10.,previous=r)

    def test_continuous_rotation_with_delayed_feedback_and_release_keeps_flange(self):
        poses=json.loads((Path(__file__).parent/'analysis/flange_runtime_poses.json').read_text())
        metrics=[]
        for label,held in poses.items():
            for arm,indices in ARMS.items():
                model=URDF(arm)
                initial_point=pose_point(model.fk([held[j-1] for j in indices]),[-106,0,0])
                for axis in (3,4,5):
                    for direction in (-1,1):
                        history=[];path=None;anchor=None;previous=None;error=0.;stop=None
                        for tick in range(1000):
                            q,v=sample(path,PERIOD) if path else (held[:],[0.]*20)
                            history.append(q[:])
                            # Model the GUI using command feedback up to 100 ms old.
                            reference=history[max(0,len(history)-6)]
                            now=tick*PERIOD
                            hw={'position':q,'command_position':q[:]}
                            actual=pose_point(model.fk([q[j-1] for j in indices]),[-106,0,0])
                            error=max(error,distance(actual,initial_point))
                            try:
                                if tick%5==0:
                                    r=make_request(self.kin,{'command_position':reference},arm,axis,direction,
                                                   8,2,4,rotation_anchor=anchor)
                                    r.update(id=tick+1,created=max(0,now-.1))
                                    desired=validate_velocity(hw,r,now,previous=previous)
                                    anchor=r['rotation_anchor'];previous=r
                                path=trajectory(q,v,desired,indices,held)
                            except ValueError as exc:
                                self.assertIn('限',str(exc));stop=str(exc);break
                        stop_path=braking(q,v)
                        for step in range(21):
                            final,_=sample(stop_path,stop_path[-1][0]*step/20.)
                            actual=pose_point(model.fk([final[j-1] for j in indices]),[-106,0,0])
                            error=max(error,distance(actual,initial_point))
                        self.assertLess(error,.1,(label,arm,axis,direction,error))
                        metrics.append({'pose':label,'arm':arm,'axis':axis,'direction':direction,
                            'simulated_seconds':tick*PERIOD,'max_flange_drift_mm':error,'stop_reason':stop})
        self.__class__.continuous_metrics=metrics

    @unittest.skipUnless(GTK,'GTK/display unavailable')
    def test_gui_shows_flange_position_for_both_arms(self):
        class Hidden(Gtk.OffscreenWindow):
            def present(self):pass
            def is_active(self):return True
        with patch.object(Gtk,'Window',Hidden):w=Window(NS(demo=True,arm='left'),self.kin)
        try:
            for arm,indices in ARMS.items():
                w.arm=arm;w.update_hardware(hardware())
                t=tcp_in_chest(arm,tcp_forward_matrix([hardware()['position'][j-1] for j in indices]))
                for i in range(3):self.assertEqual(w.values[i].get_text(),f'{t[i][3]:+.3f} mm')
        finally:w.closing=True;w.window.destroy()


if __name__=='__main__':unittest.main()
