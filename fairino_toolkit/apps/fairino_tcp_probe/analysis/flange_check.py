"""Offline fixed-flange-point IK test. Does not import ROS or send commands."""
from model_audit import *
from tcp_core import rotation,multiply

OFFSET_MM=(-106.,0.,0.)
def flange_pose(wrist):
 t=[row[:] for row in wrist]
 for i,v in enumerate(pose_point(wrist,OFFSET_MM)):t[i][3]=v
 return t

def wrist_for_flange(flange):
 t=[row[:] for row in flange]
 for i in range(3):t[i][3]-=sum(t[i][j]*OFFSET_MM[j] for j in range(3))
 return t

def main():
 kin=VendorKinematics();rows=[]
 selected={}
 for s in segments():
  if s['axis']>=3 and s['samples']:selected.setdefault(s['log'],s['samples'][0]['hardware'])
 for label,hw in selected.items():
  for arm,indices in ARMS.items():
   q0=[hw['position'][j-1] for j in indices]
   start,arm_angle=kin.forward(q0);tcp0=flange_pose(start)
   for axis in range(3):
    for direction in (-1,1):
     q=q0[:];errors=[];urdf_errors=[];max_jump=0;failure=None
     u=URDF(arm);u0=flange_pose(u.fk(q0))
     for i in range(1,201):
      # 0.1 deg steps on one anchored Cartesian rotation, up to 20 deg.
      desired=multiply(rotation(axis,math.radians(direction*i*.1)),tcp0)
      for k in range(3):desired[k][3]=tcp0[k][3]
      try:qnew=kin.inverse(wrist_for_flange(desired),arm_angle,q)
      except ValueError as e:failure=str(e);break
      max_jump=max(max_jump,max(abs(math.degrees(a-b)) for a,b in zip(qnew,q)))
      q=qnew;actual=flange_pose(kin.forward(q)[0]);errors.append(pose_error(desired,actual)[0])
      urdf_errors.append(distance(pose_point(u.fk(q),OFFSET_MM),[u0[k][3] for k in range(3)]))
     rows.append({'pose_log':label,'arm':arm,'axis':axis,'direction':direction,'steps':len(errors),
       'requested_deg':20.,'achieved_deg':len(errors)*.1,'max_flange_error_mm':max(errors,default=0),
       'independent_urdf_flange_drift_mm':max(urdf_errors,default=0),'max_joint_step_deg':max_jump,'stop_reason':failure})
 result={'offset_mm':OFFSET_MM,'definition':'center of outer flange face in supplied wrist2_Link mesh; no gripper offset',
         'method':'fixed Cartesian flange point, rotating orientation, then remove flange offset before vendor IK',
         'tests':rows}
 (HERE/'flange_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 print('attempts',len(rows),'complete',sum(r['steps']==200 for r in rows),
       'max_vendor_mm',max(r['max_flange_error_mm'] for r in rows),
       'max_urdf_mm',max(r['independent_urdf_flange_drift_mm'] for r in rows))
 for r in rows:
  if r['stop_reason']:print('STOP',r)
if __name__=='__main__':main()
