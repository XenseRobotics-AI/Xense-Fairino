"""Read-only, reproducible comparison of URDF, vendor FK/IK, logs and jog math."""
import sys,json,math,random,hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT))
from tcp_core import ARMS,BOUNDS_DEG,forward_matrix,pose_error,target_matrix
from vendor_kinematics import VendorKinematics
from tcp_jog import make_request,validate_velocity,trajectory,sample,PERIOD
DOCS=ROOT.parent.parent/'models'

def mmul(a,b):return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
def eye():return [[float(i==j) for j in range(4)] for i in range(4)]
def axis_rotation(axis,angle):
 n=math.sqrt(sum(v*v for v in axis));x,y,z=[v/n for v in axis];c=math.cos(angle);s=math.sin(angle);k=1-c
 return [[c+x*x*k,x*y*k-z*s,x*z*k+y*s,0],[y*x*k+z*s,c+y*y*k,y*z*k-x*s,0],
         [z*x*k-y*s,z*y*k+x*s,c+z*z*k,0],[0,0,0,1]]
def vals(s):return list(map(float,s.split()))
class URDF:
 def __init__(self,arm):
  self.path=DOCS/f'{arm}_arm.urdf'
  self.joints=ET.parse(self.path).getroot().findall('joint')
 def fk(self,q):
  t=eye()
  for j,angle in zip(self.joints,q):
   origin=j.find('origin');xyz=vals(origin.get('xyz'));roll,pitch,yaw=vals(origin.get('rpy'))
   o=mmul(mmul(axis_rotation([0,0,1],yaw),axis_rotation([0,1,0],pitch)),axis_rotation([1,0,0],roll))
   for i in range(3):o[i][3]=1000*xyz[i]
   t=mmul(mmul(t,o),axis_rotation(vals(j.find('axis').get('xyz')),angle))
  return t

def pose_point(t,point):return [t[i][3]+sum(t[i][j]*point[j] for j in range(3)) for i in range(3)]
def distance(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
def segments():
 out=[]
 for p in sorted((ROOT/'logs').glob('20261006-*.jsonl')):
  if p.stem<'20261006-165900':continue
  last=None;active=None
  for line in p.open():
   d=json.loads(line)
   if 'jog_started' in d:
    if active:out.append(active)
    active={'log':p.name,**d['jog_started'],'start_time':d['local_time'],'samples':[]}
    if last:active['samples'].append(last)
   if d.get('hardware'):
    last=d
    if active:
     active['samples'].append(d)
     if not d.get('jogging',False) and d.get('ready'):
      out.append(active);active=None
  if active:out.append(active)
 return out

def main():
 kin=VendorKinematics();models={arm:URDF(arm) for arm in ARMS};rng=random.Random(20261006)
 report={'source_sha256':{str(m.path):hashlib.sha256(m.path.read_bytes()).hexdigest() for m in models.values()},'model_comparison':{},'log_segments':[],'continuous_simulation':[]}
 for arm,model in models.items():
  raw=[];aligned=[];rotation_error=[]
  offset=vals(model.joints[0].find('origin').get('xyz'))[:2]
  for _ in range(200):
   q=[math.radians(rng.uniform(lo+5,hi-5)) for lo,hi in BOUNDS_DEG]
   u=model.fk(q);v=kin.forward(q)[0];raw.append(pose_error(u,v)[0])
   for i in range(2):u[i][3]-=offset[i]*1000
   mm,deg=pose_error(u,v);aligned.append(mm);rotation_error.append(deg)
  report['model_comparison'][arm]={'poses':200,'base_xy_offset_mm':[v*1000 for v in offset],
    'raw_max_position_mm':max(raw),'aligned_max_position_mm':max(aligned),'max_orientation_deg':max(rotation_error)}
 for s in segments():
  if len(s['samples'])<2:continue
  arm=s['arm'];indices=ARMS[arm]
  ref=s['samples'][0];q0=[ref['hardware']['position'][j-1] for j in indices]
  t0=models[arm].fk(q0);base_point=[t0[i][3] for i in range(3)]
  points={field:[] for field in ('position','command_position')}
  tool_points=[];angles=[];tracking=[]
  for d in s['samples']:
   for field in points:
    q=[d['hardware'][field][j-1] for j in indices];t=models[arm].fk(q)
    points[field].append([t[i][3] for i in range(3)])
   t=models[arm].fk([d['hardware']['position'][j-1] for j in indices])
   angles.append(pose_error(t0,t)[1]);tool_points.append(pose_point(t,[-106,0,0]))
   tracking.append(max(abs(math.degrees(d['hardware']['position'][j-1]-d['hardware']['command_position'][j-1])) for j in indices))
  r={k:v for k,v in s.items() if k!='samples'}
  r.update(duration_s=s['samples'][-1]['local_time']-s['start_time'],samples=len(s['samples']),
    rotation_deg=max(angles),max_origin_displacement_mm=max(distance(base_point,p) for p in points['position']),
    max_command_origin_displacement_mm=max(distance(points['command_position'][0],p) for p in points['command_position']),
    final_xyz_delta_mm=[points['position'][-1][i]-base_point[i] for i in range(3)],
    candidate_offset_point_displacement_mm=max(distance(tool_points[0],p) for p in tool_points),
    max_joint_tracking_error_deg=max(tracking))
  report['log_segments'].append(r)
 poses=[]
 for s in segments():
  if s['axis']>=3 and len(s['samples'])>2:
   if not poses or s['log']!=poses[-1][0]:poses.append((s['log'],s['samples'][0]['hardware']))
 for label,hw0 in poses:
  for arm,indices in ARMS.items():
   q0=[hw0['position'][j-1] for j in indices]
   start,angle=kin.forward(q0)
   for axis in (3,4,5):
    for direction in (-1,1):
     expected=target_matrix(start,axis,direction,.2)
     inverse=kin.inverse(expected,angle,q0)
     once=pose_error(expected,kin.forward(inverse)[0])
     hw={k:(v[:] if isinstance(v,list) else v) for k,v in hw0.items()}
     held=hw['position'][:];hw['command_position']=held[:];path=None;drifts=[];error=None
     for tick in range(1000): # 20 s at 50 Hz with 10 Hz IK updates
      q,v=sample(path,PERIOD) if path else (held[:],[0.]*20)
      hw['position']=q;hw['command_position']=q[:]
      try:
       if tick%5==0:
        r=make_request(kin,hw,arm,axis,direction,8,2,4);r.update(id=tick+1,created=tick*PERIOD)
        desired=validate_velocity(hw,r,tick*PERIOD)
       path=trajectory(q,v,desired,indices,held)
      except ValueError as e:error=str(e);break
      actual=kin.forward([q[j-1] for j in indices])[0]
      drifts.append(pose_error(start,actual)[0])
     report['continuous_simulation'].append({'pose_log':label,'arm':arm,'axis':axis,'direction':direction,
       'single_ik_error_mm':once[0],'single_ik_error_deg':once[1],
       'simulated_s':tick*PERIOD,'max_drift_mm':max(drifts,default=0),'stop_reason':error})
 (HERE/'model_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 print('MODEL',json.dumps(report['model_comparison'],ensure_ascii=False))
 for r in report['log_segments']:
  if r['axis']>=3:print('ROTATION',r['log'],r['axis'],r['direction'], 'sec',round(r['duration_s'],2),'deg',round(r['rotation_deg'],3),'origin_mm',round(r['max_origin_displacement_mm'],4),'cmd_mm',round(r['max_command_origin_displacement_mm'],4),'offset_point_mm',round(r['candidate_offset_point_displacement_mm'],3))
 print('SIMULATION_MAX_MM',max(r['max_drift_mm'] for r in report['continuous_simulation']))
 print('SINGLE_IK_MAX_MM',max(r['single_ik_error_mm'] for r in report['continuous_simulation']))
 print('REPORT',HERE/'model_audit.json')
if __name__=='__main__':main()
