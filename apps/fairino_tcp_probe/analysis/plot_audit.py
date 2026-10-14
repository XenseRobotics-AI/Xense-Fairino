import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))
from model_audit import segments,URDF,pose_point,ARMS,distance
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
HERE=Path(__file__).parent
base=next((HERE.parent.parent/'法奥轮式人形').glob('RD53*/left_arm_description/meshes'))
dtype=np.dtype([('normal','<f4',(3,)),('verts','<f4',(3,3)),('attr','<u2')])
fig,axes=plt.subplots(1,3,figsize=(16,4.8))
ax=axes[0]
for name,color in (('wrist1_Link.STL','#5a94be'),('wrist2_Link.STL','#e8a34a')):
 t=np.fromfile(base/name,dtype=dtype,offset=84)['verts']*1000
 ax.add_collection(PolyCollection(t[:,:,[0,2]],facecolors=color,alpha=.13,edgecolors='none'))
ax.scatter([0,-106],[0,0],c=['#2370b0','#cb3d35'],s=50,zorder=4)
ax.annotate('Old controlled point\nDH / URDF wrist origin',xy=(0,0),xytext=(-20,70),arrowprops={'arrowstyle':'->'},ha='center')
ax.annotate('Model flange face center',xy=(-106,0),xytext=(-90,-82),arrowprops={'arrowstyle':'->'},ha='center')
ax.annotate('',xy=(-106,-60),xytext=(0,-60),arrowprops={'arrowstyle':'<->'})
ax.text(-53,-57,'106 mm',ha='center');ax.set_xlim(-150,60);ax.set_ylim(-100,105)
ax.set_aspect('equal');ax.set_xlabel('Wrist X (mm)');ax.set_ylabel('Wrist Z (mm)');ax.set_title('1. Two different physical points')
s=next(s for s in segments() if s['log']=='20261006-170553.jsonl' and s['axis']==5 and len(s['samples'])>150)
u=URDF(s['arm']);points=[];wrists=[];times=[]
for d in s['samples']:
 q=[d['hardware']['position'][j-1] for j in ARMS[s['arm']]];t=u.fk(q)
 points.append(pose_point(t,[-106,0,0]));wrists.append([t[i][3] for i in range(3)]);times.append(d['local_time']-s['samples'][0]['local_time'])
p=np.array(points);w=np.array(wrists)
axes[1].plot(times,[distance(x,w[0]) for x in w],label='Wrist origin',color='#2370b0')
axes[1].plot(times,[distance(x,p[0]) for x in p],label='Model flange point',color='#cb3d35')
axes[1].set_xlabel('Time (s)');axes[1].set_ylabel('Displacement from start (mm)');axes[1].legend();axes[1].grid(alpha=.2)
axes[1].set_title('2. Replay: Rz rotation ~41.6 deg')
axes[2].plot(p[:,0]-p[0,0],p[:,1]-p[0,1],color='#cb3d35',label='Model flange point')
axes[2].plot(w[:,0]-w[0,0],w[:,1]-w[0,1],color='#2370b0',label='Wrist origin')
axes[2].scatter([0],[0],c='black',s=20);axes[2].set_xlabel('Delta base X (mm)');axes[2].set_ylabel('Delta base Y (mm)')
axes[2].set_aspect('equal');axes[2].grid(alpha=.2);axes[2].set_title('3. Reconstructed XY motion')
fig.suptitle('Actual joint feedback + supplied URDF/STL; flange trajectory is reconstructed, not externally measured',fontsize=12)
fig.tight_layout();fig.savefig(HERE/'tcp_audit.png',dpi=160)
print(HERE/'tcp_audit.png')
