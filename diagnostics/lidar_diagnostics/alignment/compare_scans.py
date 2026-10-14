#!/usr/bin/env python3
import gzip,json,math,sys,warnings
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
ROOT=Path(__file__).resolve().parent
INPUT=Path(sys.argv[1]) if len(sys.argv)>1 else sorted(ROOT.glob('scans-*.json.gz'))[-1]
D=json.loads(gzip.open(INPUT,'rt').read())
FONT=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':FONT.get_name(),'axes.unicode_minus':False})

def rot(deg):
 a=np.deg2rad(deg);c,s=np.cos(a),np.sin(a)
 return np.array([[c,-s],[s,c]])
def voxel(p,size=.06):
 _,i=np.unique(np.floor(p/size).astype(int),axis=0,return_index=True)
 return p[np.sort(i)]
def nearest(a,b):
 d=(a[:,None,0]-b[None,:,0])**2+(a[:,None,1]-b[None,:,1])**2
 i=np.argmin(d,axis=1)
 return np.sqrt(d[np.arange(len(a)),i]),i

def read_points(side):
 scans=D['scans'][side]
 arr=np.array([[np.nan if x is None else x for x in s['ranges']] for s in scans])
 arr[(arr<.25)|(arr>=15)]=np.nan
 with warnings.catch_warnings():
  warnings.simplefilter('ignore')
  median=np.nanmedian(arr,axis=0)
  q10,q90=np.nanpercentile(arr,[10,90],axis=0)
 stable=np.isfinite(median)&(np.sum(np.isfinite(arr),axis=0)>=len(arr)*.8)&((q90-q10)<.06)
 angles=scans[0]['angle_min']+np.arange(len(median))*scans[0]['angle_increment']
 p=np.c_[np.cos(angles[stable])*median[stable],np.sin(angles[stable])*median[stable]]
 stats={'scans':len(scans),'beams_per_scan':arr.shape[1],'stable_beams':int(stable.sum()),'median_temporal_spread_m':float(np.nanmedian((q90-q10)[stable])),'points_after_voxel':len(voxel(p)),'range_quantiles':np.nanpercentile(median,[0,25,50,75,100]).tolist(),'capture_seconds':scans[-1]['received']-scans[0]['received']}
 return p,voxel(p),stats

P={};V={};stats={}
for side in ['front','back']:P[side],V[side],stats[side]=read_points(side)
np.savez(ROOT/'stable-points.npz',front=P['front'],back=P['back'],front_voxel=V['front'],back_voxel=V['back'])
CANDIDATES=[
 {'name':'URDF：roll 180°，前 46.15° / 后 225°','fxy':[.359225,.259325],'bxy':[-.359225,-.259325],'fyaw':math.degrees(.805398),'byaw':math.degrees(3.926991),'fs':-1,'bs':-1},
 {'name':'URDF 位置和 yaw：改为 roll 0°','fxy':[.359225,.259325],'bxy':[-.359225,-.259325],'fyaw':math.degrees(.805398),'byaw':math.degrees(3.926991),'fs':1,'bs':1},
 {'name':'Xacro：roll 0°，前 −45° / 后 90°','fxy':[-.259325,.359225],'bxy':[.259325,-.359225],'fyaw':math.degrees(-.7853981),'byaw':90,'fs':1,'bs':1},
 {'name':'目测示意：roll 180°，前 45° / 后 225°','fxy':[.359225,.259325],'bxy':[-.359225,-.259325],'fyaw':45,'byaw':225,'fs':-1,'bs':-1},
]

def transform(p,xy,yaw,sign):return (p*np.array([1,sign]))@rot(yaw).T+xy

def score(a,b):
 da,_=nearest(a,b);db,_=nearest(b,a)
 return {'front_under_10cm':float(np.mean(da<.1)),'back_under_10cm':float(np.mean(db<.1)),'front_under_20cm':float(np.mean(da<.2)),'back_under_20cm':float(np.mean(db<.2)),'symmetric_mean_clipped_50cm':float((np.mean(np.minimum(da,.5))+np.mean(np.minimum(db,.5)))/2),'smallest_30pct_median':float(np.median(np.sort(np.r_[da,db])[:max(1,int(.3*(len(da)+len(db))))]))}

fig,axs=plt.subplots(2,2,figsize=(13,12))
res=[]
for ax,c in zip(axs.ravel(),CANDIDATES):
 f=transform(P['front'],c['fxy'],c['fyaw'],c['fs']);b=transform(P['back'],c['bxy'],c['byaw'],c['bs'])
 fv=transform(V['front'],c['fxy'],c['fyaw'],c['fs']);bv=transform(V['back'],c['bxy'],c['byaw'],c['bs']);metrics=score(fv,bv)
 res.append(dict(c,metrics=metrics))
 ax.scatter(f[:,0],f[:,1],s=2,c='#0079b8',label='前 .101');ax.scatter(b[:,0],b[:,1],s=2,c='#e76726',label='后 .100')
 for side,color in [('f','#0079b8'),('b','#e76726')]:
  xy=c[side+'xy'];a=math.radians(c[side+'yaw']);ax.scatter(*xy,s=20,c=color);ax.arrow(*xy,.35*math.cos(a),.35*math.sin(a),width=.025,color=color)
 ax.set_aspect('equal');ax.set_xlim(-7,7);ax.set_ylim(-7,7);ax.grid(alpha=.2);ax.legend(loc='upper right');ax.set_xlabel('车身 X / 米');ax.set_ylabel('车身 Y / 米');ax.set_title(c['name'],fontsize=12)
 ax.text(.02,.02,'稳定点距离 < 10 cm：前 %.0f%% / 后 %.0f%%'%(100*metrics['front_under_10cm'],100*metrics['back_under_10cm']),transform=ax.transAxes,fontsize=10,bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))
fig.suptitle('双雷达离线拼接比较（蓝：前雷达；橙：后雷达）\n过滤 15 m 以外、近身和变化明显的点；模型位置尚未实测确认',fontsize=15)
fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(ROOT/'model-comparison.png',dpi=150);plt.close(fig)
report={'input':str(INPUT),'stats':stats,'candidates':res,'odom_samples':len(D['odom']),'cmd_vel_samples':len(D['cmd_vel'])}
(ROOT/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False,indent=2))
