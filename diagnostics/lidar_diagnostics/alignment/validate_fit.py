#!/usr/bin/env python3
import gzip,json,math,warnings
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
ROOT=Path(__file__).resolve().parent
D=json.loads(gzip.open(sorted(ROOT.glob('scans-*.json.gz'))[-1],'rt').read())
fit=json.loads((ROOT/'relative-fit.json').read_text())['candidates'][0]
# Reuse function definitions only; do not rerun the global search.
code=(ROOT/'fit_relative.py').read_text().split("f=voxel(D['front'],.12)")[0]
ns={'__file__':str(ROOT/'fit_relative.py')};exec(code,ns)
rot,voxel,nn,icp,metrics=[ns[k] for k in ['rot','voxel','nn','icp','metrics']]

def points(scans):
 arr=np.array([[np.nan if r is None else r for r in s['ranges']] for s in scans])
 arr[(arr<.25)|(arr>=15)]=np.nan
 with warnings.catch_warnings():
  warnings.simplefilter('ignore');m=np.nanmedian(arr,axis=0);lo,hi=np.nanpercentile(arr,[10,90],axis=0)
 good=np.isfinite(m)&((hi-lo)<.06)&(np.sum(np.isfinite(arr),axis=0)>=len(arr)*.8)
 a=scans[0]['angle_min']+np.arange(len(m))*scans[0]['angle_increment']
 return voxel(np.c_[m[good]*np.cos(a[good]),m[good]*np.sin(a[good])],.05)

def refine(f,b,A,t):
 dist=(f[:,None,0]-f[None,:,0])**2+(f[:,None,1]-f[None,:,1])**2
 ix=np.argsort(dist,axis=1)[:,:7];neighbors=f[ix];center=neighbors.mean(axis=1);d=neighbors-center[:,None,:]
 cov=np.einsum('nki,nkj->nij',d,d)/7
 ev,evec=np.linalg.eigh(cov);normals=evec[:,:,0]
 valid=(ev[:,0]/np.maximum(ev[:,1],1e-10)<.08)&(ev[:,1]>.0002)&(np.sqrt(np.max(np.take_along_axis(dist,ix,axis=1),axis=1))<.3)
 condition=0;num=0
 for _ in range(30):
  p=b@A.T+t;dd,idx=nn(p,f);keep=(dd<.15)&valid[idx]
  pp=p[keep];q=f[idx[keep]];norm=normals[idx[keep]]
  residual=np.sum(norm*(pp-q),axis=1);w=1/np.maximum(1,np.abs(residual)/.025)
  J=np.c_[-pp[:,1]*norm[:,0]+pp[:,0]*norm[:,1],norm]
  condition=float(np.linalg.cond(J.T@J));num=len(J)
  if num<30 or condition>1e6:break
  delta=np.linalg.lstsq(J*np.sqrt(w[:,None]),-residual*np.sqrt(w),rcond=None)[0]
  delta*=min(1,.03/max(1e-10,np.linalg.norm(delta[1:])),math.radians(1)/max(1e-10,abs(delta[0])))
  R=rot(math.degrees(delta[0]));A=R@A;t=R@t+delta[1:]
  if np.linalg.norm(delta)<1e-6:break
 return A,t,{'matched_line_points':num,'normal_matrix_condition':condition}

def out(A,t,f,b):return {'theta_deg':float(np.rad2deg(np.arctan2(A[1,0],A[0,0]))),'A':A.tolist(),'t':t.tolist(),'baseline_m':float(np.linalg.norm(t)),**metrics(f,b@A.T+t)}
A0=np.array(fit['A']);t0=np.array(fit['t']);f=points(D['scans']['front']);b=points(D['scans']['back'])
A,t,diag=refine(f,b,A0,t0);main=out(A,t,f,b);main['refinement']=diag
segments=[]
for part in range(3):
 sample={k:v[len(v)*part//3:len(v)*(part+1)//3] for k,v in D['scans'].items()}
 ff,bb=points(sample['front']),points(sample['back']);aa,tt=icp(ff,bb,A,t,40);aa,tt,dd=refine(ff,bb,aa,tt)
 segments.append(out(aa,tt,ff,bb))
# Fit earlier half, evaluate later half without changing its transform.
train={k:points(v[:len(v)//2]) for k,v in D['scans'].items()};test={k:points(v[len(v)//2:]) for k,v in D['scans'].items()}
aa,tt=icp(train['front'],train['back'],A,t,50);aa,tt,_=refine(train['front'],train['back'],aa,tt)
heldout=metrics(test['front'],test['back']@aa.T+tt)
S=np.diag([1.,-1.]);F=rot(math.degrees(.805398))@S;B=rot(math.degrees(3.926991))@S
oldA=F.T@B;oldt=F.T@(np.array([-.359225,-.259325])-np.array([.359225,.259325]))
old=metrics(f,b@oldA.T+oldt)
report={'relative_fit':main,'three_time_segments':segments,'fit_first_half_evaluate_second_half':heldout,'urdf_baseline_m':float(np.linalg.norm(oldt)),'urdf_metrics_same_points':old,'absolute_chassis_pose_verified':False,'common_roll_flip_identifiable':False}
(ROOT/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False})
fig,axs=plt.subplots(1,3,figsize=(17,6))
oldb=b@oldA.T+oldt;newb=b@A.T+t
for ax,title,bb,trans in [(axs[0],'套用现有 URDF 参数',oldb,oldt),(axs[1],'只按实测数据拟合相对位置',newb,t),(axs[2],'共同轮廓局部放大',newb,t)]:
 ax.scatter(f[:,0],f[:,1],s=5,c='#087fb3',label='前雷达 .101');ax.scatter(bb[:,0],bb[:,1],s=5,c='#e97525',label='后雷达 .100')
 ax.scatter(0,0,s=55,marker='^',c='#087fb3',edgecolor='white',zorder=4);ax.scatter(*trans,s=55,marker='^',c='#e97525',edgecolor='white',zorder=4)
 ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='upper right');ax.set_title(title,fontsize=13);ax.set_xlabel('前雷达坐标 X / 米');ax.set_ylabel('前雷达坐标 Y / 米')
for ax in axs[:2]:ax.set_xlim(-7,8);ax.set_ylim(-7,8)
# Choose a well-observed section containing many matched points away from the sensors.
dist,_=nn(f,newb);common=f[(dist<.05)&(np.linalg.norm(f,axis=1)>1)]
grid=np.floor(common/1.5).astype(int);keys,cnt=np.unique(grid,axis=0,return_counts=True);cell=keys[np.argmax(cnt)];ctr=(cell+.5)*1.5
axs[2].set_xlim(ctr[0]-1.3,ctr[0]+1.3);axs[2].set_ylim(ctr[1]-1.3,ctr[1]+1.3)
for ax in axs[:2]:
 from matplotlib.patches import Rectangle
 ax.add_patch(Rectangle(ctr-1.3,2.6,2.6,fill=False,ls='--',color='#555',lw=1))
axs[0].text(.03,.025,'相距 10 cm 内的点：%.0f%%'%(old['under_10cm']*100),transform=axs[0].transAxes,bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))
axs[1].text(.03,.025,'相距 10 cm 内的点：%.0f%%'%(main['under_10cm']*100),transform=axs[1].transAxes,bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))
fig.suptitle('两台雷达的共同轮廓可以对齐；现有模型的安装位置不吻合\n全部在前雷达坐标系中比较，未标定车身坐标；两台视野不同，不要求所有点重合',fontsize=15)
fig.tight_layout(rect=[0,0,1,.9]);fig.savefig(ROOT/'relative-alignment.png',dpi=160);plt.close(fig)
print(json.dumps(report,ensure_ascii=False,indent=2))
