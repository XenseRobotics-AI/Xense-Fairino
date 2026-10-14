#!/usr/bin/env python3
"""Offline robust multi-start registration. Output candidates, never robot config."""
import json,math,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
D=np.load(ROOT/'stable-points.npz')

def rot(a):
 a=np.deg2rad(a);c,s=np.cos(a),np.sin(a);return np.array([[c,-s],[s,c]])
def voxel(p,size):
 _,ix=np.unique(np.floor(p/size).astype(int),axis=0,return_index=True);return p[np.sort(ix)]
def nn(a,b):
 d=(a[:,None,0]-b[None,:,0])**2+(a[:,None,1]-b[None,:,1])**2
 i=np.argmin(d,axis=1);return np.sqrt(d[np.arange(len(a)),i]),i

def metrics(a,b):
 da,_=nn(a,b);db,_=nn(b,a);v=np.r_[da,db]
 return {'score':float(np.mean(np.minimum(v,.25)**2)),
         'under_5cm':float(np.mean(v<.05)),'under_10cm':float(np.mean(v<.1)),
         'under_20cm':float(np.mean(v<.2)),
         'overlap_median_m':float(np.median(v[v<.15])) if np.any(v<.15) else 99,
         'overlap_p90_m':float(np.percentile(v[v<.15],90)) if np.any(v<.15) else 99}

def icp(f,b,A,t,steps=40):
 A=A.copy();t=t.copy()
 for k in range(steps):
  p=b@A.T+t;d,ix=nn(p,f);df,ib=nn(f,p)
  cap=.7 if k<8 else .4 if k<18 else .22
  cut=min(cap,max(.09,np.quantile(d,.6)))
  good=d<cut;fgood=df<cut
  src=np.r_[p[good],p[ib[fgood]]];dst=np.r_[f[ix[good]],f[fgood]]
  if len(src)<20:break
  distances=np.linalg.norm(src-dst,axis=1);w=1/np.maximum(1,distances/.08);w/=w.sum()
  cs=(w[:,None]*src).sum(axis=0);ct=(w[:,None]*dst).sum(axis=0)
  U,sv,Vt=np.linalg.svd((src-cs).T@(w[:,None]*(dst-ct)))
  R=Vt.T@U.T
  if np.linalg.det(R)<0:Vt[-1]*=-1;R=Vt.T@U.T
  shift=ct-R@cs
  A=R@A;t=R@t+shift
  if np.linalg.norm(shift)<1e-5 and abs(R[1,0])<1e-5:break
 return A,t

f=voxel(D['front'],.12);b=voxel(D['back'],.12)
results=[];start=time.time()
for parity in [1,-1]:
 for angle in range(0,360,15):
  for y in [-.45,0,.45]:
   A,t=icp(f,b,rot(angle)@np.diag([1,parity]),np.array([-.85,y]))
   m=metrics(f,b@A.T+t)
   baseline=float(np.linalg.norm(t))
   if .3<baseline<1.5:
    results.append({'parity':parity,'theta_deg':float(np.rad2deg(np.arctan2(A[1,0],A[0,0]))),'A':A.tolist(),'t':t.tolist(),'baseline_m':baseline,**m})
 print('FINISHED_PARITY',parity,'seconds',round(time.time()-start,1),flush=True)
results.sort(key=lambda x:x['score'])
unique=[]
for x in results:
 if not any(x['parity']==y['parity'] and np.linalg.norm(np.array(x['t'])-y['t'])<.08 and abs((x['theta_deg']-y['theta_deg']+180)%360-180)<2 for y in unique):unique.append(x)
# Refine the strongest distinct hypotheses on the original 6 cm samples.
f=voxel(D['front'],.05);b=voxel(D['back'],.05)
for x in unique[:10]:
 A,t=icp(f,b,np.array(x['A']),np.array(x['t']),60)
 x.update(A=A.tolist(),t=t.tolist(),theta_deg=float(np.rad2deg(np.arctan2(A[1,0],A[0,0]))),baseline_m=float(np.linalg.norm(t)),**metrics(f,b@A.T+t))
unique[:10]=sorted(unique[:10],key=lambda x:x['score'])
report={'point_counts':[len(f),len(b)],'seconds':time.time()-start,'candidates':unique[:12],'note':'Registration gives relative geometry only; common reflection and absolute chassis orientation are not determined.'}
(ROOT/'relative-fit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False,indent=2))
