#!/usr/bin/env python3
"""Reproducible OFFLINE wrist/IK experiment. Never imports a robot transport.

Uses the real vendor mathematics library, current production vr_plan.entry,
and an isolated experimental arm-angle selector. Does NOT change the runtime.
All rate limits and plant parameters below are experimental assumptions.
"""
import argparse
from collections import Counter, deque
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts/pico4/fairino')]
import paths  # noqa: E402
import numpy as np
from tcp_core import (BOUNDS_DEG, check_arm_limits, multiply, pose_error,
                      rotation, tcp_forward_matrix)
from tool_frames import tool_config, for_arm
from vendor_kinematics import VendorKinematics, LIBRARY
from vr_plan import entry, interpolate_tcp
from wrist_guard import WristGuard, WristBlocked, BOUNDARY, CONFIG_ID

SEED = 20261026
GUARD = WristGuard()
MIRROR = np.array([-1, 1, -1, 1, -1, 1, -1])
HOME_FILES = ('home_pose.json', 'home_pose_right_mirrored.json',
              'home_pose_left_cross_body.json')
OFFSETS = sorted({0., *[sign*x for x in (.005, .01, .025, .05, .1, .25, .5,
                                       .75, 1., 1.5, 2., 3., 4., 5., 7.5, 10., 15.)
                       for sign in (-1, 1)]}, key=abs)


def cap(x):
    if x < 0 or x > 70:
        return -math.inf
    return next(y for a, y in BOUNDARY if a >= x)


def make_segments(r6=0., r7=0.):
    """True step-boundary segments in the positive quadrant, in degrees."""
    shift = 2+r6
    points = [(0., cap(shift)-2-r7)]
    for i, (x, y) in enumerate(BOUNDARY):
        if x < shift:
            continue
        points.append((x-shift, y-2-r7))
        if i+1 < len(BOUNDARY):
            points.append((x-shift, BOUNDARY[i+1][1]-2-r7))
    points.append((70-shift, 0.))
    points = np.asarray(points)
    return points[:-1], points[1:]


SEGMENTS = make_segments()


def distance(q, reserve=(0., 0.)):
    """Signed Euclidean angular distance to the joint envelope, not mm."""
    xy = np.abs(np.degrees(q[-2:]))
    a, b = SEGMENTS if reserve == (0., 0.) else make_segments(*reserve)
    delta = b-a
    lengths = (delta*delta).sum(axis=1)
    t = np.clip(((xy-a)*delta).sum(axis=1)/np.maximum(lengths, 1e-30), 0., 1.)
    result = float(np.linalg.norm(xy-a-t[:, None]*delta, axis=1).min())
    slack = min(68-reserve[0]-xy[0], cap(xy[0]+2+reserve[0])-2-reserve[1]-xy[1])
    return result if slack >= -1e-9 else -result


def hw(q, side, measured=None, velocity=None):
    out = dict(position=[0.]*20, command_position=[0.]*20, velocity=[0.]*20)
    start = 0 if side == 'left' else 7
    out['position'][start:start+7] = np.asarray(q if measured is None else measured).tolist()
    out['command_position'][start:start+7] = np.asarray(q).tolist()
    out['velocity'][start:start+7] = np.asarray([0.]*7 if velocity is None else velocity).tolist()
    return out


def audit(kin, side, origin, q, tcp, velocity=None, actual=None, dense=17):
    q=np.asarray(q).tolist();origin=np.asarray(origin).tolist()
    velocity=None if velocity is None else np.asarray(velocity).tolist()
    actual=None if actual is None else np.asarray(actual).tolist()
    check_arm_limits(q)
    independent = multiply(tcp_forward_matrix(q), for_arm(kin.tools, side))
    mm, deg = pose_error(tcp, independent)
    if mm > .02 or deg > .01:
        raise AssertionError(('independent_fk', mm, deg))
    GUARD.arm_segment(list(origin), list(q), None if velocity is None else list(velocity))
    if actual is not None:
        GUARD.arm_segment(list(actual), list(q), list(velocity))
    # Independent dense check of the un-eroded mesh envelope, including retreat.
    for start in (origin,) if actual is None else (origin, actual):
        for u in np.linspace(0., 1., dense):
            x, y = np.abs(np.degrees(np.asarray(start[-2:])*(1-u)+np.asarray(q[-2:])*u))
            if x > 70+1e-8 or y > cap(float(min(x, 70)))+1e-8:
                raise AssertionError(('raw_envelope_crossing', float(x), float(y)))
    return mm, deg


def error_kind(exc):
    text = str(exc)
    if isinstance(exc, WristBlocked):
        return 'wrist_blocked'
    if '模型限制' in text:
        return 'joint_limit'
    if '返回码' in text:
        return 'vendor_ik_failure'
    if '不连续' in text:
        return 'roundtrip_discontinuity'
    return 'other_value_error'


def stats(values):
    a = np.array(values, dtype=float)
    return {} if not len(a) else dict(n=len(a), mean=float(a.mean()),
        p50=float(np.percentile(a, 50)), p95=float(np.percentile(a, 95)),
        p99=float(np.percentile(a, 99)), maximum=float(a.max()))


def states(side, quick=False):
    for file in HOME_FILES:
        data = json.loads((ROOT/'scripts/pico4/fairino'/file).read_text())
        yield file, np.radians(data['joint_degrees'][side]), 'home'
    rng = random.Random(SEED)
    count = 6 if quick else 24
    for i in range(count):
        q6 = rng.uniform(-66, 66)
        q7 = rng.uniform(-1, 1)*max(0., cap(abs(q6)+2)-2-3)
        q = np.array([rng.uniform(-145, 145), rng.uniform(-95, 95),
            rng.uniform(-150, 150), rng.uniform(15, 105), rng.uniform(-150, 150), q6, q7])
        if side == 'right':
            q *= MIRROR
        yield 'random_'+str(i), np.radians(q), 'random'
    for i in range(8 if quick else 40):
        q6 = (0., 3.6, 9.9, 30., 51., 52., 60., 67.9)[i % 8]
        slack = (0., .001, .05, .5, 2.5)[i//8]
        s6, s7 = (-1 if i%3 == 0 else 1), (-1 if i%4 < 2 else 1)
        q = np.array([45., 75., -100., 85., (-135., -85., -30., 30., 85., 135.)[i%6],
                      s6*q6, s7*(cap(q6+2)-2-slack)])
        if side == 'right':
            q *= MIRROR
        yield 'boundary_'+str(i), np.radians(q), 'boundary'
    for i, (joint, value) in enumerate(((0,174.9),(0,-174.9),(1,109.9),(1,-109.9),
                                       (2,174.9),(4,174.9),(3,-74.9),(3,109.9),
                                       (3,0.),(3,.0001),(3,.01),(3,1.))):
        q = np.array([45.,75.,-100.,85.,85.,0.,0.]);q[joint] = value
        if side == 'right':
            q *= MIRROR
        yield 'limit_singular_'+str(i), np.radians(q), 'limit_singular'
    q = np.array([45.,75.,-100.,85.,135.,51.,37.7])
    if side == 'right':
        q *= MIRROR
    yield 'known_boundary', np.radians(q), 'known_boundary'


def perturb(base, axis, amount):
    out = np.array(base).copy()
    if axis < 3:
        out[axis, 3] += amount
    else:
        out[:3, :3] = np.array(rotation(axis-3, math.radians(amount)))[:3, :3] @ out[:3, :3]
    return out


def goals(base, quick=False):
    yield 'hold', np.array(base)
    for magnitude in ('small', 'large'):
        if quick and magnitude == 'large':
            continue
        for axis in range(6):
            step = (.2 if magnitude=='small' else 2.) if axis<3 else (.05 if magnitude=='small' else .5)
            for sign in (-1, 1):
                yield f'{magnitude}_{axis}_{sign}', perturb(base, axis, step*sign)
    for sign in (-1, 1):
        t = np.array(base).copy();t[:3, 3] += sign*np.array([1.,-.7,.4])
        for axis in range(3,6):
            t = perturb(t, axis, sign*.15)
        yield 'mixed_'+str(sign), t


def run_matrix(kins, output, quick):
    counts = Counter();groups = {}; errors = Counter(); preflight = []
    timings = {'baseline_ms':[], 'sampled_search_ms':[]}; worst = []; samples = []
    total = 0; audited = 0; max_errors = [0.,0.]
    with (output/'matrix.jsonl').open('w') as log:
        for mode, kin in kins.items():
            for side in ('left','right'):
                for name, qarray, group in states(side, quick):
                    q = qarray.tolist()
                    try:
                        check_arm_limits(q);GUARD.arm_segment(q,q,[0.]*7)
                        checked = kin.verify_tcp(q,side)
                        base, phi = checked['matrix'], checked['arm_angle']
                    except ValueError as exc:
                        preflight.append(dict(mode=mode,side=side,name=name,reason=error_kind(exc),detail=str(exc)))
                        continue
                    group_counts = groups.setdefault(mode+'/'+side+'/'+group, Counter())
                    for motion, goal in goals(base, quick):
                        total += 1
                        row = dict(id=total, mode=mode,side=side,state=name,group=group,motion=motion,
                                   q_deg=np.degrees(q).tolist(),goal=np.asarray(goal).tolist(),phi=phi)
                        begin = time.perf_counter()
                        try:
                            plan = entry(kin,hw(q,side),side,goal,reference=q)
                            elapsed = 1000*(time.perf_counter()-begin)
                            outq = plan['target']; scale = plan['wrist_scale']
                            mm,deg=audit(kin,side,q,outq,np.array(plan['tcp']).reshape(4,4),[0.]*7)
                            audited += 1;max_errors=[max(max_errors[0],mm),max(max_errors[1],deg)]
                            outcome = 'full' if scale>=1.-1e-9 or motion=='hold' else 'partial' if scale>1e-9 else 'hold'
                            row.update(baseline=outcome,baseline_scale=scale,
                                baseline_max_joint_step_deg=float(np.max(np.abs(np.degrees(np.array(outq)-q)))),
                                baseline_error_to_requested=pose_error(goal,kin.forward_tcp(outq,side)[0]))
                        except ValueError as exc:
                            elapsed = 1000*(time.perf_counter()-begin)
                            outcome='error'; row.update(baseline=outcome,baseline_reason=error_kind(exc),baseline_detail=str(exc))
                            errors[error_kind(exc)]+=1
                        timings['baseline_ms'].append(elapsed)
                        counts['baseline_'+outcome]+=1;group_counts['baseline_'+outcome]+=1
                        if outcome != 'full' and motion != 'hold':
                            begin=time.perf_counter(); candidates=[]; rejects=Counter()
                            for offset in OFFSETS:
                                try:
                                    alt=kin.inverse_tcp(np.asarray(goal).tolist(),phi+offset,q,side)
                                    jump=float(np.max(np.abs(np.degrees(np.array(alt)-q))))
                                    if jump>5.:
                                        rejects['over_5deg_joint_change']+=1;continue
                                    GUARD.arm_segment(q,alt,[0.]*7)
                                    candidates.append((offset,alt,jump,distance(alt)))
                                except ValueError as exc:
                                    rejects[error_kind(exc)]+=1
                            timings['sampled_search_ms'].append(1000*(time.perf_counter()-begin))
                            row['search_rejections']=dict(rejects)
                            if candidates:
                                best=min(candidates,key=lambda c:(c[2],abs(c[0])))
                                mm,deg=audit(kin,side,q,best[1],goal,[0.]*7,dense=101)
                                audited+=1;max_errors=[max(max_errors[0],mm),max(max_errors[1],deg)]
                                kind='same_phi_feasible' if any(c[0]==0 for c in candidates) else 'changed_phi_feasible'
                                counts[kind]+=1;group_counts[kind]+=1
                                if outcome in ('hold','error'):
                                    counts[outcome+'_'+kind]+=1
                                row.update(search=kind,alternative_phi_delta=best[0],alternative_q_deg=np.degrees(best[1]).tolist(),
                                           alternative_max_joint_step_deg=best[2],alternative_geometric_margin_deg=best[3],
                                           minimum_time_at_30deg_s=best[2]/30.,
                                           fits_90hz_30deg_s=best[2]<=30/90,
                                           fits_450hz_30deg_s=best[2]<=30/450)
                                if kind=='changed_phi_feasible':
                                    counts['changed_phi_fits_90hz_30deg_s']+=int(best[2]<=30/90)
                                    counts['changed_phi_fits_450hz_30deg_s']+=int(best[2]<=30/450)
                                if kind=='changed_phi_feasible' and len(samples)<20:
                                    samples.append(row.copy())
                            else:
                                row['search']='not_found_in_sampled_neighborhood'
                                counts[row['search']]+=1;group_counts[row['search']]+=1
                                if len(worst)<20:worst.append(row.copy())
                        log.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
                    print(f'matrix {mode}/{side}/{name}: {total} targets',flush=True)
    return dict(total=total,counts=dict(counts),groups={k:dict(v) for k,v in groups.items()},
                preflight_rejections=preflight,errors=dict(errors),timing={k:stats(v) for k,v in timings.items()},
                audited_outputs=audited,max_independent_fk_error_mm_deg=max_errors,
                search_offsets_deg=OFFSETS,search_max_joint_displacement_deg=5.,
                success_examples=samples,failure_examples=worst)


class ExperimentalPlanner:
    """Research selector, deliberately never imported by any runtime module."""
    def __init__(self, kin, side, hz=450., weight=1., acceleration=True, fast=True):
        self.kin,self.side,self.dt=kin,side,1/hz
        self.weight,self.acceleration,self.fast=weight,acceleration,fast
        self.vmax,self.amax=30.,120.  # deg/s, deg/s^2: EXPERIMENT assumptions
        self.psimax,self.psia=5.,20.  # arm-angle speed / acceleration assumptions
        self.previous_velocity=np.zeros(7);self.previous_psi_rate=0.
        self.calls=0;self.last_reason={}

    def choose(self, reference, actual, velocity, goal):
        q=np.asarray(reference);actual=np.asarray(actual);velocity=np.asarray(velocity)
        origin,phi=self.kin.forward_tcp(q.tolist(),self.side)
        dt=self.dt; rejects=Counter();self.calls=0
        reserve=tuple(np.abs(np.degrees(velocity[-2:]))*.2)
        d0=distance(q,reserve)
        penalty=lambda d:max(0.,1-d/5.)**2
        if self.acceleration:
            rates=[0.,self.previous_psi_rate-self.psia*dt,self.previous_psi_rate,
                   self.previous_psi_rate+self.psia*dt]
        else:
            rates=[0.,-self.psimax,self.psimax]
        offsets=list(dict.fromkeys(float(np.clip(v,-self.psimax,self.psimax)*dt) for v in rates))
        for fraction in (1.,.5,.25,.125,.0625,.03125,0.):
            tcp=np.asarray(goal) if fraction==1 else np.asarray(origin) if fraction==0 else interpolate_tcp(origin,goal,fraction)
            candidates=[]
            for offset in offsets:
                try:
                    if fraction==0 and offset==0:
                        result=q.copy()
                    else:
                        self.calls+=1
                        result=np.asarray(self.kin.inverse_tcp(tcp.tolist(),phi+offset,q.tolist(),self.side))
                    v=np.degrees(result-q)/dt
                    acceleration=(v-self.previous_velocity)/dt
                    if np.max(np.abs(v))>self.vmax+1e-7:
                        rejects['velocity_limit']+=1;continue
                    if self.acceleration and np.max(np.abs(acceleration))>self.amax+1e-5:
                        rejects['acceleration_limit']+=1;continue
                    GUARD.arm_segment(q.tolist(),result.tolist(),velocity.tolist())
                    GUARD.arm_segment(actual.tolist(),result.tolist(),velocity.tolist())
                    d=distance(result,reserve)
                    cost=float(np.sum((v/self.vmax)**2)+.1*np.sum(((v-self.previous_velocity)/self.vmax)**2)
                               +self.weight*(penalty(d)-penalty(d0))/dt)
                    candidate=(cost,result,tcp,fraction,offset,v,d)
                    if self.fast and fraction==1 and offset==0 and min(d0,d)>=5:
                        candidates=[candidate];break
                    candidates.append(candidate)
                except ValueError as exc:
                    rejects[error_kind(exc)]+=1
            if candidates:
                _,result,tcp,fraction,offset,v,d=min(candidates,key=lambda c:c[0])
                self.previous_velocity=v;self.previous_psi_rate=offset/dt
                self.last_reason=dict(rejects)
                return dict(target=result,tcp=tcp,fraction=fraction,psi_delta=offset,
                            distance=d,calls=self.calls)
        self.last_reason=dict(rejects)
        raise WristBlocked('experimental_no_candidate:'+json.dumps(self.last_reason,sort_keys=True))


def named_pose(name, side):
    if name=='known_boundary':
        q=np.array([45.,75.,-100.,85.,135.,51.,37.7])
        return np.radians(q if side=='left' else q*MIRROR)
    if name=='near_boundary':
        q=np.array([45.,75.,-100.,85.,135.,51.,37.2])
        return np.radians(q if side=='left' else q*MIRROR)
    file=HOME_FILES[int(name[-1])-1]
    return np.radians(json.loads((ROOT/'scripts/pico4/fairino'/file).read_text())['joint_degrees'][side])


def hand_wave(t,duration,kind):
    phase=.5-.5*math.cos(2*math.pi*t/duration)
    value=np.zeros(6)
    if kind=='translate':value[:3]=phase*np.array([3.,-2.,1.])
    elif kind=='rotate':value[3:]=phase*np.array([1.,-.7,.5])
    elif kind=='mixed':value[:]=phase*np.array([2.,-1.,.5,.7,-.4,.3])
    return value


def run_episode(kin,mode,side,pose,kind,hz,profile,strategy,weight=1.,duration=2.):
    dt=1/hz;steps=round(duration*hz)
    q=named_pose(pose,side);measured=q.copy();actual_v=np.zeros(7)
    feedback_q=q.copy();feedback_v=actual_v.copy()
    history=deque([(0.,q.copy(),actual_v.copy())]);last_feedback_slot=-1
    tau,delay={'ideal':(0.,0.),'lag30_delay20':(.03,.02),'lag60_delay40':(.06,.04)}[profile]
    last_tcp=np.asarray(kin.forward_tcp(q.tolist(),side)[0]);initial_tcp=last_tcp.copy()
    planner=ExperimentalPlanner(kin,side,hz,weight,acceleration=strategy!='search_v')
    times=[];calls=[];trace=[];fractions=[];qvelprev=np.zeros(7);maxspeed=0.;maxacc=0.;maxerror=[0.,0.]
    minclear=distance(q);maxtracking=0.;status='completed';reason='';bad=None;task_input=0.;task_output=0.
    expected_input=2*float(np.linalg.norm(hand_wave(duration/2,duration,kind)))
    for i in range(1,steps+1):
        t=i*dt
        feedback_slot=math.floor((t+1e-12)*100.)
        if feedback_slot>last_feedback_slot:
            while len(history)>1 and history[1][0]<=t-delay:
                history.popleft()
            feedback_q=history[0][1].copy();feedback_v=history[0][2].copy();last_feedback_slot=feedback_slot
        previous_hand=hand_wave((i-1)*dt,duration,kind);hand=hand_wave(t,duration,kind)
        inc=hand-previous_hand
        goal=last_tcp.copy();goal[:3,3]+=inc[:3]
        for axis in range(3,6):goal=perturb(goal,axis,inc[axis])
        before=time.perf_counter()
        try:
            if strategy=='baseline':
                p=entry(kin,hw(q,side,feedback_q,feedback_v),side,goal,reference=q.tolist())
                result=dict(target=np.asarray(p['target']),tcp=np.array(p['tcp']).reshape(4,4),fraction=p['wrist_scale'],psi_delta=0.,calls=None)
            else:
                result=planner.choose(q,feedback_q,feedback_v,goal)
        except ValueError as exc:
            times.append((time.perf_counter()-before)*1000)
            status='planner_rejected';reason=str(exc);bad=dict(step=i,q_deg=np.degrees(q).tolist(),actual_deg=np.degrees(measured).tolist(),velocity_deg_s=np.degrees(feedback_v).tolist());break
        times.append((time.perf_counter()-before)*1000)
        target=result['target'];calls.append(result['calls']);fraction=result['fraction']
        try:
            # Fresh receiver feedback can disagree with a delayed client snapshot.
            mm,deg=audit(kin,side,q,target,result['tcp'],actual_v,measured,dense=5)
        except WristBlocked as exc:
            status='receiver_rejected';reason=str(exc);bad=dict(step=i,q_deg=np.degrees(q).tolist(),actual_deg=np.degrees(measured).tolist(),velocity_deg_s=np.degrees(actual_v).tolist());break
        maxerror=[max(maxerror[0],mm),max(maxerror[1],deg)]
        jointv=np.degrees(target-q)/dt;acc=(jointv-qvelprev)/dt
        maxspeed=max(maxspeed,float(np.max(np.abs(jointv))));maxacc=max(maxacc,float(np.max(np.abs(acc))));qvelprev=jointv
        before_actual=measured.copy()
        measured=target.copy() if tau==0 else measured+(1-math.exp(-dt/tau))*(target-measured)
        actual_v=(measured-before_actual)/dt
        minclear=min(minclear,distance(measured));maxtracking=max(maxtracking,float(np.max(np.abs(np.degrees(target-measured)))))
        history.append((t,measured.copy(),actual_v.copy()))
        fractions.append(fraction);mag=float(np.linalg.norm(inc));task_input+=mag;task_output+=fraction*mag
        q=target;last_tcp=result['tcp']
        if i%max(1,round(hz/50))==0 or i==1:
            trace.append(dict(t=t,q_deg=np.degrees(q).tolist(),actual_deg=np.degrees(measured).tolist(),
                              fraction=fraction,margin=distance(measured),psi_delta=result['psi_delta']))
    return dict(mode=mode,side=side,pose=pose,kind=kind,hz=hz,profile=profile,strategy=strategy,weight=weight,
        status=status,reason=reason,completed_steps=len(fractions),requested_steps=steps,
        full_fraction_steps=sum(f>=1-1e-9 for f in fractions),zero_fraction_steps=sum(f<=1e-9 for f in fractions),
        task_progress_ratio=task_output/expected_input if expected_input else None,
        completed_input_progress_ratio=task_output/task_input if task_input else None,
        max_command_speed_deg_s=maxspeed,max_command_acceleration_deg_s2=maxacc,
        min_actual_geometric_margin_deg=minclear,max_tracking_error_deg=maxtracking,
        max_independent_fk_error_mm_deg=maxerror,final_distance_deg=distance(q),
        final_tcp_error_to_initial_mm_deg=pose_error(initial_tcp,last_tcp),
        elapsed_planner_ms=stats(times),ik_calls=stats([c for c in calls if c is not None]),
        bad=bad,trace=trace)


def run_continuous(kins,output,quick):
    rows=[]
    specs=[]
    for mode in kins:
        for side in ('left','right'):
            for pose in ('home1','home2','home3','near_boundary','known_boundary'):
                for kind in ('mixed',) if quick else ('translate','rotate','mixed'):
                    for hz,profile in ((90.,'ideal'),(450.,'lag30_delay20')):
                        for strategy in ('baseline','search_va'):
                            specs.append((mode,side,pose,kind,hz,profile,strategy,1.))
    # Isolate the effect of a velocity cap, acceleration cap, feedback lag, and penalty.
    for weight in (0.,.1,1.,10.):
        for strategy in ('search_v','search_va'):
            for profile in ('ideal','lag60_delay40'):
                specs.append(('flange','left','near_boundary','hold',450.,profile,strategy,weight))
    with (output/'continuous.jsonl').open('w') as f:
        for n,spec in enumerate(specs):
            mode,side,pose,kind,hz,profile,strategy,weight=spec
            row=run_episode(kins[mode],mode,side,pose,kind,hz,profile,strategy,weight,duration=1. if quick else 2.)
            f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');f.flush()
            rows.append({k:v for k,v in row.items() if k!='trace'})
            if n%8==0:print(f'continuous {n+1}/{len(specs)}: {row["status"]}',flush=True)
    counts=Counter(r['strategy']+'/'+r['status'] for r in rows)
    return dict(episodes=len(rows),counts=dict(counts),rows=rows,
        assumptions=dict(joint_velocity_deg_s=30.,joint_acceleration_deg_s2=120.,arm_angle_velocity_deg_s=5.,
            arm_angle_acceleration_deg_s2=20.,feedback_hz=100.,plant='ideal or first-order joint position lag; delayed feedback; fresh receiver wrist check',
            score='sum((qdot/30)^2) + 0.1*sum(((qdot-prev_qdot)/30)^2) + weight*(P(d_next)-P(d_ref))/dt; P(d)=max(0,1-d/5)^2',
            fraction_priority='first feasible TCP fraction, then score; fractions 1,1/2,1/4,1/8,1/16,1/32,0'))


def run_benchmark(kins,output,quick):
    rows=[];loops=25 if quick else 250
    for pose in ('home1','near_boundary'):
        prepared={}
        for side in ('left','right'):
            q=named_pose(pose,side);kin=kins['flange'];tcp,phi=kin.forward_tcp(q.tolist(),side)
            prepared[side]=(q,tcp,phi)
        for variant in ('one_ik','three_ik','baseline','search_va'):
            values=[];calls=[]
            for i in range(loops+10):
                targets={side:perturb(prepared[side][1],i%6,((i%2)*2-1)*(.001 if i%6<3 else .0002))
                         for side in ('left','right')}
                begin=time.perf_counter();ncalls=0
                for side in ('left','right'):
                    q,tcp,phi=prepared[side];kin=kins['flange'];goal=targets[side]
                    try:
                        if variant in ('one_ik','three_ik'):
                            offsets=(0.,) if variant=='one_ik' else (0.,-.01,.01)
                            for offset in offsets:
                                ncalls+=1;kin.inverse_tcp(goal.tolist(),phi+offset,q.tolist(),side)
                        elif variant=='baseline':
                            entry(kin,hw(q,side),side,goal,reference=q.tolist())
                        else:
                            p=ExperimentalPlanner(kin,side)
                            try:p.choose(q,q,np.zeros(7),goal)
                            finally:ncalls+=p.calls
                    except ValueError:
                        pass
                if i>=10:values.append((time.perf_counter()-begin)*1000);calls.append(ncalls)
            rows.append(dict(pose=pose,variant=variant,bimanual_ms=stats(values),calls=stats(calls),
                             over_450hz_budget=sum(v>1000/450 for v in values),over_90hz_budget=sum(v>1000/90 for v in values)))
    return dict(rows=rows,note='Serial local bimanual compute only; includes wrapper FK checks; excludes PICO input, logging, SSH, scheduling, ROS, motor response.')


def boundary_diagnostics():
    examples=[]
    for side in ('left','right'):
        for name in ('home1','home2','home3','known_boundary'):
            q=named_pose(name,side);x,y=np.abs(np.degrees(q[-2:]))
            xmax=max(a-2 for a,b in BOUNDARY if a>=2 and b-2>=y)
            examples.append(dict(side=side,pose=name,q_deg=np.degrees(q).tolist(),
                old_clearance=GUARD.clearance(q.tolist()[-2:]),geometric_distance=distance(q),
                sixth_outward_remaining=xmax-x,seventh_outward_remaining=cap(x+2)-2-y))
    return examples


def precision_diagnostics(kins):
    rows=[]
    for mode,kin in kins.items():
        for side in ('left','right'):
            for sixth in (0.,2.,4.,6.,10.,30.,50.,52.,60.,68.):
                for slack in (0.,.05,.5):
                    q=np.array([45.,75.,-100.,85.,135.,sixth,cap(sixth+2)-2-slack])
                    if side=='right':q*=MIRROR
                    q=np.radians(q).tolist()
                    try:
                        tcp,phi=kin.forward_tcp(q,side);out=kin.inverse_tcp(tcp,phi,q,side)
                        delta=max(abs(math.degrees(a-b)) for a,b in zip(q,out))
                        try:
                            GUARD.arm_segment(q,out,[0.]*7);accepted=True
                        except WristBlocked:
                            accepted=False
                        before=GUARD.clearance(q[-2:]);after=GUARD.clearance(out[-2:])
                        rows.append(dict(mode=mode,side=side,sixth=sixth,slack=slack,
                            max_roundtrip_joint_error_deg=delta,accepted=accepted,
                            input_clearance=before if math.isfinite(before) else None,
                            output_clearance=after if math.isfinite(after) else None))
                    except ValueError as exc:
                        rows.append(dict(mode=mode,side=side,sixth=sixth,slack=slack,error=str(exc)))
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--quick',action='store_true')
    parser.add_argument('--sections',default='matrix,continuous,benchmark')
    args=parser.parse_args();output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    (output/'evaluation_source.py').write_text(Path(__file__).read_text())
    kins={m:VendorKinematics(tools=tool_config(m)) for m in ('flange','taccap')}
    tracked=[Path(__file__),LIBRARY,ROOT/'apps/fairino_upper_probe/wrist_guard.py',
             ROOT/'scripts/pico4/fairino/vr_plan.py',ROOT/'apps/fairino_tcp_probe/vendor_kinematics.py',
             ROOT/'apps/fairino_tcp_probe/tcp_core.py',ROOT/'models/taccap/tool_config.json']
    summary=dict(seed=SEED,quick=args.quick,python=sys.version,platform=platform.platform(),
        created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),wrist_config_id=CONFIG_ID,
        files={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked},
        tool_configs={mode:kin.tools for mode,kin in kins.items()},boundary=boundary_diagnostics(),
        precision=precision_diagnostics(kins),
        limitations=['No robot, PICO, ROS or network I/O.',
            'Finite sampled arm-angle search cannot prove global infeasibility.',
            '30 deg/s, 120 deg/s^2, 5 deg/s and 20 deg/s^2 are experimental parameters, not robot settings.',
            'Continuous simulation stops at first rejection; the real stopping trajectory is not simulated.',
            'No torque, motor/controller dynamics, collision geometry beyond the existing wrist envelope, or hardware calibration.'])
    start=time.perf_counter()
    for section in args.sections.split(','):
        function={'matrix':run_matrix,'continuous':run_continuous,'benchmark':run_benchmark}[section]
        print('START '+section,flush=True);summary[section]=function(kins,output,args.quick)
        summary['wall_seconds']=time.perf_counter()-start
        (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False))
        print('DONE '+section,flush=True)
    print(str(output/'summary.json'),flush=True)


if __name__=='__main__':
    main()
