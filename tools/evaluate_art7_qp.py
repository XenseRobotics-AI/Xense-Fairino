#!/usr/bin/env python3
"""Offline comparison of the independent ART7 constrained IK prototype.

Never imports robot transport. Native vendor math is only a comparison oracle.
Continuous time is simulated; wall-clock benchmarks are reported separately.
"""
import argparse
from collections import Counter, deque
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from evaluate_pico_wrist_ik import (VendorKinematics, tool_config, GUARD, cap,
    np, named_pose, perturb, hw, entry, stats, hand_wave, MIRROR)
from tcp_core import BOUNDS_DEG, check_arm_limits, multiply, pose_error, tcp_forward_matrix
from tool_frames import for_arm
from ik_experiments.art7_qp import Art7ConstrainedIK


SEED = 20261026


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def independent_fk(q, tools, side):
    return np.asarray(multiply(tcp_forward_matrix(np.asarray(q).tolist()), for_arm(tools, side)))


def verify_result(result, seed, goal, tools, side, bounds=None, measured=None,
                  velocity=None, dt=None, previous_velocity=None):
    if not result['feasible']:
        assert not result['success'], result
        return None
    q = np.asarray(result['target'])
    assert q.shape == (7,) and np.isfinite(q).all()
    check_arm_limits(q.tolist())
    if bounds is not None:
        limits = np.radians(bounds)
        assert np.all(q >= limits[:, 0]-1e-9) and np.all(q <= limits[:, 1]+1e-9)
    v = np.zeros(7) if velocity is None else np.asarray(velocity)
    GUARD.arm_segment(np.asarray(seed).tolist(), q.tolist(), v.tolist())
    if measured is not None:
        GUARD.arm_segment(np.asarray(measured).tolist(), q.tolist(), v.tolist())
    tcp = independent_fk(q, tools, side)
    actual_error = pose_error(goal, tcp)
    report_error = pose_error(result['tcp'], tcp)
    assert report_error[0] < 1e-6 and report_error[1] < 1e-4, report_error
    if result['success']:
        assert actual_error[0] <= .020001 and actual_error[1] <= .010001, actual_error
    if dt is not None:
        qdot = (q-seed)/dt
        acceleration = (qdot-previous_velocity)/dt
        assert max(abs(np.degrees(qdot))) <= 30.+1e-5, np.degrees(qdot)
        assert max(abs(np.degrees(acceleration))) <= 120.+1e-3, np.degrees(acceleration)
    return actual_error


def random_pose(rng):
    q6 = rng.uniform(-63., 63.)
    q7 = rng.uniform(-1., 1.)*max(0., cap(abs(q6)+2.)-5.)
    return np.radians([rng.uniform(-145., 145.), rng.uniform(-90., 90.),
        rng.uniform(-145., 145.), rng.uniform(15., 100.), rng.uniform(-145., 145.), q6, q7])


def model_audit(models, vendors, quick):
    rng = np.random.default_rng(SEED)
    maximum = dict(fk_mm=0., fk_deg=0., vendor_mm=0., vendor_deg=0.,
                   jacobian_position_mm_per_rad=0., jacobian_angular=0.)
    count = 0
    for key, model in models.items():
        mode, side = key; tools = tool_config(mode)
        for index in range(12 if quick else 60):
            q = random_pose(rng); tcp = np.asarray(model.forward(q))
            for prefix, oracle in [('fk', independent_fk(q, tools, side)),
                                   ('vendor', vendors[mode].forward_tcp(q.tolist(), side)[0])]:
                mm, deg = pose_error(tcp, oracle)
                maximum[prefix+'_mm'] = max(maximum[prefix+'_mm'], mm)
                maximum[prefix+'_deg'] = max(maximum[prefix+'_deg'], deg)
                assert mm < 1e-5 and deg < 1e-4
            if index < (4 if quick else 20):
                jac = np.asarray(model.jacobian(q)); numeric = np.empty((6, 7)); eps=1e-6
                for axis in range(7):
                    delta = np.eye(7)[axis]*eps
                    plus, minus = np.asarray(model.forward(q+delta)), np.asarray(model.forward(q-delta))
                    numeric[:3, axis] = (plus[:3, 3]-minus[:3, 3])/(2*eps)
                    spin = ((plus[:3, :3]-minus[:3, :3])/(2*eps)) @ tcp[:3, :3].T
                    numeric[3:, axis] = [spin[2, 1], spin[0, 2], spin[1, 0]]
                errors = np.abs(jac-numeric)
                maximum['jacobian_position_mm_per_rad'] = max(maximum['jacobian_position_mm_per_rad'], float(errors[:3].max()))
                maximum['jacobian_angular'] = max(maximum['jacobian_angular'], float(errors[3:].max()))
                assert errors[:3].max() < 1e-4 and errors[3:].max() < 1e-6
            count += 1
    return dict(fk_cases=count, jacobian_cases=4*(4 if quick else 20), maximum=maximum)


def witnesses(vendors):
    rows = []
    for mode, vendor in vendors.items():
        for side in ('left', 'right'):
            for kind in ('joint_box', 'coupled_boundary'):
                if kind == 'joint_box':
                    q = np.radians([45.,75.,-100.,85.,135.,39.8,39.8])
                    bounds = list(BOUNDS_DEG); bounds[5]=(-40.,40.); bounds[6]=(-40.,40.)
                    if side == 'right':q *= MIRROR
                    origin, phi = vendor.forward_tcp(q.tolist(), side)
                    goal = perturb(origin, 0, -2.)
                else:
                    q = named_pose('known_boundary', side); q_goal=q.copy()
                    q_goal[6] += math.radians(.1)*(1 if side=='left' else -1)
                    goal = vendor.forward_tcp(q_goal.tolist(), side)[0]
                    origin, phi = vendor.forward_tcp(q.tolist(), side);bounds=list(BOUNDS_DEG)
                model = Art7ConstrainedIK(tools=tool_config(mode), side=side, bounds_deg=bounds)
                result = model.solve(goal, q, max_iterations=100)
                verify_result(result, q, goal, model.tools, side, bounds=bounds)
                native = vendor.inverse_tcp(np.asarray(goal).tolist(), phi, q.tolist(), side)
                clipped = np.clip(native, np.radians(bounds)[:,0], np.radians(bounds)[:,1])
                row = dict(mode=mode, side=side, kind=kind, seed_deg=np.degrees(q),
                    goal=goal, result=result, native_deg=np.degrees(native),
                    clipped_tcp_error_mm_deg=pose_error(goal,independent_fk(clipped,model.tools,side)))
                if result['feasible']:
                    row['solution_deg']=np.degrees(result['target'])
                    row['first_five_change_deg']=np.degrees(result['target'][:5]-q[:5])
                try:
                    original=entry(vendor,hw(q,side),side,goal,reference=q.tolist())
                    row['original_scale']=original['wrist_scale']
                except ValueError as exc:row['original_error']=str(exc)
                rows.append(plain(row))
    return rows


def static_matrix(models, vendors, output, quick):
    rng = np.random.default_rng(SEED); rows=[]; counts=Counter(); cases=[]
    for side in ('left','right'):
        states=[(name,named_pose(name,side)) for name in ('home1','home2','home3','near_boundary','known_boundary')]
        states += [('random_'+str(i),random_pose(rng)) for i in range(2 if quick else 12)]
        for mode in ('flange','taccap'):
            for name,q in states:
                origin=vendors[mode].forward_tcp(q.tolist(),side)[0]
                goals=[('hold',origin)]
                for axis in range(6):
                    for sign in (-1,1):
                        goals.append((f'axis{axis}_{sign}',perturb(origin,axis,sign*(2. if axis<3 else .5))))
                for target_name,goal in goals:cases.append((mode,side,name,q,target_name,goal))
    with (output/'static.jsonl').open('w') as file:
        for index,(mode,side,name,q,target_name,goal) in enumerate(cases):
            model=models[(mode,side)];result=model.solve(goal,q,max_iterations=80)
            error=verify_result(result,q,goal,model.tools,side)
            category='solved' if result['success'] else 'partial' if result['feasible'] else 'rejected'
            counts['prototype_'+category]+=1
            row=dict(mode=mode,side=side,pose=name,target_name=target_name,seed_deg=np.degrees(q),
                     goal=goal,result=result,error_mm_deg=error)
            try:
                baseline=entry(vendors[mode],hw(q,side),side,goal,reference=q.tolist())
                scale=baseline['wrist_scale'];base='full' if scale>=1-1e-8 else 'hold' if scale<=1e-8 else 'partial'
                row['baseline_scale']=scale
            except ValueError as exc:base='rejected';row['baseline_error']=str(exc)
            counts['baseline_'+base]+=1;row['baseline']=base
            if result['success'] and base!='full':counts['full_goal_where_baseline_not_full']+=1
            if not result['success'] and base=='full':counts['missed_where_baseline_full']+=1
            file.write(json.dumps(plain(row),ensure_ascii=False,allow_nan=False)+'\n')
            rows.append(dict(mode=mode,side=side,pose=name,target_name=target_name,category=category,
                baseline=base,iterations=result['iterations'],qp_solves=result['qp_solves'],elapsed_ms=result['elapsed_ms']))
            if (index+1)%100==0:print(f'static {index+1}/{len(cases)}',flush=True)
    return dict(total=len(rows),counts=dict(counts),timing_including_audit_not_benchmark=False,
                solver_time_ms=stats([r['elapsed_ms'] for r in rows]),rows=rows)


def continuous(models, output, quick):
    specs=[]
    for mode in ('flange','taccap'):
        for side in ('left','right'):
            for name in ('home1','home3','near_boundary','known_boundary'):
                for profile in ('ideal','lag30_delay20'):
                    specs.append((mode,side,name,profile))
    if quick:specs=specs[::4]
    rows=[]
    with (output/'continuous.jsonl').open('w') as file:
        for index,(mode,side,name,profile) in enumerate(specs):
            model=models[(mode,side)];q=named_pose(name,side);actual=q.copy();velocity=np.zeros(7);prev=np.zeros(7)
            initial=np.asarray(model.forward(q));history=deque([(0.,q.copy(),velocity.copy())]);feedback_q=q.copy();feedback_v=velocity.copy();last_slot=-1
            hz=450.;dt=1/hz;duration=.3 if quick else 1.;steps=round(duration*hz)
            tau,delay=(0.,0.) if profile=='ideal' else (.03,.02)
            trace=[];status='completed';reason='';peakv=0.;peaka=0.;maxmm=0.;maxdeg=0.;completed=0;full=0
            for i in range(1,steps+1):
                t=i*dt;slot=math.floor((t+1e-12)*100.)
                if slot>last_slot:
                    while len(history)>1 and history[1][0]<=t-delay:history.popleft()
                    feedback_q=history[0][1].copy();feedback_v=history[0][2].copy();last_slot=slot
                wave=hand_wave(t,duration,'mixed')*.25
                goal=initial.copy();goal[:3,3]+=wave[:3]
                for axis in range(3,6):goal=perturb(goal,axis,wave[axis])
                result=model.step(goal,q,dt,prev,measured=feedback_q,measured_velocity=feedback_v,max_iterations=4)
                if not result['feasible']:
                    status='planner_rejected';reason=result['reason'];break
                verify_result(result,q,goal,model.tools,side,measured=feedback_q,velocity=feedback_v,dt=dt,previous_velocity=prev)
                target=np.asarray(result['target'])
                try:
                    GUARD.arm_segment(q.tolist(),target.tolist(),velocity.tolist())
                    GUARD.arm_segment(actual.tolist(),target.tolist(),velocity.tolist())
                except ValueError as exc:status='receiver_rejected';reason=str(exc);break
                qdot=(target-q)/dt;acc=(qdot-prev)/dt;prev=qdot.copy()
                peakv=max(peakv,float(np.abs(np.degrees(qdot)).max()));peaka=max(peaka,float(np.abs(np.degrees(acc)).max()))
                before=actual.copy();actual=target.copy() if not tau else actual+(1-math.exp(-dt/tau))*(target-actual)
                velocity=(actual-before)/dt;history.append((t,actual.copy(),velocity.copy()))
                error=pose_error(goal,independent_fk(actual,model.tools,side));maxmm=max(maxmm,error[0]);maxdeg=max(maxdeg,error[1])
                q=target;completed+=1;full+=int(result['success'])
                if i%9==0:trace.append(dict(t=t,error_mm_deg=error,q_deg=np.degrees(q),success=result['success']))
            row=dict(mode=mode,side=side,pose=name,profile=profile,status=status,reason=reason,
                     completed_steps=completed,requested_steps=steps,full_target_steps=full,
                     peak_command_velocity_deg_s=peakv,peak_command_acceleration_deg_s2=peaka,
                     max_actual_tracking_error_mm=maxmm,max_actual_tracking_error_deg=maxdeg,
                     final_tcp_error_to_initial_mm_deg=pose_error(initial,independent_fk(q,model.tools,side)),trace=trace)
            file.write(json.dumps(plain(row),ensure_ascii=False,allow_nan=False)+'\n');file.flush()
            rows.append({k:plain(v) for k,v in row.items() if k!='trace'})
            print(f'continuous {index+1}/{len(specs)}: {status}',flush=True)
    return dict(episodes=len(rows),counts=dict(Counter(r['status'] for r in rows)),rows=rows,
                assumptions='450 Hz virtual clock; absolute small sinusoidal mixed TCP target; 100 Hz feedback; joint velocity 30 deg/s and acceleration 120 deg/s2; no stopping trajectory simulation.')


def benchmark(models, quick):
    rows=[];loops=20 if quick else 150
    for mode in ('flange','taccap'):
        for name in ('home1','near_boundary','known_boundary'):
            prepared={}
            for side in ('left','right'):
                q=named_pose(name,side);prepared[side]=(q,np.asarray(models[(mode,side)].forward(q)))
            for variant in ('static_solve','bounded_step'):
                timings=[];counts=Counter()
                for i in range(loops+10):
                    # Exceed the pose tolerance so timing exercises a QP, not just hold detection.
                    goals={side:perturb(tcp,i%6,(1 if i%2 else -1)*(.05 if i%6<3 else .02)) for side,(q,tcp) in prepared.items()}
                    start=time.perf_counter();pair=[]
                    for side,(q,tcp) in prepared.items():
                        model=models[(mode,side)]
                        result=(model.solve(goals[side],q,max_iterations=4) if variant=='static_solve' else
                                model.step(goals[side],q,1/450.,np.zeros(7),max_iterations=4))
                        pair.append(result)
                    elapsed=(time.perf_counter()-start)*1000
                    if i>=10:
                        timings.append(elapsed)
                        for result in pair:counts['success' if result['success'] else 'partial' if result['feasible'] else 'rejected']+=1
                rows.append(dict(mode=mode,pose=name,variant=variant,bimanual_ms=stats(timings),
                                 over_450hz_budget=sum(t>1000/450 for t in timings),counts=dict(counts)))
    return dict(rows=rows,note='Serial bimanual local solver only, excluding transport and scheduling; 0.05 mm / 0.02 degree targets exceed convergence tolerance; 4 internal iterations maximum; no real-time claim.')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--quick',action='store_true');p.add_argument('--sections',default='model,witnesses,static,continuous,benchmark')
    args=p.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    models={(mode,side):Art7ConstrainedIK(tools=tool_config(mode),side=side) for mode in ('flange','taccap') for side in ('left','right')}
    vendors={mode:VendorKinematics(tools=tool_config(mode)) for mode in ('flange','taccap')}
    paths=[Path(__file__),ROOT/'tools/ik_experiments/art7_qp.py',ROOT/'apps/fairino_tcp_probe/tcp_core.py',ROOT/'apps/fairino_upper_probe/wrist_guard.py']
    data=dict(created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),seed=SEED,quick=args.quick,
              files={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths},
              limitations=['Offline original prototype; not connected to production or hardware.',
                           'Finite local optimization; no global IK completeness or hardware safety proof.',
                           'Dynamics/feedback model is not identified from hardware; no braking trajectory.'])
    for path in paths[:2]:(out/path.name).write_bytes(path.read_bytes())
    for section in args.sections.split(','):
        print('START '+section,flush=True)
        if section=='model':result=model_audit(models,vendors,args.quick)
        elif section=='witnesses':result=witnesses(vendors)
        elif section=='static':result=static_matrix(models,vendors,out,args.quick)
        elif section=='continuous':result=continuous(models,out,args.quick)
        elif section=='benchmark':result=benchmark(models,args.quick)
        else:raise ValueError(section)
        data[section]=result;(out/'summary.json').write_text(json.dumps(plain(data),ensure_ascii=False,indent=2,allow_nan=False))
        print('DONE '+section,flush=True)
    print(out/'summary.json',flush=True)


if __name__=='__main__':main()
