#!/usr/bin/env python3
"""Offline native IK / predictive wrist policy evaluation; never controls hardware."""
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
import evaluate_art7_qp as base
from evaluate_art7_qp import (np, tool_config, VendorKinematics, named_pose,
    independent_fk, GUARD, pose_error, perturb, hand_wave, stats, plain)
from ik_experiments.art7_native import Art7NativeIK
from ik_experiments.dynamic_wrist_policy import PredictiveWristIK


def make_model(mode, side, predictive=True, feedback_age=.06):
    native = Art7NativeIK(tools=tool_config(mode), side=side)
    return PredictiveWristIK(native, feedback_age=feedback_age) if predictive else native


def run_episode(mode, side, pose, profile, kind='mixed', duration=1., amplitude=.25,
                predictive=True, release=False):
    # Delayed 100 Hz feedback; the receiver checks newer state every 450 Hz step.
    tau, delay = {'ideal':(0.,0.), 'lag30_delay20':(.03,.02),
                  'lag60_delay40':(.06,.04)}[profile]
    model=make_model(mode, side, predictive, max(.04, delay+.02))
    q=named_pose(pose,side);actual=q.copy();actual_v=np.zeros(7);previous_v=np.zeros(7)
    feedback_q=q.copy();feedback_v=actual_v.copy();history=deque([(0.,actual.copy(),actual_v.copy())]);last_slot=-1
    dt=1/450.;motion_seconds=duration*.25 if release else duration
    tail=.5 if release else 0.;steps=round((motion_seconds+tail)/dt)
    initial=np.asarray(model.forward(q));goal=initial.copy();frozen_goal=None
    status='completed';reason='';trace=[];max_speed=0.;max_acc=0.;completed=0;full=0;holds=0
    max_command_error=[0.,0.];max_actual_error=[0.,0.];peak_target=[0.,0.];peak_actual=[0.,0.]
    margins=[];times=[];cells=Counter();first_failure=None;command_stop_time=None
    for i in range(1,steps+1):
        t=i*dt;slot=math.floor((t+1e-12)*100.)
        if slot>last_slot:
            while len(history)>1 and history[1][0]<=t-delay:history.popleft()
            feedback_q=history[0][1].copy();feedback_v=history[0][2].copy();last_slot=slot
        if release and t>motion_seconds:
            if frozen_goal is None:frozen_goal=np.asarray(model.forward(q))
            goal=frozen_goal
        else:
            wave=hand_wave(min(t,duration),duration,kind)*amplitude
            goal=initial.copy();goal[:3,3]+=wave[:3]
            for axis in range(3,6):goal=perturb(goal,axis,wave[axis])
        target_excursion=pose_error(initial,goal)
        peak_target=[max(a,b) for a,b in zip(peak_target,target_excursion)]
        before=time.perf_counter()
        braking = release and t > motion_seconds
        if braking:
            expected_braking_velocity=np.sign(previous_v)*np.maximum(np.abs(previous_v)-math.radians(120.)*dt,0.)
            expected_braking_q=q+dt*expected_braking_velocity
            validation_goal=independent_fk(expected_braking_q,model.tools,side)
            result=model.brake(q,dt,previous_v,measured=feedback_q,measured_velocity=feedback_v)
        else:
            validation_goal=goal
            result=model.step(goal,q,dt,previous_v,measured=feedback_q,
                              measured_velocity=feedback_v,max_iterations=4)
        times.append((time.perf_counter()-before)*1000.)
        if not result['feasible']:
            status='planner_rejected';reason=result['reason'];first_failure=i;break
        target=np.asarray(result['target'])
        base.verify_result(result,q,validation_goal,model.tools,side,measured=feedback_q,velocity=feedback_v,
                           dt=dt,previous_velocity=previous_v)
        try:
            GUARD.arm_segment(q.tolist(),target.tolist(),actual_v.tolist())
            GUARD.arm_segment(actual.tolist(),target.tolist(),actual_v.tolist())
        except ValueError as exc:
            status='receiver_rejected';reason=str(exc);first_failure=i;break
        if hasattr(model, 'commit'):
            model.commit(result)
        qdot=(target-q)/dt;acceleration=(qdot-previous_v)/dt
        if braking:
            assert np.max(np.abs(target-expected_braking_q))<1e-10
            assert np.all(np.abs(qdot)<=np.abs(previous_v)+1e-9)
            assert np.all(qdot*previous_v>=-1e-12)
            if np.max(np.abs(qdot))<1e-8 and command_stop_time is None:
                command_stop_time=t-motion_seconds
        max_speed=max(max_speed,float(np.max(np.abs(np.degrees(qdot)))))
        max_acc=max(max_acc,float(np.max(np.abs(np.degrees(acceleration)))))
        if np.max(np.abs(target-q))<1e-11:holds+=1
        previous_actual=actual.copy()
        actual=target.copy() if tau==0 else actual+(1-math.exp(-dt/tau))*(target-actual)
        actual_v=(actual-previous_actual)/dt
        command_error=pose_error(goal,independent_fk(target,model.tools,side))
        actual_error=pose_error(goal,independent_fk(actual,model.tools,side))
        actual_excursion=pose_error(initial,independent_fk(actual,model.tools,side))
        max_command_error=[max(a,b) for a,b in zip(max_command_error,command_error)]
        max_actual_error=[max(a,b) for a,b in zip(max_actual_error,actual_error)]
        peak_actual=[max(a,b) for a,b in zip(peak_actual,actual_excursion)]
        reserve=tuple(np.abs(np.degrees(actual_v[-2:]))*.2)
        clearance=GUARD.clearance(actual[-2:].tolist(),reserve=reserve)
        margins.append(clearance)
        if result.get('selected_rectangle_deg') is not None:
            cells[str(result['selected_rectangle_deg'])]+=1
        q=target;previous_v=qdot;completed+=1;full+=int(result['success'])
        history.append((t,actual.copy(),actual_v.copy()))
        if i%9==0 or i==1:
            trace.append(dict(t=t,command_error=command_error,actual_error=actual_error,
                q_deg=np.degrees(q),measured_deg=np.degrees(actual),
                speed_deg_s=np.degrees(qdot),clearance=clearance,
                wrist_speed_cap_deg_s=result.get('wrist_speed_cap_deg_s'),
                selected_rectangle_deg=result.get('selected_rectangle_deg')))
    if release and status=='completed' and command_stop_time is None:
        status='stop_not_completed';reason='command velocity did not reach zero within the stopping tail'
    ratio=[None if target<1e-5 else actual/target for actual,target in zip(peak_actual,peak_target)]
    failure_context = None if status == 'completed' else dict(reference_deg=np.degrees(q),
        previous_velocity_deg_s=np.degrees(previous_v),actual_deg=np.degrees(actual),
        actual_velocity_deg_s=np.degrees(actual_v),feedback_deg=np.degrees(feedback_q),
        feedback_velocity_deg_s=np.degrees(feedback_v),result=result)
    return dict(mode=mode,side=side,pose=pose,profile=profile,kind=kind,duration=duration,
        amplitude=amplitude,predictive=predictive,release=release,status=status,reason=reason,
        completed_steps=completed,requested_steps=steps,full_target_steps=full,hold_steps=holds,
        first_failure_step=first_failure,failure_context=failure_context,peak_command_velocity_deg_s=max_speed,
        peak_command_acceleration_deg_s2=max_acc,max_command_error_mm_deg=max_command_error,
        max_actual_error_mm_deg=max_actual_error,peak_target_excursion_mm_deg=peak_target,
        peak_actual_excursion_mm_deg=peak_actual,excursion_ratio=ratio,
        min_actual_dynamic_clearance_deg=min(margins) if margins else None,
        final_command_velocity_deg_s=np.degrees(previous_v),
        final_actual_velocity_deg_s=np.degrees(actual_v),command_stop_time_s=command_stop_time,
        final_actual_error_to_command_mm_deg=pose_error(independent_fk(q,model.tools,side),independent_fk(actual,model.tools,side)),
        final_error_mm_deg=pose_error(goal,independent_fk(actual,model.tools,side)),
        timing_ms=stats(times),cells=dict(cells),trace=trace)


def run_scenarios(output, phase, quick):
    specs=[]
    for mode in ('flange','taccap'):
        for side in ('left','right'):
            if phase=='continuous':
                for pose in ('home1','home3','near_boundary','known_boundary'):
                    for profile in ('ideal','lag30_delay20'):
                        for predictive in (False,True):
                            specs.append(dict(mode=mode,side=side,pose=pose,profile=profile,predictive=predictive))
            elif phase=='stress':
                for pose in ('home3','near_boundary','known_boundary'):
                    for profile in ('ideal','lag60_delay40'):
                        for kind in ('rotate','mixed'):
                            for duration in (.5,1.):
                                for amplitude in (.25,1.):
                                    specs.append(dict(mode=mode,side=side,pose=pose,profile=profile,kind=kind,duration=duration,amplitude=amplitude))
            elif phase=='release':
                for pose in ('home3','near_boundary','known_boundary'):
                    for profile in ('ideal','lag60_delay40'):
                        for kind in ('rotate','mixed'):
                            specs.append(dict(mode=mode,side=side,pose=pose,profile=profile,kind=kind,amplitude=1.,release=True))
    if quick:specs=[s for s in specs if s['mode']=='flange' and s['side']=='left' and s['pose']=='known_boundary']
    summaries=[]
    with (output/(phase+'.jsonl')).open('w') as file:
        for index,spec in enumerate(specs):
            result=run_episode(**spec)
            file.write(json.dumps(plain(result),ensure_ascii=False,allow_nan=False)+'\n');file.flush()
            summaries.append({k:plain(v) for k,v in result.items() if k!='trace'})
            if index%8==0 or index+1==len(specs):print(f'{phase} {index+1}/{len(specs)}: {result["status"]}',flush=True)
    return dict(episodes=len(summaries),counts=dict(Counter(('predictive/' if r['predictive'] else 'native/')+r['status'] for r in summaries)),rows=summaries)


def run_benchmark(quick):
    rows=[];loops=40 if quick else 500
    for mode in ('flange','taccap'):
        for pose in ('home1','near_boundary','known_boundary'):
            prepared={side:named_pose(pose,side) for side in ('left','right')}
            for variant in ('static','bounded','predictive'):
                models={side:make_model(mode,side,variant=='predictive') for side in prepared}
                origins={side:models[side].forward(q) for side,q in prepared.items()}
                times=[];counts=Counter()
                for i in range(loops+20):
                    goals={side:perturb(tcp,i%6,(1 if i%2 else -1)*(.05 if i%6<3 else .02)) for side,tcp in origins.items()}
                    # These are independent timing samples, not a synthetic trajectory.
                    # Reset is included in predictive timing, making that timing conservative.
                    before=time.perf_counter();results=[]
                    for side,q in prepared.items():
                        model=models[side]
                        if variant=='predictive':model.reset()
                        result=(model.solve(goals[side],q,max_iterations=4) if variant=='static' else
                                model.step(goals[side],q,1/450.,np.zeros(7),max_iterations=4))
                        if variant=='predictive' and result['feasible']:
                            model.commit(result)
                        results.append(result)
                    elapsed=(time.perf_counter()-before)*1000
                    if i>=20:
                        times.append(elapsed)
                        for result in results:counts['full' if result['success'] else 'partial' if result['feasible'] else 'rejected']+=1
                rows.append(dict(mode=mode,pose=pose,variant=variant,bimanual_ms=stats(times),
                    over_450hz_budget=sum(t>1000/450 for t in times),counts=dict(counts)))
    return dict(rows=rows,note='Full Python-to-C++ calls plus original Python input/output checks and predictive policy; serial bimanual, no transport or runtime scheduling; 0.05 mm/0.02 deg goals, 4 internal iterations maximum.')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--sections',default='core,continuous,stress,release,benchmark');parser.add_argument('--quick',action='store_true')
    parser.add_argument('--resume',action='store_true',help='Append phases only when all recorded source hashes match.')
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    paths=[Path(__file__),ROOT/'tools/evaluate_art7_qp.py',ROOT/'tools/ik_experiments/art7_native.cpp',
           ROOT/'tools/ik_experiments/art7_native.py',ROOT/'tools/ik_experiments/dynamic_wrist_policy.py',
           ROOT/'apps/fairino_upper_probe/wrist_guard.py',ROOT/'apps/fairino_tcp_probe/tcp_core.py']
    result=dict(created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),quick=args.quick,
        files={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths},
        assumptions=['Offline only; original receiver guard unchanged.',
            'Positive first-order, non-overshooting plant; 100 Hz bounded-age feedback; velocity evidence bounded by recent commands and measured velocity.',
            'Experimental command limits 30 deg/s and 120 deg/s2; native wall-clock compute separate from virtual-time plant.',
            'Release cases request dedicated monotone joint deceleration and hold the final stop point; no actual hardware stopping validation.'])
    if args.resume:
        previous=json.loads((out/'summary.json').read_text())
        if previous['files']!=result['files'] or previous['quick']!=args.quick:
            raise ValueError('Cannot append results from different source versions or test sizes')
        result=previous
    for path in paths:(out/path.name).write_bytes(path.read_bytes())
    previous_wall=result.get('wall_seconds',0.)
    started=time.perf_counter()
    for section in args.sections.split(','):
        print('START '+section,flush=True)
        if section=='core':
            models={(mode,side):make_model(mode,side,False) for mode in ('flange','taccap') for side in ('left','right')}
            vendors={mode:VendorKinematics(tools=tool_config(mode)) for mode in ('flange','taccap')}
            # Only the existing offline evaluator's factory is swapped. Runtime code is never imported.
            base.Art7ConstrainedIK=Art7NativeIK
            value=dict(model=base.model_audit(models,vendors,args.quick),witnesses=base.witnesses(vendors),
                       static=base.static_matrix(models,vendors,out,args.quick))
        elif section=='benchmark':value=run_benchmark(args.quick)
        else:value=run_scenarios(out,section,args.quick)
        result[section]=value;result['wall_seconds']=previous_wall+time.perf_counter()-started
        (out/'summary.json').write_text(json.dumps(plain(result),ensure_ascii=False,indent=2,allow_nan=False))
        print('DONE '+section,flush=True)
    print(out/'summary.json',flush=True)


if __name__=='__main__':main()
