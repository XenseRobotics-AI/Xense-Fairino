#!/usr/bin/env python3
"""Compare saved torso tracking at different speeds, without robot access.

Reported ripple metrics remove slow error trends; they are not vibration
measurements or a controlled experiment. State logs are approximately 10 Hz.
"""
from __future__ import annotations

import argparse
import collections
import datetime
import json
from pathlib import Path

import numpy as np

from verify_body_trajectory_offline import CORE, ROOT, DEG, load_config, sample_power


def load_session(path):
    rows=[];plans=[]
    for line_number,line in enumerate(path.open(),1):
        row=json.loads(line)
        row['_line']=line_number
        if row.get('hardware'):
            rows.append(row)
        if 'body_plan' in row:
            plans.append(row)
    return rows,plans


def unique_hardware_rows(rows):
    """Deduplicate repeated feedback cached by the reporting loop.

    stamp and hw_age are read at slightly different instants, so estimated
    receive timestamps can differ by microseconds for the same cached frame.
    Require both <1 ms timestamp separation and identical measured/command data.
    Identical stationary measurements with a later receive time remain valid.
    """
    unique=[];duplicates=0
    for row in rows:
        hw=row['hardware']
        signature=tuple(hw['position']+hw['velocity']+hw['command_position'])
        estimated=row['stamp']-row['hw_age']
        if unique:
            old=unique[-1];oldhw=old['hardware']
            oldsignature=tuple(oldhw['position']+oldhw['velocity']+oldhw['command_position'])
            if signature==oldsignature and abs(estimated-(old['stamp']-old['hw_age']))<.001:
                duplicates+=1;continue
        unique.append(row)
    return unique,duplicates


def basic(values):
    a=np.asarray(values,dtype=float)
    return dict(mean=float(np.mean(a)),std=float(np.std(a)),
                peak_to_peak=float(np.ptp(a)),
                percentile_95_minus_5=float(np.percentile(a,95)-np.percentile(a,5)),
                max_abs=float(np.max(np.abs(a))))


def detrended(values,coordinate):
    a=np.asarray(values,dtype=float);x=np.asarray(coordinate,dtype=float)
    x=(x-x.mean())/max(float(np.std(x)),1e-12)
    answer={'raw':basic(a)}
    for degree in (1,2):
        if len(x)>=degree+3:
            fit=np.polynomial.polynomial.polyfit(x,a,degree)
            residual=a-np.polynomial.polynomial.polyval(x,fit)
            answer[f'height_polynomial_degree_{degree}_removed']=basic(residual)
    return answer


def summarize_motion(samples):
    if len(samples)<4:
        return dict(sample_count=len(samples),usable=False,
                    reason='fewer than four motion feedback samples')
    height=np.asarray([s['command_height_percent'] for s in samples])
    times=np.asarray([s['hardware_time'] for s in samples])
    residual=np.asarray([s['position_error_deg'] for s in samples])
    vel_error=np.asarray([s['velocity_error_deg_s'] for s in samples])
    actual_v=np.asarray([s['actual_velocity_deg_s'] for s in samples])
    desired_v=np.asarray([s['desired_velocity_deg_s'] for s in samples])
    result=dict(sample_count=len(samples),usable=True,duration_s=float(times[-1]-times[0]),
                first_line=samples[0]['line'],last_line=samples[-1]['line'],
                first_local_time=samples[0]['local_time'],last_local_time=samples[-1]['local_time'],
                command_height_percent=[float(height.min()),float(height.max())],
                hardware_sample_interval_s=basic(np.diff(times)),
                hardware_age_s=basic([s['hardware_age_s'] for s in samples]),
                short_window_warning=bool(len(samples)<20 or times[-1]-times[0]<2.),
                joint_metrics={},derived_pitch_deg=detrended([s['pitch_error_deg'] for s in samples],height),
                derived_height_error_mm=detrended([s['height_error_mm'] for s in samples],height),
                max_model_command_mismatch_rad=max(s['command_model_mismatch_rad'] for s in samples))
    for j in range(3):
        result['joint_metrics'][f'J{j+18}']=dict(
            position_error_deg=detrended(residual[:,j],height),
            velocity_error_deg_s=detrended(vel_error[:,j],height),
            measured_velocity_deg_s=basic(actual_v[:,j]),
            desired_velocity_deg_s=basic(desired_v[:,j]))
    return result


def target_error(model,row,planned):
    hw=row['hardware'];q=np.asarray(hw['position']);command=np.asarray(hw['command_position'])
    final=np.asarray(planned['final']);actual_state=model.state(q.tolist());target_state=model.state(final.tolist())
    return dict(line=row['_line'],local_time=row['local_time'],robot_stamp=row['stamp'],
                phase=row.get('phase'),height_percent=actual_state['height'],
                estimated_hardware_stamp=row['stamp']-row['hw_age'],
                stale_display_warning=bool(row['hw_age']>.05),
                final_height_percent=target_state['height'],
                height_error_to_final_mm=actual_state['height_mm']-target_state['height_mm'],
                pitch_error_to_final_deg=actual_state['forward']-target_state['forward'],
                joint_error_to_final_deg=((q-final)*DEG)[17:20].tolist(),
                max_joint_error_to_final_deg=float(np.max(np.abs(q-final))*DEG),
                joint_tracking_error_deg=((q-command)*DEG)[17:20].tolist(),
                command_error_to_final_deg=((command-final)*DEG)[17:20].tolist(),
                measured_velocity_deg_s=(np.asarray(hw['velocity'])*DEG)[17:20].tolist(),
                hardware_age_s=row['hw_age'],global_fault=hw['error'],
                joint_faults=hw['errorcode'])


def audit_session(path,model):
    rows,events=load_session(path);result=[]
    unique_rows,duplicate_count=unique_hardware_rows(rows)
    dt=np.diff([r['stamp'] for r in rows])
    for number,event in enumerate(events,1):
        p=event['body_plan'];main_index=p['stages'].index('升降与姿态调节')
        nodes=p['points'][main_index*100:main_index*100+101]
        duration=nodes[-1]['time']-nodes[0]['time']
        start_state=model.state(nodes[0]['positions']);final_state=model.state(nodes[-1]['positions'])
        direction='down' if final_state['height']<start_state['height'] else 'up'
        # Invert the monotonic J19 command along the saved main trajectory. This
        # aligns desired velocity to hardware command position rather than PC
        # receive time; it does not estimate or remove mechanical tracking lag.
        dense_time=np.linspace(0.,duration,10001)
        dense_q,dense_v,_=sample_power(nodes,dense_time)
        increasing=dense_q[-1,18]>dense_q[0,18]
        lookup_q=dense_q[:,18] if increasing else dense_q[::-1,18]
        lookup_t=dense_time if increasing else dense_time[::-1]
        end=events[number]['local_time'] if number<len(events) else float('inf')
        available=[r for r in rows if event['local_time']<=r['local_time']<end]
        unique_available,duplicates=unique_hardware_rows(available)
        samples=[];excluded=collections.Counter()
        excluded['duplicate_cached_hardware']=duplicates
        for row in unique_available:
            if '约 ' not in row.get('phase','') or row.get('ready'):
                excluded['not_active_trajectory']+=1;continue
            hw=row['hardware'];command=np.asarray(hw['command_position']);actual=np.asarray(hw['position'])
            rel=row['local_time']-event['local_time']
            if rel<nodes[0]['time']:
                excluded['before_main_stage']+=1;continue
            progress=float(np.interp(command[18],lookup_q,lookup_t))
            if not .005*duration<progress<.995*duration:
                excluded['main_endpoint_or_outside']+=1;continue
            expected_q=np.asarray([np.interp(progress,dense_time,dense_q[:,j]) for j in range(20)])
            expected_v=np.asarray([np.interp(progress,dense_time,dense_v[:,j]) for j in range(20)])
            mismatch=float(np.max(np.abs(expected_q[15:20]-command[15:20])))
            if mismatch>1e-4:
                excluded['command_not_on_main_geometry']+=1;continue
            ca,aa=model.state(command.tolist()),model.state(actual.tolist())
            samples.append(dict(line=row['_line'],local_time=row['local_time'],
                                hardware_time=row['stamp']-row['hw_age'],
                                relative_to_plan_s=rel,inferred_main_time_s=progress,
                                command_height_percent=ca['height'],actual_height_percent=aa['height'],
                                position_error_deg=((actual-command)*DEG)[17:20].tolist(),
                                actual_velocity_deg_s=(np.asarray(hw['velocity'])*DEG)[17:20].tolist(),
                                desired_velocity_deg_s=(expected_v*DEG)[17:20].tolist(),
                                velocity_error_deg_s=((np.asarray(hw['velocity'])-expected_v)*DEG)[17:20].tolist(),
                                pitch_error_deg=aa['forward']-ca['forward'],
                                height_error_mm=aa['height_mm']-ca['height_mm'],
                                hardware_age_s=row['hw_age'],
                                command_model_mismatch_rad=mismatch))
        complete=next((r for r in available if r.get('phase','').startswith('轨迹完成')),None)
        completion=None
        if complete:
            completion={'first_completion_feedback':target_error(model,complete,p)}
            fresh=next((r for r in unique_available
                        if r['stamp']-r['hw_age']>=complete['stamp']
                        and r['hw_age']<=.08),None)
            if fresh:
                completion['first_fresh_hardware_after_completion']=target_error(model,fresh,p)
                completion['fresh_hardware_delay_after_completion_s']=fresh['stamp']-fresh['hw_age']-complete['stamp']
            after=next((r for r in unique_available
                        if r['stamp']-r['hw_age']>=complete['stamp']+1.),None)
            if after:
                completion['one_second_later']=target_error(model,after,p)
                completion['elapsed_to_later_sample_s']=after['stamp']-complete['stamp']
                completion['hardware_elapsed_to_later_sample_s']=after['stamp']-after['hw_age']-complete['stamp']
            near=[r for r in unique_available
                  if complete['stamp']<=r['stamp']-r['hw_age']<=complete['stamp']+1.]
            completion['max_actual_speed_first_second_deg_s']=(max(
                max(abs(x)*DEG for x in r['hardware']['velocity'][17:20]) for r in near) if near else None)
        summary=dict(name=f'{path.stem}_motion_{number}',session=path.stem,motion_number=number,
                     plan_line=event['_line'],plan_local_time=event['local_time'],
                     plan_local_time_iso=datetime.datetime.fromtimestamp(event['local_time']).isoformat(),
                     commanded_speed_limit_deg_s=p['speed_deg_s'],direction=direction,
                     start_height_percent=start_state['height'],target_height_percent=final_state['height'],
                     plan_duration_s=p['duration'],main_stage_start_s=nodes[0]['time'],
                     main_stage_duration_s=duration,initial_actual_state=model.state(p['start']),
                     retained_motion_samples=len(samples),exclusions=dict(excluded),
                     whole_main_motion=summarize_motion(samples),
                     comparison_windows={},completion=completion,
                     observed_fault_rows=sum(bool(r['hardware']['error']) or any(r['hardware']['errorcode']) for r in available))
        if direction=='down':
            for low,high in ((86.,98.),(90.,98.),(86.,90.),(86.,88.)):
                selected=[s for s in samples if low<=s['command_height_percent']<=high]
                summary['comparison_windows'][f'{low:g}_to_{high:g}_percent']=summarize_motion(selected)
        if path.stem=='20261023-171132':
            summary['historical_time_windows']={}
            for lo,hi in ((2.,4.),(4.,6.),(6.,8.),(8.,10.7)):
                selected=[s for s in samples if lo<=s['relative_to_plan_s']<hi]
                summary['historical_time_windows'][f'{lo:g}_to_{hi:g}_seconds']=summarize_motion(selected)
        result.append(summary)
    return dict(path=str(path),hardware_rows=len(rows),plan_count=len(events),
                unique_hardware_rows=len(unique_rows),cached_hardware_duplicates=duplicate_count,
                state_interval_s=basic(dt),approx_observed_state_hz=1/float(np.median(dt)),
                max_hardware_age_s=max(r['hw_age'] for r in rows),
                hardware_age_s=basic([r['hw_age'] for r in rows]),
                rows_hardware_age_gt_50ms=sum(r['hw_age']>.05 for r in rows),
                rows_hardware_age_gt_100ms=sum(r['hw_age']>.1 for r in rows),
                motions=result)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--old',type=Path,default=CORE/'logs/20261023-171132.jsonl')
    parser.add_argument('--new',type=Path,default=CORE/'logs/20261026-092322.jsonl')
    parser.add_argument('--output',type=Path,default=ROOT/'logs/20261026-lift-speed-comparison/independent-metrics.json')
    args=parser.parse_args();model=load_config()
    sessions=[audit_session(path,model) for path in (args.old,args.new)]
    report=dict(offline_only=True,sessions=sessions,method={
        'tracking_error':'hardware measured position minus hardware command_position, same feedback record',
        'deduplication':'Adjacent identical position/velocity/command payloads with estimated hardware time stamp-hw_age within 1 ms are counted once.',
        'desired_velocity_alignment':'Invert monotonic J19 command along saved main-stage quintic; evaluate desired velocity there. No wall-clock alignment or fitted tracking delay.',
        'detrending':'Remove degree-1 and degree-2 polynomials of commanded height from error; preserve raw mean and spread separately.',
        'matched_windows':'Descending only; commanded height 86–98%, 90–98%, 86–90%, and 86–88%. Short windows explicitly flagged.',
        'completion':'First report with phase starting 轨迹完成 may contain cached pre-completion hardware; also record first hardware timestamp after completion and first hardware timestamp >=1 s later.'},
        limitations=[
            'Only saved local logs were read. No robot connection, motion or production code changes.',
            'Approximately 10 Hz state records cannot measure or rule out vibration above about 5 Hz; timing is not perfectly uniform.',
            'Derived pitch is the sum of joint positions relative to calibrated neutral, not an independent IMU measurement.',
            'Polynomial-detrended error is a low-frequency tracking fluctuation proxy, not proof of mechanical vibration amplitude.',
            'Higher-speed passes cover matched heights in much less time and may have too few samples or oscillation cycles.',
            'Runs differ in initial pitch/horizontal offset, elapsed time, trajectory endpoints and possibly loading; not a controlled A/B trial.',
            'Configured joint speed is a cap, not the actual constant joint or vertical speed.',
            'Desired velocity alignment assumes hardware command_position represents the target for that feedback sample.',
            'Completion-report target error with stale cached hardware must not be interpreted as physical lag; compare fresh post-completion feedback.',
            'Larger physical lag may remain even when visible shaking is reduced; completion declaration is not itself a convergence proof.'])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print('Report:',args.output)
    for session in sessions:
        print(session['path'],session['plan_count'],'plans',session['approx_observed_state_hz'],'Hz')
        for motion in session['motions']:
            matched=motion['comparison_windows'].get('86_to_98_percent',{})
            if matched.get('usable'):
                q=matched['joint_metrics']['J20']['position_error_deg']
                print(motion['motion_number'],motion['commanded_speed_limit_deg_s'],matched['sample_count'],
                      'J20 detrended std',q['height_polynomial_degree_1_removed']['std'],
                      q.get('height_polynomial_degree_2_removed',{}).get('std'))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
