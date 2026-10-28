"""Offline, per-cycle diagnostics for an existing historical replay; no control I/O."""
from collections import Counter, deque
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.replay_native_pico_history import load_window, sample_goals, WINDOWS, DT, plain, stats
from tools.ik_experiments.legacy_bounded_planner import NativePlanner
from tcp_core import pose_error


def run(meta, events, profile, speed, acceleration, folder):
    planner = NativePlanner(tools=meta['tools'], speed_deg_s=speed,
        acceleration_deg_s2=acceleration, braking_deg_s2=acceleration)
    hw0 = meta['initial_hardware']; planner.reset(hw0)
    actual = np.asarray(hw0['position'], dtype=float)
    actual_v = np.asarray(hw0['velocity'], dtype=float)
    command = np.asarray(hw0['command_position'], dtype=float)
    history = deque([(0., actual.copy(), actual_v.copy(), command.copy())])
    tau, delay = (0., 0.) if profile == 'ideal' else (.06, .04)
    index = -1; prior_v = np.zeros(14); statuses = Counter()
    speed_sat = np.zeros(14, int); acc_sat = np.zeros(14, int)
    wrist_sat = np.zeros(14, int); active_samples = 0; rows = []
    errors = []; motor_errors = []; command_errors = []; iterations = []
    path = folder/f'trace-{speed:g}-{acceleration:g}-{profile}.jsonl'
    with path.open('w') as out:
        for step in range(math.ceil((meta['duration']+1.)/DT)):
            now = step*DT
            goals, index, mode = sample_goals(events, index, meta['start']+now, meta['end'])
            while len(history)>1 and history[1][0] <= now-delay+1e-10:
                history.popleft()
            _, measured, measured_v, feedback_command = history[0]
            hw = dict(position=measured.tolist(), velocity=measured_v.tolist(),
                      command_position=feedback_command.tolist())
            proposal = planner.propose(goals, hw, DT)
            planner.commit(proposal)
            new_command = command.copy()
            for side, off in (('left',0),('right',7)):
                new_command[off:off+7] = proposal['arms'][side]['target']
            v = (new_command[:14]-command[:14])/DT
            a = (v-prior_v)/DT
            prior_v = v; command = new_command
            before = actual.copy()
            actual[:14] = command[:14] if tau == 0 else actual[:14]+(1-math.exp(-DT/tau))*(command[:14]-actual[:14])
            actual_v = (actual-before)/DT
            if 'right' in goals:
                s = planner.solvers['right']; r = proposal['results']['right']
                command_tcp = s.forward(command[7:14]); actual_tcp = s.forward(actual[7:14])
                error = pose_error(goals['right'], actual_tcp)
                cmd_error = pose_error(goals['right'], command_tcp)
                motor_error = pose_error(command_tcp, actual_tcp)
                errors.append(error); command_errors.append(cmd_error); motor_errors.append(motor_error)
                active_samples += 1; statuses[r['status']] += 1; iterations.append(r['iterations'])
                speed_hit = np.abs(np.degrees(v)) >= speed-1e-6
                acceleration_hit = np.abs(np.degrees(a)) >= acceleration-1e-5
                speed_sat += speed_hit; acc_sat += acceleration_hit
                cap = np.asarray(r['wrist_speed_cap_deg_s'])
                wrist_hit = np.abs(np.degrees(v[12:14])) >= cap-1e-6
                wrist_sat[12:14] += wrist_hit
                row = dict(t=now, mode=mode, source_line=events[index]['line'],
                    goal=goals['right'], error_mm_deg=error, command_error_mm_deg=cmd_error,
                    motor_error_mm_deg=motor_error, command_deg=np.degrees(command[7:14]),
                    actual_deg=np.degrees(actual[7:14]), velocity_deg_s=np.degrees(v[7:14]),
                    acceleration_deg_s2=np.degrees(a[7:14]),
                    speed_saturated_joints=[i+1 for i in np.flatnonzero(speed_hit) if i>=7],
                    acceleration_saturated_joints=[i+1 for i in np.flatnonzero(acceleration_hit) if i>=7],
                    result=r)
                row=plain(row); rows.append(row); out.write(json.dumps(row,allow_nan=False)+'\n')
            history.append((now+DT, actual.copy(), actual_v.copy(), command.copy()))
    es=np.asarray(errors); ms=np.asarray(motor_errors); cs=np.asarray(command_errors)
    result=dict(profile=profile, speed=speed, acceleration=acceleration, active_cycles=active_samples,
        statuses=dict(statuses), speed_saturation_count=speed_sat.tolist(),
        acceleration_saturation_count=acc_sat.tolist(), wrist_cap_saturation_count=wrist_sat.tolist(),
        peak_position=rows[int(np.argmax(es[:,0]))], peak_orientation=rows[int(np.argmax(es[:,1]))],
        actual_position_mm=stats(es[:,0].tolist()), actual_orientation_deg=stats(es[:,1].tolist()),
        command_position_mm=stats(cs[:,0].tolist()), command_orientation_deg=stats(cs[:,1].tolist()),
        motor_position_mm=stats(ms[:,0].tolist()), motor_orientation_deg=stats(ms[:,1].tolist()),
        iterations=stats(iterations), trace=str(path))
    # Check exact agreement with the already reported replay, not merely a similar run.
    original_folder='final_after_joint_braking' if speed==10 else 'fast_command_30_120'
    original=json.loads((folder.parent/original_folder/'replay-summary.json').read_text())
    prior=next(e for e in original['episodes'] if e['name']==meta['name'] and e['profile']==profile)
    result['prior_peak_differences']={
        'position_mm':result['actual_position_mm']['maximum']-prior['measured_position_error_mm']['maximum'],
        'orientation_deg':result['actual_orientation_deg']['maximum']-prior['measured_orientation_error_deg']['maximum']}
    assert max(abs(x) for x in result['prior_peak_differences'].values())<1e-6
    return result


def main():
    folder=ROOT/'logs/20261026-native-pico-history/error_analysis';folder.mkdir(exist_ok=True)
    name,stem,start,duration=next(w for w in WINDOWS if w[0]=='right_wrist_flange')
    meta,events=load_window(ROOT/'.local/pico4/fairino-logs',name,stem,start,duration)
    results=[]
    for speed,acc in ((10.,30.),(30.,120.)):
        for profile in ('ideal','lag60_delay40'):
            result=run(meta,events,profile,speed,acc,folder)
            results.append(result)
            print(speed,profile, 'max',result['actual_position_mm']['maximum'],
                  result['actual_orientation_deg']['maximum'],flush=True)
    sources=['tools/ik_experiments/analyze_tracking_trace.py',
        'tools/ik_experiments/dynamic_wrist_policy.py','tools/ik_experiments/legacy_bounded_planner.py',
        'tools/replay_native_pico_history.py','tools/ik_experiments/art7_native.cpp']
    (folder/'trace-summary.json').write_text(json.dumps(dict(
        definitions=dict(position='norm(p_goal-p_simulated_actual), mm',
            orientation='SO(3) rotation distance between goal and simulated actual, degrees',
            motor_error='command FK vs simulated actual FK; geometric distances are not additive',
            scope='Same archived-goal replay, not full new client/server or measured hardware'),
        source_hashes={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in sources},
        metadata=meta,results=results),ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__': main()
