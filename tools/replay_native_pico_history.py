#!/usr/bin/env python3
"""Replay archived PICO goals through the OLD bounded planner, offline only.

This preserves the historical 10/30 bounded-step comparisons. It does not run
the current standalone direct-IK planner or describe its current behavior.

The only robot state source is existing JSONL on disk. No robot/network client
is imported. Logged pre-IK goals are preferred; their old clutch anchoring and
the absence of frames whose old IK failed are explicit replay limitations.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'scripts/pico4/fairino')]
from tools.ik_experiments.legacy_bounded_planner import NativePlanner
from tools.ik_experiments.art7_qp import rotation_log, transform
from tcp_core import MODEL_ID, pose_error, tcp_forward_matrix, multiply
from tool_frames import tool_config, validate_runtime, for_arm
from wrist_guard import WristGuard
from mounts import MAPPING_ID, tcp_in_base

WINDOWS = (
    ('home_flange_first', '20261022-103914', 1792636771.3896909, 30.),
    ('historical_wrist_refusal', '20261022-103914', 1792637364.6363294, 41.3636706),
    ('cross_body_taccap', '20261023-145015', 1792738260.2717206, 20.),
    ('flange_later', '20261023-160547', 1792742890.4647837, 20.),
    ('left_wrist_taccap', '20261022-144833', 1792651725.089575, 20.),
    ('right_wrist_flange', '20261020-205643', 1792501011.6122057, 20.),
)
RELEASE_EVENTS = {'grip_released', 'waiting_for_stationary_or_full_grip', 'input_paused'}
DT = .01
SOURCE_TIMEOUT = .15


def plain(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [plain(v) for v in value]
    return value


def stats(values):
    if not values: return None
    a = np.asarray(values, dtype=float)
    return dict(n=len(a), mean=float(a.mean()), rms=float(np.sqrt(np.mean(a*a))),
                p50=float(np.percentile(a, 50)), p95=float(np.percentile(a, 95)), maximum=float(a.max()))


def load_window(folder, name, stem, start, duration):
    path = folder/(stem+'.actions.jsonl'); end = start+duration
    tools = tool_config(); events = []; source_counts = Counter(); event_counts = Counter()
    truncated_at_home = None; digest = hashlib.sha256(); consumed = 0; previous_time = -math.inf
    source_backwards = 0; scanned = 0; first_line = None; last_line = None
    with path.open('rb') as stream:
        for lineno, raw in enumerate(stream, 1):
            digest.update(raw); consumed += len(raw); scanned += 1
            record = json.loads(raw); stamp = record.get('local_time')
            if type(stamp) not in (int, float) or not math.isfinite(stamp): continue
            if stamp < previous_time-1e-6: source_backwards += 1
            previous_time = stamp
            event = record.get('event')
            if event == 'tcp_tool_config':
                candidate = {key: record[key] for key in ('mode', 'id', 'transforms')}
                validate_runtime(candidate)
                if stamp >= start and events and candidate != tools:
                    end = stamp; truncated_at_home = 'tool_configuration_changed'; break
                tools = candidate
            if stamp < start: continue
            if stamp > end: break
            first_line = lineno if first_line is None else first_line; last_line = lineno
            if event:
                event_counts[event] += 1
                if event.startswith('home_'):
                    end = stamp; truncated_at_home = event; break
                if event in RELEASE_EVENTS:
                    events.append(dict(t=stamp, goals={}, line=lineno, kind=event, old_scale={}))
                continue
            request = record.get('request')
            if not isinstance(request, dict) or request.get('source') == 'home': continue
            if request.get('model') != MODEL_ID or request.get('tool_id', 'flange') != tools['id']:
                raise ValueError(f'{path.name}:{lineno}: archived model/tool configuration mismatch')
            goals = {}; source = None
            if record.get('tcp_chest_mm') and record.get('mapping') == MAPPING_ID:
                for side, matrix in record['tcp_chest_mm'].items():
                    goals[side] = transform(tcp_in_base(side, np.asarray(matrix).reshape(4, 4)))
                source = 'logged_pre_ik_tcp_chest'
            elif isinstance(request.get('arms'), dict):
                for side, entry in request['arms'].items():
                    goals[side] = transform(np.asarray(entry['tcp']).reshape(4, 4))
                source = 'old_solved_request_tcp_fallback'
            if not goals: continue
            source_counts[source] += 1
            scales = {side: arm.get('wrist_scale', 1.) for side, arm in request.get('arms', {}).items()}
            events.append(dict(t=stamp, goals=goals, line=lineno, kind='goals', source=source, old_scale=scales))
    if source_backwards:
        raise ValueError(f'{path.name}: nonmonotonic local_time; refusing to invent timing')
    if not any(event['goals'] for event in events):
        raise ValueError(f'{path.name}: selected window contains no replayable goals')
    status_path = folder/(stem+'.jsonl'); initial = None; status_digest = hashlib.sha256(); status_bytes = 0
    with status_path.open('rb') as stream:
        for lineno, raw in enumerate(stream, 1):
            status_digest.update(raw); status_bytes += len(raw)
            record = json.loads(raw); stamp = record.get('local_time', -math.inf)
            if stamp > start: break
            if isinstance(record.get('hardware'), dict):
                initial = dict(line=lineno, local_time=stamp, hardware=record['hardware'],
                               hw_age=record.get('hw_age'), ready=record.get('ready'), phase=record.get('phase'))
    if initial is None:
        raise ValueError(f'{status_path.name}: no preceding hardware snapshot')
    raw_age = initial.get('hw_age')
    initial_age = (start-initial['local_time']+raw_age
                   if type(raw_age) in (int, float) and math.isfinite(raw_age) and raw_age >= 0 else None)
    metadata = dict(name=name, stem=stem, actions=str(path), status_log=str(status_path),
        start=start, end=end, duration=end-start, requested_duration=duration,
        action_line_start=first_line, action_line_end=last_line, scanned_action_lines=scanned,
        consumed_action_prefix_bytes=consumed, consumed_action_prefix_sha256=digest.hexdigest(),
        consumed_status_prefix_bytes=status_bytes, consumed_status_prefix_sha256=status_digest.hexdigest(),
        source_goal_counts=dict(source_counts), source_event_counts=dict(event_counts),
        goal_records=sum(bool(event['goals']) for event in events),
        old_limited_records=sum(any(value < 1. for value in event['old_scale'].values()) for event in events),
        initial_snapshot={k: v for k, v in initial.items() if k != 'hardware'},
        initial_feedback_age=initial_age, initial_hardware=initial['hardware'], tools=tools,
        truncated_at_home=truncated_at_home)
    return metadata, events


def interpolate(a, b, amount):
    out = np.eye(4); out[:3, 3] = a[:3, 3]+amount*(b[:3, 3]-a[:3, 3])
    w = amount*rotation_log(b[:3, :3] @ a[:3, :3].T); angle = float(np.linalg.norm(w))
    if angle < 1e-12:
        rotation = np.eye(3)
    else:
        axis = w/angle
        skew = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]], [-axis[1], axis[0], 0.]])
        rotation = np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew @ skew)
    out[:3, :3] = rotation @ a[:3, :3]
    return out


def sample_goals(events, index, stamp, end):
    while index+1 < len(events) and events[index+1]['t'] <= stamp+1e-9: index += 1
    if stamp > end or index < 0:
        return {}, index, 'tail_brake' if stamp > end else 'before_first_goal'
    event = events[index]
    if not event['goals']: return {}, index, event['kind']
    if stamp-event['t'] > SOURCE_TIMEOUT: return {}, index, 'missing_frame_brake'
    goals = {side: goal.copy() for side, goal in event['goals'].items()}
    if index+1 < len(events):
        future = events[index+1]; gap = future['t']-event['t']
        if future['goals'] and 0 < gap <= SOURCE_TIMEOUT:
            fraction = min(1., max(0., (stamp-event['t'])/gap))
            for side in goals.keys() & future['goals'].keys():
                goals[side] = interpolate(goals[side], future['goals'][side], fraction)
    return goals, index, 'goals'


def episode(metadata, events, profile, output, tail_seconds=1., speed_deg_s=10.,
            acceleration_deg_s2=30., braking_deg_s2=30.):
    planner = NativePlanner(tools=metadata['tools'], speed_deg_s=speed_deg_s,
        acceleration_deg_s2=acceleration_deg_s2, braking_deg_s2=braking_deg_s2)
    guard = WristGuard()
    initial = copy.deepcopy(metadata['initial_hardware'])
    result = dict(name=metadata['name'], stem=metadata['stem'], profile=profile,
                  status='initial_rejected', reason='', source_start=metadata['start'],
                  source_end=metadata['end'], completed_steps=0, traces=[], failure=None)
    try:
        if metadata['initial_feedback_age'] is None:
            raise ValueError('recorded initial feedback age is unavailable')
        if metadata['initial_feedback_age'] > .06:
            raise ValueError('recorded initial feedback exceeds the 60 ms age limit')
        planner.reset(initial)
    except ValueError as exc:
        result['reason'] = str(exc); return result
    tau, delay = (0., 0.) if profile == 'ideal' else (.06, .04)
    actual = np.asarray(initial['position'], dtype=float)
    actual_v = np.asarray(initial['velocity'], dtype=float)
    command = np.asarray(initial['command_position'], dtype=float)
    history = deque([(0., actual.copy(), actual_v.copy(), command.copy())])
    duration = metadata['duration']+tail_seconds; steps = math.ceil(duration/DT)
    result['requested_steps'] = steps
    index = -1; command_v = np.zeros(14); max_speed = 0.; max_acc = 0.; max_actual = 0.
    min_static = math.inf; min_dynamic = math.inf; max_tracking = 0.; times = []
    position_errors = []; orientation_errors = []; active = 0; full = 0; modes = Counter()
    result['status'] = 'completed'
    trace_file = output/(metadata['name']+'-'+profile+'.jsonl')
    with trace_file.open('w') as trace:
        for step in range(steps):
            now = step*DT; stamp = metadata['start']+now
            goals, index, mode = sample_goals(events, index, stamp, metadata['end']); modes[mode] += 1
            while len(history) > 1 and history[1][0] <= now-delay+1e-10: history.popleft()
            _, measured, measured_v, feedback_command = history[0]
            hw = dict(position=measured.tolist(), velocity=measured_v.tolist(),
                      command_position=feedback_command.tolist())
            before = time.perf_counter()
            try:
                proposal = planner.propose(goals, hw, DT)
                # The simulated receiver independently uses FRESH plant state.
                for side, offset in (('left', 0), ('right', 7)):
                    arm = proposal['arms'][side]
                    guard.arm_segment(command[offset:offset+7].tolist(), arm['target'], actual_v[offset:offset+7].tolist())
                    guard.arm_segment(actual[offset:offset+7].tolist(), arm['target'], actual_v[offset:offset+7].tolist())
                    independent = multiply(tcp_forward_matrix(arm['target']), for_arm(metadata['tools'], side))
                    mm, deg = pose_error(independent, np.asarray(arm['tcp']).reshape(4, 4))
                    if mm > .02 or deg > .01: raise AssertionError('independent receiver FK mismatch')
                planner.commit(proposal)
            except (ValueError, RuntimeError, AssertionError) as exc:
                result.update(status='runtime_rejected', reason=str(exc), failure=dict(
                    step=step, virtual_time=now, source_time=stamp, source_line=events[index]['line'] if index >= 0 else None,
                    mode=mode, goal_sides=list(goals), actual_deg=np.degrees(actual[:14]).tolist(),
                    actual_velocity_deg_s=np.degrees(actual_v[:14]).tolist(),
                    reference_deg=np.degrees(command[:14]).tolist()))
                break
            times.append(1000*(time.perf_counter()-before))
            new_command = command.copy()
            for side, offset in (('left', 0), ('right', 7)):
                new_command[offset:offset+7] = proposal['arms'][side]['target']
            velocity = (new_command[:14]-command[:14])/DT; acceleration = (velocity-command_v)/DT
            if np.max(np.abs(velocity)) > math.radians(speed_deg_s)+1e-8 or np.max(np.abs(acceleration)) > math.radians(acceleration_deg_s2)+1e-6:
                raise AssertionError('replay command velocity/acceleration bound was violated')
            max_speed = max(max_speed, float(np.max(np.abs(np.degrees(velocity)))))
            max_acc = max(max_acc, float(np.max(np.abs(np.degrees(acceleration)))))
            command_v = velocity; command = new_command
            previous_actual = actual.copy()
            actual[:14] = command[:14] if tau == 0 else actual[:14]+(1-math.exp(-DT/tau))*(command[:14]-actual[:14])
            actual_v = (actual-previous_actual)/DT
            max_actual = max(max_actual, float(np.max(np.abs(np.degrees(actual_v[:14])))))
            max_tracking = max(max_tracking, float(np.max(np.abs(np.degrees(command[:14]-actual[:14])))))
            errors = {}
            for side, offset in (('left', 0), ('right', 7)):
                pair = actual[offset+5:offset+7].tolist()
                min_static = min(min_static, guard.clearance(pair))
                reserve = tuple(np.abs(np.degrees(actual_v[offset+5:offset+7]))*.2)
                min_dynamic = min(min_dynamic, guard.clearance(pair, reserve=reserve))
                if side in goals:
                    achieved = multiply(tcp_forward_matrix(actual[offset:offset+7].tolist()), for_arm(metadata['tools'], side))
                    mm, deg = pose_error(goals[side], achieved)
                    position_errors.append(mm); orientation_errors.append(deg); active += 1
                    full += int(proposal['results'][side]['success'])
                    errors[side] = dict(position_mm=mm, orientation_deg=deg)
            history.append((now+DT, actual.copy(), actual_v.copy(), command.copy()))
            result['completed_steps'] += 1
            if step%10 == 0 or step == steps-1:
                row = dict(t=now, source_time=stamp, mode=mode, goal_sides=list(goals), errors=errors,
                    command_deg=np.degrees(command[:14]).tolist(), actual_deg=np.degrees(actual[:14]).tolist(),
                    command_velocity_deg_s=np.degrees(command_v).tolist(), stopped=proposal['stopped'])
                result['traces'].append(row); trace.write(json.dumps(row, allow_nan=False)+'\n')
    result.update(max_command_speed_deg_s=max_speed, max_command_acceleration_deg_s2=max_acc,
        max_actual_speed_deg_s=max_actual, min_actual_static_clearance_deg=min_static if math.isfinite(min_static) else None,
        min_actual_dynamic_clearance_deg=min_dynamic if math.isfinite(min_dynamic) else None,
        max_tracking_error_deg=max_tracking, active_arm_samples=active, complete_target_arm_samples=full,
        measured_position_error_mm=stats(position_errors), measured_orientation_error_deg=stats(orientation_errors),
        planner_and_fresh_guard_ms=stats(times), sampled_modes=dict(modes),
        final_command_stopped=bool(np.max(np.abs(command_v)) <= 1e-10),
        final_measured_speed_deg_s=float(np.max(np.abs(np.degrees(actual_v[:14])))), trace=str(trace_file))
    if result['status'] == 'completed' and (not result['final_command_stopped'] or np.max(np.abs(actual_v[:14])) > .005):
        result['status'] = 'finished_unsettled'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--logs', type=Path, default=ROOT/'.local/pico4/fairino-logs')
    parser.add_argument('--output', type=Path, default=ROOT/'logs/20261026-native-pico-history')
    parser.add_argument('--only', help='Comma-separated default window labels')
    parser.add_argument('--speed-deg-s', type=float, default=10.,
                        help='Offline experiment only; does not change real-robot defaults')
    parser.add_argument('--acceleration-deg-s2', type=float, default=30.)
    parser.add_argument('--braking-deg-s2', type=float,
                        help='Assumed braking; defaults to the selected command acceleration')
    args = parser.parse_args(); output = args.output.resolve(); output.mkdir(parents=True, exist_ok=True)
    braking = args.acceleration_deg_s2 if args.braking_deg_s2 is None else args.braking_deg_s2
    for name, value in (('speed', args.speed_deg_s), ('acceleration', args.acceleration_deg_s2), ('braking', braking)):
        if not math.isfinite(value) or value <= 0:
            parser.error(name+' must be finite and positive')
    if braking > args.acceleration_deg_s2:
        parser.error('braking must not exceed the command acceleration')
    selected = set(args.only.split(',')) if args.only else None
    rows = []; windows = []; started = time.perf_counter()
    audit_path = ROOT/'logs/20261026-native-pico-history/audit/sessions.json'
    audit_rows = json.loads(audit_path.read_text()) if audit_path.exists() else []
    sources = [Path(__file__), ROOT/'tools/ik_experiments/legacy_bounded_planner.py',
               ROOT/'tools/ik_experiments/art7_native.py', ROOT/'tools/ik_experiments/art7_native.cpp',
               ROOT/'tools/ik_experiments/dynamic_wrist_policy.py']
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    for name, stem, start, duration in WINDOWS:
        if selected is not None and name not in selected: continue
        print('LOAD', name, flush=True)
        metadata, events = load_window(args.logs, name, stem, start, duration)
        metadata['archive_feedback_budget'] = None
        for audit in audit_rows:
            status = audit.get('status', {})
            if Path(status.get('path', '')).name != stem+'.jsonl': continue
            for data in status.get('replay_window_feedback_age', {}).values():
                if abs(data['start_local_time']-start) < 1e-5:
                    metadata['archive_feedback_budget'] = dict(data,
                        violation=bool(data.get('over_60ms_records', 0)),
                        interpretation='original active-status hardware ages; independent of synthetic follower assumptions')
        windows.append(metadata)
        (output/(name+'-source.json')).write_text(json.dumps(plain(metadata), ensure_ascii=False, indent=2))
        for profile in ('ideal', 'lag60_delay40'):
            print('REPLAY', name, profile, flush=True)
            row = episode(metadata, events, profile, output, speed_deg_s=args.speed_deg_s,
                acceleration_deg_s2=args.acceleration_deg_s2, braking_deg_s2=braking)
            rows.append(row)
            print(row['status'], row.get('completed_steps'), row['reason'], flush=True)
    summary = dict(planner_version='archived_bounded_step_not_current_direct_ik',
        windows=windows, episodes=rows, counts=dict(Counter(row['status'] for row in rows)),
        archive_feedback_budget_violating_windows=[window['name'] for window in windows
            if window.get('archive_feedback_budget') and window['archive_feedback_budget'].get('violation')],
        source_hashes=hashes, elapsed_wall_seconds=time.perf_counter()-started,
        assumptions=dict(command_hz=100, source_timeout_ms=150, feedback_age_bound_ms=60,
            joint_speed_limit_deg_s=args.speed_deg_s, joint_acceleration_limit_deg_s2=args.acceleration_deg_s2,
            braking_acceleration_deg_s2=braking,
            interpolation='linear translation, SO(3) shortest-arc interpolation between matching active arms',
            initial_state='unchanged last recorded hardware snapshot preceding each window',
            plant='ideal joint position following or first-order tau=60ms with feedback delayed40ms',
            failures='stop on first planning/receiver rejection; no simulated emergency-stop trajectory'),
        limitations=[
            'No hardware, ROS or network execution. Only arm kinematics and the existing wrist envelope are simulated.',
            'Logged pre-IK goals still contain old clutch anchoring/rebasing; this is not a new raw-controller-input closed loop.',
            'Old client logged these goals after successful old planning/validation; old IK failure frames are absent.',
            'Windows are selected representative excerpts, not an exhaustive replay of all historical sessions.',
            'Simple follower dynamics and braking assumptions have not been identified on the real robot.',
            'Synthetic ideal/delay40ms scenarios do not reproduce original feedback-age outages; archive_feedback_budget separately identifies windows that the real 60ms freshness gate would refuse.',
            'Reported timings include Python replay validation and are not a production server latency benchmark.'])
    (output/'replay-summary.json').write_text(json.dumps(plain(summary), ensure_ascii=False, indent=2, allow_nan=False))
    lines = ['Archived bounded-step PICO IK: historical comparison (not current direct IK)',
        f'No hardware/ROS/network activity. 100 Hz; {args.speed_deg_s:g} deg/s and {args.acceleration_deg_s2:g} deg/s² command limits; braking assumption {braking:g} deg/s².',
        'Original logged pre-IK TCP goals are converted from chest to DH coordinates.',
        'Initial hardware snapshots are retained; unsafe/moving/stale starts are explicitly rejected.',
        'Source goals are biased toward frames accepted by the old planner; old clutch anchoring remains.', '']
    for row in rows:
        lines.append(f"{row['name']} / {row['profile']}: {row['status']}, steps={row['completed_steps']}, {row['reason']}")
        if row.get('measured_position_error_mm'):
            lines.append(f"  TCP position p95/max={row['measured_position_error_mm']['p95']:.3f}/{row['measured_position_error_mm']['maximum']:.3f} mm; orientation p95/max={row['measured_orientation_error_deg']['p95']:.3f}/{row['measured_orientation_error_deg']['maximum']:.3f} deg")
            lines.append(f"  command speed/acceleration max={row['max_command_speed_deg_s']:.6f} deg/s / {row['max_command_acceleration_deg_s2']:.6f} deg/s²; final stopped={row['final_command_stopped']}")
    lines += ['', 'ORIGINAL FEEDBACK AGE GATE (separate from synthetic trajectories):']
    for window in windows:
        age = window.get('archive_feedback_budget')
        if age:
            lines.append(f"{window['name']}: active archived statuses={age['records']}, over60ms={age['over_60ms_records']}, max={age['maximum_ms']:.3f}ms, historical_window_would_be_refused={age['violation']}")
        else:
            lines.append(window['name']+': no complete source-feedback-age audit available')
    lines += ['', 'Counts: '+str(summary['counts']), '', *summary['limitations']]
    (output/'replay-report.txt').write_text('\n'.join(lines)+'\n')
    print(output/'replay-summary.json', flush=True)


if __name__ == '__main__':
    main()
