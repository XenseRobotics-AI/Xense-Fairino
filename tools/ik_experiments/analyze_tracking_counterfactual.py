#!/usr/bin/env python3
"""Offline counterfactuals for the archived right-wrist tracking-error window.

No robot transport, ROS, hardware control, or production configuration is used.
Frozen targets preserve the entire replay prefix and planner transaction history.
Slow paths change source time only; joint bounds and follower models stay fixed.
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

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.replay_native_pico_history import (DT, WINDOWS, load_window, sample_goals,
    plain, stats, NativePlanner, WristGuard, pose_error, tcp_forward_matrix, multiply, for_arm)

BASE = ROOT/'logs/20261026-native-pico-history'


class ReplayState:
    def __init__(self, metadata, speed, acceleration, profile):
        self.metadata = metadata
        self.planner = NativePlanner(tools=metadata['tools'], speed_deg_s=speed,
            acceleration_deg_s2=acceleration, braking_deg_s2=acceleration)
        self.planner.reset(metadata['initial_hardware'])
        self.guard = WristGuard()
        self.speed, self.acceleration = speed, acceleration
        self.tau, self.delay = (0., 0.) if profile == 'ideal' else (.06, .04)
        self.actual = np.asarray(metadata['initial_hardware']['position'], dtype=float)
        self.actual_v = np.asarray(metadata['initial_hardware']['velocity'], dtype=float)
        self.command = np.asarray(metadata['initial_hardware']['command_position'], dtype=float)
        self.command_v = np.zeros(14)
        self.history = deque([(0., self.actual.copy(), self.actual_v.copy(), self.command.copy())])
        self.next_step = 0
        self.max_command_speed = self.max_command_acceleration = 0.
        self.min_dynamic_clearance = math.inf

    def tcp(self, side, values):
        offset = 0 if side == 'left' else 7
        return multiply(tcp_forward_matrix(values[offset:offset+7].tolist()),
                        for_arm(self.metadata['tools'], side))

    def errors(self, goals, values):
        return {side: dict(zip(('position_mm', 'orientation_deg'),
                              pose_error(goal, self.tcp(side, values))))
                for side, goal in goals.items()}

    def advance(self, goals):
        now = self.next_step*DT
        while len(self.history) > 1 and self.history[1][0] <= now-self.delay+1e-10:
            self.history.popleft()
        _, measured, velocity, feedback_command = self.history[0]
        hw = dict(position=measured.tolist(), velocity=velocity.tolist(),
                  command_position=feedback_command.tolist())
        proposal = self.planner.propose(goals, hw, DT)
        # Same fresh-plant, full-path receiver validation as the original replay.
        for side, offset in (('left', 0), ('right', 7)):
            arm = proposal['arms'][side]
            for start in (self.command, self.actual):
                self.guard.arm_segment(start[offset:offset+7].tolist(), arm['target'],
                                       self.actual_v[offset:offset+7].tolist())
            independent = multiply(tcp_forward_matrix(arm['target']), for_arm(self.metadata['tools'], side))
            mm, deg = pose_error(independent, np.asarray(arm['tcp']).reshape(4, 4))
            if mm > .02 or deg > .01:
                raise AssertionError('independent FK validation failed')
        self.planner.commit(proposal)
        command = self.command.copy()
        for side, offset in (('left', 0), ('right', 7)):
            command[offset:offset+7] = proposal['arms'][side]['target']
        velocity = (command[:14]-self.command[:14])/DT
        acceleration = (velocity-self.command_v)/DT
        speed_deg = float(np.max(np.abs(np.degrees(velocity))))
        acceleration_deg = float(np.max(np.abs(np.degrees(acceleration))))
        if speed_deg > self.speed+1e-6 or acceleration_deg > self.acceleration+1e-5:
            raise AssertionError('counterfactual violated configured command V/A')
        self.max_command_speed = max(self.max_command_speed, speed_deg)
        self.max_command_acceleration = max(self.max_command_acceleration, acceleration_deg)
        self.command_v, self.command = velocity, command
        previous = self.actual.copy()
        self.actual[:14] = (command[:14] if self.tau == 0 else
                           self.actual[:14]+(1-math.exp(-DT/self.tau))*(command[:14]-self.actual[:14]))
        self.actual_v = (self.actual-previous)/DT
        for offset in (0, 7):
            pair = self.actual[offset+5:offset+7].tolist()
            reserve = tuple(np.abs(np.degrees(self.actual_v[offset+5:offset+7]))*.2)
            self.min_dynamic_clearance = min(self.min_dynamic_clearance,
                                             self.guard.clearance(pair, reserve=reserve))
        self.history.append((now+DT, self.actual.copy(), self.actual_v.copy(), self.command.copy()))
        self.next_step += 1
        return dict(t=now, measured_errors=self.errors(goals, self.actual),
                    command_errors=self.errors(goals, self.command),
                    command_deg=np.degrees(self.command[:14]).tolist(),
                    command_velocity_deg_s=np.degrees(self.command_v).tolist(),
                    success={side: proposal['results'][side]['success'] for side in goals},
                    stopped=proposal['stopped'])

    def safety_summary(self):
        return dict(max_command_speed_deg_s=self.max_command_speed,
                    max_command_acceleration_deg_s2=self.max_command_acceleration,
                    min_fresh_dynamic_wrist_clearance_deg=self.min_dynamic_clearance)


def static_diagnostic(state, goals):
    rows = {}
    for side, goal in goals.items():
        offset = 0 if side == 'left' else 7
        seed = state.command[offset:offset+7].copy()
        result = state.planner.solvers[side].solve(goal, seed, measured=seed,
                        measured_velocity=np.zeros(7), max_iterations=200)
        row = {key: plain(result.get(key)) for key in
               ('success', 'feasible', 'status', 'iterations', 'position_error_mm',
                'orientation_error_deg', 'elapsed_ms')}
        row['diagnostic_only'] = 'Static solve has no per-cycle V/A/history constraints and is never published.'
        if result.get('feasible'):
            target = np.asarray(result['target'])
            row.update(target_deg=np.degrees(target).tolist(),
                       max_joint_change_deg=float(np.max(np.abs(np.degrees(target-seed)))),
                       verified_error=dict(zip(('position_mm', 'orientation_deg'),
                           pose_error(goal, state.planner.solvers[side].forward(target)))))
        rows[side] = row
    return rows


def frozen(metadata, events, artifact, peak, speed, acceleration, profile, seconds, output):
    label = f'freeze-{peak}-{speed:g}-{acceleration:g}-{profile}'
    baseline_rows = [row for row in artifact['traces'] if 'right' in row.get('errors', {})]
    field = 'position_mm' if peak == 'position' else 'orientation_deg'
    witness = max(baseline_rows, key=lambda row: row['errors']['right'][field])
    cutoff = round(witness['t']/DT)
    state = ReplayState(metadata, speed, acceleration, profile)
    index = -1
    for step in range(cutoff+1):
        goals, index, mode = sample_goals(events, index, metadata['start']+step*DT, metadata['end'])
        row = state.advance(goals)
    fixed = {side: goal.copy() for side, goal in goals.items()}
    if not fixed:
        raise AssertionError('peak witness did not reconstruct an active goal')
    result = dict(kind='freeze', label=label, peak=peak, profile=profile,
        speed_deg_s=speed, acceleration_deg_s2=acceleration, freeze_source_time=witness['source_time'],
        freeze_simulated_time_s=witness['t'], hold_seconds=seconds,
        peak_sample_interval_s=.1, archived_witness=witness, replayed_peak=row,
        source_action_line=events[index]['line'], frozen_goals={s: g.reshape(-1).tolist() for s, g in fixed.items()},
        static_from_peak_command=static_diagnostic(state, fixed),
        status='completed', failure=None, first_under_1mm_1deg_s=None,
        first_under_solver_tolerance_s=None)
    if speed == 10 and acceleration == 30 and profile == 'lag60_delay40':
        difference = {k: row['measured_errors']['right'][k]-witness['errors']['right'][k]
                      for k in ('position_mm', 'orientation_deg')}
        result['original_trace_reproduction_delta'] = difference
        if max(abs(v) for v in difference.values()) > 1e-6:
            raise AssertionError('prefix did not reproduce the original stored peak')
    trace_path = output/(label+'.jsonl')
    errors = []; commands = []; final = row
    with trace_path.open('w') as stream:
        stream.write(json.dumps(plain(dict(hold_t=0., **row)), allow_nan=False)+'\n')
        for i in range(math.ceil(seconds/DT)):
            try:
                row = state.advance(fixed)
            except (ValueError, RuntimeError, AssertionError) as exc:
                result.update(status='rejected', failure=dict(hold_t=(i+1)*DT, reason=str(exc)))
                break
            hold_t = (i+1)*DT
            final = row
            error = row['measured_errors']['right']; errors.append(error)
            commands.append(state.command[7:14].copy())
            if error['position_mm'] <= 1 and error['orientation_deg'] <= 1:
                if result['first_under_1mm_1deg_s'] is None:
                    result['first_under_1mm_1deg_s'] = hold_t
            if error['position_mm'] <= .02 and error['orientation_deg'] <= .01:
                if result['first_under_solver_tolerance_s'] is None:
                    result['first_under_solver_tolerance_s'] = hold_t
            if i % 10 == 0 or i == math.ceil(seconds/DT)-1:
                stream.write(json.dumps(plain(dict(hold_t=hold_t, **row)), allow_nan=False)+'\n')
    result.update(final=final, hold_error_mm=stats([e['position_mm'] for e in errors]),
                  hold_error_deg=stats([e['orientation_deg'] for e in errors]),
                  safety=state.safety_summary(), trace=str(trace_path))
    if len(commands) > 100:
        result['last_second_command_span_deg'] = np.degrees(np.ptp(np.asarray(commands[-100:]), axis=0)).tolist()
    return result


def slow(metadata, events, factor, speed, acceleration, profile, output):
    label = f'slow-{factor:g}x-{speed:g}-{acceleration:g}-{profile}'
    state = ReplayState(metadata, speed, acceleration, profile)
    duration = metadata['duration']*factor+1.
    index = -1; modes = Counter(); positions = []; rotations = []
    command_positions = []; command_rotations = []; full = 0; peak_position = peak_rotation = None
    result = dict(kind='slow', label=label, factor=factor, profile=profile,
        speed_deg_s=speed, acceleration_deg_s2=acceleration,
        simulated_seconds=duration, status='completed', failure=None, steps=0)
    trace_path = output/(label+'.jsonl')
    with trace_path.open('w') as stream:
        for step in range(math.ceil(duration/DT)):
            now = step*DT
            source = metadata['start']+now/factor
            goals, index, mode = sample_goals(events, index, source, metadata['end'])
            modes[mode] += 1
            try:
                row = state.advance(goals)
            except (ValueError, RuntimeError, AssertionError) as exc:
                result.update(status='rejected', failure=dict(t=now, source_time=source, reason=str(exc)))
                break
            result['steps'] += 1
            row.update(source_time=source, source_t=now/factor, mode=mode)
            for side, error in row['measured_errors'].items():
                positions.append(error['position_mm']); rotations.append(error['orientation_deg'])
                command_positions.append(row['command_errors'][side]['position_mm'])
                command_rotations.append(row['command_errors'][side]['orientation_deg'])
                full += int(row['success'][side])
                if peak_position is None or error['position_mm'] > peak_position['error']['position_mm']:
                    peak_position = dict(error=error, t=now, source_t=now/factor)
                if peak_rotation is None or error['orientation_deg'] > peak_rotation['error']['orientation_deg']:
                    peak_rotation = dict(error=error, t=now, source_t=now/factor)
            if step % 10 == 0 or step == math.ceil(duration/DT)-1:
                stream.write(json.dumps(plain(row), allow_nan=False)+'\n')
    result.update(measured_position_error_mm=stats(positions), measured_orientation_error_deg=stats(rotations),
        command_position_error_mm=stats(command_positions), command_orientation_error_deg=stats(command_rotations),
        full_target_samples=full, active_arm_samples=len(positions), modes=dict(modes),
        peak_position=peak_position, peak_orientation=peak_rotation,
        safety=state.safety_summary(), trace=str(trace_path))
    return result


def special_slow_freeze(metadata, events, output, seconds=15.):
    """Hold the residual 10x-path witness, retaining every prefix transaction."""
    factor, cutoff_time = 10., 186.7
    state = ReplayState(metadata, 10., 30., 'lag60_delay40')
    index = -1
    for step in range(round(cutoff_time/DT)+1):
        goals, index, _ = sample_goals(events, index, metadata['start']+step*DT/factor,
                                       metadata['end'])
        row = state.advance(goals)
    fixed = {side: goal.copy() for side, goal in goals.items()}
    prior = [json.loads(line) for line in (output/'slow-10x-10-30-lag60_delay40.jsonl').open()]
    witness = min(prior, key=lambda item: abs(item['t']-cutoff_time))
    difference = {key: row['measured_errors']['right'][key]-witness['measured_errors']['right'][key]
                  for key in ('position_mm', 'orientation_deg')}
    if max(abs(value) for value in difference.values()) > 1e-6:
        raise AssertionError('special slow-prefix reproduction differs from the main experiment')

    def diagnostic():
        p = state.planner._policies['right']
        q = state.command[7:14]
        caps = p._caps[p._selected]
        v = state.actual_v[12:14]
        return dict(selected_rectangle=p._selected,
                    selected_rectangle_caps_deg=np.degrees(caps).tolist(),
                    command_pair_margin_to_rectangle_deg=np.degrees(caps-np.abs(q[-2:])).tolist(),
                    command_joint_deg=np.degrees(q).tolist(),
                    command_velocity_deg_s=np.degrees(state.command_v[7:14]).tolist(),
                    measured_joint_deg=np.degrees(state.actual[7:14]).tolist(),
                    measured_velocity_deg_s=np.degrees(state.actual_v[7:14]).tolist(),
                    fresh_dynamic_wrist_clearance_deg=state.guard.clearance(
                        state.actual[12:14].tolist(), reserve=tuple(np.abs(np.degrees(v))*.2)))

    label = 'special-freeze-slow10x-source18.67-10-30-lag60_delay40'
    result = dict(kind='frozen_special', label=label, source_factor=factor,
        freeze_virtual_time_s=cutoff_time, freeze_source_t=cutoff_time/factor,
        freeze_source_time=metadata['start']+cutoff_time/factor, hold_seconds=seconds,
        speed_deg_s=10., acceleration_deg_s2=30., profile='lag60_delay40',
        prefix_reproduction_delta=difference, replayed_peak=row, initial_diagnostic=diagnostic(),
        frozen_goals={s: g.reshape(-1).tolist() for s, g in fixed.items()},
        static_from_peak_command=static_diagnostic(state, fixed), status='completed', failure=None,
        first_under_1mm_1deg_s=None, first_under_solver_tolerance_s=None)
    trace_path = output/(label+'.jsonl')
    commands = []
    final = row
    with trace_path.open('w') as stream:
        stream.write(json.dumps(plain(dict(hold_t=0., diagnostic=diagnostic(), **row)))+'\n')
        for i in range(math.ceil(seconds/DT)):
            hold_t = (i+1)*DT
            try:
                row = state.advance(fixed)
            except (ValueError, RuntimeError, AssertionError) as exc:
                result.update(status='rejected', failure=dict(hold_t=hold_t, reason=str(exc)))
                break
            final = row
            commands.append(state.command[7:14].copy())
            error = row['measured_errors']['right']
            if error['position_mm'] <= 1 and error['orientation_deg'] <= 1:
                if result['first_under_1mm_1deg_s'] is None:
                    result['first_under_1mm_1deg_s'] = hold_t
            if error['position_mm'] <= .02 and error['orientation_deg'] <= .01:
                if result['first_under_solver_tolerance_s'] is None:
                    result['first_under_solver_tolerance_s'] = hold_t
            if i % 10 == 0 or i == math.ceil(seconds/DT)-1:
                stream.write(json.dumps(plain(dict(hold_t=hold_t, diagnostic=diagnostic(), **row)))+'\n')
    result.update(final=final, final_diagnostic=diagnostic(), safety=state.safety_summary(), trace=str(trace_path))
    if len(commands) >= 100:
        result['last_second_command_span_deg'] = np.degrees(np.ptp(np.asarray(commands[-100:]), axis=0)).tolist()
    (output/'special-slow-freeze.json').write_text(json.dumps(plain(result), indent=2, allow_nan=False))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=BASE/'error_analysis/counterfactual')
    parser.add_argument('--slow-factors', default='5,10')
    parser.add_argument('--profiles', default='ideal,lag60_delay40')
    parser.add_argument('--freeze-seconds', type=float, default=10.)
    parser.add_argument('--special-only', action='store_true',
                        help='Append the source18.67s residual hold to an existing completed main run')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    original = json.loads((BASE/'final_after_joint_braking/replay-summary.json').read_text())
    fast = json.loads((BASE/'fast_command_30_120/replay-summary.json').read_text())
    artifact = next(e for e in original['episodes'] if e['name'] == 'right_wrist_flange' and e['profile'] == 'lag60_delay40')
    source = next(w for w in original['windows'] if w['name'] == 'right_wrist_flange')
    window = next(w for w in WINDOWS if w[0] == 'right_wrist_flange')
    metadata, events = load_window(Path(source['actions']).parent, *window)
    if args.special_only:
        print('SPECIAL slow10x source18.67: replay full186.7s prefix then hold15s', flush=True)
        started = time.perf_counter()
        row = special_slow_freeze(metadata, events, args.output)
        path = args.output/'counterfactual-summary.json'
        summary = json.loads(path.read_text())
        summary['frozen_special'] = [row]
        summary['special_elapsed_wall_seconds'] = time.perf_counter()-started
        summary['special_script_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        summary['original_main_run_script_archive'] = str(args.output/'main-run-script.py')
        path.write_text(json.dumps(plain(summary), indent=2, allow_nan=False))
        print('DONE SPECIAL', row['status'], row['final']['measured_errors'],
              row['static_from_peak_command'], flush=True)
        return
    sources = [Path(__file__), ROOT/'tools/replay_native_pico_history.py',
        ROOT/'tools/ik_experiments/legacy_bounded_planner.py', ROOT/'tools/ik_experiments/art7_native.py',
        ROOT/'tools/ik_experiments/art7_native.cpp', ROOT/'tools/ik_experiments/dynamic_wrist_policy.py']
    result = dict(window=metadata, source_hashes={str(p.relative_to(ROOT)):
        hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}, frozen=[], slowed=[],
        existing_baselines=[e for data in (original, fast) for e in data['episodes']
            if e['name'] == 'right_wrist_flange'], limitations=[
        'No ROS/network/hardware execution; these are controlled kinematic simulations.',
        'Freeze witnesses come from 10 Hz trace samples, not the exact 100 Hz global maxima.',
        'Frozen runs replay the complete prefix and retain command velocity, dynamic policy and delayed follower history.',
        'Static solves deliberately remove period V/A/history constraints and are reachability diagnostics only; no static target is published.',
        'Slow runs stretch source-time goals and release/gap intervals while supplying a fresh synthetic 100 Hz path; they are not a real network timeout replay.',
        'Recorded pre-IK goals retain old clutch anchoring and omit frames rejected by the old planner.',
        'Configured speed/acceleration and first-order follower models remain hypotheses, not identified real-robot performance.',
        'Changing speed changes branch choices and optimization trajectories; differences are not additive causal percentages.'])
    started = time.perf_counter()
    def save():
        result['elapsed_wall_seconds'] = time.perf_counter()-started
        (args.output/'counterfactual-summary.json').write_text(json.dumps(plain(result), indent=2, allow_nan=False))
    for speed, acceleration in ((10., 30.), (30., 120.)):
        for peak in ('position', 'orientation'):
            print('FREEZE', speed, acceleration, peak, flush=True)
            row = frozen(metadata, events, artifact, peak, speed, acceleration,
                         'lag60_delay40', args.freeze_seconds, args.output)
            result['frozen'].append(row); save()
            print('DONE', row['label'], row['status'], row['final']['measured_errors'], flush=True)
    for factor in [float(s) for s in args.slow_factors.split(',') if s]:
        for speed, acceleration in ((10., 30.), (30., 120.)):
            for profile in args.profiles.split(','):
                print('SLOW', factor, speed, acceleration, profile, flush=True)
                row = slow(metadata, events, factor, speed, acceleration, profile, args.output)
                result['slowed'].append(row); save()
                print('DONE', row['label'], row['status'], row['measured_position_error_mm'],
                      row['measured_orientation_error_deg'], flush=True)
    print(args.output/'counterfactual-summary.json', flush=True)


if __name__ == '__main__':
    main()
