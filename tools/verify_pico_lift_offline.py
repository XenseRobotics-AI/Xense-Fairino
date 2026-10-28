#!/usr/bin/env python3
"""Historical incremental lift math checks, without ROS or a robot.

The PICO runtime now uses vr_lift_presets.PresetLiftPlanner (long-push high/mid
presets). This tool retains the old incremental planner comparison; it does not
validate or describe the current preset runtime. See test_vr_lift_presets.py and
test_vr_lift_preset_receiver.py for the current behavior.

This verifies generated targets and pure input validation, not a physical plant,
motor tracking, real scheduling, or vibration. Finite differences are equivalent
command metrics over wall-clock sample intervals, NOT measured acceleration.
Only this script and the imported pure-math modules run; no receiver is started.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
for directory in ("apps/fairino_upper_probe/body_control", "apps/fairino_tcp_probe",
                  "scripts/pico4/fairino"):
    sys.path.insert(0, str(ROOT / directory))

from body_core import load_config  # noqa: E402
from tcp_core import MODEL_ID  # noqa: E402
from vr_core import KIND, LIFT_MODE, validate  # noqa: E402
from vr_lift import DEADBAND, LiftInput, LiftPlanner  # noqa: E402


def digest(path):
    return hashlib.sha256((ROOT / path).read_bytes()).hexdigest()


def intervals(pattern, duration):
    """Clip the final interval so every case spans the same physical duration."""
    elapsed, result, i = 0., [], 0
    while elapsed < duration - 1e-10:
        dt = min(pattern[i % len(pattern)], duration - elapsed)
        result.append(dt)
        elapsed += dt
        i += 1
    return result


def gap_intervals(gap, duration=2., hz=450.):
    return intervals([1. / hz], 1.) + [gap] + intervals([1. / hz], duration - 1. - gap)


class Verification:
    def __init__(self):
        self.model = load_config()
        self.checks = []
        self.cases = {}
        self.traces = []

    def check(self, name, result, **evidence):
        self.checks.append(dict(name=name, status="pass" if result else "fail", **evidence))

    def rejects(self, name, call, match=None):
        try:
            call()
        except ValueError as exc:
            self.check(name, match is None or match in str(exc), exception=str(exc))
        else:
            self.check(name, False, exception=None)

    def start(self, height=25., forward=12., roll=0., yaw=20.):
        # Nonzero arms and head verify that lift only changes J18--J20.
        hold = [math.radians(i * .1) for i in range(20)]
        return self.model.pose(dict(height=height, forward=forward, roll=roll, yaw=yaw), hold)

    def simulate(self, name, dts, axis=1., initial=None, input_gate=False):
        q = self.start() if initial is None else initial[:]
        start = q[:]
        planner, gate = LiftPlanner(self.model.config), LiftInput()
        if input_gate:
            gate.value({"right": dict(axis_y=0., axis_click=False, joystick_available=True)}, True)
        rows = []
        elapsed = 0.
        for index, dt in enumerate([0.] + list(dts)):
            elapsed += dt
            raw = axis(elapsed, index) if callable(axis) else axis
            value = gate.value({"right": dict(axis_y=raw, axis_click=False,
                                              joystick_available=True)}, True) if input_gate else raw
            before = q[:]
            q = planner.sample(value, q, 100. + elapsed)
            state = self.model.state(q)
            row = dict(case=name, index=index, time_s=elapsed, dt_s=dt, raw_axis=raw,
                       gated_axis=value, height_mm=state['height_mm'],
                       forward_offset_mm=state['forward_offset_mm'], forward_deg=state['forward'],
                       step_mm=state['height_mm'] - self.model.state(before)['height_mm'],
                       max_joint_step_deg=max(abs(math.degrees(a-b)) for a, b in zip(q, before)),
                       max_other_joint_change_deg=max(abs(math.degrees(a-b))
                                                      for a, b in zip(q[:17], start[:17])))
            for j in (18, 19, 20):
                row[f'q{j}_deg'] = math.degrees(q[j-1])
            rows.append(row)

        # Differences use physical time, including gaps. Their sample times are
        # interval midpoints, which matters for a deliberately irregular cadence.
        derivative_times = {}
        for order, prefix in ((1, 'v'), (2, 'a'), (3, 'jerk')):
            for index, row in enumerate(rows):
                key = (order, index)
                previous = rows[index-1] if index else None
                if order == 1:
                    ta = previous['time_s'] if previous else None
                    tb = row['time_s']
                    previous_prefix = 'q'
                    unit = 'deg'
                else:
                    ta = derivative_times.get((order-1, index-1))
                    tb = derivative_times.get((order-1, index))
                    previous_prefix = 'v' if order == 2 else 'a'
                    unit = 'deg_s' if order == 2 else 'deg_s2'
                suffix = ('deg_s', 'deg_s2', 'deg_s3')[order-1]
                valid = previous is not None and ta is not None and tb is not None and tb > ta
                derivative_times[key] = (ta+tb)/2 if valid else None
                for j in (18, 19, 20):
                    before = previous.get(f'{previous_prefix}{j}_{unit}') if previous else None
                    after = row.get(f'{previous_prefix}{j}_{unit}')
                    row[f'{prefix}{j}_{suffix}'] = ((after-before)/(tb-ta)
                                                   if valid and before is not None and after is not None else None)

        def absolute_peak(prefix, suffix):
            return max((abs(row[f'{prefix}{j}_{suffix}']) for row in rows for j in (18, 19, 20)
                        if row[f'{prefix}{j}_{suffix}'] is not None), default=0.)

        duration = rows[-1]['time_s']
        state0 = self.model.state(start)
        summary = dict(samples=len(rows), duration_s=duration,
                       update_interval_min_ms=min(dts)*1000 if dts else 0.,
                       update_interval_max_ms=max(dts)*1000 if dts else 0.,
                       start_height_mm=rows[0]['height_mm'], end_height_mm=rows[-1]['height_mm'],
                       net_height_mm=rows[-1]['height_mm']-rows[0]['height_mm'],
                       mean_vertical_speed_mm_s=(rows[-1]['height_mm']-rows[0]['height_mm'])/duration,
                       max_vertical_step_mm=max(abs(row['step_mm']) for row in rows),
                       max_joint_step_deg=max(row['max_joint_step_deg'] for row in rows),
                       max_secant_joint_speed_deg_s=absolute_peak('v', 'deg_s'),
                       max_equivalent_joint_acceleration_deg_s2=absolute_peak('a', 'deg_s2'),
                       max_equivalent_joint_jerk_deg_s3=absolute_peak('jerk', 'deg_s3'),
                       mean_signed_joint_speed_deg_s={str(j): (rows[-1][f'q{j}_deg']-rows[0][f'q{j}_deg'])/duration
                                                     for j in (18, 19, 20)},
                       max_horizontal_offset_drift_mm=max(abs(row['forward_offset_mm']-state0['forward_offset_mm'])
                                                         for row in rows),
                       max_forward_angle_drift_deg=max(abs(row['forward_deg']-state0['forward']) for row in rows),
                       max_other_joint_change_deg=max(row['max_other_joint_change_deg'] for row in rows),
                       computed_lift_range_mm=[planner.low_mm, planner.high_mm])
        self.check(name+': first frame holds', rows[0]['max_joint_step_deg'] == 0.)
        self.check(name+': other joints held', summary['max_other_joint_change_deg'] == 0.)
        self.check(name+': horizontal and pitch invariants',
                   summary['max_horizontal_offset_drift_mm'] < 1e-6 and summary['max_forward_angle_drift_deg'] < 1e-8)
        self.check(name+': sampled joint speed limit',
                   summary['max_secant_joint_speed_deg_s'] <= self.model.config['default_speed_deg_s'] + 1e-6,
                   maximum_deg_s=summary['max_secant_joint_speed_deg_s'])
        self.check(name+': model height boundaries',
                   all(-1e-7 <= row['height_mm'] <= self.model.travel + 1e-7 for row in rows))
        self.cases[name] = summary
        self.traces.extend(rows)
        return summary, rows

    def run(self):
        # Fresh accepted samples; these are arrival schedules, not achieved CPU rates.
        for hz in (450, 333, 90):
            summary, _ = self.simulate(f'fixed_{hz}hz', intervals([1./hz], 2.))
            self.check(f'fixed_{hz}hz: elapsed-time compensation', abs(summary['net_height_mm']-30.) < 1e-6,
                       expected_mm=30., actual_mm=summary['net_height_mm'])
        summary, _ = self.simulate('jitter_2_2_8ms', intervals([.002, .002, .008], 2.))
        self.check('jitter: elapsed-time compensation', abs(summary['net_height_mm']-30.) < 1e-6,
                   expected_mm=30., actual_mm=summary['net_height_mm'])
        for gap in (.020, .050, .080):
            summary, rows = self.simulate(f'gap_{round(gap*1000)}ms', gap_intervals(gap))
            expected = 15.*(2.-max(0., gap-.05))
            self.check(f'gap_{round(gap*1000)}ms: capped elapsed-time integration',
                       abs(summary['net_height_mm']-expected) < 1e-6,
                       expected_mm=expected, actual_mm=summary['net_height_mm'])
            gap_row = max(rows, key=lambda row:row['dt_s'])
            self.cases[f'gap_{round(gap*1000)}ms']['gap_step_mm'] = gap_row['step_mm']

        for hz in (90, 333, 450):
            self.simulate(f'startup_{hz}hz', intervals([1./hz], .8),
                          axis=lambda t, _: 0. if t < .2 else 1.)
            self.simulate(f'reversal_{hz}hz', intervals([1./hz], 1.2),
                          axis=lambda t, _: 1. if t < .6 else -1.)
            summary, rows = self.simulate(f'pause_resume_{hz}hz', intervals([1./hz], 1.2),
                          axis=lambda t, _: 0. if .4 <= t < .8 else 1.)
            transitions = [i for i in range(1, len(rows))
                           if abs(rows[i-1]['gated_axis']) <= DEADBAND < abs(rows[i]['gated_axis'])]
            neutral = [row for row in rows if abs(row['gated_axis']) <= DEADBAND]
            self.check(f'pause_resume_{hz}hz: neutral and first resume hold',
                       all(row['max_joint_step_deg'] == 0. for row in neutral)
                       and all(rows[i]['max_joint_step_deg'] == 0. for i in transitions))

        dead_values = [0., .1999, -.1999, .2, -.2]
        summary, _ = self.simulate('deadband_inside', intervals([1./90], 1.),
                                  axis=lambda _, i: dead_values[i % len(dead_values)], input_gate=True)
        self.check('deadband: no motion within inclusive 0.2', summary['max_joint_step_deg'] == 0.)
        edge_values = [0., .1999, .2001, .21, .1999, -.2001, -.21, -.1999]
        self.simulate('deadband_edge_jitter', intervals([1./90], 1.),
                      axis=lambda _, i: edge_values[i % len(edge_values)], input_gate=True)
        for height in (0., 25., 50., 90., 99.9, 100.):
            for direction in (-1., 1.):
                self.simulate(f'height_{height:g}_axis_{direction:+g}', intervals([1./90], .5),
                              axis=direction, initial=self.start(height))

        offset = self.start(25.)
        offset[18] += math.radians(.4)
        offset[17] -= math.radians(.4)
        self.simulate('nonzero_horizontal_offset', intervals([1./333], 2.), initial=offset)
        for direction, height in ((1., 0.), (-1., 100.)):
            summary, _ = self.simulate('full_up' if direction > 0 else 'full_down', [.02]*2500,
                                      axis=direction, initial=self.start(height))
            expected = self.model.travel if direction > 0 else 0.
            self.check('full travel reaches '+('high' if direction > 0 else 'low'),
                       abs(summary['end_height_mm']-expected) < 1e-6,
                       expected_mm=expected, actual_mm=summary['end_height_mm'])

        # Pure input gate, bad data, calibration and tilt constraints.
        gate = LiftInput()
        def frame(axis=0., click=False, available=True):
            return {'right': dict(axis_y=axis, axis_click=click, joystick_available=available)}
        self.check('input gate: cannot start while stick already held', gate.value(frame(1.), True) == 0.)
        for name, value, active in (('pause', frame(), False), ('missing', None, True),
                                    ('click', frame(click=True), True), ('unavailable', frame(available=False), True)):
            gate.value(frame(), True)
            was_armed = gate.value(frame(1.), True) == 1.
            stopped = gate.value(value, active) == 0.
            blocked = gate.value(frame(1.), True) == 0.
            gate.value(frame(), True)
            resumed = gate.value(frame(-1.), True) == -1.
            self.check('input gate: '+name+' requires neutral rearm', was_armed and stopped and blocked and resumed)

        planner = LiftPlanner(self.model.config)
        q = self.start()
        for label, value in (('boolean', True), ('NaN', float('nan')), ('infinity', float('inf')),
                             ('out of range', 1.01), ('text', 'up')):
            self.rejects('invalid axis '+label, lambda value=value: planner.sample(value, q, 10.))
        self.rejects('missing upright calibration', lambda:LiftPlanner({**self.model.config, 'upright_confirmed':False}))
        tilted = self.start(90., forward=0., roll=10.)
        planner = LiftPlanner(self.model.config)
        planner.sample(-1., tilted, 10.)
        self.rejects('descent through roll-height gate', lambda:planner.sample(-1., tilted, 10.02), '侧倾')

        planner = LiftPlanner(self.model.config)
        planner.sample(1., q, 10.)
        q = planner.sample(1., q, 10.02)
        hold = planner.sample(0., q, 20.)
        resumed = planner.sample(1., hold, 60.)
        next_q = planner.sample(1., resumed, 65.)
        dz = self.model.state(next_q)['height_mm'] - self.model.state(resumed)['height_mm']
        self.check('long neutral pause has no accumulated catchup', hold == q and resumed == q)
        # Planner only: a real receiver rejects expired requests before this call.
        self.check('planner-only long active gap capped to 50ms', 0. <= dz <= .75 + 1e-7,
                   observed_step_mm=dz, note='Synthetic planner call only; not an accepted 5-second-old receiver request.')

        request = dict(kind=KIND, model=MODEL_ID, id=1, created=10., arms={}, lift=1.,
                       lift_mode=LIFT_MODE, lift_epoch=1)
        for age in (0., .249, .25):
            self.check(f'lease: fresh request age {age:g}s accepted', validate({}, request, 10.+age) == {})
        for age in (.250001, .3, -.001):
            self.rejects(f'lease: request age {age:g}s rejected', lambda age=age:validate({}, request, 10.+age))

    def result(self):
        return dict(schema_version=1, scope='Historical incremental planner mathematics plus current protocol lease checks only; not current preset runtime validation. No ROS, robot, or network.',
                    configuration=self.model.config, travel_mm=self.model.travel,
                    source_sha256={path:digest(path) for path in (
                        'tools/verify_pico_lift_offline.py',
                        'scripts/pico4/fairino/vr_lift.py', 'scripts/pico4/fairino/vr_core.py',
                        'scripts/pico4/fairino/vr_robot.py', 'apps/fairino_upper_probe/body_control/body_core.py',
                        'apps/fairino_upper_probe/body_control/body_config.json')},
                    metric_definition={
                        'velocity':'Delta joint target / actual synthetic inter-arrival time, in degrees/s.',
                        'acceleration':'Difference of secant velocities / separation of velocity midpoint times.',
                        'jerk':'Difference of equivalent accelerations / separation of acceleration midpoint times.',
                        'not_physical':'These finite differences are not measured motor velocity, acceleration, jerk or vibration.',
                        'arrival':'Schedules represent fresh accepted targets. They do not benchmark achieved controller/CPU rates.',
                    },
                    status='fail' if any(c['status']=='fail' for c in self.checks) else 'checks_pass_with_design_gaps',
                    counts=dict(passed=sum(c['status']=='pass' for c in self.checks),
                                failed=sum(c['status']=='fail' for c in self.checks), scenarios=len(self.cases)),
                    checks=self.checks, scenarios=self.cases,
                    design_gaps=[
                        dict(id='no_acceleration_or_jerk_limit', status='confirmed_design_gap',
                             finding='LiftPlanner limits height speed and per-step joint displacement, not acceleration or jerk. '
                                     'Start/reverse/neutral can change target secant velocity in one update.',
                             evidence_cases=['startup_450hz', 'reversal_450hz', 'pause_resume_450hz']),
                        dict(id='irregular_target_steps', status='confirmed_design_gap',
                             finding='Elapsed-time compensation preserves integrated height for intervals <=50ms, '
                                     'but longer intervals produce larger target steps. Gaps >50ms lose motion rather than catch up.',
                             evidence_cases=['fixed_450hz','jitter_2_2_8ms','gap_20ms','gap_50ms','gap_80ms']),
                        dict(id='position_only_receiver', status='source_confirmed_not_hardware_validated',
                             finding='VRProbe.publish_positions sends a single t=0 position point with empty velocity/acceleration. '
                                     'This script does not simulate JTC, servo dynamics, hardware tracking or chassis mechanics.',
                             source='scripts/pico4/fairino/vr_robot.py:118'),
                    ],
                    not_verified=['Physical cause of growing vibration', 'Actual hardware acceleration/jerk and tracking',
                                  'Real-time rate, network latency and buffering', 'Mechanical compliance, load and drive parameters',
                                  'Receiver stop execution on real hardware; existing mocked receiver tests cover software behavior'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'logs/20261026-lift-offline-verification')
    args = parser.parse_args()
    verification = Verification()
    verification.run()
    result = verification.result()
    args.output.mkdir(parents=True, exist_ok=True)
    path = args.output/'pico-lift.json'
    path.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    with (args.output/'pico-lift-traces.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(verification.traces[0]))
        writer.writeheader()
        writer.writerows(verification.traces)
    print(json.dumps(dict(status=result['status'], **result['counts'], report=str(path)), ensure_ascii=False))
    return int(result['counts']['failed'] > 0)


if __name__ == '__main__':
    raise SystemExit(main())
