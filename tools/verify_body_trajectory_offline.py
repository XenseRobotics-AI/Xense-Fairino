#!/usr/bin/env python3
"""Offline torso trajectory audit. Imports no ROS or robot/network transport.

Uses an independent power-basis quintic solution, rather than the production
Bezier evaluator, to inspect recorded and synthetic joint trajectories. All
reported geometric errors are model-internal, not physical robot accuracy.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / 'apps/fairino_upper_probe/body_control'
sys.path.insert(0, str(CORE))
from body_core import BodyModel, ease, load_config, plan, sample_interval  # noqa: E402

DEG = 180.0 / math.pi
MOVING = tuple(range(15, 20))
FOLD = (17, 18, 19)
FIELDS = ('height', 'forward', 'roll', 'yaw')
HERMITE_MATRIX = np.array([[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]])


def power_coefficients(left, right):
    """Solve six endpoint conditions in normalized time u, independently."""
    h = right['time'] - left['time']
    if not math.isfinite(h) or h <= 0:
        raise ValueError('trajectory times must strictly increase')
    q0, q1 = (np.asarray(p['positions'], dtype=float) for p in (left, right))
    v0, v1 = (np.asarray(p['velocities'], dtype=float) for p in (left, right))
    a0, a1 = (np.asarray(p['accelerations'], dtype=float) for p in (left, right))
    c = np.empty((20, 6))
    c[:, 0], c[:, 1], c[:, 2] = q0, h * v0, .5 * h * h * a0
    residual = np.vstack((q1 - q0 - c[:, 1] - c[:, 2],
                          h * v1 - c[:, 1] - 2 * c[:, 2],
                          h * h * a1 - 2 * c[:, 2]))
    c[:, 3:] = np.linalg.solve(HERMITE_MATRIX, residual).T
    return c, h


def derivative(c, order, h):
    out = c.copy()
    for _ in range(order):
        out = out[:, 1:] * np.arange(1, out.shape[1])
    return out / h ** order


def evaluate(c, u):
    u = np.asarray(u, dtype=float)
    result = np.zeros((u.size, c.shape[0]))
    for k in range(c.shape[1] - 1, -1, -1):
        result = result * u.reshape(-1, 1) + c[:, k]
    return result


def scalar_peak(c):
    """Magnitude extremum on [0,1], including roots of the derivative."""
    dc = c[1:] * np.arange(1, len(c))
    while len(dc) > 1 and abs(dc[-1]) < 1e-14 * max(1., float(np.max(np.abs(dc)))):
        dc = dc[:-1]
    candidates = [0., 1.]
    if len(dc) > 1:
        for root in np.polynomial.polynomial.polyroots(dc):
            if abs(root.imag) < 1e-7 and -1e-10 <= root.real <= 1. + 1e-10:
                candidates.append(float(np.clip(root.real, 0., 1.)))
    values = np.polynomial.polynomial.polyval(candidates, c)
    at = int(np.argmax(np.abs(values)))
    return float(abs(values[at])), candidates[at]


def stage_nodes(points, number):
    return points[number * 100:number * 100 + 101]


def state_fields(model, q):
    s = model.state(list(q))
    return {key: s[key] for key in FIELDS}


def analytic_stage(model, label, nodes):
    """Reconstruct geometric or joint-alignment path from its endpoints."""
    first, last = nodes[0], nodes[-1]
    q0, q1 = np.asarray(first['positions']), np.asarray(last['positions'])
    duration = last['time'] - first['time']
    geometric = label in ('升降与姿态调节', '到达高位后侧倾')
    start, end = state_fields(model, q0), state_fields(model, q1)
    diff = {key: end[key] - start[key] for key in FIELDS}
    dz = diff['height'] / 100 * model.travel

    def at(normalized):
        u, du, ddu = ease(normalized)
        if not geometric:
            delta = q1 - q0
            return q0 + delta * u, delta * du / duration, delta * ddu / duration ** 2
        state = {key: start[key] + diff[key] * u for key in FIELDS}
        q, qz, qzz = model.curve(state, q0)
        qs = np.asarray(qz) * dz
        qss = np.asarray(qzz) * dz * dz
        qs[15], qs[16] = math.radians(diff['yaw']), math.radians(diff['roll'])
        qs[17] += math.radians(diff['forward'])
        return (np.asarray(q), qs * du / duration,
                (qss * du * du + qs * ddu) / duration ** 2)

    return at, geometric, start, end


def fk_many(model, q):
    lower, knee = q[:, 19], q[:, 18]
    x = model.l1 * np.sin(lower) + model.l2 * np.sin(lower + knee)
    z = model.l1 * np.cos(lower) + model.l2 * np.cos(lower + knee)
    return np.column_stack((z - model.z_low, x - model.x,
                            q[:, 17:20].sum(axis=1) * DEG - model.neutral,
                            q[:, 16] * DEG, q[:, 15] * DEG))


def sample_power(nodes, relative_times):
    times = np.asarray([p['time'] for p in nodes])
    absolute = relative_times + times[0]
    index = np.clip(np.searchsorted(times, absolute, side='right') - 1, 0, len(nodes) - 2)
    outputs = [np.empty((len(relative_times), 20)) for _ in range(3)]
    for i in np.unique(index):
        selected = index == i
        c, h = power_coefficients(nodes[i], nodes[i + 1])
        u = np.clip((absolute[selected] - times[i]) / h, 0., 1.)
        for order, out in enumerate(outputs):
            out[selected] = evaluate(derivative(c, order, h), u)
    return outputs


def denser_nodes(at, duration, count):
    nodes = []
    for i in range(count + 1):
        q, v, a = at(i / count)
        nodes.append(dict(time=duration * i / count,
                          positions=q.tolist(), velocities=v.tolist(), accelerations=a.tolist()))
    return nodes


def analyze_stage(model, label, nodes, hz, dense_count):
    at, geometric, start, end = analytic_stage(model, label, nodes)
    duration = nodes[-1]['time'] - nodes[0]['time']
    # A 500 Hz grid plus endpoint: this is an offline grid, not measured timing.
    times = np.arange(0., duration, 1. / hz)
    if not len(times) or duration - times[-1] > 1e-10:
        times = np.append(times, duration)
    else:
        times[-1] = duration
    interpolated = sample_power(nodes, times)
    exact = [np.empty_like(interpolated[0]) for _ in range(3)]
    for i, t in enumerate(times):
        values = at(min(1., t / duration))
        for order in range(3):
            exact[order][i] = values[order]
    dense = sample_power(denser_nodes(at, duration, dense_count), times)
    node_match = [0., 0., 0.]
    for p in nodes:
        expected = at((p['time'] - nodes[0]['time']) / duration)
        for order, field in enumerate(('positions', 'velocities', 'accelerations')):
            node_match[order] = max(node_match[order], float(np.max(np.abs(expected[order] - p[field]))))
    # Check analytic derivatives independently from positions, with a five-point
    # finite difference in normalized time. Two step sizes expose cancellation.
    finite_difference = []
    for ds in (.001, .0005):
        derivative_errors = np.zeros(2)
        for s in (.1, .3, .5, .7, .9):
            qm2, qm1, q0, qp1, qp2 = [at(s+k*ds)[0] for k in (-2,-1,0,1,2)]
            fd_v = (qm2-8*qm1+8*qp1-qp2)/(12*ds*duration)
            fd_a = (-qp2+16*qp1-30*q0+16*qm1-qm2)/(12*(ds*duration)**2)
            expected = at(s)
            for order, numeric in enumerate((fd_v, fd_a)):
                derivative_errors[order] = max(derivative_errors[order],
                    float(np.max(np.abs(numeric-expected[order+1]))))
        finite_difference.append(dict(normalized_step=ds,
                                      max_velocity_error_rad_s=float(derivative_errors[0]),
                                      max_acceleration_error_rad_s2=float(derivative_errors[1])))
    maxima = np.zeros((3, 20))
    location = {}
    crosscheck = [0., 0.]
    continuity = np.zeros(3)
    jerk_jump = np.zeros(20)
    previous_end = None
    endpoint_jerks = []
    all_finite = all(np.isfinite(array).all() for array in (*interpolated, *exact, *dense))
    for i, (left, right) in enumerate(zip(nodes, nodes[1:])):
        c, h = power_coefficients(left, right)
        ends = [evaluate(derivative(c, k, h), [0., 1.]) for k in range(4)]
        if previous_end is not None:
            for order in range(3):
                continuity[order] = max(continuity[order], float(np.max(np.abs(ends[order][0] - previous_end[order]))))
            jerk_jump = np.maximum(jerk_jump, np.abs(ends[3][0] - previous_end[3]))
        previous_end = [array[1] for array in ends]
        if i == 0:
            endpoint_jerks.append(ends[3][0])
        if i == len(nodes) - 2:
            endpoint_jerks.append(ends[3][1])
        for order in range(1, 4):
            dc = derivative(c, order, h)
            for joint in MOVING:
                magnitude, where = scalar_peak(dc[joint])
                if magnitude > maxima[order - 1, joint]:
                    maxima[order - 1, joint] = magnitude
                    location[f'J{joint+1}_order{order}'] = left['time'] + h * where
        for u in (.13, .37, .73):
            q, v = sample_interval(left, right, left['time'] + u * h)
            for order, legacy in enumerate((q, v)):
                independent = evaluate(derivative(c, order, h), [u])[0]
                crosscheck[order] = max(crosscheck[order], float(np.max(np.abs(independent - legacy))))
    fk = fk_many(model, interpolated[0])
    expected_fk = fk_many(model, exact[0])
    height_bounds = (min(start['height'], end['height']) / 100 * model.travel,
                     max(start['height'], end['height']) / 100 * model.travel)
    height_excess = max(0., float(height_bounds[0] - fk[:, 0].min()),
                        float(fk[:, 0].max() - height_bounds[1]))
    result = dict(label=label, geometric_vertical_stage=geometric, duration_s=duration,
                  intervals=len(nodes)-1, node_spacing_s=duration/(len(nodes)-1),
                  offline_samples=len(times), nominal_500hz_cycles=duration*500.,
                  finite=bool(all_finite), reconstruction_node_error_rad_rad_s_rad_s2=node_match,
                  geometry_position_finite_difference_checks=finite_difference,
                  power_vs_production_bezier_max_error_rad_rad_s=crosscheck,
                  continuity_max_error_rad_rad_s_rad_s2=continuity.tolist(),
                  peak_speed_deg_s=(maxima[0]*DEG).tolist(),
                  peak_acceleration_deg_s2=(maxima[1]*DEG).tolist(),
                  peak_jerk_deg_s3=(maxima[2]*DEG).tolist(),
                  peak_locations_trajectory_seconds=location,
                  interior_jerk_jump_deg_s3=(jerk_jump*DEG).tolist(),
                  start_jerk_deg_s3=(endpoint_jerks[0]*DEG).tolist(),
                  end_jerk_deg_s3=(endpoint_jerks[1]*DEG).tolist(),
                  interpolation_error_deg_deg_s_deg_s2=[
                      (np.max(np.abs(a-b), axis=0)*DEG).tolist()
                      for a, b in zip(interpolated, exact)],
                  denser_interpolation_error_deg_deg_s_deg_s2=[
                      (np.max(np.abs(a-b), axis=0)*DEG).tolist()
                      for a, b in zip(dense, exact)],
                  fixed_vs_denser_max_position_deg=float(np.max(np.abs(interpolated[0]-dense[0]))*DEG),
                  fk_error_height_mm_horizontal_mm_pitch_roll_yaw_deg=np.max(np.abs(fk-expected_fk),axis=0).tolist(),
                  vertical_height_envelope_excess_mm=height_excess if geometric else None,
                  vertical_horizontal_deviation_mm=float(np.max(np.abs(fk[:,1]))) if geometric else None,
                  held_nonbody_axis_max_error_rad=float(np.max(np.abs(interpolated[0][:,:15]-nodes[0]['positions'][:15]))),
                  start_body_state=start, end_body_state=end,
                  alignment_geometry_note=None if geometric else
                      'Joint-space alignment/hold is intentionally not required to follow a vertical Cartesian line.')
    return result


def analyze_plan(name, model, planned, hz, dense_count):
    points = planned['points']
    stages = [analyze_stage(model, label, stage_nodes(points, i), hz, dense_count)
              for i, label in enumerate(planned['stages'])]
    failures = []
    if len(points) != 100*len(stages)+1:
        failures.append('unexpected concatenated node count')
    boundary_jumps = []
    for i, stage in enumerate(stages):
        prefix = f'{i+1}:{stage["label"]}'
        if not stage['finite']:
            failures.append(prefix+': non-finite')
        for name_, tolerance in (
            ('reconstruction_node_error_rad_rad_s_rad_s2', 1e-6),
            ('power_vs_production_bezier_max_error_rad_rad_s', 1e-7),
            ('continuity_max_error_rad_rad_s_rad_s2', 1e-6)):
            if max(stage[name_]) > tolerance:
                failures.append(prefix+': '+name_)
        if max(stage['peak_speed_deg_s']) > planned['speed_deg_s']+1e-6:
            failures.append(prefix+': speed bound exceeded')
        if stage['held_nonbody_axis_max_error_rad'] > 1e-10:
            failures.append(prefix+': unrelated held joint changed')
        if not any(check['max_velocity_error_rad_s'] < 1e-6
                   and check['max_acceleration_error_rad_s2'] < 1e-4
                   for check in stage['geometry_position_finite_difference_checks']):
            failures.append(prefix+': analytic derivatives disagree with position finite difference')
        if stage['geometric_vertical_stage']:
            if stage['vertical_height_envelope_excess_mm'] > 1e-5:
                failures.append(prefix+': height envelope exceeded')
            if stage['vertical_horizontal_deviation_mm'] > 1e-5:
                failures.append(prefix+': vertical line deviation')
            if max(stage['fk_error_height_mm_horizontal_mm_pitch_roll_yaw_deg'][2:]) > 1e-7:
                failures.append(prefix+': attitude mismatch')
        for end in (0, 100):
            node = stage_nodes(points, i)[end]
            if max(abs(x) for x in node['velocities']+node['accelerations']) > 1e-8:
                failures.append(prefix+': stage endpoint v/a not zero')
        preceding = np.zeros(20) if i == 0 else np.asarray(stages[i-1]['end_jerk_deg_s3'])
        boundary_jumps.append(dict(boundary='rest_to_stage' if i == 0 else 'between_stages',
                                   time_s=points[100*i]['time'],
                                   delta_jerk_deg_s3=(np.asarray(stage['start_jerk_deg_s3'])-preceding).tolist()))
    boundary_jumps.append(dict(boundary='stage_to_rest',time_s=points[-1]['time'],
                               delta_jerk_deg_s3=(-np.asarray(stages[-1]['end_jerk_deg_s3'])).tolist()))
    return dict(name=name, passed=not failures, failures=failures,
                speed_limit_deg_s=planned['speed_deg_s'], duration_s=planned['duration'],
                initial_first_node_s=points[0]['time'], first_node_transition_excluded=True,
                node_count=len(points), stage_count=len(stages),
                max_speed_deg_s=max(max(s['peak_speed_deg_s']) for s in stages),
                max_acceleration_deg_s2=max(max(s['peak_acceleration_deg_s2']) for s in stages),
                max_jerk_deg_s3=max(max(s['peak_jerk_deg_s3']) for s in stages),
                max_model_position_error_deg=max(max(s['interpolation_error_deg_deg_s_deg_s2'][0]) for s in stages),
                max_fixed_vs_denser_position_deg=max(s['fixed_vs_denser_max_position_deg'] for s in stages),
                stage_boundary_jerk_jumps=boundary_jumps, stages=stages)


def state(height, forward=0., roll=0., yaw=0.):
    return dict(height=float(height),forward=float(forward),roll=float(roll),yaw=float(yaw))


def synthetic_cases(model):
    entries = []
    for a,b in ((0,50),(50,100),(100,50),(50,0),(0,100),(100,0),
                (99.9,100),(100,99.9),(0,.1),(.1,0),(50,50.01),(50.01,50),(50,50)):
        entries.append((f'height_{a}_to_{b}',state(a),state(b)))
    entries.extend([
        ('pitch_at_mid',state(50),state(50,45)),
        ('lift_with_pitch',state(20),state(80,30)),
        ('descend_remove_pitch',state(80,30),state(20)),
        ('yaw_sweep',state(50,yaw=-90),state(50,yaw=90)),
        ('roll_high',state(100,roll=-15),state(100,roll=15)),
        ('descend_unroll',state(100,roll=15),state(50)),
        ('ascend_then_roll',state(50),state(100,roll=-15)),
        ('roll_gate_edge',state(90,5,15),state(89.9,5)),
        ('compound_lift_yaw',state(10,yaw=-30),state(95,4,10,30)),
    ])
    hold = [.013*(i+1) for i in range(20)]
    for name, start, goal in entries:
        actual = model.pose(start,hold)
        yield name, actual, actual[:], goal, 5.
    for speed in (.5, 10.):
        actual = model.pose(state(99.9),hold)
        yield f'tiny_high_speed_{speed}',actual,actual[:],state(100),speed
    actual = model.pose(state(50),hold)
    actual[17] += math.radians(1.)
    actual[19] -= math.radians(1.)
    yield 'horizontal_off_path_alignment',actual,actual[:],state(70),5.
    actual = model.pose(state(50),hold)
    actual[16] = math.radians(3.)  # Measured pose may violate planned roll gate.
    actual[17] += math.radians(1.)
    actual[19] -= math.radians(1.)  # Also require a separate path-alignment stage.
    yield 'four_stages_from_measured_roll',actual,actual[:],state(100,roll=10),5.


def read_first_historical_plan(path):
    with path.open() as stream:
        for line_number,line in enumerate(stream,1):
            row=json.loads(line)
            if 'body_plan' in row:
                return row['body_plan'],line_number
    raise ValueError('no body_plan event')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--history',type=Path,default=CORE/'logs/20261023-171132.jsonl')
    parser.add_argument('--output',type=Path,default=ROOT/'logs/20261026-lift-offline-verification/body-trajectory.json')
    parser.add_argument('--hz',type=float,default=500.)
    parser.add_argument('--dense-intervals',type=int,default=400)
    args=parser.parse_args()
    if not math.isfinite(args.hz) or args.hz<=0 or args.dense_intervals<100:
        parser.error('positive sample frequency and at least 100 dense intervals required')
    model=load_config()
    history,line=read_first_historical_plan(args.history)
    reference_error=max(abs(float(a)-float(b)) for name,ref in model.references().items()
                        for a,b in [(ref['height_mm'],history['references'][name]['height_mm']),
                                    *zip(ref['fold_joint_deg'],history['references'][name]['fold_joint_deg'])])
    results=[]
    historical_result=analyze_plan('historical_20261023_171132',model,history,args.hz,args.dense_intervals)
    if reference_error>1e-9:
        historical_result['failures'].append('current model references do not match historical plan')
        historical_result['passed']=False
    results.append(historical_result)
    for name,actual,hold,goal,speed in synthetic_cases(model):
        try:
            planned=plan(model,actual,hold,goal,speed)
            results.append(analyze_plan(name,model,planned,args.hz,args.dense_intervals))
        except Exception as exc:
            results.append(dict(name=name,passed=False,failures=[f'{type(exc).__name__}: {exc}']))
    rejected=[]
    for name,goal in [('height_below',state(-1)),('height_above',state(101)),
                      ('roll_below_gate',state(50,roll=1)),('pitch_above',state(50,46)),
                      ('yaw_above',state(50,yaw=91))]:
        try:
            q=model.pose(state(50),[0.]*20);plan(model,q,q,goal,5.)
        except ValueError as exc:
            rejected.append(dict(name=name,passed=True,message=str(exc)))
        else:
            rejected.append(dict(name=name,passed=False,message='invalid target was accepted'))
    failures=[dict(name=r['name'],failures=r.get('failures',[r.get('message')]))
              for r in results+rejected if not r['passed']]
    report=dict(schema=1,offline_only=True,python=platform.python_version(),numpy=np.__version__,
                source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in (CORE/'body_core.py',CORE/'body_config.json',Path(__file__))},
                history=dict(path=str(args.history),first_plan_line=line,
                             reference_error_mm_or_deg=reference_error),
                method=dict(interpolation='Independent normalized-time power basis, endpoint linear-system solve',
                            derivative_extrema='Endpoints and real roots of derivative polynomials',
                            sampled_geometry_hz=args.hz,dense_comparison_intervals=args.dense_intervals,
                            dense_comparison='Same endpoints, same physical duration; only knot density changes',
                            first_node_transition_excluded=True),
                summary=dict(trajectory_cases=len(results),invalid_goal_cases=len(rejected),
                             passed=sum(r['passed'] for r in results+rejected),
                             failed=len(failures),failures=failures,
                             trajectory_stage_count=sum(r.get('stage_count',0) for r in results),
                             max_model_position_error_deg=max(r.get('max_model_position_error_deg',0) for r in results),
                             max_fixed_vs_denser_position_deg=max(r.get('max_fixed_vs_denser_position_deg',0) for r in results)),
                limitations=[
                    'No robot or network connection. No motion, controller setting, production code or gain was changed.',
                    '500 Hz is an offline sampling grid based on configuration, not measured real-time scheduling.',
                    'Geometric errors compare two versions of the same model; they do not quantify physical robot accuracy.',
                    'Power-basis extrema use floating-point polynomial roots; geometry errors use a finite time grid.',
                    'The initial interval before the first 0.1-second node is not simulated.',
                    'Alignment stages intentionally permit non-vertical motion.',
                    'No physical acceleration or jerk threshold is assumed. Their peaks and discontinuities are diagnostic only.',
                    'No motor dynamics, inertia, friction, backlash, drive servo or mechanical compliance is simulated.',
                    'More knots can alter floating-point rounding and do not establish improved physical behavior.'],
                cases=results,invalid_goal_cases=rejected)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps(report['summary'],indent=2,ensure_ascii=False))
    print('Report:',args.output)
    return 1 if failures else 0


if __name__=='__main__':
    raise SystemExit(main())
