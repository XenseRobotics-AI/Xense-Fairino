#!/usr/bin/env python3
"""Offline Demo scan: stretch past reach, pull back, hold. Reports elbow branch flips.

Uses the real fairino_qp Demo engine and synthetic PICO frames; no robot or headset.
"""
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'scripts/pico4/fairino'), str(ROOT/'src'), str(ROOT)]
import paths  # noqa: E402,F401
import numpy as np  # noqa: E402
from tool_frames import tool_config  # noqa: E402
from vr_demo import DemoEngine  # noqa: E402

HZ = 100


def frame(t, grip, p):
    def hand():
        return dict(pose=[p[0], p[1], p[2], 0., 0., 0., 1.], grip=grip, trigger=0., trigger_available=True,
                    primary=False, secondary=False, axis_x=0., axis_y=0., axis_click=False, joystick_available=True)
    f = dict(stamp=int(t*1e9)+1, head=None, received_monotonic=t, left=hand(), right=hand())
    f['samples'] = [dict(f)]
    return f


def run(seed, tool, wall):
    rng = np.random.default_rng(seed)
    engine = DemoEngine(mode='first_person', sensitivity=1., tools=tool_config(tool), ik_backend='fairino_qp')
    t, rows, messages = 0., [], []

    def step(grip, position):
        nonlocal t
        t += 1/HZ
        start = time.perf_counter()
        state = engine.step(frame(t, grip, position), t)
        wall.append(1000*(time.perf_counter()-start))
        rows.append(np.degrees(np.array(engine.q)))
        messages.append(state['message'])

    for _ in range(5):
        step(0., (0., 0., 0.))
    q0, current, waypoints = rows[-1].copy(), np.zeros(3), []
    for _ in range(3):
        d = rng.normal(size=3)
        d[2] = -abs(d[2])-.3  # PICO -z is forward.
        waypoints.append(d/np.linalg.norm(d)*rng.uniform(.3, .5))
    waypoints.append(np.zeros(3))
    for target in waypoints:
        n = int(rng.uniform(.6, 1.5)*HZ)
        for i in range(n):
            u = (i+1)/n
            u = u*u*(3-2*u)
            step(1., tuple(current+(target-current)*u))
        current = target
    for _ in range(2*HZ):
        step(1., (0., 0., 0.))
    q = np.array(rows)
    return dict(seed=seed, elbow_min_deg=round(float(q[:, [3, 10]].min()), 1),
                arm_residual_deg=round(float(np.abs(q[-1, :14]-q0[:14]).max()), 1),
                max_arm_deg_s=round(float(np.abs(np.diff(q[:, :14], axis=0)).max()*HZ), 1),
                paused=next((m for m in messages if '暂停' in m), None))


def main():
    tool = os.getenv('TOOL', 'flange')
    wall = []
    out = [run(seed, tool, wall) for seed in range(int(os.getenv('RUNS', '12')))]
    print(json.dumps(dict(tool=tool, runs=len(out), elbow_flips=sum(o['elbow_min_deg'] < 0 for o in out),
                          paused=sum(bool(o['paused']) for o in out),
                          arms_back_within_1deg=sum(o['arm_residual_deg'] < 1 for o in out),
                          max_arm_deg_s=max(o['max_arm_deg_s'] for o in out),
                          cycle_ms_p50_p95_max=[round(float(x), 1) for x in np.percentile(wall, [50, 95, 100])]),
                     ensure_ascii=False))
    for o in out:
        print(json.dumps(o, ensure_ascii=False))


if __name__ == '__main__':
    main()
