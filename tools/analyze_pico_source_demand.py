#!/usr/bin/env python3
"""Offline source-demand evidence; imports only pure mathematical helpers.

Neither adjacent planned targets nor their derivatives are motor measurements.
The 100 Hz zero-order hold uses the same archive time base as the replay.
"""
from __future__ import annotations

import argparse
import bisect
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "scripts/pico4/fairino"), str(ROOT / "apps/fairino_tcp_probe")]
from mounts import tcp_in_chest
from tcp_core import tcp_forward_matrix

RELEASE = {"grip_released", "waiting_for_stationary_or_full_grip", "input_paused"}


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: plain(v) for key, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    return value


def stats(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return {"n": 0}
    return dict(n=len(a), p50=np.percentile(a, 50), p95=np.percentile(a, 95),
                p99=np.percentile(a, 99), maximum=a.max(), mean=a.mean())


def pose_distance(a, b):
    mm = np.linalg.norm(a[:3, 3] - b[:3, 3])
    c = np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1) * .5, -1., 1.)
    return float(mm), float(np.degrees(np.arccos(c)))


def differences(rows, *, nominal_dt=None):
    output = []
    previous = {}
    for row in rows:
        for side, item in row["arms"].items():
            old = previous.get(side)
            if old and old["segment"] == row["segment"]:
                dt = nominal_dt if nominal_dt is not None else row["t"] - old["t"]
                if .001 <= dt <= .05:
                    mm, deg = pose_distance(old["arm"]["goal"], item["goal"])
                    dq = np.degrees(item["target"] - old["arm"]["target"])
                    output.append(dict(t=row["t"], side=side, segment=row["segment"],
                        line=row["line"], previous_line=old["line"], dt=dt,
                        translation_step_mm=mm, rotation_step_deg=deg,
                        translation_speed_mm_s=mm/dt, rotation_speed_deg_s=deg/dt,
                        planned_joint_velocity_deg_s=dq/dt,
                        max_planned_joint_speed_deg_s=max(abs(dq/dt))))
            previous[side] = dict(t=row["t"], line=row["line"], segment=row["segment"], arm=item)
        for side in list(previous):
            if side not in row["arms"]:
                del previous[side]
    return output


def difference_summary(rows):
    out = {}
    for side in sorted(set(row["side"] for row in rows)):
        subset = [row for row in rows if row["side"] == side]
        out[side] = {key: stats([row[key] for row in subset]) for key in (
            "translation_step_mm", "rotation_step_deg", "translation_speed_mm_s",
            "rotation_speed_deg_s", "max_planned_joint_speed_deg_s")}
        out[side]["fraction_old_joint_plan_over_10_deg_s"] = np.mean([r["max_planned_joint_speed_deg_s"] > 10 for r in subset])
        out[side]["fraction_old_joint_plan_over_30_deg_s"] = np.mean([r["max_planned_joint_speed_deg_s"] > 30 for r in subset])
        out[side]["orientation_demand_over_global_70_deg_s"] = np.mean([r["rotation_speed_deg_s"] > 70 for r in subset])
        out[side]["planned_joint_velocity_abs_per_axis"] = [stats([abs(r["planned_joint_velocity_deg_s"][axis]) for r in subset]) for axis in range(7)]
        out[side]["largest_translation_steps"] = sorted(subset, key=lambda r: r["translation_step_mm"], reverse=True)[:5]
        out[side]["largest_rotation_steps"] = sorted(subset, key=lambda r: r["rotation_step_deg"], reverse=True)[:5]
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stem", default="20261020-205643")
    parser.add_argument("--start", type=float, default=1792501011.6122057)
    parser.add_argument("--duration", type=float, default=20.)
    parser.add_argument("--source", type=Path, default=ROOT / ".local/pico4/fairino-logs")
    parser.add_argument("--output", type=Path, default=ROOT / "logs/20261026-native-pico-history/error_analysis/source_demand")
    args = parser.parse_args()
    start, end = args.start, args.start + args.duration
    events, rows, releases = [], [], []
    segment, tools, all_events = 0, {}, Counter()
    last_goal_sides = set()
    for number, line in enumerate((args.source / (args.stem + ".actions.jsonl")).open(), 1):
        row = json.loads(line)
        t = row.get("local_time")
        if row.get("event") == "tcp_tool_config":
            tools = row
        if t is None or t < start:
            continue
        if t > end:
            break
        event = row.get("event")
        if event:
            all_events[event] += 1
            if event in RELEASE:
                segment += 1
                item = dict(t=t-start, line=number, segment=segment, arms={}, event=event,
                            reason=row.get("reason"))
                events.append(item)
                releases.append(item)
                last_goal_sides = set()
            continue
        arms = row.get("request", {}).get("arms", {})
        goals = row.get("tcp_chest_mm") or {}
        if not goals:
            continue
        if set(goals) != last_goal_sides:
            segment += 1
        last_goal_sides = set(goals)
        item = dict(t=t-start, line=number, segment=segment, arms={}, event="goals",
                    vr_stamp=row.get("vr_stamp"), plan_monotonic=row.get("plan_monotonic"),
                    sensitivity=row.get("sensitivity"))
        for side, value in goals.items():
            arm = arms[side]
            item["arms"][side] = dict(goal=np.asarray(value).reshape(4,4), target=np.array(arm["target"]),
                old_solved_tcp=tcp_in_chest(side, np.asarray(arm["tcp"]).reshape(4,4)),
                scale=arm.get("wrist_scale", 1.))
        rows.append(item)
        events.append(item)
    times = [r["t"] for r in events]
    sampled = []
    for n in range(round(args.duration / .01) + 1):
        t = n * .01
        i = bisect.bisect_right(times, t) - 1
        row = events[i] if i >= 0 else None
        if row is None or t-row["t"] > .15:
            sampled.append(dict(t=t, line=None, segment=-1, arms={}))
        else:
            sampled.append({**row, "t": t})
    raw_differences = differences(rows)
    sampled_differences = differences(sampled, nominal_dt=.01)
    measured, old_tracking = [], []
    for number, line in enumerate((args.source / (args.stem + ".jsonl")).open(), 1):
        row = json.loads(line); t = row.get("local_time", 0.)
        if t < start:
            continue
        if t > end:
            break
        hw = row.get("hardware")
        if not hw:
            continue
        measured.append(dict(t=t-start, line=number, velocity_deg_s=np.degrees(hw["velocity"][:14]),
            position_deg=np.degrees(hw["position"][:14]), command_deg=np.degrees(hw["command_position"][:14]),
            hardware_time=row.get("stamp", float("nan"))-row.get("hw_age", float("nan")),
            hw_age_ms=row.get("hw_age", float("nan"))*1000))
        i = bisect.bisect_right(times, t-start)-1
        goal = events[i] if i >= 0 else None
        if not goal or t-start-goal["t"] > .15:
            continue
        for side, arm in goal["arms"].items():
            offset = 0 if side == "left" else 7
            fk = np.asarray(tcp_forward_matrix(hw["position"][offset:offset+7]))
            tool = np.asarray(tools["transforms"][side])
            actual = tcp_in_chest(side, fk @ tool)
            mm, deg = pose_distance(actual, arm["goal"])
            old_tracking.append(dict(t=t-start, line=number, goal_line=goal["line"], side=side,
                position_error_mm=mm, orientation_error_deg=deg))
    joint_ranges = {}
    for side in sorted(set(side for r in rows for side in r["arms"])):
        q = np.degrees([r["arms"][side]["target"] for r in rows if side in r["arms"]])
        offset = 0 if side == "left" else 7
        v = np.asarray([r["velocity_deg_s"][offset:offset+7] for r in measured])
        actualq = np.asarray([r["position_deg"][offset:offset+7] for r in measured])
        command_rates = []
        last_hw = None
        for record in measured:
            if last_hw is not None:
                delta_time = record["hardware_time"] - last_hw["hardware_time"]
                if abs(delta_time) <= 1e-6:
                    continue
                if .001 <= delta_time <= .4:
                    command_rates.append((record["command_deg"][offset:offset+7]
                                          - last_hw["command_deg"][offset:offset+7]) / delta_time)
            last_hw = record
        command_rates = np.asarray(command_rates)
        joint_ranges[side] = dict(old_target_min_deg=q.min(axis=0), old_target_max_deg=q.max(axis=0),
            old_target_excursion_deg=np.ptp(q,axis=0), measured_min_deg=actualq.min(axis=0),
            measured_max_deg=actualq.max(axis=0), measured_max_abs_speed_deg_s=np.max(abs(v),axis=0),
            measured_peak_axis_speed=stats(np.max(abs(v),axis=1)),
            command_feedback_derived_peak_axis_speed=stats(np.max(abs(command_rates),axis=1)),
            command_feedback_derived_max_abs_speed_deg_s=np.max(abs(command_rates),axis=0),
            measured_fraction_any_axis_over_10_deg_s=np.mean(np.max(abs(v),axis=1)>10),
            measured_fraction_any_axis_over_30_deg_s=np.mean(np.max(abs(v),axis=1)>30))
    goal_solved = [pose_distance(a["goal"],a["old_solved_tcp"]) for r in rows for a in r["arms"].values()]
    sensitivity_segments = []
    for segment_id in sorted(set(r["segment"] for r in rows)):
        segment_rows = [r for r in rows if r["segment"] == segment_id]
        sensitivity_segments.append(dict(start=segment_rows[0]["t"], end=segment_rows[-1]["t"],
            first_line=segment_rows[0]["line"], last_line=segment_rows[-1]["line"],
            sensitivity_counts=Counter(str(r["sensitivity"]) for r in segment_rows)))
    summary = dict(stem=args.stem,start=start,duration=args.duration,source_goal_rows=len(rows),
        raw_derivative_clock="archive local_time (PC logging wall clock), matching replay scheduling; differs from old audit's plan_monotonic derivatives",
        sides=Counter(side for r in rows for side in r["arms"]), tools=tools,
        event_counts=all_events,release_events=releases,
        sensitivity_segments=sensitivity_segments,
        goals_with_wrist_scale_below_one=sum(a["scale"]<1 for r in rows for a in r["arms"].values()),
        old_solved_vs_original_goal=dict(position_error_mm=stats([p[0] for p in goal_solved]),orientation_error_deg=stats([p[1] for p in goal_solved])),
        adjacent_source_demand=difference_summary(raw_differences),
        resampled_100hz_demand=difference_summary(sampled_differences),
        joint_ranges_and_recorded_measured_speed=joint_ranges,
        old_feedback_asof_logged_goal=dict(records=len(old_tracking),
            position_error_mm=stats([r["position_error_mm"] for r in old_tracking]),
            orientation_error_deg=stats([r["orientation_error_deg"] for r in old_tracking]),
            largest_position_errors=sorted(old_tracking,key=lambda r:r["position_error_mm"],reverse=True)[:5],
            largest_orientation_errors=sorted(old_tracking,key=lambda r:r["orientation_error_deg"],reverse=True)[:5]),
        limitations=[
            "Adjacent old planned-joint differences describe one old IK branch, not a necessary lower bound across all redundant IK solutions or measured motor speed.",
            "Instantaneous differences use archive PC receipt/planning time and can contain packet/sampling jitter; 100Hz held-goal differences are also supplied.",
            "Explicit release/input-pause boundaries and changing active-arm sets reset derivatives; derivative pairs with dt outside 1..50ms are excluded.",
            "Archived measured velocity is hardware-reported speed, repeated across status messages; old feedback tracking compares its FK to the latest available PC goal, so includes communication timing.",
            "Command feedback derivatives use changes of recorded command_position at distinct inferred server hardware receive times (status stamp minus hw_age), not a command_velocity signal or guaranteed motor speed.",
            "With seven revolute axes each limited to10deg/s, endpoint angular speed cannot exceed70deg/s in any configuration, but direction-dependent limits can be lower; exceeding this is necessary evidence, not a full cause attribution.",
            "A successful old IK and safe joint position do not guarantee that the new time-bounded local QP can reach it at the demanded time.",
            "The second and third right_wrist_flange active segments used archived sensitivity1.0 versus the new client's default0.25. Holding reference and controller-relative motion equal gives4x translation and rotation-vector amplitude. These absolute archived TCPs are not a faithful reconstruction of new default-sensitivity closed-loop PICO operation.",
        ])
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output / "summary.json").write_text(json.dumps(plain(summary),ensure_ascii=False,indent=2)+"\n")
    for name, data in (("source_differences",raw_differences),("demand_100hz",sampled_differences),("old_tracking",old_tracking),("measured",measured)):
        with (args.output/(name+".jsonl")).open("w") as out:
            for record in data:
                out.write(json.dumps(plain(record),ensure_ascii=False)+"\n")
    print(json.dumps(plain({"output":str(args.output),"source_goals":len(rows),"measured_records":len(measured),"joint_ranges":joint_ranges}),ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
