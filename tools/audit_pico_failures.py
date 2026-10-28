#!/usr/bin/env python3
"""Read existing PICO logs without importing, contacting, or controlling a robot.

Regular high-rate status/target lines are scanned for metadata; event, fault and
terminal records are decoded in full. Counts are log records, never inferred
physical failures. Exit-log causes take precedence over the subsequent remote
SSH-disconnected cleanup message. No source logs are modified.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
from zoneinfo import ZoneInfo


EVENT = re.compile(rb'"event":\s*"([^"\\]+)"')
TIME = re.compile(rb'"local_time":\s*([0-9.]+)')
SCALE = re.compile(rb'"wrist_scale":\s*([-+0-9.eE]+)')
ERROR = re.compile(rb'"error":\s*"(?:[^"\\]|\\.)+"')
HW_AGE = re.compile(rb'"hw_age":\s*([-+0-9.eE]+)')
TZ = ZoneInfo("Asia/Hong_Kong")
REPLAY_WINDOWS = [
    ("20261022-103914", "first", 1792636771.3896909, 1792636801.3896909),
    ("20261022-103914", "boundary", 1792637364.6363294, 1792637406.0),
    ("20261023-145015", "tool", 1792738260.2717206, 1792738280.2717206),
    ("20261023-160547", "flange", 1792742890.4647837, 1792742910.4647837),
    ("20261022-144833", "near_wrist", 1792651725.089575, 1792651745.089575),
    ("20261020-205643", "mirror", 1792501011.6122057, 1792501031.6122057),
]


def evidence(path, line, row):
    return {"path": str(path), "line": line, **row}


def age_summary(values):
    values = sorted(values)
    if not values:
        return {"records": 0}
    def percentile(q):
        location = (len(values) - 1) * q
        lo = int(location)
        hi = min(lo + 1, len(values) - 1)
        return 1000 * (values[lo] + (values[hi] - values[lo]) * (location - lo))
    return {"records": len(values), "p50_ms": percentile(.5), "p95_ms": percentile(.95),
            "p99_ms": percentile(.99), "maximum_ms": 1000 * values[-1],
            "over_60ms_records": sum(v > .06 for v in values),
            "over_60ms_fraction": sum(v > .06 for v in values) / len(values)}


def category(reason, message):
    if reason in ("gui_stop", "keyboard_interrupt"):
        return "operator_stop"
    if "未确认新鲜双手柄" in message:
        return "startup_input_not_armed"
    if message.startswith("底盘控制："):
        if "底盘故障" in message:
            return "chassis_hardware_fault"
        if "往返通信超时" in message:
            return "chassis_ack_timeout"
        if "反馈" in message:
            return "chassis_feedback_timeout"
        if "回正期间" in message:
            return "chassis_wheel_motion_during_alignment"
        if "ROS" in message:
            return "chassis_ros_timeout"
        return "chassis_other"
    if message.startswith("夹爪控制："):
        return "gripper_stream_or_heartbeat_timeout"
    if re.match(r"J\d+ 速度", message):
        return "upper_held_speed_threshold"
    if "控制器未确认停止保持目标" in message:
        return "upper_stop_confirmation_timeout"
    if "关节尚未静止" in message:
        return "upper_not_stationary"
    if "上半身故障" in message:
        return "upper_hardware_fault"
    if "PICO 数据中断" in message:
        return "pico_stream_timeout"
    if "机器人状态反馈" in message:
        return "upper_client_feedback_timeout"
    if "SSH 往返通信超时" in message:
        return "upper_ack_timeout"
    if message == "ROS 请求超时":
        return "upper_ros_timeout"
    if message in ("SSH 已断开或用户退出", "窗口失焦或用户退出", "启动已取消"):
        return "remote_exit_trigger_unspecified"
    return "unclassified" if message else "no_recorded_trigger"


def scan_status(path):
    out = {"path": str(path), "records": 0, "errors": [], "fault_snapshots": [],
           "jog_rejected": [], "finished": [], "enabled_observed": False,
           "ready_observed": False, "selected_decode_errors": []}
    last_line = None
    last_stop = None
    active_ages = []
    all_enabled_ages = []
    active_over_60ms = []
    windows = {name: {"start_local_time": start, "end_local_time": end, "values": []}
               for stem, name, start, end in REPLAY_WINDOWS if stem == path.stem}
    for number, line in enumerate(path.open("rb"), 1):
        out["records"] = number
        match = TIME.search(line)
        if match:
            stamp = float(match[1])
            out.setdefault("first_local_time", stamp)
            out["last_local_time"] = stamp
        if not out["enabled_observed"] and b'"enabled": [true' in line:
            out["enabled_observed"] = True
        if b'"ready": true' in line:
            out["ready_observed"] = True
        age = HW_AGE.search(line)
        if age and b'"enabled": [true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true]' in line:
            all_enabled_ages.append(float(age[1]))
        if b'"jogging": true' in line and b'"ready": true' in line and age:
            value = float(age[1])
            active_ages.append(value)
            for window in windows.values():
                if window["start_local_time"] <= stamp <= window["end_local_time"]:
                    window["values"].append(value)
            if value > .06:
                active_over_60ms.append({"line": number, "local_time": stamp, "hw_age_ms": value * 1000})
        selected = (b'"stamp"' not in line or b'"finished"' in line
                    or b'"fault_snapshot"' in line or ERROR.search(line))
        if b'"stop_confirmation"' in line:
            last_stop = (number, line)
        last_line = (number, line)
        if not selected:
            continue
        try:
            row = json.loads(line)
        except (ValueError, UnicodeError) as exc:
            out["selected_decode_errors"].append({"line": number, "error": str(exc)})
            continue
        base = {"local_time": row.get("local_time")}
        if row.get("error"):
            item = evidence(path, number, {**base, "message": row["error"]})
            if last_stop:
                stop_row = json.loads(last_stop[1])
                item["preceding_stop_confirmation"] = evidence(
                    path, last_stop[0], {"local_time": stop_row.get("local_time"),
                                        "stop_confirmation": stop_row["stop_confirmation"]})
            out["errors"].append(item)
        if "fault_snapshot" in row:
            out["fault_snapshots"].append(evidence(path, number, row))
        if "jog_rejected" in row:
            out["jog_rejected"].append(evidence(path, number, {**base, "message": row["jog_rejected"]}))
        if row.get("finished"):
            out["finished"].append(evidence(path, number, {
                **base, "disabled_confirmed": row.get("disabled_confirmed"),
                "phase": row.get("phase")}))
    if last_line:
        try:
            row = json.loads(last_line[1])
            hw = row.get("hardware") or {}
            out["last_record"] = evidence(path, last_line[0], {
                "phase": row.get("phase"), "local_time": row.get("local_time"),
                "finished": row.get("finished"), "disabled_confirmed": row.get("disabled_confirmed"),
                "enabled_count": sum(hw.get("enabled", [])) if hw else None})
        except ValueError:
            pass
    out["active_feedback_age"] = age_summary(active_ages)
    out["all_enabled_feedback_age"] = age_summary(all_enabled_ages)
    out["active_feedback_over_60ms"] = active_over_60ms
    out["replay_window_feedback_age"] = {
        name: {"start_local_time": window["start_local_time"], "end_local_time": window["end_local_time"],
               **age_summary(window["values"])} for name, window in windows.items()}
    out["_active_ages"] = active_ages
    out["_all_enabled_ages"] = all_enabled_ages
    return out


def scan_actions(path):
    out = {"path": str(path), "events": Counter(), "teleop_records": 0,
           "original_tcp_records": 0, "wrist_scaled_records": 0, "minimum_wrist_scale": None,
           "pauses": [], "home_aborted": [], "first_teleop": None}
    for number, line in enumerate(path.open("rb"), 1):
        match = EVENT.search(line)
        if match:
            event = match[1].decode()
            out["events"][event] += 1
            if event in ("timing_config", "tcp_tool_config", "input_paused", "home_aborted"):
                row = json.loads(line)
                item = evidence(path, number, row)
                if event == "input_paused":
                    out["pauses"].append(item)
                elif event == "home_aborted":
                    out["home_aborted"].append(item)
                else:
                    out[event] = item
        elif b'"request"' in line:
            out["teleop_records"] += 1
            if b'"tcp_chest_mm"' in line:
                out["original_tcp_records"] += 1
            scales = [float(m[1]) for m in SCALE.finditer(line)]
            if scales:
                val = min(scales)
                old = out["minimum_wrist_scale"]
                out["minimum_wrist_scale"] = min(val, old) if old is not None else val
                if val < 1 - 1e-12:
                    out["wrist_scaled_records"] += 1
                    if "first_wrist_scaled" not in out:
                        out["first_wrist_scaled"] = evidence(path, number, {"local_time": float(TIME.search(line)[1]), "scale": val})
            if out["first_teleop"] is None:
                row = json.loads(line)
                out["first_teleop"] = evidence(path, number, {
                    key: row.get(key) for key in ("local_time", "plan_monotonic", "mapping", "mode", "rotation_mode")})
                out["first_teleop"]["reference"] = {
                    side: arm.get("reference") for side, arm in row["request"].get("arms", {}).items()}
    return out


def file_hash(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", action="append", type=Path)
    parser.add_argument("--output", type=Path, default=Path("logs/20261026-native-pico-history/audit"))
    args = parser.parse_args()
    roots = args.source or [Path(".local/pico4/fairino-logs"), Path("/home/lim/fairino-robot-toolkit/.local/pico4/fairino-logs")]
    files = []
    aliases = []
    seen = {}
    for root in roots:
        for path in sorted(root.glob("*.jsonl")):
            key = (path.name, path.stat().st_size)
            same = next((other for other in seen.get(key, []) if file_hash(other) == file_hash(path)), None)
            if same:
                aliases.append({"duplicate": str(path), "canonical": str(same)})
            else:
                files.append(path)
                seen.setdefault(key, []).append(path)
    exits = {}
    standalone_exits = []
    for path in files:
        if ".exit." not in path.name:
            continue
        rows = [evidence(path, i, json.loads(line)) for i, line in enumerate(path.open(), 1)]
        attached = None
        for row in rows:
            log_path = (row.get("connection") or {}).get("log_path")
            if log_path:
                attached = Path(log_path).name
                break
        if attached:
            exits.setdefault(attached, []).extend(rows)
        else:
            standalone_exits.append(rows)
    sessions = []
    active_ages = []
    enabled_ages = []
    for path in files:
        if ".actions." in path.name or ".exit." in path.name:
            continue
        status = scan_status(path)
        active_ages.extend(status.pop("_active_ages"))
        enabled_ages.extend(status.pop("_all_enabled_ages"))
        action_path = path.with_suffix(".actions.jsonl")
        actions = scan_actions(action_path) if action_path.exists() else None
        exit_rows = exits.get(path.name, [])
        stop = next((r for r in exit_rows if r.get("event") == "stopping"), None)
        cleanup = next((r for r in exit_rows if r.get("event") == "cleanup_finished"), None)
        if stop:
            cause = {"reason": stop.get("reason"), "message": stop.get("error") or "", "evidence": stop}
        else:
            error = next(iter(status["errors"]), None)
            cause = {"reason": None, "message": error["message"] if error else "", "evidence": error}
        cause["category"] = category(cause["reason"], cause["message"])
        terminal = status["finished"][-1] if status["finished"] else None
        upper_confirmed = bool((terminal and terminal["disabled_confirmed"] is True)
                               or (cleanup and cleanup.get("upper_disabled_confirmed") is True))
        sessions.append({"id": path.stem, "status": status, "actions": actions,
                         "exit_records": exit_rows, "cause": cause,
                         "upper_disabled_confirmed": upper_confirmed,
                         "cleanup": cleanup})
        print(path.name, cause["category"], "upper_disabled_confirmed=" + str(upper_confirmed), flush=True)
    root_causes = Counter(s["cause"]["category"] for s in sessions)
    summary = {
        "sources": [{"path": str(p.resolve()), "exists": p.exists()} for p in roots],
        "unique_files": len(files), "unique_bytes": sum(p.stat().st_size for p in files),
        "duplicate_files_excluded": aliases, "sessions": len(sessions),
        "session_trigger_categories": root_causes,
        "sessions_with_exit_log": sum(bool(s["exit_records"]) for s in sessions),
        "upper_disabled_confirmed_sessions": sum(s["upper_disabled_confirmed"] for s in sessions),
        "status_error_records": sum(len(s["status"]["errors"]) for s in sessions),
        "fault_snapshot_records": sum(len(s["status"]["fault_snapshots"]) for s in sessions),
        "jog_rejected_records": sum(len(s["status"]["jog_rejected"]) for s in sessions),
        "sessions_with_jog_rejected": sum(bool(s["status"]["jog_rejected"]) for s in sessions),
        "teleop_records": sum((s["actions"] or {}).get("teleop_records", 0) for s in sessions),
        "wrist_scaled_records": sum((s["actions"] or {}).get("wrist_scaled_records", 0) for s in sessions),
        "standalone_exit_logs": len(standalone_exits),
        "active_feedback_age_definition": "status records with jogging=true AND ready=true; repeated hardware frames count as status observations, not independent hardware frames",
        "active_feedback_age": age_summary(active_ages),
        "all_enabled_feedback_age": age_summary(enabled_ages),
        "sessions_with_active_feedback_over_60ms": sum(bool(s["status"]["active_feedback_over_60ms"]) for s in sessions),
        "limitations": [
            "Session triggers combine explicit client stopping cause with remote error only when no client exit log exists.",
            "Remote SSH-disconnected or window-exit errors without client exit logs do not establish abnormal failure.",
            "jog_rejected/input_paused are pause records, not disable attempts or independent physical incidents.",
            "A missing disabled-confirmed terminal record is unknown, not proof that disabling failed.",
            "High-rate normal lines use metadata scanning; only selected event/terminal JSON records are fully decoded.",
            "TCP chest goals were recorded only after old request/validate succeeded; rejected client IK goals may be absent.",
            "Archives cover local files only and do not establish current hardware health or causal motor faults.",
        ],
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    (args.output / "sessions.json").write_text(json.dumps(sessions, ensure_ascii=False, indent=2) + "\n")
    (args.output / "standalone_exits.json").write_text(json.dumps(standalone_exits, ensure_ascii=False, indent=2) + "\n")
    lines = ["PICO historical log audit (read-only; no robot connection)",
             "Local timestamps use Asia/Hong_Kong.", json.dumps(summary, ensure_ascii=False, indent=2), "", "Session evidence:"]
    for s in sessions:
        stamp = s["status"].get("first_local_time")
        when = datetime.fromtimestamp(stamp, TZ).isoformat() if stamp else "unknown"
        ev = s["cause"]["evidence"] or {}
        lines.append(f'{when} {s["id"]} {s["cause"]["category"]}: {s["cause"]["message"]} | upper_confirmed={s["upper_disabled_confirmed"]} | {ev.get("path")}:{ev.get("line")}')
    (args.output / "report.txt").write_text("\n".join(lines) + "\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
