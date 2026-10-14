#!/usr/bin/env python3
"""Inspect or reset faults on this robot; uses system SSH and Python only."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parent
HOSTS = {"upper": "192.168.58.1", "base": "192.168.58.3"}
LABELS = {"upper": "上半身", "base": "底盘"}
PREFIX = "FAULT_RESET_RESULT "


def remote_command(part, execute):
    lines = ["set -e", "source /opt/ros/humble/setup.bash"]
    if part == "upper":
        lines += ["source /opt/fairino_art_plugin/plugin/install/setup.bash",
                  "export ROS_DOMAIN_ID=0",
                  "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/fairino/fastrtps_udp.xml"]
    else:
        root = "/opt/fairino_AMR_project_B/reference_chassis_runtime/install"
        for pkg in ("hardware_state_broadcaster_msgs", "fairino_hardware_command_msgs"):
            lines.append(f"source {root}/{pkg}/share/{pkg}/local_setup.bash")
        lines += ["export ROS_DOMAIN_ID=1",
                  "export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fairino_AMR_project_B/config/fastrtps_udp.xml"]
    lines += ["export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0",
              "exec python3 -u -c " + shlex.quote((ROOT / "fault_reset_remote.py").read_text())
              + " " + part + " " + ("reset" if execute else "check")]
    return "\n".join(lines)


def parse_report(output):
    lines = [line[len(PREFIX):] for line in output.splitlines() if line.startswith(PREFIX)]
    if len(lines) != 1:
        raise RuntimeError("未收到完整复查结果；若已请求清错，执行情况不确定，请只读复查，勿自动重试")
    return json.loads(lines[0])


def run_part(part, execute, user):
    argv = ["ssh", "-T", "-o", "StrictHostKeyChecking=accept-new", "-o", "ConnectTimeout=8",
            "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=3",
            f"{user}@{HOSTS[part]}", "bash -c " + shlex.quote(remote_command(part, execute))]
    # SSH prompts on the terminal. Passwords never enter this program or its log.
    result = subprocess.run(argv, stdout=subprocess.PIPE, text=True, timeout=90)
    report = parse_report(result.stdout)
    report["ssh_exit_status"] = result.returncode
    if result.returncode != 0:
        report["ok"] = False
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="上半身/底盘单次故障复位；默认只检查，不使能、不发运动目标。")
    parser.add_argument("--part", choices=("all", "upper", "base"), default="all")
    parser.add_argument("--execute", action="store_true", help="每块选中的控制板至多发送一次 reseterror()")
    parser.add_argument("--user", default="fairino", help="机器人 SSH 用户名")
    args = parser.parse_args(argv)
    if not args.user or args.user.startswith("-") or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in args.user):
        parser.error("无效 SSH 用户名")
    parts = ("upper", "base") if args.part == "all" else (args.part,)
    log = ROOT / "diagnostics" / (datetime.now().strftime("%Y%m%d-%H%M%S-%f") + "-fault-reset")
    log.mkdir(parents=True)
    print("模式：" + ("单次清错；先确认遥操/键控已退出、机器人静止、急停已释放。" if args.execute else "只读检查，不发送复位。"), flush=True)
    failed = False
    for part in parts:
        print(f"连接{LABELS[part]} {HOSTS[part]}（SSH 密码由终端提示输入）", flush=True)
        try:
            report = run_part(part, args.execute, args.user)
        except (Exception, KeyboardInterrupt) as exc:
            report = {"part": part, "ok": False,
                      "failure": str(exc) or "用户中断",
                      "note": "连接或结果读取失败；若已请求清错，结果不确定。请先只读复查，未自动重试。"}
        report["host"] = HOSTS[part]
        report["local_time"] = datetime.now().astimezone().isoformat()
        (log / f"{part}.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        for field in ("before", "after"):
            state = report.get(field)
            if state:
                faults = ", ".join(f"{j.upper()}={e}" for j, e in state["joint_errors"].items()) or "无"
                print(f"  {field}: 总故障 {state['error_hex']}，关节故障 {faults}，使能 {len(state['enabled_axes'])}")
        if args.execute:
            print("  " + (report.get("skipped") or "清除成功，已复查故障为零且未使能。") if report.get("ok")
                  else "  未确认清除成功：" + report.get("failure", "请查看日志"))
        else:
            print("  " + ("满足清错前置条件。" if report.get("ready_to_reset") else report.get("blocked_reason", report.get("failure", "请查看日志"))))
        failed |= not report.get("ok", False)
        if args.execute and not report.get("ok"):
            print("已停止后续清错；不会重复发送。")
            break
    print("记录：", log)
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
