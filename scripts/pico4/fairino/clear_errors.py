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


def run_part(part, execute, user, *, host=None, password=None):
    if password is not None:
        # Embedded GUI uses the identical remote reset implementation.
        import sys
        sys.path.insert(0, str(ROOT.parents[2] / 'tools'))
        from ssh_read import run
        return parse_report(run(host or HOSTS[part], remote_command(part, execute),
                                user=user, password=password, timeout=90, check=False))
    argv = ["ssh", "-T", "-o", "StrictHostKeyChecking=accept-new", "-o", "ConnectTimeout=8",
            "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=3",
            f"{user}@{host or HOSTS[part]}", "bash -c " + shlex.quote(remote_command(part, execute))]
    # SSH prompts on the terminal. Passwords never enter this program or its log.
    result = subprocess.run(argv, stdout=subprocess.PIPE, text=True, timeout=90)
    report = parse_report(result.stdout)
    report["ssh_exit_status"] = result.returncode
    if result.returncode != 0:
        report["ok"] = False
    return report


def report_lines(report, execute):
    """Shared CLI/workbench result text, including partially completed resets."""
    lines = []
    phase = {'preflight': '前置检查', 'reset': '发送复位', 'verify_reset': '复查故障',
             'disable': '下使能', 'verify_final': '最终复查', 'complete': '完成'}
    if report.get('phase'):
        lines.append('阶段：' + phase.get(report['phase'], report['phase']))
    for field, title in (('before', '之前'), ('after_reset', '复位后'), ('after', '最后反馈')):
        state = report.get(field) or (report.get('latest') if field == 'after' else None)
        if state:
            faults = ', '.join(f'{j.upper()}={e}' for j, e in state['joint_errors'].items()) or '无'
            enabled = ', '.join(n.upper() for n in state['enabled_axes']) or '无'
            lines.append(f'{title}：故障 {state["error_hex"]}，关节故障 {faults}，'
                         f'使能 {len(state["enabled_axes"])}（{enabled}）')
    if report.get('controller_stopped'):
        lines.append('已停用底盘运动控制器')
    if execute:
        if report.get('send_attempted'):
            result = report.get('command_result', '执行结果未确认')
            if report.get('accepted') is False:
                result = '请求被厂家拒绝'
            lines.append('复位请求已发送；返回：' + str(result))
        else:
            lines.append(report.get('skipped') or '复位请求未发送')
        if report.get('disable_attempted'):
            lines.append('下使能请求已发送；返回：' + str(report.get('disable_result', '执行结果未确认')))
        if report.get('ok') and report.get('cleared_and_disabled'):
            lines.append('完成：已复查故障为零、静止且全部下使能。')
        else:
            lines.append('未完成：' + (report.get('failure') or report.get('blocked_reason') or '请检查最后反馈与日志'))
    else:
        lines.append(report.get('failure') or report.get('blocked_reason') or
                     ('满足清错前置条件（允许部分轴仍使能）。' if report.get('ready_to_reset') else '只读检查完成'))
    if report.get('odometry_warning'):
        lines.append(report['odometry_warning'])
    return lines


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
    if args.execute:
        print("流程：停止运动控制并确认静止 → 必要时单次复位 → 故障归零后下使能 → 最终复查。允许部分轴仍使能，不自动恢复运动。", flush=True)
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
        for line in report_lines(report, args.execute):
            print('  ' + line)
        failed |= not report.get("ok", False)
        if args.execute and not report.get("ok"):
            print("已停止后续清错；不会重复发送。")
            break
    print("记录：", log)
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
