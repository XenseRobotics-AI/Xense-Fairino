"""Explicit one-shot chassis recovery; transmits current worker code over SSH."""
import argparse
import base64
from datetime import datetime
import json
from pathlib import Path
import re
import shlex
import zlib

from fairino_slam.connection.ssh import run
from fairino_slam.paths import LOG_ROOT

HERE = Path(__file__).resolve().parent
PREFIX = "CHASSIS_RECOVERY_RESULT "
REMOTE_TIMEOUT = 180
SUCCESS_FIELDS = ("cleared_and_disabled", "temporary_controllers_removed", "navigation_stopped")


def remote_command(execute=False):
    bundle = {name: (HERE/name).read_text()
              for name in ("fault_reset_remote.py", "recovery_remote.py")}
    encoded = base64.b64encode(zlib.compress(json.dumps(bundle).encode(), 9)).decode()
    loader = (
        "import base64,zlib,json,sys,types;"
        "bundle=json.loads(zlib.decompress(base64.b64decode(" + repr(encoded) + ")));"
        "f=types.ModuleType('fault_reset_remote');sys.modules[f.__name__]=f;"
        "exec(compile(bundle['fault_reset_remote.py'],'fault_reset_remote.py','exec'),f.__dict__);"
        "r=types.ModuleType('recovery_remote');sys.modules[r.__name__]=r;"
        "exec(compile(bundle['recovery_remote.py'],'recovery_remote.py','exec'),r.__dict__);"
        "raise SystemExit(r.main(execute=" + repr(bool(execute)) + "))")
    root = "/opt/fairino_AMR_project_B/reference_chassis_runtime/install"
    lines = ["set -e", "source /opt/ros/humble/setup.bash"]
    for package in ("hardware_state_broadcaster_msgs", "fairino_hardware_command_msgs"):
        lines.append(f"source {root}/{package}/share/{package}/local_setup.bash")
    lines.extend((
        "export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0",
        "export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fairino_AMR_project_B/config/fastrtps_udp.xml",
        "exec python3 -u -c " + shlex.quote(loader)))
    return "\n".join(lines)


def parse_report(output, execute=False):
    lines = [line[len(PREFIX):] for line in output.splitlines() if line.startswith(PREFIX)]
    if len(lines) != 1:
        raise RuntimeError("未收到唯一的完整恢复结果；执行情况不确定，请只读复查，未自动重试")
    try:
        def invalid_constant(value):
            raise ValueError(value)
        report = json.loads(lines[0], parse_constant=invalid_constant)
    except (ValueError, TypeError) as exc:
        raise RuntimeError("恢复结果格式无效；执行情况不确定，未自动重试") from exc
    if not isinstance(report, dict) or type(report.get("ok")) is not bool:
        raise RuntimeError("恢复结果缺少明确状态；执行情况不确定，未自动重试")
    if execute and report["ok"] and (
        report.get("phase") != "complete"
        or any(report.get(field) is not True for field in SUCCESS_FIELDS)
        or report.get("enable_sent") is not False
        or report.get("motion_sent") is not False
    ):
        raise RuntimeError("远端结果未完整确认下使能及控制器清理；未判为恢复成功，未自动重试")
    return report


def run_recovery(host, user, password=None, execute=True):
    """One explicit attempt. No automatic retry after an uncertain SSH/action result."""
    if (not isinstance(host, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]+", host)
            or host.startswith("-") or not isinstance(user, str)
            or not re.fullmatch(r"[A-Za-z0-9_-]+", user) or user.startswith("-")):
        raise ValueError("无效的 SSH 主机或用户名")
    folder = LOG_ROOT/"recovery"/datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    folder.mkdir(parents=True)
    metadata = {"host": host, "user": user, "execute": bool(execute),
                "local_time": datetime.now().astimezone().isoformat()}
    try:
        output = run(host, remote_command(execute), user=user, password=password,
                     timeout=REMOTE_TIMEOUT, check=False)
        report = parse_report(output, execute)
    except Exception as exc:
        reason = str(exc)
        if password:
            reason = reason.replace(password, "[redacted]")
        report = {"ok": False, "phase": "connection_or_result", "failure": reason,
                  "result_uncertain": bool(execute), "automatic_retry": False}
        (folder/"result.json").write_text(json.dumps(
            {**report, **metadata}, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
        raise RuntimeError(
            "恢复未得到完整确认：" + reason +
            "；未自动重试，请先只读复查。日志：" + str(folder)) from exc
    report.update(metadata)
    report["log_directory"] = str(folder)
    (folder/"result.json").write_text(json.dumps(
        report, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
    return report


def report_lines(report):
    phases = {
        "preflight": "检查反馈与控制器", "lock": "等待底盘控制权",
        "stop_navigation": "停止遗留导航", "pause_odometry": "暂停里程计",
        "stop_controllers": "停用运动控制器", "unload_controllers": "卸载遗留临时控制器",
        "prepare": "停用运动控制器",
        "prepare_base": "停用运动控制器", "reset": "单次清除故障",
        "verify_reset": "复查故障", "disable": "确认下使能",
        "verify_final": "持续复查硬件", "unload": "卸载遗留临时控制器",
        "verify_controllers": "复查控制器", "final_check": "最终复查反馈与控制器",
        "complete": "恢复完成",
        "connection_or_result": "连接或结果不确定",
    }
    lines = []
    if report.get("phase"):
        lines.append("阶段：" + phases.get(report["phase"], report["phase"]))
    if report.get("navigation_stopped"):
        lines.append("导航服务已停止。")
    if report.get("send_attempted"):
        lines.append("已请求单次故障复位：" + str(report.get("command_result", "结果未确认")))
    if report.get("disable_attempted"):
        lines.append("已请求下使能：" + str(report.get("disable_result", "结果未确认")))
    if report.get("temporary_controllers_removed"):
        lines.append("遗留临时控制器已卸载。")
    if report.get("execute"):
        if report.get("ok") and all(report.get(k) is True for k in SUCCESS_FIELDS):
            lines.append("恢复完成：故障为零、静止且全部下使能。可重新接入；不会自动继续旧目标。")
        else:
            lines.append("恢复未完成：" + str(report.get("failure") or
                         report.get("blocked_reason") or "状态未完整确认"))
    else:
        lines.append(str(report.get("failure") or report.get("blocked_reason") or
                         "只读检查完成，未执行清理或复位。"))
    if report.get("odometry_warning"):
        lines.append(str(report["odometry_warning"]))
    if report.get("odometry_paused"):
        lines.append("里程计已暂停；重新接入后按提示重新初始化定位。")
    if report.get("log_directory"):
        lines.append("记录：" + str(report["log_directory"]))
    return lines


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="底盘接入恢复：默认只读检查；显式执行时停止导航、清理遗留控制器、单次清错并确认下使能，不使能或自动恢复目标。")
    parser.add_argument("--host", default="192.168.58.3")
    parser.add_argument("--user", default="fairino")
    parser.add_argument("--execute", action="store_true",
                        help="执行一次恢复；请先正常退出所有底盘控制会话，锁仍被占用时不会抢占")
    args = parser.parse_args(argv)
    try:
        report = run_recovery(args.host, args.user, execute=args.execute)
    except (Exception, KeyboardInterrupt) as exc:
        print(str(exc) or "用户中断；若操作已开始，请只读复查，未自动重试", flush=True)
        return 1
    for line in report_lines(report):
        print(line, flush=True)
    return int(not report.get("ok", False))


if __name__ == "__main__":
    raise SystemExit(main())
