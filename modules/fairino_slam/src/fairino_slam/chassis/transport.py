#!/usr/bin/env python3
"""Shared SSH transport for mapping drive and navigation chassis execution."""
import base64
import getpass
import json
import queue
import re
import shlex
import sys
import threading
import time
import zlib
from pathlib import Path

from .control_core import check_speeds, configuration_snapshot, DEFAULT_LINEAR, DEFAULT_ANGULAR
from .source import render_control_core
from fairino_slam.paths import LOG_ROOT

HERE = Path(__file__).resolve().parent
PREFIX = "@KEYCTRL "


def remote_command(args):
    bundle = json.dumps({"control_core.py": render_control_core(configuration_snapshot()),
                         "motion_feedback.py": (HERE / "motion_feedback.py").read_text(),
                         "robot_worker.py": (HERE / "robot_worker.py").read_text()}).encode()
    encoded = base64.b64encode(zlib.compress(bundle)).decode()
    argv = ["robot_worker.py", "--linear", str(args.linear), "--angular", str(args.angular)]
    if getattr(args, "navigation", False):
        argv.append("--navigation")
    if args.check:
        argv.append("--check")
    if args.prepare_check:
        argv.append("--prepare-check")
    loader = ("import base64,zlib,json,sys,types;"
              f"bundle=json.loads(zlib.decompress(base64.b64decode({encoded!r})));"
              "m=types.ModuleType('control_core');sys.modules['control_core']=m;"
              "exec(compile(bundle['control_core.py'],'control_core.py','exec'),m.__dict__);"
              "f=types.ModuleType('motion_feedback');sys.modules['motion_feedback']=f;"
              "exec(compile(bundle['motion_feedback.py'],'motion_feedback.py','exec'),f.__dict__);"
              f"sys.argv={argv!r};"
              "exec(compile(bundle['robot_worker.py'],'robot_worker.py','exec'),{'__name__':'__main__'})")
    root = "/opt/fairino_AMR_project_B/reference_chassis_runtime/install"
    lines = ["set -e", "source /opt/ros/humble/setup.bash"]
    for pkg in ("hardware_state_broadcaster_msgs", "fairino_hardware_command_msgs"):
        lines.append(f"source {root}/{pkg}/share/{pkg}/local_setup.bash")
    lines += ["export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
              "export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fairino_AMR_project_B/config/fastrtps_udp.xml",
              "echo KEYCTRL_CONNECTED", "exec python3 -u -c " + shlex.quote(loader)]
    return "bash -c " + shlex.quote("\n".join(lines))


class Connection:
    def __init__(self, args, password=None):
        if (not re.fullmatch(r"[A-Za-z0-9_.:-]+", args.host)
                or not re.fullmatch(r"[A-Za-z0-9_-]+", args.user)
                or args.user.startswith("-") or args.host.startswith("-")):
            raise ValueError("无效的 SSH 主机或用户名")
        import pexpect
        self.pexpect = pexpect
        self.events = queue.Queue()
        self.latest = {}
        self.last_error = ''
        self.last_status = 0.0
        self.last_send = 0.0
        self.write_lock = threading.Lock()
        self.navigation = bool(getattr(args, "navigation", False))
        self.speeds = check_speeds(args.linear, args.angular)
        if self.navigation and self.speeds != (DEFAULT_LINEAR, DEFAULT_ANGULAR):
            raise ValueError("导航执行使用固定的默认速度限制")
        self.done = threading.Event()
        self.log = None
        self.child = pexpect.spawn("ssh", ["-T", "-o", "ConnectTimeout=5",
            "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
            "-o", "StrictHostKeyChecking=accept-new", f"{args.user}@{args.host}",
            remote_command(args)], encoding="utf-8", codec_errors="replace",
            timeout=12, maxread=65536, echo=False)
        self.child.delaybeforesend = None
        try:
            attempts = 0
            while True:
                result = self.child.expect(["KEYCTRL_CONNECTED", r"(?i)password:", pexpect.EOF])
                if result == 0:
                    break
                if result == 1:
                    attempts += 1
                    if password is not None and attempts > 1:
                        raise RuntimeError("SSH 密码认证失败")
                    self.child.sendline(password if password is not None else getpass.getpass(f"SSH 密码（{args.user}@{args.host}）："))
                else:
                    raise RuntimeError("SSH 连接失败：" + self.child.before[-1500:])
        except BaseException:
            self.child.close(force=True)
            raise
        logs = Path(getattr(args, "log_directory", LOG_ROOT / "chassis"))
        logs.mkdir(parents=True, exist_ok=True)
        self.log_path = logs / (time.strftime("%Y%m%d-%H%M%S") + ".jsonl")
        self.log = self.log_path.open("a", buffering=1)
        threading.Thread(target=self.reader, daemon=True).start()

    def reader(self):
        try:
            while True:
                try:
                    line = self.child.readline()
                except self.pexpect.TIMEOUT:
                    continue
                if not line:
                    break
                line = line.rstrip()
                if line.startswith(PREFIX):
                    try:
                        data = json.loads(line[len(PREFIX):])
                    except ValueError:
                        continue
                    if data.get("error") and not self.last_error:
                        self.last_error = str(data["error"])
                    if "stamp" in data:
                        self.latest = data
                        self.last_status = time.monotonic()
                    self.events.put(data)
                    self.log.write(json.dumps({"local_time": time.time(), **data}, ensure_ascii=False) + "\n")
                elif line:
                    self.log.write(json.dumps({"ros_log": line}, ensure_ascii=False) + "\n")
        except (self.pexpect.EOF, OSError, ValueError):
            pass
        finally:
            self.done.set()
            self.events.put({"connection_closed": True})

    def send(self, keys=(), focused=True, stop=False, speeds=None):
        if self.navigation and keys:
            raise ValueError("导航执行不接受键盘运动输入")
        if self.done.is_set():
            return
        # No replay after a delayed feedback stream: stop is latched robot-side.
        now = time.monotonic()
        stale = self.last_status != 0 and now - self.last_status > 0.5
        linear, angular = check_speeds(*(self.speeds if speeds is None else speeds))
        if self.navigation and (linear, angular) != (DEFAULT_LINEAR, DEFAULT_ANGULAR):
            raise ValueError("导航执行使用固定的默认速度限制")
        packet = {"keys": sorted(keys), "focused": bool(focused),
                  "stop": bool(stop or stale), "ack": self.latest.get("stamp", 0),
                  "linear": linear, "angular": angular,
                  "client_status_age": now-self.last_status if self.last_status else None,
                  "client_send_gap": now-self.last_send if self.last_send else None}
        self.last_send = now
        with self.write_lock:
            try:
                self.child.sendline(json.dumps(packet))
            except (OSError, self.pexpect.EOF):
                self.done.set()

    def finish(self):
        self.send(stop=True)
        # Keep SSH alive while the remote worker tries zero + disable + cleanup.
        if not self.done.wait(20):
            print("远端尚未确认停止。请检查机器人并使用物理急停。", file=sys.stderr)
        if self.done.is_set():
            self.child.close()
            if self.log:
                self.log.close()
