#!/usr/bin/env python3
"""Record chassis odometry over SSH and plot a manual motion test. Read only."""

import argparse
import datetime
import json
import math
import os
from pathlib import Path
import re
import shlex
import time

ODOM_TOPICS = (
    "/fairino_keyboard/odom",
    "/eight_dof_chassis_controller/odom",
    "/odom",
)
EVENT_PREFIX = "ODOMETRY_EVENT "


def observe(duration):
    import signal
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from tf2_msgs.msg import TFMessage
    from hardware_state_broadcaster_msgs.msg import HardwareState
    from controller_manager_msgs.srv import ListControllers
    from rcl_interfaces.srv import GetParameters

    rclpy.init()
    node = rclpy.create_node("fairino_odometry_test_readonly_" + str(os.getpid()))
    samples = {topic: [] for topic in ODOM_TOPICS}
    commands = []
    transforms, matches = [], []
    hardware = {"messages": 0, "latest": None, "global_errors": set()}
    subscriptions = []
    clients = []
    futures = []
    started = time.monotonic()
    stopping = False
    sent = {topic: 0 for topic in ODOM_TOPICS}
    sent_commands = 0
    sent_transforms = sent_matches = 0
    metadata = {}

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def emit(stage):
        nonlocal sent_commands, sent_transforms, sent_matches
        status_file = Path.home()/'.local/share/fairino-odometry/status.json'
        try:
            metadata['odometry'] = json.loads(status_file.read_text())
        except (OSError, ValueError):
            pass
        for key, future in futures:
            if not future.done() or key in metadata:
                continue
            if key == "ekf":
                values = future.result().values
                metadata[key] = {"odom0": values[0].string_value,
                                 "imu0_config": list(values[1].bool_array_value)}
            else:
                metadata[key] = {c.name: c.state for c in future.result().controller}
        event = dict(metadata, stage=stage, elapsed_s=time.monotonic() - started,
                     samples={topic: rows[sent[topic]:] for topic, rows in samples.items()},
                     publishers={topic: node.count_publishers(topic) for topic in ODOM_TOPICS},
                     keyboard_commands=commands[sent_commands:],
                     transforms=transforms[sent_transforms:], scan_matches=matches[sent_matches:],
                     hardware=dict(hardware, global_errors=sorted(hardware["global_errors"])))
        for topic, rows in samples.items():
            sent[topic] = len(rows)
        sent_commands = len(commands)
        sent_transforms, sent_matches = len(transforms), len(matches)
        if stage == "ready":
            event["started_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        if stage in ("complete", "interrupted"):
            event["finished_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        print(EVENT_PREFIX + json.dumps(event, ensure_ascii=False), flush=True)

    def odometry_callback(topic):
        def receive(msg):
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
            samples[topic].append({
                "t": time.monotonic() - started,
                "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "x": p.x, "y": p.y,
                "yaw": math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z)),
                "vx": v.linear.x, "vy": v.linear.y, "wz": v.angular.z,
                "x_variance": msg.pose.covariance[0],
            })
        return receive

    def command_callback(msg):
        commands.append({"t": time.monotonic() - started,
                         "vx": msg.linear.x, "vy": msg.linear.y, "wz": msg.angular.z})

    def transform_callback(msg):
        for tf in msg.transforms:
            if (tf.header.frame_id, tf.child_frame_id) not in (('map', 'odom'), ('odom', 'base_footprint')):
                continue
            p, q = tf.transform.translation, tf.transform.rotation
            transforms.append({'t': time.monotonic()-started,
                               'stamp': tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9,
                               'frame_id': tf.header.frame_id, 'child_frame_id': tf.child_frame_id,
                               'x': p.x, 'y': p.y,
                               'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))})

    def match_callback(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        matches.append({'t': time.monotonic()-started,
                        'stamp': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                        'frame_id': msg.header.frame_id, 'x': p.x, 'y': p.y,
                        'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))})

    def hardware_callback(msg):
        hardware["messages"] += 1
        hardware["global_errors"].add(int(msg.hexerrorcode_curerrcode))
        hardware["latest"] = {
            "global_error": hex(int(msg.hexerrorcode_curerrcode)),
            "faults": {j: e for j, e in zip(msg.joint_names, msg.errorcode) if e},
            "enabled_axes": [j for j, sw in zip(msg.joint_names, msg.stateword)
                             if (int(sw) & 0x006F) == 0x0027],
            "joint_names": list(msg.joint_names),
            "position": list(msg.position), "velocity": list(msg.velocity),
        }

    qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
    try:
        for topic in ODOM_TOPICS:
            subscriptions.append(node.create_subscription(
                Odometry, topic, odometry_callback(topic), qos))
        subscriptions.append(node.create_subscription(
            Twist, "/fairino_keyboard/cmd_vel", command_callback, qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(
            HardwareState, "/hardware_states", hardware_callback, qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(TFMessage, '/tf', transform_callback, 200))
        subscriptions.append(node.create_subscription(PoseWithCovarianceStamped, '/pose', match_callback, 100))
        discovery_end = time.monotonic() + 3
        while not stopping and time.monotonic() < discovery_end:
            rclpy.spin_once(node, timeout_sec=.02)

        client = node.create_client(ListControllers, "/controller_manager/list_controllers")
        clients.append(client)
        if client.wait_for_service(timeout_sec=1):
            futures.append(("controllers_at_start", client.call_async(ListControllers.Request())))
        client = node.create_client(GetParameters, "/ekf_filter_node/get_parameters")
        clients.append(client)
        if client.wait_for_service(timeout_sec=1):
            request = GetParameters.Request()
            request.names = ["odom0", "imu0_config"]
            futures.append(("ekf", client.call_async(request)))

        for rows in samples.values():
            rows.clear()
        commands.clear()
        transforms.clear()
        matches.clear()
        hardware.update(messages=0, latest=None, global_errors=set())
        started = time.monotonic()
        emit("ready")
        next_emit = started + 1
        while not stopping and rclpy.ok() and time.monotonic() - started < duration:
            rclpy.spin_once(node, timeout_sec=.01)
            if time.monotonic() >= next_emit:
                emit("progress")
                next_emit = time.monotonic() + 1
        emit("interrupted" if stopping or not rclpy.ok() else "complete")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def receive_report(host, user, command, duration, output, password=None):
    """Save each received batch immediately; Ctrl+C asks only this observer to stop."""
    import getpass
    import pexpect

    if not re.fullmatch(r"[A-Za-z0-9_.:-]+", host) or not re.fullmatch(r"[A-Za-z0-9_-]+", user):
        raise ValueError("Invalid SSH host/user")
    output.mkdir(parents=True, exist_ok=True)
    report = {"mode": "read_only", "requested_duration_s": duration, "duration_s": 0,
              "capture_status": "connection_lost", "topics": {
                  topic: {"publishers": 0, "samples": []} for topic in ODOM_TOPICS},
              "keyboard_commands": [], "hardware": {"messages": 0, "latest": None, "global_errors": []}}
    report.update(transforms=[], scan_matches=[])
    child = pexpect.spawn("ssh", ["-tt", "-o", "StrictHostKeyChecking=accept-new",
                          "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=5",
                          user + "@" + host, "bash -c " + shlex.quote(command)],
                          encoding="utf-8", timeout=1, maxread=65536, echo=False)
    asked = False
    interrupted = False
    deadline = time.monotonic() + duration + 45
    next_progress = 0
    errors = []
    try:
        with (output / "events.jsonl").open("w", buffering=1) as events:
            while time.monotonic() < deadline:
                try:
                    event = child.expect([r"(?i)password:", r"\r?\n", pexpect.EOF, pexpect.TIMEOUT], timeout=1)
                    if event == 0:
                        if asked:
                            report["error"] = "SSH authentication rejected"
                            break
                        if password is None:
                            password = getpass.getpass(f"SSH 密码 ({user}@{host}): ")
                        child.sendline(password)
                        asked = True
                        continue
                    if event == 3:
                        continue
                    if event == 2:
                        if report["capture_status"] not in ("complete", "interrupted"):
                            report["error"] = "SSH ended before the final batch: " + "\n".join(errors[-4:])
                        break
                    line = child.before.strip()
                    if not line.startswith(EVENT_PREFIX):
                        if line:
                            errors.append(line[-1000:])
                        continue
                    value = json.loads(line[len(EVENT_PREFIX):])
                    events.write(json.dumps(value, ensure_ascii=False) + "\n")
                    events.flush()
                    for topic, rows in value["samples"].items():
                        report["topics"][topic]["samples"].extend(rows)
                        report["topics"][topic]["publishers"] = value["publishers"][topic]
                    report["keyboard_commands"].extend(value["keyboard_commands"])
                    report['transforms'].extend(value.get('transforms', []))
                    report['scan_matches'].extend(value.get('scan_matches', []))
                    report["hardware"] = value["hardware"]
                    report["duration_s"] = value["elapsed_s"]
                    for key in ("started_utc", "finished_utc", "controllers_at_start", "ekf", "odometry"):
                        if key in value:
                            report[key] = value[key]
                    if value["stage"] == "ready":
                        print("采集已开始。现在可以启动键控；按一次 Ctrl+C 可提前结束并保存。", flush=True)
                        next_progress = 5
                    elif value["stage"] in ("complete", "interrupted"):
                        report["capture_status"] = value["stage"]
                        break
                    elif value["elapsed_s"] >= next_progress:
                        raw = len(report["topics"][ODOM_TOPICS[1]]["samples"])
                        filtered = len(report["topics"]["/odom"]["samples"])
                        print(f"已采集 {value['elapsed_s']:.0f}/{duration:g} 秒：厂家轮速 {raw} 条，建图 odom {filtered} 条，雷达匹配 {len(report['scan_matches'])} 次", flush=True)
                        next_progress = value["elapsed_s"] + 5
                except KeyboardInterrupt:
                    if interrupted:
                        break
                    interrupted = True
                    report["capture_status"] = "interrupted"
                    print("正在结束只读采集并保存已收到的数据……", flush=True)
                    child.sendcontrol("c")
                    deadline = time.monotonic() + 8
            else:
                report["error"] = "Capture timed out; received batches have been preserved"
    finally:
        child.close(force=True)
    report.setdefault("finished_utc", datetime.datetime.now(datetime.timezone.utc).isoformat())
    return report


def measurements(rows):
    result = {"messages": len(rows)}
    if not rows:
        return result
    elapsed = rows[-1]["t"] - rows[0]["t"]
    path = math.fsum(math.hypot(b["x"] - a["x"], b["y"] - a["y"])
                     for a, b in zip(rows, rows[1:]))
    yaw_change = math.fsum(math.atan2(math.sin(b["yaw"] - a["yaw"]),
                                    math.cos(b["yaw"] - a["yaw"]))
                          for a, b in zip(rows, rows[1:]))
    return dict(result, received_hz=(len(rows) - 1) / elapsed if elapsed > 0 else None,
                observed_s=elapsed, path_length_m=path,
                net_displacement_m=math.hypot(rows[-1]["x"] - rows[0]["x"],
                                              rows[-1]["y"] - rows[0]["y"]),
                yaw_change_deg=math.degrees(yaw_change),
                latest_x_variance=rows[-1]["x_variance"])


def plot_report(report, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    font = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if font.is_file():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams["axes.unicode_minus"] = False
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.6), layout="constrained")
    colors = ("#147faf", "#329469", "#c04451")
    output_label = '原始直通 /odom' if report.get('odometry', {}).get('mode') == 'raw' else '建图 /odom'
    labels = ('旧键控私有里程计', '厂家轮速里程计', output_label)
    available = 0
    for topic, color, label in zip(ODOM_TOPICS, colors, labels):
        rows = report["topics"][topic]["samples"]
        if not rows:
            continue
        available += 1
        first = rows[0]
        c, s = math.cos(first["yaw"]), math.sin(first["yaw"])
        xs = [c * (r["x"] - first["x"]) + s * (r["y"] - first["y"]) for r in rows]
        ys = [-s * (r["x"] - first["x"]) + c * (r["y"] - first["y"]) for r in rows]
        axes[0].plot(xs, ys, color=color, label=label, linewidth=2)
        axes[1].plot([r["t"] for r in rows], [r["vx"] for r in rows],
                     color=color, label=label, linewidth=1.7)
    commands = report["keyboard_commands"]
    if commands:
        axes[1].plot([r["t"] for r in commands], [r["vx"] for r in commands],
                     color="#65748a", linestyle="--", label="键控前进速度指令")
    axes[0].set(title="各话题相对首帧的轨迹", xlabel="首帧朝向 X / m", ylabel="Y / m")
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].scatter([0], [0], marker="+", color="#222b38", s=80)
    axes[1].set(title="前进速度：指令与里程计反馈", xlabel="采样时间 / s", ylabel="速度 / (m/s)")
    for ax in axes:
        ax.grid(alpha=.25)
        if available:
            ax.legend()
    missing = [label for topic, label in zip(ODOM_TOPICS, labels)
               if not report["topics"][topic]["samples"]]
    subtitle = "无消息：" + "、".join(missing) if missing else "轨迹按各自首帧位置与朝向归一化"
    fig.suptitle("底盘里程计只读测试\n" + subtitle, fontsize=14, fontweight="bold")
    fig.savefig(output / "odometry-test.png", dpi=150)
    plt.close(fig)


def main(argv=None, password=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.58.3")
    parser.add_argument("--user", default="fairino")
    parser.add_argument("--duration", type=float, default=60)
    parser.add_argument("--reference-distance-m", type=float,
                        help="Manually measured path length over the same recording interval")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--observer", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not 3 <= args.duration <= 3600:
        parser.error("--duration must be between 3 and 3600 seconds")
    if args.reference_distance_m is not None and args.reference_distance_m <= 0:
        parser.error("--reference-distance-m must be positive")
    if args.observer:
        observe(args.duration)
        return 0

    output = args.output or Path(__file__).resolve().parents[1] / "logs" / (
        datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + "-odometry-test")
    setup = (
        "source /opt/ros/humble/setup.bash\n"
        "source /opt/fairino_AMR_project_B/AMR_SAFT/install/setup.bash\n"
        "source /opt/fairino_AMR_project_B/reference_chassis_runtime/install/"
        "hardware_state_broadcaster_msgs/share/hardware_state_broadcaster_msgs/local_setup.bash\n"
        "export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0\n"
        "export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fairino_AMR_project_B/config/fastrtps_udp.xml\n"
    )
    command = setup + "python3 -u -c " + shlex.quote(Path(__file__).read_text())
    command += " --observer --duration " + str(args.duration)
    print(f"正在连接机器人，计划只读采集 {args.duration:g} 秒。结果保存到 {output}", flush=True)
    if password is None:
        password = os.environ.pop("FAIRINO_SSH_PASSWORD", None)
    report = receive_report(args.host, args.user, command, args.duration, output, password)
    summary = {key: report.get(key) for key in (
        "mode", "capture_status", "started_utc", "finished_utc", "duration_s",
        "controllers_at_start", "ekf", "hardware", "error")}
    summary['odometry_status'] = report.get('odometry')
    summary["odometry"] = {topic: measurements(value["samples"])
                           for topic, value in report["topics"].items()}
    summary["keyboard_command_messages"] = len(report["keyboard_commands"])
    summary.update(tf_messages=len(report['transforms']), scan_match_messages=len(report['scan_matches']))
    raw = summary['odometry'][ODOM_TOPICS[1]]
    if raw['messages'] == 0:
        raw = summary['odometry'][ODOM_TOPICS[0]]
    if args.reference_distance_m is not None and raw["messages"] > 1:
        raw["reference_path_length_m"] = args.reference_distance_m
        raw["path_length_error_percent"] = 100 * (
            raw["path_length_m"] - args.reference_distance_m) / args.reference_distance_m
    output.mkdir(parents=True, exist_ok=True)
    (output / "samples.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    plot_report(report, output)
    print(f"采集状态：{report['capture_status']}；已保存 {report['duration_s']:.1f} 秒的数据。")
    for topic, values in summary["odometry"].items():
        if values["messages"] < 2:
            print(topic, f"{values['messages']} 条消息")
        else:
            print(topic, f"{values['messages']} 条，{values['received_hz']:.2f} Hz，"
                  f"路程 {values['path_length_m']:.4f} m，转角 {values['yaw_change_deg']:.3f}°")
    if "path_length_error_percent" in raw:
        print(f"对照人工实测距离，轮速路程误差 {raw['path_length_error_percent']:.2f}%")
    print(f"TF {summary['tf_messages']} 条；成功雷达匹配 {summary['scan_match_messages']} 次。")
    if report.get("error"):
        print("采集说明：", report["error"])
    print("记录：", output / "summary.json")
    print("图表：", output / "odometry-test.png")
    return int(report["capture_status"] == "connection_lost")


if __name__ == "__main__":
    raise SystemExit(main())
