#!/usr/bin/env python3
"""Own raw odometry forwarding or EKF, exclusively. Never command hardware."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

SOURCE = '/eight_dof_chassis_controller/odom'
RUNTIME = Path.home() / '.local/share/fairino-odometry'


def input_problem(raw, hardware, now):
    if raw is None or now-raw['received'] > .4:
        return '原始里程计断流'
    if abs(raw['stamp_age'] + now-raw['received']) > .5:
        return '原始里程计时间戳异常'
    if not all(math.isfinite(x) for x in raw['values']):
        return '原始里程计包含非有限值'
    if raw.get('problem'):
        return raw['problem']
    if hardware is None or now-hardware['received'] > .4:
        return '硬件状态断流'
    if hardware['fault']:
        return '底盘硬件故障'
    return None


def write_status(**state):
    RUNTIME.mkdir(parents=True, exist_ok=True)
    temporary = RUNTIME / 'status.tmp'
    temporary.write_text(json.dumps({'time': time.time(), **state}))
    temporary.replace(RUNTIME / 'status.json')


def main(argv=None):
    import rclpy
    from nav_msgs.msg import Odometry
    from hardware_state_broadcaster_msgs.msg import HardwareState
    from std_srvs.srv import Trigger
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from geometry_msgs.msg import TransformStamped
    import tf2_ros
    try:
        from .raw_odometry import RawSession
    except ImportError:
        from raw_odometry import RawSession
    RUNTIME.mkdir(parents=True, exist_ok=True)
    lease = (RUNTIME / 'guard.lock').open('a')
    fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    rclpy.init(args=[])
    node = rclpy.create_node('fairino_odometry_guard')
    raw = hardware = child = None
    raw_active = False
    raw_session = None
    config = RUNTIME / 'mode.json'
    default_mode = json.loads(config.read_text())['mode'] if config.is_file() else 'ekf'
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['raw', 'ekf'], default=default_mode)
    options, args = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    mode = options.mode
    if mode not in ('raw', 'ekf'):
        raise ValueError('未知的里程计模式')
    node.declare_parameter('mode', mode)
    node.declare_parameter('source', SOURCE)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)
    broadcaster = tf2_ros.TransformBroadcaster(node) if mode == 'raw' else None
    publisher = node.create_publisher(Odometry, '/odom', 20) if mode == 'raw' else None
    armed, running = True, True
    good_since = None
    epoch = None
    reason = '等待原始里程计与硬件状态'

    def status():
        write_status(running=raw_active or child is not None, armed=armed, epoch=epoch,
                     reason=reason, mode=mode, source=SOURCE, output='/odom')

    def on_odom(msg):
        nonlocal raw
        p, t, q = msg.pose.pose.position, msg.twist.twist, msg.pose.pose.orientation
        raw = {'received': time.monotonic(),
               'stamp_age': node.get_clock().now().nanoseconds*1e-9-msg.header.stamp.sec-msg.header.stamp.nanosec*1e-9,
               'values': [p.x, p.y, q.x, q.y, q.z, q.w, t.linear.x, t.linear.y, t.angular.z]}
        if raw_active and not input_problem(raw, hardware, time.monotonic()):
            try:
                transform = buffer.lookup_transform('base_footprint', 'base_link', Time()).transform if msg.child_frame_id == 'base_link' else None
                output = raw_session.accept(msg, transform)
                tf = TransformStamped()
                tf.header = output.header
                tf.child_frame_id = output.child_frame_id
                p = output.pose.pose.position
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p.x, p.y, p.z
                tf.transform.rotation = output.pose.pose.orientation
                publisher.publish(output)
                broadcaster.sendTransform(tf)
            except Exception as exc:
                raw['problem'] = str(exc)

    def on_hardware(msg):
        nonlocal hardware
        hardware = {'received': time.monotonic(), 'fault': bool(msg.hexerrorcode_curerrcode or any(msg.errorcode))}

    def end_child():
        nonlocal child, raw_active, raw_session
        raw_active, raw_session = False, None
        if child and child.poll() is None:
            # The executable is direct: no ros2 CLI parent left holding a child.
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.terminate()
                child.wait(timeout=3)
        child = None

    def pause(_, response):
        nonlocal armed, good_since, reason
        armed, good_since = False, None
        reason = '操作员暂停里程计；新建图需重新初始化'
        end_child()
        response.success, response.message = True, reason
        status()
        return response

    def resume(_, response):
        nonlocal armed, good_since, reason
        if raw_active or (child and child.poll() is None):
            response.success, response.message = True, '里程计已运行，未重新初始化'
            return response
        armed, good_since = True, None
        reason = '已请求恢复；等待健康输入，将开启新的里程计会话'
        response.success, response.message = True, reason
        return response

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    subs = [node.create_subscription(Odometry, SOURCE, on_odom, qos_profile_sensor_data),
            node.create_subscription(HardwareState, '/hardware_states', on_hardware, qos_profile_sensor_data)]
    services = [node.create_service(Trigger, '/fairino_odometry/pause', pause),
                node.create_service(Trigger, '/fairino_odometry/resume', resume)]
    last_status = 0.
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            problem = input_problem(raw, hardware, now)
            if (child is not None and (child.poll() is not None or problem)) or (raw_active and problem):
                reason = problem or 'EKF 进程退出'
                end_child()
                armed = False  # Input recovery alone never resumes an old map.
            if problem:
                good_since = None
                if armed:
                    reason = problem
            elif armed and child is None and not raw_active:
                good_since = good_since or now
                if now-good_since >= 1.:
                    # Refuse a second EKF (e.g. an old startup script still running).
                    names = node.get_node_names_and_namespaces()
                    others = [p for p in node.get_publishers_info_by_topic('/odom')
                              if p.node_name != node.get_name() or p.node_namespace != node.get_namespace()]
                    if any(n == 'ekf_filter_node' for n, _ in names) or others:
                        armed = False
                        reason = '已有 EKF 或其他 /odom 发布者，拒绝重复发布'
                    else:
                        epoch = str(uuid.uuid4())
                        (RUNTIME / 'epoch').write_text(epoch)
                        if mode == 'raw':
                            raw_session = RawSession()
                            raw_active = True
                            reason = '原始轮速直通；不融合、不外推，输入异常后停止'
                        else:
                            child = subprocess.Popen(['/opt/robot_localization_ws/install/robot_localization/lib/robot_localization/ekf_node', *args])
                            reason = 'EKF 正在运行；输入异常后停止，需显式恢复'
            if now-last_status > .5:
                if raw_active:
                    others = [p for p in node.get_publishers_info_by_topic('/odom')
                              if p.node_name != node.get_name() or p.node_namespace != node.get_namespace()]
                    if others or any(n == 'ekf_filter_node' for n, _ in node.get_node_names_and_namespaces()):
                        end_child()
                        armed = False
                        reason = '检测到另一里程计发布者，已停止原始直通；需排除冲突后恢复'
                status()
                last_status = now
    finally:
        end_child()
        write_status(running=False, armed=False, epoch=epoch, mode=mode, source=SOURCE, reason='守护进程已退出')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
