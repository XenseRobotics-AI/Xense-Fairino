#!/usr/bin/env python3
"""Monitor localization and bound Nav2 output; this node never enables motors."""
import json
import math
import os
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
MAP_RUNTIME = Path.home()/'.local/share/fairino-map-control'
sys.path.insert(0, str(MAP_RUNTIME))
from remote_stream import LocalizationMonitor, yaw
from localization import read_session
from motion import CommandGate
from lifecycle import LifecycleMonitor
from control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR


def main():
    import rclpy
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from action_msgs.msg import GoalStatusArray
    from action_msgs.srv import CancelGoal
    import tf2_ros
    rclpy.init()
    node = rclpy.create_node('velocity_gate', namespace='/fairino_navigation')
    tf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf, node)
    monitor = LocalizationMonitor()
    lifecycle = LifecycleMonitor(node, HERE)
    gate = CommandGate(DEFAULT_LINEAR, DEFAULT_ANGULAR)
    topics = {}
    source_ok = False
    pose_source_ok = False
    last_graph = 0.
    last_write = 0.
    last_tick = None
    sources = []
    canceled = set()
    reported_failure = None
    last_goal = {}
    start_stamp = node.get_clock().now().nanoseconds*1e-9
    pub = node.create_publisher(Twist, '/fairino_navigation/cmd_vel', 1)
    cancel = node.create_client(CancelGoal, '/fairino_navigation/navigate_to_pose/_action/cancel_goal')

    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def record(topic, msg):
        topics[topic] = (time.monotonic(), node.get_clock().now().nanoseconds*1e-9-stamp(msg))

    def odom(msg):
        record('/odom', msg)
        p, v = msg.pose.pose, msg.twist.twist
        monitor.on_odometry([p.position.x, p.position.y, yaw(p.orientation)],
                            [v.linear.x, v.linear.y, v.angular.z])

    def pose(msg):
        monitor.set_session(read_session())
        p = msg.pose.pose
        monitor.accept(stamp(msg), node.get_clock().now().nanoseconds*1e-9-stamp(msg),
                       [p.position.x, p.position.y, yaw(p.orientation)],
                       list(msg.pose.covariance), msg.header.frame_id)

    def command(msg):
        try:
            gate.receive([msg.linear.x, msg.linear.y, msg.linear.z,
                          msg.angular.x, msg.angular.y, msg.angular.z], time.monotonic())
        except ValueError:
            gate.fail('收到无效或非二维速度指令', time.monotonic(), code='invalid_command')
            gate.command = None

    def goals(msg):
        nonlocal last_goal
        if msg.status_list:
            latest = max(msg.status_list, key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
            last_goal = {'id': bytes(latest.goal_info.goal_id.uuid).hex(), 'status': latest.status}
        active = [s for s in msg.status_list if s.status == 2 and
                  s.goal_info.stamp.sec+s.goal_info.stamp.nanosec*1e-9 >= start_stamp]
        # Multiple executing goals are ambiguous; no forwarding.
        gate.set_goal(bytes(active[0].goal_info.goal_id.uuid).hex() if len(active) == 1 else None,
                      time.monotonic())

    subs = [
        node.create_subscription(Twist, '/fairino_navigation/cmd_vel_raw', command, 1),
        node.create_subscription(Odometry, '/odom', odom, 10),
        node.create_subscription(LaserScan, '/scan', lambda m: record('/scan', m), qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/front/scan', lambda m: record('/front/scan', m), qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/back/scan', lambda m: record('/back/scan', m), qos_profile_sensor_data),
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', pose,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(GoalStatusArray, '/fairino_navigation/navigate_to_pose/_action/status', goals,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    def tick():
        nonlocal source_ok, pose_source_ok, last_graph, last_write, reported_failure, last_tick, sources
        now = time.monotonic()
        tick_interval = None if last_tick is None else max(0., now-last_tick)
        last_tick = now
        if now-last_graph > 1:
            last_graph = now
            publishers = node.get_publishers_info_by_topic('/fairino_navigation/cmd_vel_raw')
            sources = [{'name': p.node_name, 'namespace': p.node_namespace} for p in publishers]
            source_ok = len(publishers) == 1 and publishers[0].node_name == 'controller_server' and publishers[0].node_namespace == '/fairino_navigation'
            pose_source_ok = [p.node_name for p in node.get_publishers_info_by_topic('/amcl_pose')] == ['amcl']
        state = read_session()
        monitor.set_session(state)
        try:
            root = Path.home()/'.local/share/fairino-odometry'
            odometry = json.loads((root/'status.json').read_text())
            odometry['epoch'] = (root/'epoch').read_text().strip()
            if abs(time.time()-odometry['time']) > 2:
                odometry = {}
        except (OSError, ValueError, KeyError):
            odometry = {}
        tf_ready = False
        tf_age = None
        try:
            transform = tf.lookup_transform('map', 'base_footprint', Time())
            age = node.get_clock().now().nanoseconds*1e-9-stamp(transform)
            tf_age = age
            tf_ready = abs(age) < .5 and stamp(transform) >= (state.get('initial_pose_stamp') or float('inf'))
        except Exception:
            pass
        ages = {topic: {'age': max(now-received, abs(age+now-received)),
                        'receipt_age': max(0., now-received), 'stamp_age': age+now-received}
                for topic, (received, age) in topics.items()}
        health = monitor.status(node.get_clock().now().nanoseconds*1e-9, odometry, ages,
                                tf_ready, bool(state.get('active')), pose_source_ok)
        ready = health['ready'] and all(ages.get(t, {}).get('age', 100) <= .5
                                       for t in ('/front/scan', '/back/scan'))
        token = (state.get('id'), state.get('initial_pose_stamp'), odometry.get('epoch'))
        lifecycle.tick(now, ready, token, bool(gate.goal))
        stack_ready = lifecycle.active(now)
        stale_scans = [t for t in ('/front/scan', '/back/scan')
                       if ages.get(t, {}).get('age', 100) > .5]
        readiness_reason = (health['reason'] if not health['ready'] else
                            '双雷达观测过期：'+', '.join(stale_scans) if stale_scans else
                            '导航节点未激活或生命周期反馈过期' if not stack_ready else None)
        output = gate.output(now, ready and stack_ready, token, source_ok, readiness_reason)
        # Publish the stop before diagnostic output or file writes.
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = output
        pub.publish(msg)
        if gate.failure and gate.failure['goal'] != reported_failure:
            reported_failure = gate.failure['goal']
            gate.capture_failure_context({
                'time': time.time(), 'topic_ages': ages, 'session_token': list(token),
                'localization_reason': health['reason'], 'localization_ready': ready,
                'source_ok': source_ok, 'command_sources': sources,
                'graph_check_age': max(0., now-last_graph),
                'tf_ready': tf_ready, 'tf_age': tf_age, 'gate_tick_interval': tick_interval,
                'lifecycle': {n: {'state': v[0], 'age': max(0., now-v[1])}
                              for n, v in lifecycle.states.items()}})
            event = {'event': 'navigation_protective_stop', 'time': time.time(),
                     **gate.failure, 'topic_ages': ages, 'session_token': list(token),
                     'localization_reason': health['reason'], 'source_ok': source_ok,
                     'lifecycle_states': {n: v[0] for n, v in lifecycle.states.items()}}
            # Sparse events go to journald, independent of the latest status file.
            print(json.dumps(event, ensure_ascii=False), flush=True)
        if gate.failed_goal and gate.failed_goal not in canceled and cancel.service_is_ready():
            request = CancelGoal.Request()
            request.goal_info.goal_id.uuid = list(bytes.fromhex(gate.failed_goal))
            cancel.call_async(request)
            canceled.add(gate.failed_goal)
        if now-last_write > .2:
            last_write = now
            data = {'time': time.time(), 'localization_ready': ready, 'controller_ready': source_ok and stack_ready, 'session': state.get('id'),
                    'prepared': lifecycle.prepared(now), 'run_id': lifecycle.run_id,
                    'activation': lifecycle.activation, 'session_token': list(token),
                    'lifecycle_states': {n: v[0] for n, v in lifecycle.states.items()},
                    'goal': gate.goal, 'last_goal': last_goal, 'failed_goal': gate.failed_goal, 'failure': gate.failure, 'reason': gate.reason,
                    'localization_reason': health['reason'], 'command': output,
                    'command_stream': gate.telemetry(now), 'gate_tick_interval': tick_interval,
                    'linear_limit': DEFAULT_LINEAR, 'angular_limit': DEFAULT_ANGULAR}
            tmp = HERE/'status.tmp'
            tmp.write_text(json.dumps(data, ensure_ascii=False))
            tmp.replace(HERE/'status.json')
    timer = node.create_timer(.05, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
