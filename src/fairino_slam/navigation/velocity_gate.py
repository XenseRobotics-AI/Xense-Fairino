#!/usr/bin/env python3
"""Monitor localization and bound Nav2 output; this node never enables motors."""
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
MAP_RUNTIME = Path.home()/'.local/share/fairino-map-control'
if globals().get('__package__'):
    from fairino_slam.mapping.remote_stream import CallbackReceiver, LocalizationMonitor, yaw
    from fairino_slam.mapping.localization import read_session
    from .motion import CommandGate
    from .lifecycle import LifecycleMonitor
    from .runtime_policy import TF_MAX_AGE
    from .tf_receiver import TfReceiver
    from .coordination import FeedbackInbox, wait_timeout_from_profile, publisher_state
    from .chassis_receiver import ChassisReceiver
    from fairino_slam.chassis.motion_feedback import FeedbackMonitor
    from fairino_slam.chassis.control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR, FEEDBACK_TIMEOUT
else:
    # Prefer this navigation deployment's shared monitor; mapping is only the
    # fallback for localization.py, which remains owned by the mapping runtime.
    sys.path.insert(0, str(MAP_RUNTIME))
    sys.path.insert(0, str(HERE))
    from remote_stream import CallbackReceiver, LocalizationMonitor, yaw
    from localization import read_session
    from motion import CommandGate
    from lifecycle import LifecycleMonitor
    from runtime_policy import TF_MAX_AGE
    from tf_receiver import TfReceiver
    from coordination import FeedbackInbox, wait_timeout_from_profile, publisher_state
    from chassis_receiver import ChassisReceiver
    from motion_feedback import FeedbackMonitor
    from control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR, FEEDBACK_TIMEOUT


def localization_tf_health(buffer, now_ros, initial_stamp, query_time):
    """Diagnose TF edges without changing the combined-transform safety limit."""
    result = {'ready': False, 'reason': None, 'transforms': {},
              'initial_pose_stamp': None, 'max_combined_age': TF_MAX_AGE}
    valid_initial = (isinstance(initial_stamp, (int, float))
                     and not isinstance(initial_stamp, bool)
                     and math.isfinite(initial_stamp) and initial_stamp > 0)
    if valid_initial:
        result['initial_pose_stamp'] = initial_stamp
    for key, parent, child in (
            ('map_base', 'map', 'base_footprint'),
            ('map_odom', 'map', 'odom'),
            ('odom_base', 'odom', 'base_footprint')):
        entry = {'parent': parent, 'child': child, 'stamp': None, 'age': None}
        try:
            transform = buffer.lookup_transform(parent, child, query_time)
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec*1e-9
            if not math.isfinite(stamp):
                raise ValueError('ROS 当前时间或 TF 时间戳不是有限数值')
            entry['stamp'] = stamp
        except Exception as exc:
            entry['error'] = type(exc).__name__ + ': ' + str(exc)
        result['transforms'][key] = entry
    return tf_health_at(result, now_ros)


def tf_health_at(previous, now_ros):
    """Re-evaluate cached TF stamps after synchronous work, without more I/O."""
    result = dict(previous, ready=False, reason=None)
    result['transforms'] = {key: dict(entry) for key, entry in previous['transforms'].items()}
    valid_now = (isinstance(now_ros, (int, float)) and not isinstance(now_ros, bool)
                 and math.isfinite(now_ros))
    for entry in result['transforms'].values():
        if entry['stamp'] is not None:
            if valid_now:
                entry['age'] = now_ros-entry['stamp']
            else:
                entry['age'] = None
                entry['error'] = 'ValueError: ROS 当前时间或 TF 时间戳不是有限数值'
    initial_stamp = result['initial_pose_stamp']
    combined = result['transforms']['map_base']
    if 'error' in combined:
        result['reason'] = '定位 TF 未就绪：map → base_footprint 查询失败（' + combined['error'] + '）'
    elif initial_stamp is None:
        result['reason'] = '定位 TF 未就绪：本次初始位姿时间戳无效'
    elif abs(combined['age']) >= TF_MAX_AGE:
        direction = '超前' if combined['age'] < 0 else '已过期'
        result['reason'] = ('定位 TF ' + direction + f" {abs(combined['age']):.3f} 秒"
                            f'：map → base_footprint；要求时间差小于 {TF_MAX_AGE:.1f} 秒')
    elif combined['stamp'] < initial_stamp:
        result['reason'] = ('定位 TF 早于本次初始化：map → base_footprint；'
                            f"TF={combined['stamp']:.3f}，初始化={initial_stamp:.3f}")
    else:
        # Individual edge lookups are evidence only. The original gate checks
        # the latest common map->base transform, not AMCL's future-dated edge.
        result['ready'] = True
    return result


def readiness_snapshot(monitor, state, odometry, topics, read_tf, pose_source_ok,
                       monotonic, ros_time, wall_time):
    """Read the receiving thread's latest TF, then sample clocks for every age."""
    tf_health = read_tf()
    now_wall, now_ros, now = wall_time(), ros_time(), monotonic()
    if abs(now_wall-odometry.get('time', 0.)) > 2:
        odometry = {}
    tf_health = tf_health_at(tf_health, now_ros)
    ages = {topic: {'age': max(now-received, abs(age+now-received)),
                    'received_age': now-received, 'stamp_age_at_receive': age,
                    'stamp_age': age+now-received}
            for topic, (received, age) in topics.items()}
    health = monitor.status(now_ros, odometry, ages, tf_health['ready'],
                            bool(state.get('active')), pose_source_ok)
    if health.get('reason') == '定位 TF 未就绪或已过期':
        health['reason'] = tf_health['reason'] or health['reason']
    stale_scans = [t for t in ('/front/scan', '/back/scan')
                   if ages.get(t, {}).get('age', 100) > .5]
    return {'now': now, 'now_ros': now_ros, 'tf': tf_health, 'topic_ages': ages,
            'health': health, 'ready': health['ready'] and not stale_scans,
            'stale_scans': stale_scans}


def gate_output(gate, lifecycle, token, source_ok, snapshot, monotonic, coordination_check=None):
    """Recheck after lifecycle work; the caller must publish before logging or I/O."""
    before = snapshot()
    started = monotonic()
    lifecycle.tick(before['now'], before['ready'], token, bool(gate.goal))
    lifecycle_seconds = monotonic()-started
    current = snapshot()
    now, health = current['now'], current['health']
    stack_ready = lifecycle.active(now)
    chassis = (coordination_check(now, current['now_ros']) if coordination_check is not None
               else {'ready': True, 'reason': None})
    current['chassis_motion'] = chassis
    reason = (health['reason'] if not health['ready'] else
              '双雷达观测过期：'+', '.join(current['stale_scans']) if current['stale_scans'] else
              '导航节点未激活或生命周期反馈过期' if not stack_ready else
              chassis.get('reason') if not chassis['ready'] else None)
    current.update(stack_ready=stack_ready, lifecycle_seconds=lifecycle_seconds, readiness_reason=reason,
                   output=gate.output(now, current['ready'] and stack_ready and chassis['ready'],
                                      token, source_ok, reason))
    return current



def publish_checked(gate, current, token, source_ok, publish, monotonic):
    """A concurrent input fault or pause must win before the actual publication."""
    with gate.lock:
        output = gate.output(
            monotonic(), current['ready'] and current['stack_ready'] and current['chassis_motion']['ready'],
            token, source_ok, current.get('readiness_reason'))
        publish(output)
        return output



def receive_execution_state(gate, message, now, now_ros, goal_ros_stamp, source_ok):
    """Bind BT status to the current goal and reject stale/reordered samples."""
    if not source_ok or gate.goal is None or len(message.status) != 1:
        return False
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        return False
    sent = stamp.sec + stamp.nanosec*1e-9
    age = now_ros-sent
    previous_stamp = (gate.execution or {}).get('stamp', float('-inf'))
    if sent < goal_ros_stamp or sent <= previous_stamp or not -.1 <= age <= .6:
        return False
    state = message.status[0]
    gate.set_execution(state.name, state.message, now)
    if gate.execution is not None and gate.execution['received'] == now:
        gate.execution.update(stamp=sent, received=now-max(0., age))
    return True


def main():
    import rclpy
    from rclpy.time import Time
    from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from action_msgs.msg import GoalStatusArray
    from action_msgs.srv import CancelGoal
    import tf2_ros
    rclpy.init()
    node = receiver = chassis_receiver = input_receiver = pub = None
    try:
        node = rclpy.create_node('velocity_gate', namespace='/fairino_navigation')
        receiver = TfReceiver(rclpy, tf2_ros, SingleThreadedExecutor)
        tf = receiver.buffer
        monitor = LocalizationMonitor()
        monitor.set_session(read_session())
        sensor_lock = threading.RLock()
        lifecycle = LifecycleMonitor(node, HERE)
        gate = CommandGate(DEFAULT_LINEAR, DEFAULT_ANGULAR)
        chassis_feedback = FeedbackMonitor(FEEDBACK_TIMEOUT)
        inbox = FeedbackInbox(chassis_feedback, wait_timeout_from_profile(HERE/'nav2.yaml'))
        chassis_receiver = ChassisReceiver(
            rclpy, SingleThreadedExecutor, DiagnosticArray,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), inbox)
        chassis_source_ok = False
        chassis_sources = None
        topics = {}
        source_ok = False
        pose_source_ok = False
        last_graph = 0.
        last_graph_seconds = 0.
        last_tick_seconds = 0.
        last_write = 0.
        canceled = set()
        reported_failure = None
        last_goal = {}
        goal_ros_stamp = float("inf")
        execution_source_ok = False
        start_stamp = node.get_clock().now().nanoseconds*1e-9
        pub = node.create_publisher(Twist, '/fairino_navigation/cmd_vel', 1)
        cancel = node.create_client(CancelGoal, '/fairino_navigation/navigate_to_pose/_action/cancel_goal')

        def stamp(msg):
            return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

        def record(topic, msg):
            with sensor_lock:
                topics[topic] = (time.monotonic(), node.get_clock().now().nanoseconds*1e-9-stamp(msg))

        def odom(msg):
            with sensor_lock:
                record('/odom', msg)
                p, v = msg.pose.pose, msg.twist.twist
                monitor.on_odometry([p.position.x, p.position.y, yaw(p.orientation)],
                                    [v.linear.x, v.linear.y, v.angular.z], stamp(msg))

        def pose(msg):
            with sensor_lock:
                p = msg.pose.pose
                monitor.accept(stamp(msg), node.get_clock().now().nanoseconds*1e-9-stamp(msg),
                               [p.position.x, p.position.y, yaw(p.orientation)],
                               list(msg.pose.covariance), msg.header.frame_id)

        def command(msg):
            with gate.lock:
                try:
                    gate.receive([msg.linear.x, msg.linear.y, msg.linear.z,
                                  msg.angular.x, msg.angular.y, msg.angular.z], time.monotonic())
                except ValueError:
                    gate.fail('收到无效或非二维速度指令', time.monotonic())
                    gate.command = None

        def goals(msg):
            nonlocal last_goal, goal_ros_stamp
            if msg.status_list:
                latest = max(msg.status_list, key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
                last_goal = {'id': bytes(latest.goal_info.goal_id.uuid).hex(), 'status': latest.status}
            active = [s for s in msg.status_list if s.status == 2 and
                      s.goal_info.stamp.sec+s.goal_info.stamp.nanosec*1e-9 >= start_stamp]
            # Multiple executing goals are ambiguous; no forwarding.
            with gate.lock:
                gate.set_goal(bytes(active[0].goal_info.goal_id.uuid).hex() if len(active) == 1 else None,
                              time.monotonic())
                inbox.set_goal(gate.goal)
                goal_ros_stamp = (active[0].goal_info.stamp.sec +
                                  active[0].goal_info.stamp.nanosec*1e-9
                                  if len(active) == 1 else float("inf"))

        def execution_state(msg):
            with gate.lock:
                receive_execution_state(gate, msg, time.monotonic(),
                                        node.get_clock().now().nanoseconds*1e-9,
                                        goal_ros_stamp, execution_source_ok)

        def register_inputs(input_node):
            return [
                input_node.create_subscription(DiagnosticArray, '/fairino_navigation/execution_state/navigate_to_pose',
                                               execution_state, 10),
                input_node.create_subscription(Twist, '/fairino_navigation/cmd_vel_raw', command, 1),
                # Odometry is an absolute pose/twist, not an increment to replay.
                # A depth-10 reliable queue was feeding old poses under load even
                # while the matching odom TF was fresh. Keep the newest sample;
                # source stamps and the existing .75 s watchdog remain binding.
                input_node.create_subscription(
                    Odometry, '/odom', odom,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
                input_node.create_subscription(LaserScan, '/scan', lambda m: record('/scan', m), qos_profile_sensor_data),
                input_node.create_subscription(LaserScan, '/front/scan', lambda m: record('/front/scan', m), qos_profile_sensor_data),
                input_node.create_subscription(LaserScan, '/back/scan', lambda m: record('/back/scan', m), qos_profile_sensor_data),
                input_node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', pose,
                                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
        input_receiver = CallbackReceiver(rclpy, SingleThreadedExecutor, 'velocity_gate_inputs', register_inputs)
        subs = [node.create_subscription(
            GoalStatusArray, '/fairino_navigation/navigate_to_pose/_action/status', goals,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
        def tick():
            nonlocal source_ok, pose_source_ok, last_graph, last_write, reported_failure, chassis_source_ok
            nonlocal last_graph_seconds, last_tick_seconds, execution_source_ok, chassis_sources
            tick_started = time.monotonic()
            if tick_started-last_graph > 1:
                last_graph = tick_started
                publishers = node.get_publishers_info_by_topic('/fairino_navigation/cmd_vel_raw')
                source_ok = len(publishers) == 1 and publishers[0].node_name == 'controller_server' and publishers[0].node_namespace == '/fairino_navigation'
                pose_source_ok = [p.node_name for p in node.get_publishers_info_by_topic('/amcl_pose')] == ['amcl']
                execution_publishers = node.get_publishers_info_by_topic('/fairino_navigation/execution_state/navigate_to_pose')
                execution_source_ok = (len(execution_publishers) == 1 and
                                       execution_publishers[0].node_name == 'bt_navigator_navigate_to_pose_rclcpp_node' and
                                       execution_publishers[0].node_namespace == '/fairino_navigation')
                if gate.execution is not None and not execution_source_ok:
                    gate.fail('导航执行状态发布器缺失或来源不唯一；本次目标已停止', time.monotonic())
                chassis_publishers = node.get_publishers_info_by_topic('/fairino_navigation/chassis_motion_state')
                discovered = publisher_state(chassis_publishers)
                if discovered != chassis_sources:
                    print(json.dumps(dict(event='chassis_feedback_sources', time=time.time(),
                                          **discovered), ensure_ascii=False), flush=True)
                chassis_sources = discovered
                chassis_source_ok = discovered['source_ok']
                inbox.set_source(chassis_source_ok, discovered)
                last_graph_seconds = time.monotonic()-last_graph
            state = read_session()
            with sensor_lock:
                monitor.set_session(state)
            try:
                root = Path.home()/'.local/share/fairino-odometry'
                odometry = json.loads((root/'status.json').read_text())
                odometry['epoch'] = (root/'epoch').read_text().strip()
                if abs(time.time()-odometry['time']) > 2:
                    odometry = {}
            except (OSError, ValueError, KeyError):
                odometry = {}
            ros_time = lambda: node.get_clock().now().nanoseconds*1e-9
            def read_tf():
                return localization_tf_health(tf, ros_time(), state.get('initial_pose_stamp'), Time())
            token = (state.get('id'), state.get('initial_pose_stamp'), odometry.get('epoch'))
            def snapshot():
                # TF/graph/file work runs without the input lock so reception
                # continues. Only the final in-memory health snapshot is locked.
                tf_health = read_tf()
                with sensor_lock:
                    result = readiness_snapshot(monitor, state, odometry, topics, lambda: tf_health,
                                                pose_source_ok, time.monotonic, ros_time, time.time)
                if input_receiver.error:
                    result['ready'] = False
                    result['health'].update(ready=False, reason='导航数据接收线程异常：'+input_receiver.error)
                return result
            msg = Twist()
            current = gate_output(
                gate, lifecycle, token, source_ok, snapshot, time.monotonic,
                lambda now, now_ros: chassis_receiver.check(gate, chassis_source_ok))
            def publish_velocity(values):
                msg.linear.x, msg.linear.y, msg.angular.z = values
                pub.publish(msg)
            # Input callbacks may change the gate after health checks. Recheck
            # and publish under the short gate lock; never lock it across I/O/TF.
            output = publish_checked(gate, current, token, source_ok, publish_velocity, time.monotonic)
            current['output'] = output
            if gate.goal is None and not gate.failure and not current['chassis_motion']['ready']:
                gate.reason = current['chassis_motion']['reason']
            now, ready, stack_ready = current['now'], current['ready'], current['stack_ready']
            health, ages, tf_health = current['health'], current['topic_ages'], current['tf']
            timing = {'tick_to_publish_seconds': time.monotonic()-tick_started,
                      'previous_tick_seconds': last_tick_seconds,
                      'last_graph_seconds': last_graph_seconds,
                      'last_graph_age_seconds': now-last_graph,
                      'lifecycle_seconds': current['lifecycle_seconds'],
                      'sample_monotonic': now, 'sample_ros_time': current['now_ros']}
            if gate.failure and gate.failure['goal'] != reported_failure:
                reported_failure = gate.failure['goal']
                event = {'event': 'navigation_protective_stop', 'time': time.time(),
                         **gate.failure, 'execution': gate.execution, 'topic_ages': ages, 'session_token': list(token),
                         'localization_reason': health['reason'], 'source_ok': source_ok,
                         'tf': tf_health, 'localization_health': health, 'timing': timing,
                         'tf_receiver': receiver.status(), 'chassis_receiver': chassis_receiver.status(),
                         'input_receiver': input_receiver.status(),
                         'chassis_motion': current['chassis_motion'],
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
                data = {'time': time.time(), 'localization_ready': ready, 'controller_ready': source_ok and stack_ready and current['chassis_motion']['ready'], 'session': state.get('id'),
                        'prepared': lifecycle.prepared(now), 'run_id': lifecycle.run_id,
                        'activation': lifecycle.activation, 'session_token': list(token),
                        'lifecycle_states': {n: v[0] for n, v in lifecycle.states.items()},
                        'execution': gate.execution,
                        'goal': gate.goal, 'last_goal': last_goal, 'failed_goal': gate.failed_goal, 'failure': gate.failure, 'reason': gate.reason,
                        'localization_reason': health['reason'], 'command': output, 'topic_ages': ages,
                        'tf': tf_health, 'localization_health': health, 'timing': timing,
                        'tf_receiver': receiver.status(), 'chassis_receiver': chassis_receiver.status(),
                         'input_receiver': input_receiver.status(),
                         'chassis_motion': current['chassis_motion'],
                        'linear_limit': DEFAULT_LINEAR, 'angular_limit': DEFAULT_ANGULAR}
                tmp = HERE/'status.tmp'
                tmp.write_text(json.dumps(data, ensure_ascii=False))
                tmp.replace(HERE/'status.json')
            last_tick_seconds = time.monotonic()-tick_started
        timer = node.create_timer(.05, tick)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if pub is not None and rclpy.ok():
                pub.publish(Twist())
        finally:
            try:
                try:
                    try:
                        if input_receiver is not None:
                            input_receiver.close()
                    finally:
                        if chassis_receiver is not None:
                            chassis_receiver.close()
                finally:
                    if receiver is not None:
                        receiver.close()
            finally:
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()



if __name__ == '__main__':
    main()
