"""Exercise the real mapping manager/SLAM only on loopback ROS domain 197."""
import collections
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def verify(runtime, profile, manager, reader=None):
    if os.environ.get('ROS_DOMAIN_ID') != '197' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Mapping verification requires isolated loopback domain 197')
    import rclpy
    import tf2_ros
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import Odometry, OccupancyGrid
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import LaserScan
    from slam_toolbox.srv import Pause, SerializePoseGraph

    rclpy.init()
    node = rclpy.create_node('isolated_mapping_runtime_verification')
    odom = node.create_publisher(Odometry, '/odom', 20)
    raw = node.create_publisher(Odometry, '/eight_dof_chassis_controller/odom', 20)
    scans = node.create_publisher(LaserScan, '/scan', 20)
    broadcaster = tf2_ros.TransformBroadcaster(node)
    static = tf2_ros.StaticTransformBroadcaster(node)
    base = TransformStamped()
    base.header.frame_id, base.child_frame_id = 'base_footprint', 'base_link'
    base.transform.translation.z, base.transform.rotation.w = .065, 1.
    static.sendTransform(base)
    poses, grids = [], []
    phase = 'initial'

    def receive_pose(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        poses.append({'phase': phase, 'stamp': message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                      'pose': [p.x, p.y, math.atan2(2*q.w*q.z, 1-2*q.z*q.z)]})

    subscriptions = [
        node.create_subscription(PoseWithCovarianceStamped, '/pose', receive_pose, 20),
        node.create_subscription(OccupancyGrid, '/map', lambda m: grids.append((m.info.width, m.info.height)), 10)]
    raw_on = True
    pose = (0., 0., 0.)
    result = {'domain': 197, 'loopback_only': True, 'checks': [], 'phase_counts': {}}
    process = None
    reader_process = None
    previous_pose, previous_time = None, None
    with tempfile.TemporaryDirectory(prefix='fairino-mapping-verify-') as temp:
        root = Path(temp)
        epoch = root/'odom-epoch'
        epoch.write_text('synthetic-session')
        status = root/'odom-status.json'
        config = root/'mapping.yaml'
        config.write_text(profile)

        def step(seconds, name=None, trajectory=None):
            nonlocal phase, pose, previous_pose, previous_time
            if name:
                phase = name
            started = time.monotonic()
            while time.monotonic()-started < seconds:
                if trajectory:
                    pose = trajectory(min(1., (time.monotonic()-started)/seconds))
                x, y, heading = pose
                sampled_at = time.monotonic()
                stamp = node.get_clock().now().to_msg()
                temporary = status.with_suffix('.tmp')
                temporary.write_text(json.dumps({'time': time.time(), 'mode': 'raw', 'running': raw_on,
                                                'source': '/eight_dof_chassis_controller/odom'}))
                temporary.replace(status)
                if raw_on:
                    message = Odometry()
                    message.header.frame_id, message.child_frame_id = 'odom', 'base_footprint'
                    message.header.stamp = stamp
                    message.pose.pose.position.x, message.pose.pose.position.y = float(x), float(y)
                    message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
                    if previous_pose is not None and sampled_at > previous_time:
                        dx, dy = x-previous_pose[0], y-previous_pose[1]
                        dt = sampled_at-previous_time
                        message.twist.twist.linear.x = (math.cos(heading)*dx+math.sin(heading)*dy)/dt
                        message.twist.twist.linear.y = (-math.sin(heading)*dx+math.cos(heading)*dy)/dt
                        dh = heading-previous_pose[2]
                        message.twist.twist.angular.z = math.atan2(math.sin(dh), math.cos(dh))/dt
                    raw.publish(message)
                    odom.publish(message)
                    tf = TransformStamped()
                    tf.header, tf.child_frame_id = message.header, 'base_footprint'
                    tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
                    tf.transform.rotation = message.pose.pose.orientation
                    broadcaster.sendTransform(tf)
                previous_pose, previous_time = pose, sampled_at
                scan = LaserScan()
                scan.header.frame_id, scan.header.stamp = 'base_link', stamp
                # Match the real merged /scan angular spacing and point count.
                scan.angle_min, scan.angle_increment = -math.pi, .0038
                scan.angle_max = math.pi
                scan.range_min, scan.range_max, scan.scan_time = .1, 20., .05
                ranges = []
                for index in range(1654):
                    angle = heading+scan.angle_min+index*scan.angle_increment
                    dx, dy = math.cos(angle), math.sin(angle)
                    distances = []
                    if abs(dx) > 1e-8:
                        distances += [(wall-x)/dx for wall in (-3., 5.) if (wall-x)/dx > 0]
                    if abs(dy) > 1e-8:
                        distances += [(wall-y)/dy for wall in (-2., 4.) if (wall-y)/dy > 0]
                    ranges.append(min(distances))
                scan.ranges = ranges
                scans.publish(scan)
                until = time.monotonic()+.04
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.005)

        def call(service, kind, request):
            client = node.create_client(kind, service)
            try:
                deadline = time.monotonic()+8
                while not client.service_is_ready() and time.monotonic() < deadline:
                    step(.05)
                if not client.service_is_ready():
                    raise RuntimeError('Isolated service missing: ' + service)
                future = client.call_async(request)
                while not future.done() and time.monotonic() < deadline:
                    step(.05)
                if not future.done():
                    raise RuntimeError('Isolated service timed out: ' + service)
                return future.result()
            finally:
                node.destroy_client(client)

        def params(names):
            request = GetParameters.Request()
            request.names = names
            return call('/slam_toolbox/get_parameters', GetParameters, request).values

        wrapper = manager + '\n'
        # The original module's __main__ block must not run. All writable
        # state and the odometry epoch are confined to this temporary folder.
        launch = ('namespace={"__name__":"isolated_mapping_manager"};exec(' + repr(wrapper) + ',namespace);'
                  'from pathlib import Path;namespace.update(' +
                  'RUNTIME=Path(' + repr(str(root/'manager')) + '),'
                  'MAPPING_PROFILE=Path(' + repr(str(config)) + '),'
                  'SLAM_RUNTIME=Path(' + repr(runtime) + '),'
                  'ODOMETRY_EPOCH=Path(' + repr(str(epoch)) + '),'
                  'ODOMETRY_STATUS=Path(' + repr(str(status)) + '),'
                  'MAPPING_EPOCH=Path(' + repr(str(root/'manager/mapping-epoch')) + '));'
                  'namespace["serve_mapping"]()')
        log_path = root/'mapping.log'
        reader_path = root/'reader.log'
        with log_path.open('w') as log, reader_path.open('w') as reader_log:
            try:
                step(.3)
                if reader:
                    wrapper = ('namespace={"__name__":"isolated_map_reader"};exec(' + repr(reader) + ',namespace);'
                               'from pathlib import Path;import json;namespace.update('
                               'ODOMETRY_STATUS=Path(' + repr(str(status)) + '),'
                               'MAPPING_EPOCH=Path(' + repr(str(root/'manager/mapping-epoch')) + '));'
                               'namespace["emit"]=lambda p:print("READER_FRAME "+json.dumps('
                               '{k:p.get(k) for k in ["type","map_mode","scan_match","mapping_paused","pose","odometry_mode"]}),flush=True) '
                               'if p.get("type")=="frame" else None;namespace["main"]()')
                    reader_process = subprocess.Popen([sys.executable, '-u', '-c', wrapper], stdout=reader_log,
                                                      stderr=subprocess.STDOUT, start_new_session=True)
                process = subprocess.Popen([sys.executable, '-u', '-c', launch], stdout=log,
                                           stderr=subprocess.STDOUT, start_new_session=True)
                step(5, 'stationary_initial')
                values = params(['fairino_scan_gate', 'minimum_travel_distance', 'minimum_travel_heading', 'map_update_interval'])
                assert values[0].string_value == 'backend_motion_gate_v1', values
                assert abs(values[1].double_value-.1) < 1e-9
                assert abs(values[2].double_value-math.radians(5)) < 1e-9
                assert values[3].double_value == 1.
                assert len(poses) == 1, ('stationary initial keyframes', len(poses))
                result['checks'].extend(['runtime_marker_and_profile', 'stationary_not_repeated'])
                result['phase_counts']['initial'] = len(poses)
                step(3, 'forward_0.4m', lambda t: (.4*t, 0., 0.))
                assert len(poses) >= 3, ('short straight movement not matched', poses)
                before = len(poses)
                result['phase_counts']['forward'] = before
                step(3, 'return', lambda t: (.4*(1-t), 0., 0.))
                assert len(poses) >= before+2, ('return not matched', poses)
                before = len(poses)
                result['phase_counts']['return'] = before
                step(4, 'rotate_in_place', lambda t: (0., 0., .8*t))
                assert len(poses) >= before+4, ('in-place rotation not matched', poses)
                result['phase_counts']['rotation'] = len(poses)
                result['checks'].extend(['short_forward_and_return', 'in_place_rotation'])
                pose = (0., 0., .8)
                step(.5, 'stationary_after_turn')
                before = len(poses)
                step(2)
                assert len(poses) == before, ('stationary repeats after turn', len(poses), before)
                result['checks'].append('stationary_after_turn_not_repeated')
                call('/slam_toolbox/pause_new_measurements', Pause, Pause.Request())
                assert params(['paused_new_measurements'])[0].bool_value
                before = len(poses)
                step(2, 'paused_translation', lambda t: (.25*t, 0., .8))
                assert len(poses) == before, 'paused mapping still accepted scans'
                call('/slam_toolbox/pause_new_measurements', Pause, Pause.Request())
                step(1, 'resumed')
                assert len(poses) > before, 'explicit resume did not match new pose'
                result['checks'].append('pause_and_explicit_resume')
                request = SerializePoseGraph.Request()
                request.filename = str(root/'synthetic-map')
                assert call('/slam_toolbox/serialize_map', SerializePoseGraph, request).result == 0
                assert (root/'synthetic-map.posegraph').is_file() and (root/'synthetic-map.data').is_file()
                result['checks'].append('serialize_synthetic_pose_graph')
                assert len(grids) >= 8 and all(w > 0 and h > 0 for w, h in grids), grids
                result['map_publications'] = len(grids)
                result['checks'].append('one_second_map_publication')
                raw_on = False
                step(2.5, 'wheel_dropout')
                assert params(['paused_new_measurements'])[0].bool_value, 'watchdog failed to pause on dropout'
                result['checks'].append('input_dropout_auto_pauses')
            finally:
                if process is not None and process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
                if reader_process is not None and reader_process.poll() is None:
                    reader_process.send_signal(signal.SIGINT)
                    try:
                        reader_process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(reader_process.pid, signal.SIGKILL)
                        reader_process.wait(timeout=5)
                result['log_tail'] = log_path.read_text()[-2500:]
                node.destroy_node()
                rclpy.shutdown()
            assert process.returncode == 0, ('manager shutdown failed', process.returncode, result['log_tail'])
            if reader_process is not None:
                assert reader_process.returncode == 0, reader_path.read_text()[-2000:]
                frames = [json.loads(line[len('READER_FRAME '):]) for line in reader_path.read_text().splitlines()
                          if line.startswith('READER_FRAME ')]
                matches = [frame['scan_match'] for frame in frames if frame.get('scan_match')]
                assert any(match['source_confirmed'] and match['count_observed'] >= 3 and match['state'] == 'recent'
                           for match in matches), ('reader did not see fresh matches', frames[-3:])
                assert any(match['state'] == 'waiting_motion' for match in matches), 'reader did not recognize stationary keyframe wait'
                assert any(frame.get('mapping_paused') for frame in frames), 'reader did not report pause'
                assert any(frame.get('pose') for frame in frames), 'reader did not transform robot pose'
                result['checks'].append('viewer_reads_matches_stationary_pause_and_tf')
                result['viewer_frames'] = len(frames)
        result['checks'].append('paused_manager_clean_shutdown')
        result['phase_counts']['final'] = len(poses)
        result['passed'] = True
    return result
