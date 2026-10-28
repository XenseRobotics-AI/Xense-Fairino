#!/usr/bin/env python3
"""Stream the existing map, TF, scans and odometry over stdout. Read only."""
import argparse
import array
import base64
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import time
import zlib

PREFIX = 'MAP_VIEWER '
ODOMETRY_STATUS = Path.home()/'.local/share/fairino-odometry/status.json'
MAPPING_EPOCH = Path.home()/'.local/share/fairino-map-control/odometry-epoch'
NAVIGATION_STATUS = Path.home()/'.local/share/fairino-navigation/status.json'
PATH_TOPICS = {'global': '/fairino_navigation/received_global_plan',
               'local': '/fairino_navigation/local_plan'}


class ScanMatchMonitor:
    """Track accepted SLAM poses separately from map/TF publication."""
    def __init__(self):
        self.session = None
        self.started = 0.
        self.latest = self.odometry = None
        self.count = 0
        self.observed_since = time.monotonic()
        self.travel = self.rotation = 0.
        self.distance_threshold = .1
        self.angle_threshold = math.radians(5)

    def start_session(self, token, started):
        if token != self.session:
            self.session, self.started = token, started
            self.latest = None
            self.count = 0
            self.observed_since = time.monotonic()
            self.travel = self.rotation = 0.

    def on_odometry(self, pose, twist=None):
        if not all(math.isfinite(x) for x in pose):
            return
        moving = twist is None or (all(math.isfinite(x) for x in twist)
                                    and (math.hypot(*twist[:2]) >= .005 or abs(twist[2]) >= .005))
        if self.odometry is not None and moving:
            self.travel += math.hypot(pose[0]-self.odometry[0], pose[1]-self.odometry[1])
            delta = pose[2]-self.odometry[2]
            self.rotation += abs(math.atan2(math.sin(delta), math.cos(delta)))
        self.odometry = tuple(pose)

    def accept(self, stamp, stamp_age, pose, received):
        if (not all(math.isfinite(x) for x in [stamp, stamp_age, received, *pose])
                or stamp <= 0 or stamp < self.started or stamp_age < -.5 or stamp_age > 2
                or (self.latest and stamp <= self.latest['stamp'])):
            return False
        self.latest = {'stamp': stamp, 'stamp_age': stamp_age,
                       'pose': list(pose), 'received': received}
        self.count += 1
        self.travel = self.rotation = 0.
        return True

    def status(self, now):
        result = {'count_observed': self.count, 'age': None, 'state': 'waiting',
                  'waiting_age': max(0., now-self.observed_since),
                  'travel_since_match': self.travel, 'rotation_since_match': self.rotation,
                  'distance_threshold': self.distance_threshold, 'angle_threshold': self.angle_threshold}
        if self.latest:
            elapsed = max(0., now-self.latest['received'])
            age = max(0., elapsed, self.latest['stamp_age']+elapsed)
            moved = self.travel >= self.distance_threshold or self.rotation >= self.angle_threshold
            result.update(age=age, stamp=self.latest['stamp'], pose=self.latest['pose'],
                          state='recent' if age <= 2 else 'stale' if moved else 'waiting_motion')
        return result



class LocalizationMonitor:
    """Session-bound AMCL estimates; stationary poses need not republish."""
    def __init__(self):
        self.session = {}
        self.sample = None
        self.odometry = None
        self.travel = self.rotation = 0.

    def set_session(self, session):
        if (session.get('id'), session.get('initial_pose_stamp')) != (
                self.session.get('id'), self.session.get('initial_pose_stamp')):
            self.sample = None
            self.travel = self.rotation = 0.
            self.odometry = None
        self.session = dict(session)

    def on_odometry(self, pose, twist):
        if not all(math.isfinite(v) for v in (*pose, *twist)):
            return
        if self.odometry is not None and (math.hypot(*twist[:2]) >= .005 or abs(twist[2]) >= .005):
            self.travel += math.hypot(pose[0]-self.odometry[0], pose[1]-self.odometry[1])
            d = pose[2]-self.odometry[2]
            self.rotation += abs(math.atan2(math.sin(d), math.cos(d)))
        self.odometry = tuple(pose)

    def accept(self, stamp, age, pose, covariance, frame):
        cutoff = self.session.get('initial_pose_stamp')
        values = [stamp, age, *pose, *covariance]
        if (not self.session.get('active') or cutoff is None or frame != 'map'
                or len(covariance) != 36 or not all(math.isfinite(v) for v in values)
                or stamp < cutoff or age < -.5
                or any(covariance[i] < 0 for i in (0, 7, 35))
                or (self.sample and stamp <= self.sample['stamp'])):
            return False
        # Largest XY covariance eigenvalue accounts for an oblique uncertainty ellipse.
        xx, yy, xy = covariance[0], covariance[7], (covariance[1]+covariance[6])/2
        if xy*xy > xx*yy + 1e-9:
            return False
        variance = .5*(xx+yy+math.sqrt((xx-yy)**2+4*xy**2))
        self.sample = {'stamp': stamp, 'pose': list(pose),
                       'matches': (self.sample or {}).get('matches', 0)+1,
                       'covariance_xy': [xx, xy, yy],
                       'xy_std': math.sqrt(max(0., variance)),
                       'yaw_std': math.sqrt(covariance[35])}
        self.travel = self.rotation = 0.
        return True

    def status(self, now_ros, odometry, topics, tf_ready, amcl_active, source_confirmed):
        result = dict(self.sample or {}, id=self.session.get('id'), active=self.session.get('active', False),
                      ready=False, estimate_visible=False, state='stopped',
                      initialization=dict(self.session.get('initialization') or {}),
                      reason=self.session.get('reason', '定位尚未启动'))
        if not result['active']:
            return result
        result['state'] = 'waiting'
        if not amcl_active or not source_confirmed:
            result['reason'] = '等待唯一的活动 AMCL 节点'
        elif abs(now_ros-self.session.get('watchdog_time', 0)) > 5:
            result['reason'] = '定位监视服务未更新，请停止后重新启动定位'
        elif not odometry.get('running') or odometry.get('epoch') != self.session.get('epoch'):
            result['reason'] = '里程计未就绪或会话已变化，请重新启动定位'
        elif any(topics.get(t, {}).get('age') is None or topics[t]['age'] > .75
                 for t in ('/scan', '/odom')):
            result['reason'] = '雷达或里程计断流，定位不可用'
        elif self.session.get('initial_pose_stamp') is None:
            result['reason'] = '请在地图上设置初始位置与朝向'
        elif not self.sample:
            result['reason'] = '等待本次初始位姿后的 AMCL 雷达匹配'
        elif not tf_ready:
            result['reason'] = '定位 TF 未就绪或已过期'
        elif now_ros-self.sample['stamp'] > 2 and (self.travel >= .1 or self.rotation >= math.radians(5)):
            result.update(state='stale', reason='运动后未获新定位匹配，当前主要随里程计推算')
        elif result['initialization'].get('state') == 'refining':
            stage = result['initialization']
            result.update(state='initializing', estimate_visible=True,
                          reason=f"静止匹配请求 {stage.get('updates', 0)}/{stage.get('maximum', 6)}；橙色箭头为候选位置")
        elif result['initialization'].get('state') == 'failed':
            result.update(state='uncertain', estimate_visible=True,
                          reason=result['initialization'].get('note', '初始化匹配失败，请重新设置初始位姿'))
        elif self.sample['xy_std'] > .5 or self.sample['yaw_std'] > math.radians(20):
            result.update(state='uncertain', estimate_visible=True,
                          reason='候选定位尚未收敛，请核对初始范围与雷达重合；当前不可导航')
        else:
            result.update(ready=True, estimate_visible=True, state='tracking',
                          reason='定位估计正在更新，请核对雷达与墙面重合' if now_ros-self.sample['stamp'] <= 2
                          else '静止或移动未达更新阈值，保留本次定位估计')
        return result


def display_session(mode, source, mapping_session, localization, odometry_epoch):
    """Stable identity, independent of map updates, TF poses and poll times."""
    return [mode, source, mapping_session if mode == 'mapping' else None,
            localization.get('id') if mode == 'saved' else None,
            localization.get('initial_pose_stamp') if mode == 'saved' else None,
            odometry_epoch]


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def bounded_points(points, maximum, width):
    """Keep real, evenly spaced samples including endpoints; never synthesize."""
    if not points or any(len(p) != width or not all(finite_number(v) for v in p) for p in points):
        raise ValueError('观测点为空或含无效坐标')
    count = len(points)
    indices = (range(count) if count <= maximum else
               [i*(count-1)//(maximum-1) for i in range(maximum)])
    return [list(points[i]) for i in indices]


def transform_points(points, transform):
    """Rigid XY transform evaluated at the original message timestamp."""
    if len(transform) != 3 or not all(finite_number(v) for v in transform):
        raise ValueError('路径 TF 含无效坐标')
    x, y, heading = transform
    c, s = math.cos(heading), math.sin(heading)
    return [[x+c*px-s*py, y+s*px+c*py] for px, py in points]


class DisplayTelemetry:
    """Read-only overlays, isolated from localization and motion decisions."""
    def __init__(self):
        self.session = None
        self.active = False
        self.cutoff = None
        self.expected_navigation_session = None
        self.particle = None
        self.particle_reason = '等待本次定位的 AMCL 粒子云'
        self.particle_odometry = None
        self.travel = self.rotation = 0.
        self.paths = {}
        self.path_reasons = {kind: '导航尚未执行' for kind in PATH_TOPICS}
        self.navigation = None
        self.navigation_cutoff = None

    def set_session(self, token, active, cutoff, navigation_session):
        if token != self.session:
            self.particle = None
            self.particle_reason = '等待本次定位的 AMCL 粒子云'
            self.particle_odometry = None
            self.travel = self.rotation = 0.
            self.paths.clear()
            self.navigation = None
            self.navigation_cutoff = None
            self.path_reasons = {kind: '地图或定位会话已变化，等待新路径' for kind in PATH_TOPICS}
        self.session = list(token)
        self.active = bool(active and finite_number(cutoff) and cutoff > 0)
        self.cutoff = cutoff
        self.expected_navigation_session = list(navigation_session)
        if not self.active:
            self.particle = None
            self.paths.clear()
            self.particle_reason = '请启动定位并设置初始位姿'

    def on_odometry(self, pose, twist):
        if not all(finite_number(v) for v in [*pose, *twist]):
            return
        moving = math.hypot(*twist[:2]) >= .005 or abs(twist[2]) >= .005
        if self.particle_odometry is not None and moving:
            self.travel += math.hypot(pose[0]-self.particle_odometry[0], pose[1]-self.particle_odometry[1])
            delta = pose[2]-self.particle_odometry[2]
            self.rotation += abs(math.atan2(math.sin(delta), math.cos(delta)))
        self.particle_odometry = tuple(pose)

    def reject_particles(self, reason):
        self.particle = None
        self.particle_reason = reason

    def accept_particles(self, points, stamp, frame, now_ros, now, source_confirmed):
        if not self.active:
            self.reject_particles('请启动定位并设置初始位姿')
            return False
        if not source_confirmed:
            self.reject_particles('等待唯一 AMCL 粒子云发布器')
            return False
        if (frame != 'map' or not finite_number(stamp) or stamp < self.cutoff
                or not -.5 <= now_ros-stamp <= 2):
            self.reject_particles('粒子云坐标系、时间戳或定位会话不匹配')
            return False
        if self.particle and stamp <= self.particle['stamp']:
            return False
        try:
            sampled = bounded_points(points, 600, 4)
            if any(p[3] < 0 for p in points):
                raise ValueError('粒子权重无效')
        except (TypeError, ValueError) as exc:
            self.reject_particles(str(exc))
            return False
        self.particle = {'points': sampled, 'count': len(points), 'stamp': stamp, 'frame': frame,
                         'received': now, 'stamp_age': now_ros-stamp}
        self.travel = self.rotation = 0.
        return True

    def set_navigation(self, state, now_ros, now_wall):
        state = state if isinstance(state, dict) else {}
        stamp = state.get('time')
        fresh = finite_number(stamp) and abs(now_wall-stamp) <= 2
        goal, run_id = state.get('goal'), state.get('run_id')
        live = (self.active and fresh and isinstance(goal, str) and bool(goal)
                and isinstance(run_id, str) and bool(run_id)
                and state.get('controller_ready') and state.get('localization_ready')
                and state.get('session_token') == self.expected_navigation_session)
        token = (run_id, goal) if live else None
        if token != self.navigation:
            self.paths.clear()
            self.path_reasons = {kind: '等待本次目标的控制器路径' if live else '导航未执行或状态已过期'
                                 for kind in PATH_TOPICS}
            # Path has no goal UUID. Only accept observations newer than when
            # this active-goal context was first seen; wait for the next replan.
            self.navigation_cutoff = now_ros if live else None
        self.navigation = token

    def reject_path(self, kind, reason):
        self.paths.pop(kind, None)
        self.path_reasons[kind] = reason

    def accept_path(self, kind, points, stamp, frame, now_ros, now, source_confirmed,
                    transform=None):
        if not self.navigation:
            self.reject_path(kind, '导航未执行或状态已过期')
            return False
        if not source_confirmed:
            self.reject_path(kind, '等待唯一的导航控制器路径发布器')
            return False
        maximum_age = 3. if kind == 'global' else .75
        if (not finite_number(stamp) or stamp < max(self.cutoff, self.navigation_cutoff)
                or not -.5 <= now_ros-stamp <= maximum_age):
            self.reject_path(kind, '路径时间戳过期或早于本次目标')
            return False
        # DWB's pruned global plan can retain its original timestamp. Receipt
        # times alone must not make an old plan look newly planned.
        if self.paths.get(kind) and stamp < self.paths[kind]['stamp']:
            return False
        try:
            sampled = bounded_points(points, 400, 2)
            if frame != 'map':
                if not frame or transform is None:
                    raise ValueError('等待路径采样时刻到 map 的 TF')
                sampled = transform_points(sampled, transform)
        except (TypeError, ValueError) as exc:
            self.reject_path(kind, str(exc))
            return False
        self.paths[kind] = {'points': sampled, 'count': len(points), 'stamp': stamp,
                            'frame': 'map', 'source_frame': frame, 'received': now,
                            'stamp_age': now_ros-stamp}
        return True

    @staticmethod
    def age(sample, now):
        elapsed = max(0., now-sample['received'])
        return max(elapsed, abs(sample['stamp_age']+elapsed))

    def snapshot(self, now, localization_visible, localization_reason, particle_source,
                 path_sources):
        particles = {'points': [], 'count': 0, 'age': None, 'valid': False,
                     'reason': self.particle_reason, 'state': 'waiting', 'frame': 'map',
                     'session_id': self.expected_navigation_session[0] if self.expected_navigation_session else None}
        sample = self.particle
        if sample:
            age = self.age(sample, now)
            particles.update(count=sample['count'], stamp=sample['stamp'], age=age)
            if not self.active or not localization_visible:
                particles.update(reason=localization_reason, state='unavailable')
            elif not particle_source:
                particles.update(reason='AMCL 粒子云发布器不可用或不唯一', state='unavailable')
            elif age > 2 and (self.travel >= .1 or self.rotation >= math.radians(5)):
                particles.update(reason='运动后粒子云未更新', state='stale')
            else:
                particles.update(points=sample['points'], valid=True,
                                 state='fresh' if age <= 2 else 'retained',
                                 reason='AMCL 实际粒子' if age <= 2 else '保留上次粒子云；静止时 AMCL 不必持续发布')
        paths = {}
        for kind, topic in PATH_TOPICS.items():
            result = {'points': [], 'age': None, 'valid': False, 'reason': self.path_reasons[kind],
                      'frame': 'map', 'topic': topic}
            sample = self.paths.get(kind)
            if sample:
                age = self.age(sample, now)
                result.update(stamp=sample['stamp'], age=age, count=sample['count'],
                              source_frame=sample['source_frame'])
                if not self.navigation:
                    result['reason'] = '导航未执行或状态已过期'
                elif not localization_visible:
                    result['reason'] = localization_reason
                elif not path_sources.get(kind):
                    result['reason'] = '导航控制器路径发布器不可用或不唯一'
                elif age > (3. if kind == 'global' else .75):
                    result['reason'] = '控制器路径已过期'
                else:
                    result.update(points=sample['points'], valid=True,
                                  reason='控制器当前路径' if kind == 'global' else '控制器局部预测轨迹',
                                  run_id=self.navigation[0], goal_id=self.navigation[1],
                                  association='active_goal_context')
            paths[kind] = result
        return particles, paths


def emit(payload):
    print(PREFIX + json.dumps(payload, separators=(',', ':'), allow_nan=False), flush=True)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def observation_yaw(q):
    values = [q.x, q.y, q.z, q.w]
    if not all(finite_number(v) for v in values):
        raise ValueError('观测姿态含无效四元数')
    norm = sum(v*v for v in values)
    if norm < 1e-12:
        raise ValueError('观测姿态为零四元数')
    x, y, z, w = [v/math.sqrt(norm) for v in values]
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def path_observation(msg, lookup_transform):
    """Parse a ROS Path; lookup receives the original ROS stamp, never latest."""
    frame = msg.header.frame_id
    stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    if frame not in ('map', 'odom') or not finite_number(stamp) or stamp <= 0:
        raise ValueError('路径坐标系或时间戳不可用')
    if any(p.header.frame_id and p.header.frame_id != frame for p in msg.poses):
        raise ValueError('路径内含不一致的坐标系')
    transform = None
    if frame != 'map':
        try:
            tf = lookup_transform('map', frame, msg.header.stamp)
            transform = [tf.transform.translation.x, tf.transform.translation.y,
                         observation_yaw(tf.transform.rotation)]
        except Exception as exc:
            raise ValueError('等待路径采样时刻到 map 的 TF') from exc
    return [[p.pose.position.x, p.pose.position.y] for p in msg.poses], stamp, frame, transform


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=0)
    args = parser.parse_args()
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rcl_interfaces.srv import GetParameters
    import tf2_ros

    rclpy.init()
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = rclpy.create_node('fairino_map_viewer_readonly_' + str(os.getpid()))
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)
    subscriptions = []
    odom_topics = ['/fairino_keyboard/odom', '/eight_dof_chassis_controller/odom', '/odom']
    scan_topics = ['/front/scan', '/back/scan']
    history = {t: deque(maxlen=200) for t in ['/map', '/scan'] + odom_topics + scan_topics}
    odom = {}
    scans = {t: deque(maxlen=12) for t in scan_topics}
    map_frame = 'map'
    map_signature = None
    ekf_source = None
    localization = LocalizationMonitor()
    localization_state = {}
    amcl_state = None
    amcl_checked = 0.
    amcl_publishers = []
    matches = ScanMatchMonitor()
    mapping_parameters = {}
    match_publishers = []
    telemetry = DisplayTelemetry()
    particle_source = False
    pending_particles = None
    particles_available = True
    path_sources = {kind: False for kind in PATH_TOPICS}
    saved_file = Path('/opt/fairino_AMR_project/maps/active/map.yaml')

    def unique_publisher(topic, name, namespace):
        publishers = node.get_publishers_info_by_topic(topic)
        return (len(publishers) == 1 and publishers[0].node_name == name
                and publishers[0].node_namespace == namespace)

    def match_session():
        try:
            stat = MAPPING_EPOCH.stat()
            matches.start_session(stat.st_mtime_ns, stat.st_mtime)
        except OSError:
            pass

    def on_match(msg):
        if msg.header.frame_id != map_frame:
            return
        match_session()
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        age = node.get_clock().now().nanoseconds*1e-9-stamp
        matches.accept(stamp, age, [p.x, p.y, yaw(msg.pose.pose.orientation)], time.monotonic())

    def refresh_localization():
        nonlocal localization_state
        try:
            localization_state = json.loads((MAPPING_EPOCH.parent/'localization.json').read_text())
        except (OSError, ValueError):
            localization_state = {}
        localization.set_session(localization_state)

    def on_localization(msg):
        refresh_localization()
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        localization.accept(stamp, node.get_clock().now().nanoseconds*1e-9-stamp,
                            [p.x, p.y, yaw(msg.pose.pose.orientation)],
                            list(msg.pose.covariance), msg.header.frame_id)

    def refresh_display_session():
        # Read current identities before accepting a latched/queued overlay.
        # The read-only SSH worker owns this cache, never a motion-control node.
        refresh_localization()
        try:
            epoch = (ODOMETRY_STATUS.parent/'epoch').read_text().strip()
        except OSError:
            epoch = None
        odometry_status['epoch'] = epoch
        try:
            mapping_session = MAPPING_EPOCH.stat().st_mtime_ns if mode == 'mapping' else None
        except OSError:
            mapping_session = None
        source = str(saved_file.resolve()) if saved_file.is_file() else None
        token = display_session(mode, source, mapping_session, localization_state, epoch)
        cutoff = localization_state.get('initial_pose_stamp')
        telemetry.set_session(token, mode == 'saved' and localization_state.get('active')
                              and epoch is not None and epoch == localization_state.get('epoch'),
                              cutoff, [localization_state.get('id'), cutoff, epoch])
        try:
            nav_state = json.loads(NAVIGATION_STATUS.read_text())
        except (OSError, ValueError):
            nav_state = {}
        telemetry.set_navigation(nav_state, node.get_clock().now().nanoseconds*1e-9, time.time())
        return token

    def on_particles(msg):
        nonlocal pending_particles
        # Process after refreshing the map mode. The first AMCL cloud can
        # arrive before the initial graph poll, and may be the last while still.
        pending_particles = msg

    def consume_particles(msg):
        nonlocal particle_source
        if not particle_source:
            particle_source = unique_publisher('/particle_cloud', 'amcl', '/')
        try:
            points = [[p.pose.position.x, p.pose.position.y, observation_yaw(p.pose.orientation), p.weight]
                      for p in msg.particles]
        except (AttributeError, TypeError, ValueError) as exc:
            telemetry.reject_particles('粒子云数据不可用：'+str(exc))
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        telemetry.accept_particles(points, stamp, msg.header.frame_id,
                                   node.get_clock().now().nanoseconds*1e-9,
                                   time.monotonic(), particle_source)

    def on_path(kind):
        def receive(msg):
            refresh_display_session()
            if not path_sources[kind]:
                path_sources[kind] = unique_publisher(PATH_TOPICS[kind], 'controller_server', '/fairino_navigation')
            try:
                points, stamp, frame, transform = path_observation(
                    msg, lambda target, source, stamp: buffer.lookup_transform(target, source, Time.from_msg(stamp)))
            except (AttributeError, TypeError, ValueError) as exc:
                telemetry.reject_path(kind, str(exc))
                return
            telemetry.accept_path(kind, points, stamp, frame, node.get_clock().now().nanoseconds*1e-9,
                                  time.monotonic(), path_sources[kind], transform)
        return receive

    def on_map(msg):
        nonlocal map_frame, map_signature
        history['/map'].append(time.monotonic())
        map_frame = msg.header.frame_id
        raw = array.array('b', msg.data).tobytes()
        p = msg.info.origin.position
        origin = [p.x, p.y, yaw(msg.info.origin.orientation)]
        signature = (msg.info.width, msg.info.height, msg.info.resolution, *origin, zlib.crc32(raw))
        if signature == map_signature:
            return
        map_signature = signature
        emit({'type': 'map', 'frame': map_frame, 'width': msg.info.width,
              'height': msg.info.height, 'resolution': msg.info.resolution,
              'origin': origin, 'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
              'data': base64.b64encode(zlib.compress(raw)).decode()})

    def on_odom(topic):
        def receive(msg):
            history[topic].append(time.monotonic())
            p = msg.pose.pose.position
            t = msg.twist.twist
            values = [p.x, p.y, yaw(msg.pose.pose.orientation), t.linear.x,
                      t.linear.y, t.angular.z, msg.pose.covariance[0],
                      msg.pose.covariance[7], max(msg.pose.covariance[0], msg.pose.covariance[7])]
            odom[topic] = dict(zip(['x', 'y', 'yaw', 'vx', 'vy', 'wz', 'x_variance', 'y_variance', 'position_variance'],
                                  [v if math.isfinite(v) else None for v in values]))
            if topic == '/odom':
                matches.on_odometry(values[:3], values[3:6])
                localization.on_odometry(values[:3], values[3:6])
                telemetry.on_odometry(values[:3], values[3:6])
        return receive

    def on_scan(topic):
        def receive(msg):
            now = time.monotonic()
            history[topic].append(now)
            scans[topic].append((now, msg))
        return receive

    subscriptions.append(node.create_subscription(
        OccupancyGrid, '/map', on_map,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    for topic in odom_topics:
        subscriptions.append(node.create_subscription(Odometry, topic, on_odom(topic), 20))
    for topic in scan_topics:
        subscriptions.append(node.create_subscription(LaserScan, topic, on_scan(topic), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', on_localization,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    try:
        from nav2_msgs.msg import ParticleCloud
    except ImportError:
        particles_available = False
    else:
        subscriptions.append(node.create_subscription(
            ParticleCloud, '/particle_cloud', on_particles, qos_profile_sensor_data))
    for kind, topic in PATH_TOPICS.items():
        subscriptions.append(node.create_subscription(NavPath, topic, on_path(kind), 1))
    subscriptions.append(node.create_subscription(
        LaserScan, '/scan', lambda msg: history['/scan'].append(time.monotonic()), qos_profile_sensor_data))
    from lifecycle_msgs.srv import GetState
    amcl_client = node.create_client(GetState, '/amcl/get_state')
    amcl_future = None
    amcl_requested = 0.

    subscriptions.append(node.create_subscription(PoseWithCovarianceStamped, '/pose', on_match, 20))
    params = node.create_client(GetParameters, '/ekf_filter_node/get_parameters')
    slam_params = node.create_client(GetParameters, '/slam_toolbox/get_parameters')
    future = None
    slam_future = None
    mapping_paused = None
    map_publishers = []
    mode = 'unknown'
    mode_since = time.time()
    started = last_sent = time.monotonic()
    last_query = 0
    last_mode_query = 0
    odometry_status = {}
    emit({'type': 'ready', 'mode': 'read_only', 'domain': int(os.environ.get('ROS_DOMAIN_ID', '1'))})
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            if args.duration and now - started >= args.duration:
                break
            if amcl_future is not None and amcl_future.done():
                try:
                    response = amcl_future.result()
                    amcl_state = response.current_state.id if response else None
                except Exception:
                    amcl_state = None
                amcl_checked = now
                amcl_future = None
            elif amcl_future is not None and now-amcl_requested > 3:
                amcl_future.cancel()
                amcl_future = None
                amcl_state = None
            if future is not None and future.done():
                result = future.result()
                if result and result.values:
                    ekf_source = result.values[0].string_value or None
                future = None
            if slam_future is not None and slam_future.done():
                result = slam_future.result()
                mapping_paused = result.values[0].bool_value if result and result.values and result.values[0].type == 1 else None
                if result:
                    for key, value in zip(['minimum_travel_distance', 'minimum_travel_heading'], result.values[1:]):
                        if value.type == 3 and math.isfinite(value.double_value) and value.double_value > 0:
                            mapping_parameters[key] = value.double_value
                    matches.distance_threshold = mapping_parameters.get('minimum_travel_distance', .1)
                    matches.angle_threshold = mapping_parameters.get('minimum_travel_heading', math.radians(5))
                slam_future = None
            if now - last_mode_query > 2:
                last_mode_query = now
                refresh_localization()
                amcl_publishers = [p.node_name for p in node.get_publishers_info_by_topic('/amcl_pose')]
                particle_source = unique_publisher('/particle_cloud', 'amcl', '/')
                path_sources = {kind: unique_publisher(topic, 'controller_server', '/fairino_navigation')
                                for kind, topic in PATH_TOPICS.items()}
                if amcl_future is None and amcl_client.service_is_ready():
                    amcl_future = amcl_client.call_async(GetState.Request())
                    amcl_requested = now
                try:
                    odometry_status = json.loads(ODOMETRY_STATUS.read_text())
                    if abs(time.time()-odometry_status['time']) > 2:
                        odometry_status = {}
                except (OSError, ValueError, KeyError):
                    odometry_status = {}
                try:
                    odometry_status['epoch'] = (ODOMETRY_STATUS.parent/'epoch').read_text().strip()
                except OSError:
                    odometry_status['epoch'] = None
                map_publishers = sorted({p.node_name for p in node.get_publishers_info_by_topic('/map')})
                match_publishers = sorted({p.node_name for p in node.get_publishers_info_by_topic('/pose')})
                new_mode = 'mapping' if 'slam_toolbox' in map_publishers else 'saved' if 'map_server' in map_publishers else 'empty' if not saved_file.is_file() else 'unknown'
                if new_mode != mode:
                    mode = new_mode
                    mode_since = time.time()
                    mapping_paused = None
                    if mode == 'empty':
                        map_signature = None
                        emit({'type': 'clear_map'})
                if mode == 'mapping':
                    match_session()
                if mode == 'mapping' and slam_future is None and slam_params.service_is_ready():
                    request = GetParameters.Request()
                    request.names = ['paused_new_measurements', 'minimum_travel_distance', 'minimum_travel_heading']
                    slam_future = slam_params.call_async(request)
            if future is None and now - last_query > 10 and params.service_is_ready():
                request = GetParameters.Request()
                request.names = ['odom0']
                future = params.call_async(request)
                last_query = now
            if now - last_sent < .25:
                continue
            last_sent = now
            current_display_session = refresh_display_session()
            if pending_particles is not None:
                consume_particles(pending_particles)
                pending_particles = None
            elif not particles_available:
                telemetry.reject_particles('此 ROS 环境缺少 nav2_msgs/ParticleCloud')
            topics = {}
            for topic, times in history.items():
                recent = [t for t in times if now - t < 3]
                hz = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 else 0
                topics[topic] = {'age': now - times[-1] if times else None, 'hz': hz,
                                 'latest': odom.get(topic)}
            pose = None
            tf_error = None
            localized = False
            try:
                tf = buffer.lookup_transform(map_frame, 'base_footprint', Time())
                age = (node.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
                if abs(age) <= 1 and tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9 >= mode_since:
                    p = tf.transform.translation
                    pose = [p.x, p.y, yaw(tf.transform.rotation)]
                else:
                    tf_error = '机器人 TF 已过期'
            except Exception:
                tf_error = '等待 map 到 base_footprint 的 TF'
            # Require a map->odom transform from this initial-pose request.
            cutoff = localization_state.get('initial_pose_stamp')
            localization_tf_ready = pose is not None
            try:
                map_tf = buffer.lookup_transform(map_frame, 'odom', Time())
                map_stamp = map_tf.header.stamp.sec + map_tf.header.stamp.nanosec*1e-9
                localization_tf_ready = localization_tf_ready and cutoff is not None and map_stamp >= cutoff
            except Exception:
                localization_tf_ready = False
            localization_status = localization.status(
                node.get_clock().now().nanoseconds*1e-9, odometry_status, topics,
                localization_tf_ready, amcl_state == 3 and now-amcl_checked < 4,
                amcl_publishers == ['amcl'])
            localized = localization_status['estimate_visible']
            particles, navigation_paths = telemetry.snapshot(
                now, localized, localization_status['reason'], particle_source, path_sources)
            if mode == 'empty' or (mode != 'mapping' and not localized):
                pose = None
                tf_error = '已有地图定位未就绪'
            transforms = {}
            for key, parent, child in [('map_odom', map_frame, 'odom'), ('odom_base', 'odom', 'base_footprint')]:
                try:
                    tf = buffer.lookup_transform(parent, child, Time())
                    p = tf.transform.translation
                    transforms[key] = {'stamp': tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9,
                                       'pose': [p.x, p.y, yaw(tf.transform.rotation)]}
                except Exception:
                    pass
            points = {}
            local_points = {}
            for topic, pending in scans.items():
                points[topic] = []
                local_points[topic] = []
                for received, scan in reversed(pending):
                    if now - received > .75:
                        break
                    if not local_points[topic]:
                        try:
                            local_tf = buffer.lookup_transform('base_footprint', scan.header.frame_id, Time())
                            p = local_tf.transform.translation
                            a = yaw(local_tf.transform.rotation)
                            local_points[topic] = [
                                [round(p.x + r * math.cos(a + scan.angle_min + i * scan.angle_increment), 3),
                                 round(p.y + r * math.sin(a + scan.angle_min + i * scan.angle_increment), 3)]
                                for i, r in enumerate(scan.ranges)
                                if i % 3 == 0 and math.isfinite(r) and scan.range_min <= r <= min(scan.range_max, 20)]
                        except Exception:
                            pass
                    if mode == 'empty' or (mode != 'mapping' and not localized):
                        break
                    try:
                        tf = buffer.lookup_transform(map_frame, scan.header.frame_id, Time.from_msg(scan.header.stamp))
                    except Exception:
                        continue
                    p = tf.transform.translation
                    angle = yaw(tf.transform.rotation)
                    points[topic] = [
                        [round(p.x + r * math.cos(angle + scan.angle_min + i * scan.angle_increment), 3),
                         round(p.y + r * math.sin(angle + scan.angle_min + i * scan.angle_increment), 3)]
                        for i, r in enumerate(scan.ranges)
                        if i % 3 == 0 and math.isfinite(r) and scan.range_min <= r <= min(scan.range_max, 20)
                    ]
                    break
            emit({'type': 'frame', 'time': time.time(), 'topics': topics,
                  'ekf_source': ekf_source, 'pose': pose, 'tf_error': tf_error,
                  'odometry_mode': odometry_status.get('mode', 'ekf'),
                  'odometry_source': odometry_status.get('source', ekf_source),
                  'odometry_status': odometry_status,
                  'scan_match': dict(matches.status(now), source_confirmed=match_publishers == ['slam_toolbox']) if mode == 'mapping' else None,
                  'localization': localization_status,
                  'display_session': current_display_session,
                  'particles': particles, 'navigation_paths': navigation_paths,
                  'mapping_parameters': mapping_parameters, 'transforms': transforms,
                  'scans': points, 'local_scans': local_points, 'frame': map_frame,
                  'map_mode': mode, 'mapping_paused': mapping_paused, 'map_publishers': map_publishers,
                  'saved_map_source': str(saved_file.resolve()) if saved_file.is_file() else None,
                  'saved_map_modified': saved_file.stat().st_mtime if saved_file.is_file() else None})
    except BrokenPipeError:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        emit({'type': 'error', 'message': str(exc)})
        raise SystemExit(1)
