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
import threading
from functools import wraps
import time
import zlib

PREFIX = 'MAP_VIEWER '
ODOMETRY_STATUS = Path.home()/'.local/share/fairino-odometry/status.json'
MAPPING_EPOCH = Path.home()/'.local/share/fairino-map-control/odometry-epoch'


def synchronized(method):
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return wrapped


class CallbackReceiver:
    """Receive callbacks on an owned executor, independent of rendering/control I/O."""
    def __init__(self, rclpy, executor_factory, name, register, namespace='/fairino_navigation'):
        self.node = self.executor = self.thread = None
        self.resources = []
        self.error = None
        self._closing = self._closed = False
        try:
            self.node = rclpy.create_node(name, namespace=namespace)
            self.resources = register(self.node)
            self.executor = executor_factory(context=self.node.context)
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self._spin, name=name, daemon=True)
            self.thread.start()
        except BaseException:
            self.close()
            raise

    def _spin(self):
        if self._closing:
            return
        try:
            self.executor.spin()
        except Exception as exc:
            if not self._closing and self.node.context.ok():
                self.error = type(exc).__name__+': '+str(exc)

    def status(self):
        return {'running': bool(self.thread and self.thread.is_alive()), 'error': self.error}

    def close(self):
        if self._closed:
            return
        self._closing = True
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.)
        if self.thread is not None and self.thread.ident is not None:
            self.thread.join(timeout=2.)
            if self.thread.is_alive():
                raise RuntimeError('数据接收线程未能停止；保留仍在使用的节点')
        for resource in self.resources:
            if hasattr(resource, 'unregister'):
                resource.unregister()
        if self.executor is not None and self.node is not None:
            self.executor.remove_node(self.node)
        if self.node is not None:
            self.node.destroy_node()
        self._closed = True


class ScanMatchMonitor:
    """Track accepted SLAM poses separately from map/TF publication."""
    def __init__(self):
        self._lock = threading.RLock()
        self.session = None
        self.started = 0.
        self.latest = self.odometry = None
        self.count = 0
        self.observed_since = time.monotonic()
        self.travel = self.rotation = 0.
        self.distance_threshold = .1
        self.angle_threshold = math.radians(5)

    @synchronized
    def start_session(self, token, started):
        if token != self.session:
            self.session, self.started = token, started
            self.latest = None
            self.count = 0
            self.observed_since = time.monotonic()
            self.travel = self.rotation = 0.

    @synchronized
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

    @synchronized
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

    @synchronized
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
    """AMCL freshness follows its motion trigger, with bounded update waiting."""
    UPDATE_WAIT = 2.
    ODOM_HISTORY_SECONDS = 3.
    ODOM_HISTORY_LIMIT = 1024

    def __init__(self):
        self._lock = threading.RLock()
        self.session = {}
        self.sample = None
        self.odometry = None
        self.history = deque(maxlen=self.ODOM_HISTORY_LIMIT)
        self.reference = None
        self.reference_stamp = None
        self.baseline_covered = False
        self.travel = self.rotation = 0.
        self.update_due_since = None
        self.parameter_error = None
        self.update_min_d, self.update_min_a = .1, math.radians(5)

    @staticmethod
    def _finite(value):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

    @staticmethod
    def _angle(value):
        return math.atan2(math.sin(value), math.cos(value))

    @synchronized
    def set_session(self, session):
        if (session.get('id'), session.get('initial_pose_stamp')) != (
                self.session.get('id'), self.session.get('initial_pose_stamp')):
            self.sample = None
            self.travel = self.rotation = 0.
            self.odometry = self.reference = None
            self.reference_stamp = None
            self.baseline_covered = False
            self.update_due_since = None
            self.history.clear()
        self.session = dict(session)
        self.parameter_error = None
        if 'amcl_parameters' not in session:
            # Existing sessions predate this snapshot; match the managed defaults.
            self.update_min_d, self.update_min_a = .1, math.radians(5)
            return
        parameters = session['amcl_parameters']
        values = ([parameters.get(name) for name in ('update_min_d', 'update_min_a')]
                  if isinstance(parameters, dict) else [None, None])
        if any(not self._finite(v) or v < 0 for v in values):
            self.parameter_error = 'AMCL 更新阈值无效，请重新启动定位并检查参数'
            return
        self.update_min_d, self.update_min_a = map(float, values)

    def _delta(self):
        if self.reference is None or self.odometry is None:
            return (0., 0., 0.)
        return (self.odometry[0]-self.reference[0], self.odometry[1]-self.reference[1],
                self._angle(self.odometry[2]-self.reference[2]))

    def _associate_sample(self):
        """Rebuild travel after the AMCL timestamp, if odometry brackets it."""
        if not self.sample or not self.history:
            return False
        stamp = self.sample['stamp']
        if not self.history[0][0] <= stamp <= self.history[-1][0]:
            return False
        points = list(self.history)
        reference = None
        for index, (right_stamp, right_pose, _) in enumerate(points):
            if right_stamp == stamp:
                reference = right_pose
                break
            if right_stamp > stamp and index:
                left_stamp, left_pose, _ = points[index-1]
                fraction = (stamp-left_stamp)/(right_stamp-left_stamp)
                reference = (left_pose[0]+fraction*(right_pose[0]-left_pose[0]),
                             left_pose[1]+fraction*(right_pose[1]-left_pose[1]),
                             self._angle(left_pose[2]+fraction*self._angle(right_pose[2]-left_pose[2])))
                break
        if reference is None:
            return False
        self.reference, self.reference_stamp = tuple(reference), stamp
        self.baseline_covered = True
        self.travel = self.rotation = 0.
        self.update_due_since = None
        previous = self.reference
        for odom_stamp, pose, moving in points:
            if odom_stamp <= stamp:
                continue
            if moving:
                self.travel += math.hypot(pose[0]-previous[0], pose[1]-previous[1])
                self.rotation += abs(self._angle(pose[2]-previous[2]))
            # A delayed but newer AMCL sample must not restart the deadline at
            # receipt time if its reference predates the same threshold crossing.
            if (self.update_due_since is None and
                    (abs(pose[0]-self.reference[0]) > self.update_min_d
                     or abs(pose[1]-self.reference[1]) > self.update_min_d
                     or abs(self._angle(pose[2]-self.reference[2])) > self.update_min_a)):
                self.update_due_since = odom_stamp
            previous = pose
        return True

    @synchronized
    def on_odometry(self, pose, twist, stamp=None):
        if (len(pose) != 3 or len(twist) != 3
                or not all(self._finite(v) for v in (*pose, *twist))
                or stamp is not None and (not self._finite(stamp) or stamp <= 0)):
            return
        if stamp is not None and self.history and stamp <= self.history[-1][0]:
            return
        moving = math.hypot(*twist[:2]) >= .005 or abs(twist[2]) >= .005
        if self.odometry is not None and moving:
            self.travel += math.hypot(pose[0]-self.odometry[0], pose[1]-self.odometry[1])
            self.rotation += abs(self._angle(pose[2]-self.odometry[2]))
        self.odometry = tuple(pose)
        if stamp is not None:
            self.history.append((float(stamp), self.odometry, moving))
            # Keep one older point for interpolation at the window boundary.
            while len(self.history) > 2 and stamp-self.history[1][0] > self.ODOM_HISTORY_SECONDS:
                self.history.popleft()
        if self.sample and not self.baseline_covered and self._associate_sample():
            return
        if self.sample and self.reference is None:
            # Latched poses older than our bounded history cannot be reconstructed.
            # Start tracking at the first observed odometry and expose this fact.
            self.reference = self.odometry
            self.reference_stamp = stamp
            self.travel = self.rotation = 0.

    @synchronized
    def accept(self, stamp, age, pose, covariance, frame):
        cutoff = self.session.get('initial_pose_stamp')
        values = [stamp, age, *pose, *covariance]
        if (not self.session.get('active') or not self._finite(cutoff) or cutoff <= 0
                or frame != 'map' or len(pose) != 3 or len(covariance) != 36
                or not all(self._finite(v) for v in values)
                or stamp <= 0 or stamp < cutoff or age < -.5
                or any(covariance[i] < 0 for i in (0, 7, 35))
                or (self.sample and stamp <= self.sample['stamp'])):
            return False
        # The first latched pose may predate our odometry observation. Later
        # delayed poses must not erase motion that can no longer be reconstructed.
        if self.sample and ((self.history and stamp < self.history[0][0])
                            or (not self.history and age > self.UPDATE_WAIT)):
            return False
        xx, yy, xy = covariance[0], covariance[7], (covariance[1]+covariance[6])/2
        if xy*xy > xx*yy + 1e-9:
            return False
        variance = .5*(xx+yy+math.sqrt((xx-yy)**2+4*xy**2))
        self.sample = {'stamp': stamp, 'pose': list(pose),
                       'matches': (self.sample or {}).get('matches', 0)+1,
                       'xy_std': math.sqrt(max(0., variance)),
                       'yaw_std': math.sqrt(covariance[35])}
        self.travel = self.rotation = 0.
        self.update_due_since = None
        self.baseline_covered = False
        self.reference = self.odometry
        self.reference_stamp = self.history[-1][0] if self.history else None
        self._associate_sample()
        return True

    @synchronized
    def status(self, now_ros, odometry, topics, tf_ready, amcl_active, source_confirmed):
        watchdog = self.session.get('watchdog_time')
        entries = [topics.get(name) for name in ('/scan', '/odom')] if isinstance(topics, dict) else []
        input_ages = [entry.get('age') if isinstance(entry, dict) else None for entry in entries]
        delta = self._delta()
        due = (abs(delta[0]) > self.update_min_d or abs(delta[1]) > self.update_min_d
               or abs(delta[2]) > self.update_min_a)
        if (self.sample and due and self.update_due_since is None
                and self._finite(now_ros) and not self.parameter_error):
            self.update_due_since = now_ros
        wait_age = (max(0., now_ros-self.update_due_since)
                    if self.update_due_since is not None and self._finite(now_ros) else None)
        hard_distance = max(3*math.sqrt(2)*self.update_min_d, .3)
        hard_angle = max(3*self.update_min_a, math.radians(15))
        hard_limit = self.travel >= hard_distance or self.rotation >= hard_angle
        result = dict(self.sample or {}, id=self.session.get('id'), active=self.session.get('active', False),
                      ready=False, estimate_visible=False, state='stopped',
                      initialization=dict(self.session.get('initialization') or {}),
                      reason=self.session.get('reason', '定位尚未启动'),
                      match_age=(max(0., now_ros-self.sample['stamp'])
                                 if self.sample and self._finite(now_ros) else None),
                      update_wait_age=wait_age, update_wait_limit=self.UPDATE_WAIT,
                      odometry_delta=list(delta), travel_since_match=self.travel,
                      rotation_since_match=self.rotation, baseline_covered=self.baseline_covered,
                      baseline_stamp=self.reference_stamp,
                      update_min_d=self.update_min_d, update_min_a=self.update_min_a,
                      hard_distance_limit=hard_distance, hard_angle_limit=hard_angle)
        if not result['active']:
            return result
        result['state'] = 'waiting'
        if self.parameter_error:
            result['reason'] = self.parameter_error
        elif not self._finite(now_ros):
            result['reason'] = '定位时钟无效'
        elif self.sample and self.sample['stamp']-now_ros > .5:
            result['reason'] = '定位匹配时间晚于当前时钟，请重新启动定位'
        elif self.update_due_since is not None and self.update_due_since-now_ros > .5:
            result['reason'] = '定位时钟回退，请重新启动定位'
        elif not amcl_active or not source_confirmed:
            result['reason'] = '等待唯一的活动 AMCL 节点'
        elif not self._finite(watchdog) or watchdog <= 0 or abs(now_ros-watchdog) > 5:
            result['reason'] = '定位监视服务未更新，请停止后重新启动定位'
        elif not odometry.get('running') or odometry.get('epoch') != self.session.get('epoch'):
            result['reason'] = '里程计未就绪或会话已变化，请重新启动定位'
        elif len(input_ages) != 2 or any(not self._finite(age) or not 0 <= age <= .75
                                         for age in input_ages):
            missing = []
            for topic, age in zip(('/scan', '/odom'), input_ages):
                label = '融合雷达' if topic == '/scan' else '里程计'
                if not self._finite(age) or not 0 <= age <= .75:
                    missing.append(label+('未收到有效数据' if not self._finite(age)
                                         else f'数据年龄 {age:.3f} 秒（要求 0～0.750 秒）'))
            result['reason'] = '定位输入不可用：'+('；'.join(missing) or '雷达/里程计数据缺失')
        elif self.session.get('initial_pose_stamp') is None:
            result['reason'] = '请在地图上设置初始位置与朝向'
        elif not self.sample:
            result['reason'] = '等待本次初始位姿后的 AMCL 雷达匹配'
        elif not tf_ready:
            result['reason'] = '定位 TF 未就绪或已过期'
        elif hard_limit:
            result.update(state='stale', reason='未获新定位匹配且累计运动超限，当前主要随里程计推算')
        elif wait_age is not None and wait_age >= self.UPDATE_WAIT:
            result.update(state='stale', reason='运动达到更新门槛后 2 秒仍未获新定位匹配，当前主要随里程计推算')
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
            reason = (f'已达 AMCL 更新门槛，等待新雷达匹配（{wait_age:.1f}/{self.UPDATE_WAIT:.1f} 秒）'
                      if wait_age is not None else
                      '定位估计正在更新，请核对雷达与墙面重合' if now_ros-self.sample['stamp'] <= 2
                      else '定位正常，可导航；当前运动量未达到下一次雷达匹配阈值')
            result.update(ready=True, estimate_visible=True, state='tracking', reason=reason)
        return result



FOOTPRINT_TOPICS = {
    'local': '/fairino_navigation/local_costmap/published_footprint',
    'global': '/fairino_navigation/global_costmap/published_footprint',
}


class FootprintObserver:
    """Bounded display-only samples of Nav2's actual padded, oriented footprints."""
    MAX_AGE = 1.
    MAX_VERTICES = 64

    def __init__(self):
        self.pending = {name: deque(maxlen=4) for name in FOOTPRINT_TOPICS}
        self.context = None

    @staticmethod
    def finite(value):
        return (isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value))

    def clear(self):
        for samples in self.pending.values():
            samples.clear()

    def set_context(self, token):
        if token != self.context:
            self.clear()
            self.context = token

    def accept(self, name, msg, received):
        if name not in self.pending or not self.finite(received):
            return False
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            frame = msg.header.frame_id
            points = [[p.x, p.y, p.z] for p in msg.polygon.points]
        except (AttributeError, TypeError, ValueError):
            return False
        if (not self.finite(stamp) or stamp <= 0 or not isinstance(frame, str)
                or not frame or frame.startswith('/')
                or not 3 <= len(points) <= self.MAX_VERTICES
                or any(not self.finite(v) for point in points for v in point)):
            return False
        if points[0] == points[-1]:
            points.pop()
        if len({tuple(p[:2]) for p in points}) < 3:
            return False
        area = sum(a[0]*b[1]-b[0]*a[1]
                   for a, b in zip(points, points[1:]+points[:1]))
        if not math.isfinite(area) or abs(area) <= 1e-10:
            return False
        if self.pending[name] and stamp <= self.pending[name][-1]['stamp']:
            return False
        self.pending[name].append({
            'stamp': stamp, 'stamp_msg': msg.header.stamp, 'received': received,
            'source_frame': frame, 'points': points})
        return True

    @staticmethod
    def transformed(points, transform):
        p, q = transform.transform.translation, transform.transform.rotation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if (any(not FootprintObserver.finite(v) for v in values)
                or abs(sum(v*v for v in values[3:])-1.) > 1e-3):
            raise ValueError('Invalid footprint transform')
        # Full quaternion rotation before XY projection; no extra robot-pose
        # transform or padding: Nav2 already applied both to the polygon.
        rows = (
            (1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.z*q.w), 2*(q.x*q.z+q.y*q.w)),
            (2*(q.x*q.y+q.z*q.w), 1-2*(q.x*q.x+q.z*q.z), 2*(q.y*q.z-q.x*q.w)))
        result = [[offset+sum(a*b for a, b in zip(row, point))
                   for offset, row in zip((p.x, p.y), rows)] for point in points]
        if any(not math.isfinite(v) for point in result for v in point):
            raise ValueError('Invalid transformed footprint')
        area = sum(a[0]*b[1]-b[0]*a[1]
                   for a, b in zip(result, result[1:]+result[:1]))
        if not math.isfinite(area) or abs(area) <= 1e-10:
            raise ValueError('Degenerate transformed footprint')
        return result

    def snapshot(self, now, now_ros, map_frame, pose_visible, cutoff, lookup):
        if (not pose_visible or not map_frame
                or any(not self.finite(t) for t in (now, now_ros, cutoff))):
            return {}
        result = {}
        for name, samples in self.pending.items():
            for sample in reversed(samples):
                receipt_age, stamp_age = now-sample['received'], now_ros-sample['stamp']
                if (not 0 <= receipt_age <= self.MAX_AGE
                        or not -.05 <= stamp_age <= self.MAX_AGE
                        or sample['stamp'] < cutoff):
                    continue
                try:
                    points = ([p[:2] for p in sample['points']]
                              if sample['source_frame'] == map_frame else
                              self.transformed(sample['points'],
                                               lookup(map_frame, sample['source_frame'],
                                                      sample['stamp_msg'])))
                except Exception:
                    # The latest polygon can precede its corresponding TF.
                    # A still-fresh earlier sample is preferable to guessing.
                    continue
                result[name] = {'points': points, 'stamp': sample['stamp'],
                                'age': max(0., receipt_age, stamp_age),
                                'frame_id': map_frame,
                                'source': FOOTPRINT_TOPICS[name],
                                'source_frame': sample['source_frame']}
                break
        return result


def emit(payload):
    print(PREFIX + json.dumps(payload, separators=(',', ':'), allow_nan=False), flush=True)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=0)
    args = parser.parse_args()
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.executors import SingleThreadedExecutor
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseWithCovarianceStamped, PolygonStamped
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
    sensor_receiver = None
    subscriptions = []
    odom_topics = ['/fairino_keyboard/odom', '/eight_dof_chassis_controller/odom', '/odom']
    scan_topics = ['/front/scan', '/back/scan']
    history = {t: deque(maxlen=200) for t in ['/map', '/scan'] + odom_topics + scan_topics}
    odom = {}
    scans = {t: deque(maxlen=12) for t in scan_topics}
    footprints = FootprintObserver()
    map_frame = 'map'
    map_signature = None
    map_revision = 0
    map_changed_at = 0.
    ekf_source = None
    localization = LocalizationMonitor()
    localization_state = {}
    amcl_state = None
    amcl_checked = 0.
    amcl_publishers = []
    matches = ScanMatchMonitor()
    mapping_parameters = {}
    match_publishers = []

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

    def on_map(msg):
        nonlocal map_frame, map_signature, map_revision, map_changed_at
        history['/map'].append(time.monotonic())
        map_frame = msg.header.frame_id
        raw = array.array('b', msg.data).tobytes()
        p = msg.info.origin.position
        origin = [p.x, p.y, yaw(msg.info.origin.orientation)]
        signature = (map_frame, msg.info.width, msg.info.height, msg.info.resolution, *origin, zlib.crc32(raw))
        if signature == map_signature:
            return
        map_signature = signature
        footprints.clear()
        map_revision += 1
        map_changed_at = node.get_clock().now().nanoseconds*1e-9
        emit({'type': 'map', 'frame': map_frame, 'map_revision': map_revision, 'width': msg.info.width,
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
                localization.on_odometry(values[:3], values[3:6],
                                         msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
        return receive

    def on_scan(topic):
        def receive(msg):
            now = time.monotonic()
            history[topic].append(now)
            scans[topic].append((now, msg))
        return receive

    def footprint_context():
        return (mode, map_frame, map_revision, localization_state.get('id'),
                localization_state.get('initial_pose_stamp'), localization_state.get('epoch'))

    def on_footprint(name):
        def receive(msg):
            footprints.set_context(footprint_context())
            footprints.accept(name, msg, time.monotonic())
        return receive

    for name, topic in FOOTPRINT_TOPICS.items():
        subscriptions.append(node.create_subscription(
            PolygonStamped, topic, on_footprint(name), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        OccupancyGrid, '/map', on_map,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    def register_inputs(input_node):
        inputs = [tf2_ros.TransformListener(buffer, input_node, spin_thread=False)]
        for topic in odom_topics:
            inputs.append(input_node.create_subscription(Odometry, topic, on_odom(topic), 20))
        for topic in scan_topics:
            inputs.append(input_node.create_subscription(LaserScan, topic, on_scan(topic), qos_profile_sensor_data))
        inputs.append(input_node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', on_localization,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        inputs.append(input_node.create_subscription(
            LaserScan, '/scan', lambda msg: history['/scan'].append(time.monotonic()), qos_profile_sensor_data))
        return inputs
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
    refresh_localization()
    try:
        sensor_receiver = CallbackReceiver(
            rclpy, SingleThreadedExecutor, 'fairino_map_inputs_'+str(os.getpid()),
            register_inputs, namespace='/')
        emit({'type': 'ready', 'mode': 'read_only', 'domain': int(os.environ.get('ROS_DOMAIN_ID', '1'))})
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
                saved_file = Path('/opt/fairino_AMR_project/maps/active/map.yaml')
                new_mode = 'mapping' if 'slam_toolbox' in map_publishers else 'saved' if 'map_server' in map_publishers else 'empty' if not saved_file.is_file() else 'unknown'
                if new_mode != mode:
                    mode = new_mode
                    mode_since = time.time()
                    footprints.clear()
                    mapping_paused = None
                    if mode == 'empty':
                        map_signature = None
                        map_revision += 1
                        map_changed_at = node.get_clock().now().nanoseconds*1e-9
                        emit({'type': 'clear_map', 'map_revision': map_revision})
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
            topics = {}
            for topic, times in history.items():
                recent = [t for t in list(times) if now - t < 3]
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
            now = time.monotonic()
            for topic, times in history.items():
                latest = list(times)
                topics[topic]['age'] = max(0., time.monotonic()-latest[-1]) if latest else None
            localization_status = localization.status(
                node.get_clock().now().nanoseconds*1e-9, odometry_status, topics,
                localization_tf_ready, amcl_state == 3 and now-amcl_checked < 4,
                amcl_publishers == ['amcl'])
            if sensor_receiver.error:
                localization_status.update(ready=False, estimate_visible=False,
                                           reason='定位数据接收线程异常：'+sensor_receiver.error)
            localized = localization_status['estimate_visible']
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
                for received, scan in reversed(list(pending)):
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
            footprints.set_context(footprint_context())
            footprint_cutoff = max(mode_since, map_changed_at,
                                   cutoff if FootprintObserver.finite(cutoff) else 0.)
            footprint_data = footprints.snapshot(
                time.monotonic(), node.get_clock().now().nanoseconds*1e-9,
                map_frame, pose is not None, footprint_cutoff,
                lambda target, source, stamp: buffer.lookup_transform(target, source, Time.from_msg(stamp)))
            emit({'type': 'frame', 'time': time.time(), 'topics': topics,
                  'ekf_source': ekf_source, 'pose': pose, 'tf_error': tf_error,
                  'odometry_mode': odometry_status.get('mode', 'ekf'),
                  'odometry_source': odometry_status.get('source', ekf_source),
                  'odometry_status': odometry_status,
                  'scan_match': dict(matches.status(now), source_confirmed=match_publishers == ['slam_toolbox']) if mode == 'mapping' else None,
                  'localization': localization_status, 'input_receiver': sensor_receiver.status(),
                  'mapping_parameters': mapping_parameters, 'transforms': transforms,
                  'scans': points, 'local_scans': local_points, 'frame': map_frame,
                  'footprints': footprint_data, 'map_revision': map_revision,
                  'map_mode': mode, 'mapping_paused': mapping_paused, 'map_publishers': map_publishers,
                  'saved_map_source': str(saved_file.resolve()) if saved_file.is_file() else None,
                  'saved_map_modified': saved_file.stat().st_mtime if saved_file.is_file() else None})
    except BrokenPipeError:
        pass
    finally:
        try:
            if sensor_receiver is not None:
                sensor_receiver.close()
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
