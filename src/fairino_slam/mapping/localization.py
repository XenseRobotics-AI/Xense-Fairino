"""AMCL lifecycle/session control. No motor or navigation commands."""
import json
import math
from pathlib import Path
import time
import uuid

RUNTIME = Path.home() / '.local/share/fairino-map-control'
SESSION = RUNTIME / 'localization.json'
PROFILE = RUNTIME / 'localization.yaml'
EPOCH = Path.home() / '.local/share/fairino-odometry/epoch'


def read_session():
    try:
        return json.loads(SESSION.read_text())
    except (OSError, ValueError):
        return {}


def write_session(state):
    SESSION.parent.mkdir(parents=True, exist_ok=True)
    tmp = SESSION.with_suffix('.tmp')
    tmp.write_text(json.dumps(state, ensure_ascii=False, allow_nan=False))
    tmp.replace(SESSION)


def epoch():
    return EPOCH.read_text().strip() if EPOCH.is_file() else None


def initial_pose(x, y, yaw, xy_std=.5, yaw_std=math.radians(15)):
    values = (x, y, yaw, xy_std, yaw_std)
    if not all(math.isfinite(v) for v in values) or xy_std <= 0 or not 0 < yaw_std <= math.pi:
        raise ValueError('初始位姿须为有限数值；位置标准差须大于零，角度标准差须在 (0, 180°] 内')
    cov = [0.] * 36
    cov[0] = cov[7] = xy_std ** 2
    cov[35] = yaw_std ** 2
    return {'x': x, 'y': y, 'z': 0., 'qz': math.sin(yaw / 2),
            'qw': math.cos(yaw / 2), 'covariance': cov}


class Localization:
    def __init__(self, ros):
        self.ros = ros

    def stop(self, reason='定位已停止'):
        old = read_session()
        write_session(dict(old, active=False, reason=reason))
        nodes = self.ros.node.get_node_names_and_namespaces()
        if not any(name == 'amcl' and ns == '/' for name, ns in nodes):
            return
        state = self.ros.state('/amcl')
        if state == 3:
            self.ros.transition('/amcl', 4)
        if self.ros.state('/amcl') == 2:
            self.ros.transition('/amcl', 2)
        if self.ros.state('/amcl') != 1:
            raise RuntimeError('AMCL 尚未停止，拒绝切换地图或启动建图')

    def start(self, map_path, wait=0):
        if self.ros.slam_present():
            raise RuntimeError('请先结束建图并选用保存的地图，再启动定位')
        if not map_path.is_file():
            raise RuntimeError('尚无选用的地图；请先建图、另存并选用地图')
        import yaml
        params = yaml.safe_load(PROFILE.read_text())['amcl']['ros__parameters']
        monitor_parameters = {}
        for name in ('update_min_d', 'update_min_a', 'transform_tolerance'):
            value = params.get(name)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < 0):
                raise ValueError('AMCL 参数必须为有限非负数：'+name)
            params[name] = monitor_parameters[name] = float(value)
        if read_session().get('active'):
            raise RuntimeError('定位会话已运行；需要重新初始化时请先停止定位')
        self.ros.require_good_input(wait)
        token = epoch()
        if not token:
            raise RuntimeError('里程计会话未建立，请先启动底盘里程计')
        self.stop()
        try:
            self.ros.set_params('/amcl', params)
            # Reload the explicitly selected map, not a previous lifecycle cache.
            self.ros.map_server(False)
            self.ros.set_params('/map_server', {'yaml_filename': str(map_path)})
            if not self.ros.map_server(True):
                raise RuntimeError('没有可加载的地图')
            self.ros.transition('/amcl', 1)
            self.ros.transition('/amcl', 3)
            if self.ros.state('/amcl') != 3:
                raise RuntimeError('AMCL 未成功激活')
            state = {'id': uuid.uuid4().hex, 'active': True, 'started': time.time(),
                     'map': str(map_path.resolve()), 'epoch': token,
                     'amcl_parameters': monitor_parameters,
                     'initial_pose_stamp': None, 'reason': '等待设置初始位姿'}
            write_session(state)
            return state
        except Exception:
            self.stop('启动定位失败，请检查后重试')
            raise

    def set_pose(self, x, y, yaw, xy_std=.5, yaw_std=math.radians(15), session_id=None):
        data = initial_pose(x, y, yaw, xy_std, yaw_std)
        state = read_session()
        if session_id is not None and session_id != state.get('id'):
            raise RuntimeError('定位会话已改变，请在当前地图上重新设置初始位姿')
        if not state.get('active') or self.ros.state('/amcl') != 3 or self.ros.slam_present():
            raise RuntimeError('请先启动已有地图定位')
        if state.get('epoch') != epoch():
            self.stop('里程计已重新初始化，请重新启动定位')
            raise RuntimeError('里程计会话已变化，请重新启动定位并设置初始位姿')
        self.ros.require_good_input()
        from nav2_msgs.srv import SetInitialPose
        request = SetInitialPose.Request()
        msg = request.pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.ros.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = data['x'], data['y']
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = data['qz'], data['qw']
        msg.pose.covariance = data['covariance']
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # The request initializes a search distribution, not an exact pose.
        # The watchdog requests bounded updates from fresh stationary scans.
        state.update(initial_pose_stamp=stamp, initial_pose={**data, 'yaw': yaw},
                     initialization={'state': 'refining', 'updates': 0, 'maximum': 6},
                     reason='初始位姿已请求，正在静止雷达匹配')
        write_session(state)
        try:
            self.ros.call(SetInitialPose, '/set_initial_pose', request)
        except Exception:
            state.update(initial_pose_stamp=None,
                         initialization={'state': 'failed', 'updates': 0, 'maximum': 6},
                         reason='初始位姿请求未确认，请重新设置')
            write_session(state)
            raise
        return data



class InitialPoseRefinement:
    """Bounded stationary updates; never drive or declare a pose correct."""
    def __init__(self):
        self.key = None
        self.started = self.last_request = None
        self.last_scan = None

    def step(self, ros, state, now):
        key = (state.get('id'), state.get('initial_pose_stamp'))
        stage = dict(state.get('initialization') or {})
        if not state.get('active') or key[1] is None or stage.get('state') != 'refining':
            return False
        if key != self.key:
            self.key, self.started = key, now
            self.last_request = self.last_scan = None
        if now-self.started >= 12:
            stage.update(state='completed', note='静止匹配窗口结束，请核对候选位置与雷达重合')
        else:
            odom = ros.latest.get('/odom', (None, {}))[1]
            if any(abs(odom.get(k, 0.)) > .005 for k in ('vx', 'vy', 'wz')):
                stage.update(state='completed', note='机器人已移动，转入常规定位更新')
            else:
                scan = ros.latest.get('/scan', (None, {}))[0]
                if (scan is None or now-scan > .75 or scan == self.last_scan or
                        self.last_request is not None and now-self.last_request < 1.):
                    return False
                try:
                    from std_srvs.srv import Empty
                    ros.call(Empty, '/request_nomotion_update', Empty.Request(), timeout=2)
                except Exception as exc:
                    stage.update(state='failed', note='静止匹配请求失败：'+str(exc))
                else:
                    stage['updates'] = stage.get('updates', 0)+1
                    if stage['updates'] >= stage.get('maximum', 6):
                        stage.update(state='completed', note='静止匹配请求完成，请核对雷达与墙面重合')
                self.last_request, self.last_scan = now, scan
        state['initialization'] = stage
        return True

def watch(ros, lock_factory):
    token = read_session().get('id')
    try:
        _watch(ros, lock_factory)
    finally:
        # A simultaneous explicit mode change owns cleanup while holding the lock.
        try:
            with lock_factory():
                state = read_session()
                if state.get('active') and state.get('id') == token:
                    Localization(ros).stop('定位监视服务已退出，请重新启动定位')
        except BlockingIOError:
            pass


def _watch(ros, lock_factory):
    """Stop AMCL after lost input or odometry reset; never restart automatically."""
    state = read_session()
    token = state.get('id')
    bad_since = None
    heartbeat = 0.
    refinement = InitialPoseRefinement()
    ros.spin(2)
    while read_session().get('active') and read_session().get('id') == token:
        ros.spin(.2)
        if time.monotonic()-heartbeat > 1:
            try:
                with lock_factory():
                    current = read_session()
                    if not current.get('active') or current.get('id') != token:
                        return
                    current['watchdog_time'] = time.time()
                    write_session(current)
                    heartbeat = time.monotonic()
            except BlockingIOError:
                pass
        try:
            if not ros.source:
                ros.refresh_source()
            reason = ros.input_reason()
            if epoch() != state.get('epoch'):
                reason = '里程计会话已变化，请重新启动定位'
            if ros.slam_present():
                reason = '发现建图节点，定位已停止'
        except Exception as exc:
            reason = str(exc)
        if not reason:
            bad_since = None
            try:
                with lock_factory():
                    current = read_session()
                    if current.get('active') and current.get('id') == token:
                        if refinement.step(ros, current, time.monotonic()):
                            current['watchdog_time'] = time.time()
                            write_session(current)
            except BlockingIOError:
                pass
            continue
        bad_since = bad_since or time.monotonic()
        if time.monotonic() - bad_since < 1.5:
            continue
        try:
            with lock_factory():
                current = read_session()
                if current.get('active') and current.get('id') == token:
                    Localization(ros).stop(reason)
            return
        except BlockingIOError:
            continue
