#!/usr/bin/env python3
"""Explicit map mode changes. No motor, controller or navigation goal commands."""
import argparse
import datetime
import fcntl
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import re
import signal
import shutil
import subprocess
import time

PROJECT = Path('/opt/fairino_AMR_project')
RUNTIME = Path.home() / '.local/share/fairino-map-control'
MAPS = Path.home() / 'maps/manual'
UNIT = 'fairino-manual-mapping.service'
ODOMETRY_EPOCH = Path.home() / '.local/share/fairino-odometry/epoch'
MAPPING_EPOCH = RUNTIME / 'odometry-epoch'
ODOMETRY_STATUS = ODOMETRY_EPOCH.with_name('status.json')
MAPPING_PROFILE = RUNTIME / 'mapping.yaml'
SLAM_RUNTIME = RUNTIME / 'slam/current'
LOCALIZATION_UNIT = 'fairino-localization.service'


@contextmanager
def mode_lock():
    RUNTIME.mkdir(parents=True, exist_ok=True)
    with (RUNTIME / 'mode.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield



def mapping_command():
    manifest = SLAM_RUNTIME / 'manifest.json'
    executable = SLAM_RUNTIME / 'bin/sync_slam_toolbox_node'
    if not executable.is_file() or not manifest.is_file() or not MAPPING_PROFILE.is_file():
        raise RuntimeError('建图修复程序尚未部署，请运行 tools/install_mapping_runtime.py --execute')
    if json.loads(manifest.read_text()).get('scan_gate') != 'backend_motion_gate_v1':
        raise RuntimeError('SLAM 扫描筛选修复版本未确认，拒绝使用旧建图程序')
    return [str(executable.resolve()), '--ros-args', '-r', '__node:=slam_toolbox',
            '-p', 'use_sim_time:=false', '--params-file', str(MAPPING_PROFILE)]


def read_epoch(path):
    return (path.read_text().strip() or None) if path.is_file() else None


def require_same_epoch():
    if read_epoch(ODOMETRY_EPOCH) != read_epoch(MAPPING_EPOCH):
        raise RuntimeError('里程计已经重新初始化，旧地图坐标连续性未确认；请另存/结束本次建图，再准备新建图')


def run(*args, timeout=30):
    result = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip() or '命令失败：' + ' '.join(args))
    return result.stdout.strip()


def validate_name(name):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', name):
        raise ValueError('地图名请使用 1–80 个英文字母、数字、下划线或短横线')
    return name


def guard_reason(source, latest, now, mode='ekf'):
    for topic, label in [(source, '原始轮速输入'), ('/odom', '建图里程计'), ('/scan', '合并雷达')]:
        item = latest.get(topic)
        if not topic or item is None or now - item[0] > .5:
            return label + '无新数据；请先修复里程计接线并启动数据源'
        stamp_age = item[1].get('stamp_age', 0) + now - item[0]
        if not math.isfinite(stamp_age) or abs(stamp_age) > .5:
            return label + '时间戳异常'
    state = latest['/odom'][1]
    values = [state.get(k) for k in ('variance', 'vx', 'vy', 'wz')]
    if any(v is None or not math.isfinite(v) for v in values):
        return '建图里程计包含无效数值'
    if mode == 'ekf' and state['variance'] > 1:
        return '滤波位置不确定度过大；先修复并重新初始化 EKF'
    return None


class ROS:
    def __init__(self):
        import rclpy
        from nav_msgs.msg import OccupancyGrid, Odometry
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        import tf2_ros
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('fairino_map_control_' + str(os.getpid()))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.latest = {}
        self.grid = None
        self.subscriptions = []
        self.source = None
        self.odom_mode = 'ekf'
        self.raw_topics = set()

        def odom_callback(topic):
            def cb(msg):
                t = msg.twist.twist
                self.latest[topic] = (time.monotonic(), {
                    'variance': max(msg.pose.covariance[0], msg.pose.covariance[7]),
                    'vx': t.linear.x, 'vy': t.linear.y, 'wz': t.angular.z,
                    'stamp_age': self.stamp_age(msg)})
            return cb
        self.odom_callback = odom_callback
        self.subscriptions.append(self.node.create_subscription(Odometry, '/odom', odom_callback('/odom'), 20))
        self.subscriptions.append(self.node.create_subscription(
            LaserScan, '/scan', lambda m: self.latest.update({'/scan': (time.monotonic(), {'stamp_age': self.stamp_age(m)})}),
            qos_profile_sensor_data))
        self.subscriptions.append(self.node.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def stamp_age(self, msg):
        return self.node.get_clock().now().nanoseconds * 1e-9 - msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9

    def on_map(self, msg):
        self.grid = msg

    def spin(self, seconds=.1):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=.02)

    def call(self, typ, name, request, timeout=8):
        client = self.node.create_client(typ, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError('服务不可用：' + name)
            future = client.call_async(request)
            self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            if not future.done():
                raise RuntimeError('服务响应超时：' + name)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def params(self, node, names):
        from rcl_interfaces.srv import GetParameters
        request = GetParameters.Request()
        request.names = names
        response = self.call(GetParameters, node + '/get_parameters', request)
        fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value'}
        return {k: getattr(v, fields[v.type]) if v.type in fields else None for k, v in zip(names, response.values)}

    def set_params(self, node, values):
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.parameter import Parameter
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(key, value=value).to_parameter_msg() for key, value in values.items()]
        result = self.call(SetParametersAtomically, node + '/set_parameters_atomically', request).result
        if not result.successful:
            raise RuntimeError('参数更新失败：' + node + '：' + result.reason)

    def refresh_source(self):
        from nav_msgs.msg import Odometry
        status = json.loads(ODOMETRY_STATUS.read_text()) if ODOMETRY_STATUS.is_file() else {}
        self.odom_mode = status.get('mode', 'ekf')
        if self.odom_mode == 'raw':
            self.source = status.get('source')
        else:
            self.source = self.params('/ekf_filter_node', ['odom0'])['odom0']
        if self.source and not self.source.startswith('/'):
            self.source = '/' + self.source
        if self.source and self.source not in self.raw_topics:
            self.subscriptions.append(self.node.create_subscription(
                Odometry, self.source, self.odom_callback(self.source), 20))
            self.raw_topics.add(self.source)

    def input_reason(self):
        reason = guard_reason(self.source, self.latest, time.monotonic(), self.odom_mode)
        if reason:
            return reason
        if self.odom_mode == 'raw':
            try:
                status = json.loads(ODOMETRY_STATUS.read_text())
                if abs(time.time()-status['time']) > 1.5 or not status['running'] or status.get('mode') != 'raw':
                    return '原始里程计直通未就绪；请启动底盘并等待健康反馈'
            except (OSError, ValueError, KeyError):
                return '无法确认原始里程计直通状态'
        from rclpy.time import Time
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'base_footprint', Time())
            if abs(self.stamp_age(transform)) > .5:
                return '里程计 TF 已过期'
        except Exception:
            return '等待 odom → base_footprint 的坐标变换'
        return None

    def require_good_input(self, wait=0):
        deadline = time.monotonic() + max(3, wait)
        good_since = None
        reason = '里程计尚未启动'
        while time.monotonic() < deadline:
            if not self.source:
                try:
                    self.refresh_source()
                except RuntimeError as exc:
                    reason = str(exc)
                    continue
            self.spin(.1)
            reason = self.input_reason()
            if reason:
                good_since = None
            else:
                good_since = good_since or time.monotonic()
                if time.monotonic() - good_since > 1:
                    return
        raise RuntimeError(reason or '输入尚未稳定，请稍后再试')

    def state(self, name):
        from lifecycle_msgs.srv import GetState
        return self.call(GetState, name + '/get_state', GetState.Request()).current_state.id

    def transition(self, name, value):
        from lifecycle_msgs.srv import ChangeState
        request = ChangeState.Request()
        request.transition.id = value
        if not self.call(ChangeState, name + '/change_state', request).success:
            raise RuntimeError('生命周期切换失败：' + name)

    def map_server(self, active):
        if active and not (PROJECT / 'maps/active/map.yaml').is_file():
            self.map_server(False)
            return False
        state = self.state('/map_server')
        if not active:
            if state == 3:
                self.transition('/map_server', 4)
            if self.state('/map_server') == 2:
                self.transition('/map_server', 2)
            return
        if state == 1:
            self.transition('/map_server', 1)
        if self.state('/map_server') == 2:
            self.transition('/map_server', 3)
        if self.state('/map_server') != 3:
            raise RuntimeError('已有地图服务未成功激活')
        return True

    def slam_present(self):
        return any(name == 'slam_toolbox' for name, _ in self.node.get_node_names_and_namespaces())

    def pause(self, wanted):
        from slam_toolbox.srv import Pause
        current = self.params('/slam_toolbox', ['paused_new_measurements'])['paused_new_measurements']
        if current is None:
            raise RuntimeError('无法确认建图暂停状态')
        if current != wanted:
            self.call(Pause, '/slam_toolbox/pause_new_measurements', Pause.Request())
        if self.params('/slam_toolbox', ['paused_new_measurements'])['paused_new_measurements'] != wanted:
            raise RuntimeError('建图暂停状态未达到要求')

    def save(self, name):
        from slam_toolbox.srv import SerializePoseGraph
        self.pause(True)
        interval = self.params('/slam_toolbox', ['map_update_interval'])['map_update_interval'] or 10
        self.spin(min(30, max(1, interval)) + .5)
        destination = MAPS / validate_name(name)
        destination.mkdir(parents=True, exist_ok=False)
        # Save a separate map and pose graph; never overwrite maps/active.
        run('ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(destination / 'map'),
            '-t', '/map', '--fmt', 'pgm', '--ros-args', '-p', 'save_map_timeout:=10.0', timeout=25)
        request = SerializePoseGraph.Request()
        request.filename = str(destination / 'map')
        result = self.call(SerializePoseGraph, '/slam_toolbox/serialize_map', request, timeout=15)
        if result.result != 0:
            raise RuntimeError('栅格地图已保存，但位姿图保存失败：' + str(destination))
        return str(destination)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def stop_slam():
    subprocess.run(['sudo', '-n', 'systemctl', 'stop', UNIT], check=False, capture_output=True)
    victims = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = entry.joinpath('cmdline').read_bytes().split(b'\0')
            if args and Path(os.fsdecode(args[0])).name == 'sync_slam_toolbox_node' and b'__node:=slam_toolbox' in args:
                victims.append(int(entry.name))
                os.kill(int(entry.name), signal.SIGINT)
        except (OSError, ProcessLookupError):
            continue
    deadline = time.monotonic() + 8
    while any(Path('/proc', str(pid), 'cmdline').exists() for pid in victims) and time.monotonic() < deadline:
        time.sleep(.1)
    for pid in victims:
        p = Path('/proc', str(pid), 'cmdline')
        if p.exists() and p.read_bytes():
            raise RuntimeError('SLAM 进程未退出，保留现场，不启动静态地图')


def clear_libraries(ros):
    if ros.slam_present():
        ros.save('archive-before-clear-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        stop_slam()
    from localization import Localization
    Localization(ros).stop('清理地图库，定位已停止')
    ros.map_server(False)
    archive = RUNTIME / 'cleared-libraries' / datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    archive.mkdir(parents=True)
    manifest = {'archive': str(archive), 'moved': []}
    roots = [Path('/opt/fairino_AMR_project_A/maps'), Path('/opt/fairino_AMR_project_B/maps'), Path.home() / 'maps']
    for index, source in enumerate(roots):
        if not source.exists():
            continue
        files = [str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()]
        destination = archive / ('library-' + str(index))
        run('sudo', '-n', 'mv', '--', str(source), str(destination))
        run('sudo', '-n', 'install', '-d', '-o', 'fairino', '-g', 'fairino', '-m', '0755', str(source))
        (source / ('manual' if source == Path.home() / 'maps' else 'active')).mkdir()
        manifest['moved'].append({'from': str(source), 'to': str(destination), 'files': files})
    (archive / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    return manifest


def serve_mapping():
    ros = ROS()
    child = None
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        ros.require_good_input()
        epoch = read_epoch(ODOMETRY_EPOCH)
        RUNTIME.mkdir(parents=True, exist_ok=True)
        MAPPING_EPOCH.write_text(epoch or '')
        child = subprocess.Popen(mapping_command())
        started = time.monotonic()
        last_guard = 0
        while running and child.poll() is None:
            ros.spin(.1)
            if not running or child.poll() is not None:
                break
            if time.monotonic() - started < 5 or time.monotonic() - last_guard < 1:
                continue
            last_guard = time.monotonic()
            reason = ros.input_reason()
            if read_epoch(ODOMETRY_EPOCH) != epoch:
                reason = '里程计会话已变化；旧地图不允许继续累计'
            if reason:
                ros.pause(True)
                print('自动暂停建图：' + reason, flush=True)
                # Remain running, so the user can save or explicitly resume.
                while running and child.poll() is None:
                    ros.spin(.2)
                    if time.monotonic() - last_guard > 2:
                        last_guard = time.monotonic()
                        if not running or child.poll() is not None:
                            break
                        try:
                            paused = ros.params('/slam_toolbox', ['paused_new_measurements'])['paused_new_measurements']
                        except RuntimeError:
                            if not running or child.poll() is not None:
                                break
                            raise
                        if not paused:
                            break
    finally:
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                child.terminate()
                child.wait(timeout=5)
        ros.close()


def main():
    from localization import Localization, read_session, watch
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['status', 'check', 'start', 'pause', 'resume', 'view', 'save', 'use', 'clear', 'serve', 'maps', 'localize', 'stop-localization', 'initial-pose', 'watch-localization'])
    parser.add_argument('--name')
    parser.add_argument('--wait', type=float, default=0)
    parser.add_argument('--session-id')
    parser.add_argument('--x', type=float)
    parser.add_argument('--y', type=float)
    parser.add_argument('--yaw', type=float, help='map frame yaw, radians')
    parser.add_argument('--xy-std', type=float, default=.5)
    parser.add_argument('--yaw-std', type=float, default=math.radians(15))
    args = parser.parse_args()
    if not math.isfinite(args.wait) or not 0 <= args.wait <= 300:
        parser.error('--wait 必须在 0 到 300 秒之间')
    if args.action == 'initial-pose' and any(v is None for v in (args.x, args.y, args.yaw)):
        parser.error('initial-pose requires --x --y --yaw')

    if args.action == 'serve':
        serve_mapping()
        return
    ros = ROS()
    lock = None
    try:
        if args.action == 'watch-localization':
            watch(ros, mode_lock)
            return
        if args.action not in ('status', 'maps', 'check'):
            lock = mode_lock()
            try:
                lock.__enter__()
            except BlockingIOError:
                lock = None
                raise RuntimeError('另一项地图/定位操作正在进行，请稍后再试')
        ros.spin(2)
        result = {'action': args.action}
        if args.action == 'status':
            result['map_publishers'] = [p.node_name for p in ros.node.get_publishers_info_by_topic('/map')]
            result['slam_present'] = ros.slam_present()
            result['map_server_state'] = ros.state('/map_server')
            result['localization'] = read_session()
            result['amcl_state'] = ros.state('/amcl')
            result['has_saved_map'] = (PROJECT / 'maps/active/map.yaml').is_file()
            if result['slam_present']:
                result.update(ros.params('/slam_toolbox', ['mode', 'paused_new_measurements']))
        elif args.action == 'maps':
            result['maps'] = [p.name for p in sorted(MAPS.iterdir())
                              if p.is_dir() and (p/'map.yaml').is_file() and (p/'map.pgm').is_file()] if MAPS.is_dir() else []
            active = PROJECT / 'maps/active/map.yaml'
            result['selected'] = str(active.resolve()) if active.is_file() else None
        elif args.action == 'localize':
            # Verify the watchdog has been installed before lifecycle changes.
            run('systemctl', 'cat', LOCALIZATION_UNIT)
            result['localization'] = Localization(ros).start(PROJECT/'maps/active/map.yaml', args.wait)
            try:
                run('sudo', '-n', 'systemctl', 'restart', LOCALIZATION_UNIT)
                run('systemctl', 'is-active', '--quiet', LOCALIZATION_UNIT)
            except Exception:
                Localization(ros).stop('定位监视服务启动失败')
                raise
            result['message'] = 'AMCL 已启动，请在地图上设置初始位置与朝向'
        elif args.action == 'stop-localization':
            Localization(ros).stop()
            result['message'] = '定位已停止，保留静态底图'
        elif args.action == 'initial-pose':
            result['initial_pose'] = Localization(ros).set_pose(args.x, args.y, args.yaw, args.xy_std, args.yaw_std, args.session_id)
            result['message'] = '初始位姿请求已送达；请等待雷达匹配，并检查墙面与地图是否重合'
        elif args.action == 'check':
            ros.require_good_input(args.wait)
            result['message'] = '建图输入正在更新；精度仍需实际运动验证'
        elif args.action == 'start':
            if ros.slam_present():
                raise RuntimeError('已有建图会话，请暂停/继续或保存后返回已有地图')
            mapping_command()  # Fail before changing map mode if deployment is incomplete.
            ros.require_good_input(args.wait)
            Localization(ros).stop('已切换到建图')
            ros.map_server(False)
            try:
                run('sudo', '-n', 'systemctl', 'start', UNIT)
                ros.spin(6)
                if not ros.slam_present():
                    raise RuntimeError('SLAM 未成功启动，请查看建图服务日志')
            except Exception:
                stop_slam()
                ros.map_server(True)
                raise
            result['message'] = '新建图已开始'
        elif args.action == 'pause':
            ros.pause(True)
            result['message'] = '已停止接收新的建图扫描；工作地图仍可保存'
        elif args.action == 'resume':
            ros.require_good_input(args.wait)
            require_same_epoch()
            ros.pause(False)
            result['message'] = '已继续建图'
        elif args.action == 'save':
            result['saved_directory'] = ros.save(args.name or datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
            result['message'] = '地图和位姿图已另存；建图保持暂停，未覆盖默认地图'
        elif args.action == 'view':
            Localization(ros).stop('已返回静态底图')
            if ros.slam_present():
                result['archived_directory'] = ros.save('archive-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
                stop_slam()
                ros.spin(2)
            loaded = ros.map_server(True)
            result['message'] = '已切换到保存好的静态地图，当前未运行建图' if loaded else '已结束建图；默认底图为空'
        elif args.action == 'clear':
            result.update(clear_libraries(ros))
            result['message'] = '所有项目地图及手动地图已移出地图库；默认底图为空，恢复备份已保留'
        elif args.action == 'use':
            name = validate_name(args.name or '')
            selected = MAPS / name
            if not (selected / 'map.yaml').is_file() or not (selected / 'map.pgm').is_file():
                raise RuntimeError('地图不存在或不完整：' + str(selected))
            if ros.slam_present():
                result['archived_directory'] = ros.save('archive-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
                stop_slam()
            Localization(ros).stop('地图已切换，请重新启动定位并设置初始位姿')
            ros.map_server(False)
            active = PROJECT / 'maps/active'
            backup = RUNTIME / 'replaced-defaults' / datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
            backup.parent.mkdir(parents=True, exist_ok=True)
            if active.exists() or active.is_symlink():
                shutil.move(str(active), str(backup))
            active.symlink_to(selected, target_is_directory=True)
            ros.set_params('/map_server', {'yaml_filename': str(active/'map.yaml')})
            ros.map_server(True)
            result.update(message='已明确选用该地图作为固定默认底图', selected=str(selected), previous_default=str(backup))
        print('MAP_CONTROL ' + json.dumps(result, ensure_ascii=False), flush=True)
    finally:
        ros.close()
        if lock is not None:
            lock.__exit__(None, None, None)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('MAP_CONTROL ' + json.dumps({'error': str(exc)}, ensure_ascii=False), flush=True)
        raise SystemExit(1)
