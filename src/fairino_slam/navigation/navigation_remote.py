#!/usr/bin/env python3
"""Explicit operations on the isolated Nav2 stack; no motor enable commands."""
import argparse
import fcntl
import uuid
import json
import math
from pathlib import Path
import subprocess
import time

HERE = Path(__file__).resolve().parent
UNIT = 'fairino-navigation.service'
PREFIX = 'NAVIGATION '
SAVED_MAP = Path('/opt/fairino_AMR_project/maps/active/map.yaml')
SESSION_FILE = Path.home()/'.local/share/fairino-map-control/localization.json'

if __package__:
    from .tf_receiver import TfReceiver
    from .runtime_policy import (TF_MAX_AGE, TF_READY_WAIT, PREVIEW_MAX_ATTEMPTS,
                                 PREVIEW_RETRY_DELAY, PREVIEW_TIMEOUT)
else:
    from tf_receiver import TfReceiver
    from runtime_policy import (TF_MAX_AGE, TF_READY_WAIT, PREVIEW_MAX_ATTEMPTS,
                                PREVIEW_RETRY_DELAY, PREVIEW_TIMEOUT)


HEADING_MODES = ('hold_initial', 'path_tangent')


def deployed_heading_mode():
    """Read the same generated configuration loaded by the deployed Nav2 launch."""
    import yaml
    path = HERE/'nav2.yaml'
    try:
        config = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError('无法读取已部署导航朝向配置 nav2.yaml；请完整部署导航组件') from exc
    try:
        mode = config['controller_server']['ros__parameters']['FollowPath']['heading_mode']
    except (KeyError, TypeError) as exc:
        raise RuntimeError('已部署 nav2.yaml 缺少 FollowPath.heading_mode；请完整部署导航组件') from exc
    if not isinstance(mode, str) or mode not in HEADING_MODES:
        raise RuntimeError('已部署导航朝向策略无效：'+repr(mode)+'；请检查并重新部署导航配置')
    return mode


def require_heading_mode(expected=None):
    # An omitted expectation belongs to the legacy hold-initial interface.
    # Never infer path-tangent behavior for a client unaware of the new policy.
    expected = 'hold_initial' if expected is None else expected
    deployed = deployed_heading_mode()
    if deployed != expected:
        raise RuntimeError(f'导航朝向策略不一致：客户端要求 {expected}，机器人已部署 {deployed}；'
                           '请统一配置并重新部署，未发送目标')
    return deployed


def status():
    try:
        result = json.loads((HERE/'status.json').read_text())
    except (OSError, ValueError):
        result = {}
    try:
        result['running'] = subprocess.run(
            ['systemctl', 'is-active', '--quiet', UNIT], timeout=1.).returncode == 0
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError('导航服务状态查询超时；状态未知，不会自动重试') from exc
    if not result['running'] or abs(time.time()-result.get('time', 0)) > 1:
        result.update(localization_ready=False, controller_ready=False, prepared=False, stale=True)
    try:
        result['heading_mode'] = deployed_heading_mode()
        result.pop('heading_mode_error', None)
    except RuntimeError as exc:
        result['heading_mode'] = None
        result['heading_mode_error'] = str(exc)
    return result


def require_chassis_motion(state):
    # Older status readers omit this field; the current gate always reports it
    # and includes feedback readiness in controller_ready.
    if 'chassis_motion' not in state:
        return
    motion = state['chassis_motion']
    if not isinstance(motion, dict) or motion.get('ready') is not True:
        reason = motion.get('reason') if isinstance(motion, dict) else None
        if not isinstance(reason, str) or not reason:
            reason = '底盘运动反馈无效；请完整更新导航、重新打开面板并接入导航底盘'
        raise RuntimeError('底盘协调尚未就绪：'+reason)


def require_ready():
    state = status()
    require_chassis_motion(state)
    if not state.get('localization_ready') or not state.get('controller_ready'):
        raise RuntimeError('导航/定位尚未就绪：'+state.get('localization_reason', '请先启动底盘里程计、地图定位及导航服务'))
    return state



def prepare():
    # The same kernel lock as the chassis worker prevents creating Nav2
    # participants while a keyboard/navigation motor session is running.
    with (Path.home()/'.fairino_keyboard.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('请先结束底盘控制，再准备导航')
        state = status()
        if state.get('goal') or (state.get('last_goal') or {}).get('status') in (1, 2, 3):
            raise RuntimeError('请先结束已有导航目标')
        subprocess.run(['sudo', '-n', 'systemctl', 'start', UNIT], check=True)
        if globals().get('__package__'):
            from .lifecycle import configure
        else:
            from lifecycle import configure
        configure()
        deadline = time.monotonic()+8
        stable_since = None
        while time.monotonic() < deadline:
            if status().get('prepared'):
                stable_since = stable_since or time.monotonic()
                if time.monotonic()-stable_since >= 2:
                    return {'message': '导航节点已配置待机；尚未激活或使能底盘'}
            else:
                stable_since = None
            time.sleep(.2)
        raise RuntimeError('导航待机状态未确认；不会接入底盘')


def activate():
    # No rclpy node, process start or DDS discovery on the enabled chassis path.
    with (HERE/'activation.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        state = status()
        if not state.get('running') or state.get('stale') or not state.get('run_id'):
            raise RuntimeError('请先接入导航底盘，完成导航节点准备')
        require_chassis_motion(state)
        if not state.get('localization_ready'):
            raise RuntimeError('定位尚未就绪：'+state.get('localization_reason', ''))
        if state.get('activation', {}).get('state') == 'pending':
            raise RuntimeError('上一次导航激活尚未结束，请等待状态确认')
        request = {'id': uuid.uuid4().hex, 'run_id': state['run_id'],
                   'session': state.get('session_token'), 'expires': time.time()+35}
        temp = HERE/'activation-request.tmp'
        temp.write_text(json.dumps(request))
        temp.replace(HERE/'activation-request.json')
        deadline = time.monotonic()+40
        while time.monotonic() < deadline:
            result = status()
            activation = result.get('activation') or {}
            if activation.get('id') == request['id']:
                if activation.get('state') == 'failed':
                    raise RuntimeError(activation.get('error', '导航激活失败'))
                if activation.get('state') == 'completed' and result.get('controller_ready'):
                    return {'message': '导航节点已激活；未发送运动目标'}
            if result.get('run_id') != request['run_id'] or not result.get('running'):
                raise RuntimeError('导航进程已改变或停止')
            time.sleep(.2)
        raise RuntimeError('导航激活未获确认；请查询状态，勿重复点击')



def finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def transient_tf(state, initial_stamp):
    """Only known missing-TF or stale-TF evidence can extend a preflight wait."""
    tf = state.get('tf')
    health = state.get('localization_health')
    if (not isinstance(tf, dict) or not isinstance(health, dict)
            or tf.get('ready') is not False
            or tf.get('initial_pose_stamp') != initial_stamp
            or health.get('reason') != tf.get('reason')):
        return False
    transforms = tf.get('transforms')
    combined = transforms.get('map_base') if isinstance(transforms, dict) else None
    if not isinstance(combined, dict):
        return False
    reason = tf.get('reason')
    if not isinstance(reason, str):
        return False
    error = combined.get('error')
    if error is not None:
        return (isinstance(error, str)
                and error.partition(':')[0] in
                ('LookupException', 'ConnectivityException', 'ExtrapolationException')
                and reason.startswith('定位 TF 未就绪：map → base_footprint 查询失败'))
    return (reason.startswith('定位 TF 已过期 ')
            and finite(combined.get('age')) and combined['age'] >= TF_MAX_AGE
            and finite(combined.get('stamp')) and combined['stamp'] >= initial_stamp)


def session_snapshot(expected_id=None, allow_tf_wait=False):
    """Read one stable request identity without treating every outage as retryable."""
    state = status()
    if (state.get('running') is not True or state.get('stale') or not state.get('run_id')):
        raise RuntimeError('导航进程或控制服务未就绪；未发送导航目标')
    require_chassis_motion(state)
    if state.get('controller_ready') is not True:
        raise RuntimeError('导航进程或控制服务未就绪；未发送导航目标')
    last = state.get('last_goal')
    if not isinstance(last, dict):
        raise RuntimeError('导航目标状态未知；未发送导航目标')
    if state.get('goal') or (last and last.get('status') in (1, 2, 3)):
        raise RuntimeError('已有导航目标，请先 cancel 再发送新目标')
    if last and (type(last.get('status')) is not int or last['status'] not in (4, 5, 6)):
        raise RuntimeError('导航目标状态未知；未发送导航目标')
    try:
        session = json.loads(SESSION_FILE.read_text())
    except (OSError, ValueError) as exc:
        raise RuntimeError('无法读取定位会话；未发送导航目标') from exc
    if not isinstance(session, dict):
        raise RuntimeError('定位会话无效；未发送导航目标')
    stamp = session.get('initial_pose_stamp')
    if (session.get('active') is not True or not session.get('id')
            or not finite(stamp) or stamp <= 0
            or not isinstance(session.get('map'), str) or not session['map']
            or state.get('session') != session['id']
            or expected_id is not None and expected_id != session['id']):
        raise RuntimeError('地图或定位会话已变化，请重新设置目标')
    token = state.get('session_token')
    if (not isinstance(token, (list, tuple)) or len(token) != 3
            or token[0] != session['id'] or token[1] != stamp or not token[2]
            or token[2] != session.get('epoch')):
        raise RuntimeError('定位/里程计会话尚未确认；未发送导航目标')
    failure = state.get('failure')
    failed_goal = state.get('failed_goal')
    if (failure is not None and not isinstance(failure, dict)
            or failed_goal is not None and not isinstance(failed_goal, str)):
        raise RuntimeError('导航保护状态未知；未发送导航目标')
    try:
        protection = json.dumps([failed_goal, failure], sort_keys=True, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise RuntimeError('导航保护状态无效；未发送导航目标') from exc
    identity = {'id': session['id'], 'map': session['map'], 'initial_pose_stamp': stamp,
                'odometry_epoch': token[2], 'run_id': state['run_id'],
                'protection': protection}
    if state.get('localization_ready') is not True:
        if (state.get('localization_ready') is not False or not allow_tf_wait
                or not transient_tf(state, stamp)):
            raise RuntimeError('导航/定位尚未就绪：'+state.get('localization_reason', '状态未知'))
    return identity, state


def heading_session(expected_id=None, allow_tf_wait=False):
    return session_snapshot(expected_id, allow_tf_wait)[0]


def check_session(expected, allow_tf_wait=False):
    identity, state = session_snapshot(expected['id'], allow_tf_wait)
    if identity != expected:
        raise RuntimeError('等待或发送前地图、定位、里程计、导航会话已变化，或出现新的保护故障；未发送目标')
    return state


class HeadingTransformError(ValueError):
    """A structured TF validation failure; only genuinely old samples may wait."""
    def __init__(self, reason_code, message, stamp, now, initial_pose_stamp):
        self.reason_code = reason_code
        self.age = now-stamp if finite(now) and finite(stamp) else None
        self.stamp, self.initial_pose_stamp = stamp, initial_pose_stamp
        def label(value):
            return f'{value:.6f}' if finite(value) else repr(value)
        details = (f'age={label(self.age)}s，要求 -0.050 <= age < {TF_MAX_AGE:.3f}s；'
                   f'stamp={label(stamp)}，initial={label(initial_pose_stamp)}')
        super().__init__(message+'（'+details+'）')


def heading_from_transform(transform, now, initial_pose_stamp):
    """Validate full TF pose with distinct stale, pre-init and clock diagnostics."""
    t = transform.transform.translation
    q = transform.transform.rotation
    stamp = transform.header.stamp.sec + transform.header.stamp.nanosec*1e-9
    def invalid(code, message):
        return HeadingTransformError(code, message, stamp, now, initial_pose_stamp)
    if not finite(now) or now <= 0:
        raise invalid('invalid_clock', '当前定位 TF 校验时钟无效')
    if not finite(stamp) or stamp <= 0:
        raise invalid('invalid_stamp', '当前定位 TF 时间戳无效')
    if not finite(initial_pose_stamp) or initial_pose_stamp <= 0:
        raise invalid('invalid_initialization', '本次初始化时间戳无效')
    if (any(not finite(value) for value in (t.x, t.y, t.z, q.x, q.y, q.z, q.w))
            or transform.header.frame_id != 'map' or transform.child_frame_id != 'base_footprint'):
        raise invalid('invalid_pose', '当前定位 TF 的坐标系或位姿无效')
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if abs(norm-1.) > 1e-3:
        raise invalid('invalid_orientation', '当前定位 TF 朝向四元数无效')
    if stamp < initial_pose_stamp:
        raise invalid('pre_initialization', '当前定位 TF 早于本次初始化')
    age = now-stamp
    if age < -.05:
        raise invalid('future', '当前定位 TF 时间超前')
    if age >= TF_MAX_AGE:
        raise invalid('stale', '当前定位 TF 已过期')
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
    return {'x': t.x, 'y': t.y, 'yaw': yaw, 'stamp': stamp}


def create_heading_receiver():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    import tf2_ros
    return TfReceiver(rclpy, tf2_ros, SingleThreadedExecutor,
                      node_name='navigation_command_tf',
                      thread_name='fairino-navigation-command-tf')


def read_heading(node, receiver, expected):
    """Sample the independently updated buffer after all synchronous checks."""
    from rclpy.time import Time
    health = receiver.status()
    if health.get('running') is not True or health.get('error'):
        raise RuntimeError('定位 TF 接收器未运行：'+str(health.get('error') or '接收线程已停止'))
    transform = receiver.buffer.lookup_transform('map', 'base_footprint', Time())
    return heading_from_transform(transform, node.get_clock().now().nanoseconds*1e-9,
                                  expected['initial_pose_stamp'])


def capture_heading(node, expected, timeout=TF_READY_WAIT, *, deadline=None, receiver=None):
    """Wait boundedly while an isolated receiver keeps TF current during checks."""
    import rclpy
    import tf2_ros
    deadline = time.monotonic()+timeout if deadline is None else deadline
    owned = receiver is None
    receiver = create_heading_receiver() if owned else receiver
    reason = '尚未收到 map 到 base_footprint 的 TF'
    try:
        while time.monotonic() < deadline:
            state = check_session(expected, allow_tf_wait=True)
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                reason = '导航会话检查耗尽朝向等待预算；最后诊断：'+reason
                break
            # Only the action node is spun here. TF subscriptions belong to the
            # receiver's separate node/executor and run during blocking checks.
            rclpy.spin_once(node, timeout_sec=min(.05, remaining))
            if state.get('localization_ready') is not True:
                reason = state.get('localization_reason', reason)
                continue
            state = check_session(expected, allow_tf_wait=True)
            if time.monotonic() >= deadline:
                reason = '导航会话复核耗尽朝向等待预算；最后诊断：'+reason
                break
            if state.get('localization_ready') is not True:
                reason = state.get('localization_reason', reason)
                continue
            try:
                # The last synchronous state check can be slow; never return
                # a pose sampled before that check.
                pose = read_heading(node, receiver, expected)
            except tf2_ros.TransformException as exc:
                if type(exc).__name__ not in ('LookupException', 'ConnectivityException',
                                               'ExtrapolationException'):
                    raise RuntimeError('当前定位 TF 查询无效；未发送导航目标') from exc
                reason = str(exc)
                continue
            except HeadingTransformError as exc:
                if exc.reason_code != 'stale':
                    raise RuntimeError(str(exc)+'；未发送导航目标') from exc
                reason = str(exc)
                continue
            if time.monotonic() >= deadline:
                reason = '定位 TF 读取耗尽朝向等待预算；最后诊断：'+reason
                break
            return pose
        raise RuntimeError('无法锁定当前朝向：'+reason+'；等待已结束，未发送导航目标')
    finally:
        if owned:
            receiver.close()


def request_timeout(deadline, maximum):
    remaining = deadline-time.monotonic() if deadline is not None else maximum
    if remaining <= 0:
        raise RuntimeError('路径预览总等待时间已用尽；不会继续重试')
    return min(maximum, remaining)


def wait_retry(node, expected, deadline):
    """Wait between confirmed planning aborts while monitoring request identity."""
    import rclpy
    retry_at = time.monotonic()+PREVIEW_RETRY_DELAY
    while time.monotonic() < retry_at:
        check_session(expected, allow_tf_wait=True)
        remaining = min(retry_at-time.monotonic(), request_timeout(deadline, .05))
        if remaining > 0:
            rclpy.spin_once(node, timeout_sec=remaining)



def require_saved_map(source, modified):
    try:
        valid = (isinstance(source, str) and source and finite(modified) and
                 str(SAVED_MAP.resolve(strict=True)) == source and SAVED_MAP.stat().st_mtime == modified)
    except OSError:
        valid = False
    if not valid:
        raise RuntimeError('保存点所属地图或地图版本已变化；未发送目标')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['prepare', 'start', 'stop', 'status', 'plan', 'goal', 'cancel'])
    for name in ('x', 'y'):
        parser.add_argument('--'+name, type=float)
    parser.add_argument('--yaw', type=float, help='终点朝向（rad）；hold_initial 配合 --arrival-heading 到位后转向')
    parser.add_argument('--heading-mode', choices=HEADING_MODES,
                        help='客户端预期的已部署朝向策略；省略仅兼容 hold_initial')
    parser.add_argument('--session-id')
    parser.add_argument('--arrival-heading', type=int, choices=(0, 1), default=0)
    parser.add_argument('--saved-pose', type=int, choices=(0, 1), default=0)
    parser.add_argument('--saved-map-source')
    parser.add_argument('--saved-map-modified', type=float)
    args = parser.parse_args()
    if args.action in ('plan', 'goal') and not all(v is not None and math.isfinite(v) for v in (args.x, args.y)):
        parser.error('plan/goal requires finite --x --y (metres)')
    if args.action == 'status':
        return status()
    if args.action == 'prepare':
        return prepare()
    if args.action == 'start':
        selected = Path('/opt/fairino_AMR_project/maps/active/map.yaml')
        if not selected.is_file():
            raise RuntimeError('默认地图为空；先建图、保存并明确选用地图')
        session = json.loads((Path.home()/'.local/share/fairino-map-control/localization.json').read_text())
        if not session.get('active') or session.get('initial_pose_stamp') is None:
            raise RuntimeError('请先启动定位并设置初始位姿')
        return activate()
    if args.action == 'stop':
        subprocess.run(['sudo', '-n', 'systemctl', 'stop', UNIT], check=True)
        return {'message': '导航服务已停止；底盘执行终端 Ctrl+C 可下使能'}
    if args.action in ('plan', 'goal'):
        heading_mode = require_heading_mode(args.heading_mode)
        if (heading_mode == 'path_tangent' or args.saved_pose or args.arrival_heading) and not finite(args.yaw):
            raise RuntimeError('终点转向需要有限的目标 --yaw（弧度）；未发送目标')
        if args.saved_pose:
            require_saved_map(args.saved_map_source, args.saved_map_modified)
            if not (HERE/'navigate_to_saved_pose.xml').is_file():
                raise RuntimeError('机器人尚未部署保存点导航组件；未发送目标')
        if args.arrival_heading and not (HERE/'navigate_to_saved_pose.xml').is_file():
            raise RuntimeError('终点转向组件尚未部署；未发送目标')
        expected_session = heading_session(args.session_id, allow_tf_wait=True)
    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from action_msgs.srv import CancelGoal
    rclpy.init()
    node = rclpy.create_node('fairino_navigation_command')
    receiver = None
    def wait(future, seconds):
        rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
        if not future.done():
            raise RuntimeError('导航请求超时；请查询 status，勿盲目重复发送目标')
        return future.result()
    try:
        if args.action == 'cancel':
            client = node.create_client(CancelGoal, '/fairino_navigation/navigate_to_pose/_action/cancel_goal')
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('导航目标服务不可用')
            result = wait(client.call_async(CancelGoal.Request()), 5)
            return {'message': '已请求取消导航目标', 'return_code': result.return_code,
                    'goals_canceling': len(result.goals_canceling)}
        preview_deadline = time.monotonic()+PREVIEW_TIMEOUT if args.action == 'plan' else None
        receiver = create_heading_receiver()
        action = ComputePathToPose if args.action == 'plan' else NavigateToPose
        topic = 'compute_path_to_pose' if args.action == 'plan' else 'navigate_to_pose'
        client = ActionClient(node, action, '/fairino_navigation/'+topic)
        if not client.wait_for_server(timeout_sec=request_timeout(preview_deadline, 5)):
            raise RuntimeError('导航动作服务不可用：'+topic)
        attempts = PREVIEW_MAX_ATTEMPTS if args.action == 'plan' else 1
        for attempt in range(1, attempts+1):
            if attempt > 1:
                wait_retry(node, expected_session, preview_deadline)
            ready_deadline = time.monotonic()+request_timeout(preview_deadline, TF_READY_WAIT)
            capture_heading(node, expected_session, deadline=ready_deadline, receiver=receiver)
            check_session(expected_session)
            require_heading_mode(heading_mode)
            # The receiver stays live while session and deployed-mode checks
            # block. Refresh the actual send pose after both have completed.
            if time.monotonic() >= ready_deadline:
                raise RuntimeError('发送前检查耗尽朝向等待预算；未发送导航目标')
            try:
                start_pose = read_heading(node, receiver, expected_session)
            except HeadingTransformError as exc:
                raise RuntimeError('发送前'+str(exc)+'；未发送导航目标') from exc
            if args.saved_pose:
                require_saved_map(args.saved_map_source, args.saved_map_modified)
            goal_yaw = args.yaw if heading_mode == 'path_tangent' or args.saved_pose or args.arrival_heading else start_pose['yaw']
            goal = action.Goal()
            if args.action == 'goal' and (args.saved_pose or args.arrival_heading) and heading_mode == 'hold_initial':
                goal.behavior_tree = str(HERE/'navigate_to_saved_pose.xml')
            pose = goal.goal if args.action == 'plan' else goal.pose
            pose.header.frame_id = 'map'
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = args.x, args.y
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(goal_yaw/2), math.cos(goal_yaw/2)
            if args.action == 'plan':
                goal.planner_id = 'GridBased'
                goal.use_start = False
            ack_timeout = request_timeout(preview_deadline, 5)
            handle = wait(client.send_goal_async(goal), ack_timeout)
            if not handle.accepted:
                raise RuntimeError('导航服务拒绝目标；不会自动重试')
            ignored_yaw = heading_mode == 'hold_initial' and not (args.saved_pose or args.arrival_heading) and args.yaw is not None
            heading = {'heading_policy': heading_mode, 'goal_yaw': goal_yaw,
                       'locked_yaw': goal_yaw, 'heading_stamp': start_pose['stamp'],
                       'start_pose': start_pose, 'requested_yaw_ignored': ignored_yaw}
            degrees = math.degrees(goal_yaw)
            angle_note = f'{degrees:.1f}°' if math.isfinite(degrees) else f'{goal_yaw:.6g} rad'
            note = (f"保持启动朝向 {angle_note}" if heading_mode == 'hold_initial'
                    else f"沿路径方向行驶；目标朝向 {angle_note}")
            if (args.saved_pose or args.arrival_heading) and heading_mode == 'hold_initial':
                note = f'先保持启动朝向到达位置，再对齐目标朝向 {angle_note}'
            if ignored_yaw:
                note += '（已忽略输入的 --yaw）'
            if args.action == 'goal':
                return {'message': '目标已接收；'+note+'；由状态查询确认结果',
                        'goal_id': bytes(handle.goal_id.uuid).hex(), **heading}
            try:
                result = wait(handle.get_result_async(), request_timeout(preview_deadline, 10))
            except Exception:
                # Cancellation acknowledgement is not proof of a final result.
                # Preserve the original error and never resend an uncertain action.
                try:
                    wait(handle.cancel_goal_async(), 3)
                except Exception:
                    pass
                raise
            check_session(expected_session, allow_tf_wait=True)
            require_heading_mode(heading_mode)
            if result.status == 6 and attempt < attempts:
                continue
            if result.status != 4:
                raise RuntimeError('二维路径规划失败，action status='+str(result.status)
                                   +f'；已尝试 {attempt}/{attempts} 次')
            points = [[p.pose.position.x, p.pose.position.y] for p in result.result.path.poses]
            if not points:
                raise RuntimeError('规划器返回空路径；不会自动重试')
            return {'message': '仅计算路径，未发送运动目标；'+note,
                    'frame': 'map', 'points': points, 'attempts': attempt, **heading}

    finally:
        try:
            if receiver is not None:
                receiver.close()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        print(PREFIX+json.dumps(main(), ensure_ascii=False, allow_nan=False))
    except Exception as exc:
        print(PREFIX+json.dumps({'error': str(exc)}, ensure_ascii=False))
        raise SystemExit(1)
