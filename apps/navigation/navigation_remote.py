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


def status():
    try:
        result = json.loads((HERE/'status.json').read_text())
    except (OSError, ValueError):
        result = {}
    result['running'] = subprocess.run(['systemctl', 'is-active', '--quiet', UNIT]).returncode == 0
    if not result['running'] or abs(time.time()-result.get('time', 0)) > 1:
        result.update(localization_ready=False, controller_ready=False, prepared=False, stale=True)
    return result


def require_ready():
    state = status()
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
        # A stopped service leaves its last status file behind. That old goal
        # cannot survive a new process; do not let it permanently block prepare.
        # While the service is running, even stale active-goal data still blocks.
        if state.get('running') and (state.get('goal') or
                                     (state.get('last_goal') or {}).get('status') in (1, 2, 3)):
            raise RuntimeError('请先结束已有导航目标')
        subprocess.run(['sudo', '-n', 'systemctl', 'start', UNIT], check=True)
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['prepare', 'start', 'stop', 'status', 'plan', 'goal', 'cancel'])
    for name in ('x', 'y', 'yaw'):
        parser.add_argument('--'+name, type=float)
    parser.add_argument('--session-id')
    args = parser.parse_args()
    if args.action in ('plan', 'goal') and not all(v is not None and math.isfinite(v) for v in (args.x, args.y, args.yaw)):
        parser.error('plan/goal requires finite --x --y --yaw (metres/radians)')
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
        state = require_ready()
        if args.session_id:
            session = json.loads((Path.home()/'.local/share/fairino-map-control/localization.json').read_text())
            if (not session.get('active') or session.get('id') != args.session_id
                    or state.get('session') != args.session_id):
                raise RuntimeError('地图或定位会话已变化，请重新设置目标')
        if args.action == 'goal' and (state.get('goal') or state.get('last_goal', {}).get('status') in (1, 2, 3)):
            raise RuntimeError('已有导航目标，请先 cancel 再发送新目标')
    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from action_msgs.srv import CancelGoal
    rclpy.init()
    node = rclpy.create_node('fairino_navigation_command')
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
        action = ComputePathToPose if args.action == 'plan' else NavigateToPose
        topic = 'compute_path_to_pose' if args.action == 'plan' else 'navigate_to_pose'
        client = ActionClient(node, action, '/fairino_navigation/'+topic)
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('导航动作服务不可用：'+topic)
        goal = action.Goal()
        pose = goal.goal if args.action == 'plan' else goal.pose
        pose.header.frame_id = 'map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = args.x, args.y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(args.yaw/2), math.cos(args.yaw/2)
        if args.action == 'plan':
            goal.planner_id = 'GridBased'
            goal.use_start = False
        handle = wait(client.send_goal_async(goal), 5)
        if not handle.accepted:
            raise RuntimeError('导航服务拒绝目标')
        if args.action == 'goal':
            return {'message': '目标已接收；由状态查询确认结果', 'goal_id': bytes(handle.goal_id.uuid).hex()}
        try:
            result = wait(handle.get_result_async(), 10)
        except Exception:
            wait(handle.cancel_goal_async(), 3)
            raise
        if result.status != 4:
            raise RuntimeError('二维路径规划失败，action status='+str(result.status))
        points = [[p.pose.position.x, p.pose.position.y] for p in result.result.path.poses]
        if not points:
            raise RuntimeError('规划器返回空路径')
        return {'message': '仅计算路径，未发送运动目标', 'frame': 'map', 'points': points}
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        print(PREFIX+json.dumps(main(), ensure_ascii=False, allow_nan=False))
    except Exception as exc:
        print(PREFIX+json.dumps({'error': str(exc)}, ensure_ascii=False))
        raise SystemExit(1)
