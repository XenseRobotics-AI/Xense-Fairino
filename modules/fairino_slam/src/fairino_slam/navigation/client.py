#!/usr/bin/env python3
"""Nav2 operations; drive explicitly enables the existing guarded chassis owner."""
import argparse
import datetime
import getpass
import json
import math
import os
from pathlib import Path
import shlex
import sys
import time

from fairino_slam.paths import LOG_ROOT
from fairino_slam.configuration import load_navigation
from fairino_slam.connection.ssh import run
from fairino_slam.mapping.client import ENV

REMOTE = '/home/fairino/.local/share/fairino-navigation/navigation_remote.py'

HEADING_MODES = ('hold_initial', 'path_tangent')


def configured_heading_mode():
    mode = load_navigation()['nav2']['controller_server']['ros__parameters']['FollowPath']['heading_mode']
    if mode not in HEADING_MODES:
        raise ValueError('未知导航朝向策略：'+str(mode))
    return mode


def request_pose(pose):
    """Bind a request to the expected deployed strategy; never change it remotely."""
    result = dict(pose or {})
    mode = result.get('heading_mode')
    if mode is None:
        mode = configured_heading_mode()
    if mode not in HEADING_MODES:
        raise ValueError('未知导航朝向策略：'+str(mode))
    result['heading_mode'] = mode
    yaw = result.get('yaw')
    if (mode == 'path_tangent' or result.get('saved_pose') or result.get('arrival_heading')) and (
            isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw)):
        raise ValueError('沿路径朝向策略或终点转向需要有限的目标朝向 --yaw，单位弧度')
    return result


def execute(action, host, user, password, pose=None):
    command = ['python3', REMOTE, action]
    if action in ('plan', 'goal'):
        pose = request_pose(pose)
    if pose:
        for key, value in pose.items():
            command += ['--'+key.replace('_', '-'), str(value)]
    try:
        output = run(host, ENV+shlex.join(command), user=user, password=password, timeout=60)
    except RuntimeError as exc:
        output = str(exc)
    rows = [line[len('NAVIGATION '):] for line in output.splitlines() if line.startswith('NAVIGATION ')]
    if not rows:
        legacy_yaw = 'plan/goal requires finite --x --y --yaw' in output
        legacy_mode = any(flag in output for flag in ('--heading-mode', '--saved-pose', '--saved-map-source', '--saved-map-modified', '--arrival-heading')) and 'unrecognized argument' in output
        if action in ('plan', 'goal') and (legacy_yaw or legacy_mode):
            raise RuntimeError("机器人端导航程序仍为旧版，尚不支持当前朝向策略请求。请先结束导航并确认底盘下使能，再在控制电脑运行 bash scripts/run.sh deploy navigation --execute 更新完整导航组件，随后重启面板。未自动修改朝向或重试目标。")
        raise RuntimeError(output[-1500:])
    result = json.loads(rows[-1])
    if 'error' in result:
        raise RuntimeError(result['error'])
    return result


def drive(args, password):
    state = execute('status', args.host, args.user, password)
    if (state.get('goal') or state.get('last_goal', {}).get('status') in (1, 2, 3)) and not state.get('stale'):
        raise RuntimeError('请先取消已有导航目标，再接入底盘执行')
    from fairino_slam.chassis.control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR
    from fairino_slam.chassis.transport import Connection
    args.check = args.prepare_check = False
    args.navigation = True
    args.log_directory = str(LOG_ROOT/'navigation/drive')
    args.linear, args.angular = DEFAULT_LINEAR, DEFAULT_ANGULAR
    print('显式接入导航执行：将复用底盘回正、使能与反馈保护；Ctrl+C 停止并下使能。', flush=True)
    execute('prepare', args.host, args.user, password)
    connection = Connection(args, password=password)
    phase = None
    try:
        while not connection.done.is_set():
            connection.send(keys=(), focused=True)
            current = connection.latest.get('phase')
            if current != phase:
                phase = current
                print(current or '等待底盘反馈', flush=True)
            time.sleep(.05)
    except KeyboardInterrupt:
        pass
    finally:
        # Keep reading the worker's cleanup report; do not cut SSH before disable.
        if not connection.done.is_set():
            connection.send(stop=True)
            connection.done.wait(20)
        if not connection.done.is_set():
            connection.child.close(force=True)
            raise RuntimeError('等待底盘退出超时，请核对停止状态')
        if not connection.latest.get('disabled_confirmed'):
            raise RuntimeError('底盘未返回已确认下使能的结束状态，请检查终端日志')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'start', 'stop', 'status', 'plan', 'goal', 'cancel', 'drive'])
    parser.add_argument('--host', default='192.168.58.3')
    parser.add_argument('--user', default='fairino')
    for name in ('x', 'y'):
        parser.add_argument('--'+name, type=float)
    parser.add_argument('--yaw', type=float, help='map 绝对终点朝向（弧度）；到达位置后转向，省略时保持启动朝向')
    args = parser.parse_args()
    pose = None
    if args.action in ('goal', 'plan'):
        if not all(v is not None and math.isfinite(v) for v in (args.x, args.y)):
            parser.error('需要有限的 --x --y，单位米')
        try:
            pose = request_pose({name: getattr(args, name) for name in ('x', 'y', 'yaw')
                                 if getattr(args, name) is not None})
        except ValueError as exc:
            parser.error(str(exc))
        if args.yaw is not None:
            if not math.isfinite(args.yaw):
                parser.error('目标朝向必须为有限数值')
            if pose['heading_mode'] == 'hold_initial':
                pose['arrival_heading'] = 1
    password = os.environ.pop('FAIRINO_SSH_PASSWORD', None) or getpass.getpass(f'SSH 密码（{args.user}@{args.host}）：')
    try:
        if args.action == 'drive':
            drive(args, password)
            return 0
        result = execute(args.action, args.host, args.user, password, pose)
        if 'points' in result:
            folder = LOG_ROOT/'navigation'
            folder.mkdir(parents=True, exist_ok=True)
            path = folder/('plan-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.json')
            path.write_text(json.dumps(result, ensure_ascii=False, indent=2))
            result = {**{key: value for key, value in result.items() if key != 'points'},
                      'point_count': len(result['points']), 'saved': str(path)}
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except Exception as exc:
        print('操作未完成：'+str(exc))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
