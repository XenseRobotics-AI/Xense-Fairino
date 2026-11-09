#!/usr/bin/env python3
"""Explicit SSH commands for viewing, starting, pausing and saving robot maps."""
import argparse
import getpass
import json
import os
from pathlib import Path
import shlex
from fairino_slam.connection.ssh import run

REMOTE = '/home/fairino/.local/share/fairino-map-control/map_control_remote.py'
ENV = '''source /opt/ros/humble/setup.bash
source /opt/nav2_ws/install/setup.bash
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fairino_AMR_project/config/fastrtps_udp.xml
'''


def execute(action, host='192.168.58.3', user='fairino', password=None, name=None, wait=0, pose=None):
    argv = ['python3', REMOTE, action, '--wait', str(wait)]
    if name:
        argv += ['--name', name]
    if pose is not None:
        for key in ('x', 'y', 'yaw', 'xy_std', 'yaw_std', 'session_id'):
            if key in pose:
                argv += ['--' + key.replace('_', '-'), str(pose[key])]
    try:
        text = run(host, ENV + shlex.join(argv), user=user, password=password, timeout=max(100, wait + 45))
    except RuntimeError as exc:
        text = str(exc)
    rows = [line[len('MAP_CONTROL '):] for line in text.splitlines() if line.startswith('MAP_CONTROL ')]
    if not rows:
        raise RuntimeError(text[-1500:])
    result = json.loads(rows[-1])
    if result.get('error'):
        raise RuntimeError(result['error'])
    return result


def main():
    parser = argparse.ArgumentParser(description='地图模式管理；不使能或移动机器人。')
    parser.add_argument('action', choices=['status', 'check', 'start', 'pause', 'resume', 'view', 'save', 'use', 'clear', 'maps', 'localize', 'stop-localization', 'initial-pose'])
    parser.add_argument('--host', default='192.168.58.3')
    parser.add_argument('--user', default='fairino')
    parser.add_argument('--name', help='保存到新目录，禁止覆盖已有地图')
    parser.add_argument('--wait', type=float, default=0, help='等待有效建图输入的秒数；可先运行此命令再准备里程计输入')
    parser.add_argument('--x', type=float)
    parser.add_argument('--y', type=float)
    parser.add_argument('--yaw', type=float, help='map 坐标系中的弧度，逆时针为正')
    args = parser.parse_args()
    if args.action == 'initial-pose' and any(v is None for v in (args.x, args.y, args.yaw)):
        parser.error('initial-pose requires --x --y --yaw')
    if not 0 <= args.wait <= 300:
        parser.error('--wait 必须在 0 到 300 秒之间')
    password = os.environ.pop('FAIRINO_SSH_PASSWORD', None)
    if password is None:
        password = getpass.getpass(f'SSH 密码（{args.user}@{args.host}）: ')
    try:
        if args.action in ('start', 'resume', 'localize') and args.wait:
            print(f'等待有效的里程计、TF 和雷达输入，最多 {args.wait:g} 秒；请确保里程计与雷达数据源就绪。', flush=True)
        pose = {'x': args.x, 'y': args.y, 'yaw': args.yaw} if args.action == 'initial-pose' else None
        result = execute(args.action, args.host, args.user, password, args.name, args.wait, pose=pose)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except Exception as exc:
        print('操作未完成：' + str(exc))
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
