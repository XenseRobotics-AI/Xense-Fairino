#!/usr/bin/env python3
"""Install shared chassis + guarded raw/EKF odometry. Never restart hardware."""
import argparse
import base64
import getpass
import json
from pathlib import Path
import shlex

from fairino_slam.connection.ssh import run
from fairino_slam.configuration import load_ros_profile
from fairino_slam.chassis.source import render_control_core

from fairino_slam.paths import PACKAGE_ROOT, PROJECT_ROOT as ROOT, RESOURCE_ROOT, CONFIG_ROOT, LOG_ROOT


def remote_script(execute=False, mode='raw'):
    if mode not in ('raw', 'ekf'):
        raise ValueError('Unknown odometry mode')
    load_ros_profile('mapping.yaml')
    load_ros_profile('localization.yaml')
    files = {'odom_guard.py': PACKAGE_ROOT/'odometry/odom_guard.py',
             'raw_odometry.py': PACKAGE_ROOT/'odometry/raw_odometry.py',
             'map_control_remote.py': PACKAGE_ROOT/'mapping/map_control_remote.py',
             'mapping.yaml': CONFIG_ROOT/'mapping.yaml',
             'localization.py': PACKAGE_ROOT/'mapping/localization.py',
             'localization.yaml': CONFIG_ROOT/'localization.yaml'}
    payload = {name: base64.b64encode(path.read_bytes()).decode() for name, path in files.items()}
    core = render_control_core()
    script = r'''
import ast, datetime, json, os, pathlib, shutil, subprocess, yaml
namespace = {}
exec(CORE, namespace)
project = pathlib.Path('/opt/fairino_AMR_project').resolve()
if project.name != 'fairino_AMR_project_B':
    raise RuntimeError('当前项目不是已核对的 B 配置，未修改')
runtime = pathlib.Path.home()/'.local/share/fairino-odometry'
config = project/'reference_chassis_runtime/install/fairino_hardware/share/fairino_hardware/config/ros2_controllers.yaml'
commands = project/'reference_chassis_runtime/install/fairino_hardware_command_controller/share/fairino_hardware_command_controller/config/command_controller_params.yaml'
ekf = project/'config/localization/ekf.yaml'
startup = project/'start.sh'
changes = {}
data = yaml.safe_load(config.read_text())
data['eight_dof_chassis_controller']['ros__parameters'] = namespace['chassis_parameters'](data['eight_dof_chassis_controller']['ros__parameters'])
changes[config] = yaml.safe_dump(data, sort_keys=False)
data = yaml.safe_load(commands.read_text())
auto = data['fairino_hardware_command_controller']['ros__parameters']['auto_start']
if not isinstance(auto['enable_auto_execute'], bool):
    raise RuntimeError('未知的开机自动执行字段')
auto['enable_auto_execute'] = False
changes[commands] = yaml.safe_dump(data, sort_keys=False)
data = yaml.safe_load(ekf.read_text())
params = data['ekf_filter_node']['ros__parameters']
params['odom0'] = '/eight_dof_chassis_controller/odom'
changes[ekf] = yaml.safe_dump(data, sort_keys=False)
original = startup.read_text()
old = 'setsid ros2 run robot_localization ekf_node '
new = 'setsid python3 /home/fairino/.local/share/fairino-odometry/odom_guard.py '
if original.count(old) == 1:
    updated = original.replace(old, new)
elif original.count(new) == 1:
    updated = original
else:
    raise RuntimeError('EKF 启动段与已核对版本不同，未修改')
updated = updated.replace('starting reference ROS chassis stack with automatic command execution enabled',
                          'starting reference ROS chassis stack; automatic motor enable disabled')
updated = updated.replace('starting EKF (odometry/filtered -> /odom)', 'starting guarded EKF (waits for healthy wheel odometry)')
updated = updated.replace('starting guarded EKF (waits for healthy wheel odometry)',
                          'starting guarded odometry (mode selected in mode.json)')
changes[startup] = updated
changes[runtime/'mode.json'] = json.dumps({'mode': MODE}, indent=2)+'\n'
for name, encoded in PAYLOAD.items():
    content = base64.b64decode(encoded).decode()
    if name.endswith('.py'):
        ast.parse(content)
    else:
        yaml.safe_load(content)
    target = runtime/name if name in ('odom_guard.py', 'raw_odometry.py') else pathlib.Path.home()/'.local/share/fairino-map-control'/name
    changes[target] = content
report = {'execute': EXECUTE, 'changed_files': [str(p) for p in changes],
          'auto_motor_enable': False, 'controller': 'eight_dof_chassis_controller',
          'raw_odom': '/eight_dof_chassis_controller/odom',
          'odometry_mode': MODE,
          'hardware_restarted': False, 'effective_after_robot_restart': True}
if EXECUTE:
    backup = runtime/'backups'/datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    backup.mkdir(parents=True)
    entries = []
    # Stage and validate all files before touching configuration.
    for i, (target, content) in enumerate(changes.items()):
        staged = backup/(str(i)+'.new')
        staged.write_text(content)
        if target == startup:
            subprocess.run(['bash', '-n', str(staged)], check=True)
        saved = backup/(str(i)+'.original')
        if target.exists():
            shutil.copy2(target, saved)
        stat = target.stat() if target.exists() else None
        entries.append({'target':str(target), 'original':str(saved) if saved.exists() else None,
                        'uid':stat.st_uid if stat else os.getuid(), 'gid':stat.st_gid if stat else os.getgid(),
                        'mode':oct(stat.st_mode & 0o777)[2:] if stat else '644', 'staged':str(staged)})
    (backup/'files.json').write_text(json.dumps(entries, indent=2))
    installed = []
    try:
        for entry in entries:
            subprocess.run(['sudo','-n','install','-D','-m',entry['mode'],'-o',str(entry['uid']),'-g',str(entry['gid']),entry['staged'],entry['target']],check=True)
            installed.append(entry)
    except Exception:
        for entry in reversed(installed):
            if entry['original']:
                subprocess.run(['sudo','-n','install','-m',entry['mode'],'-o',str(entry['uid']),'-g',str(entry['gid']),entry['original'],entry['target']],check=True)
            else:
                pathlib.Path(entry['target']).unlink(missing_ok=True)
        raise
    report['backup'] = str(backup)
    (backup/'manifest.json').write_text(json.dumps(report, indent=2))
print('CONTROL_STACK_INSTALL ' + json.dumps(report))
'''
    return 'import base64\nCORE = '+repr(core)+'\nPAYLOAD = '+repr(payload)+'\nEXECUTE = '+repr(execute)+'\nMODE = '+repr(mode)+'\n'+script


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.58.3')
    parser.add_argument('--user', default='fairino')
    parser.add_argument('--execute', action='store_true', help='备份并写入配置；不重启、不使能，重启机器人后生效')
    parser.add_argument('--odometry', choices=['raw', 'ekf'], default='raw', help='默认使用原始轮速；ekf 保留用于后续融合调试')
    parser.add_argument('--apply-now', action='store_true', help='安装后只重启里程计守护；要求控制空闲、静止、全部下使能且未建图')
    args = parser.parse_args()
    if args.apply_now and not args.execute:
        parser.error('--apply-now 需要同时指定 --execute')
    password = getpass.getpass(f'SSH 密码（{args.user}@{args.host}）：')
    result = run(args.host, 'python3 -u -c '+shlex.quote(remote_script(args.execute, args.odometry)),
                 user=args.user, password=password, timeout=45)
    line = next(line for line in result.splitlines() if line.startswith('CONTROL_STACK_INSTALL '))
    print(json.dumps(json.loads(line[len('CONTROL_STACK_INSTALL '):]), ensure_ascii=False, indent=2))
    if args.apply_now:
        script = (PACKAGE_ROOT/'deployment/restart_odometry_remote.py').read_text()
        environment = '''source /opt/ros/humble/setup.bash
source /opt/fairino_AMR_project_B/reference_chassis_runtime/install/hardware_state_broadcaster_msgs/share/hardware_state_broadcaster_msgs/local_setup.bash
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE
'''
        result = run(args.host, environment+'python3 -u -c '+shlex.quote(script)+' --execute',
                     user=args.user, password=password, timeout=35)
        line = next(line for line in result.splitlines() if line.startswith('ODOMETRY_RESTART '))
        print(json.dumps(json.loads(line[len('ODOMETRY_RESTART '):]), ensure_ascii=False, indent=2))
        print('里程计模式已立即生效；未重启驱动。其他启动配置仍需下次机器人重启后生效。')
    elif args.execute:
        print('未重启运行中的驱动。写入配置在机器人静止、下使能并重启后生效。')


if __name__ == '__main__':
    main()
