#!/usr/bin/env python3
"""Stage/install the opt-in 2D Nav2 runtime; never enable motors or start services."""
import argparse
import base64
import getpass
import json
from pathlib import Path
import shlex
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from build_navigation import build
from apps.navigation.footprint import ChassisFootprint
from ssh_read import run
from map_mode import ENV


def remote_script(execute=False):
    files = {name: ROOT/'apps/navigation'/name for name in (
        'footprint.py', 'motion.py', 'lifecycle.py', 'velocity_gate.py', 'navigation_remote.py',
        'navigation.launch.py', 'navigate_to_pose.xml', 'navigate_through_poses.xml')}
    files['control_core.py'] = ROOT/'apps/fairino_keyboard/control_core.py'
    files['remote_stream.py'] = ROOT/'apps/map_viewer/remote_stream.py'
    payload = {name: base64.b64encode(path.read_bytes()).decode() for name, path in files.items()}
    payload['nav2.yaml'] = base64.b64encode(yaml.safe_dump(build(), sort_keys=False).encode()).decode()
    points = ChassisFootprint(ROOT/'models/amr_8dof.urdf').points
    program = r'''
import ast, base64, datetime, json, pathlib, shutil, subprocess, sys, types, yaml
home = pathlib.Path.home()
runtime = home/'.local/share/fairino-navigation'
unit = pathlib.Path('/etc/systemd/system/fairino-navigation.service')
source = pathlib.Path('/opt/fairino_AMR_project_B/AMR_SAFT/install/amr_8dof_description/share/amr_8dof_description/urdf/amr_8dof.urdf')
if not (home/'.local/share/fairino-map-control/localization.py').is_file():
    raise RuntimeError('请先使用 tools/install_map_mode.py 部署定位层')
module = types.ModuleType('verify_footprint')
sys.modules[module.__name__] = module
exec(base64.b64decode(PAYLOAD['footprint.py']), module.__dict__)
actual = module.ChassisFootprint(source).points
if actual != EXPECTED:
    raise RuntimeError('机器人当前底盘碰撞轮廓与仓库模型不一致，请先更新模型再生成配置')
for package in ('nav2_planner', 'nav2_smac_planner', 'nav2_controller', 'dwb_core', 'nav2_bt_navigator', 'nav2_lifecycle_manager'):
    subprocess.run(['ros2', 'pkg', 'prefix', package], check=True, capture_output=True)
files = {runtime/name: base64.b64decode(value).decode() for name, value in PAYLOAD.items()}
files[runtime/'runtime.sh'] = """#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/nav2_ws/install/setup.bash
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fairino_AMR_project/config/fastrtps_udp.xml
exec nice -n 10 ros2 launch /home/fairino/.local/share/fairino-navigation/navigation.launch.py
"""
files[unit] = """# Managed by fairino-robot-toolkit: explicit 2D navigation
[Unit]
Description=Fairino opt-in planar Nav2 stack
After=network-online.target amr_startup.service
[Service]
Type=simple
User=fairino
Group=fairino
ExecStart=/bin/bash /home/fairino/.local/share/fairino-navigation/runtime.sh
KillSignal=SIGINT
KillMode=control-group
TimeoutStopSec=20
Restart=no
"""
for path, content in files.items():
    if path.suffix == '.py':
        ast.parse(content)
    elif path.suffix == '.yaml':
        yaml.safe_load(content)
if unit.exists() and 'Managed by fairino-robot-toolkit' not in unit.read_text():
    raise RuntimeError('目标服务不由本工具管理，未修改')
report = {'execute': EXECUTE, 'files': [str(p) for p in files], 'footprint': actual,
          'service_started': False, 'motor_enabled': False, 'map_changed': False}
if EXECUTE:
    if subprocess.run(['systemctl', 'is-active', '--quiet', unit.name]).returncode == 0:
        raise RuntimeError('请先停止导航服务再安装更新')
    backup = runtime/'backups'/datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    backup.mkdir(parents=True)
    entries = []
    for index, (target, content) in enumerate(files.items()):
        staged = backup/(str(index)+'.new')
        staged.write_text(content)
        if target.suffix == '.sh':
            subprocess.run(['bash', '-n', str(staged)], check=True)
        original = backup/(str(index)+'.original')
        if target.exists():
            shutil.copy2(target, original)
        entries.append({'target': str(target), 'original': str(original) if original.exists() else None,
                        'staged': str(staged), 'owner': 'root' if target == unit else 'fairino'})
    (backup/'files.json').write_text(json.dumps(entries, indent=2))
    installed = []
    try:
        for entry in entries:
            subprocess.run(['sudo', '-n', 'install', '-D', '-m', '0644', '-o', entry['owner'],
                            '-g', entry['owner'], entry['staged'], entry['target']], check=True)
            installed.append(entry)
        subprocess.run(['sudo', '-n', 'systemctl', 'daemon-reload'], check=True)
    except Exception:
        for entry in reversed(installed):
            if entry['original']:
                subprocess.run(['sudo', '-n', 'install', '-m', '0644', '-o', entry['owner'],
                                '-g', entry['owner'], entry['original'], entry['target']], check=True)
            else:
                subprocess.run(['sudo', '-n', 'rm', '--', entry['target']], check=True)
        subprocess.run(['sudo', '-n', 'systemctl', 'daemon-reload'], check=True)
        raise
    report['backup'] = str(backup)
    (backup/'manifest.json').write_text(json.dumps(report, indent=2))
print('NAVIGATION_INSTALL '+json.dumps(report))
'''
    return 'PAYLOAD = '+repr(payload)+'\nEXPECTED = '+repr(points)+'\nEXECUTE = '+repr(execute)+'\n'+program


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--host', default='192.168.58.3')
    parser.add_argument('--user', default='fairino')
    args = parser.parse_args()
    password = getpass.getpass(f'SSH 密码（{args.user}@{args.host}）：')
    output = run(args.host, ENV+'python3 -u -c '+shlex.quote(remote_script(args.execute)),
                 user=args.user, password=password, timeout=60)
    result = json.loads(next(line[len('NAVIGATION_INSTALL '):] for line in output.splitlines()
                             if line.startswith('NAVIGATION_INSTALL ')))
    folder = ROOT/'logs/navigation'
    folder.mkdir(parents=True, exist_ok=True)
    (folder/'installation.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
