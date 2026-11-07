#!/usr/bin/env python3
"""Back up and install opt-in mapping. Does not restart the chassis service."""
import argparse
import base64
import getpass
import json
import os
from pathlib import Path
import shlex
from fairino_slam.connection.ssh import run
from fairino_slam.configuration import load_ros_profile

from fairino_slam.paths import PACKAGE_ROOT, PROJECT_ROOT as ROOT, RESOURCE_ROOT, CONFIG_ROOT, LOG_ROOT


def remote_script():
    load_ros_profile('mapping.yaml')
    load_ros_profile('localization.yaml')
    payload = base64.b64encode((PACKAGE_ROOT/'mapping/map_control_remote.py').read_bytes()).decode()
    profile = base64.b64encode((CONFIG_ROOT/'mapping.yaml').read_bytes()).decode()
    localization = {name: base64.b64encode(path.read_bytes()).decode() for name, path in {
        'localization.py': PACKAGE_ROOT/'mapping/localization.py',
        'localization.yaml': CONFIG_ROOT/'localization.yaml'}.items()}
    script = r"""
import base64, datetime, hashlib, json, os, pathlib, shutil, subprocess
home = pathlib.Path.home()
runtime = home / '.local/share/fairino-map-control'
runtime.mkdir(parents=True, exist_ok=True)
backup = runtime / 'backups' / datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
backup.mkdir(parents=True)
project = pathlib.Path('/opt/fairino_AMR_project').resolve()
startup = project / 'start.sh'
original = startup.read_text()
expected = '${AMR_NAV2_SLAM:-true}'
if original.count(expected) not in (0, 2):
    raise RuntimeError('启动脚本结构与预期不符，未修改')
if expected not in original and '${AMR_NAV2_SLAM:-false}' not in original:
    raise RuntimeError('找不到默认建图开关，未修改')
shutil.copy2(startup, backup / 'start.sh')
for name in ('map.pgm', 'map.yaml'):
    if (project / 'maps/active' / name).is_file():
        shutil.copy2(project / 'maps/active' / name, backup / name)
files = {
    str(runtime / 'map_control_remote.py'): base64.b64decode(PAYLOAD).decode(),
    str(runtime / 'mapping.yaml'): base64.b64decode(PROFILE).decode(),
    str(runtime / 'mapping_runtime.sh'): '''#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/nav2_ws/install/setup.bash
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fairino_AMR_project/config/fastrtps_udp.xml
exec python3 /home/fairino/.local/share/fairino-map-control/map_control_remote.py serve
''',
    '/etc/systemd/system/amr_startup.service.d/40-manual-mapping.conf': '''# Managed by fairino-robot-toolkit: opt-in mapping
[Service]
Environment="AMR_NAV2_SLAM=false"
''',
    '/etc/systemd/system/fairino-manual-mapping.service': '''# Managed by fairino-robot-toolkit: opt-in mapping
[Unit]
Description=Fairino explicit mapping session with input watchdog
After=network-online.target amr_startup.service
[Service]
Type=simple
User=fairino
Group=fairino
ExecStart=/bin/bash /home/fairino/.local/share/fairino-map-control/mapping_runtime.sh
KillSignal=SIGINT
KillMode=control-group
TimeoutStopSec=20
Restart=no
'''
}
files.update({str(runtime/name): base64.b64decode(value).decode() for name, value in LOCALIZATION.items()})
files[str(runtime/'localization_runtime.sh')] = files[str(runtime/'mapping_runtime.sh')].replace(' serve\n', ' watch-localization\n')
files['/etc/systemd/system/fairino-localization.service'] = '''# Managed by fairino-robot-toolkit: explicit localization watchdog
[Unit]
Description=Fairino AMCL input and odometry-session watchdog
After=network-online.target amr_startup.service
[Service]
Type=simple
User=fairino
Group=fairino
ExecStart=/bin/bash /home/fairino/.local/share/fairino-map-control/localization_runtime.sh
KillSignal=SIGINT
TimeoutStopSec=10
Restart=no
'''
for name, content in files.items():
    target = pathlib.Path(name)
    if target.exists():
        if name.startswith('/etc/') and 'Managed by fairino-robot-toolkit' not in target.read_text():
            raise RuntimeError('目标系统配置已存在且不由本工具管理：' + name)
        shutil.copy2(target, backup / target.name)
staged = backup / 'start.sh.new'
updated = original.replace(expected, '${AMR_NAV2_SLAM:-false}')
updated = updated.replace('[[ -f "$NAV2_MAP" ]] || fail "Nav2 map missing: $NAV2_MAP"',
    '[[ -f "$NAV2_MAP" ]] || log "No saved map: default map is empty; mapping requires an explicit start"')
staged.write_text(updated)
subprocess.run(['bash', '-n', str(staged)], check=True)
for index, (name, content) in enumerate(files.items()):
    tmp = backup / ('new-' + str(index))
    tmp.write_text(content)
    owner = 'root' if name.startswith('/etc/') else 'fairino'
    subprocess.run(['sudo', '-n', 'install', '-D', '-m', '0644', '-o', owner, '-g', owner, str(tmp), name], check=True)
stat = startup.stat()
subprocess.run(['sudo','-n','install','-m',oct(stat.st_mode & 0o777)[2:],'-o',str(stat.st_uid),'-g',str(stat.st_gid),str(staged),str(startup)], check=True)
subprocess.run(['python3','-m','py_compile',str(runtime/'map_control_remote.py')],check=True)
subprocess.run(['bash','-n',str(runtime/'mapping_runtime.sh')],check=True)
subprocess.run(['sudo','-n','systemctl','daemon-reload'],check=True)
report = {'backup': str(backup), 'project': str(project), 'changed_files': [str(startup)] + list(files),
          'default_slam': False, 'chassis_service_restarted': False,
          'active_map_sha256': hashlib.sha256((project/'maps/active/map.pgm').read_bytes()).hexdigest() if (project/'maps/active/map.pgm').is_file() else None}
(backup/'manifest.json').write_text(json.dumps(report,indent=2))
print('MAP_INSTALL ' + json.dumps(report))
"""
    # PAYLOAD is a Python string constant, not a shell interpolation.
    script = 'LOCALIZATION = ' + repr(localization) + '\n' + 'PAYLOAD = ' + repr(payload) + '\nPROFILE = ' + repr(profile) + '\n' + script
    return script


def main(password=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.58.3')
    parser.add_argument('--user', default='fairino')
    parser.add_argument('--execute', action='store_true', help='备份并安装地图/定位服务；不启动服务')
    args = parser.parse_args()
    if not args.execute:
        print('将安装地图与定位服务；加 --execute 才连接机器人并写入配置。')
        return 0
    script = remote_script()
    if password is None:
        password = os.environ.pop('FAIRINO_SSH_PASSWORD', None) or getpass.getpass(f'SSH 密码（{args.user}@{args.host}）: ')
    output = run(args.host, 'python3 -u -c ' + shlex.quote(script), user=args.user, password=password, timeout=45)
    result = json.loads(next(s[len('MAP_INSTALL '):] for s in output.splitlines() if s.startswith('MAP_INSTALL ')))
    folder = LOG_ROOT/'deployment/mapping'
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'installation.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
