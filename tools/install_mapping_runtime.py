#!/usr/bin/env python3
"""Build, isolate-test and select the mapping-only SLAM repair. No motor commands."""
import argparse
import base64
import getpass
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import tempfile
import urllib.request
import zlib

from ssh_read import run

ROOT = Path(__file__).resolve().parents[1]
SUPPORT = ROOT / 'apps/map_viewer/slam_runtime'
ENV = 'source /opt/ros/humble/setup.bash\nsource /opt/nav2_ws/install/setup.bash\n'


def source_bundle():
    hashes = json.loads((SUPPORT / 'sources.json').read_text())
    cache = ROOT / '.local/mapping-runtime/upstream-2.6.10'
    for name, expected in hashes.items():
        path = cache / name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            url = 'https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/2.6.10/' + name
            with urllib.request.urlopen(url, timeout=30) as response:
                data = response.read()
            if hashlib.sha256(data).hexdigest() != expected:
                raise RuntimeError('官方源码哈希与锁定版本不符：' + name)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
    with tempfile.TemporaryDirectory(prefix='fairino-slam-source-') as temp:
        directory = Path(temp)
        for name in hashes:
            path = directory / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes((cache / name).read_bytes())
        subprocess.run(['patch', '--batch', '--fuzz=0', '-p1', '-i', str(SUPPORT / 'scan-admission.patch')],
                       cwd=directory, check=True, capture_output=True, text=True)
        (directory / 'CMakeLists.txt').write_bytes((SUPPORT / 'CMakeLists.txt').read_bytes())
        files = {str(p.relative_to(directory)): base64.b64encode(p.read_bytes()).decode()
                 for p in directory.rglob('*') if p.is_file()}
    revision = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()[:16]
    return {'files': files, 'revision': revision}


def stage(host, user, password):
    bundle = source_bundle()
    code = (ROOT / 'tools/build_mapping_runtime_remote.py').read_text()
    code += '\nprint("MAPPING_RUNTIME " + json.dumps(build(' + repr(bundle) + ')), flush=True)\n'
    output = remote(code, host, user, password, timeout=900)
    return json.loads(next(line[len('MAPPING_RUNTIME '):] for line in output.splitlines()
                           if line.startswith('MAPPING_RUNTIME ')))


def remote(code, host, user, password, timeout=90, isolated=False):
    payload = base64.b64encode(zlib.compress(code.encode(), 9)).decode()
    launcher = 'import base64,zlib;exec(zlib.decompress(base64.b64decode(' + repr(payload) + ')))'
    environment = ENV
    if isolated:
        environment += ('export ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n'
                        'unset FASTRTPS_DEFAULT_PROFILES_FILE\n')
    return run(host, environment + 'python3 -u -c ' + shlex.quote(launcher),
               user=user, password=password, timeout=timeout)


def verify(staged, host, user, password):
    profile = (ROOT / 'apps/map_viewer/mapping.yaml').read_text()
    manager = (ROOT / 'apps/map_viewer/map_control_remote.py').read_text()
    reader = (ROOT / 'apps/map_viewer/remote_stream.py').read_text()
    code = (ROOT / 'tools/verify_mapping_runtime_remote.py').read_text()
    code += '\nprint("MAPPING_VERIFIED " + json.dumps(verify(' + repr(staged['runtime']) + ',' + repr(profile) + ',' + repr(manager) + ',' + repr(reader) + ')), flush=True)\n'
    output = remote(code, host, user, password, timeout=100, isolated=True)
    return json.loads(next(line[len('MAPPING_VERIFIED '):] for line in output.splitlines()
                           if line.startswith('MAPPING_VERIFIED ')))


def activate(staged, host, user, password):
    verification = verify(staged, host, user, password)
    files = {name: (ROOT / 'apps/map_viewer' / name).read_text()
             for name in ('mapping.yaml', 'map_control_remote.py')}
    code = (ROOT / 'tools/select_mapping_runtime_remote.py').read_text()
    code += '\nprint("MAPPING_SELECTED " + json.dumps(select(' + repr(staged) + ',' + repr(files) + ',' + repr(verification) + ')), flush=True)\n'
    output = remote(code, host, user, password)
    return json.loads(next(line[len('MAPPING_SELECTED '):] for line in output.splitlines()
                           if line.startswith('MAPPING_SELECTED ')))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.58.3')
    parser.add_argument('--user', default='fairino')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--stage-only', action='store_true', help='只编译独立版本，不选择它')
    args = parser.parse_args()
    if not args.execute:
        print('会编译独立 SLAM 2.6.10 修复、在 Domain 197 测试，通过后仅更新下次建图入口；加 --execute 执行。')
        return
    password = getpass.getpass(f'SSH 密码（{args.user}@{args.host}）：')
    result = stage(args.host, args.user, password)
    if not args.stage_only:
        result = activate(result, args.host, args.user, password)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
