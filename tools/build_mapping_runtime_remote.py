"""Build a private mapping executable. Never changes the selected runtime."""
import base64
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def build(bundle):
    version = ET.parse('/opt/ros/humble/share/slam_toolbox/package.xml').getroot().findtext('version')
    if version != '2.6.10':
        raise RuntimeError('独立修复只适用于已验证的 SLAM Toolbox 2.6.10，当前：' + str(version))
    parent = Path.home() / '.local/share/fairino-map-control/slam'
    parent.mkdir(parents=True, exist_ok=True)
    with (parent / 'build.lock').open('a') as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        revision = bundle['revision']
        if len(revision) != 16 or any(c not in '0123456789abcdef' for c in revision):
            raise ValueError('Invalid build revision')
        root = parent / ('2.6.10-' + revision)
        manifest = root / 'manifest.json'
        if manifest.is_file():
            result = json.loads(manifest.read_text())
            if all((root / name).is_file() and hashlib.sha256((root / name).read_bytes()).hexdigest() == sha
                   for name, sha in result['artifacts'].items()):
                return result
        source = root / 'source'
        source.mkdir(parents=True, exist_ok=True)
        for name, data in bundle['files'].items():
            path = Path(name)
            if path.is_absolute() or '..' in path.parts:
                raise ValueError('Invalid source path')
            target = source / path
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(base64.b64decode(data))
        environment = dict(os.environ, ROS_DOMAIN_ID='197', ROS_LOCALHOST_ONLY='1')
        environment.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
        with (root / 'build.log').open('w') as log:
            for command in [
                ['cmake', '-S', str(source), '-B', str(root / 'build'), '-DCMAKE_BUILD_TYPE=Release',
                 '-DCMAKE_INSTALL_PREFIX=' + str(root)],
                ['cmake', '--build', str(root / 'build'), '--parallel', '2'],
                ['cmake', '--install', str(root / 'build')]]:
                print('MAPPING_BUILD ' + ' '.join(command), flush=True)
                result = subprocess.run(command, env=environment, stdout=log, stderr=subprocess.STDOUT)
                if result.returncode:
                    log.flush()
                    raise RuntimeError('独立建图程序编译失败：' + str(root / 'build.log') + '\n' +
                                       (root / 'build.log').read_text()[-5000:])
        artifacts = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                     for folder in ('bin', 'lib') for p in (root / folder).glob('*') if p.is_file()}
        result = {'runtime': str(root), 'revision': revision, 'upstream_version': version,
                  'scan_gate': 'backend_motion_gate_v1', 'artifacts': artifacts,
                  'activated': False, 'hardware_restarted': False}
        manifest.write_text(json.dumps(result, indent=2))
        return result
