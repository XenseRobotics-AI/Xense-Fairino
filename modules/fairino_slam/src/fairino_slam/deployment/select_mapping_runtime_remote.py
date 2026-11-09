"""Select a tested mapping runtime for future sessions only. No service restarts."""
import ast
import datetime
import fcntl
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import yaml


def select(staged, files, verification):
    if not verification.get('passed') or verification.get('domain') != 197 or not verification.get('loopback_only'):
        raise RuntimeError('隔离建图测试未通过，未更新入口')
    root = Path(staged['runtime'])
    manifest = json.loads((root/'manifest.json').read_text())
    if manifest.get('scan_gate') != 'backend_motion_gate_v1':
        raise RuntimeError('扫描筛选修复版本未确认')
    for name, expected in manifest['artifacts'].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError('编译产物已变化，未更新入口：' + name)
    runtime = Path.home()/'.local/share/fairino-map-control'
    runtime.mkdir(parents=True, exist_ok=True)
    with (runtime/'install.lock').open('a') as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        state = subprocess.run(['systemctl', 'is-active', 'fairino-manual-mapping.service'],
                               capture_output=True, text=True, timeout=8).stdout.strip()
        if state in ('active', 'activating', 'deactivating'):
            raise RuntimeError('建图服务正在运行，请结束当前建图后再部署；未停止现场会话')
        for process in Path('/proc').iterdir():
            if not process.name.isdigit():
                continue
            try:
                args = process.joinpath('cmdline').read_bytes().split(b'\0')
            except OSError:
                continue
            if args and Path(args[0].decode(errors='replace')).name in ('sync_slam_toolbox_node', 'async_slam_toolbox_node'):
                raise RuntimeError('仍有 SLAM 进程，未切换入口')
        backup = runtime/'backups'/datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f-mapping-runtime')
        backup.mkdir(parents=True)
        current = runtime/'slam/current'
        if current.exists() and not current.is_symlink():
            raise RuntimeError('current 不是受管理的符号链接，未覆盖')
        previous = str(current.readlink()) if current.is_symlink() else None
        (backup/'previous-runtime.json').write_text(json.dumps({'target': previous}))
        originals = {}
        for name, content in files.items():
            if name not in ('mapping.yaml', 'map_control_remote.py'):
                raise ValueError('Unexpected managed file')
            ast.parse(content) if name.endswith('.py') else yaml.safe_load(content)
            target = runtime/name
            originals[name] = target.read_bytes() if target.exists() else None
            if target.exists():
                shutil.copy2(target, backup/name)
            (backup/(name+'.new')).write_text(content)
        changed = []
        try:
            for name in files:
                target = runtime/name
                temporary = target.with_suffix(target.suffix+'.new')
                temporary.write_text(files[name])
                temporary.replace(target)
                changed.append(name)
            temporary = current.with_name('current.new')
            temporary.unlink(missing_ok=True)
            temporary.symlink_to(root, target_is_directory=True)
            temporary.replace(current)
        except Exception:
            for name in changed:
                if originals[name] is None:
                    (runtime/name).unlink(missing_ok=True)
                else:
                    (runtime/name).write_bytes(originals[name])
            raise
        (backup/'verification.json').write_text(json.dumps(verification, indent=2))
        result = dict(staged, activated=True, backup=str(backup), previous_runtime=previous,
                      verification=verification, effective='next explicit mapping start',
                      changed_files=[str(runtime/name) for name in files]+[str(current)])
        (backup/'manifest.json').write_text(json.dumps(result, indent=2))
        return result
