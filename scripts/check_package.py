#!/usr/bin/env python3
"""Build and validate source/wheel distributions in temporary directories.

This checks the local Python distribution only. It never deploys to or connects
with a robot, and leaves no build/ or dist/ directory in the working checkout.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import sys
import tarfile
import tempfile
import zipfile


IGNORED = {
    '.git', '.venv', '.local', 'logs', 'build', 'dist', '__pycache__',
    '.pytest_cache', '.ruff_cache', '.mypy_cache', '.DS_Store',
}
PACKAGE = 'fairino_slam'


def ignore_generated(_directory: str, names: list[str]) -> set[str]:
    return {name for name in names
            if name in IGNORED or name.endswith(('.egg-info', '.pyc', '.pyo'))}


def run(command: list[str], *, cwd: Path, env: dict[str, str], label: str) -> str:
    print(f'{label}…', flush=True)
    completed = subprocess.run(command, cwd=cwd, env=env, capture_output=True,
                               text=True, timeout=300)
    if completed.returncode:
        output = (completed.stdout + '\n' + completed.stderr).strip()
        raise RuntimeError(f'{label}失败（退出码 {completed.returncode}）\n{output[-14000:]}')
    return completed.stdout + completed.stderr


def unique(directory: Path, pattern: str) -> Path:
    matches = sorted(directory.glob(pattern))
    if len(matches) != 1:
        raise RuntimeError(f'预期一个 {pattern}，实际找到 {len(matches)} 个')
    return matches[0]


def source_members(archive: tarfile.TarFile) -> dict[str, tarfile.TarInfo]:
    """Strip the single distribution prefix and reject unsafe archive members."""
    result: dict[str, tarfile.TarInfo] = {}
    prefixes = set()
    for member in archive.getmembers():
        name = PurePosixPath(member.name)
        if name.is_absolute() or '..' in name.parts:
            raise RuntimeError(f'源码包包含非法路径：{member.name}')
        if member.issym() or member.islnk():
            raise RuntimeError(f'源码包不应包含链接：{member.name}')
        if not name.parts:
            continue
        prefixes.add(name.parts[0])
        if len(name.parts) > 1 and member.isfile():
            relative = PurePosixPath(*name.parts[1:]).as_posix()
            if relative in result:
                raise RuntimeError(f'源码包包含重复文件：{relative}')
            result[relative] = member
    if len(prefixes) != 1:
        raise RuntimeError('源码包必须只有一个顶层目录')
    return result


def extract_source(sdist: Path, destination: Path, source: Path) -> None:
    required = {'README.md', 'THIRD_PARTY_NOTICES.md', 'pyproject.toml', 'setup.py',
                'scripts/run.sh', 'scripts/setup.sh', '.editorconfig', 'MANIFEST.in'}
    for subtree in ('config', 'ros2', 'tests', 'docs', 'scripts', 'reports'):
        folder = source / subtree
        if not folder.is_dir():
            raise RuntimeError(f'源码工程缺少目录：{subtree}')
        required.update(path.relative_to(source).as_posix()
                        for path in folder.rglob('*') if path.is_file()
                        and not any(part in IGNORED for part in path.relative_to(source).parts)
                        and path.suffix not in ('.pyc', '.pyo'))
    with tarfile.open(sdist, 'r:gz') as archive:
        members = source_members(archive)
        missing = sorted(required.difference(members))
        if missing:
            raise RuntimeError('源码包缺少必要文件：\n' + '\n'.join(missing))
        # Extract regular files ourselves, avoiding tar link/path semantics.
        for relative, member in members.items():
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            stream = archive.extractfile(member)
            if stream is None:
                raise RuntimeError(f'无法读取源码包文件：{relative}')
            with stream:
                target.write_bytes(stream.read())
            target.chmod(member.mode & 0o777)
    print(f'源码包文件检查通过：包含 {len(required)} 个必要文件。', flush=True)


def extract_wheel(wheel: Path, destination: Path, source: Path) -> None:
    package_root = source / 'src' / PACKAGE
    required = {path.relative_to(source / 'src').as_posix()
                for path in package_root.rglob('*.py')}
    required.update(path.relative_to(source / 'src').as_posix()
                    for path in (package_root / 'resources').rglob('*') if path.is_file())
    bundled = {}
    for subtree in ('config', 'ros2'):
        files = {path.relative_to(source / subtree).as_posix(): path.read_bytes()
                 for path in (source / subtree).rglob('*') if path.is_file()}
        if not files:
            raise RuntimeError('源码工程缺少资源目录：' + subtree)
        prefix = PACKAGE + '/resources/' + subtree + '/'
        bundled[prefix] = files
        required.update(prefix + relative for relative in files)
    with zipfile.ZipFile(wheel) as archive:
        names = set(archive.namelist())
        missing = sorted(required.difference(names))
        if missing:
            raise RuntimeError('安装包缺少源码或资源：\n' + '\n'.join(missing))
        for prefix, files in bundled.items():
            packaged = {name[len(prefix):] for name in names
                        if name.startswith(prefix) and not name.endswith('/')}
            if packaged != set(files):
                raise RuntimeError('安装包资源清单与源码不一致：' + prefix)
            for relative, expected in files.items():
                if archive.read(prefix + relative) != expected:
                    raise RuntimeError('安装包资源内容与源码不一致：' + relative)
        for member in archive.infolist():
            name = PurePosixPath(member.filename)
            if name.is_absolute() or '..' in name.parts:
                raise RuntimeError(f'安装包包含非法路径：{member.filename}')
            if member.is_dir():
                continue
            target = destination.joinpath(*name.parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(archive.read(member))
    print(f'安装包资源检查通过：包含 {len(required)} 个源码和资源文件。', flush=True)


def check(project: Path) -> None:
    project = project.resolve()
    if not (project / 'pyproject.toml').is_file() or not (project / 'src' / PACKAGE).is_dir():
        raise RuntimeError(f'不是有效的 Fairino SLAM 工程：{project}')
    for subtree in ('config', 'ros2'):
        if (project / 'src' / PACKAGE / 'resources' / subtree).exists():
            raise RuntimeError('源码不能存在包内资源副本：' + subtree)
    with tempfile.TemporaryDirectory(prefix='fairino-slam-package-') as temporary:
        work = Path(temporary)
        source, artifacts = work / 'source', work / 'artifacts'
        unpacked, installed = work / 'sdist', work / 'wheel'
        shutil.copytree(project, source, ignore=ignore_generated)
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PIP_NO_INDEX='1',
                   MPLBACKEND='Agg', MPLCONFIGDIR=str(work / 'matplotlib'),
                   FAIRINO_SLAM_LOG_DIR=str(work / 'logs'))
        env.pop('PYTHONPATH', None)
        run([sys.executable, '-m', 'build', '--no-isolation', '--outdir', str(artifacts)],
            cwd=source, env=env, label='构建 sdist 和 wheel')
        sdist, wheel = unique(artifacts, '*.tar.gz'), unique(artifacts, '*.whl')
        extract_source(sdist, unpacked, source)
        # Exercise the distributed shell entrypoint from outside the checkout.
        env['FAIRINO_SLAM_PYTHON'] = sys.executable
        result = run(['bash', str(unpacked / 'scripts/run.sh'), 'test'],
                     cwd=work, env=env, label='通过源码包入口运行离线回归')
        # unittest writes its summary to stderr, already captured above.
        for line in result.splitlines():
            if line.startswith('Ran ') or line == 'OK' or line.startswith('OK ('):
                print(line, flush=True)
        extract_wheel(wheel, installed, source)
        # -I excludes the checkout and PYTHONPATH. Insert only the extracted wheel
        # before installed dependencies, and assert the actual import location.
        code = '''
import pathlib, sys
root = pathlib.Path(sys.argv[1]).resolve()
sys.path.insert(0, str(root))
import fairino_slam
actual = pathlib.Path(fairino_slam.__file__).resolve()
if not actual.is_relative_to(root):
    raise RuntimeError('Imported checkout instead of wheel: ' + str(actual))
from fairino_slam.paths import CONFIG_ROOT, ROS_ROOT
if CONFIG_ROOT != root / 'fairino_slam/resources/config':
    raise RuntimeError('Wheel did not select its bundled configuration: ' + str(CONFIG_ROOT))
if ROS_ROOT != root / 'fairino_slam/resources/ros2':
    raise RuntimeError('Wheel did not select its bundled controller source')
from fairino_slam.diagnostics.offline import validate
validate()
print('安装包隔离导入和离线配置检查通过。')
'''
        env.pop('PYTHONPATH', None)
        output = run([sys.executable, '-I', '-c', code, str(installed)],
                     cwd=work, env=env, label='验证安装包独立导入')
        print(output.strip(), flush=True)
    print('打包检查全部通过，临时构建文件已清理。', flush=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path,
                        default=Path(__file__).resolve().parents[1],
                        help='工程根目录；默认使用 scripts/ 的上级目录')
    args = parser.parse_args(argv)
    try:
        check(args.project_root)
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
