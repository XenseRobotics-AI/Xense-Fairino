#!/usr/bin/env python3
"""Build the offline fairino_ik extension with the PICO interpreter and isolated wheels."""
import argparse
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import sysconfig

ROOT = Path(__file__).resolve().parents[2]
CACHE = (ROOT / '.local/fairino-ik').resolve()  # .local may be a symlink shared by worktrees
VENDOR = ROOT / 'third_party/fairino_ik'
DEPENDENCIES = CACHE / 'site-packages'
PREFIX = DEPENDENCIES / 'cmeel.prefix'
WHEELS = ('pin==4.1.0', 'eiquadprog==1.3.2', 'pybind11==3.0.1', 'cmeel-boost==1.90.0',
          'cmeel-urdfdom[build]==6.0.0', 'cmeel-tinyxml2==11.0.0', 'cmeel-eigen==3.4.1')


def build_dir():
    digest = hashlib.sha256(sysconfig.get_config_var('SOABI').encode())
    for path in sorted([VENDOR / 'CMakeLists.txt', Path(__file__), *VENDOR.glob('cpp/**/*')]):
        if path.is_file():
            digest.update(path.name.encode())
            digest.update(path.read_bytes())
    return CACHE / 'builds' / digest.hexdigest()[:20]


def extension_path():
    return build_dir() / 'fairino_ik' / ('_native' + sysconfig.get_config_var('EXT_SUFFIX'))


def check_linkage(path):
    listing = subprocess.check_output(['ldd', str(path)], text=True)
    system = {'libc.so.6', 'libstdc++.so.6', 'libm.so.6', 'libgcc_s.so.1',
              'librt.so.1', 'libpthread.so.0', 'libdl.so.2', 'libgomp.so.1'}
    for line in listing.splitlines():
        if '=>' in line:
            name, resolved = line.split('=>', 1)
            location = resolved.strip().split(' (')[0]
            if (location == 'not found'
                    or name.strip() not in system and not Path(location).resolve().is_relative_to(PREFIX)):
                raise RuntimeError(f'Unexpected fairino_ik dependency: {line.strip()}')
    return listing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install', action='store_true', help='download the isolated build dependencies')
    args = parser.parse_args()
    temp = CACHE / 'tmp'
    temp.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, TMPDIR=str(temp), PYTHONDONTWRITEBYTECODE='1')
    if args.install:
        subprocess.run([sys.executable, '-m', 'pip', 'install', '--only-binary=:all:', '--no-compile',
                        '--disable-pip-version-check', '--cache-dir', str(CACHE / 'pip-cache'),
                        '--upgrade', '--target', str(DEPENDENCIES), *WHEELS], env=env, check=True)
    if not (PREFIX / 'include/pinocchio').is_dir():
        parser.error('Missing isolated dependencies. Run with --install.')
    # assimp looks for zlib next to itself; the cmeel zlib wheel uses lib64.
    zlib = PREFIX / 'lib/libz.so.1'
    if not zlib.exists() and (PREFIX / 'lib64/libz.so.1').exists():
        zlib.symlink_to('../lib64/libz.so.1')
    build = build_dir()
    build.mkdir(parents=True, exist_ok=True)
    # The upstream install RPATH is relative to the extension package.
    link = build / 'cmeel.prefix'
    if not link.exists():
        link.symlink_to(PREFIX, target_is_directory=True)
    env.update(FAIRINO_PINOCCHIO_PREFIX=str(PREFIX), LD_LIBRARY_PATH=str(PREFIX / 'lib'),
               CMAKE_PREFIX_PATH=str(PREFIX), PYTHONPATH='')
    subprocess.run(['cmake', '-S', str(VENDOR), '-B', str(build / 'cmake'), '-G', 'Ninja',
                    '-DCMAKE_BUILD_TYPE=Release', '-DPython_EXECUTABLE=' + sys.executable,
                    '-Dpybind11_DIR=' + str(DEPENDENCIES / 'pybind11/share/cmake/pybind11'),
                    '-DCMAKE_INSTALL_PREFIX=' + str(build)], env=env, check=True)
    subprocess.run(['cmake', '--build', str(build / 'cmake'), '--parallel', '1'], env=env, check=True)
    subprocess.run(['cmake', '--install', str(build / 'cmake')], env=env, check=True)
    (CACHE / 'ldd.txt').write_text(check_linkage(extension_path()))
    print(extension_path())


if __name__ == '__main__':
    main()
