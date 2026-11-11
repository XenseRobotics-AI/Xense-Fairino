#!/usr/bin/env python3
"""Install/build the optional TRON2 solver without modifying the PICO env.

Run with the same interpreter as PICO, normally .local/pico4/env/bin/python.
--install is the only operation which downloads anything; PICO startup never
downloads dependencies or builds C++. Outputs stay under ignored .local/.
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'apps/fairino_tcp_probe')]
from tools.ik_experiments.tron2_art7 import CACHE, VENDOR, source_digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install', action='store_true', help='download optional native build dependencies')
    args = parser.parse_args()
    dependencies = CACHE/'site-packages'
    if args.install:
        subprocess.run([sys.executable, '-m', 'pip', 'install', '--target', str(dependencies),
            'pin==3.9.0', 'cmeel-urdfdom==4.0.1', 'cmeel-tinyxml2==10.0.0',
            'eiquadprog==1.3.0', 'pybind11==3.1.0', 'setuptools>=68', 'wheel'], check=True)
        subprocess.run([sys.executable, '-m', 'pip', 'install', '--target', str(CACHE/'eigen-deps'),
                        'cmeel-eigen==3.4.1'], check=True)
    prefix = dependencies/'cmeel.prefix'
    if not (prefix/'include/pinocchio').is_dir():
        parser.error('Missing isolated Pinocchio dependencies; run again with --install')
    build = CACHE/'builds'/source_digest()
    environment = os.environ.copy()
    environment['TRON2_PINOCCHIO_PREFIX'] = str(prefix)
    environment['TRON2_EIGEN_INCLUDE'] = str(CACHE/'eigen-deps/cmeel.prefix/include/eigen3')
    environment['PYTHONPATH'] = str(dependencies)
    # Do not permit a sourced ROS environment to substitute another Pinocchio.
    environment['LD_LIBRARY_PATH'] = str(prefix/'lib')
    subprocess.run([sys.executable, 'setup.py', 'build_ext', '--build-lib', str(build),
                    '--build-temp', str(build/'temp')], cwd=VENDOR, env=environment, check=True)
    print('TRON2 IK built:', build)


if __name__ == '__main__':
    main()
