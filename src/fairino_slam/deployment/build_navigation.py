#!/usr/bin/env python3
"""Generate Nav2 parameters from config/ and the checked chassis URDF."""
import argparse
from pathlib import Path
import yaml

from fairino_slam.paths import RESOURCE_ROOT, GENERATED_ROOT
from fairino_slam.configuration import load_chassis
from fairino_slam.navigation.footprint import ChassisFootprint
from fairino_slam.navigation.profile import build_profile


def build():
    speed = load_chassis()['speed']
    return build_profile(ChassisFootprint(RESOURCE_ROOT/'models/amr_8dof.urdf').points,
                         speed['default_linear'], speed['default_angular'])


def write_config(target=None):
    target = Path(target) if target is not None else GENERATED_ROOT/'nav2.yaml'
    data = build()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text('# Generated from config/chassis.yaml and config/navigation.yaml; do not edit.\n'
                      + yaml.safe_dump(data, sort_keys=False))
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='只校验配置源和生成结果，不写文件')
    args = parser.parse_args()
    if args.check:
        data = build()
        if yaml.safe_load(yaml.safe_dump(data)) != data:
            raise RuntimeError('导航配置无法无损序列化')
        print('Navigation configuration and chassis defaults are valid')
    else:
        print(write_config())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
