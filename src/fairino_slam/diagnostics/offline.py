"""Validate local distribution and run regression tests without ROS or SSH."""
import argparse
import ast
import importlib.util
import json
import sys
import unittest
import xml.etree.ElementTree as ET

from fairino_slam.paths import PACKAGE_ROOT, PROJECT_ROOT, RESOURCE_ROOT, CONFIG_ROOT, ROS_ROOT


def validate():
    import yaml
    from fairino_slam.configuration import load_chassis, load_navigation, load_ros_profile
    from fairino_slam.deployment.build_navigation import build
    from fairino_slam.deployment import install_control_stack, install_map_mode, install_navigation
    files = sorted(PACKAGE_ROOT.rglob("*.py"))
    for path in files:
        ast.parse(path.read_text(), filename=str(path))
    load_chassis()
    load_navigation()
    load_ros_profile('mapping.yaml')
    load_ros_profile('localization.yaml')
    for name in ('navigate_to_pose.xml', 'navigate_through_poses.xml'):
        ET.parse(CONFIG_ROOT / 'behavior_trees' / name)
    ET.parse(RESOURCE_ROOT / 'models/amr_8dof.urdf')
    for package in ('fairino_lookahead_controller', 'fairino_straight_planner'):
        for name in ('package.xml', 'plugin.xml'):
            ET.parse(ROS_ROOT / package / name)
    generated = build()
    if yaml.safe_load(yaml.safe_dump(generated)) != generated:
        raise RuntimeError('导航配置无法无损序列化')
    scripts = {
        "control_raw": install_control_stack.remote_script(False, "raw"),
        "control_ekf": install_control_stack.remote_script(False, "ekf"),
        "mapping": install_map_mode.remote_script(),
        "navigation": install_navigation.remote_script(False),
    }
    for name, source in scripts.items():
        ast.parse(source, filename=name + "_payload.py")
    hashes = json.loads((RESOURCE_ROOT / "slam_toolbox/sources.json").read_text())
    if "LICENSE" not in hashes or len(hashes) != 8:
        raise RuntimeError("SLAM 固定源码清单不完整")
    return {"python_files": len(files), "configuration_root": str(CONFIG_ROOT),
            "generated_navigation_config": "validated",
            "deployment_payloads": list(scripts), "robot_connected": False,
            "gtk_available": importlib.util.find_spec("gi") is not None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tests", action="store_true", help="同时运行工程内离线回归")
    args = parser.parse_args()
    result = validate()
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    if args.tests:
        tests = PROJECT_ROOT / "tests"
        if not tests.is_dir():
            raise RuntimeError("回归测试需要源代码工程；安装包可运行 check")
        sys.path.insert(0, str(PROJECT_ROOT))
        suite = unittest.defaultTestLoader.discover(str(tests), top_level_dir=str(PROJECT_ROOT))
        outcome = unittest.TextTestRunner(verbosity=2).run(suite)
        return int(not outcome.wasSuccessful())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
