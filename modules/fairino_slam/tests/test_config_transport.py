"""Configuration snapshots reach every standalone robot source without SSH."""
import ast
import base64
import builtins
import copy
import json
import math
from pathlib import Path
import shlex
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import zlib

import yaml

from fairino_slam.chassis import control_core, source, transport
from fairino_slam import configuration
from fairino_slam.configuration import ConfigurationError, load_chassis, load_ros_profile
from fairino_slam.deployment import (
    install_control_stack, install_map_mode, install_mapping_runtime, install_navigation,
)
from fairino_slam.diagnostics import check_control_link


def literal_assignments(program):
    result = {}
    for node in ast.parse(program).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if isinstance(target, ast.Name):
                try:
                    result[target.id] = ast.literal_eval(node.value)
                except (ValueError, TypeError):
                    pass
    return result


def unpack_shell_bundle(command):
    """Inspect quoted Python without invoking a shell or the remote command."""
    shell = shlex.split(command)[-1]
    loader = shlex.split(shell)[-1]
    for node in ast.walk(ast.parse(loader)):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'b64decode'):
            encoded = ast.literal_eval(node.args[0])
            return json.loads(zlib.decompress(base64.b64decode(encoded)))
    raise AssertionError('No configuration/source bundle in SSH loader')


def isolated_core(program):
    original_import = builtins.__import__

    def without_desktop(name, *args, **kwargs):
        if name == 'yaml' or name.startswith('fairino_slam'):
            raise ImportError('Robot cannot import desktop dependency: ' + name)
        return original_import(name, *args, **kwargs)

    namespace = {'__name__': 'remote_configuration_probe',
                 '__builtins__': {**vars(builtins), '__import__': without_desktop}}
    exec(compile(program, 'remote_control_core.py', 'exec'), namespace)
    if '__file__' in namespace:
        raise AssertionError('The probe must not supply a file location')
    return namespace


class ConfigurationTransportTests(unittest.TestCase):
    def changed_config(self):
        config = copy.deepcopy(load_chassis())
        config['speed'].update(default_linear=.08, default_angular=.12, min_speed=.01,
                               max_linear=.18, max_angular=.35)
        config['limits']['linear']['max_acceleration'] = .16
        return config

    def test_rendered_core_uses_snapshot_without_package_yaml_or_file(self):
        with patch.object(source, 'load_chassis', return_value=self.changed_config()):
            program = source.render_control_core()
        core = isolated_core(program)
        self.assertEqual(core['DEFAULT_LINEAR'], .08)
        self.assertEqual(core['DEFAULT_ANGULAR'], .12)
        self.assertEqual(core['MAX_LINEAR'], .18)
        self.assertEqual(core['MAX_ANGULAR'], .35)
        vx, vy, yaw = core['limit_planar_twist']([1., 1., 0., 0., 0., 1.])
        self.assertAlmostEqual(math.hypot(vx, vy), .08)
        self.assertEqual(yaw, .12)
        params = core['chassis_parameters']({
            'limits': {'base': {'linear': {}, 'angular': {}},
                       'steering': {}, 'drive': {}}})
        self.assertEqual(params['limits']['base']['linear']['max_velocity'], .18)
        self.assertEqual(params['limits']['base']['linear']['max_acceleration'], .16)

    def test_all_remote_paths_carry_identical_validated_snapshot(self):
        options = SimpleNamespace(linear=.08, angular=.12, navigation=True,
                                  check=False, prepare_check=False)
        diagnostic = SimpleNamespace(seconds=5, nice=0)
        config = self.changed_config()
        with patch.object(source, 'load_chassis', return_value=config), \
                patch.object(transport, 'configuration_snapshot', return_value=config), \
                patch.object(check_control_link, 'configuration_snapshot', return_value=config):
            expected = source.render_control_core()
            worker = unpack_shell_bundle(transport.remote_command(options))['control_core.py']
            probe = unpack_shell_bundle(check_control_link.command(diagnostic))['control_core.py']
            installer = literal_assignments(install_control_stack.remote_script())['CORE']
            navigation = literal_assignments(install_navigation.remote_script())['PAYLOAD']
            deployed = base64.b64decode(navigation['control_core.py']).decode()
        for name, program in (('worker', worker), ('diagnostic', probe),
                              ('control installer', installer), ('navigation', deployed)):
            with self.subTest(path=name):
                self.assertEqual(program, expected)
                self.assertEqual(isolated_core(program)['DEFAULT_LINEAR'], .08)

    def test_new_connections_keep_process_snapshot_when_files_have_changed(self):
        current = control_core.configuration_snapshot()
        changed = self.changed_config()
        if changed == current:
            changed['speed']['default_linear'] = .09
        self.assertNotEqual(changed, current)
        options = SimpleNamespace(linear=control_core.DEFAULT_LINEAR,
                                  angular=control_core.DEFAULT_ANGULAR,
                                  navigation=True, check=False, prepare_check=False)
        with patch.object(source, 'load_chassis', return_value=changed) as disk_read:
            worker = unpack_shell_bundle(transport.remote_command(options))['control_core.py']
            probe = unpack_shell_bundle(check_control_link.command(
                SimpleNamespace(seconds=5, nice=0)))['control_core.py']
            disk_read.assert_not_called()
            fresh = isolated_core(source.render_control_core())['configuration_snapshot']()
            disk_read.assert_called_once_with()
        self.assertEqual(fresh, changed)
        for program in (worker, probe):
            core = isolated_core(program)
            self.assertEqual(core['configuration_snapshot'](), current)
            self.assertEqual(core['DEFAULT_LINEAR'], control_core.DEFAULT_LINEAR)
            self.assertEqual(core['DEFAULT_ANGULAR'], control_core.DEFAULT_ANGULAR)
        detached = control_core.configuration_snapshot()
        detached['speed']['default_linear'] = -1
        self.assertEqual(control_core.configuration_snapshot(), current)

    def test_explicit_snapshot_must_pass_validation(self):
        invalid = self.changed_config()
        invalid['speed']['default_linear'] = float('nan')
        with self.assertRaisesRegex(ConfigurationError, 'default_linear'):
            source.render_control_core(invalid)

    def test_navigation_deployment_rejects_invalid_behavior_tree(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'behavior_trees').mkdir()
            (root / 'behavior_trees/navigate_to_pose.xml').write_text('<root><bad>')
            with patch.object(install_navigation, 'CONFIG_ROOT', root), \
                    patch.object(install_navigation, 'run') as remote:
                with self.assertRaises(ET.ParseError):
                    install_navigation.remote_script()
                remote.assert_not_called()

    def test_deployment_rejects_invalid_mapping_before_remote_operations(self):
        mapping = load_ros_profile('mapping.yaml')
        localization = load_ros_profile('localization.yaml')
        mapping['slam_toolbox']['ros__parameters']['resolution'] = -1
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'mapping.yaml').write_text(yaml.safe_dump(mapping))
            (root / 'localization.yaml').write_text(yaml.safe_dump(localization))
            with patch.object(configuration, 'CONFIG_ROOT', root), \
                    patch.object(install_mapping_runtime, 'remote') as remote:
                calls = (
                    install_control_stack.remote_script,
                    install_map_mode.remote_script,
                    lambda: install_mapping_runtime.verify({}, 'robot', 'user', 'unused'),
                    lambda: install_mapping_runtime.activate({}, 'robot', 'user', 'unused'),
                )
                for call in calls:
                    with self.subTest(entrypoint=call), self.assertRaisesRegex(ConfigurationError, 'resolution'):
                        call()
                remote.assert_not_called()

    def test_deployment_rejects_invalid_localization_before_rendering_payload(self):
        mapping = load_ros_profile('mapping.yaml')
        localization = load_ros_profile('localization.yaml')
        localization['amcl']['ros__parameters']['max_particles'] = 0
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'mapping.yaml').write_text(yaml.safe_dump(mapping))
            (root / 'localization.yaml').write_text(yaml.safe_dump(localization))
            with patch.object(configuration, 'CONFIG_ROOT', root):
                for call in (install_control_stack.remote_script, install_map_mode.remote_script):
                    with self.subTest(entrypoint=call), self.assertRaisesRegex(ConfigurationError, 'max_particles'):
                        call()


if __name__ == '__main__':
    unittest.main()
