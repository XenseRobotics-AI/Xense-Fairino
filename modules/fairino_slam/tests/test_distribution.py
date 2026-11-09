"""Test the source-to-robot packaging boundary introduced by the migration."""
import ast
import base64
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from fairino_slam.paths import PACKAGE_ROOT, RESOURCE_ROOT
from fairino_slam.deployment import install_control_stack, install_map_mode, install_navigation


def constants(source):
    values = {}
    for node in ast.parse(source).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            try:
                values[node.targets[0].id] = ast.literal_eval(node.value)
            except (ValueError, TypeError):
                pass
    return values


class DistributionTests(unittest.TestCase):
    def test_every_public_help_is_offline(self):
        from fairino_slam.__main__ import COMMANDS, DEPLOY
        commands = [[name] for name in COMMANDS] + [['deploy', name] for name in DEPLOY]
        code = '''
import sys
def guard(event, args):
    if event in ('socket.connect', 'socket.getaddrinfo'):
        raise RuntimeError('Help attempted network access')
    if event == 'subprocess.Popen':
        executable = str(args[0]).rsplit('/', 1)[-1]
        if executable in ('ssh', 'sudo', 'runuser', 'ros2', 'systemctl'):
            raise RuntimeError('Help attempted robot/system command')
sys.addaudithook(guard)
from fairino_slam.__main__ import main
raise SystemExit(main(sys.argv[1:]))
'''
        for command in commands:
            with self.subTest(command=command):
                result = subprocess.run([sys.executable, '-c', code, *command, '--help'],
                                        capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertTrue(result.stdout.strip())

    def test_control_and_navigation_payloads_are_complete_and_parseable(self):
        control = constants(install_control_stack.remote_script())
        self.assertEqual(set(control['PAYLOAD']), {
            'odom_guard.py', 'raw_odometry.py', 'map_control_remote.py', 'mapping.yaml',
            'localization.py', 'localization.yaml'})
        navigation = constants(install_navigation.remote_script())
        self.assertEqual(set(navigation['PAYLOAD']), {
            'footprint.py', 'motion.py', 'coordination.py', 'chassis_receiver.py', 'motion_feedback.py', 'lifecycle.py', 'runtime_policy.py', 'tf_receiver.py', 'velocity_gate.py',
            'navigation_remote.py', 'navigation.launch.py', 'navigate_to_pose.xml',
            'navigate_through_poses.xml', 'navigate_to_saved_pose.xml', 'control_core.py', 'remote_stream.py', 'nav2.yaml'})
        for payload in (control['PAYLOAD'], navigation['PAYLOAD']):
            for name, encoded in payload.items():
                if name.endswith('.py'):
                    ast.parse(base64.b64decode(encoded).decode(), filename=name)
        self.assertIn('LICENSE', (RESOURCE_ROOT/'slam_toolbox/sources.json').read_text())

    def test_navigation_payload_keeps_heading_mode_and_corresponding_footprint(self):
        import json
        import yaml
        from fairino_slam.configuration import load_navigation
        from fairino_slam.deployment.build_navigation import build
        for mode, vertex_count in (('hold_initial', 64), ('path_tangent', 4)):
            with self.subTest(mode=mode):
                settings = load_navigation()
                settings['nav2']['controller_server']['ros__parameters']['FollowPath']['heading_mode'] = mode
                with patch('fairino_slam.navigation.profile.load_navigation', return_value=settings):
                    payload = constants(install_navigation.remote_script())['PAYLOAD']
                    profile = yaml.safe_load(base64.b64decode(payload['nav2.yaml']))
                    self.assertEqual(profile, build())
                self.assertEqual(profile['controller_server']['ros__parameters']['FollowPath']['heading_mode'], mode)
                local = profile['local_costmap']['local_costmap']['ros__parameters']
                global_map = profile['global_costmap']['global_costmap']['ros__parameters']
                self.assertEqual(len(json.loads(local['footprint'])), 4)
                self.assertEqual(len(json.loads(global_map['footprint'])), vertex_count)
                self.assertEqual(global_map['footprint_padding'], 0.)

    def test_mapping_installer_includes_localization_and_profiles(self):
        payload = constants(install_map_mode.remote_script())
        self.assertEqual(set(payload['LOCALIZATION']), {'localization.py', 'localization.yaml'})
        self.assertEqual(base64.b64decode(payload['PAYLOAD']),
                         (PACKAGE_ROOT/'mapping/map_control_remote.py').read_bytes())

    def test_remote_sources_import_without_local_package_or_file_variable(self):
        # Match the isolated SLAM verifier's exec contract exactly: no __file__.
        for relative in ('mapping/map_control_remote.py', 'mapping/remote_stream.py',
                         'diagnostics/record_odometry.py'):
            namespace = {'__name__': 'migration_remote_probe'}
            exec(compile((PACKAGE_ROOT/relative).read_text(), relative, 'exec'), namespace)

    def test_flat_runtime_modules_do_not_require_installed_package(self):
        navigation = constants(install_navigation.remote_script())['PAYLOAD']
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for name, value in navigation.items():
                (root/name).write_bytes(base64.b64decode(value))
            for name in ('localization.py', 'map_control_remote.py'):
                (root/name).write_bytes((PACKAGE_ROOT/'mapping'/name).read_bytes())
            code = '''
import importlib.abc, sys
class BlockLocalPackage(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.startswith('fairino_slam'):
            raise ModuleNotFoundError(fullname)
sys.meta_path.insert(0, BlockLocalPackage())
sys.path.insert(0, sys.argv[1])
import motion, lifecycle, remote_stream, localization, map_control_remote, velocity_gate, navigation_remote, runtime_policy, tf_receiver
assert velocity_gate.TF_MAX_AGE == navigation_remote.TF_MAX_AGE == runtime_policy.TF_MAX_AGE
assert velocity_gate.TfReceiver is tf_receiver.TfReceiver
assert navigation_remote.TfReceiver is tf_receiver.TfReceiver
assert motion.COMMAND_TIMEOUT == runtime_policy.COMMAND_TIMEOUT == .6
gate = motion.CommandGate(.06, .1)
gate.set_goal('goal', 0.)
gate.receive([.03, 0, 0, 0, 0, 0], 0.)
assert gate.output(.45, True, 'session') == (.03, 0., 0.)
assert gate.output(.61, True, 'session') == (0., 0., 0.)
assert '> 0.600 秒' in gate.failure['reason']
'''
            result = subprocess.run([sys.executable, '-I', '-c', code, folder],
                                    capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_no_imports_of_original_repository_packages(self):
        for path in PACKAGE_ROOT.rglob('*.py'):
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node, ast.Import):
                    names = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom):
                    names = [node.module or '']
                else:
                    continue
                for name in names:
                    self.assertFalse(name.startswith(('apps.', 'scripts.pico4', 'tools.')),
                                     f'{path}: {name}')


if __name__ == '__main__':
    unittest.main()
