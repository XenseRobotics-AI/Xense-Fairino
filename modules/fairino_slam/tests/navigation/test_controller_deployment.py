"""Navigation plugin deployment checks never execute ROS, SSH, or a compiler."""
import ast
import base64
from pathlib import Path
import tempfile
import shlex
import lzma
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from fairino_slam.deployment import build_controller_remote as remote
from fairino_slam.deployment.install_navigation import remote_script, remote_command
from fairino_slam.paths import ROS_ROOT


PACKAGES = ('fairino_lookahead_controller', 'fairino_straight_planner')


def minimal_sources():
    return {name: {'CMakeLists.txt': base64.b64encode(b'project(test)').decode()}
            for name in PACKAGES}


class ControllerDeploymentTests(unittest.TestCase):
    def test_payload_contains_both_plugins_and_builds_before_runtime_switch(self):
        program = remote_script()
        constants = {}
        for statement in ast.parse(program).body:
            if isinstance(statement, ast.Assign) and isinstance(statement.targets[0], ast.Name):
                try:
                    constants[statement.targets[0].id] = ast.literal_eval(statement.value)
                except (ValueError, TypeError):
                    pass
        packages = constants['NAVIGATION_PLUGIN_SOURCES']
        self.assertEqual(set(packages), set(PACKAGES))
        required = {
            'fairino_lookahead_controller': (
                'CMakeLists.txt', 'package.xml', 'plugin.xml', 'src/controller.cpp',
                'src/plugin_check.cpp', 'include/fairino_lookahead_controller/core.hpp',
                'include/fairino_lookahead_controller/collision.hpp',
                'include/fairino_lookahead_controller/coordination.hpp',
                'include/fairino_lookahead_controller/replan.hpp', 'src/replan_bt.cpp'),
            'fairino_straight_planner': (
                'CMakeLists.txt', 'package.xml', 'plugin.xml', 'src/planner.cpp',
                'src/plugin_check.cpp', 'include/fairino_straight_planner/core.hpp',
                'include/fairino_straight_planner/planner_api.hpp',
                'include/fairino_straight_planner/planner_exceptions.hpp'),
        }
        for package, names in required.items():
            for name in names:
                self.assertEqual(base64.b64decode(packages[package][name]),
                                 (ROS_ROOT/package/name).read_bytes())
            self.assertFalse(any(name.startswith('test/') for name in packages[package]))
        self.assertLess(program.index('navigation_install = build_navigation_plugins'),
                        program.index("for entry in entries:"))
        self.assertNotIn("'systemctl', 'start'", program)
        self.assertNotIn('enable()', program)

    def test_feedback_dependencies_are_checked_before_build_or_runtime_update(self):
        program = remote_script()
        tree = ast.parse(program)
        package_checks = [n for n in ast.walk(tree) if isinstance(n, ast.For)
                          and isinstance(n.target, ast.Name) and n.target.id == 'package']
        self.assertEqual(len(package_checks), 1)
        self.assertIn('diagnostic_msgs', ast.literal_eval(package_checks[0].iter))
        dependency = "from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue"
        self.assertIn(dependency, program)
        self.assertLess(program.index(dependency), program.index('navigation_install = build_navigation_plugins'))
        self.assertLess(program.index(dependency), program.index("for entry in entries:"))

    def test_transport_compresses_cpp_sources_below_single_argument_limit(self):
        command = remote_command()
        self.assertLess(len(command.encode()), 120000)
        tree = ast.parse(shlex.split(command)[-1])
        values = [node.value for node in ast.walk(tree) if isinstance(node, ast.Constant) and isinstance(node.value, str)]
        self.assertEqual(len(values), 1)
        self.assertEqual(lzma.decompress(base64.b85decode(values[0])).decode(), remote_script())

    def test_build_failure_does_not_install_or_change_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            marker = runtime/'runtime.sh'
            marker.write_text('existing runtime')
            with patch.object(remote.shutil, 'which', return_value='/tool'), \
                 patch.object(remote.subprocess, 'run', return_value=SimpleNamespace(returncode=1)) as run:
                with self.assertRaisesRegex(RuntimeError, '未更新导航服务'):
                    remote.build_controller(runtime, minimal_sources()[PACKAGES[0]])
                run.assert_called_once()
                self.assertEqual(marker.read_text(), 'existing runtime')

    def fake_builder(self, commands, installs, fail_package=None, fail_probe=None, missing_artifact=None):
        def run(command, **kwargs):
            commands.append(command)
            if command[:2] == ['cmake', '-S']:
                name = Path(command[2]).name
                installs[name] = Path(next(v.split('=', 1)[1] for v in command
                                          if v.startswith('-DCMAKE_INSTALL_PREFIX=')))
                if name == fail_package:
                    return SimpleNamespace(returncode=1)
            if command[:2] == ['cmake', '--install']:
                name = Path(command[2]).name
                install = installs[name]
                extra = ('lib/libfairino_replan_bt_node.so',) if name == PACKAGES[0] else ()
                for relative in extra + ('share/' + name + '/local_setup.bash',
                                 'lib/lib' + name + '.so', 'lib/' + name + '/plugin_check'):
                    if relative == missing_artifact:
                        continue
                    path = install/relative
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text('test artifact')
            if len(command) == 1:
                checker = Path(command[0])
                name = checker.parent.name
                install = installs[name]
                self.assertEqual(kwargs['env']['AMENT_PREFIX_PATH'].split(':')[0], str(install))
                self.assertEqual(kwargs['env']['LD_LIBRARY_PATH'].split(':')[0], str(install/'lib'))
                if name == fail_probe:
                    return SimpleNamespace(returncode=1)
            return SimpleNamespace(returncode=0)
        return run

    def test_single_controller_compatibility_entry_keeps_isolated_prefix_and_load_probe(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            commands, installs = [], {}
            with patch.object(remote.shutil, 'which', return_value='/tool'), \
                 patch.object(remote.subprocess, 'run', side_effect=self.fake_builder(commands, installs)):
                result = remote.build_controller(runtime, minimal_sources()[PACKAGES[0]])
            self.assertEqual(result, installs[PACKAGES[0]])
            self.assertTrue(result.is_relative_to(runtime/'controller-builds'))
            self.assertEqual(commands[-1], [str(result/'lib'/PACKAGES[0]/'plugin_check')])

    def test_navigation_overlay_contains_both_packages_and_checks_both_before_return(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            marker = runtime/'runtime.sh'
            marker.write_text('existing runtime')
            commands, installs = [], {}
            with patch.object(remote.shutil, 'which', return_value='/tool'), \
                 patch.object(remote.subprocess, 'run', side_effect=self.fake_builder(commands, installs)):
                result = remote.build_navigation_plugins(runtime, minimal_sources())
            self.assertEqual(set(installs), set(PACKAGES))
            self.assertEqual(set(installs.values()), {result})
            self.assertTrue(result.is_relative_to(runtime/'navigation-plugin-builds'))
            for package in PACKAGES:
                self.assertTrue((result/'lib'/('lib' + package + '.so')).is_file())
                self.assertTrue((result.parent/'source'/package/'CMakeLists.txt').is_file())
            self.assertEqual(commands[-2:], [[str(result/'lib'/package/'plugin_check')] for package in PACKAGES])
            self.assertEqual(sum(command[:2] == ['cmake', '--install'] for command in commands[:-2]), 2)
            self.assertEqual(marker.read_text(), 'existing runtime')

    def test_second_package_build_failure_does_not_probe_or_change_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            marker = runtime/'runtime.sh'
            marker.write_text('existing runtime')
            commands, installs = [], {}
            with patch.object(remote.shutil, 'which', return_value='/tool'), \
                 patch.object(remote.subprocess, 'run', side_effect=self.fake_builder(
                     commands, installs, fail_package=PACKAGES[1])):
                with self.assertRaisesRegex(RuntimeError, '未更新导航服务'):
                    remote.build_navigation_plugins(runtime, minimal_sources())
            self.assertEqual(set(installs), set(PACKAGES))
            self.assertFalse(any(len(command) == 1 for command in commands))
            self.assertEqual(marker.read_text(), 'existing runtime')

    def test_either_plugin_load_failure_keeps_existing_runtime(self):
        for package in PACKAGES:
            with self.subTest(package=package), tempfile.TemporaryDirectory() as directory:
                runtime = Path(directory)
                marker = runtime/'runtime.sh'
                marker.write_text('existing runtime')
                commands, installs = [], {}
                with patch.object(remote.shutil, 'which', return_value='/tool'), \
                     patch.object(remote.subprocess, 'run', side_effect=self.fake_builder(
                         commands, installs, fail_probe=package)):
                    with self.assertRaisesRegex(RuntimeError, '未更新导航服务'):
                        remote.build_navigation_plugins(runtime, minimal_sources())
                self.assertEqual(sum(command[:2] == ['cmake', '--install'] for command in commands), 2)
                self.assertEqual(marker.read_text(), 'existing runtime')

    def test_missing_planner_artifact_prevents_load_and_runtime_switch(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            marker = runtime/'runtime.sh'
            marker.write_text('existing runtime')
            commands, installs = [], {}
            with patch.object(remote.shutil, 'which', return_value='/tool'), \
                 patch.object(remote.subprocess, 'run', side_effect=self.fake_builder(
                     commands, installs, missing_artifact='lib/libfairino_straight_planner.so')):
                with self.assertRaisesRegex(RuntimeError, '安装产物不完整.*未更新导航服务'):
                    remote.build_navigation_plugins(runtime, minimal_sources())
            self.assertFalse(any(len(command) == 1 for command in commands))
            self.assertEqual(marker.read_text(), 'existing runtime')

    def test_missing_bt_artifact_prevents_runtime_switch(self):
        with tempfile.TemporaryDirectory() as directory:
            commands, installs = [], {}
            with patch.object(remote.shutil, 'which', return_value='/tool'), \
                 patch.object(remote.subprocess, 'run', side_effect=self.fake_builder(
                     commands, installs, missing_artifact='lib/libfairino_replan_bt_node.so')):
                with self.assertRaisesRegex(RuntimeError, '安装产物不完整.*未更新导航服务'):
                    remote.build_navigation_plugins(directory, minimal_sources())
            self.assertFalse(any(len(command) == 1 for command in commands))

    def test_navigation_overlay_rejects_missing_package_before_staging(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(remote.subprocess, 'run') as run:
            with self.assertRaisesRegex(ValueError, 'requires both'):
                remote.build_navigation_plugins(directory, {PACKAGES[0]: minimal_sources()[PACKAGES[0]]})
            run.assert_not_called()
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_source_paths_cannot_escape_build_directory(self):
        for relative in ('../outside', '/outside', './CMakeLists.txt', 'a/../outside', ''):
            with self.subTest(relative=relative), tempfile.TemporaryDirectory() as directory, \
                 patch.object(remote.shutil, 'which', return_value='/tool'), \
                 patch.object(remote.subprocess, 'run') as run:
                sources = minimal_sources()
                sources[PACKAGES[1]] = {relative: ''}
                with self.assertRaises(ValueError):
                    remote.build_navigation_plugins(directory, sources)
                run.assert_not_called()
                self.assertEqual(list(Path(directory).iterdir()), [])

    def test_invalid_later_package_encoding_does_not_stage_earlier_sources(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(remote.subprocess, 'run') as run:
            sources = minimal_sources()
            sources[PACKAGES[1]] = {'CMakeLists.txt': 'not base64!'}
            with self.assertRaises(ValueError):
                remote.build_navigation_plugins(directory, sources)
            run.assert_not_called()
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_build_timeout_reports_log_and_keeps_existing_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            marker = runtime/'runtime.sh'
            marker.write_text('existing runtime')
            with patch.object(remote.shutil, 'which', return_value='/tool'), \
                 patch.object(remote.subprocess, 'run', side_effect=remote.subprocess.TimeoutExpired('cmake', 600)):
                with self.assertRaisesRegex(RuntimeError, '超时.*未更新导航服务.*build.log'):
                    remote.build_navigation_plugins(runtime, minimal_sources())
            self.assertEqual(marker.read_text(), 'existing runtime')


if __name__ == '__main__':
    unittest.main()
