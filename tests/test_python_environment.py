"""Exercise real shell entrypoints using isolated recording Python executables."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


FAKE_PYTHON = r'''
import json, os, pathlib, shutil, sys
with open(os.environ['FAIRINO_TEST_INVOCATIONS'], 'a') as stream:
    stream.write(json.dumps({
        'python': sys.argv[0], 'args': sys.argv[1:],
        'selected': os.environ.get('FAIRINO_SLAM_PYTHON'),
    }) + '\n')
if sys.argv[1:2] == ['-c'] and 'sys.version_info' in sys.argv[2]:
    if pathlib.Path(sys.argv[0] + '.invalid').exists():
        raise SystemExit(1)
if sys.argv[1:3] == ['-m', 'venv']:
    destination = pathlib.Path(sys.argv[-1]) / 'bin' / 'python'
    destination.parent.mkdir(parents=True)
    shutil.copy2(sys.argv[0], destination)
'''


class PythonEnvironmentTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='fairino environment ')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / 'project with spaces'
        shutil.copytree(Path(__file__).resolve().parents[1] / 'scripts',
                        self.root / 'scripts')
        # Test interpreter selection independently of compiler availability.
        (self.root / 'scripts/check_controller.sh').write_text('#!/usr/bin/env bash\nexit 0\n')
        self.log = Path(self.temporary.name) / 'invocations.jsonl'
        self.path_python = self.make_python(Path(self.temporary.name) / 'path bin/python3')
        self.env = dict(os.environ)
        for key in ('FAIRINO_SLAM_PYTHON', 'FAIRINO_SLAM_SYSTEM_PYTHON',
                    'CONDA_PREFIX', 'VIRTUAL_ENV', 'PYTHONHOME', 'PYTHONPATH'):
            self.env.pop(key, None)
        self.env.update(
            PATH=str(self.path_python.parent) + os.pathsep + os.environ['PATH'],
            FAIRINO_TEST_INVOCATIONS=str(self.log), PYTHONDONTWRITEBYTECODE='1')

    def make_python(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('#!' + sys.executable + '\n' + FAKE_PYTHON)
        path.chmod(0o755)
        return path

    def invoke(self, entry, *arguments, extra_env=None):
        self.log.unlink(missing_ok=True)
        env = dict(self.env)
        env.update(extra_env or {})
        result = subprocess.run(['bash', str(self.root / 'scripts' / entry), *arguments],
                                cwd=self.temporary.name, env=env, text=True,
                                capture_output=True, timeout=15)
        calls = ([json.loads(line) for line in self.log.read_text().splitlines()]
                 if self.log.exists() else [])
        return result, calls

    def entries(self):
        return [('setup.sh', 'cli'), ('run.sh', 'check'), ('check.sh', '--package')]

    def assert_selected(self, result, calls, python):
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue(calls)
        self.assertEqual({call['python'] for call in calls}, {str(python)})

    def test_explicit_python_overrides_conda_venv_and_path(self):
        explicit = self.make_python(Path(self.temporary.name) / 'explicit python/bin/python')
        conda = self.make_python(Path(self.temporary.name) / 'conda environment/bin/python')
        self.make_python(self.root / '.venv/bin/python')
        for entry, argument in self.entries():
            with self.subTest(entry=entry):
                result, calls = self.invoke(entry, argument, extra_env={
                    'FAIRINO_SLAM_PYTHON': str(explicit),
                    'CONDA_PREFIX': str(conda.parent.parent)})
                self.assert_selected(result, calls, explicit)

    def test_conda_bypasses_damaged_venv_without_touching_it(self):
        conda = self.make_python(Path(self.temporary.name) / 'conda environment/bin/python')
        damaged = self.root / '.venv/bin/python'
        damaged.parent.mkdir(parents=True)
        damaged.write_text('copied or incomplete environment; preserve me\n')
        damaged.chmod(0o644)
        before = damaged.read_bytes(), damaged.stat().st_mode
        for entry, argument in self.entries():
            with self.subTest(entry=entry):
                result, calls = self.invoke(entry, argument, extra_env={
                    'CONDA_PREFIX': str(conda.parent.parent)})
                self.assert_selected(result, calls, conda)
                self.assertEqual((damaged.read_bytes(), damaged.stat().st_mode), before)
                self.assertFalse(any(call['args'][:2] == ['-m', 'venv'] for call in calls))

    def test_invalid_explicit_or_conda_python_never_falls_back(self):
        self.make_python(self.root / '.venv/bin/python')
        missing = Path(self.temporary.name) / 'missing python'
        for override in ({'FAIRINO_SLAM_PYTHON': str(missing)},
                         {'CONDA_PREFIX': str(missing)}):
            for entry, argument in self.entries():
                with self.subTest(override=override, entry=entry):
                    result, calls = self.invoke(entry, argument, extra_env=override)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertEqual(calls, [])
                    self.assertIn(str(missing), result.stderr)

    def test_python_validation_failure_prevents_install_and_fallback(self):
        invalid = self.make_python(Path(self.temporary.name) / 'invalid version/bin/python')
        Path(str(invalid) + '.invalid').touch()
        for override in ({'FAIRINO_SLAM_PYTHON': str(invalid)},
                         {'CONDA_PREFIX': str(invalid.parent.parent)}):
            result, calls = self.invoke('setup.sh', 'cli', extra_env=override)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(len(calls), 1)
            self.assertEqual(calls[0]['python'], str(invalid))
            self.assertEqual(calls[0]['args'][0], '-c')
            self.assertFalse((self.root / '.venv').exists())

    def test_setup_creates_venv_only_when_no_environment_exists(self):
        result, calls = self.invoke('setup.sh', 'dev')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        created = self.root / '.venv/bin/python'
        self.assertTrue(created.is_file())
        creates = [call for call in calls if call['args'][:2] == ['-m', 'venv']]
        self.assertEqual(len(creates), 1)
        self.assertEqual(creates[0]['python'], str(self.path_python))
        self.assertEqual(creates[0]['args'][-1], str(self.root / '.venv'))
        self.assertEqual(calls[-1]['python'], str(created))
        self.assertEqual(calls[-1]['args'],
                         ['-m', 'pip', 'install', '-e', str(self.root) + '[dev]'])

    def test_existing_venv_is_reused_without_conda(self):
        python = self.make_python(self.root / '.venv/bin/python')
        for entry, argument in self.entries():
            with self.subTest(entry=entry):
                result, calls = self.invoke(entry, argument)
                self.assert_selected(result, calls, python)
                self.assertFalse(any(call['args'][:2] == ['-m', 'venv'] for call in calls))

    def test_damaged_venv_without_override_is_preserved_and_rejected(self):
        damaged = self.root / '.venv'
        damaged.mkdir()
        marker = damaged / 'existing-data'
        marker.write_text('preserve')
        for entry, argument in self.entries():
            with self.subTest(entry=entry):
                result, calls = self.invoke(entry, argument)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(calls, [])
                self.assertIn('.venv', result.stderr)
                self.assertEqual(marker.read_text(), 'preserve')
                self.assertEqual(list(damaged.iterdir()), [marker])

    def test_run_and_check_fall_back_to_path_and_package_uses_same_python(self):
        result, calls = self.invoke('run.sh', 'check')
        self.assert_selected(result, calls, self.path_python)
        self.assertEqual(calls[-1]['args'], ['-m', 'fairino_slam', 'check'])
        result, calls = self.invoke('check.sh', '--package')
        self.assert_selected(result, calls, self.path_python)
        tests = [call for call in calls if call['args'] == ['-m', 'fairino_slam', 'test']]
        package = [call for call in calls
                   if call['args'] == [str(self.root / 'scripts/check_package.py')]]
        self.assertEqual(len(tests), 1)
        self.assertEqual(len(package), 1)
        self.assertEqual(tests[0]['selected'], str(self.path_python))
        self.assertEqual(package[0]['selected'], str(self.path_python))
        self.assertFalse((self.root / '.venv').exists())


if __name__ == '__main__':
    unittest.main()
