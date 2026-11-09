"""Copy the single editable config tree into wheel build output only.

Project metadata stays in pyproject.toml. Use scripts/setup.sh for installation.
"""
from pathlib import Path
import shutil

from setuptools import setup
from setuptools.command.build_py import build_py


SOURCE_ROOT = Path(__file__).resolve().parent
RESOURCE_TREES = ("config", "ros2")


class BuildPy(build_py):
    def run(self):
        super().run()
        if self.editable_mode:
            return
        for name in RESOURCE_TREES:
            source = SOURCE_ROOT / name
            if not source.is_dir():
                raise RuntimeError("Missing source resource directory: " + str(source))
            target = Path(self.build_lib) / 'fairino_slam/resources' / name
            if target.exists():
                shutil.rmtree(target)
            shutil.copytree(source, target, ignore=shutil.ignore_patterns(
                '__pycache__', '*.pyc', '*.pyo', '.DS_Store', 'build', 'install', 'log'))

    def get_outputs(self, include_bytecode=True):
        outputs = super().get_outputs(include_bytecode)
        if not self.editable_mode:
            for name in RESOURCE_TREES:
                target = Path(self.build_lib) / "fairino_slam/resources" / name
                outputs.extend(str(path) for path in target.rglob("*") if path.is_file())
        return outputs


setup(cmdclass={"build_py": BuildPy})
