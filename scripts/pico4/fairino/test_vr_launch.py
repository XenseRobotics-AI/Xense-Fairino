"""Run the real shell launcher against an argv-only stub, never robot code."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT/'src'))
sys.path.insert(0, str(HERE))

from vr_input import POSE_FRAMES, controller_sides, rotation_mapping


class LauncherMappingTests(unittest.TestCase):
    def launch_options(self, *extra):
        with tempfile.TemporaryDirectory(prefix='fairino launch ') as temporary:
            root = Path(temporary)
            launcher = root/'run.sh'
            shutil.copyfile(ROOT/'run.sh', launcher)
            stub = root/'scripts/pico4/run.sh'
            stub.parent.mkdir(parents=True)
            # NUL-delimited argv preserves boundaries and only prints arguments.
            # No real PICO entry point or robot worker exists in this directory.
            stub.write_text('#!/usr/bin/env bash\nset -euo pipefail\nprintf "%s\\0" "$@"\n')
            result = subprocess.run(
                ['bash', str(launcher), 'vr', *extra], cwd=root,
                env={**os.environ, 'BASH_ENV': '/dev/null'},
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, timeout=5)
        argv = result.stdout.decode().split('\0')
        self.assertEqual(argv.pop(), '')
        self.assertEqual(argv.pop(0), 'fairino')
        self.assertIn('--execute', argv)
        parser = argparse.ArgumentParser()
        parser.add_argument('--mode', choices=tuple(POSE_FRAMES), default='first_person')
        parser.add_argument('--hand-mapping', choices=('auto', 'same', 'crossed'), default='auto')
        parser.add_argument('--rotation-mode', choices=('auto', *POSE_FRAMES), default='auto')
        options, _ = parser.parse_known_args(argv)
        return options

    def test_default_still_resolves_to_facing_crossed_hands_and_mirrored_rotation(self):
        options = self.launch_options()
        self.assertEqual(options.mode, 'mirror_same_direction')
        self.assertEqual(controller_sides(options.mode, options.hand_mapping),
                         {'left': 'right', 'right': 'left'})
        self.assertEqual(rotation_mapping(options.mode, options.rotation_mode), 'mirror')

    def test_first_person_override_selects_same_side_hands_and_first_person_rotation(self):
        options = self.launch_options('--mode', 'first_person')
        self.assertEqual(options.mode, 'first_person')
        self.assertEqual(controller_sides(options.mode, options.hand_mapping),
                         {'left': 'left', 'right': 'right'})
        self.assertEqual(rotation_mapping(options.mode, options.rotation_mode), 'first_person')

    def test_explicit_user_hand_and_rotation_mappings_override_automatic_mode(self):
        options = self.launch_options('--mode', 'first_person',
                                      '--hand-mapping', 'crossed', '--rotation-mode', 'mirror')
        self.assertEqual(options.mode, 'first_person')
        self.assertEqual(options.hand_mapping, 'crossed')
        self.assertEqual(options.rotation_mode, 'mirror')
        self.assertEqual(controller_sides(options.mode, options.hand_mapping),
                         {'left': 'right', 'right': 'left'})
        self.assertEqual(rotation_mapping(options.mode, options.rotation_mode), 'mirror')


if __name__ == '__main__':
    unittest.main()
