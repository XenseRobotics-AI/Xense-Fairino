"""Invalid settings fail explicitly; file edits affect local and generated control."""
import copy
from pathlib import Path
import runpy
import shutil
import tempfile
import unittest
from unittest.mock import patch

import yaml

from fairino_slam import configuration
from fairino_slam.deployment.build_navigation import build, write_config
from fairino_slam.paths import PACKAGE_ROOT


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name) / 'config'
        shutil.copytree(configuration.CONFIG_ROOT, self.root)
        patcher = patch.object(configuration, 'CONFIG_ROOT', self.root)
        patcher.start()
        self.addCleanup(patcher.stop)

    def write(self, name, data):
        (self.root / name).write_text(yaml.safe_dump(data, sort_keys=False))

    def test_missing_config_does_not_fall_back_to_old_defaults(self):
        (self.root / 'chassis.yaml').unlink()
        with self.assertRaisesRegex(configuration.ConfigurationError, 'chassis.yaml'):
            configuration.load_chassis()

    def test_duplicate_keys_and_invalid_yaml_are_rejected_with_filename(self):
        for content in ('speed: 1\nspeed: 2\n', 'speed: [', '[]', '1: value'):
            with self.subTest(content=content):
                (self.root / 'chassis.yaml').write_text(content)
                with self.assertRaisesRegex(configuration.ConfigurationError, 'chassis.yaml'):
                    configuration.load_chassis()

    def test_invalid_chassis_numbers_and_ranges_are_rejected(self):
        original = configuration.load_chassis()
        for value in (True, '0.1', float('nan'), float('inf'), 0., -1.,
                      original['speed']['max_linear'] + 1.):
            data = copy.deepcopy(original)
            data['speed']['default_linear'] = value
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'chassis.yaml'):
                configuration.validate_chassis(data)
        data = copy.deepcopy(original)
        data['limits']['drive']['feedback_max_velocity'] = data['limits']['drive']['max_velocity'] / 2
        with self.assertRaisesRegex(ValueError, 'feedback_max_velocity'):
            configuration.validate_chassis(data)

    def test_missing_misspelled_and_non_mapping_sections_are_rejected(self):
        original = configuration.load_chassis()
        for mode in ('missing', 'typo', 'type'):
            data = copy.deepcopy(original)
            if mode == 'missing':
                del data['speed']['default_linear']
            elif mode == 'typo':
                data['speed']['defaut_linear'] = .06
            else:
                data['speed'] = None
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, 'speed'):
                configuration.validate_chassis(data)

    def test_file_edits_reach_local_core_and_generated_file(self):
        data = configuration.load_chassis()
        data['speed'].update(default_linear=.08, default_angular=.12)
        data['limits']['linear']['max_acceleration'] = .15
        self.write('chassis.yaml', data)
        core = runpy.run_path(str(PACKAGE_ROOT / 'chassis/control_core.py'))
        self.assertEqual(core['key_twist']('wj'), (.08, 0., .12))
        target = self.root.parent / 'generated/nav2.yaml'
        self.assertEqual(write_config(target), target)
        actual = yaml.safe_load(target.read_text())
        self.assertEqual(actual, build())
        follow = actual['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(follow['max_linear_speed'], core['DEFAULT_LINEAR'])
        self.assertEqual(follow['max_angular_speed'], core['DEFAULT_ANGULAR'])
        self.assertEqual(follow['linear_acceleration'], .15)
        self.assertFalse((self.root / 'nav2.yaml').exists())

    def test_invalid_mapping_and_localization_ranges_fail_before_use(self):
        for filename, node, minimum, maximum in (
                ('mapping.yaml', 'slam_toolbox', 'min_laser_range', 'max_laser_range'),
                ('localization.yaml', 'amcl', 'min_particles', 'max_particles')):
            data = configuration.load_ros_profile(filename)
            parameters = data[node]['ros__parameters']
            parameters[minimum] = parameters[maximum] + 1
            self.write(filename, data)
            with self.subTest(filename=filename), self.assertRaisesRegex(ValueError, minimum):
                configuration.load_ros_profile(filename)


if __name__ == '__main__':
    unittest.main()
