"""The matching watchdog uses the same thresholds configured in AMCL."""
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
import yaml

from fairino_slam.mapping import localization
from . import test_localization as fixtures


class SessionParameterTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.LifecycleTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)

    def test_start_records_applied_amcl_thresholds(self):
        data = yaml.safe_load(localization.PROFILE.read_text())
        params = data['amcl']['ros__parameters']
        params.update(update_min_d=.17, update_min_a=.13, transform_tolerance=.4)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'localization.yaml'
            path.write_text(yaml.safe_dump(data))
            with patch.object(localization, 'PROFILE', path):
                state = self.fixture.control.start(self.fixture.map)
        expected = {name: params[name] for name in
                    ('update_min_d', 'update_min_a', 'transform_tolerance')}
        self.assertEqual(state['amcl_parameters'], expected)
        self.assertEqual(localization.read_session()['amcl_parameters'], expected)
        applied = next(value for name, value in self.fixture.ros.events if name == '/amcl')
        self.assertEqual({key: applied[key] for key in expected}, expected)

    def test_invalid_parameters_fail_before_any_ros_changes(self):
        original = yaml.safe_load(localization.PROFILE.read_text())
        for value in (None, True, -1, float('nan'), float('inf')):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as directory:
                original['amcl']['ros__parameters']['update_min_d'] = value
                path = Path(directory)/'localization.yaml'
                path.write_text(yaml.safe_dump(original))
                with patch.object(localization, 'PROFILE', path):
                    with self.assertRaisesRegex(ValueError, 'update_min_d'):
                        self.fixture.control.start(self.fixture.map)
                self.assertEqual(self.fixture.ros.events, [])
