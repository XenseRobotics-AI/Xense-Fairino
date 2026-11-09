"""Console routing checks, with no device or display connection."""
import importlib
from pathlib import Path
import queue
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from apps.workbench.slam_support import ensure_slam


class SlamEntryTests(unittest.TestCase):
    def test_legacy_commands_use_the_same_backend_as_bundled_cli(self):
        ensure_slam()
        pairs = {
            'build_navigation': 'deployment.build_navigation',
            'install_navigation': 'deployment.install_navigation',
            'install_map_mode': 'deployment.install_map_mode',
            'install_control_stack': 'deployment.install_control_stack',
            'map_mode': 'mapping.client',
            'navigation_mode': 'navigation.client',
        }
        for old, new in pairs.items():
            with self.subTest(command=old):
                self.assertIs(importlib.import_module('tools.' + old),
                              importlib.import_module('fairino_slam.' + new))

    def test_configuration_comes_from_bundled_slam_and_uses_new_plugins(self):
        ensure_slam()
        from fairino_slam.paths import CONFIG_ROOT
        from tools.build_navigation import build
        self.assertEqual(CONFIG_ROOT, ROOT / 'modules/fairino_slam/config')
        profile = build()
        controller = profile['controller_server']['ros__parameters']
        self.assertEqual(controller['controller_frequency'], 3.)
        self.assertIn('Lookahead', controller['FollowPath']['plugin'])
        self.assertIn('Straight', profile['planner_server']['ros__parameters']['GridBased']['plugin'])


class ModelSourceTests(unittest.TestCase):
    def app(self, demo):
        from apps.workbench.app import Workbench
        hardware = {'joint_names': [f'j{i}' for i in range(1, 21)],
                    'position': [.1] * 20, 'enabled': [0] * 20,
                    'errorcode': [0] * 20, 'error': 0}
        app = NS(events=queue.Queue(), routes=Mock(), observers={'upper': NS(snapshot=lambda: {
            'hardware': hardware, 'age': 0., 'received': time.monotonic()})},
            hardware_labels={'upper': Mock()}, gripper=NS(session=None),
            teleop=NS(model_snapshot=lambda: demo, gripper_feedback={}),
            model=NS(update=Mock(), model=NS(body_state=Mock(return_value=None))),
            body_readout=Mock(), message=Mock(), entries={}, leases={},
            closing=False, retired_threads=[], refresh_control_access=Mock(),
            args=NS(live_arms=True))
        self.assertTrue(Workbench.tick(app))
        return app, hardware

    def test_demo_model_cannot_be_overwritten_by_live_observer(self):
        simulated = {'joint_names': [f'j{i}' for i in range(1, 21)], 'position': [.7] * 20}
        demo = dict(hardware=simulated, grippers={'left': .4}, fresh=True,
                    active=True, wrist_collision={'left': True})
        app, real = self.app(demo)
        self.assertIs(app.model.update.call_args.args[0], simulated)
        self.assertTrue(app.model.update.call_args.kwargs['demo'])
        self.assertEqual(app.model.update.call_args.kwargs['wrist_collision'], {'left': True})
        app.model.model.body_state.assert_called_once_with(simulated)

    def test_stopped_demo_stays_demo_until_user_switches_source(self):
        app, _ = self.app(dict(hardware=None, grippers={}, fresh=False, active=False))
        self.assertIsNone(app.model.update.call_args.args[0])
        self.assertFalse(app.model.update.call_args.kwargs['live_arms'])
        self.assertTrue(app.model.update.call_args.kwargs['demo'])

    def test_robot_mode_keeps_real_telemetry(self):
        app, hardware = self.app(None)
        self.assertIs(app.model.update.call_args.args[0], hardware)
        self.assertNotIn('demo', app.model.update.call_args.kwargs)


if __name__ == '__main__':
    unittest.main()
