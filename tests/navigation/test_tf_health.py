"""TF diagnostics preserve timing gates and expose failures without ROS."""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

from fairino_slam.navigation import velocity_gate


def transform(stamp):
    if isinstance(stamp, float) and not math.isfinite(stamp):
        return NS(header=NS(stamp=NS(sec=stamp, nanosec=0)))
    sec = int(stamp)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=round((stamp-sec)*1e9))))


class Buffer:
    def __init__(self, combined=9.9, map_odom=10.3, odom_base=9.9):
        self.values = {('map', 'base_footprint'): combined,
                       ('map', 'odom'): map_odom,
                       ('odom', 'base_footprint'): odom_base}
        self.calls = []

    def lookup_transform(self, parent, child, query_time):
        self.calls.append((parent, child, query_time))
        value = self.values[parent, child]
        if isinstance(value, Exception):
            raise value
        return transform(value)


class TfHealthTests(unittest.TestCase):
    def health(self, buffer=None, now=10., initial=9.):
        return velocity_gate.localization_tf_health(buffer or Buffer(), now, initial, object())

    def test_amcl_future_edge_keeps_current_combined_pose_usable(self):
        buffer = Buffer(map_odom=10.3)
        query = object()
        result = velocity_gate.localization_tf_health(buffer, 10., 9., query)
        self.assertTrue(result['ready'])
        self.assertIsNone(result['reason'])
        self.assertEqual(result['max_combined_age'], 1.)
        self.assertAlmostEqual(result['transforms']['map_odom']['age'], -.3)
        self.assertAlmostEqual(result['transforms']['map_base']['age'], .1)
        self.assertEqual({(a, b) for a, b, _ in buffer.calls}, set(buffer.values))
        self.assertTrue(all(t is query for _, _, t in buffer.calls))

    def test_combined_staleness_cannot_be_hidden_by_new_amcl_edge(self):
        result = self.health(Buffer(combined=8.9, map_odom=10.3), initial=8.)
        self.assertFalse(result['ready'])
        self.assertIn('已过期', result['reason'])
        self.assertIn('1.100', result['reason'])
        self.assertIn('要求时间差小于 1.0 秒', result['reason'])
        self.assertAlmostEqual(result['transforms']['map_base']['age'], 1.1)

    def test_exact_one_second_and_excess_future_offset_are_rejected(self):
        for stamp, reason in ((9., '已过期'), (8.9, '已过期'),
                              (11., '超前'), (11.1, '超前')):
            with self.subTest(stamp=stamp):
                result = self.health(Buffer(combined=stamp))
                self.assertFalse(result['ready'])
                self.assertIn(reason, result['reason'])
        for stamp in (9.001, 10.999):
            self.assertTrue(self.health(Buffer(combined=stamp))['ready'])

    def test_transform_before_initialization_is_distinguished(self):
        result = self.health(Buffer(combined=9.8), initial=9.9)
        self.assertFalse(result['ready'])
        self.assertIn('早于本次初始化', result['reason'])
        self.assertEqual(result['initial_pose_stamp'], 9.9)
        self.assertIn('TF=9.800', result['reason'])
        self.assertTrue(self.health(Buffer(combined=9.9), initial=9.9)['ready'])

    def test_lookup_errors_preserve_details_and_other_edge_evidence(self):
        buffer = Buffer(combined=RuntimeError('no transform chain'))
        result = self.health(buffer)
        self.assertFalse(result['ready'])
        self.assertIn('RuntimeError: no transform chain', result['reason'])
        self.assertEqual(len(buffer.calls), 3)
        self.assertIsNone(result['transforms']['map_base']['stamp'])
        self.assertAlmostEqual(result['transforms']['odom_base']['stamp'], 9.9)

    def test_individual_edge_failure_does_not_change_combined_gate(self):
        result = self.health(Buffer(map_odom=RuntimeError('edge unavailable')))
        self.assertTrue(result['ready'])
        self.assertIn('edge unavailable', result['transforms']['map_odom']['error'])

    def test_missing_or_nonfinite_times_fail_with_json_safe_diagnostics(self):
        for initial in (None, 0., False, float('nan'), float('inf')):
            with self.subTest(initial=initial):
                result = self.health(initial=initial)
                self.assertFalse(result['ready'])
                self.assertIn('初始位姿时间戳无效', result['reason'])
                json.dumps(result, allow_nan=False)
        for now, stamp in ((float('nan'), 9.9), (10., float('inf'))):
            with self.subTest(now=now, stamp=stamp):
                result = self.health(Buffer(combined=stamp), now=now)
                self.assertFalse(result['ready'])
                self.assertIn('有限数值', result['reason'])
                json.dumps(result, allow_nan=False)


class FlatRuntimeImportTests(unittest.TestCase):
    def test_navigation_monitor_wins_over_old_mapping_copy(self):
        with tempfile.TemporaryDirectory(prefix='fairino tf imports ') as folder:
            home = Path(folder)
            navigation = home/'navigation runtime'
            mapping = home/'.local/share/fairino-map-control'
            navigation.mkdir()
            mapping.mkdir(parents=True)
            source = Path(velocity_gate.__file__).read_text()
            (navigation/'velocity_gate.py').write_text(source)
            (navigation/'tf_receiver.py').write_text(
                Path(velocity_gate.__file__).with_name('tf_receiver.py').read_text())
            (navigation/'remote_stream.py').write_text(
                "class CallbackReceiver: pass\nclass LocalizationMonitor:\n    origin = 'navigation'\n"
                "def yaw(value): return 0.\n")
            (mapping/'remote_stream.py').write_text(
                "raise AssertionError('stale mapping monitor was imported')\n")
            (mapping/'localization.py').write_text(
                "def read_session(): return {'origin': 'mapping'}\n")
            (navigation/'chassis_receiver.py').write_text(
                Path(velocity_gate.__file__).with_name('chassis_receiver.py').read_text())
            (navigation/'coordination.py').write_text(
                Path(velocity_gate.__file__).with_name('coordination.py').read_text())
            (navigation/'motion_feedback.py').write_text(
                (Path(velocity_gate.__file__).parents[1]/'chassis/motion_feedback.py').read_text())
            (navigation/'motion.py').write_text("class CommandGate: pass\n")
            (navigation/'lifecycle.py').write_text("class LifecycleMonitor: pass\n")
            (navigation/'runtime_policy.py').write_text("TF_MAX_AGE = 1.\n")
            (navigation/'control_core.py').write_text(
                "DEFAULT_LINEAR = .06\nDEFAULT_ANGULAR = .1\nFEEDBACK_TIMEOUT = .4\n")
            code = (
                "from pathlib import Path\nimport runpy, sys\n"
                "Path.home = classmethod(lambda cls: Path(sys.argv[1]))\n"
                "result = runpy.run_path(sys.argv[2], run_name='flat_probe')\n"
                "assert result['LocalizationMonitor'].origin == 'navigation'\n"
                "assert result['read_session']()['origin'] == 'mapping'\n"
                "print('navigation monitor; mapping localization fallback')\n")
            completed = subprocess.run(
                [sys.executable, '-B', '-c', code, str(home),
                 str(navigation/'velocity_gate.py')],
                capture_output=True, text=True, timeout=10)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn('navigation monitor', completed.stdout)


if __name__ == '__main__':
    unittest.main()
