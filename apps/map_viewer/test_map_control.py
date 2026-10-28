import unittest
import time
import json
import math
from pathlib import Path
import tempfile
from unittest.mock import Mock, patch

from map_control_remote import ROS, guard_reason, validate_name


class MappingInputTests(unittest.TestCase):
    def valid(self):
        return {
            '/wheel/odom': (10, {'stamp_age': .01}),
            '/odom': (10, {'stamp_age': .01, 'variance': .01, 'vx': 0, 'vy': 0, 'wz': 0}),
            '/scan': (10, {'stamp_age': .02}),
        }

    def test_stationary_zero_velocity_is_valid(self):
        self.assertIsNone(guard_reason('/wheel/odom', self.valid(), 10.1))

    def test_filtered_output_cannot_hide_wheel_dropout(self):
        latest = self.valid()
        del latest['/wheel/odom']
        self.assertIn('轮速输入', guard_reason('/wheel/odom', latest, 10.1))

    def test_stale_arrival_or_timestamp_blocks_mapping(self):
        latest = self.valid()
        latest['/wheel/odom'] = (9, {'stamp_age': 0})
        self.assertIsNotNone(guard_reason('/wheel/odom', latest, 10.1))
        latest = self.valid()
        latest['/wheel/odom'][1]['stamp_age'] = -2
        self.assertIn('时间戳', guard_reason('/wheel/odom', latest, 10.1))

    def test_large_covariance_and_nonfinite_output_block_mapping(self):
        latest = self.valid()
        latest['/odom'][1]['variance'] = 1e15
        self.assertIn('不确定度', guard_reason('/wheel/odom', latest, 10.1))
        latest['/odom'][1]['variance'] = float('nan')
        self.assertIn('无效', guard_reason('/wheel/odom', latest, 10.1))

    def test_raw_mode_needs_no_ekf_and_does_not_apply_ekf_variance_threshold(self):
        latest = self.valid()
        latest['/odom'][1]['variance'] = 42.
        self.assertIsNone(guard_reason('/wheel/odom', latest, 10.1, mode='raw'))
        del latest['/wheel/odom']
        self.assertIn('轮速输入', guard_reason('/wheel/odom', latest, 10.1, mode='raw'))

    def test_original_stamp_ages_after_receipt_even_when_receipt_itself_is_fresh(self):
        latest = self.valid()
        latest['/odom'][1]['stamp_age'] = .45
        self.assertIn('时间戳', guard_reason('/wheel/odom', latest, 10.1, mode='raw'))

    def test_map_names_cannot_escape_new_directory(self):
        self.assertEqual(validate_name('room_20261016-01'), 'room_20261016-01')
        for name in ('../active/map', '/tmp/map', 'map\\image', '', '.', '..',
                     'x;rm', 'x\ny', ' x', 'x ', 'x\ty', 'x\u3000y',
                     'x$(id)', 'x`id`', 'x&y', 'x|y', 'x*y', 'x?y',
                     'x"y', "x'y", '-room', '_room', '图'*81, '\u4dc0', '😀'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                validate_name(name)

    def test_chinese_and_ascii_map_names_preserve_characters_and_length(self):
        for name in ('碧迪15楼实验室', '实验室_15F-01', 'room_20261016-01', 'A', '0',
                     '图'*80, 'a'*80, '\u3400', '\u4dbf', '\u4e00', '\u9fff'):
            with self.subTest(name=name):
                self.assertEqual(validate_name(name), name)

    def test_invalid_save_name_does_not_pause_or_touch_ros(self):
        ros = Mock()
        for name in ('../active/map', '碧迪 15楼实验室', 'map.yaml', '图'*81):
            with self.subTest(name=name), self.assertRaises(ValueError):
                ROS.save(ros, name)
        ros.pause.assert_not_called()
        ros.params.assert_not_called()
        ros.spin.assert_not_called()
        ros.call.assert_not_called()


class DisplayHealthTests(unittest.TestCase):
    def test_raw_mode_does_not_claim_ekf_or_apply_its_variance_gate(self):
        from viewer import health
        state = {'received': time.monotonic(), 'frame': {'map_mode': 'mapping',
                 'odometry_mode': 'raw', 'odometry_source': '/wheel/odom',
                 'scan_match': {'source_confirmed': True, 'age': .1},
                 'topics': {'/wheel/odom': {'age': .01}, '/map': {'age': .1},
                            '/odom': {'age': .01, 'latest': {'position_variance': 42.}}}}}
        text, color = health(state)
        self.assertIn('原始轮速直通', text)
        self.assertEqual(color, '#34745c')
        state['frame']['mapping_paused'] = True
        text, color = health(state)
        self.assertIn('不再采集', text)
        self.assertNotIn('超', text)
        self.assertNotIn('EKF 位置方差', text)

    def test_pause_remains_visible_and_reports_current_max_xy_variance(self):
        from viewer import health
        state = {'received': time.monotonic(), 'frame': {'map_mode': 'mapping',
                 'mapping_paused': True, 'topics': {'/odom': {'age': .01,
                 'latest': {'x_variance': .1, 'position_variance': 1.2}}}}}
        text, color = health(state)
        self.assertIn('不再采集', text)
        self.assertIn('1.20 m²', text)
        self.assertEqual(color, '#b53a36')
        state['frame']['topics']['/odom']['age'] = 5
        text, _ = health(state)
        self.assertNotIn('1.20', text)  # Old covariance must not masquerade as live.
        self.assertIn('继续建图', text)

    def test_idle_view_does_not_claim_the_odometry_is_broken(self):
        from viewer import health
        text, _ = health({'received': time.monotonic(), 'frame': {'map_mode': 'empty'}})
        self.assertIn('启动底盘', text)
        self.assertNotIn('修复', text)

    def valid_mapping(self):
        return {'received': time.monotonic(), 'frame': {'map_mode': 'mapping',
                'odometry_mode': 'raw', 'odometry_source': '/wheel/odom',
                'topics': {'/wheel/odom': {'age': .01}, '/map': {'age': .01},
                           '/odom': {'age': .01, 'latest': {'position_variance': 0.}}}}}

    def test_republished_map_and_tf_do_not_claim_successful_matching(self):
        from viewer import health
        state = self.valid_mapping()
        text, color = health(state)
        self.assertIn('等待', text)
        self.assertNotEqual(color, '#34745c')
        state['frame']['scan_match'] = {'source_confirmed': True, 'age': 20.,
                                      'travel_since_match': .6, 'rotation_since_match': 0.}
        text, color = health(state)
        self.assertIn('未获新雷达匹配', text)
        self.assertEqual(color, '#b53a36')

    def test_stationary_old_match_waits_for_motion_without_false_failure(self):
        from viewer import health
        state = self.valid_mapping()
        state['frame']['scan_match'] = {'source_confirmed': True, 'age': 120.,
                                      'travel_since_match': .001, 'rotation_since_match': .001}
        text, color = health(state)
        self.assertIn('静止', text)
        self.assertEqual(color, '#526579')

    def test_motion_without_even_a_first_match_is_not_left_waiting_forever(self):
        from viewer import health
        state = self.valid_mapping()
        state['frame']['scan_match'] = {'source_confirmed': True, 'age': None,
                                      'travel_since_match': .3, 'waiting_age': 5.}
        text, color = health(state)
        self.assertIn('运动后仍未收到', text)
        self.assertEqual(color, '#b53a36')


class ScanMatchMonitoringTests(unittest.TestCase):
    def test_short_return_trip_and_full_turn_are_still_detected_as_motion(self):
        from remote_stream import ScanMatchMonitor
        monitor = ScanMatchMonitor()
        monitor.start_session('A', 100)
        monitor.on_odometry((0., 0., 0.))
        self.assertTrue(monitor.accept(101., .01, (0., 0., 0.), 10.))
        for x in (.06, .12, .06, 0.):
            monitor.on_odometry((x, 0., 0.))
        self.assertEqual(monitor.status(13.)['state'], 'stale')
        monitor.accept(102., .01, (0., 0., 0.), 14.)
        for angle in (.5, 1., 2., 3., -2., -1., 0.):
            monitor.on_odometry((0., 0., angle))
        self.assertGreater(monitor.status(17.)['rotation_since_match'], 6.)
        self.assertEqual(monitor.status(17.)['state'], 'stale')

    def test_session_stale_duplicate_and_future_messages_do_not_look_fresh(self):
        from remote_stream import ScanMatchMonitor
        monitor = ScanMatchMonitor()
        monitor.start_session('A', 100)
        self.assertFalse(monitor.accept(99., 0., (0., 0., 0.), 10.))
        self.assertFalse(monitor.accept(101., -.6, (0., 0., 0.), 10.))
        self.assertFalse(monitor.accept(101., 3., (0., 0., 0.), 10.))
        self.assertTrue(monitor.accept(101., .2, (0., 0., 0.), 10.))
        self.assertFalse(monitor.accept(101., 0., (0., 0., 0.), 12.))
        self.assertAlmostEqual(monitor.status(11.)['age'], 1.2)
        monitor.start_session('B', 200)
        self.assertIsNone(monitor.status(12.)['age'])
        self.assertEqual(monitor.status(12.)['count_observed'], 0)
        self.assertFalse(monitor.accept(102., 0., (0., 0., 0.), 12.))

    def test_static_noise_does_not_accumulate_into_a_movement_alarm(self):
        from remote_stream import ScanMatchMonitor
        monitor = ScanMatchMonitor()
        monitor.on_odometry((0., 0., 0.), (0., 0., 0.))
        monitor.accept(101., 0., (0., 0., 0.), 10.)
        for index in range(3000):
            monitor.on_odometry((index%2*.00001, 0., index%2*.00001), (0., 0., 0.))
        self.assertEqual(monitor.status(100.)['state'], 'waiting_motion')


class MappingRuntimeTests(unittest.TestCase):
    def test_missing_or_unrepaired_runtime_never_falls_back_to_system_slam(self):
        import map_control_remote as mapping
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile = root/'mapping.yaml'
            runtime = root/'runtime'
            with patch.object(mapping, 'MAPPING_PROFILE', profile), patch.object(mapping, 'SLAM_RUNTIME', runtime):
                with self.assertRaisesRegex(RuntimeError, '尚未部署'):
                    mapping.mapping_command()
                (runtime/'bin').mkdir(parents=True)
                (runtime/'bin/sync_slam_toolbox_node').touch()
                profile.write_text('slam_toolbox: {}')
                (runtime/'manifest.json').write_text(json.dumps({'scan_gate': 'old'}))
                with self.assertRaisesRegex(RuntimeError, '未确认'):
                    mapping.mapping_command()
                (runtime/'manifest.json').write_text(json.dumps({'scan_gate': 'backend_motion_gate_v1'}))
                command = mapping.mapping_command()
                self.assertEqual(command[0], str(runtime/'bin/sync_slam_toolbox_node'))
                self.assertEqual(command[-1], str(profile))


if __name__ == '__main__':
    unittest.main()
