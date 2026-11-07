"""Configuration edits must reach generated Nav2 parameters without code edits."""
import copy
import json
import math
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import yaml

from fairino_slam import configuration
from fairino_slam.navigation.profile import build_profile
from fairino_slam.navigation.settings import validate_navigation


POINTS = ((-.375, -.3), (.375, -.3), (.375, .3), (-.375, .3))


class NavigationConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.chassis = configuration.load_chassis()
        self.navigation = configuration.load_navigation()

    def test_yaml_edits_control_speed_acceleration_timing_and_geometry(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / 'config'
            shutil.copytree(configuration.CONFIG_ROOT, root)
            chassis = copy.deepcopy(self.chassis)
            chassis['speed'].update(default_linear=.08, default_angular=.12)
            chassis['limits']['linear']['max_acceleration'] = .15
            chassis['limits']['angular']['max_acceleration'] = .3
            navigation = copy.deepcopy(self.navigation)
            controller = navigation['nav2']['controller_server']['ros__parameters']
            controller['controller_frequency'] = 20.
            controller['goal_checker']['xy_goal_tolerance'] = .07
            navigation['geometry']['footprint_padding'] = .04
            navigation['geometry']['minimum_inflation_radius'] = .9
            navigation['nav2']['local_costmap']['local_costmap']['ros__parameters']['width'] = 6
            for name, data in (('chassis.yaml', chassis), ('navigation.yaml', navigation)):
                (root / name).write_text(yaml.safe_dump(data, sort_keys=False))
            with patch.object(configuration, 'CONFIG_ROOT', root):
                actual = build_profile(POINTS, .08, .12)
            result = actual['controller_server']['ros__parameters']
            follow = result['FollowPath']
            self.assertEqual(follow['max_linear_speed'], .08)
            self.assertEqual(follow['max_angular_speed'], .12)
            self.assertEqual(follow['linear_acceleration'], .15)
            self.assertEqual(follow['angular_acceleration'], .3)
            self.assertEqual(follow['control_period'], .05)
            self.assertEqual(result['goal_checker']['xy_goal_tolerance'], .07)
            local = actual['local_costmap']['local_costmap']['ros__parameters']
            global_map = actual['global_costmap']['global_costmap']['ros__parameters']
            self.assertEqual(local['width'], 6)
            self.assertEqual(local['footprint_padding'], .04)
            self.assertEqual(local['inflation_layer']['inflation_radius'], .9)
            radius = math.hypot(.375 + .04, .3 + .04)
            circle = json.loads(global_map['footprint'])
            for a, b in zip(circle, circle[1:]+circle[:1]):
                edge_distance = (a[0]*b[1]-a[1]*b[0])/math.hypot(b[0]-a[0], b[1]-a[1])
                self.assertGreaterEqual(edge_distance, radius)
                self.assertLess(edge_distance-radius, 1e-5)
            self.assertEqual(global_map['footprint_padding'], 0.)

    def test_straight_line_switch_reaches_both_heading_profiles(self):
        for enabled in (True, False):
            for mode in ('hold_initial', 'path_tangent'):
                with self.subTest(enabled=enabled, mode=mode):
                    data = copy.deepcopy(self.navigation)
                    data['nav2']['planner_server']['ros__parameters']['GridBased']['straight_line_enabled'] = enabled
                    data['nav2']['controller_server']['ros__parameters']['FollowPath']['heading_mode'] = mode
                    validate_navigation(data, self.chassis)
                    with patch('fairino_slam.navigation.profile.load_navigation', return_value=data):
                        result = build_profile(POINTS, .08, .12)
                    planner = result['planner_server']['ros__parameters']['GridBased']
                    self.assertEqual(planner['plugin'], 'fairino_straight_planner::StraightLinePlanner')
                    self.assertIs(planner['straight_line_enabled'], enabled)
                    self.assertFalse(planner['use_final_approach_orientation'])
                    self.assertEqual(planner['cost_travel_multiplier'], 2.)
                    self.assertEqual(planner['smoother']['max_iterations'], 0)

    def test_straight_line_switch_requires_explicit_boolean(self):
        for value in ('false', 0, 1, None):
            with self.subTest(value=value):
                data = copy.deepcopy(self.navigation)
                data['nav2']['planner_server']['ros__parameters']['GridBased']['straight_line_enabled'] = value
                with self.assertRaisesRegex(ValueError, 'straight_line_enabled'):
                    validate_navigation(data, self.chassis)

    def test_coordination_config_reaches_profile_with_shared_feedback_timeout(self):
        chassis = copy.deepcopy(self.chassis)
        chassis['timeouts']['feedback'] = .35
        settings = copy.deepcopy(self.navigation)
        desired = dict(coordination_linear_speed=.015, coordination_angular_speed=.03,
                       coordination_wait_timeout=12., coordination_direction_tolerance=.3)
        settings['nav2']['controller_server']['ros__parameters']['FollowPath'].update(desired)
        validate_navigation(settings, chassis)
        with patch('fairino_slam.navigation.profile.load_chassis', return_value=chassis), \
             patch('fairino_slam.navigation.profile.load_navigation', return_value=settings):
            result = build_profile(POINTS, .08, .12)['controller_server']['ros__parameters']['FollowPath']
        for key, value in desired.items():
            self.assertEqual(result[key], value)
        self.assertEqual(result['chassis_feedback_timeout'], .35)

    def test_coordination_bounds_reject_unsafe_or_ineffective_settings(self):
        cases = (
            ('coordination_linear_speed', 1e-4),
            ('coordination_linear_speed', self.chassis['speed']['default_linear']+.01),
            ('coordination_angular_speed', 1e-4),
            ('coordination_angular_speed', self.chassis['speed']['default_angular']+.01),
            ('coordination_direction_tolerance', 0.),
            ('coordination_direction_tolerance', math.pi/2),
            ('coordination_wait_timeout', self.chassis['timeouts']['feedback']),
            ('coordination_wait_timeout', 61.),
            ('coordination_wait_timeout', self.navigation['nav2']['controller_server']['ros__parameters']
             ['progress_checker']['movement_time_allowance']),
        )
        for field, value in cases:
            with self.subTest(field=field, value=value):
                settings = copy.deepcopy(self.navigation)
                settings['nav2']['controller_server']['ros__parameters']['FollowPath'][field] = value
                with self.assertRaisesRegex(ValueError, field):
                    validate_navigation(settings, self.chassis)

    def test_requested_navigation_speed_cannot_be_below_coordination_cap(self):
        for linear, angular, field in ((.01, .1, 'coordination_linear_speed'),
                                       (.06, .02, 'coordination_angular_speed')):
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, field):
                build_profile(POINTS, linear, angular)

    def test_invalid_numeric_parameters_fail_with_field_name(self):
        paths = [
            (('geometry', 'footprint_padding'), -1.),
            (('nav2', 'controller_server', 'ros__parameters', 'controller_frequency'), 0),
            (('nav2', 'controller_server', 'ros__parameters', 'controller_frequency'), float('nan')),
            (('nav2', 'controller_server', 'ros__parameters', 'controller_frequency'), True),
            (('nav2', 'controller_server', 'ros__parameters', 'FollowPath', 'position_gain'), float('nan')),
            (('nav2', 'controller_server', 'ros__parameters', 'FollowPath', 'lookahead_distance'), 0),
            (('nav2', 'controller_server', 'ros__parameters', 'goal_checker', 'xy_goal_tolerance'), -1.),
            (('nav2', 'local_costmap', 'local_costmap', 'ros__parameters', 'width'), -1),
        ]
        for path, value in paths:
            with self.subTest(path=path, value=value):
                data = copy.deepcopy(self.navigation)
                target = data
                for part in path[:-1]:
                    target = target[part]
                target[path[-1]] = value
                with self.assertRaisesRegex(ValueError, path[-1]):
                    validate_navigation(data, self.chassis)

    def test_heading_modes_select_geometry_and_preserve_requested_goal_heading(self):
        for mode, vertices in (('hold_initial', 64), ('path_tangent', 4)):
            with self.subTest(mode=mode):
                data = copy.deepcopy(self.navigation)
                data['nav2']['controller_server']['ros__parameters']['FollowPath']['heading_mode'] = mode
                validate_navigation(data, self.chassis)
                with patch('fairino_slam.navigation.profile.load_navigation', return_value=data):
                    result = build_profile(POINTS, .06, .1)
                follower = result['controller_server']['ros__parameters']['FollowPath']
                self.assertEqual(follower['heading_mode'], mode)
                global_map = result['global_costmap']['global_costmap']['ros__parameters']
                local = result['local_costmap']['local_costmap']['ros__parameters']
                self.assertEqual(len(json.loads(global_map['footprint'])), vertices)
                self.assertEqual(json.loads(local['footprint']), [list(p) for p in POINTS])
                self.assertEqual(global_map['footprint_padding'], 0.)
                self.assertFalse(result['planner_server']['ros__parameters']['GridBased']['use_final_approach_orientation'])

    def test_unknown_heading_modes_fail_before_deployment(self):
        for mode in ('tangent', 'path', 'HOLD_INITIAL', '', None, True):
            with self.subTest(mode=mode):
                data = copy.deepcopy(self.navigation)
                data['nav2']['controller_server']['ros__parameters']['FollowPath']['heading_mode'] = mode
                with self.assertRaisesRegex(ValueError, 'heading_mode'):
                    validate_navigation(data, self.chassis)

    def test_integer_yaml_numbers_generate_ros_double_parameters(self):
        settings = copy.deepcopy(self.navigation)
        controller = settings['nav2']['controller_server']['ros__parameters']
        controller['controller_frequency'] = 20
        controller['min_x_velocity_threshold'] = 0
        controller['goal_checker']['yaw_goal_tolerance'] = 1
        controller['FollowPath']['heading_gain'] = 2
        with patch('fairino_slam.navigation.profile.load_navigation', return_value=settings):
            result = build_profile(POINTS, .06, .1)['controller_server']['ros__parameters']
        for value in (result['controller_frequency'], result['min_x_velocity_threshold'],
                      result['goal_checker']['yaw_goal_tolerance'],
                      result['FollowPath']['yaw_goal_tolerance'], result['FollowPath']['heading_gain']):
            self.assertIs(type(value), float)
        self.assertIs(type(result['goal_checker']['stateful']), bool)

    def test_missing_unknown_and_derived_parameters_are_rejected(self):
        data = copy.deepcopy(self.navigation)
        del data['nav2']['controller_server']['ros__parameters']['controller_frequency']
        with self.assertRaisesRegex(ValueError, 'controller_frequency'):
            validate_navigation(data, self.chassis)
        for field in ('max_linear_speed', 'linear_acceleration', 'xy_goal_tolerance', 'control_period', 'chassis_feedback_timeout', 'typo'):
            data = copy.deepcopy(self.navigation)
            data['nav2']['controller_server']['ros__parameters']['FollowPath'][field] = .1
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, field):
                validate_navigation(data, self.chassis)
        data = copy.deepcopy(self.navigation)
        data['extra'] = {}
        with self.assertRaisesRegex(ValueError, 'extra'):
            validate_navigation(data, self.chassis)

    def test_incompatible_ranges_are_rejected_before_deployment(self):
        data = copy.deepcopy(self.navigation)
        scan = data['nav2']['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['front']
        scan['obstacle_min_range'] = scan['obstacle_max_range'] + 1
        with self.assertRaisesRegex(ValueError, 'obstacle_min_range'):
            validate_navigation(data, self.chassis)
        data = copy.deepcopy(self.navigation)
        data['nav2']['controller_server']['ros__parameters']['controller_frequency'] = 1.
        with self.assertRaisesRegex(ValueError, 'controller_frequency'):
            validate_navigation(data, self.chassis)
        data = copy.deepcopy(self.navigation)
        data['nav2']['controller_server']['ros__parameters']['min_y_velocity_threshold'] = self.chassis['speed']['default_linear']
        with self.assertRaisesRegex(ValueError, 'min_y_velocity_threshold'):
            validate_navigation(data, self.chassis)

    def test_control_period_must_be_shorter_than_600ms_command_timeout(self):
        for period in (.5, .6, .61):
            with self.subTest(period=period):
                data = copy.deepcopy(self.navigation)
                data['nav2']['controller_server']['ros__parameters']['controller_frequency'] = 1. / period
                if period < .6:
                    validate_navigation(data, self.chassis)
                else:
                    with self.assertRaisesRegex(ValueError, '控制周期必须短于 0.6 秒'):
                        validate_navigation(data, self.chassis)

    def test_build_rejects_invalid_planar_speed_arguments(self):
        for linear, angular in ((True, .1), (float('inf'), .1), (0, .1),
                                (.06, float('nan')),
                                (self.chassis['speed']['max_linear'] + .1, .1)):
            with self.subTest(linear=linear, angular=angular), self.assertRaises(ValueError):
                build_profile(POINTS, linear, angular)


if __name__ == '__main__':
    unittest.main()
