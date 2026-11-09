from fairino_slam.paths import RESOURCE_ROOT
from fairino_slam.configuration import load_navigation
import json
import math
import struct
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from fairino_slam.navigation.footprint import ChassisFootprint, FootprintRequest, selector
from fairino_slam.navigation.motion import CommandGate, bounded_twist
from fairino_slam.navigation.runtime_policy import COMMAND_TIMEOUT
from fairino_slam.navigation.profile import _global_footprint
from fairino_slam.chassis.control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR
from fairino_slam.deployment.build_navigation import build


class ChassisGeometryTests(unittest.TestCase):
    def test_runtime_urdf_projects_to_expected_chassis_rectangle(self):
        shape = selector(RESOURCE_ROOT/'models/amr_8dof.urdf').compute('chassis', FootprintRequest(12., {}))
        self.assertEqual(shape.points, ((-.375, -.3), (.375, -.3), (.375, .3), (-.375, .3)))
        self.assertEqual(shape.frame_id, 'base_link')
        self.assertEqual(shape.stamp, 12.)
        area = sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(shape.points, shape.points[1:]+shape.points[:1]))/2
        self.assertAlmostEqual(area, .45)

    def test_collision_origin_yaw_and_translation_are_applied(self):
        text = '<robot><link name="base_link"><collision><origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/><geometry><box size="2 1 0.5"/></geometry></collision></link></robot>'
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'test.urdf'
            path.write_text(text)
            points = ChassisFootprint(path).points
        self.assertEqual(points, ((.5, 1.), (1.5, 1.), (1.5, 3.), (.5, 3.)))

    def test_unknown_collision_geometry_never_becomes_empty_free_space(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'test.urdf'
            path.write_text('<robot><link name="base_link"><collision><geometry><mesh filename="missing.stl"/></geometry></collision></link></robot>')
            with self.assertRaisesRegex(ValueError, 'unsupported'):
                ChassisFootprint(path)
            path.write_text('<robot><link name="base_link"/></robot>')
            with self.assertRaises(ValueError):
                ChassisFootprint(path)


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.config = load_navigation()['nav2']

    def test_profile_is_generated_from_current_defaults(self):
        import yaml
        profile = build()
        self.assertEqual(yaml.safe_load(yaml.safe_dump(profile)), profile)
        controller = profile['controller_server']['ros__parameters']
        follower = controller['FollowPath']
        self.assertEqual(follower['max_linear_speed'], DEFAULT_LINEAR)
        self.assertEqual(follower['max_angular_speed'], DEFAULT_ANGULAR)
        self.assertEqual(follower['heading_mode'], self.config['controller_server']['ros__parameters']['FollowPath']['heading_mode'])
        self.assertEqual(follower['lookahead_distance'], self.config['controller_server']['ros__parameters']['FollowPath']['lookahead_distance'])
        self.assertLess(controller['min_y_velocity_threshold'], DEFAULT_LINEAR)
        self.assertLess(controller['goal_checker']['trans_stopped_velocity'], DEFAULT_LINEAR)

    def test_follower_uses_control_period_and_stopped_goal_checker(self):
        profile = build()
        controller = profile['controller_server']['ros__parameters']
        follower = controller['FollowPath']
        self.assertEqual(follower['plugin'], 'fairino_lookahead_controller::HolonomicLookaheadController')
        self.assertAlmostEqual(follower['control_period'] * controller['controller_frequency'], 1.)
        self.assertLess(follower['control_period'], COMMAND_TIMEOUT)
        self.assertGreaterEqual(follower['collision_horizon'], follower['control_period'])
        self.assertEqual(controller['goal_checker']['plugin'], 'nav2_controller::StoppedGoalChecker')
        self.assertFalse(controller['goal_checker']['stateful'])
        self.assertEqual(follower['xy_goal_tolerance'], controller['goal_checker']['xy_goal_tolerance'])
        self.assertEqual(follower['yaw_goal_tolerance'], controller['goal_checker']['yaw_goal_tolerance'])
        self.assertNotIn('trajectory_generator_name', follower)

    def test_smac_global_inscribed_radius_contains_every_rotated_local_corner(self):
        settings = load_navigation()
        settings['nav2']['controller_server']['ros__parameters']['FollowPath']['heading_mode'] = 'hold_initial'
        with patch('fairino_slam.navigation.profile.load_navigation', return_value=settings):
            profile = build()
        local = profile['local_costmap']['local_costmap']['ros__parameters']
        global_map = profile['global_costmap']['global_costmap']['ros__parameters']
        circle = json.loads(global_map['footprint'])
        points = json.loads(local['footprint'])
        padding = local['footprint_padding']
        required = max(math.hypot(abs(x)+padding, abs(y)+padding) for x, y in points)
        self.assertEqual(len(points), 4)
        self.assertEqual(global_map['footprint_padding'], 0.)
        self.assertGreater(len(circle), 16)
        self.assertLessEqual(len(circle), 64)  # Live observer transport limit.
        # Check geometry after Nav2's float parsing as well as JSON generation.
        parsed = [[struct.unpack('f', struct.pack('f', value))[0] for value in point]
                  for point in circle]
        for polygon in (circle, parsed):
            with self.subTest(float_parsed=polygon is parsed):
                for a, b in zip(polygon, polygon[1:]+polygon[:1]):
                    cross = a[0]*b[1]-a[1]*b[0]
                    self.assertGreater(cross, 0.)  # Nondegenerate, CCW edges.
                    distance = cross/math.hypot(b[0]-a[0], b[1]-a[1])
                    # Every edge is outside the circle containing all rotated
                    # padded corners, not merely outside the axis-aligned body.
                    self.assertGreaterEqual(distance, required)
                    self.assertLess(distance-required, 1e-5)
                for x, y in polygon:
                    # A circle approximation must not retain the square corners.
                    self.assertLess(math.hypot(x, y)-required, .001)
        self.assertGreater(global_map['inflation_layer']['inflation_radius'],
                           max(math.hypot(x, y) for x, y in circle))
        self.assertEqual(profile['planner_server']['ros__parameters']['GridBased']['allow_unknown'],
                         self.config['planner_server']['ros__parameters']['GridBased']['allow_unknown'])
        self.assertEqual(global_map['obstacle_layer']['footprint_clearing_enabled'],
                         self.config['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['footprint_clearing_enabled'])
        self.assertEqual(profile['planner_server']['ros__parameters']['GridBased']['smoother'],
                         self.config['planner_server']['ros__parameters']['GridBased']['smoother'])

    def test_square_geometry_is_retained_outside_fixed_heading_strategy(self):
        # Preserve the original shape for path heading and legacy geometry callers.
        for mode in (None, 'legacy', 'path_tangent'):
            with self.subTest(heading_mode=mode):
                polygon, radius = _global_footprint(.508355, mode)
                self.assertEqual(radius, .508355)
                self.assertEqual(polygon, [[-radius, -radius], [radius, -radius],
                                           [radius, radius], [-radius, radius]])
        circle, radius = _global_footprint(.508355, 'hold_initial')
        self.assertGreater(len(circle), 16)
        self.assertGreaterEqual(radius, .508355)

    def test_both_costmaps_use_each_lidars_own_ray_origin(self):
        profile = build()
        for name in ('local_costmap', 'global_costmap'):
            cm = profile[name][name]['ros__parameters']
            self.assertNotIn('voxel_layer', cm['plugins'])
            layer = cm['obstacle_layer']
            expected = self.config[name][name]['ros__parameters']
            for scanner in ('front', 'back'):
                self.assertEqual(layer[scanner]['topic'], expected['obstacle_layer'][scanner]['topic'])
            self.assertEqual(cm['static_layer']['map_topic'], expected['static_layer']['map_topic'])


class VelocityTests(unittest.TestCase):
    def test_diagonal_and_rotation_limits_match_chassis_defaults(self):
        x, y, yaw = bounded_twist([DEFAULT_LINEAR, DEFAULT_LINEAR, 0, 0, 0, 2*DEFAULT_ANGULAR], DEFAULT_LINEAR, DEFAULT_ANGULAR)
        self.assertAlmostEqual(math.hypot(x, y), DEFAULT_LINEAR)
        self.assertAlmostEqual(x, y)
        self.assertEqual(yaw, DEFAULT_ANGULAR)
        self.assertEqual(bounded_twist([-.02, .01, 0, 0, 0, -.03], .06, .1), (-.02, .01, -.03))

    def test_nonfinite_and_nonplanar_commands_are_rejected(self):
        for values in ([float('nan'), 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0]*5):
            with self.assertRaises(ValueError):
                bounded_twist(values, .06, .1)

    def test_old_command_and_no_goal_never_move(self):
        gate = CommandGate(.06, .1)
        gate.receive([.06, 0, 0, 0, 0, 0], 1.)
        self.assertEqual(gate.output(1.1, True, 'A'), (0., 0., 0.))
        gate.set_goal('goal1', 1.2)
        self.assertEqual(gate.output(1.21, True, 'A'), (0., 0., 0.))

    def test_localization_loss_latches_until_new_goal_not_merely_fresh_feedback(self):
        gate = CommandGate(.06, .1)
        gate.set_goal('goal1', 1.)
        gate.receive([.06, 0, 0, 0, 0, 0], 1.1)
        self.assertEqual(gate.output(1.11, True, 'A')[0], .06)
        self.assertEqual(gate.output(1.12, False, 'A')[0], 0.)
        gate.receive([.06, 0, 0, 0, 0, 0], 1.13)
        self.assertEqual(gate.output(1.14, True, 'A')[0], 0.)
        gate.set_goal('goal2', 1.2)
        gate.receive([.06, 0, 0, 0, 0, 0], 1.21)
        self.assertEqual(gate.output(1.22, True, 'A')[0], .06)

    def test_command_timeout_allows_600ms_then_latches_a_stop(self):
        for age in (.3, .45, .59, .6):
            with self.subTest(age=age):
                gate = CommandGate(.06, .1)
                gate.set_goal('goal', 0.)
                gate.receive([.03, 0, 0, 0, 0, 0], 0.)
                self.assertEqual(gate.output(age, True, 'A'), (.03, 0., 0.))
                self.assertIsNone(gate.failure)
                self.assertEqual(gate.output(.600001, True, 'A'), (0., 0., 0.))
                self.assertIn('> 0.600 秒', gate.failure['reason'])
                gate.receive([.03, 0, 0, 0, 0, 0], .61)
                self.assertEqual(gate.output(.62, True, 'A'), (0., 0., 0.))

    def test_epoch_change_or_command_dropout_stops_goal(self):
        for changed, now in (('B', 1.2), ('A', 2.)):
            gate = CommandGate(.06, .1)
            gate.set_goal('goal', 1.)
            gate.receive([.06, 0, 0, 0, 0, 0], 1.1)
            gate.output(1.11, True, 'A')
            self.assertEqual(gate.output(now, True, changed), (0., 0., 0.))
            self.assertEqual(gate.failed_goal, 'goal')

    def test_one_goal_continues_for_30_seconds_with_fresh_control_and_status(self):
        gate = CommandGate(.06, .1)
        gate.set_goal('goal', 1.)
        for tick in range(301):
            now = 1.+tick*.1
            # Repeated action status must not discard a running command.
            gate.set_goal('goal', now)
            gate.receive([.03, -.02, 0, 0, 0, .04], now)
            self.assertEqual(gate.output(now+.025, True, 'A'), (.03, -.02, .04))
        self.assertIsNone(gate.failed_goal)

    def test_failure_survives_cancellation_and_recovery_until_explicit_new_goal(self):
        gate = CommandGate(.06, .1)
        gate.set_goal('goal', 1.)
        gate.receive([.03, 0, 0, 0, 0, 0], 1.1)
        gate.output(1.11, True, 'A')
        self.assertEqual(gate.output(1.71, True, 'A'), (0., 0., 0.))
        failure = dict(gate.failure)
        self.assertAlmostEqual(failure['command_age'], .61)
        self.assertIn('断流', failure['reason'])
        gate.fail('secondary error', 1.72)
        gate.set_goal(None, 1.8)  # Nav2 acknowledges cancellation.
        gate.output(1.9, True, 'A')
        self.assertEqual(gate.failure, failure)
        self.assertEqual(gate.reason, failure['reason'])
        gate.set_goal('goal', 2.)
        gate.receive([.03, 0, 0, 0, 0, 0], 2.01)
        self.assertEqual(gate.output(2.02, True, 'A'), (0., 0., 0.))
        gate.set_goal('new-goal', 2.1)
        gate.receive([.03, 0, 0, 0, 0, 0], 2.11)
        self.assertEqual(gate.output(2.12, True, 'A'), (.03, 0., 0.))
        self.assertIsNone(gate.failure)

    def test_health_and_invalid_input_failures_keep_specific_first_cause(self):
        for reason in ('后雷达观测过期', '定位更新停滞', '导航节点未激活'):
            gate = CommandGate(.06, .1)
            gate.set_goal('goal', 1.)
            gate.output(1.1, False, 'A', readiness_reason=reason)
            gate.fail('secondary', 1.2)
            self.assertIn(reason, gate.failure['reason'])
            self.assertEqual(gate.output(1.3, True, 'A'), (0., 0., 0.))
        gate = CommandGate(.06, .1)
        gate.fail('invalid idle command', 1.)
        self.assertIsNone(gate.failure)
        gate.set_goal('goal', 2.)
        gate.fail('invalid planar command', 2.1)
        gate.receive([.03, 0, 0, 0, 0, 0], 2.11)
        self.assertEqual(gate.output(2.12, True, 'A'), (0., 0., 0.))

    def test_worker_independently_limits_and_rejects_unexpected_publishers(self):
        from fairino_slam.chassis import robot_worker
        worker = robot_worker.Worker.__new__(robot_worker.Worker)
        worker.node = Mock()
        worker.nav_received = 10.
        worker.nav_graph_checked = 0.
        worker.nav_command = (.02, 0., 0.)
        worker.node.get_publishers_info_by_topic.return_value = [Mock(node_name='other', node_namespace='/')]
        with patch.object(robot_worker.time, 'monotonic', return_value=10.1):
            with self.assertRaisesRegex(RuntimeError, '来源'):
                worker.navigation_command()
        worker.node.get_publishers_info_by_topic.return_value = [Mock(node_name='velocity_gate', node_namespace='/fairino_navigation')]
        with patch.object(robot_worker.time, 'monotonic', return_value=11.):
            self.assertEqual(worker.navigation_command(), (0., 0., 0.))


class DeploymentTests(unittest.TestCase):
    def test_embedded_installer_compiles_and_never_starts_service(self):
        from fairino_slam.deployment.install_navigation import remote_script
        source = remote_script()
        compile(source, '<navigation installer>', 'exec')
        self.assertIn('daemon-reload', source)
        self.assertNotIn("'systemctl', 'start'", source)
        self.assertIn('actual != EXPECTED', source)

    def test_navigation_mode_is_explicit_in_ssh_worker_bundle(self):
        import ast
        import shlex
        from types import SimpleNamespace
        from fairino_slam.chassis.transport import remote_command
        def worker_arguments(options):
            shell = shlex.split(remote_command(options))[2]
            loader = shlex.split(shell.splitlines()[-1])[-1]
            for node in ast.parse(loader).body:
                if (isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Attribute)
                        and node.targets[0].attr == 'argv'):
                    return ast.literal_eval(node.value)
            self.fail('Missing worker command line in SSH bundle')
        args = SimpleNamespace(linear=.06, angular=.1, check=False, prepare_check=False, navigation=True)
        self.assertIn('--navigation', worker_arguments(args))
        args.navigation = False
        # Mapping uses manual input; navigation must be selected explicitly.
        self.assertNotIn('--navigation', worker_arguments(args))


if __name__ == '__main__':
    unittest.main()
