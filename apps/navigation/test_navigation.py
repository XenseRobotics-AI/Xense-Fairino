import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'tools'))
sys.path.insert(0, str(ROOT/'apps/fairino_keyboard'))
from apps.navigation.footprint import ChassisFootprint, FootprintRequest, selector
from apps.navigation.motion import CommandGate, bounded_twist
from apps.fairino_keyboard.control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR
from build_navigation import build


class ChassisGeometryTests(unittest.TestCase):
    def test_runtime_urdf_projects_to_expected_chassis_rectangle(self):
        shape = selector(ROOT/'models/amr_8dof.urdf').compute('chassis', FootprintRequest(12., {}))
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
    def test_profile_is_generated_from_current_slam_defaults(self):
        import yaml
        from fairino_slam.chassis.control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR
        profile = build()
        self.assertEqual(yaml.safe_load((ROOT/'apps/navigation/nav2.yaml').read_text()), profile)
        controller = profile['controller_server']['ros__parameters']
        follow = controller['FollowPath']
        self.assertEqual(follow['plugin'], 'fairino_lookahead_controller::HolonomicLookaheadController')
        self.assertEqual(follow['max_linear_speed'], DEFAULT_LINEAR)
        self.assertEqual(follow['max_angular_speed'], DEFAULT_ANGULAR)
        self.assertEqual(follow['heading_mode'], 'hold_initial')
        self.assertLess(controller['min_y_velocity_threshold'], DEFAULT_LINEAR)

    def test_controller_lookahead_period_and_collision_horizon_follow_config(self):
        profile = build()
        controller = profile['controller_server']['ros__parameters']
        follow = controller['FollowPath']
        self.assertEqual(controller['controller_frequency'], 3.)
        self.assertAlmostEqual(follow['control_period'] * controller['controller_frequency'], 1.)
        self.assertEqual(follow['lookahead_distance'], .2)
        self.assertGreater(follow['collision_horizon'], follow['control_period'])
        self.assertGreater(follow['linear_acceleration'], 0.)
        self.assertGreater(follow['angular_acceleration'], 0.)

    def test_smac_global_inscribed_radius_contains_every_rotated_local_corner(self):
        profile = build()
        local = profile['local_costmap']['local_costmap']['ros__parameters']
        global_map = profile['global_costmap']['global_costmap']['ros__parameters']
        # SLAM uses a circumscribed regular polygon rather than the old square.
        polygon = json.loads(global_map['footprint'])
        points = json.loads(local['footprint'])
        padding = local['footprint_padding']
        padded_radius = max(math.hypot(abs(x)+padding, abs(y)+padding) for x,y in points)
        for a, b in zip(polygon, polygon[1:]+polygon[:1]):
            edge_distance = (a[0]*b[1]-a[1]*b[0])/math.hypot(b[0]-a[0], b[1]-a[1])
            self.assertGreaterEqual(edge_distance+1e-12, padded_radius)
        self.assertGreater(global_map['inflation_layer']['inflation_radius'], padded_radius)
        planner = profile['planner_server']['ros__parameters']['GridBased']
        self.assertEqual(planner['plugin'], 'fairino_straight_planner::StraightLinePlanner')
        self.assertTrue(planner['straight_line_enabled'])
        self.assertFalse(planner['allow_unknown'])
        self.assertFalse(global_map['obstacle_layer']['footprint_clearing_enabled'])
        self.assertEqual(planner['smoother']['max_iterations'], 0)

    def test_both_costmaps_use_each_lidars_own_ray_origin(self):
        profile = build()
        for name in ('local_costmap', 'global_costmap'):
            cm = profile[name][name]['ros__parameters']
            self.assertNotIn('voxel_layer', cm['plugins'])
            layer = cm['obstacle_layer']
            self.assertEqual(layer['front']['topic'], '/front/scan')
            self.assertEqual(layer['back']['topic'], '/back/scan')
            self.assertEqual(cm['static_layer']['map_topic'], '/map')


class VelocityTests(unittest.TestCase):
    def test_diagonal_and_rotation_limits_match_keyboard_defaults(self):
        x, y, yaw = bounded_twist([.06, .06, 0, 0, 0, .4], DEFAULT_LINEAR, DEFAULT_ANGULAR)
        self.assertAlmostEqual(math.hypot(x, y), .06)
        self.assertAlmostEqual(x, y)
        self.assertEqual(yaw, .1)
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
        self.assertEqual(gate.output(1.41, True, 'A'), (0., 0., 0.))
        failure = dict(gate.failure)
        self.assertAlmostEqual(failure['command_age'], .31)
        self.assertIn('断流', failure['reason'])
        gate.fail('secondary error', 1.42)
        gate.set_goal(None, 1.5)  # Nav2 acknowledges cancellation.
        gate.output(1.6, True, 'A')
        self.assertEqual(gate.failure, failure)
        self.assertEqual(gate.reason, failure['reason'])
        gate.set_goal('goal', 1.7)
        gate.receive([.03, 0, 0, 0, 0, 0], 1.71)
        self.assertEqual(gate.output(1.72, True, 'A'), (0., 0., 0.))
        gate.set_goal('new-goal', 1.8)
        gate.receive([.03, 0, 0, 0, 0, 0], 1.81)
        self.assertEqual(gate.output(1.82, True, 'A'), (.03, 0., 0.))
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
        import robot_worker
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
        from install_navigation import remote_script
        source = remote_script()
        compile(source, '<navigation installer>', 'exec')
        self.assertIn('daemon-reload', source)
        self.assertNotIn("'systemctl', 'start'", source)
        self.assertIn('actual != EXPECTED', source)

    def test_navigation_mode_is_explicit_in_ssh_worker_bundle(self):
        from types import SimpleNamespace
        from keyboard_control import remote_command
        args = SimpleNamespace(linear=.06, angular=.1, check=False, prepare_check=False, navigation=True)
        self.assertIn('--navigation', remote_command(args))
        args.navigation = False
        # The flag is only in the encoded source when navigation is not requested.
        self.assertNotIn("'--navigation'", remote_command(args))


if __name__ == '__main__':
    unittest.main()
