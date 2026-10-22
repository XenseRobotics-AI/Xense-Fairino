"""Build a Humble Nav2 profile from chassis geometry and shared speed defaults."""
import math
import json


def build_profile(points, linear, angular):
    if not (0 < linear <= .2 and 0 < angular <= .4):
        raise ValueError('Invalid chassis speed limits')
    padding = .02
    # Smac2D checks inflation costs, not a yaw-dependent polygon. Its global
    # footprint therefore has an inscribed circle containing the padded chassis.
    radius = max(math.hypot(abs(x)+padding, abs(y)+padding) for x, y in points)
    global_square = [[-radius, -radius], [radius, -radius], [radius, radius], [-radius, radius]]
    def costmap(global_map):
        scans = {}
        for name in ('front', 'back'):
            scans[name] = {
                'topic': '/'+name+'/scan', 'data_type': 'LaserScan',
                'marking': True, 'clearing': True, 'inf_is_valid': True,
                'expected_update_rate': .5, 'observation_persistence': 0.,
                'obstacle_min_range': .1, 'obstacle_max_range': 12.,
                'raytrace_min_range': .1, 'raytrace_max_range': 15.,
                'min_obstacle_height': -.2, 'max_obstacle_height': 2.}
        result = {
            'use_sim_time': False, 'global_frame': 'map' if global_map else 'odom',
            'robot_base_frame': 'base_link', 'resolution': .05,
            'transform_tolerance': .3, 'update_frequency': 2. if global_map else 5.,
            'publish_frequency': 2., 'always_send_full_costmap': True,
            'footprint': json.dumps(global_square if global_map else points),
            'footprint_padding': 0. if global_map else padding,
            'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
            'static_layer': {'plugin': 'nav2_costmap_2d::StaticLayer',
                             'map_topic': '/map', 'map_subscribe_transient_local': True},
            'obstacle_layer': {'plugin': 'nav2_costmap_2d::ObstacleLayer',
                               'observation_sources': 'front back',
                               'footprint_clearing_enabled': not global_map, **scans},
            'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer',
                                'inflation_radius': max(.75, radius+.2), 'cost_scaling_factor': 3.}}
        if global_map:
            result['track_unknown_space'] = True
        else:
            result.update(rolling_window=True, width=4, height=4, track_unknown_space=True)
        return {'ros__parameters': result}
    return {
        'planner_server': {'ros__parameters': {
            'use_sim_time': False, 'expected_planner_frequency': 1.,
            'planner_plugins': ['GridBased'],
            'GridBased': {'plugin': 'nav2_smac_planner/SmacPlanner2D',
                          'tolerance': .05, 'allow_unknown': False,
                          'downsample_costmap': False, 'max_planning_time': 2.,
                          'cost_travel_multiplier': 2., 'use_final_approach_orientation': False,
                          'smoother': {'max_iterations': 0}}}},
        'controller_server': {'ros__parameters': {
            'use_sim_time': False, 'controller_frequency': 10.,
            'odom_topic': '/odom', 'min_x_velocity_threshold': .001,
            'min_y_velocity_threshold': .001, 'min_theta_velocity_threshold': .001,
            'failure_tolerance': .3, 'progress_checker_plugin': 'progress_checker',
            'goal_checker_plugins': ['goal_checker'], 'controller_plugins': ['FollowPath'],
            'progress_checker': {'plugin': 'nav2_controller::SimpleProgressChecker',
                                 'required_movement_radius': .03, 'movement_time_allowance': 60.},
            'goal_checker': {'plugin': 'nav2_controller::SimpleGoalChecker',
                             'xy_goal_tolerance': .05, 'yaw_goal_tolerance': .05, 'stateful': True},
            'FollowPath': {
                'plugin': 'dwb_core::DWBLocalPlanner',
                # Sample only velocities reachable from measured odometry over
                # one control period. StandardTrajectoryGenerator samples over
                # sim_time and may issue a large target jump in a single cycle.
                'trajectory_generator_name': 'dwb_plugins::LimitedAccelGenerator',
                'sim_period': .1,
                'min_vel_x': -linear, 'max_vel_x': linear,
                'min_vel_y': -linear, 'max_vel_y': linear,
                'max_vel_theta': angular, 'max_speed_xy': linear,
                'min_speed_xy': 0., 'min_speed_theta': 0.,
                'acc_lim_x': .2, 'acc_lim_y': .2, 'acc_lim_theta': .4,
                'decel_lim_x': -.2, 'decel_lim_y': -.2, 'decel_lim_theta': -.4,
                'vx_samples': 5, 'vy_samples': 5, 'vtheta_samples': 7,
                'sim_time': 3., 'linear_granularity': .02, 'angular_granularity': .02,
                'transform_tolerance': .3, 'trans_stopped_velocity': .005,
                'xy_goal_tolerance': .05, 'short_circuit_trajectory_evaluation': True,
                'stateful': True,
                'critics': ['ObstacleFootprint', 'RotateToGoal', 'Oscillation', 'PathAlign', 'GoalAlign', 'PathDist', 'GoalDist'],
                'ObstacleFootprint.scale': 1., 'PathAlign.scale': 8.,
                'PathAlign.forward_point_distance': .1, 'GoalAlign.scale': 4.,
                'GoalAlign.forward_point_distance': .1, 'PathDist.scale': 32.,
                'GoalDist.scale': 24., 'RotateToGoal.scale': 32.,
                'RotateToGoal.slowing_factor': 2., 'RotateToGoal.lookahead_time': -1.}}},
        'local_costmap': {'local_costmap': costmap(False)},
        'global_costmap': {'global_costmap': costmap(True)},
        'bt_navigator': {'ros__parameters': {
            'use_sim_time': False, 'global_frame': 'map', 'robot_base_frame': 'base_link',
            'odom_topic': '/odom', 'bt_loop_duration': 50, 'default_server_timeout': 1000,
            'wait_for_service_timeout': 1000,
            'plugin_lib_names': ['nav2_compute_path_to_pose_action_bt_node',
                                 'nav2_compute_path_through_poses_action_bt_node',
                                 'nav2_follow_path_action_bt_node',
                                 'nav2_rate_controller_bt_node', 'nav2_pipeline_sequence_bt_node']}}}
