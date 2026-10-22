"""Isolated 2D Nav2 servers. Existing localization owns map->odom."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    root = Path(__file__).resolve().parent
    namespace = 'fairino_navigation'
    params = RewrittenYaml(source_file=str(root/'nav2.yaml'), root_key=namespace,
                           param_rewrites={}, convert_types=True)
    nodes = []
    for package, executable in (('nav2_planner', 'planner_server'),
                                ('nav2_controller', 'controller_server'),
                                ('nav2_bt_navigator', 'bt_navigator')):
        overrides = {}
        if executable == 'bt_navigator':
            overrides = {
                'default_nav_to_pose_bt_xml': str(root/'navigate_to_pose.xml'),
                'default_nav_through_poses_bt_xml': str(root/'navigate_through_poses.xml')}
        nodes.append(Node(
            package=package, executable=executable, name=executable,
            namespace=namespace, output='screen', parameters=[params, overrides],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static'),
                        ('cmd_vel', '/fairino_navigation/cmd_vel_raw')]))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager', namespace=namespace, output='screen',
        parameters=[{'autostart': False, 'bond_timeout': 4.,
                     'attempt_respawn_reconnection': False,
                     'node_names': ['planner_server', 'controller_server', 'bt_navigator']}]))
    nodes.append(ExecuteProcess(cmd=['python3', str(root/'velocity_gate.py')], output='screen'))
    # Spread DDS discovery / process creation across startup instead of creating
    # every participant at once beside the real-time chassis feedback stream.
    # Gate starts first and emits zero without an explicit navigation goal.
    ordered = [nodes[-1], *nodes[:-1]]
    return LaunchDescription([ordered[0]] + [
        TimerAction(period=3.0 * index, actions=[action])
        for index, action in enumerate(ordered[1:], 1)])
