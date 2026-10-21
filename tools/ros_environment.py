"""ROS environment for passive hardware observers on the two robot computers."""


def feedback_environment(part):
    lines = ['set -e', 'source /opt/ros/humble/setup.bash']
    if part == 'upper':
        lines += [
            'source /opt/fairino_art_plugin/plugin/install/setup.bash',
            'export ROS_DOMAIN_ID=0',
            # Match the working upper-body control clients. Default local
            # transports can discover /hardware_states without receiving it.
            'export FASTRTPS_DEFAULT_PROFILES_FILE=/home/fairino/fastrtps_udp.xml',
        ]
    elif part == 'base':
        lines += [
            'source /opt/fairino_AMR_project_B/reference_chassis_runtime/install/'
            'hardware_state_broadcaster_msgs/share/hardware_state_broadcaster_msgs/local_setup.bash',
            'export ROS_DOMAIN_ID=1',
            'unset FASTRTPS_DEFAULT_PROFILES_FILE',
        ]
    else:
        raise ValueError(f'Unknown robot part: {part}')
    lines.append('export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0')
    return '\n'.join(lines) + '\n'
