from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('xbox_control')
    params = os.path.join(pkg, 'config', 'xbox_control_all_params.yaml')

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 0.0
        }],
        output='screen'
    )

    xbox_input = Node(
        package='xbox_control',
        executable='xbox_input',
        name='xbox_input',
        parameters=[params],
        output='screen'
    )

    xbox_cmdvel = Node(
        package='xbox_control',
        executable='xbox_cmdvel',
        name='xbox_cmdvel',
        parameters=[params],
        output='screen'
    )

    return LaunchDescription([joy, xbox_input, xbox_cmdvel])
