from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('xbox_control')
    params_file = os.path.join(pkg_share, 'config', 'xbox_control_debug.yaml')

    return LaunchDescription([
        # joy_node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,            # -1 = auto, หรือ 0/1 ตาม js0/js1
                'deadzone': 0.05,           # deadzone for analog stick
                'autorepeat_rate': 1.0      # 0.0 = disable autorepeat
            }],
            output='screen'
        ),

        # xbox_control node
        Node(
            package='xbox_control',
            executable='xbox_control_debug',
            name='xbox_control_debug',
            parameters=[params_file],
            output='screen'
        )
    ])
