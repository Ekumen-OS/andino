import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joystick_config = os.path.join(get_package_share_directory('carpincho_bringup'),'config','joystick.yaml')

    joy_linux_node = Node(
            package='joy_linux',
            executable='joy_linux_node',
            parameters=[joystick_config],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joystick_config],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )

    return LaunchDescription([
        joy_linux_node,
        teleop_node,
    ])
