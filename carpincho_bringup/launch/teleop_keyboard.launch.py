# Copyright 2023 Franco Cipollone

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    teleop_node = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard_node',
            remappings=[('/cmd_vel','/cmd_vel_keyboard')]
         )

    return LaunchDescription([
        teleop_node,
    ])
