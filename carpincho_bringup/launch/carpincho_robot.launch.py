# Copyright 2023 Franco Cipollone

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Obtains share directory paths.
pkg_carpincho_bringup = get_package_share_directory('carpincho_bringup')
pkg_carpincho_control = get_package_share_directory('carpincho_control')
pkg_carpincho_description = get_package_share_directory('carpincho_description')

def generate_launch_description():
    # Includes carpincho_description launch file
    include_carpincho_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_carpincho_description, 'launch', 'carpincho_description.launch.py'),
        ),
        launch_arguments={
            'rsp': 'True',
        }.items()
    )

    # Include carpincho_control launch file
    include_carpincho_control =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_carpincho_control, 'launch', 'carpincho_control.launch.py'),
        ),
        launch_arguments={
        }.items()
    )

    # Waits for carpincho_description to set up robot_state_publisher
    carpincho_control_timer = TimerAction(period=3.0,
                actions=[include_carpincho_control])

    twist_mux_params = os.path.join(pkg_carpincho_bringup,'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_controller/cmd_vel_unstamped')]
        )

    return LaunchDescription([
        include_carpincho_description,
        carpincho_control_timer,
        twist_mux,
    ])
