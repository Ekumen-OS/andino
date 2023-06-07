# Copyright 2023 Franco Cipollone

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Arguments
    rsp_argument = DeclareLaunchArgument('rsp', default_value='true',
                          description='Run robot state publisher node.')

    # Obtains carpincho_description's share directory path.
    pkg_carpincho_description = get_package_share_directory('carpincho_description')

    # Obtain urdf from xacro files.
    doc = xacro.process_file(os.path.join(pkg_carpincho_description, 'urdf', 'carpincho.urdf.xacro'))
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc,
              'publish_frequency': 30.0}

    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='',
                output='both',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('rsp'))
    )

    return LaunchDescription([
        rsp_argument,
        rsp,
    ])
