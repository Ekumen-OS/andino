# Copyright 2023, Ekumen Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Spawn an Andino robot in Gazebo, also launch the robot_state_publisher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from xacro import process_file


def get_robot_description(use_ros_control: str) -> str:
    """
    Obtain the urdf from the xacro file.

    This replace package tag by file tag to works with gazebo
    # See  https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1284

    Arguments:
        use_ros_control -- false to use diff drive gazebo plugin

    Returns
    -------
        urdf of the robot with gazebo data

    """
    doc = process_file(
        os.path.join(
            get_package_share_directory('andino_gz_classic'), 'urdf', 'andino_gz_classic.xacro'
        ),
        mappings={
            'use_gazebo_ros_control': use_ros_control,
            'use_real_ros_control': 'false',
            'use_fixed_caster': 'false',
        },
    )
    robot_desc = doc.toprettyxml(indent='  ')
    folder = get_package_share_directory('andino_description')
    robot_desc = robot_desc.replace(
        'package://andino_description/', f'file://{folder}/'
    )
    return robot_desc


def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    use_ros_control = LaunchConfiguration('use_gazebo_ros_control')
    entity = LaunchConfiguration('entity')
    robot_description_topic = LaunchConfiguration('robot_description_topic')
    rsp_frequency = LaunchConfiguration('rsp_frequency')

    x_argument = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial x pose of andino in the simulation',
    )
    y_argument = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial y pose of andino in the simulation',
    )
    z_argument = DeclareLaunchArgument(
        'initial_pose_z',
        default_value='0.05',
        description='Initial z pose of andino in the simulation',
    )
    yaw_argument = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Initial yaw pose of andino in the simulation',
    )
    gazebo_ros_control_argument = DeclareLaunchArgument(
        'use_gazebo_ros_control',
        default_value='false',
        description='True to use the gazebo_ros_control plugin',
    )
    entity_argument = DeclareLaunchArgument(
        'entity', default_value='andino', description='Name of the robot'
    )
    robot_desc_argument = DeclareLaunchArgument(
        'robot_description_topic',
        default_value='/robot_description',
        description='robot description topic ',
    )
    rsp_frequency_argument = DeclareLaunchArgument(
        'rsp_frequency',
        default_value='30.0',
        description='robot state publisher frequency',
    )
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    # TODO (olmerg) Multirobot. How to change the name of topic with entity parameter
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'publish_frequency': rsp_frequency,
                'robot_description': get_robot_description('false'),
            }
        ],
        remappings=remappings,
        condition=IfCondition(PythonExpression(["'", use_ros_control, "' == 'false'"])),
    )

    rsp_control = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'publish_frequency': rsp_frequency,
                'robot_description': get_robot_description('true'),
            }
        ],
        remappings=remappings,
        condition=IfCondition(use_ros_control),
    )
    robot_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic',
            robot_description_topic,
            '-entity',
            entity,
            '-x',
            initial_pose_x,
            '-y',
            initial_pose_y,
            '-z',
            initial_pose_z,
            '-R',
            '0.0',
            '-P',
            '0.0',
            '-Y',
            initial_pose_yaw,
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        condition=IfCondition(use_ros_control),
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_controller', '--controller-manager', '/controller_manager'],
        condition=IfCondition(use_ros_control),
    )

    return LaunchDescription(
        [
            use_sim_time_argument,
            x_argument,
            y_argument,
            z_argument,
            robot_desc_argument,
            rsp_frequency_argument,
            yaw_argument,
            gazebo_ros_control_argument,
            entity_argument,
            rsp,
            rsp_control,
            robot_spawn,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
        ]
    )
