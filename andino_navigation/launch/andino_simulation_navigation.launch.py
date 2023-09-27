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

"""Run navigation2 pkg with Andino robot in Gazebo."""


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the launch directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    andino_navigation_dir = get_package_share_directory('andino_navigation')
    andino_gz_classic_dir = get_package_share_directory('andino_gz_classic')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(andino_navigation_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS 2 parameters file to use for all launched nodes',
    )

    # Include andino simulation
    include_andino = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(andino_gz_classic_dir, 'launch', 'andino_one_robot.launch.py')
        ),
        launch_arguments={
            'use_gazebo_ros_control': LaunchConfiguration(
                'use_gazebo_ros_control', default='false'
            ),
            'initial_pose_x': LaunchConfiguration('initial_pose_x', default='-2.00'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y', default='-0.50'),
            'initial_pose_z': LaunchConfiguration('initial_pose_z', default='0.01'),
            'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw', default='0.00'),
            'world': LaunchConfiguration(
                'world',
                default=os.path.join(nav2_bringup_dir, 'worlds', 'world_only.model'),
            ),
            'rviz_config_file': LaunchConfiguration(
                'rviz_config_file',
                default=os.path.join(
                    andino_navigation_dir, 'rviz', 'nav2_default_view.rviz'
                ),
            ),
            'use_sim_time': use_sim_time,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(andino_navigation_dir, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration(
                'map',
                default=os.path.join(
                    nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml'
                ),
            ),
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(include_andino)
    ld.add_action(bringup_cmd)

    return ld
