# BSD 3-Clause License
#
# Copyright (c) 2023, Ekumen Inc.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# Obtains share directory paths.
pkg_andino_bringup = get_package_share_directory('andino_bringup')
pkg_andino_control = get_package_share_directory('andino_control')
pkg_andino_description = get_package_share_directory('andino_description')

def generate_launch_description():
    # Declares launch arguments
    camera_arg = DeclareLaunchArgument(
            'include_camera',
            default_value='True',
            description='Indicates whether to include camera launch.')
    camera =  LaunchConfiguration('include_camera')
    rplidar_arg = DeclareLaunchArgument(
            'include_rplidar',
            default_value='True',
            description='Indicates whether to include rplidar launch.')
    rplidar =  LaunchConfiguration('include_rplidar')

    # Includes andino_description launch file
    include_andino_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_andino_description, 'launch', 'andino_description.launch.py'),
        ),
    )

    # Include andino_control launch file
    include_andino_control =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_andino_control, 'launch', 'andino_control.launch.py'),
        ),
        launch_arguments={
        }.items()
    )

    # Include rplidar launch file
    include_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_andino_bringup, 'launch', 'rplidar.launch.py'),
        ),
        launch_arguments={
            "serial_port": '/dev/ttyUSB_LIDAR',
        }.items(),
                condition=IfCondition(rplidar)
    )
    # Include camera launch file
    include_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_andino_bringup, 'launch', 'camera.launch.py'),
        ),
        launch_arguments={
        }.items(),
                condition=IfCondition(camera)
    )

    # TODO(francocipollone): Improve concatenation of launch files.
    #
    # Waits for andino_description to set up robot_state_publisher.
    andino_control_timer = TimerAction(period=5.0, actions=[include_andino_control])
    # Defer sensors launch to avoid overhead while robot_state_publisher is setting up.
    rplidar_timer = TimerAction(period=3.0, actions=[include_rplidar])
    camera_timer = TimerAction(period=3.0, actions=[include_camera])

    return LaunchDescription([
        include_andino_description,
        andino_control_timer,
        camera_arg,
        camera_timer,
        rplidar_arg,
        rplidar_timer,
    ])
