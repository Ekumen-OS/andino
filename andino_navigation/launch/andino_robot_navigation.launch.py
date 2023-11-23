
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
    # [ ] Navigation Bringup
    # [ ] Slam Toolbox
    # [ ] RVIZ (aparte)
    # [ ] Andino bringup?
    # Get the launch directory
    andino_bringup_dir = get_package_share_directory('andino_bringup')
    andino_navigation_dir = get_package_share_directory('andino_navigation')

    params_file = LaunchConfiguration('params_file')

    delcare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(andino_navigation_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS 2 parameters file to use for all launched nodes',
    )
    
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(andino_navigation_dir, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'slam': 'True',
            'params_file': params_file,
        }.items(),
    )

    include_andino_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(andino_bringup_dir, 'launch', 'andino_robot.launch.py')
        )
    )

    ld = LaunchDescription()

    ld.add_action(delcare_params_file_cmd)

    ld.add_action(bringup_cmd)
    ld.add_action(include_andino_bringup)
    
    return ld
