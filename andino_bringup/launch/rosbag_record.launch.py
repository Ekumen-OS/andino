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

import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution

## Executes rosbag record command:
## - https://github.com/ros2/rosbag2
##
## Considerations:
## - Uses mcap format by default.
##    - Docs: https://github.com/ros2/rosbag2/tree/humble/rosbag2_storage_mcap
##    - Uses zstd_fast profile by default.
## - Uses sqlite3 format if mcap_format is set to False.
## - rosbag_output_folder indicates the output folder of the rosbag.
##   - The rosbag name is generated automatically with the following format: andino_<current_date_time>

def generate_launch_description():
    # Declares launch arguments
    mcap_format_arg = DeclareLaunchArgument(
            'mcap_format',
            default_value='True',
            description='Indicates whether to use mcap format.')
    mcap_format =  LaunchConfiguration('mcap_format')

    mcap_profile_preset_arg = DeclareLaunchArgument(
            'mcap_profile_preset',
            default_value='zstd_fast',
            description='Indicates the profile for recording the bag. Options: [fastwrite, zstd_fast, zstd_small]')
    mcap_profile_preset =  LaunchConfiguration('mcap_profile_preset')

    rosbag_output_folder_arg = DeclareLaunchArgument(
            'rosbag_output_folder',
            default_value='andino_rosbag_records',
            description='Indicates the output name of the rosbag.')
    rosbag_output_folder =  LaunchConfiguration('rosbag_output_folder')


    current_date_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    rosbag_name = "andino_" + current_date_time

    rosbag_mcap_execute = ExecuteProcess(
            cmd=[['ros2 ','bag ', 'record ', '--storage ', 'mcap ',
                  '-a ', '--storage-preset-profile ', mcap_profile_preset,
                  ' -o ', PathJoinSubstitution([rosbag_output_folder, TextSubstitution(text=rosbag_name)])
            ]],
            output='screen',
            shell=True,
            condition=IfCondition(mcap_format)
        )

    rosbag_sqlite_execute = ExecuteProcess(
            cmd=[['ros2 ','bag ', 'record ', '--storage ', 'sqlite3 ',
                  '-a ', ' -o ', PathJoinSubstitution([rosbag_output_folder, TextSubstitution(text=rosbag_name)])
            ]],
            output='screen',
            shell=True,
            condition=IfCondition(PythonExpression(['not ', mcap_format]))
        )

    return LaunchDescription([
        mcap_format_arg,
        mcap_profile_preset_arg,
        rosbag_output_folder_arg,
        rosbag_mcap_execute,
        rosbag_sqlite_execute,
    ])
