# Licensed under the 3-Clause BSD License
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

"""Launch Gazebo with a world that has Andino."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_andino_gazebo = get_package_share_directory("andino_gazebo")

    use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    world_argument = DeclareLaunchArgument(
        "world",
        default_value=[
            os.path.join(pkg_andino_gazebo, "worlds", "empty_world.world"),
            "",
        ],
        description="SDF world file",
    )
    use_rviz_argument = DeclareLaunchArgument(
        "rviz", default_value="true", description="Open RViz."
    )

    # Include andino
    include_andino = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_andino_gazebo, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={"use_gazebo_ros_control": "false"}.items(),
    )
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
        )
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", os.path.join(pkg_andino_gazebo, "rviz", "andino_gazebo.rviz")],
        condition=IfCondition(use_rviz),
    )

    andino_visualization_timer = TimerAction(period=5.0, actions=[rviz])
    return LaunchDescription(
        [
            use_sim_time_argument,
            world_argument,
            use_rviz_argument,
            gazebo,
            include_andino,
            andino_visualization_timer,
        ]
    )
