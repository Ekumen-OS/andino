# BSD 3-Clause License

# Copyright (c) 2023, Ekumen Inc.
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Arguments
    rviz_argument = DeclareLaunchArgument('rviz', default_value='true',
                          description='Open RViz.')
    rsp_argument = DeclareLaunchArgument('rsp', default_value='true',
                          description='Run robot state publisher node.')
    jsp_argument = DeclareLaunchArgument('jsp', default_value='true',
                          description='Run joint state publisher node.')

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

    # Joint state publisher gui
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp'))
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_carpincho_description, 'rviz', 'carpincho_description.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        jsp_argument,
        rviz_argument,
        rsp_argument,
        jsp_gui,
        rviz,
        rsp,
    ])
