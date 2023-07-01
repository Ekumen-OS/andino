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

"""Launch Gazebo with a world that has Andino."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from xacro import process_file

def get_robot_description(use_ros_control:str)->str:
    doc = process_file(os.path.join(get_package_share_directory('andino_gazebo'), 'urdf', 'andino.gazebo.xacro'),
                             mappings={'use_gazebo_ros_control': use_ros_control})
    robot_desc = doc.toprettyxml(indent='  ')
    folder=get_package_share_directory('andino_description')
    robot_desc =robot_desc.replace('package://andino_description/','file://'+str(folder)+'/')
    return robot_desc

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_argument = DeclareLaunchArgument('use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_andino_gazebo = get_package_share_directory('andino_gazebo')
    pkg_andino_control = get_package_share_directory('andino_control')
    
    # Include andino
    include_andino = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_andino_gazebo, 'launch', 'spawn_robot.launch.py'),
        ),
        launch_arguments={'use_gazebo_ros_control':'true'}.items()
    )

    robot_description = get_robot_description('true')
    controller_params_file = os.path.join(pkg_andino_control,'config','andino_controllers.yaml')

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description':robot_description},
                    controller_params_file],

        output="both",
    )

    # Include ros control

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_controller'],
        output='screen'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
        )
    
    # RViz
    rviz = Node(
        package= 'rviz2',
        executable= 'rviz2',
        parameters= [{'use_sim_time':use_sim_time}],
        arguments=['-d', os.path.join(pkg_andino_gazebo, 'rviz', 'andino_gazebo.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    andino_visualization_timer = TimerAction(period=5.0, actions=[control_node,load_joint_state_controller])
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_base_controller],
            )
        ),
        use_sim_time_argument,
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_andino_gazebo, 'worlds', 'empty_world.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        gazebo,
        include_andino,
        andino_visualization_timer
    ])
