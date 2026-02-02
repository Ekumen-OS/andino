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
import xacro

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    # Obtains andino_description's share directory path.
    pkg_andino_description = get_package_share_directory('andino_description')

    # Obtain urdf from xacro files.
    arguments = {
        'yaml_config_dir': os.path.join(pkg_andino_description, 'config', 'andino')
    }
    doc = xacro.process_file(
        os.path.join(pkg_andino_description, 'urdf', 'andino.urdf.xacro'),
        mappings=arguments,
    )
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc, 'publish_frequency': 30.0}

    # Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='both',
        parameters=[params],
    )

    return LaunchDescription(
        [
            rsp,
        ]
    )
