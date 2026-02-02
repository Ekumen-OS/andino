#!/bin/bash

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

set -e

# Prints information about usage.
function show_help() {
  echo $'\nUsage:\t build.sh [OPTIONS] \n
  Options:\n
  \t-i --image_name\t\t Name of the image to be built (default ros2_andino).\n
  \t-d --ros_distro\t\t ROS 2 distro to use: humble or jazzy (default humble).\n
  Example:\n
  \tbuild.sh --ros_distro jazzy --image_name custom_image_name\n'
}

echo "Building the docker image for ros2 andino development."

SCRIPT_FOLDER_PATH="$(cd "$(dirname "$0")"; pwd)"
CONTEXT_FOLDER_PATH="$(cd "$(dirname "$0")"; cd .. ; pwd)"

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -i|--image_name) IMAGE_NAME="${2}"; shift ;;
        -d|--ros_distro) ROS_DISTRO="${2}"; shift ;;
        -h|--help) show_help ; exit 1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Update the arguments to default values if needed.
ROS_DISTRO=${ROS_DISTRO:-humble}
IMAGE_NAME=${IMAGE_NAME:-ros2_${ROS_DISTRO}_andino}
DOCKERFILE_PATH=$SCRIPT_FOLDER_PATH/Dockerfile

USERID=$(id -u)
USER=$(whoami)

echo "Building image for ROS 2 $ROS_DISTRO..."

sudo docker build -t $IMAGE_NAME \
     --file $DOCKERFILE_PATH \
     --build-arg USERID=$USERID \
     --build-arg USER=$USER \
     --build-arg ROS_DISTRO=$ROS_DISTRO \
     $CONTEXT_FOLDER_PATH
