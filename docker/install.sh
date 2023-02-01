#!/usr/bin/env bash

apt-get update && apt-get install --no-install-recommends -y $(cat requirements.txt)

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ros related packages
apt-get update && apt-get install --no-install-recommends -y $(cat requirements_ros.txt)

# Add OSRF repositories to apt sources list
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install ignition related packages
apt-get update && apt-get install --no-install-recommends -y $(cat requirements_ign.txt)
