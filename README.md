# Carpincho Bot

TODO: Add Carpincho description and motivation

## :package: Package Overview

- [`carpincho_description`](./carpincho_description): Contains the URDF description of the robot.
- [`carpincho_firmware`](./carpincho_firmware): Contains the code be run in the microcontroller.
- [`carpincho_base`](./carpincho_base): [ROS Control hardware interface](http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface) is implemented.
- [`carpincho_control`](./carpincho_control/): It launches the [`controller_manager`](http://wiki.ros.org/controller_manager) for the real robot and loads up the diff_drive_controller and the joint_state_controller.
- [`carpincho_bringup`](./carpincho_bringup): Contains mainly launch files in order to launch all related driver and nodes to be used in the real robot.
- [`carpincho_simulation`](./carpincho_simulation): Contains the files to launch the Robot with Gazebo.
- [`docker`](./docker): Contains the scripts to build and run a docker container with Gazebo and ROS.

## Hardware

TODO: Adds list of hardware parts.
TODO: Add link to a building_carpincho.md doc. or explain it here.

## Installation

TODO:

## Usage

TODO:


