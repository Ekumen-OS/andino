# Carpincho Bot

Carpincho is a fully open-source diff drive robot designed for educational purposes and low-cost applications.
It is fully integrated with ROS2 and it is a great base platform to improve skills over the robotics field.
With its open-source design, anyone can modify and customize the robot to suit their specific needs.

## :package: Package Overview

- :robot: [`carpincho_hardware`](./carpincho_hardware): Contains information about the Carpincho assembly and hardware parts.
- :ledger: [`carpincho_description`](./carpincho_description): Contains the URDF description of the robot.
- :oncoming_automobile: [`carpincho_firmware`](./carpincho_firmware): Contains the code be run in the microcontroller for interfacing low level hardware with the SBC.
- :computer: [`carpincho_base`](./carpincho_base): [ROS Control hardware interface](http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface) is implemented.
- :control_knobs: [`carpincho_control`](./carpincho_control/): It launches the [`controller_manager`](http://wiki.ros.org/controller_manager) for the real robot and loads up the diff_drive_controller and the joint_state_controller.
- :rocket: [`carpincho_bringup`](./carpincho_bringup): Contains mainly launch files in order to launch all related driver and nodes to be used in the real robot.


## Robot Assembly

Visit [`carpincho_hardware`](./carpincho_hardware/) for assembly instructions.

## Installation

### Platforms

 - ROS2: Humble Hawksbill
 - OS:
    - Ubuntu 22.04 Jammy Jellyfish
    - Ubuntu Mate 22.04 (On real robot (e.g: Raspberry Pi 4B))

### Build from Source

#### Dependencies

1. Install [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. Install [colcon](https://colcon.readthedocs.io/en/released/user/installation.html)

#### colcon workspace

Packages here provided are colcon packages. As such a colcon workspace is expected:

1. Create colcon workspace
```
mkdir -p ~/ws/src
```

2. Clone this repository in the `src` folder
```
cd ~/ws/src
```
```
git clone <Insert Correct Path> --> TODO
```

3. Install dependencies via `rosdep`
```
cd ~/ws
```
```
rosdep install --from-paths src --ignore-src -i -y
```

4. Build the packages
```
colcon build
```

5. Finally, source the built packages
If using `bash`:
```
source install/setup.bash
```

`Note`: Whether your are installing the packages in your dev machine or in your robot the procedure is the same.

## Usage

TODO

## Code development

Note that a [`Docker`](./docker) folder is provided for easy setting up the workspace.
