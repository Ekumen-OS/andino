# Carpincho Bot

Carpincho is a fully open-source diff drive robot designed for educational purposes and low-cost applications.
It is fully integrated with ROS2 and it is a great base platform to improve skills over the robotics field.
With its open-source design, anyone can modify and customize the robot to suit their specific needs.

<img src="docs/real_robot.jpeg" width=500>


## :package: Package Overview

- :robot: [`carpincho_hardware`](./carpincho_hardware): Contains information about the Carpincho assembly and hardware parts.
- :ledger: [`carpincho_description`](./carpincho_description): Contains the URDF description of the robot.
- :oncoming_automobile: [`carpincho_firmware`](./carpincho_firmware): Contains the code be run in the microcontroller for interfacing low level hardware with the SBC.
- :computer: [`carpincho_base`](./carpincho_base): [ROS Control hardware interface](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html) is implemented.
- :control_knobs: [`carpincho_control`](./carpincho_control/): It launches the [`controller_manager`](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html) along with the [ros2 controllers](https://control.ros.org/master/doc/ros2_controllers/doc/controllers_index.html):  [diff_drive_controller](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html) and the [joint_state_broadcaster](https://control.ros.org/master/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html).
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

### Robot bringup

`carpincho_bringup` contains launch files that concentrates the process that brings up the robot.

After installing and sourcing the carpincho's packages simply run.

```
ros2 launch carpincho_bringup carpincho_robot.launch.py
```

This launch files initializes the differential drive controller and brings ups the system to interface with ROS.
By default sensors like the camera and the lidar are initialized. This can be disabled via arguments and manage each initialization separately. See `ros2 launch carpincho_bringup carpincho_robot.launch.py -s ` for checking out the arguments.
 - include_rplidar: `true` as default.
 - include_camera: `true` as default.


After the robot is launched, use `ROS 2 CLI` for inspecting environment. E.g: By doing `ros2 topic list` the available topics can be displayed.

### Teleoperation

Launch files for using the keyboard or a joystick for teleoperating the robot are provided.

[`twist_mux`](http://wiki.ros.org/twist_mux) is used to at the same time accept command velocities from different topics using certain priority for each one of them (See [twist_mux config](carpincho_bringup/config/twist_mux.yaml)). Available topics are (ordering by priority):
  - cmd_vel
  - cmd_vel_keyboard
  - cmd_vel_joy

### RViz

Use:
```
ros2 launch carpincho_bringup rviz.launch.py
```

For starting `rviz2` visualization with a provided configuration.

## Media

### Slam

Using the robot for mapping.

https://github.com/ekumenlabs/carpinchobot/assets/53065142/b189b9f3-1fd9-479b-a187-650d264f4629

## Code development

Note that a [`Docker`](./docker) folder is provided for easy setting up the workspace.
