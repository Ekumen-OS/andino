# andino_bringup

## Description
This package contains mainly launch files in order to launch all related driver and nodes to be used in the real robot. Some configuration files are also added to set up the nodes launched in the 'config' folder.

## Launch Files

### Main file

The main launch file is the `andino_robot.launch.py` file. This file executes all necessary nodes for Andino Robot to work.

It includes:

- `andino_description.launch.py`: file to load the 3D model of the robot from the URDF.
- `andino_control.launch.py`: file for the `ros2_control` nodes and drivers.
- `camera.launch.py`: file to execute the camera drivers, using `v4l2_camera` package. If not necessary, it can be disabled using the `include_camera` parameter.
- `rplidar.launch.py`: file to execute the drivers of the RP 2D LiDAR attached to the robot. If not necessary, it can be disabled using the `include_rplidar` parameter.

### Other launch files included

These launch files are included just in case you want to command the robot in different ways, or visualize the information:

- `rosbag_record.launch.py`: executes the `ros2 bag record` command to store a '.bag' file with the specified topics. If no topics are specified, it will record all of them.
- `rviz.launch.py`: executes the RViz visualization tool with the 'andino.rviz' configuration by default, to be able to visualize the 3D model of the robot and information from the topics.
- `teleop_joystick.launch.py`: this file allows you to command the robot using a controller with a joystick, using the `teleop_twist_joy` and the `joy_linux` packages.
- `teleop_keyboard.launch.py`: this file allows you to command the robot using a keyboard, using the `teleop_twist_keyboard` package.
