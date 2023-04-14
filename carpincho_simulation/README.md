# carpincho_simulation

## Description

This packages is in charge launching a Gazebo simulation using the [`URDF`](http://wiki.ros.org/urdf) file provided by 
[`carpincho_description`](../carpincho_description/) package.

It uses the same controllers as the real robot by running with [`gazebo_ros_control`](http://wiki.ros.org/gazebo_ros_control).

## Usage
To launch the Carpincho simulation environment using Gazebo Diff Drive, run the [`bringup.launch`](launch/bringup.launch) file:

```sh
roslaunch carpincho_simulation bringup.launch
```

To launch the Carpincho simulation environment using Gazebo ROS Control, run the [`bringup.launch`](launch/bringup.launch) file with the `use_gazebo_ros_control` argument:

```sh
roslaunch carpincho_simulation bringup.launch use_gazebo_ros_control:=true
```

To have a better idea on the arguments that you can pass to the [`bringup.launch`](launch/bringup.launch) file, you can use:

```sh
roslaunch --ros-args carpincho_simulation bringup.launch
```

To launch the teleop run the [`teleop.launch`](../carpincho_bringup/launch/teleop.launch) file:

```sh
roslaunch carpincho_bringup teleop.launch
```
