# carpincho_base

## Description

The hardware-software-ros interaction in the `carpincho` project is developed using [ROS Control](http://wiki.ros.org/ros_control).

This package:
 - Creates the `CarpinchoHWInterface` from the `hardware_interface::RobotHW`.
 - Run `carpincho_hw_interface` node which instantiates `CarpinchoHWInterface` and adds it to the [controller manager]([`controller_manager`](http://wiki.ros.org/controller_manager)).
 - Provides scripts to be loaded in the microcontroller.


As expected, the `CarpinchoHWInterface` overrides two methods:
 - [read()](http://docs.ros.org/en/noetic/api/hardware_interface/html/c++/classhardware__interface_1_1RobotHW.html#a8c2f35853432383afa331ee712b24fe7):
   The joint states are read from the microcontroller which publishes its information over ROSSerial.
   The correspondent [JoinStateHandles](http://docs.ros.org/en/jade/api/hardware_interface/html/c++/classhardware__interface_1_1JointStateHandle.html) are updated with the joint states information.
 - [write()](http://docs.ros.org/en/noetic/api/hardware_interface/html/c++/classhardware__interface_1_1RobotHW.html#a40d06ffd5d382e67c4b0473020b6c256):
   Joint velocity commands are published to the microcontroller which is in charge to command the motors. The joint velocity commands arrives from the diff-drive controller

## Launch files

 - [`carpincho.launch`](launch/carpincho.launch):
    - Run `carpincho_hw_interface`.
    - Includes [`carpincho_control`](../carpincho_control/)'s launch files which executes the DiffDrive Controller.

See [`carpincho_control`](../carpincho_control/) package.
