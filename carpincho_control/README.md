# carpincho_control

## Description

This packages is in charge of spawning controllers to be integrated with the hardware interface provided by [`carpincho_base`](../carpincho_base/) package.

The controller being used are:
 - [`mobile_base_controller`](http://wiki.ros.org/diff_drive_controller)(diff_drive_controller): Controller for differential drive wheel systems.

 - [`joint_state_controller`](http://wiki.ros.org/joint_state_controller): This controller [simply](https://answers.ros.org/question/321078/what-does-joint_state_controller-read/) reads the JoinStates through the `JointStateInterface`, convert them to [sensor_msgs/JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html) messages and publishes them.

The controllers expects some parameters for their proper configuration. They are configured via [`config/carpincho_control.yaml`](config/carpincho_control.yaml) parameters.
