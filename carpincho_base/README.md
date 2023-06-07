# carpincho_base

## Description

The hardware-software-ros interaction in the `carpincho` project is developed using [ROS2 Control](https://control.ros.org/master/index.html).

This package:
 - Provides a communication with microcontroller:
   - `carpincho_base::MotorDriver` is in charge of the Serial communication for commanding the motors.
     - An application is provided for evaluating the communication: Check `applications/motor_driver_demo.cpp`. To use this application simply execute `motor_driver_demo --help` to see the options.
 - TODO

TODO

## Launch files

TODO

## Notes

 - Serial communication: In case the serial port is denied to be open, probably the user should be added to the `plugdev` and `dialout` groups:
    ```
    sudo usermod -a -G dialout $USER
    sudo usermod -a -G plugdev $USER
    ```
