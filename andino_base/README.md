# andino_base

## Description

The hardware-software-ros interaction in the `andino` project is developed using [ROS 2 Control](https://control.ros.org/master/index.html).

This package:
 - Implements `andino`'s [hardware interface](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html).
 - Provides a communication with microcontroller:
   - `andino_base::MotorDriver` class is in charge of the Serial communication for commanding the motors.
     - An application is provided for evaluating the communication: Check `applications/motor_driver_demo.cpp`. To use this application simply execute `motor_driver_demo --help` to see the options.
   - This communication module is used by the hardware interface implementation.

## Hardware Interface

In `ros2_control` hardware system components are libraries, dynamically loaded by the controller manager using the pluginlib interface.

For extra information about the hardware components see [Hardware Components](https://control.ros.org/master/doc/getting_started/getting_started.html#overview-hardware-components).

The hardware interface accepts some parameters that are passed via the urdf description within the `ros2_control` tag (Check [`andino_description`](../andino_description/urdf/include/andino_control.urdf.xacro))

| Params               | Description |
| :---                 |    :----:   |
| left_wheel_name      | Name of the left wheel joint. |
| right_wheel_name     | Name of the right wheel joint. |
| serial_device        | Path to the serial device. |
| baud_rate            | Baud rate of the serial communication. |
| timeout              | Timeout for the communication. |
| enc_ticks_per_rev    | Encoder ticks per revolution of the wheel. |

### State interfaces

This hardware interface implements the following state interfaces per joint (for left and right joint):
 - *Position*: The position is obtained via encoder information from the microcontroller.
 - *Velocity*: Velocity is calculated via encoder information from the microcontroller.

### Command interfaces

This hardware interface uses the following command interfaces per joint (for left and right joint):
 - *Velocity*: The velocity received (rad/s) is traduced to microcontroller's velocity nomenclature for the motors.


## Motor Driver Application

An application for testing the connection with the microcontroller is provided.
After installing this package the application called `motor_driver_demo` can be used.
```
motor_driver_demo --help
```

This application allows verifying the communication with the microcontroller for controlling the motors. Commands for reading the encoders or individually setting a velocity for the motors is some of the possibilities.

## Extra Notes

 - Serial communication: In case the serial port is denied to be open, probably the user should be added to the `plugdev` and `dialout` groups:
    ```
    sudo usermod -a -G dialout $USER
    sudo usermod -a -G plugdev $USER
    ```
