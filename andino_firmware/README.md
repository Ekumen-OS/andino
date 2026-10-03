# andino_firmware

Firmware code to be run in the arduino microcontroller for proper control of the motors of the robot.

## Connection

Check `encoder_driver.h` and `motor_driver.h` files to check the expected pins for the connection.

## Installation

### Arduino
In Arduino IDE, go to `tools->Manage Libraries ...` and install:
- "Adafruit BNO055"

Verify and Upload `andino_firmware.ino` to your arduino board.

### PlatformIO
1. Install dependencies `sudo apt-get install python3.10-venv`
2. Install platformio
```
curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 /tmp/get-platformio.py
```
3. Add platformio to your $PATH:
```
echo "PATH=\"\$PATH:\$HOME/.platformio/penv/bin\"" >> $HOME/.bashrc
source $HOME/.bashrc
```
4. Build and upload the firmware
   - If you're using an arduino uno `pio run --target upload -e uno`
   - If you're using an arduino nano `pio run --target upload -e nanoatmega328`

## Description

Via `serial` connection (57600 baud) it is possible to interact with the microcontroller. The interface is described in the [commands.h](src/commands.h) file. Here are the most used commands:


 - Get encoder values: `'getenc'`
 - Set open-loop speed for the motors[pwm] `'setpwm <left> <right>'`
   - Example to move forward full speed: `'setpwm 255 255'`
   - Range `[-255 -> 255]`
 - Set closed-loop speed for the motors[ticks/sec] `'setspd <left> <right>'`
   - Important!: See the `Test it!` section.
 - Set PID values: `'setpid <kp> <kd> <ki> <offset>'`

Note: Remember the carriage return character at the end of the message.


## Test it!

A serial port connection must be created at 57600 bauds. You can use the serial monitor from Arduino IDE for example.

* Open loop verification:
  - Send `setpwm 255 255` to go full speed
  - Send `setpwm 0 0` to stop it.

* Read the encoders
  - Send `getenc` to get the encoders values.

* Get the ticks per revolution of your motor.
  - First set the encoders to zero, (resetting with `rstenc`).
  - Then rotate your motors as many revs you want,(say 10 for example) and then divide the encoder ticks per the number of revs. -> Then you get the ticks per revolution. Save this value, it is calibration for the control loop.

* Closed loop verification
  - Send `setspd <tps> <tps>` where `tps` stands for `ticks per second`. For example if your motor-encoder system gets 700 ticks per revolution then sending `setspd 700 700` will rotate both motors at 1 rev per sec. (~3.14rad/sec)

## Commands

Every command replies with exactly one line terminated by `\n`:

* Read commands reply with their data, space-separated (see the table below).
* Write commands reply with `[OK]`.
* Any failure replies with `[ERROR] <description>`, e.g. `[ERROR] Unknown command`, `[ERROR] Invalid arguments` (missing, extra, non-numeric or out-of-range arguments) or `[ERROR] IMU unavailable`.

| Command | Description | Args | Example | Result |
| --- | --- | --- | --- | --- |
| `a` | Read Analog GPIO pin | pin_number | `a 0` |  |
| `getch` | Read encoder digital input value | encoder (0: left, 1: right) channel (0: A, 1: B) | `getch 0 0` | `0` or `1` |
| `getenc` | Get encoder tick values |  | `getenc` | `<left> <right>` |
| `rstenc` | Reset encoder values |  | `rstenc` |  |
| `setspd` | Set closed-loop speed for the motors[ticks/sec] | left_tps right_tps | `setspd 700 700` |  |
| `setpwm` | Set open-loop speed for the motors[pwm] | left_pwm right_pwm | `setpwm 255 255` |  |
| `setpid` | Set PID values | kp kd ki offset | `setpid 30 20 10 50` |  |
| `hasimu` | Get if IMU is connected |  | `hasimu` | `0` if not connected, `1` if connected |
| `getencimu` | Get IMU data and encoder tick values |  | `getencimu` | `<left> <right>  <orientation_X> <orientation_Y> <orientation_Z> <orientation_W> <angular_velocity_X> <angular_velocity_Y> <angular_velocity_Z> <linear_acceleration_X> <linear_acceleration_Y> <linear_acceleration_Z>` |
