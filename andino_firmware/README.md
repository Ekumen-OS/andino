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


 - Get encoder values: `'e'`
 - Set open-loop speed for the motors[pwm] `'o <left> <right>'`
   - Example to move forward full speed: `'o 255 255'`
   - Range `[-255 -> 255]`
 - Set closed-loop speed for the motors[ticks/sec] `'m <left> <right>'`
   - Important!: See the `Test it!` section.
 - Set PID values: `'u <kp> <kd> <ki> <offset>'`

Note: Remember the carriage return character at the end of the message.


## Test it!

A serial port connection must be created at 57600 bauds. You can use the serial monitor from Arduino IDE for example.

* Open loop verification:
  - Send `o 255 255` to go full speed
  - Send `o 0 0` to stop it.

* Read the encoders
  - Send `e` to get the encoders values.

* Get the ticks per revolution of your motor.
  - First set the encoders to zero, (reeboting with `r`).
  - Then rotate your motors as many revs you want,(say 10 for example) and then divide the encoder ticks per the number of revs. -> Then you get the ticks per revolution. Save this value, it is calibration for the control loop.

* Closed loop verification
  - Send `m <tps> <tps>` where `tps` stands for `ticks per second`. For example if your motor-encoder system gets 700 ticks per revolution then sending `m 700 700` will rotate both motors at 1 rev per sec. (~3.14rad/sec)
