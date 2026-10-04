# andino_firmware

Firmware for the Arduino microcontroller that controls the Andino robot. It drives the two wheel motors with a closed-loop speed controller, counts the wheel encoder ticks, and reads an optional BNO055 IMU. It talks to the host computer over a simple text-based serial protocol (see [Serial Interface](#serial-interface)).

---

## Project Structure

The project follows a layered architecture organized within the standard PlatformIO directory structure:

```text
andino_firmware/
├── include/andino/     # Header files organized by layer
│   ├── app/            # Application logic
│   ├── drivers/        # Hardware-independent drivers
│   ├── hal/            # Hardware Abstraction Layer (interfaces)
│   └── bsp/            # Board Support Package (Arduino specialization)
├── src/                # Implementation files, mirroring the layers above
│   └── main.cpp        # Wires the Arduino specializations into the App and runs it
├── test/               # Unit tests organized by layer (run on the host)
│   ├── app/
│   └── drivers/
├── docker/             # Containerized development environment
├── platformio.ini      # Build system configuration
└── README.md
```

The `app` and `drivers` layers only depend on the abstract `hal` interfaces, so they can be compiled and unit tested on a host machine. The `bsp` layer provides the Arduino specializations of those interfaces, and `src/main.cpp` is the only place where both are connected.

---

## How it works

1. **Closed-loop control** – `setspd` enables a PID controller per wheel. Every 33 ms (`kPidRate`, 30 Hz) the controller compares the ticks counted in that period with the target and updates the motor PWM. The target is converted from ticks/s to ticks per period using integer division, so the speed resolution is `kPidRate` ticks/s. `setspd 0 0` stops the motors and disables the PID.
2. **Open-loop control** – `setpwm` disables the PID and drives the motors with the given PWM value directly (positive values go forward, negative values backward).
3. **Auto-stop** – If no `setspd` or `setpwm` command is received for `kAutoStopWindow` (3s), the motors are stopped and the PID is disabled, so a robot never keeps driving after losing its host. Keep sending the command periodically to keep the robot moving.

---

## Building & Flashing

The firmware is built with [PlatformIO](https://platformio.org/). You can build, test, and flash it using the provided Docker image (recommended, no local setup needed). There are two firmware targets:

| Board | PlatformIO environment |
|-------|------------------------|
| Arduino Uno | `uno` |
| Arduino Nano | `nanoatmega328` |

Only Docker and Docker Compose are required to be installed on your system, no local compilers or PlatformIO installations are needed. See the [Docker README](./docker/README.md) for instructions and further details.

---

## Hardware

### Wiring

The pin assignment is defined in [`include/andino/app/hw.h`](./include/andino/app/hw.h):

| Function | Pin | Arduino pin |
|----------|-----|-------------|
| Left encoder, channel A | `PD2` | `2` |
| Left encoder, channel B | `PD3` | `3` |
| Right encoder, channel A | `PC2` | `A2` (`16`) |
| Right encoder, channel B | `PC3` | `A3` (`17`) |
| Left motor, backward (PWM) | `PD6` | `6` |
| Left motor, forward (PWM) | `PB2` | `10` |
| Left motor, enable | `PB5` | `13` |
| Right motor, backward (PWM) | `PD5` | `5` |
| Right motor, forward (PWM) | `PB1` | `9` |
| Right motor, enable | `PB4` | `12` |
| IMU, I2C SCL | `PC5` | `A5` (`19`) |
| IMU, I2C SDA | `PC4` | `A4` (`18`) |

*Note: the enable input of an L298N motor driver can be jumped directly to 5V if the board has a jumper for it.*

The encoder inputs use pin change interrupts. The IMU is an optional Adafruit BNO055 on the I2C bus, the firmware works without it, and the `hasimu` command tells whether it was detected at boot.

### Configuration

Application constants are defined in [`include/andino/app/constants.h`](./include/andino/app/constants.h):

| Constant | Default | Description |
|----------|---------|-------------|
| `kBaudrate` | `57600` | Serial port baud rate |
| `kAutoStopWindow` | `3000` | Time without a motor command after which the motors are stopped [ms] |
| `kPwmMax` | `255` | Maximum PWM duty cycle |
| `kPidRate` | `30` | PID computation rate [Hz] |
| `kPidKp` | `30` | Default PID proportional gain |
| `kPidKd` | `10` | Default PID derivative gain |
| `kPidKi` | `0` | Default PID integral gain |
| `kPidKo` | `10` | Default PID output gain |

---

## Serial Interface

The firmware is controlled through a text-based protocol over the serial port (57600 baud).

### Commands

The command names are defined in [`include/andino/app/commands.h`](./include/andino/app/commands.h).

| Command | Description | Args | Example | Result |
| --- | --- | --- | --- | --- |
| `getch` | Read an encoder digital input value | `encoder` (0: left, 1: right), `channel` (0: A, 1: B) | `getch 0 0` | `0` or `1` |
| `getenc` | Get the encoder tick values |  | `getenc` | `<left> <right>` |
| `rstenc` | Reset the encoder tick values to zero |  | `rstenc` | `[OK]` |
| `setspd` | Set the closed-loop speed of the motors [ticks/s] | `left_tps` `right_tps` | `setspd 700 700` | `[OK]` |
| `setpwm` | Set the open-loop speed of the motors [PWM, `-255` to `255`] | `left_pwm` `right_pwm` | `setpwm 255 255` | `[OK]` |
| `setpid` | Set the PID tuning gains (`ko` must be greater than zero) | `kp` `kd` `ki` `ko` | `setpid 30 10 0 10` | `[OK]` |
| `hasimu` | Get whether the IMU was detected |  | `hasimu` | `0` if not connected, `1` if connected |
| `getencimu` | Get the encoder tick values and the IMU data |  | `getencimu` | `<left> <right> <orientation_x> <orientation_y> <orientation_z> <orientation_w> <angular_velocity_x> <angular_velocity_y> <angular_velocity_z> <linear_acceleration_x> <linear_acceleration_y> <linear_acceleration_z>` |

The IMU values are the orientation as a quaternion, the angular velocity in rad/s and the linear acceleration in m/s². `getencimu` replies `[ERROR] IMU unavailable` if no IMU was detected.

### Test it!

Create a serial connection at 57600 baud, e.g. with the PlatformIO serial monitor (`pio device monitor`, or through Docker: `docker compose -f docker/compose.yaml run --rm dev pio device monitor`). Remember to terminate every message with a carriage return.

* **Open loop verification**
  - Send `setpwm 255 255` to go full speed.
  - Send `setpwm 0 0` to stop it.

* **Read the encoders**
  - Send `getenc` to get the encoder values.

* **Get the ticks per revolution of your motor**
  - First, reset the encoders to zero with `rstenc`.
  - Then rotate your motors as many revolutions as you want (say 10) and divide the encoder ticks by the number of revolutions. Save this value, it is the calibration for the control loop.

* **Closed loop verification**
  - Send `setspd <tps> <tps>`, where `tps` stands for ticks per second. For example, if your motor-encoder system gets 700 ticks per revolution, `setspd 700 700` rotates both motors at 1 revolution per second (~6.28 rad/s).
