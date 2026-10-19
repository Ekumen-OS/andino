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
│   └── bsp/            # Board Support Package: HAL implementations (arduino/, avr/, stm32/)
├── src/                # Implementation files, mirroring the layers above
│   └── main.cpp        # Wires the BSP implementations of the selected board into the App
├── boards/             # Board packages: pins, serial settings and configuration of each board
│   ├── arduino_uno/
│   ├── arduino_nano/
│   └── stm32_bluepill/
├── test/               # Unit tests organized by layer (run on the host)
│   ├── app/
│   └── drivers/
├── docker/             # Containerized development environment
├── platformio.ini      # Build system configuration
└── README.md
```

The `app` and `drivers` layers only depend on the abstract `hal` interfaces, so they can be compiled and unit tested on a host machine. The `bsp` layer provides the specializations of those interfaces, and `src/main.cpp` is the only place where both are connected, using the board package of the board being built.

---

## How it works

1. **Closed-loop control** – `setspd` enables a PID controller per wheel. Every 33 ms (`pid_rate_hz`, 30 Hz) the controller compares the ticks counted in that period with the target and updates the motor PWM. The target is converted from ticks/s to ticks per period using integer division, so the speed resolution is `pid_rate_hz` ticks/s. `setspd 0 0` stops the motors and disables the PID.
2. **Open-loop control** – `setpwm` disables the PID and drives the motors with the given PWM value directly (positive values go forward, negative values backward).
3. **Auto-stop** – If no `setspd` or `setpwm` command is received for `auto_stop_window_ms` (3 s), the motors are stopped and the PID is disabled, so a robot never keeps driving after losing its host. Keep sending the command periodically to keep the robot moving.

---

## Building & Flashing

The firmware is built with [PlatformIO](https://platformio.org/). You can build, test, and flash it using the provided Docker image (recommended, no local setup needed). These are the supported boards:

| Board | PlatformIO environment | Board package |
|-------|------------------------|---------------|
| Arduino Uno | `uno` | [`boards/arduino_uno`](./boards/arduino_uno) |
| Arduino Nano | `nanoatmega328` | [`boards/arduino_nano`](./boards/arduino_nano) |
| STM32 Blue Pill (STM32F103C8T6) | `bluepill_f103c8` | [`boards/stm32_bluepill`](./boards/stm32_bluepill) |

Only Docker and Docker Compose are required to be installed on your system, no local compilers or PlatformIO installations are needed. See the [Docker README](./docker/README.md) for instructions and further details.

### Supporting a new board

What changes from one board to another is kept in a **board package**, so adding a board doesn't touch the application:

1. Create `boards/<board>/andino/board.h`. It defines `andino::Board` (the pins and the serial baud rate) and `andino::kAppConfig` (the application configuration). Use [`boards/arduino_uno/andino/board.h`](./boards/arduino_uno/andino/board.h) as a model: it documents every pin. A board that is identical to an existing one can just include its package, as the Arduino Nano does.
2. Add a PlatformIO environment for it in [`platformio.ini`](./platformio.ini), which selects the package with `-I boards/<board>` and builds the BSP implementations that fit its microcontroller (`bsp/arduino` for the Arduino API, plus the platform specific ones, like `bsp/avr`). Add it to `default_envs` so CI builds it.
3. If the board needs a different implementation of a HAL interface (for example, interrupts on another microcontroller family), add it in a new `bsp/<platform>` directory, like `bsp/stm32` for the Blue Pill.

---

## Hardware

### Wiring

The pin assignment is defined in the board package of each board.

#### Arduino Uno and Nano

Defined in [`boards/arduino_uno/andino/board.h`](./boards/arduino_uno/andino/board.h) (the Arduino Nano uses the same one):

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

The encoder inputs use interrupts (pin change interrupts on the AVR boards). The IMU is an optional Adafruit BNO055 on the I2C bus, the firmware works without it, and the `hasimu` command tells whether it was detected at boot.

#### STM32 Blue Pill

> **Note**: the support for this board is built and statically analyzed by the CI, but it hasn't been validated on a robot yet.

Defined in [`boards/stm32_bluepill/andino/board.h`](./boards/stm32_bluepill/andino/board.h):

| Function | Pin |
|----------|-----|
| Left encoder, channel A | `PB12` |
| Left encoder, channel B | `PB13` |
| Right encoder, channel A | `PB14` |
| Right encoder, channel B | `PB15` |
| Left motor, backward (PWM, `TIM2_CH1`) | `PA0` |
| Left motor, forward (PWM, `TIM2_CH2`) | `PA1` |
| Left motor, enable | `PA4` |
| Right motor, backward (PWM, `TIM2_CH3`) | `PA2` |
| Right motor, forward (PWM, `TIM2_CH4`) | `PA3` |
| Right motor, enable | `PA5` |
| IMU, I2C SCL (`I2C1`) | `PB6` |
| IMU, I2C SDA (`I2C1`) | `PB7` |

The encoder pins must have different numbers (here 12 to 15), because the pins with the same number of every port share an external interrupt line. `PA11` and `PA12` are used by USB, and `PA13` and `PA14` by the debugger.

The serial port of the Blue Pill is its **USB port** (USB CDC), so it needs no USB-to-serial adapter. It shows up on the host as a serial port, usually `/dev/ttyACM0`, and the baud rate doesn't apply. The board is flashed with an **ST-Link** connected to its SWD header (`SWDIO`, `SWCLK`, `GND` and `3.3V`).

### Configuration

The application is configured with an `App::Config` (see [`include/andino/app/app.h`](./include/andino/app/app.h)), which is defined in the board package (`andino::kAppConfig`) and passed to the `App` by [`src/main.cpp`](./src/main.cpp):

| Setting | Default | Description |
|---------|---------|-------------|
| `auto_stop_window_ms` | `3000` | Time without a motor command after which the motors are stopped [ms] |
| `pid_rate_hz` | `30` | PID computation rate [Hz] |
| `pid_gains` | `{30, 10, 0, 10}` | Initial PID tuning gains `{kp, kd, ki, ko}` (they can be changed with `setpid`) |

The serial baud rate (`57600`) is `Board::kSerialBaudRate`, in the board package, and it must match the one used by the host. The maximum motor speed (PWM duty cycle `255`) is `Motor::kMaxSpeed`, in [`include/andino/drivers/motor.h`](./include/andino/drivers/motor.h).

---

## Serial Interface

The firmware is controlled through a text-based protocol over the serial port (57600 baud, except on the Blue Pill, whose serial port is USB and has no baud rate).

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
| `ver` | Get the firmware version (the package version) |  | `ver` | `0.2.0` |
| `getencimu` | Get the encoder tick values and the IMU data |  | `getencimu` | `<left> <right> <orientation_x> <orientation_y> <orientation_z> <orientation_w> <angular_velocity_x> <angular_velocity_y> <angular_velocity_z> <linear_acceleration_x> <linear_acceleration_y> <linear_acceleration_z>` |

The IMU values are the orientation as a quaternion, the angular velocity in rad/s and the linear acceleration in m/s². `getencimu` replies `[ERROR] IMU unavailable` if no IMU was detected.

### Test it!

Create a serial connection (at 57600 baud, unless the board is a Blue Pill), e.g. with the PlatformIO serial monitor (`pio device monitor`, or through Docker: `docker compose -f docker/compose.yaml run --rm dev pio device monitor`). Remember to terminate every message with a carriage return.

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

---

## Hardware validation

[`tools/hw_check.py`](./tools/hw_check.py) validates that the hardware connections to the MCU and the firmware flashing work as expected. Connect the board through USB, flash the firmware, and run (from the `andino_firmware` directory):

```bash
docker compose -f docker/compose.yaml run --rm dev python tools/hw_check.py --port /dev/ttyUSB0
```

The serial port of the board can be found with `docker compose -f docker/compose.yaml run --rm dev pio device list` (usually `/dev/ttyUSB0` for the Arduino boards and `/dev/ttyACM0` for the Blue Pill). If `--port` is omitted, the tool will attempt to use the only device connected, if there is one.

The tool runs all the checks, one after the other:

| Check | What it verifies |
|-------|------------------|
| Firmware | The MCU answers, the firmware booted correctly and its version (`ver`) is reported. |
| IMU | The sensor is detected and reports sane values at rest. |
| Encoders at rest | The encoder counts are readable and stable. |
| Encoders by hand | Both channels of each encoder toggle and are counted while you rotate the wheel by hand. |
| Motors | For each wheel and direction: the commanded wheel moves, in the right direction, and its encoder counts it. |
| Closed loop | The measured speed matches the one requested with `setspd`. |

The checks have their own unit tests, run against a simulated robot and user:

```bash
docker compose -f docker/compose.yaml run --rm dev python -m unittest discover -s tools
```
