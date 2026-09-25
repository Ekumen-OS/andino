# andino_firmware_rust
Rust Firmware code to be run in the arduino microcontroller for proper control of the motors of the robot.

This firmware is an alternative to the existing [Arduino C](https://github.com/Ekumen-OS/andino/tree/main/andino_firmware) code, but written in embedded Rust (`no_std`) using the [AVR-HAL](https://github.com/rahix/avr-hal) -> `arduino_hal` crate.

## Installation 

### Build locally

Install dependencies:

```bash
# AVR toolchain (Ubuntu/Debian)
sudo apt-get install gcc-avr binutils-avr avr-libc avrdude

# ravedude (flash tool)
cargo +stable install ravedude
```

Compile app program and flash it:

```bash
# Build firmware
cargo build --release

# Flash to the board
cargo run --bin app --release
```

**Note:** On Linux, grant serial port permissions if needed:

```bash
sudo chmod a+rw /dev/ttyUSB0
```

> Always compile in `--release` mode. Debug builds have been known to cause issues on the ATmega328p.

### Docker

Run the following command to build and run the container. Inside, compile and flash the app program:

```bash
docker compose -f docker/docker-compose.yaml run --rm rust bash
$ cargo run --bin app --release
```

## Arduino HAL examples

The repository contains as well alternative example binary programs to serve as reference tutorials for how to write embedded Rust code for an Arduino Nano (or in general any AVR Microncontroller).
Look at the `/examples` folder, and execute the following command for running the desire binary:

```bash
cargo run --bin <example_name> --release
# e.g.: cargo run --bin motor --release
```

## Description 

Once flashed, the robot communicates over serial at the configured baud rate (ravedude is configured at 57600 baud). Commands are single-letter strings, optionally followed by space-separated integer arguments, terminated with a newline.

| Command | String | Args | Description |
|---------|--------|------|-------------|
| Read encoders | `e` | — | Returns left and right encoder tick counts |
| Reset encoders | `r` | — | Resets both encoder counts to zero |
| Set motors speed | `m <left> <right>` | ticks/s | Sets target speed via PID closed-loop control |
| Set motors PWM | `o <left> <right>` | 0–255 | Sets motor PWM directly, bypassing PID |
| Set PID gains | `u <kp> <kd> <ki> <ko>` | integers | Updates PID tuning gains for both motors |

Responses are plain ASCII lines. On success most commands reply `OK`. Read commands reply with the requested values. Errors reply with `ERROR: <message>`.

**Auto-stop:** if no `m` or `o` command is received within the auto-stop window, the motors are halted automatically.

## Test it!

Use the `/examples` binary programs to test specific features of the system (motor, digital pins, encoders, etc) or flash the app program and try the following options:

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
