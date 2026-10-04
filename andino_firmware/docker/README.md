# Docker Development Environment

This directory contains the Docker and Docker Compose environment configured for developing, compiling, testing, and flashing the Andino firmware.

The Docker environment pre-installs Python 3, GCC, G++, Make, PlatformIO Core, and essential USB tools (udev, libusb) to make firmware development seamless without needing to install anything on your host machine except Docker.

---

## Directory Layout

- **`Dockerfile`**: Reusable environment container image definition.
- **`requirements.txt`**: Pinned versions of the Python packages installed in the image.
- **`compose.yaml`**: Docker Compose configuration referencing `Dockerfile` and setting up volume mappings, the PlatformIO cache and USB passthrough.

---

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/install/)

---

## Quick Start

Execute all the commands in this document from the **`andino_firmware`** directory (the parent of this one).

### 1. Build the Docker Image
To build the development environment image:
```bash
docker compose -f docker/compose.yaml build
```

### 2. Compile the Firmware
To build the firmware target `Arduino Uno` inside the container:
```bash
docker compose -f docker/compose.yaml run --rm dev pio run -e uno
```

To build the firmware target `Arduino Nano` inside the container:
```bash
docker compose -f docker/compose.yaml run --rm dev pio run -e nanoatmega328
```

### 3. Run Native Unit Tests
To compile and execute the native unit tests (they are built with AddressSanitizer and UndefinedBehaviorSanitizer enabled):
```bash
docker compose -f docker/compose.yaml run --rm dev pio test -e desktop
```

### 4. Run Static Code Analysis
To run the static code analysis checks (Cppcheck) on the firmware sources:
```bash
docker compose -f docker/compose.yaml run --rm dev pio check
```

### 5. Interactive Shell
To launch an interactive bash shell inside the development container:
```bash
docker compose -f docker/compose.yaml run --rm dev
```
From here you can execute any `pio` commands, run linting tools, or inspect files.

---

## Flashing the Microcontroller (USB Passthrough)

The Docker Compose configuration is set up with:
- `privileged: true`
- `/dev:/dev` mount
- `network_mode: host`

This allows the container full access to USB devices connected to the host machine. To upload/flash your firmware to the Arduino board:

1. Connect the Arduino to your host machine's USB port.
2. Check that the board is detected (it usually shows up as `/dev/ttyUSB0` or `/dev/ttyACM0`):
   ```bash
   docker compose -f docker/compose.yaml run --rm dev pio device list
   ```
3. Run the upload command:
   ```bash
   # Arduino Uno
   docker compose -f docker/compose.yaml run --rm dev pio run -e uno --target upload

   # Arduino Nano
   docker compose -f docker/compose.yaml run --rm dev pio run -e nanoatmega328 --target upload
   ```

---

## Talking to the Firmware (Serial Monitor)

With the firmware flashed, you can open a serial connection to the board (57600 baud, as configured in `platformio.ini`) to send the firmware commands described in the [firmware README](../README.md#serial-interface):
```bash
docker compose -f docker/compose.yaml run --rm dev pio device monitor
```

Press `Ctrl+C` to exit the monitor.

---

## PlatformIO Cache

The PlatformIO packages (platforms, toolchains and libraries) are stored in a named Docker volume, `andino_firmware_pio_cache`, so they are downloaded only once and survive the removal of the container. The project directory is mounted as the container's working directory, so the build output (`.pio/`) is created in your host's `andino_firmware` directory, owned by your user.

To start from scratch (e.g. to fix a corrupted package cache), remove the volume and the image:
```bash
docker compose -f docker/compose.yaml down --volumes
docker image rm andino-firmware-dev:latest
```
