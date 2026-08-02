# Docker Development Environment

This directory contains the Docker and Docker Compose environment configured for developing, compiling, testing, and flashing the Andino firmware.

The Docker environment pre-installs Python 3, GCC, G++, Make, PlatformIO Core, and essential USB tools (udev, libusb) to make firmware development seamless without needing to install anything on your host machine except Docker.

---

## Directory Layout

- **`Dockerfile`**: Reusable environment container image definition.
- **`compose.yaml`**: Docker Compose configuration referencing `Dockerfile` and setting up volume mappings and USB passthrough.

---

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/install/)

---

## Quick Start

Execute these commands from the **andino_firmware** directory:

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
To compile and execute the native unit tests:
```bash
docker compose -f docker/compose.yaml run --rm dev pio test -e desktop
```

### 4. Flash the Microcontroller (USB Passthrough)

The Docker Compose configuration is set up with:
- `privileged: true`
- `/dev:/dev` mount
- `network_mode: host`

This allows the container full access to USB devices connected to the host machine. 

To upload/flash your firmware to the Arduino board:

Connect it to your host machine's USB port, then run the upload command from the root of the repository:
```bash
# Arduino Uno
docker compose -f docker/compose.yaml run --rm dev pio run -e uno --target upload

# Arduino Nano
docker compose -f docker/compose.yaml run --rm dev pio run -e nanoatmega328 --target upload
```

### 5. Interactive Shell
To launch an interactive bash shell inside the development container:
```bash
docker compose -f docker/compose.yaml run --rm dev
```
From here you can execute any `pio` commands, run linting tools, or inspect files.
