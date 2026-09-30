# Andino with Pixi

[Pixi](https://pixi.sh) is a fast, cross-platform package manager built on the conda ecosystem.
It provides a reproducible development environment for andino without requiring a system ROS installation or Docker.

## Prerequisites

1. **Install pixi** (one-liner):

    ```bash
    curl -fsSL https://pixi.sh/install.sh | bash
    ```

2. **Install `libserial-dev`** — this C++ serial library is not yet available in conda-forge and must be provided by the system package manager:

    ```bash
    sudo apt install libserial-dev
    ```

## Quick Start

```bash
# Clone and enter the repository
git clone https://github.com/Ekumen-OS/andino.git
cd andino

# Install the default environment (ROS 2 Humble)
pixi install

# Build all packages
pixi run build

# Open a shell with the full environment activated
pixi shell
```

After `pixi run build` completes, the `install/setup.sh` workspace overlay is sourced automatically on every subsequent `pixi shell` or `pixi run` invocation (see [`activate.sh`](activate.sh)).

## Available Tasks

| Task | Command | Description |
|------|---------|-------------|
| `build` | `pixi run build` | Build all packages with `colcon build --symlink-install` |
| `test` | `pixi run test` | Run all tests (depends on `build`) |
| `bringup` | `pixi run bringup` | Launch the full robot bringup |
| `teleop-keyboard` | `pixi run teleop-keyboard` | Keyboard teleoperation |
| `teleop-joystick` | `pixi run teleop-joystick` | Joystick teleoperation |
| `view-description` | `pixi run view-description` | Visualize the robot URDF in RViz |

## Environments

The workspace ships two environments:

| Environment | ROS distro | Command prefix |
|-------------|-----------|----------------|
| `default` | ROS 2 Humble | `pixi run <task>` |
| `jazzy` | ROS 2 Jazzy | `pixi run -e jazzy <task>` |

Examples:

```bash
# Build against ROS 2 Humble (default)
pixi run build

# Build against ROS 2 Jazzy
pixi run -e jazzy build

# Open a Jazzy shell
pixi shell -e jazzy
```

## Clean Build

To remove build artifacts and start fresh:

```bash
rm -rf build install log
pixi run build
```

## Comparison with Docker

| | Docker | Pixi |
|-|--------|------|
| Isolation | Full container | Conda environment |
| ROS install | System (`apt`) | Conda (RoboStack) |
| Hardware access | Requires device mapping | Direct host access |
| GUI tools (RViz) | Requires X11 forwarding | Works natively |
| Setup overhead | Pull/build image | `pixi install` |

Both approaches are supported and maintained. Docker is recommended for deployment on the robot; pixi is recommended for desktop development.

## Troubleshooting

**`ros2: command not found` in `pixi shell`**
Run `pixi run build` first. The `ros2` CLI and all workspace overlays are only available after the initial build populates `install/setup.sh`.

**`libserial` headers not found during build**
Install the system package: `sudo apt install libserial-dev`.

**Environment solve fails**
Ensure you are on a `linux-64` host (x86-64 Linux). The `linux-aarch64` (Raspberry Pi) platform is not yet included because some RoboStack packages (`joy-linux`, `v4l2-camera`) do not publish `linux-aarch64` builds.
