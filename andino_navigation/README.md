# Andino Navigation

We rely on [Nav2](https://github.com/ros-planning/navigation2) stack in order to navigate Andino.

# Usage

## Real Robot

### Prerequisites

1. Andino robot needs to be up and running:
  ```sh
  ros2 launch andino_bringup andino_robot.launch.py
  ```

2. We need a previously recorded map in order to navigate on.
   Refer to [andino_slam](../andino_slam/README.md) to learn how to record a map with Andino.

3. Run Nav Stack (see instructions in [Run nav stack](#run-nav-stack)).

## Simulation

### Prerequisites

1. Andino simulation needs to be up and running. You can use several simulators, for example Gazebo Classic:
  ```sh
  ros2 launch andino_gz_classic andino_one_robot.launch.py
  ```

2. We need a previously recorded map in order to navigate on.
   Refer to [andino_slam](../andino_slam/README.md) to learn how to record a map with Andino.

3. Run Nav Stack (see instructions in [Run nav stack](#run-nav-stack)).

<a name="runnavstack"></a>
## Run Nav Stack

```sh
ros2 launch andino_navigation bringup.launch.py map:=<path-to-my-map-yaml-file>
```

By default, [config file](params/nav2_params.yaml) is used. For using a custom param file use:

```sh
ros2 launch andino_navigation bringup.launch.py map:=<path-to-my-map-yaml-file> params_file:=<path-to-my-param-file>
```
