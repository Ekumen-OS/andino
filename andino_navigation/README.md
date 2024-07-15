# Andino Navigation

We rely on [Nav2](https://github.com/ros-planning/navigation2) stack in order to navigate Andino.

# Usage

## Prerequisites
  1. Run the mobility stack in a real Andino robot or a simulated one:

_Real robot_
```
ros2 launch andino_bringup andino_robot.launch.py
```

_Example with Gazebo Sim_
All the simulation engines supported by Andino live in other repositories (check the `related projects` section on main README)
```
ros2 launch andino_gz andino_gz.launch.py
```

  1. Provide a recorded map. Refer to [andino_slam](../andino_slam/README.md) to learn how to record a map with Andino.

## Run Nav Stack

```sh
ros2 launch andino_navigation bringup.launch.py map:=<path-to-my-map-yaml-file>
```

By default, [config file](params/nav2_params.yaml) is used. For using a custom param file use:

```sh
ros2 launch andino_navigation bringup.launch.py map:=<path-to-my-map-yaml-file> params_file:=<path-to-my-param-file>
```
