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

### Run nav stack

```sh
ros2 launch andino_navigation bringup.launch.py map:=<path-to-my-map-yaml-file>
```

By default, [config file](params/nav2_params.yaml) is used. For using a custom param file use:
```sh
ros2 launch andino_navigation bringup.launch.py map:=<path-to-my-map-yaml-file> params_file:=<path-to-my-param-file>
```

## Simulation

A launch file for running the andino_gz_classic simulation and the nav2 stack is provided.
It uses the [turtlebot3_world](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master) world (_Apache 2 license_) by default.

```
 ros2 launch andino_navigation andino_simulation_navigation.launch.py
```

To test the navigation inside rviz:

- click in 2D pose estimate button and select the initial pose of the robot
- click in nav2 Goal button and select the final point.
- the robot will start to move to the selected goal.

You test adding obstacles inside the Gazebo simulation or use the rviz button Waypoint/ nav through Poses mode to select sequential targets.

This package has been tested with the Andino robot with `diff drive plugin` in gazebo. If you change the world you should change the map but also it is recommended to tune navigation [parameters](params/nav2_params.yaml).
