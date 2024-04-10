# Andino Apps

This package contains more complex launh files that execute applications with the andino robot.

# Usage

## Gazebo classic simulation + Nav2

A launch file for running the andino_gz_classic simulation and the nav2 stack is provided.
It uses the [turtlebot3_world](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master) world (_Apache 2 license_) by default.

```
 ros2 launch andino_apps andino_simulation_navigation.launch.py
```

To test the navigation inside rviz:

- click in 2D pose estimate button and select the initial pose of the robot
- click in nav2 Goal button and select the final point.
- the robot will start to move to the selected goal.

You test adding obstacles inside the Gazebo simulation or use the rviz button Waypoint/ nav through Poses mode to select sequential targets.

This package has been tested with the Andino robot with `diff drive plugin` in gazebo. If you change the world you should change the map but also it is recommended to tune navigation [parameters](params/nav2_params.yaml).
