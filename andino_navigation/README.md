# Nav2 with Andino robot
*Note*: Nav2 is only configured to work with the simulation for the moment.
 Support to real robot is forthcoming.

## Build

Install package dependencies:

```
rosdep install --from-paths src -i -y
```

Build the package:

```
colcon build
```

Note: `--symlink-install` can be added if needed.

Finally, source the install folder
```
. install/setup.bash
```

# Usage 

This package has the next option to be executed.

## Andino with nav2 in Gazebo Simulation

This launch file use the [turtlebot3_world](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master) like world example which has an Apache 2 license.

```
 . /usr/share/gazebo/setup.bash
 ros2 launch andino_navigation andino_simulation_navigation.launch.py 
```

To test the navigation inside rviz:

- click in 2D pose estimate button and select the initial pose of the robot
- click in nav2 Goal button and select the final point.
- the robot will start to move to the selected goal.

You test adding obstacles inside the Gazebo simulation or use the rviz button Waypoint/ nav through Poses mode to select sequential targets.

This package has been tested with the Andino robot with `diff drive plugin` in gazebo. If you change the world you should change the map but also it is recommended to tune navigation [parameters](params/nav2_params.yaml).
