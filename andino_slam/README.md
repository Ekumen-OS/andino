# andino_slam

## Description

For achieving SLAM we rely on the great [`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox) package.

This package provides a launch file that wraps `slam_toolbox`'s `online_async_launch.py`, forwarding Andino-specific configuration. This ensures compatibility across ROS 2 distributions (Humble, Jazzy) by delegating lifecycle node management to `slam_toolbox` itself.

## Usage

After the robot bringup, simply execute the provided launch file.

```sh
ros2 launch andino_slam slam_toolbox_online_async.launch.py
```

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `slam_params_file` | `andino_slam/config/slam_toolbox_online_async.yaml` | Full path to the ROS 2 parameters file for the `slam_toolbox` node |
| `use_sim_time` | `false` | Use simulation/Gazebo clock |

For example, to pass a custom parameters file:

```sh
ros2 launch andino_slam slam_toolbox_online_async.launch.py slam_params_file:=<my_path>
```

Or to use simulation time:

```sh
ros2 launch andino_slam slam_toolbox_online_async.launch.py use_sim_time:=true
```

For saving the map you can use `map_saver_cli` node provided by Nav2.
```sh
ros2 run nav2_map_server map_saver_cli -f <my-map-name>
```

You can modify the threshold for free space (0.25) and occupied space (0.65) by using
`--free` and `--occ` arguments.

```sh
ros2 run nav2_map_server map_saver_cli --free 0.15 -f <my-map-name>
```
More information at:
 - https://navigation.ros.org/configuration/packages/configuring-map-server.html
 - https://github.com/ros-planning/navigation2/tree/main/nav2_map_server


Once you have the map saved, you can navigate on it!
Go to [`andino_navigation`](../andino_navigation/README.md) to learn how.
