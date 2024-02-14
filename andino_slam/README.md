# andino_slam

## Description

For achieving SLAM we rely on the great [`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox/tree/ros2) package.

## Usage

After the robot bring up, simply execute the provided launch file.

```
ros2 launch andino_slam slam_toolbox_online_async.launch.py
```

Several configuration can be forwarded to the `slam_toolbox_node`. By default, the configuration parameters are obtained from [config/slam_toolbox_only_async.yaml](config/slam_toolbox_online_async.yaml). In case a custom file is wanted to be passed, simply use the launch file argument for indicating the path to a new file.

```
ros2 launch andino_slam slam_toolbox_online_async.launch.py slams_param_file:=<my_path>
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
