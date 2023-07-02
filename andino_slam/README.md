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
