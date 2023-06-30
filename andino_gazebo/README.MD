
# Andino simulation

WIP

```
ros2 launch andino_gazebo andino_one_robot.launch.py initial_pose_x:=3.0
```

## Spawn an Andino robot

```
ros2 launch andino_gazebo spawn_robot.launch.py initial_pose_x:=3.0 entity:=andino robot_description_topic:=/andino/robot_description
```

Spawn multiple andino robots has some issues, first one the tf of the diff drive is published in tf and not in namespace/tf, so the re-mmapings of the static_tf is deactivated to avoid confusion.

## ros_control (not tested yet)

It is in the xacro file but has not been tested, it is necessary to made another launch with ros control because I do not have how to pass the parameter to the xacro converter.

