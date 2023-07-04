
# Gazebo Classic simulation for the Andino robot 

## Build

```
rosdep install --from-paths src --ignore-src -i -y
colcon build --symlink-install
. /usr/share/gazebo/setup.bash
. install/setup.bash
```

This package has the next option to be executed

## Andino simulation with Gazebo diff drive plugin


```
ros2 launch andino_gazebo andino_one_robot.launch.py initial_pose_x:=3.0
```

Note that this will execute gazebo , spawn andino and run rviz. Like parameters:

- `use_sim_time:=true` . This parameter indicate to the rviz that it should work with simulation time
- `rviz:=true` . This parameter let to decide if you want to run rviz with this launch. False can be useful if you want to view the rviz in another computer. 
- `worlds:=empty_world.world` . SDF file of the world where andino will run. Note that the world should be available in gazebo paths.

Finally, all the parameters of spawn an andino robot to put the robot in specific position are available.

## Spawn an Andino robot

```
ros2 launch andino_gazebo spawn_robot.launch.py initial_pose_x:=3.0 entity:=andino robot_description_topic:=/andino/robot_description
```
The parameters of this launch let to put the robot in any place of the simulation.

- `use_sim_time`: Use simulation (Gazebo) clock if true. (default: 'true')

- `initial_pose_x`: Initial x pose of andino in the simulation. (default: '0.0')

- `initial_pose_y`: Initial y pose of andino in the simulation. (default: '0.0')

- `initial_pose_z`: Initial z pose of andino in the simulation. (default: '0.0')

- `robot_description_topic`: robot description topic. (default: '/robot_description')

- `initial_pose_yaw`: Initial yaw pose of andino in the simulation. (default: '0.0')

- `use_gazebo_ros_control`: True to use the gazebo_ros_control plugin.  (default: 'false')

- `entity`: Name of the robot. (default: 'andino')

Spawn multiple andino robots has some issues, so no namespace is created.

## ros_control (not tested yet)

It is in the xacro file but has not been tested, it is necessary to made another launch with ros control because I do not have how to pass the parameter to the xacro converter.

```
ros2 launch andino_gazebo andino_with_ros_control.launch.py
```

