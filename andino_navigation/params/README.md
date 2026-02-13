# Nav2 Parameters

Two parameter files are provided to support both ROS 2 Humble and Jazzy distributions:

- **`nav2_params_humble.yaml`** — For use with Nav2 on **ROS 2 Humble**.
- **`nav2_params_jazzy.yaml`** — For use with Nav2 on **ROS 2 Jazzy**.

The appropriate file is automatically selected at launch time based on the `$ROS_DISTRO` environment variable. Users can also override this by passing the `params_file` launch argument.

## Why two files?

Nav2 introduced several breaking changes between Humble and Jazzy that make a single parameter file incompatible across both distros. Key differences include:

| Area | Humble | Jazzy |
|---|---|---|
| `bt_navigator` | `plugin_lib_names` list required | `navigators` map + `error_code_names`; BT plugins are built-in |
| `bt_navigator` sub-nodes | Separate `bt_navigator_navigate_*_rclcpp_node` entries | Removed (integrated into `bt_navigator`) |
| `controller_server` | `progress_checker_plugin` (singular string) | `progress_checker_plugins` (list) |
| `behavior_server` plugins | `nav2_behaviors/Spin` (slash separator) | `nav2_behaviors::Spin` (double colon separator) |
| `behavior_server` topics | `costmap_topic`, `footprint_topic` | `local_costmap_topic`, `global_costmap_topic`, `local_footprint_topic`, `global_footprint_topic` |
| `planner_server` plugin | `nav2_navfn_planner/NavfnPlanner` | `nav2_navfn_planner::NavfnPlanner` |
| New nodes in Jazzy | N/A | `collision_monitor` added; `docking_server` and `route_server` exist upstream but are excluded by Andino's custom navigation launch |

Additionally, on Jazzy, Andino uses its own `navigation_launch.py` instead of nav2_bringup's default. This is because Jazzy's upstream `navigation_launch.py` includes `docking_server` (opennav_docking) and `route_server` (nav2_route) in its lifecycle-managed nodes — features Andino doesn't use. Rather than carrying dead configuration for those nodes, we maintain a custom launch file that only starts the nodes Andino needs.

> **Note:** The Andino-specific tuning (velocity limits, costmap sizes, robot radius, etc.) is kept consistent across both files. When modifying robot-specific parameters, make sure to update **both** files.
