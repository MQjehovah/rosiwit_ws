# rosiwit_exploration_ros2

Frontier-based autonomous exploration for ROS 2 Humble.

The node consumes a live `nav_msgs/OccupancyGrid`, finds reachable frontier
clusters (free cells touching unknown space), and sends navigation goals until no
frontier remains. Mapping itself is still performed by `rosiwit_slam_ros2`.

## Quick start (full simulation)

```bash
source /opt/ros/humble/setup.bash
source ~/rosiwit_ws/ros2_ws/install/setup.bash

ros2 launch rosiwit_exploration_ros2 explore_mapping.launch.py \
  world_name:=$HOME/rosiwit_ws/ros2_ws/install/rosiwit_simulator/share/rosiwit_simulator/world/house.world \
  save_map_on_finish:=true \
  save_map_path:=/tmp/rosiwit_exploration_map
```

This starts Gazebo, `rosiwit_slam_ros2`, `rosiwit_navigation_ros2` and the
frontier explorer. The custom navigation node accepts goals on `/goal_pose`, so
the default `goal_interface:=pose` is used.

## Existing stack only

```bash
ros2 launch rosiwit_exploration_ros2 explore.launch.py \
  use_sim_time:=true goal_interface:=pose
```

## Nav2

When a Nav2 stack is running, use its action server:

```bash
ros2 launch rosiwit_exploration_ros2 explore.launch.py \
  use_sim_time:=true goal_interface:=nav2 goal_frame:=map
```

`auto` uses the `navigate_to_pose` action when its server is ready and falls
back to `/goal_pose` otherwise.

## Interfaces

| Type | Name | Description |
|---|---|---|
| Subscriber | `/map` (`nav_msgs/OccupancyGrid`, transient local) | Live SLAM grid |
| Subscriber | `/odom` (`nav_msgs/Odometry`) | Pose fallback when TF is unavailable |
| Publisher | `/goal_pose` (`geometry_msgs/PoseStamped`) | Goal for `rosiwit_navigation_ros2` |
| Publisher | `frontier_markers` (`visualization_msgs/MarkerArray`) | RViz visualization |
| Service | `~/set_enabled` (`std_srvs/SetBool`) | Start/stop exploring |
| Service | `~/status` (`std_srvs/Trigger`) | Current exploration state |

With `goal_interface:=nav2`, goals are sent to the `navigate_to_pose` action
instead of `/goal_pose`.

## Important parameters

| Parameter | Default | Meaning |
|---|---|---|
| `map_topic` | `/map` | OccupancyGrid source |
| `goal_interface` | `pose` | `pose`, `nav2` or `auto` |
| `goal_frame` | `odom` | Goal frame (`map` for Nav2) |
| `free_thresh` / `occupied_thresh` | `50` / `55` | Occupancy thresholds |
| `min_frontier_cells` | `8` | Reject tiny frontier clusters |
| `goal_clearance` | `0.6` | Free radius required around a goal |
| `min_goal_distance` | `0.8` | Ignore goals too close to the robot |
| `max_goal_distance` | `30.0` | Ignore distant goals |
| `frontier_size_weight` | `0.02` | Prefer larger frontiers |
| `goal_timeout` | `45.0` | Abandon a goal after this many seconds |
| `finish_patience` | `20` | Empty ticks before declaring completion |
| `save_map_on_finish` | `false` | Run `map_saver_cli` at the end |

## Notes

- The SLAM node must publish a periodically updated `/map` in `mapping` mode.
  `rosiwit_slam_ros2` has been updated to regenerate the 2D grid from the latest
  observations and publish it every map timer tick.
- `explore_mapping.launch.py` loads `config/navigation_params.yaml` explicitly.
  It keeps `unknown` cells blocked (so the robot cannot shortcut through
  unmapped walls), uses a finer safety margin, lower speed, and pure pursuit.
- `/map` is published at 5 Hz (`map_publish_period` in `simulator.yaml`).
  `/global_costmap` and `/local_costmap` are republished at 10 Hz so newly
  discovered walls appear promptly in RViz and in the controller.
- `local_costmap_size` (default 10 m) controls the local window; the global
  costmap always covers the full SLAM map.
- Goals at frontiers are only accepted when the goal cell is reachable through
  known free space and has `goal_clearance` free space around it.
- Reached/failed goals are blacklisted for `blacklist_duration` seconds to avoid
  repeatedly targeting the same frontier.
