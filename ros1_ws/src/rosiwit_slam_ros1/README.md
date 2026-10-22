# rosiwit_slam_ros1 — rosiwit_slam 的 ROS1 薄接口层

**算法用新的 `rosiwit_slam` 核心，对外兼容旧的 `xz_3d_slam`（`E:\workspace_slam\3d_slam`）。**

本包不含任何 SLAM 算法，只做 ROS1 接口与旧接口形状的粘合；核心 `src/rosiwit_slam`
（纯 C++/PCL/Eigen，无 ROS）通过 CMake `add_subdirectory` **内联构建**、源码共享，
与 ROS2 包 `rosiwit_slam_ros2` 等价。

## 仓库结构

```
src/rosiwit_slam            (算法核心: 无 ROS; 可独立 cmake + ctest)
ros1_ws/src/rosiwit_slam_ros1   (本包: ROS1/catkin 薄接口; 由 ros1_ws/COLCON_IGNORE 标记)
ros2_ws/src/rosiwit_slam_ros2   (ROS2/ament 薄接口)
```

## 单一接口

节点固定使用一个核心实例和一个 ROS1 接口：

- `Ros1SlamInterface`：完整兼容接口（`slam/info`、`/map`、服务、动态 `map->odom` TF 等）
- `RosiwitInterface`：只保留不冲突的新输出（`lio_odom`/`lio_path`/`cloud_map`/`grid_map`/
  `body_cloud`/`world_cloud`）

输入话题由机器人现有 `conf/slam_conf.json` + `sensor_config.json` 决定，避免与底盘驱动/launch
配置脱节。`/odom` 由 `Ros1SlamInterface` 统一订阅并喂给核心，定位模式下的 `imu_ekf` 直接使用。

## 构建本包（ROS1）

本工作区含 `COLCON_IGNORE`，不被 colcon(ROS2) 处理。放进 **ROS1 工作区**：

```bash
cd ~/ros1_ws
ln -s <this_repo>/ros1_ws/src/rosiwit_slam_ros1 src/rosiwit_slam_ros1
ln -s <this_repo>/src/rosiwit_slam            src/rosiwit_slam   # 核心与接口包同级
catkin_make                                   # 需要 xzrobot_ros_dependency 包在环境里
catkin_make run_tests_rosiwit_slam_ros1       # 4 个 gtest
```

独立验证核心（无需 ROS）：

```bash
cd src/rosiwit_slam && cmake -S . -B build && cmake --build build -j && ctest --test-dir build
```

启动：

```bash
roslaunch rosiwit_slam_ros1 rosiwit_slam.launch          # 只启动 SLAM 节点
roslaunch rosiwit_slam_ros1 xz_3d_slam.launch model:=XZ-SW50   # 机器人整机 launch
```

### 载入已有地图（docs/maps/map）

```bash
rosservice call /switch_map "map_name: 'map'
map_dir: '/home/jmq/rosiwit_ws/docs/maps/map/map.yaml'
map_resolution: 0.05
map_origin: [-9.84335, -5.85306, 0.0]
map_negate: false
map_occupied_thresh: 0.65
map_free_thresh: 0.25
initial_pose:
  header: {frame_id: 'map'}
  pose:
    pose:
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
vir_obs: ''"
```

`/switch_map` 会读取 `map_dir` 去掉后缀后的 `map.pgm` + `map.yaml` 发布 `/map`，
并用同目录的 `map.pcd` 载入定位地图（没有 `.xzmap` 时走 PCD fallback），
随后自动切到 localization。请把 `map_dir` 改成你机器上的实际路径。

> 导航端若出现 `STOPPED_ODOM`，或 RViz 的 2D Nav Goal 没反应，先检查
> `rostopic hz /lio_odom` 和 `/slam/info` 的 `location_status`。地图加载后必须有
> 有效定位；可以用 PCD 重新初始化：
>
> ```bash
> rosservice call /load_map "path: '/home/jmq/rosiwit_ws/docs/maps/map/map.pcd'"
> rosservice call /set_slam_mode "mode: 'localization'"
> rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
> "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
> ```
>
> 然后在 RViz 里先用 **2D Pose Estimate**（`/initialpose`），再点 **2D Nav Goal**；
> 确认 Nav Goal 工具的话题是 `/move_base_simple/goal`（`rostopic info` 应能看到
> `/rviz_...` 发布者）。

## 接口速览

**新接口**（`RosiwitInterface`）
- 订阅：`imu`/`lidar`（由 `config_file` 指定）、`/initialpose`、可选 `odom_topic`/`vision.vo_topic`/`localization.absolute.topic`
- 发布：`lio_odom`/`odom`、`lio_path`、`body_cloud`/`world_cloud`/`cloud_map`、`grid_map`/`map`
- TF：`world_frame->body_frame`（动态）、`map->odom`（静态）
- 服务：`save_map`/`load_map`/`save_grid_map`/`set_slam_mode`

**兼容接口**（`Ros1SlamInterface`）
- 14 话题：`global_map`/`global_grid_map`/`/map`/`curr_scan_point`/`scan_f_map`/`curr_submap_point`/
  `or_curr_scan_point`/`curr_localization_pose`/`mapping_path`/`loop_constraint`/`slam/info`/
  `points_2d`/`slam_warning_code`/`origin_point`
- TF：`map->odom`（动态）；8 服务：`/slam/mode_switch`/`/switch_map`/`/relocation`/
  `/stop_map_operation`/`/global_localization`/`/finish_task`/`/start_task`/`update_map_operation`

## 备注
- `origin_point` / `or_curr_scan_point` 以 `frame_id="map"` 发布（照抄旧实现）；点其实是**未配准**的
  输入点，frame_id 仅作兼容标记。
- 点云不做逐点去畸变（curvature 置 0）；如需 deskew，请在驱动层/前端处理。
- 配置为 YAML，键名与 ROS2 `config/default.yaml` 同一核心解析。
