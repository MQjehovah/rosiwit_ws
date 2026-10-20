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

## 两种 profile（`~input_profile`）

| profile | 行为 |
|---|---|
| `rosiwit`（默认） | 只挂 **新接口**（`RosiwitInterface`）：`lio_odom`/`odom`/`lio_path`/`body_cloud`/`world_cloud`/`cloud_map`/`grid_map`/`map`、TF、4 个服务 |
| `legacy` | 新接口**抑制部分输出**，并在**同一个核心实例**上再挂 **旧 `xz_3d_slam` 接口**（`LegacyXz3dSlamInterface`），逐条复刻旧话题/服务/TF/告警 |

`legacy` 下：`RosiwitInterface` 不发 `odom` 别名、不发任何 TF、不发 `/map`（都归旧接口独占），
其余 `lio_odom`/`lio_path`/`cloud_map`/`grid_map`/`body_cloud`/`world_cloud` 照发（被本仓库
其它模块订阅）。输入话题名由旧配置 `conf/slam_conf.json` + `sensor_config.json` 决定。

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
roslaunch rosiwit_slam_ros1 rosiwit_slam.launch          # 新接口 (profile=rosiwit)
roslaunch rosiwit_slam_ros1 xz_3d_slam.launch model:=XZ-SW50   # 兼容旧系统 (profile=legacy)
```

## 接口速览

**新接口**（`RosiwitInterface`）
- 订阅：`imu`/`lidar`（由 `config_file` 指定）、`/initialpose`、可选 `odom_topic`/`vision.vo_topic`/`localization.absolute.topic`
- 发布：`lio_odom`/`odom`、`lio_path`、`body_cloud`/`world_cloud`/`cloud_map`、`grid_map`/`map`
- TF：`world_frame->body_frame`（动态）、`map->odom`（静态）
- 服务：`save_map`/`load_map`/`save_grid_map`/`set_slam_mode`

**旧接口**（`LegacyXz3dSlamInterface`，仅 `legacy` profile）
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
