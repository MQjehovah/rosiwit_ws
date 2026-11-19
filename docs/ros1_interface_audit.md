# ROS1 接口层审计：与真机 `xzrobot_ros_dependency` 的一致性

对比对象：设备 14004361 上**正在运行的厂商节点 `/xz_3d_slam`** 与我们的
`rosiwit_slam`（源码 `ros1_ws/src/rosiwit_slam_ros1`）。

## 1. 厂商类型使用清单（全部来自 `xzrobot_ros_dependency`，未新建任何消息）

| 类型 | 我们的用途 | 设备上是否存在 |
|---|---|---|
| `SlamInfo` | 发布 `/slam/info`（App 唯一订阅的定位状态） | ✅ |
| `SwitchMap` | 服务 `/switch_map`、`/localization/switch_map` | ✅ |
| `Relocation` | 服务 `/relocation`、`/localization/relocalize`、`/localization/set_initial_pose` | ✅ |
| `SlamModeSwitchService` | 服务 `/slam/mode_switch` | ✅ |
| `StopMapOperation` | 服务 `/stop_map_operation` | ✅ |
| `UpdateMap` | 服务 `update_map_operation` | ✅ |
| `TagDetect` | 服务 `/start_task`、`/finish_task` | ✅ |
| `CommonBoolService` | `/slam/dump_recorder` | ✅ |
| `CommonIntRequestService` | 内部错误码/状态请求 | ✅ |
| `LocationInfo` | 发布 `/location_info`（**我们加的，见 §4**） | ✅（但厂商不发） |

十个类型的**定义与设备上的厂商包逐字节一致**（MD5 相同），没有自建/魔改消息。

## 2. 话题对比

| | 厂商 `/xz_3d_slam` | 我们 | 差异 |
|---|---|---|---|
| 发布 | `/curr_localization_pose` `/curr_scan_point` `/curr_submap_point` `/global_grid_map` `/global_map` `/loop_constraint` `/map` `/mapping_path` `/or_curr_scan_point` `/origin_point` `/points_2d` `/scan_f_map` `/slam/info` `/slam_warning_code` `/tf` | 同左 **+** `/lio_odom` `/world_cloud` `/slam/fusion_diag` `/location_info` | 多 4 个（3 个诊断 + 1 个见 §4） |
| 订阅 | `/initialpose` `/lift/state` `/lift_type` `/odom` `/scan` `/tf` `/tf_static` `/vanjee_lidar_imu_packets` `/vanjee_points722` | `/scan` `/odom` `/lift/state` `/lift_type` + imu/lidar | **缺 `/initialpose`**（见 §3） |

## 3. 必须修：缺 `/initialpose` 订阅

实测：`/initialpose` 的发布者是 **`/xzrobot_app`**，厂商节点订阅它：

```
--- /initialpose ---
Type: geometry_msgs/PoseWithCovarianceStamped
Publishers: /xzrobot_app      Subscribers: /xz_3d_slam
```

即 App 有一条"发初始位姿"的路径走这个话题。我们不订阅 → **App 用这种方式给位姿时会被忽略**。
用标准消息类型，不需要新接口。

## 4. 需要清理的"私加接口"

| 接口 | 厂商有? | 结论 |
|---|---|---|
| `/localization/relocalize` | ❌ | 与 `/relocation` 完全同一处理器 → **多余，删** |
| `/localization/set_initial_pose` | ❌ | 同上 → **删** |
| `/localization/switch_map` | ❌ | 与 `/switch_map` 同一处理器 → **删** |
| `/location_info`（发布） | ❌（厂商不发） | App 只订 `/slam/info`；`/xzrobot_record` 记录用。**非必要，删** |
| `/slam/dump_recorder` | ❌ | 用于离线导出 dump（我们的工作流）→ **保留**，但标注为私有 |
| `/lio_odom` `/world_cloud` `/slam/fusion_diag` | ❌ | 诊断用；应受 `debug_publish`（默认 false）控制，目前**无条件发布** → 改为跟随开关 |

## 5. 包内自建 srv（死代码）

`ros1_ws/src/rosiwit_slam_ros1/srv/`：`SaveMap.srv` `LoadMap.srv` `SaveGridMap.srv` `SetSlamMode.srv`

- `initServer()` 里**没有 advertise** 它们；运行期 `rosservice list` 也没有 `save_map/load_map/...`
- 只有 `add_service_files()` + `generate_messages` 让构建变慢
- → **删掉 srv/ 与对应 CMake/package.xml 条目**（符合"非必要不私加接口"）

## 6. 违反"不要改 ros dependency"的地方

`ros1_ws/src/xzrobot_ros_dependency/` 是厂商包的**工作区副本**（296 KB，入库于 `9de21a4`）：

- 它会在 catkin 构建时**覆盖** `/opt/ros/melodic` 里的厂商包
- 而且我们已经从这份副本里删过 `LocalizationState.msg`
- 设备上本来就装了厂商包，这份副本完全多余且危险（定义漂移 → 话题 MD5 与 App 不匹配）

→ **删除该副本**，`find_package(xzrobot_ros_dependency)` 直接指向 `/opt/ros/melodic`。

### 6.1 但 `LocalizationState` 还有人用（不能直接删了就走）

| 包 | 用法 |
|---|---|
| `rosiwit_slam_ros1` | 曾经发布 `/localization/state`（已移除，App 不用） |
| `rosiwit_navigation_ros1` | **订阅** `/localization/state`，用 `health` 决定是否停车（`node.cpp:5071/5076`） |

也就是说这个自建消息现在寄生在厂商包副本里，供**我们自己的导航**使用。

三种处理方式（推荐 A）：

| 方案 | 做法 | 评价 |
|---|---|---|
| **A（推荐）** | 彻底删 `LocalizationState`；导航改用 `/slam/info` 的 `location_status`（**厂商类型**，语义 0/1/2 与 health 一一对应） | 私有接口归零，最符合"非必要不私加" |
| B | 把 `LocalizationState.msg` 放进 `rosiwit_slam_ros1/msg/`，导航依赖该包 | 私有接口仍在，但"写在包内"合规 |
| C | 放进导航包，slam 依赖导航 | 依赖方向不对，不推荐 |

A 需要改导航的订阅与判停逻辑（`health==LOST` → `location_status==2`），改动小但要编译验证。

## 7. 结论与动作清单

| # | 动作 | 风险 |
|---|---|---|
| 1 | 订阅 `/initialpose`（PoseWithCovarianceStamped）→ 走现有 setInitialPose 流程 | 低（只增不减） |
| 2 | 删 3 个 `/localization/*` 别名服务 | 低（App 不用） |
| 3 | 删 `/location_info` 发布 | 低（App 不用） |
| 4 | `/lio_odom` `/world_cloud` `/slam/fusion_diag` 跟随 `debug_publish` | 低 |
| 5 | 删包内 4 个未使用 srv | 低（无人 advertise） |
| 6 | 删工作区里的 `xzrobot_ros_dependency` 副本 | **中**：导航还依赖它里面的 `LocalizationState`，需先做 §6.1-A |
| 7 | 保留 `/slam/dump_recorder`（离线 dump 工作流） | — |

顺序建议：**1 → 2 → 3 → 4 → 5 → §6.1-A（导航改用 `/slam/info`）→ 6**，每步都编译验证。

## 8. 编译验证（容器服务器 192.168.31.34）

镜像：`xzrobot_ros_compile:1.0.3`（amd64 / Ubuntu 18.04 / ROS **melodic** / PCL1.8 /
OpenCV3.2 / Eigen3.3.4 / 已装 `xzrobot_ros_dependency`）。镜像里**没有** Ceres、Sophus，
且其 `xzrobot_ros_dependency` 是**老版本（无 `SlamInfo.h`）**，所以：

```bash
# 容器内（一次性）
apt-get install -y libceres-dev ros-melodic-tf2-geometry-msgs
cp -r <sophus> /opt/ros/melodic/include/            # ieskf 需要
# 用仓库里的厂商包快照编译（不能用镜像里那份老的）
cd /ws/ros1_ws && catkin_make -j4 -DCATKIN_ENABLE_TESTING=OFF \
   -DCMAKE_BUILD_TYPE=Release -DCMAKE_DISABLE_FIND_PACKAGE_GTSAM=ON
```

**结果（2026-11-20）**：

| 包 | 结果 |
|---|---|
| `rosiwit_slam`（核心） | ✅ `Built target rosiwit_slam` |
| `rosiwit_slam_ros1`（接口层） | ✅ `Built target rosiwit_slam_ros1_node` |
| `rosiwit_navigation_ros1` | ✅ `Built target rosiwit_navigation_ros1_node` |

编译中额外修掉的两个**真问题**（与本轮接口整改无关，但 bionic 上才会暴露）：

| 文件 | 问题 | 修复 |
|---|---|---|
| `include/slam_core/ground_posegraph.h` | `Eigen::SimplicialLDLT` 依赖 `<Eigen/SparseCholesky>`；而 PCL 定义了 `EIGEN_MPL2_ONLY`，该模块直接 `#error` | 改用 `Eigen::ConjugateGradient`（`<Eigen/IterativeLinearSolvers>`，MPL2 可用） |
| `xzrobot_ros_dependency/{CMakeLists,package.xml}`（**我们的快照**） | 厂家原版 `package.xml` 只声明 `actionlib*`，但消息用了 std/geometry/sensor/nav/tf2/visualization；`msg/` 已含 action 展开消息，再 `add_action_files` 会出现同名 target | 补 6 个 `<depend>`；CMakeLists 按原版文件清单生成且**不加** `add_action_files` |

## 9. ARM64 构建 + deb 打包（可装到设备）

**镜像**：`xzrobot_ros_melodic_compile:arm-0.1.3` —— **arm64 / 2.79 GB**，
Ubuntu 18.04.6 + melodic + Ceres 1.13 + PCL 1.8.1 + OpenCV 3.2 + Eigen 3.3.4 + gcc 7.5。
（不含 `xzrobot_ros_dependency` 与 Sophus，需挂载提供。）

x86 宿主机上跑 arm64 容器需要 qemu（宿主机重启后 binfmt 会丢）：

```bash
sudo apt-get install -y qemu-user-static      # 装完自动注册 /proc/sys/fs/binfmt_misc/qemu-aarch64
```

构建 + 出包（实测 **~50 分钟**，-j8 仿真）：

```bash
docker run --rm -v /tmp/arm_build:/ws -w /ws \
  xzrobot_ros_melodic_compile:arm-0.1.3 bash -lc '
    cp -r /ws/third_party/sophus /opt/ros/melodic/include/
    source /opt/ros/melodic/setup.bash
    cd /ws/ros1_ws && catkin_make -j8 -DCATKIN_ENABLE_TESTING=OFF \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_DISABLE_FIND_PACKAGE_GTSAM=ON'
```

> 注意：**不要**用 `catkin_make install` 出包 —— 它会重新 configure/增量编译，而新容器里
> 没有 Sophus 副本，会在 `fast_lio2_frontend.cpp` 处失败。直接从 `devel/` 打包即可。

打包内容（`dpkg-deb --build`）：

```
/opt/ros/melodic/lib/rosiwit_slam_ros1/{rosiwit_slam_ros1_node, bag_stitch_map, map_cleanup, repair_map_grid}
/opt/ros/melodic/lib/{librosiwit_slam.so, librosiwit_common.so, librosiwit_slam_ros1_lib.so}
/opt/ros/melodic/share/rosiwit_slam_ros1/{launch, config, package.xml}
```

**踩点**：第一版只打了 `rosiwit_slam_ros1_lib.so`，设备上 `ldd` 报 `librosiwit_slam.so` /
`librosiwit_common.so` 缺失 —— 核心的两个 .so 必须一起打进包。

**结果**：`rosiwit-slam-ros1_0.1.1_arm64.deb`（1.3 MB，`Architecture: arm64`），
设备 `dpkg -i` 成功、`ldd` **零缺失**；不覆盖厂商 `xz_3d_slam`，`dpkg -r rosiwit-slam-ros1` 可卸载。

<!-- 执行状态见提交信息 -->
