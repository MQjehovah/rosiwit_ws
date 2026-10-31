# FAST-LIVO2 (ROS 2 Humble) — 本地引入说明

本目录是 **FAST-LIVO2** 的 ROS 2 移植版源码，作为第三方包引入（vendored），
包名保持上游的 `fast_livo`（`ros2 launch fast_livo ...`）。只做了少量为了能编译/跑通的修补，
见下方“本地改动”。

## 来源

| 项 | 值 |
|---|---|
| 上游仓库 | https://github.com/Rhymer-Lcy/FAST-LIVO2-ROS2-MID360-Fisheye |
| 上游提交 | `1fcd0d05cadaeb25ca59fd87cda95aaaee41e3ea`（2026-07-21，merge upstream main） |
| 上游的上游 | https://github.com/hku-mars/FAST-LIVO2 （HKU MARS，论文 T-RO'24） |
| ROS 2 移植来源 | https://github.com/Robotic-Developer-Road/FAST-LIVO2 （`humble` 分支） |
| 许可证 | **GPL-2.0**（见 `LICENSE`；商用需联系作者另行授权） |

它是目前维护较活跃、且与上游算法同步的 ROS 2 Humble 版本之一（另一支是
`v4rl-ucy/FAST-LIVO2-ROS2`，代码更旧）。

## 本地改动（相对上游 commit）

1. **`CMakeLists.txt` / `package.xml`：移除未使用的 `pcl_ros` 依赖**
   代码里没有任何 `pcl_ros` 的头文件/符号，但上游强制 `find_package(pcl_ros REQUIRED)`，
   在只装了 `pcl_conversions` 的环境会直接配置失败。删除后依赖只剩
   `pcl_conversions` / `cv_bridge` / `image_transport` / `tf2_ros` 等。
2. **`launch/*.launch.py`：新增 `use_sim_time` 参数**（默认 `False`，播包时传 `true`）。
   节点输出（`/aft_mapped_to_init`、`/path`、点云时间戳）用的是 `node->now()`，
   播 bag 时不设 sim time 会导致时间戳/`/path` 异常。上游 README 让传
   `use_sim_time:=True`，但 launch 里其实没有这个参数。
3. **`launch/mapping_avia_marslvig.launch.py`、`launch/mapping_ouster_ntu.launch.py`：
   把相机参数 YAML 也传给 `fast_livo` 节点**。
   上游这两个 launch 只把 `camera_*.yaml` 传给 `parameter_blackboard`，而 `vikit`
   实际是从 `fast_livo` 节点自己的参数里读 `camera.model/camera.fx/...`（见
   `vikit_ros/include/vikit/camera_loader.h`），漏传会抛
   `Camera model not correctly specified`。
4. **`launch/mapping_ouster_ntu.launch.py`：去掉多余的 `parameter_blackboard`
   与 `image_transport republish(compressed->raw)` 节点**。前者已无必要，后者在没装
   `compressed_image_transport` 插件时会在 `use_respawn=True` 下反复崩溃重启
   （NTU 数据集本身是 raw `Image`）。
5. **`config/camera_NTU_VIRAL.yaml`：参数改为 `camera:` 命名空间**
   （`camera.model/fx/fy/...`）。上游此文件还是旧的 `cam_model/cam_fx/...` 写法，
   与本分支 `vikit` 的读取方式不一致，直接用会失败。数值未改。
6. **内置 `vikit`（相机模型库），不再依赖外部 `vikit_common` / `vikit_ros` 包**
   - `include/vikit/*.h` + `src/vikit/*.cpp` 直接放在本包内，`CMakeLists.txt` 编译成静态库 `vikit` 并链入，
     删除 `find_package(vikit_common/vikit_ros)`、`ament_export_dependencies` 里的 vikit 项，
     以及上游写死的 `${CMAKE_SOURCE_DIR}/../../install/vikit_*/lib/*.so` 链接。
   - 来源：`Rhymer-Lcy/rpg_vikit_ros2_fisheye`（commit `0b5548d`，2026-06）。除 `vikit_ros/src/output_helper.cpp`
     （ROS 可视化辅助，本算法未使用）外，`vikit_common` 的头文件/实现全部合入。
   - `#include <vikit/...>` 写法保持不变，源码零改动。
   - 顺带补了两处原先被 vikit 的 ament 导出“顺带”带进来的依赖：
     `tf2_geometry_msgs`（`include/utils/utils.h` 用到，package.xml/CMake 显式声明）；
     `Sophus::Sophus`（ros-humble-sophus 是 header-only INTERFACE target，`Sophus_INCLUDE_DIRS` 为空，
     需要 `link_libraries(Sophus::Sophus)` 才能拿到 include 路径）。
   - **许可证注意**：`vikit_common` 为 **GPLv3**，FAST-LIVO2 为 **GPL-2.0**；合并到同一二进制后对外分发应按
     GPLv3 处理（内部使用不受影响）。
7. **不包含 `Supplementary/`**（上游附带的 19 MB PDF），其余（含 `pics/`、`Log/` 示例轨迹）保留。
8. **新增通用 `sensor_msgs/PointCloud2` 支持（`preprocess.lidar_type: 8` = `GENERIC`）**
   - 按字段名自动识别：`x/y/z`、可选 `intensity`（或 `i`/`reflectivity`）、可选逐点时间
     （`time` / `timestamp` / `t` / `offset_time` / `curvature`，自动判断 s / ns / 绝对时间戳），
     换算成 `curvature`（ms）供去畸变使用；没有时间字段也能跑（不做逐点补偿）。
   - 用原始 `data` + `PointField.offset/datatype` 解析（`memcpy`，兼容未对齐字段），
     不依赖任何特定驱动的点结构体。
   - 新增 `launch/mapping_generic.launch.py` 和 `config/vanjee.yaml` 作为示例。
9. **`livox_ros_driver2` 变成可选依赖（默认关闭）**
   - `-DFASTLIVO_WITH_LIVOX_MSGS=ON` 才编译 `CustomMsg` 分支（Avia / Retail_Street 用）；
     默认 OFF 时**完全不需要**该包，只用通用 PointCloud2 即可。
   - `lidar_type=1`（AVIA）在未编译该选项时会打错误并自动退回 `GENERIC`。
10. **点云地图保存改用 int64 体素索引**
   上游 `savePCD()` 用 `pcl::VoxelGrid`，其内部网格索引是 int32；当体素较小、场景范围大时
   （`(extent/leaf)^3 > 2^31`）PCL 会打印 `Leaf size is too small ... Integer indices would overflow`
   并**静默返回原始点云**（等于没降采样）。已在 `LIVMapper.cpp` 中用 `voxelDownsampleInt64()` 替代：
   21 bit/轴打包 int64 key + 哈希表，按体素做质心/颜色平均；0.05 m 体素在整条街场景下也正常。

除以上外，算法/C++ 源码未改动。

## 依赖

- **`vikit` 已内置**（`include/vikit`、`src/vikit`），编译时作为静态库链入，无需再单独放
  `vikit_common` / `vikit_ros` 包。
- **`livox_ros_driver2`（可选，默认不需要）**
  只有想看 `CustomMsg`（Avia / Retail_Street）时才需要，编译时加
  `-DFASTLIVO_WITH_LIVOX_MSGS=ON`；默认关闭，通用 PointCloud2 不受影响。
  需要时：
  ```bash
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git -b 1.2.6
  # 然后按上游 build.sh humble 编译，或参考其 ROS2 打包方式
  ```
  只是类型依赖，运行时不需要启动 Livox 驱动（本工作区不放传感器驱动）。
- 系统依赖：`libeigen3-dev`、`libpcl-dev`、`libopencv-dev`、`ros-humble-sophus`、
  `ros-humble-cv-bridge`、`ros-humble-image-transport`、`ros-humble-pcl-conversions`、
  `ros-humble-tf2-ros`、`libfmt-dev`

## 构建 / 运行 / 验证

见 `docs/dev/fast-livo2-ros2.md`。
