# FAST-LIVO2 并入 SLAM 核心 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 FAST-LIVO2 作为 `rosiwit_slam` 核心内的一个 ROS 无关前端（`fast_livo2_frontend`），支持完整 LiDAR-惯性-视觉紧耦合，并保持现有 `fast_lio2` 行为不变。

**Architecture:** 在核心 `src/rosiwit_slam/src/algorithms/fast_livo2/` 内移植 `LIVMapper`/`VIO`/`voxel_map`/`preprocess`/`vikit`，去掉所有 ROS 依赖；核心新增 `ImageFrame` 输入与 `IFrontend::onImage()`；ROS 接口层只负责把 `sensor_msgs` 转成核心类型（含 CustomMsg/PointCloud2 解析）并发布输出。

**Tech Stack:** C++17、PCL、OpenCV、Eigen、Sophus、yaml-cpp、OpenMP；ROS1 Melodic / ROS2 Humble 接口层。

**Spec:** `docs/superpowers/specs/2026-10-31-fast-livo2-core-extraction-design.md`

## Global Constraints

- 核心 `src/rosiwit_slam` 必须保持 **ROS 无关**：不得出现 `rclcpp`/`ros/ros.h`/`sensor_msgs`/`livox_ros_driver2`。
- 核心必须能 standalone 构建：`cd src/rosiwit_slam && cmake -S . -B build && cmake --build build && ctest --test-dir build`。
- C++17；兼容 Ubuntu 18.04（GCC 7 / OpenCV 3.x）与 22.04（GCC 11 / OpenCV 4.x），避免仅 4.x 的 API。
- 不改变 `fast_lio2_frontend` / `point_lio_frontend` 的默认行为与精度。
- 不新增 ROS 依赖；`vikit` 以源码编入核心（GPLv3，对外分发按 GPLv3）。
- 图像输入类型：`cv::Mat`（BGR8 或 MONO8），时间与 IMU/LiDAR 同一时间基准（秒）。
- 点云输入：核心只接受 `LidarFrame`（`curvature` 已是逐点 ms）。
- 所有新增配置默认不改变旧行为；缺省 `frontend` 仍是 `fast_lio2_frontend`。

## Review Focus

1. **选中 `fast_livo2` 后一直没有图像**：核心必须继续输出 LiDAR-惯性位姿，不能卡住或丢帧。
2. **图像时间戳重复/倒序**：前端缓冲区必须保持单调、不崩溃，重复帧直接丢弃。
3. **IMU 断档 / 管线丢帧**：前端不能死锁，`getOdometry` 仍返回上一帧位姿。
4. **MONO8 与 BGR8 图像**：VIO 都要能处理（内部转灰度），不能假设三通道。
5. **点云没有逐点时间（curvature 全 0）或含 NaN**：预处理不能产生无效点/崩溃，LIO 仍可运行。

## 文件结构

```
src/rosiwit_slam/
├── include/slam_core/
│   ├── slam_types.h                 # + ImageFrame / CameraIntrinsics / SlamOutput::rgb_world_cloud
│   ├── i_frontend.h                 # + onImage() / getRGBCloud()（默认空实现）
│   └── i_slam_algorithm.h           # + onImage()（默认空实现）
├── include/algorithms/fast_livo2/
│   ├── fast_livo2_frontend.h        # 对外前端
│   └── fast_livo2_config.h          # 纯配置结构
├── src/slam_core/
│   ├── slam_factory.cpp             # 注册 fast_livo2_frontend
│   └── slam_pipeline.cpp            # onImage 转发 / rgb 输出
├── src/algorithms/fast_livo2/
│   ├── fast_livo2_frontend.cpp
│   ├── parse_config.{h,cpp}
│   ├── preprocess.{h,cpp}
│   ├── imu_processing.{h,cpp}
│   ├── voxel_map.{h,cpp}
│   ├── livo_mapper.{h,cpp}
│   ├── frame.{h,cpp}
│   ├── visual_point.{h,cpp}
│   ├── vio.{h,cpp}
│   └── vikit/ (include/ + src/)
└── test/
    ├── test_fast_livo2_config.cpp
    ├── test_fast_livo2_preprocess.cpp
    ├── test_fast_livo2_camera.cpp
    └── test_fast_livo2_frontend.cpp
```

---

### Task 1: 核心类型与接口扩展

**Files:**
- Modify: `src/rosiwit_slam/include/slam_core/slam_types.h`
- Modify: `src/rosiwit_slam/include/slam_core/i_frontend.h`
- Modify: `src/rosiwit_slam/include/slam_core/i_slam_algorithm.h`
- Test: `src/rosiwit_slam/test/test_fast_livo2_frontend.cpp`（新建，先只测类型/默认实现）

**Interfaces:**
- Produces:
  - `struct ImageFrame { cv::Mat image; double time = -1.0; std::string frame_id; };`
  - `struct CameraIntrinsics { std::string model; int width, height; double scale, fx, fy, cx, cy, d0, d1, d2, d3; };`
  - `SlamOutput::rgb_world_cloud`（`pcl::PointCloud<pcl::PointXYZRGB>::Ptr`，可空）
  - `IFrontend::onImage(const ImageFrame&)`、`IFrontend::getRGBCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&)`
  - `ISlamAlgorithm::onImage(const ImageFrame&)`

- [ ] **Step 1: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include "slam_core/slam_types.h"
#include "slam_core/i_frontend.h"

struct DummyFrontend : rosiwit_slam::IFrontend {
    bool init(const std::string&) override { return true; }
    void onImu(const rosiwit_slam::IMUSample&) override {}
    void onLidar(const rosiwit_slam::LidarFrame&) override {}
    bool getOdometry(rosiwit_slam::PoseStamped&) override { return false; }
    bool getClouds(rosiwit_slam::CloudType::Ptr&, rosiwit_slam::CloudType::Ptr&) override { return false; }
    bool getKeyFrame(rosiwit_slam::KeyFrame&) override { return false; }
    bool getGlobalMap(rosiwit_slam::PointVec&) override { return false; }
};

TEST(FastLivo2Types, Defaults) {
    rosiwit_slam::ImageFrame f;
    EXPECT_LT(f.time, 0.0);
    rosiwit_slam::CameraIntrinsics c;
    EXPECT_EQ(c.model, "Pinhole");
    rosiwit_slam::SlamOutput out;
    EXPECT_EQ(out.rgb_world_cloud, nullptr);
    DummyFrontend fe;
    fe.onImage(f);                       // 默认空实现，不崩溃
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb;
    EXPECT_FALSE(fe.getRGBCloud(rgb));
}
```

- [ ] **Step 2: 运行确认失败**

Run: `cd src/rosiwit_slam && cmake -S . -B build && cmake --build build --target test_fast_livo2_frontend`
Expected: 编译失败（`ImageFrame`/`onImage`/`rgb_world_cloud` 不存在）

- [ ] **Step 3: 实现类型与默认接口**

`cv::Mat` 需要 `#include <opencv2/core.hpp>`；`rgb_world_cloud` 用 `pcl::PointCloud<pcl::PointXYZRGB>::Ptr` 默认 `nullptr`。
`onImage`/`getRGBCloud` 在 `IFrontend` 里给空实现 `{}` / `{ return false; }`；`ISlamAlgorithm::onImage` 同样给空实现。

- [ ] **Step 4: 运行确认通过**

Run: `cmake --build build && ctest --test-dir build -R FastLivo2Types`
Expected: PASS

- [ ] **Step 5: 提交**

```bash
git add src/rosiwit_slam/include/slam_core src/rosiwit_slam/test/test_fast_livo2_frontend.cpp
git commit -m "feat(slam-core): add image input types and frontend hooks for LIVO"
```

---

### Task 2: 前端骨架与工厂注册

**Files:**
- Create: `src/rosiwit_slam/include/algorithms/fast_livo2/fast_livo2_frontend.h`
- Create: `src/rosiwit_slam/src/algorithms/fast_livo2/fast_livo2_frontend.cpp`
- Modify: `src/rosiwit_slam/src/slam_core/slam_factory.cpp`
- Modify: `src/rosiwit_slam/test/test_fast_livo2_frontend.cpp`

**Interfaces:**
- Consumes: Task 1 的 `IFrontend` 新接口。
- Produces: `class FastLivo2Frontend : public IFrontend`；`SlamFactory::createFrontend("fast_livo2_frontend")` 返回非空。
  - 本任务 `init()` 只校验配置文件存在并返回 `true`（真正解析在 Task 3）。

- [ ] **Step 1: 加失败测试**

```cpp
TEST(FastLivo2Factory, CreatesFrontend) {
    auto fe = rosiwit_slam::SlamFactory::createFrontend("fast_livo2_frontend");
    ASSERT_NE(fe, nullptr);
}
```

- [ ] **Step 2: 运行确认失败**（返回 nullptr）

- [ ] **Step 3: 实现骨架 + 注册**

`FastLivo2Frontend` 的 `onImu/onLidar/onImage` 先空实现，`getOdometry/getClouds/getKeyFrame/getGlobalMap` 返回 `false`。
`SlamFactory::createFrontend` 增加分支。

- [ ] **Step 4: 运行确认通过**

Run: `ctest --test-dir build -R FastLivo2Factory`
Expected: PASS；并确认 `ctest` 全量仍通过（`fast_lio2` 相关测试不回归）。

- [ ] **Step 5: 提交**

```bash
git add src/rosiwit_slam/include/algorithms/fast_livo2 src/rosiwit_slam/src/algorithms/fast_livo2 src/rosiwit_slam/src/slam_core/slam_factory.cpp src/rosiwit_slam/test/test_fast_livo2_frontend.cpp
git commit -m "feat(slam-core): register fast_livo2 frontend skeleton"
```

---

### Task 3: 配置结构与解析

**Files:**
- Create: `src/rosiwit_slam/include/algorithms/fast_livo2/fast_livo2_config.h`
- Create: `src/rosiwit_slam/src/algorithms/fast_livo2/parse_config.{h,cpp}`
- Modify: `src/rosiwit_slam/src/algorithms/fast_livo2/fast_livo2_frontend.cpp`
- Test: `src/rosiwit_slam/test/test_fast_livo2_config.cpp`

**Interfaces:**
- Produces:
  - `struct FastLivo2Config { CameraIntrinsics camera; Eigen::Matrix3d lidar_imu_rot; Eigen::Vector3d lidar_imu_trans; Eigen::Matrix3d cam_lidar_rot; Eigen::Vector3d cam_lidar_trans; double lidar_time_offset, img_time_offset, imu_time_offset; bool img_en; int lidar_type, point_filter_num; double blind, filter_size_surf; int scan_line; ... }`
  - `bool parseFastLivo2Config(const std::string& yaml_path, FastLivo2Config& out, std::string& err);`
- 规则：根级 `r_il`/`t_il`（TF 派生）优先于 `mapping.extrin_calib.lidar_imu_*`；缺 `mapping.camera` 时返回 false 并给出错误。

- [ ] **Step 1: 写失败测试**

用临时 yaml（`std::ofstream` 写 `global/modules/frontend`、`mapping.camera.*`、`mapping.extrin_calib.*`、`mapping.fast_livo2.*`），断言：
- `camera.fx` 读到预期值；
- 根级 `r_il/t_il` 覆盖 `mapping.extrin_calib`；
- 缺 `camera` 段时 `parseFastLivo2Config` 返回 false 且 `err` 非空。

- [ ] **Step 2: 运行确认失败**（函数不存在）

- [ ] **Step 3: 实现解析**

使用 `yaml-cpp`（核心已 `find_package(yaml-cpp QUIET)`；若未找到需在 CMake 里改为 REQUIRED 并更新依赖说明）。解析失败一律返回 false。

- [ ] **Step 4: 运行确认通过 + 提交**

```bash
git add src/rosiwit_slam/include/algorithms/fast_livo2/fast_livo2_config.h src/rosiwit_slam/src/algorithms/fast_livo2/parse_config.* src/rosiwit_slam/test/test_fast_livo2_config.cpp
git commit -m "feat(slam-core): parse fast_livo2 config (camera/extrinsics/time offsets)"
```

---

### Task 4: 移植预处理（无 ROS）

**Files:**
- Create: `src/rosiwit_slam/src/algorithms/fast_livo2/preprocess.{h,cpp}`
- Test: `src/rosiwit_slam/test/test_fast_livo2_preprocess.cpp`

**Interfaces:**
- Consumes: `slam_core/slam_types.h`（`CloudType`/`PointType`）。
- Produces: `class FastLivo2Preprocess { void configure(const FastLivo2Config&); void process(const CloudType& in, CloudType& out); }`
- 只处理已带 `curvature`（ms）的 `CloudType`；实现 `blind`、`point_filter_num`、NaN 丢弃；不再包含 CustomMsg/`sensor_msgs`。

- [ ] **Step 1: 写失败测试**

构造 10 个点（含 1 个 NaN、1 个盲区内、`curvature` 0/50/100），配置 `point_filter_num=2, blind=0.5`，断言：
- 输出不含 NaN、不含盲区内点；
- 保留点的 `curvature` 原样；
- 空输入输出为空且不崩。

- [ ] **Step 2: 运行确认失败**（类不存在）

- [ ] **Step 3: 实现**

从独立包 `src/preprocess.cpp` 的 `generic_pcl_handler` 提取核心循环（字段解析不在核心）；`curvature` 直接拷贝，不再做时间单位判断。

- [ ] **Step 4: 运行确认通过 + 提交**

```bash
git add src/rosiwit_slam/src/algorithms/fast_livo2/preprocess.* src/rosiwit_slam/test/test_fast_livo2_preprocess.cpp
git commit -m "feat(slam-core): port fast_livo2 preprocessing (ROS-free)"
```

---

### Task 5: 移植 IMU 预积分 + 体素地图 + LIO 主循环

**Files:**
- Create: `src/rosiwit_slam/src/algorithms/fast_livo2/imu_processing.{h,cpp}`
- Create: `src/rosiwit_slam/src/algorithms/fast_livo2/voxel_map.{h,cpp}`
- Create: `src/rosiwit_slam/src/algorithms/fast_livo2/livo_mapper.{h,cpp}`
- Modify: `src/rosiwit_slam/src/algorithms/fast_livo2/fast_livo2_frontend.cpp`
- Modify: `src/rosiwit_slam/test/test_fast_livo2_frontend.cpp`

**Interfaces:**
- Consumes: Task 3 配置、Task 4 预处理。
- Produces:
  - `class LivoMapper { bool init(const FastLivo2Config&); void addImu(const IMUSample&); bool processLidar(const LidarFrame&); bool getOdometry(PoseStamped&); bool getClouds(...); }`
  - `FastLivo2Frontend` 的 `onImu/onLidar/getOdometry/getClouds` 接通；`img_en=false` 时只跑 LIO。

- [ ] **Step 1: 加失败测试（静态点云 + 静止 IMU）**

喂 30 帧相同点云 + 200Hz 静止 IMU，断言：
- `getOdometry` 返回 true，位姿有限（无 NaN）；
- 位置漂移 < 0.5 m（静止场景）；
- 无图像时不会阻塞（每帧都有返回）。

- [ ] **Step 2: 运行确认失败**（未实现）

- [ ] **Step 3: 移植并接通**

去 ROS 化要点：`rclcpp::Node` 参数 → `FastLivo2Config`；发布器 → 输出缓存；
`RCLCPP_*` → `std::cerr`；`tf`/`Time` → `double` 秒；`image_transport` 全部删除。
`voxel_map` 只保留 `PointType`/`PointToPlane` 相关，无消息类型。

- [ ] **Step 4: 构建 + 测试通过 + 提交**

Run: `cmake --build build && ctest --test-dir build -R FastLivo2`
Expected: PASS

```bash
git add src/rosiwit_slam/src/algorithms/fast_livo2 src/rosiwit_slam/test/test_fast_livo2_frontend.cpp
git commit -m "feat(slam-core): port fast_livo2 LIO backend and frontend wiring"
```

---

### Task 6: 内置 vikit 相机模型

**Files:**
- Create: `src/rosiwit_slam/src/algorithms/fast_livo2/vikit/include/vikit/*.h`
- Create: `src/rosiwit_slam/src/algorithms/fast_livo2/vikit/src/*.cpp`
- Modify: `src/rosiwit_slam/CMakeLists.txt`
- Test: `src/rosiwit_slam/test/test_fast_livo2_camera.cpp`

**Interfaces:**
- Produces: `vk::PinholeCamera` 等（与独立包同名）；`FastLivo2Config.camera` → `vk::AbstractCamera`。

- [ ] **Step 1: 写失败测试**

用 `fx=400,fy=400,cx=320,cy=240` 构造 PinholeCamera，断言 `cam->fx()==400`、
`project(0,0,1)` 的像素接近 `(320,240)`。

- [ ] **Step 2: 运行确认失败**

- [ ] **Step 3: 迁入 vikit 并加入 CMake**

把 `ros2_ws/src/fast_livo2/src/vikit/*.cpp` 与 `include/vikit/*.h` 移到核心；CMake 增加
`fast_livo2` 的 include 路径；避免与系统 vikit 冲突（命名空间 `vk` 不变，目标为内部 static lib）。

- [ ] **Step 4: 构建 + 测试通过 + 提交**

```bash
git add src/rosiwit_slam/src/algorithms/fast_livo2/vikit src/rosiwit_slam/test/test_fast_livo2_camera.cpp src/rosiwit_slam/CMakeLists.txt
git commit -m "feat(slam-core): vendor vikit camera models into fast_livo2"
```

---

### Task 7: 移植 VIO 与图像入口

**Files:**
- Create: `src/rosiwit_slam/src/algorithms/fast_livo2/{frame,visual_point,vio}.{h,cpp}`
- Modify: `src/rosiwit_slam/src/algorithms/fast_livo2/livo_mapper.{h,cpp}`
- Modify: `src/rosiwit_slam/src/algorithms/fast_livo2/fast_livo2_frontend.cpp`
- Test: `src/rosiwit_slam/test/test_fast_livo2_frontend.cpp`（补 MONO8/BGR8 用例）

**Interfaces:**
- Consumes: Task 5 的 `LivoMapper`、Task 6 的相机。
- Produces: `LivoMapper::addImage(const ImageFrame&)`；`sync_packages` 的 `WAIT→LIO→VIO` 状态机。
- 图像统一转灰度（BGR8 → `cv::cvtColor`，MONO8 直接用），再做 VIO。

- [ ] **Step 1: 加失败测试**

- 同一帧 MONO8 与 BGR8 输入，断言处理后都得到非空的视觉特征数（或至少不崩、`VIO` 计数增加）；
- 重复时间戳的图像被丢弃（计数不增加）。

- [ ] **Step 2: 运行确认失败**

- [ ] **Step 3: 移植 VIO + 接通图像**

`img_cbk`/`jpeg_callback` 合并为 `addImage`；不再依赖 `cv_bridge`；保留光流/光度误差逻辑。

- [ ] **Step 4: 构建 + 测试通过 + 提交**

```bash
git add src/rosiwit_slam/src/algorithms/fast_livo2 src/rosiwit_slam/test/test_fast_livo2_frontend.cpp
git commit -m "feat(slam-core): port fast_livo2 VIO and image input"
```

---

### Task 8: 前端语义（关键帧 / 彩色点云 / 线程安全）

**Files:**
- Modify: `src/rosiwit_slam/src/algorithms/fast_livo2/fast_livo2_frontend.{h,cpp}`
- Modify: `src/rosiwit_slam/src/slam_core/slam_pipeline.cpp`（rgb 输出透传）
- Test: `src/rosiwit_slam/test/test_fast_livo2_frontend.cpp`

**Interfaces:**
- Produces:
  - `getKeyFrame` 距离阈值 1.0 m / 角度 0.17 rad（对齐 `FastLio2Frontend`）；
  - `getRGBCloud` 返回当前帧彩色世界系点云；
  - `getOdometry` 在无更新帧返回上一帧位姿（`true`）。

- [ ] **Step 1: 加失败测试**

- 连续两帧位置差 < 1 m：第二帧 `getKeyFrame` 为 false；> 1 m 为 true；
- 无图像时每帧 `getOdometry` 都为 true 且时间戳单调；
- `getRGBCloud` 在 `img_en=false` 时返回 false。

- [ ] **Step 2: 运行确认失败**

- [ ] **Step 3: 实现 + 管线透传**

`SlamPipeline` 在 `SlamOutput` 里填 `rgb_world_cloud`（若前端有）；加锁拷贝，避免与 VIO 线程竞争。

- [ ] **Step 4: 运行确认通过 + 提交**

```bash
git add src/rosiwit_slam/src/algorithms/fast_livo2 src/rosiwit_slam/src/slam_core/slam_pipeline.cpp src/rosiwit_slam/test/test_fast_livo2_frontend.cpp
git commit -m "feat(slam-core): fast_livo2 keyframes, rgb cloud and latest-pose semantics"
```

---

### Task 9: ROS2 接口接入图像

**Files:**
- Modify: `ros2_ws/src/rosiwit_slam_ros2/src/ros_interface/slam_node.cpp`
- Modify: `ros2_ws/src/rosiwit_slam_ros2/include/ros_interface/slam_node.h`
- Modify: `ros2_ws/src/rosiwit_slam_ros2/config/default.yaml`
- Modify: `ros2_ws/src/rosiwit_slam_ros2/CMakeLists.txt`（如新增 cv_bridge 依赖）

**Interfaces:**
- Consumes: Task 1 的 `onImage`、Task 8 的输出。
- Produces: 订阅 `sensor_msgs/Image` 与 `sensor_msgs/CompressedImage`（topic 由 `input.image_topic` 配置）；
  `cv::Mat` 直接注入；`input.disable_prefilter` 为 true 时不调用 `RosUtils::ros2PCL` 的抽稀/裁剪；
  发布 `/world_cloud_rgb`（`rgb_world_cloud` 非空时）。

- [ ] **Step 1: 加测试**：用 `ros2 topic pub` 发一张合成 `Image`，`ros2 topic hz /world_cloud_rgb` 有输出（或单元测试 `slam_node` 的图像转换函数）。

- [ ] **Step 2: 实现订阅与转换**（`Image` 按 encoding 转 BGR8/MONO8；`CompressedImage` 用 `cv::imdecode`）。

- [ ] **Step 3: 构建 `colcon build --packages-select rosiwit_slam_ros2` 通过**。

- [ ] **Step 4: 提交**

```bash
git add ros2_ws/src/rosiwit_slam_ros2
git commit -m "feat(slam-ros2): feed camera images and publish colored cloud for fast_livo2"
```

---

### Task 10: 数据集验证（NTU VIRAL eee_03）

**Files:**
- Create: `scripts/eval_ate.py`（SE3 Umeyama，读 TUM 位姿与 GT）
- Modify: `docs/dev/fast-livo2-ros2.md`（追加核心版验证记录）

**Interfaces:**
- 使用 Task 9 的 ROS2 节点 + `fast_livo2_frontend`，跑 `eee_03_ros2`（1x，181 s）。

- [ ] **Step 1: 记录基线**：先跑独立包（已有 `docs/artifacts/fastlivo2_ntu_eee03/metrics.txt`，ATE 0.257 m）作为对照。

- [ ] **Step 2: 跑核心版**（关闭后端/回环以对齐口径）：
  `global.modules.frontend: fast_livo2_frontend`、`backend` 置空/关闭、`mapping.camera.*` 填 NTU 内参。

- [ ] **Step 3: 导出 `/lio_odom`（或 `odom_pose`）为 TUM，计算 ATE。**

Expected: ATE ≤ 0.30 m；轨迹长度 ~137 m；VIO 持续更新；无 error。

- [ ] **Step 4: 提交结果与文档**

```bash
git add scripts/eval_ate.py docs/dev/fast-livo2-ros2.md docs/artifacts/
git commit -m "test(slam-core): verify fast_livo2 core on NTU VIRAL eee_03"
```

---

### Task 11: fast_lio2 回归

**Files:**
- Create: `scripts/record_odom_baseline.py`（或复用现有录制脚本）
- Modify: `docs/dev/fast-livo2-ros2.md`

- [ ] **Step 1: 改动前基线**：用本地 bag（或 NTU）跑 `fast_lio2_frontend`，保存 `odom_pose` 为 TUM。

- [ ] **Step 2: 改动后重跑**，用 `scripts/eval_ate.py` 对比两条轨迹的最大漂移。

Expected: 最大漂移 < 1 cm，ATE 变化 < 0.01 m。

- [ ] **Step 3: 提交**

```bash
git add scripts/record_odom_baseline.py docs/dev/fast-livo2-ros2.md
git commit -m "test(slam-core): fast_lio2 regression baseline after LIVO integration"
```

---

### Task 12: 文档与收尾

**Files:**
- Modify: `README.md`（`src/rosiwit_slam`）
- Modify: `docs/dev/fast-livo2-ros2.md`
- Modify: `docs/superpowers/specs/2026-10-31-fast-livo2-core-extraction-design.md`（状态改为"已实现"）

- [ ] **Step 1: 更新架构文档**（核心算法列表加 `fast_livo2`，前端选择说明、yaml 示例、GPLv3 提示）。

- [ ] **Step 2: 全量构建 + 测试**

Run:
```bash
cd src/rosiwit_slam && cmake -S . -B build && cmake --build build && ctest --test-dir build
cd ../../ros2_ws && colcon build --packages-select rosiwit_slam_ros2
```
Expected: 全部通过。

- [ ] **Step 3: 提交**

```bash
git add README.md docs/
git commit -m "docs(slam-core): document fast_livo2 frontend"
```

---

## Self-Review

- **Spec coverage**：类型/接口（Task 1）、前端与工厂（Task 2）、配置（Task 3）、预处理（Task 4）、
  LIO（Task 5）、vikit（Task 6）、VIO/图像（Task 7）、语义与输出（Task 8）、ROS2（Task 9）、
  验证（Task 10）、回归（Task 11）、文档（Task 12）。ROS1 图像接入按 spec 非目标，未列任务。
- **Review Focus 测试归属**：1/2/3 → Task 8；4 → Task 7；5 → Task 4。
- **类型一致性**：`ImageFrame`/`CameraIntrinsics`/`FastLivo2Config`/`LivoMapper` 在 Task 1/3/5 定义，
  后续任务只引用。
- **比例**：计划长度与 spec 相当，未展开算法实现代码。
