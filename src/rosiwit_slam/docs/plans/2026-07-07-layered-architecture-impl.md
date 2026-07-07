# rosiwit_slam 三层分层架构重构 — 实现计划

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 将 `rosiwit_slam` 重构为层次分明的三层架构(ROS 接口层 / SLAM 接口层 / 算法层),支持运行时切换 SLAM 算法,本次落地 FastLIO2 一个实现。

**Architecture:** Node 极薄(只转格式+发布);`ISlamAlgorithm` 抽象 + `SlamBase` 同步基类 + `SlamFactory` 运行时工厂构成 SLAM 接口层;`FastLio2Algorithm` 适配器封装现有 `MapBuilder`+`IESKF` 作为算法层。详见 `docs/plans/2026-07-07-layered-architecture-design.md`。

**Tech Stack:** ROS2 Humble (C++17), PCL, Eigen3, Sophus, yaml-cpp, ament_cmake_gtest。所有构建/运行命令在 **WSL Ubuntu-22.04** 的 `~/rosiwit_ws` 下执行。

---

## 环境与约定

- **工作区根**: `~/rosiwit_ws`(WSL),包路径 `~/rosiwit_ws/src/rosiwit_slam`(下文简称 `$PKG`)。
- **构建**: `colcon build --packages-select rosiwit_slam --symlink-install`
- **测试**: `colcon test --packages-select rosiwit_slam --event-handlers console_direct+`
- **运行**: `ros2 launch rosiwit_slam slam.launch.py`
- **路径前缀**: 所有相对路径以 `$PKG/` 为根。
- **每步后必编译**: 本计划保证每个 Task 结束时包可编译;Phase 边界可运行。
- **不要在重构中途混入功能改动**,保持行为等价。
- **遗留代码**:`include/fast_lio2_slam/common/*`、`src/common/*`、`src/data_preprocessor/*`、`src/map_manager/*`、`src/localization/*`、`src/loop_closure/*`、`src/odom_fusion/*`、`test/test_types.cpp` 当前**未被活跃 Node 引用**,Phase E 统一归档到 `legacy/`。

---

## Phase A — 新增 SLAM 接口层(纯新增,不碰活跃代码)

每个 Task 后 `colcon build` 应通过(新文件被 GLOB 自动纳入,但尚未被引用,不影响现有 Node)。

### Task A1: slam_core 通用类型 `slam_types.h`

**Files:**
- Create: `include/slam_core/slam_types.h`

**Step 1: 写头文件**

```cpp
// include/slam_core/slam_types.h
#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <memory>
#include <vector>

namespace rosiwit_slam {

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using PointVec  = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;

struct IMUSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc  = V3D::Zero();
    V3D gyro = V3D::Zero();
    double time = -1.0;
};

struct LidarFrame {
    CloudType::Ptr cloud;
    double start_time = -1.0;
    double end_time   = -1.0;
};

enum class SlamState { INITIALIZING, READY, RUNNING, LOST };

struct PoseStamped {
    double time  = -1.0;
    M3D rot      = M3D::Identity();   // r_wi: world <- imu body
    V3D trans    = V3D::Zero();       // t_wi
    V3D vel      = V3D::Zero();       // world-frame velocity
};

struct SlamOutput {
    SlamState state = SlamState::INITIALIZING;
    PoseStamped pose;
    CloudType::Ptr body_cloud;
    CloudType::Ptr world_cloud;
    bool has_new_pose = false;
};

} // namespace rosiwit_slam
```

**Step 2: 验证编译**
Run: `colcon build --packages-select rosiwit_slam --symlink-install`
Expected: SUCCESS(新头未引用,不影响现有目标)

**Step 3: Commit**
```bash
git add include/slam_core/slam_types.h
git commit -m "refactor(slam): add slam_core common types (IMUSample/LidarFrame/SlamOutput)"
```

---

### Task A2: 抽象接口 `i_slam_algorithm.h`

**Files:**
- Create: `include/slam_core/i_slam_algorithm.h`

**Step 1: 写头文件**

```cpp
// include/slam_core/i_slam_algorithm.h
#pragma once
#include <string>
#include <functional>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

class ISlamAlgorithm {
public:
    virtual ~ISlamAlgorithm() = default;

    virtual bool        init(const std::string& config_path) = 0;
    virtual void        onImu(const IMUSample& s) = 0;
    virtual void        onLidar(const LidarFrame& f) = 0;
    virtual SlamState   state() const = 0;
    virtual std::string name() const = 0;

    using OutputCallback = std::function<void(const SlamOutput&)>;
    virtual void        setOutputCallback(OutputCallback cb) = 0;

    // 可选:周期性提取全局地图点(供 Node 发布 cloud_map)
    virtual bool getGlobalMap(PointVec& out_points) { (void)out_points; return false; }
};

} // namespace rosiwit_slam
```

**Step 2: 验证编译**
Run: `colcon build --packages-select rosiwit_slam`
Expected: SUCCESS

**Step 3: Commit**
```bash
git add include/slam_core/i_slam_algorithm.h
git commit -m "refactor(slam): add ISlamAlgorithm abstract interface"
```

---

### Task A3: SlamFactory 占位 + 单元测试(TDD)

**Files:**
- Create: `include/slam_core/slam_factory.h`
- Create: `src/slam_core/slam_factory.cpp`
- Create: `test/test_slam_factory.cpp`
- Modify: `CMakeLists.txt`(注册新测试)

**Step 1: 写失败测试**

```cpp
// test/test_slam_factory.cpp
#include <gtest/gtest.h>
#include "slam_core/slam_factory.h"

TEST(SlamFactory, CreateUnknownReturnsNull) {
    auto p = rosiwit_slam::SlamFactory::create("nonexistent_algo");
    EXPECT_EQ(p, nullptr);
}

TEST(SlamFactory, ListContainsFastLio2) {
    auto names = rosiwit_slam::SlamFactory::listNames();
    // 占位阶段尚未注册,先断言包含 factory 机制;A3 完成后注册 fast_lio2
    EXPECT_GE(names.size(), 1u);
    bool has_fast_lio2 = false;
    for (auto& n : names) if (n == "fast_lio2") has_fast_lio2 = true;
    EXPECT_TRUE(has_fast_lio2);
}
```

> 说明:`listNames()` 测试在 Task C1 注册 FastLio2Algorithm 后才全绿。本 Task 先让 `CreateUnknownReturnsNull` 通过;`ListContainsFastLio2` 此刻**预期失败**(红灯),作为后续注册的驱动测试。若不想留红灯,可暂时只保留第一个用例,Task C1 再加第二个。

**Step 2: 写头文件**

```cpp
// include/slam_core/slam_factory.h
#pragma once
#include <memory>
#include <string>
#include <vector>
#include "slam_core/i_slam_algorithm.h"

namespace rosiwit_slam {

class SlamFactory {
public:
    static std::unique_ptr<ISlamAlgorithm> create(const std::string& name);
    static std::vector<std::string> listNames();
};

} // namespace rosiwit_slam
```

**Step 3: 写占位实现**

```cpp
// src/slam_core/slam_factory.cpp
#include "slam_core/slam_factory.h"
// Task C1 将在此 #include "algorithms/fast_lio2/fast_lio2_algorithm.h"

namespace rosiwit_slam {

std::unique_ptr<ISlamAlgorithm> SlamFactory::create(const std::string& name) {
    // Task C1: if (name == "fast_lio2") return std::make_unique<FastLio2Algorithm>();
    (void)name;
    return nullptr;
}

std::vector<std::string> SlamFactory::listNames() {
    return { "fast_lio2" };  // 占位声明;create() 在 Task C1 才真正支持
}

} // namespace rosiwit_slam
```

**Step 4: 注册测试到 CMake**
在 `CMakeLists.txt` 的 `BUILD_TESTING` 块内、`test_types` 之后追加:

```cmake
        ament_add_gtest(test_slam_factory test/test_slam_factory.cpp)
        target_include_directories(test_slam_factory PRIVATE
            include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS})
        target_link_libraries(test_slam_factory ${PCL_LIBRARIES} Sophus::Sophus)
        install(TARGETS test_slam_factory DESTINATION lib/${PROJECT_NAME})
```

**Step 5: 运行测试**
Run: `colcon build --packages-select rosiwit_slam && colcon test --packages-select rosiwit_slam`
Expected: `CreateUnknownReturnsNull` PASS;`ListContainsFastLio2` 视 Step 1 取舍 PASS 或 FAIL。

**Step 6: Commit**
```bash
git add include/slam_core/slam_factory.h src/slam_core/slam_factory.cpp test/test_slam_factory.cpp CMakeLists.txt
git commit -m "refactor(slam): add SlamFactory skeleton + unit test"
```

---

### Task A4: SlamBase 同步基类 + 单元测试(TDD)

把现有 `FastLio2Node::syncPackage()`(`src/ros_interface/fast_lio2_node.cpp:139-165`)的逻辑上提到 SLAM 层,改为基于 `IMUSample`。

**Files:**
- Create: `include/slam_core/slam_base.h`
- Create: `src/slam_core/slam_base.cpp`
- Create: `test/test_slam_base_sync.cpp`
- Modify: `CMakeLists.txt`(注册测试)

**Step 1: 写失败测试**

```cpp
// test/test_slam_base_sync.cpp
#include <gtest/gtest.h>
#include "slam_core/slam_base.h"

using namespace rosiwit_slam;

namespace {
// 最小桩:记录被同步的包
class CountingAlgo : public SlamBase {
public:
    int synced_count = 0;
    double last_end_time = -1.0;
protected:
    bool onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) override {
        ++synced_count;
        last_end_time = pkg.cloud_end_time;
        out.has_new_pose = true;
        return true;
    }
};
}

TEST(SlamBaseSync, NoDataNoSync) {
    CountingAlgo a;
    EXPECT_EQ(a.synced_count, 0);
    EXPECT_FALSE(a.tryPopAndProcess());   // 无数据
}

TEST(SlamBaseSync, SyncsWhenImuCoversLidarSpan) {
    CountingAlgo a;
    // lidar 帧 [10.0, 10.1]
    LidarFrame f; f.start_time = 10.0; f.end_time = 10.1;
    f.cloud.reset(new CloudType);
    f.cloud->push_back(PointType{});
    a.onLidar(f);
    EXPECT_EQ(a.synced_count, 0);         // 还没有覆盖到 end_time 的 IMU

    // IMU 序列覆盖 [9.9 .. 10.1]
    for (int i = 0; i <= 5; ++i) {
        IMUSample s; s.time = 9.9 + i * 0.04; a.onImu(s);
    }
    EXPECT_GE(a.synced_count, 1);
    EXPECT_NEAR(a.last_end_time, 10.1, 1e-9);
}

TEST(SlamBaseSync, ForwardsOutputViaCallback) {
    CountingAlgo a;
    SlamOutput captured;
    bool got = false;
    a.setOutputCallback([&](const SlamOutput& o){ captured = o; got = true; });

    LidarFrame f; f.start_time = 1.0; f.end_time = 1.05;
    f.cloud.reset(new CloudType); f.cloud->push_back(PointType{});
    a.onLidar(f);
    for (int i = 0; i < 4; ++i) { IMUSample s; s.time = 1.0 + i*0.02; a.onImu(s); }
    EXPECT_TRUE(got);
    EXPECT_TRUE(captured.has_new_pose);
}
```

**Step 2: 写头文件**

```cpp
// include/slam_core/slam_base.h
#pragma once
#include <deque>
#include <mutex>
#include <functional>
#include "slam_core/i_slam_algorithm.h"

namespace rosiwit_slam {

// SLAM 层的同步包:基于 IMUSample(算法无关)
struct SyncPackage {
    std::vector<IMUSample> imus;
    LidarFrame frame;                 // cloud + start/end_time
};

class SlamBase : public ISlamAlgorithm {
public:
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;   // 内部尝试同步并触发回调
    void setOutputCallback(OutputCallback cb) override { m_cb = std::move(cb); }
    SlamState state() const override { return m_state; }

    // 测试/轮询入口:尝试同步一帧,成功则调用 onSyncedPackage 并 emitOutput
    bool tryPopAndProcess();

protected:
    // 子类实现:处理一帧已同步数据,产出输出。返回 false 表示尚未产出
    virtual bool onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) = 0;
    void emitOutput(const SlamOutput& out) { if (m_cb) m_cb(out); }
    void setState(SlamState s) { m_state = s; }

private:
    bool popSyncedPackage(SyncPackage& out);

    std::deque<IMUSample>  m_imu_buf;
    std::deque<LidarFrame> m_lidar_buf;
    mutable std::mutex     m_buf_mutex;
    bool   m_lidar_pushed = false;
    SyncPackage m_pending;
    OutputCallback m_cb;
    SlamState m_state = SlamState::INITIALIZING;
};

} // namespace rosiwit_slam
```

**Step 3: 写实现(移植 fast_lio2_node.cpp:139-165 的同步逻辑)**

```cpp
// src/slam_core/slam_base.cpp
#include "slam_core/slam_base.h"
#include <algorithm>

namespace rosiwit_slam {

void SlamBase::onImu(const IMUSample& s) {
    std::lock_guard<std::mutex> lock(m_buf_mutex);
    if (s.time >= 0 && !m_imu_buf.empty() && s.time < m_imu_buf.back().time) {
        m_imu_buf.clear();   // 乱序,重置
    }
    m_imu_buf.push_back(s);
    // 入队后尝试同步(onLidar 的逻辑也调,保证 IMU-only 也能驱动)
    tryPopAndProcess();
}

void SlamBase::onLidar(const LidarFrame& f) {
    std::lock_guard<std::mutex> lock(m_buf_mutex);
    if (!m_lidar_buf.empty() && f.start_time < m_lidar_buf.back().start_time) {
        m_lidar_buf.clear();
    }
    m_lidar_buf.push_back(f);
    tryPopAndProcess();   // 注:lock 已持有,见下方说明
}

bool SlamBase::popSyncedPackage(SyncPackage& out) {
    // 调用方持锁
    if (m_imu_buf.empty() || m_lidar_buf.empty()) return false;
    if (!m_lidar_pushed) {
        m_pending.frame = m_lidar_buf.front();
        // 按点时间排序(沿用现状对 curvature 的排序语义由上层 Node 完成)
        m_lidar_pushed = true;
    }
    if (m_imu_buf.back().time < m_pending.frame.end_time) return false;

    m_pending.imus.clear();
    while (!m_imu_buf.empty() && m_imu_buf.front().time < m_pending.frame.end_time) {
        m_pending.imus.push_back(m_imu_buf.front());
        m_imu_buf.pop_front();
    }
    out = m_pending;
    m_lidar_buf.pop_front();
    m_lidar_pushed = false;
    return true;
}

bool SlamBase::tryPopAndProcess() {
    SyncPackage pkg;
    {
        std::lock_guard<std::mutex> lock(m_buf_mutex);
        if (!popSyncedPackage(pkg)) return false;
    }
    SlamOutput out;
    if (onSyncedPackage(pkg, out)) emitOutput(out);
    return true;
}

} // namespace rosiwit_slam
```

> ⚠️ **死锁注意**:`onLidar()` 持锁后调用 `tryPopAndProcess()`,后者再次 `lock_guard` 同一 `m_buf_mutex` → 死锁。**两个修法二选一**(实现时只选其一):
> (a) 把 `m_buf_mutex` 改为 `std::recursive_mutex`;
> (b) `onImu/onLidar` 不直接调 `tryPopAndProcess()`,由 Node 的 timer 周期调用 `tryPopAndProcess()`(轮询模式,更简单且无线程重入风险)。
> **推荐 (b)**:`onImu/onLidar` 只入缓冲,`tryPopAndProcess()` 公开供 Node timer 调用。Step 3 实现里删除 `onImu/onLidar` 中的 `tryPopAndProcess()` 调用即可。测试用例直接调 `a.tryPopAndProcess()`。

**Step 4: 注册测试到 CMake**(同 A3 Step 4 模式)

```cmake
        ament_add_gtest(test_slam_base_sync test/test_slam_base_sync.cpp)
        target_include_directories(test_slam_base_sync PRIVATE
            include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS})
        target_link_libraries(test_slam_base_sync ${PCL_LIBRARIES} Sophus::Sophus)
        install(TARGETS test_slam_base_sync DESTINATION lib/${PROJECT_NAME})
```

**Step 5: 运行测试**
Run: `colcon build --packages-select rosiwit_slam && colcon test --packages-select rosiwit_slam`
Expected: 3 个用例全 PASS(采用修法 b,测试里手动调 `tryPopAndProcess()`;上面测试代码已在 `NoDataNoSync` 显式调用)

**Step 6: Commit**
```bash
git add include/slam_core/slam_base.h src/slam_core/slam_base.cpp test/test_slam_base_sync.cpp CMakeLists.txt
git commit -m "refactor(slam): add SlamBase sync base class + unit tests"
```

---

## Phase B — 平移 FastLIO2 算法核心

### Task B1: 搬移 map_builder 到 algorithms/fast_lio2/

`map_builder` 核心自包含(只内部互引用),可整体平移。

**Files:**
- Move: `src/map_builder/*` → `src/algorithms/fast_lio2/map_builder/*`(8 对 .h/.cpp:`ieskf`, `imu_processor`, `lidar_processor`, `ikd_Tree`, `commons`, `utils`, `map_builder`)
- Modify: `CMakeLists.txt`(include 路径)
- Modify: `include/fast_lio2_slam/ros_interface/fast_lio2_node.h`(临时 include 调整,Task D 会删掉它)

**Step 1: 移动文件(WSL bash)**
```bash
cd ~/rosiwit_ws/src/rosiwit_slam
mkdir -p src/algorithms/fast_lio2
git mv src/map_builder src/algorithms/fast_lio2/map_builder
```

**Step 2: 更新 CMake include_directories**
`CMakeLists.txt:98` 把 `src/map_builder` 改为 `src/algorithms/fast_lio2/map_builder`:
```cmake
include_directories(
    include
    src
    src/algorithms/fast_lio2/map_builder
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
)
```

**Step 3: 临时修复 fast_lio2_node.h 的 include**
`include/fast_lio2_slam/ros_interface/fast_lio2_node.h:17-19` 当前:
```cpp
#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"
```
`map_builder/commons.h` 现在在新路径,但 CMake 仍把 `src/algorithms/fast_lio2/map_builder` 加进了 include 目录,所以 `"commons.h"`/`"map_builder.h"`/`"utils.h"` 仍可解析(因为这些目录在 include path 里)。**验证**:不改 .h,直接编译;若失败,把 include 改为带目录前缀。这一步的 Node 是临时的,Task D 会替换。

**Step 4: 验证编译**
Run: `colcon build --packages-select rosiwit_slam`
Expected: SUCCESS(FastLIO2 行为不变)
If FAIL: 多半是 include 解析,补 `src/algorithms/fast_lio2` 到 include_directories 或修 .h 前缀。

**Step 5: Commit**
```bash
git add -A
git commit -m "refactor(fast_lio2): relocate map_builder core to algorithms/fast_lio2/"
```

---

## Phase C — FastLio2Algorithm 适配器

### Task C1: FastLio2Algorithm 实现 ISlamAlgorithm

把现有 `FastLio2Node` 的 `loadParameters`(yaml 读取)、`syncPackage` 已由 `SlamBase` 接管、`process` 调用、发布数据组装(从 IESKF/LidarProcessor 读状态→SlamOutput)整合进适配器。

**Files:**
- Create: `include/algorithms/fast_lio2/fast_lio2_algorithm.h`
- Create: `src/algorithms/fast_lio2/fast_lio2_algorithm.cpp`
- Modify: `src/slam_core/slam_factory.cpp`(注册 fast_lio2)

**Step 1: 写头文件**

```cpp
// include/algorithms/fast_lio2/fast_lio2_algorithm.h
#pragma once
#include <memory>
#include "slam_core/slam_base.h"
#include "commons.h"          // FastLIO2 专属(Config/IMUData)
#include "map_builder.h"      // MapBuilder
#include "ieskf.h"            // IESKF

namespace rosiwit_slam {

class FastLio2Algorithm : public SlamBase {
public:
    bool init(const std::string& config_path) override;
    std::string name() const override { return "fast_lio2"; }
    bool getGlobalMap(PointVec& out_points) override;

protected:
    bool onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) override;

private:
    std::shared_ptr<IESKF>      m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    Config                      m_config;          // FastLIO2 专属配置
    bool m_inited = false;
};

} // namespace rosiwit_slam
```

**Step 2: 写实现**

```cpp
// src/algorithms/fast_lio2/fast_lio2_algorithm.cpp
#include "algorithms/fast_lio2/fast_lio2_algorithm.h"
#include "lidar_processor.h"
#include <yaml-cpp/yaml.h>

namespace rosiwit_slam {

bool FastLio2Algorithm::init(const std::string& config_path) {
    YAML::Node config = YAML::LoadFile(config_path);
    if (!config) return false;

    m_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
    m_config.lidar_min_range  = config["lidar_min_range"].as<double>();
    m_config.lidar_max_range  = config["lidar_max_range"].as<double>();
    m_config.scan_resolution  = config["scan_resolution"].as<double>();
    m_config.map_resolution   = config["map_resolution"].as<double>();
    m_config.cube_len         = config["cube_len"].as<double>();
    m_config.det_range        = config["det_range"].as<double>();
    m_config.move_thresh      = config["move_thresh"].as<double>();
    m_config.na  = config["na"].as<double>();
    m_config.ng  = config["ng"].as<double>();
    m_config.nba = config["nba"].as<double>();
    m_config.nbg = config["nbg"].as<double>();
    m_config.imu_init_num    = config["imu_init_num"].as<int>();
    m_config.near_search_num = config["near_search_num"].as<int>();
    m_config.ieskf_max_iter  = config["ieskf_max_iter"].as<int>();
    m_config.gravity_align   = config["gravity_align"].as<bool>();
    m_config.esti_il         = config["esti_il"].as<bool>();
    auto t_il = config["t_il"].as<std::vector<double>>();
    auto r_il = config["r_il"].as<std::vector<double>>();
    m_config.t_il << t_il[0], t_il[1], t_il[2];
    m_config.r_il << r_il[0], r_il[1], r_il[2], r_il[3], r_il[4], r_il[5], r_il[6], r_il[7], r_il[8];
    m_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

    m_kf = std::make_shared<IESKF>();
    m_kf->setMaxIter(static_cast<size_t>(m_config.ieskf_max_iter));
    m_builder = std::make_shared<MapBuilder>(m_config, m_kf);
    m_inited = true;
    return true;
}

bool FastLio2Algorithm::onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) {
    if (!m_inited) return false;

    // IMUSample(SLAM 层) → IMUData(FastLIO2 内部)
    ::Vec<IMUData> fast_imus;
    fast_imus.reserve(pkg.imus.size());
    for (auto& s : pkg.imus) {
        double t = s.time;
        fast_imus.emplace_back(s.acc, s.gyro, t);
    }
    ::SyncPackage fast_pkg;
    fast_pkg.cloud = pkg.frame.cloud;
    fast_pkg.cloud_start_time = pkg.frame.start_time;
    fast_pkg.cloud_end_time   = pkg.frame.end_time;
    fast_pkg.imus = std::move(fast_imus);

    m_builder->process(fast_pkg);
    if (m_builder->status() != BuilderStatus::MAPPING) return false;

    setState(SlamState::RUNNING);
    out.state = SlamState::RUNNING;
    out.pose.time   = pkg.frame.end_time;
    out.pose.rot    = m_kf->x().r_wi;
    out.pose.trans  = m_kf->x().t_wi;
    out.pose.vel    = m_kf->x().v;
    auto lp = m_builder->lidar_processor();
    out.body_cloud  = LidarProcessor::transformCloud(pkg.frame.cloud, m_kf->x().r_il, m_kf->x().t_il);
    out.world_cloud = LidarProcessor::transformCloud(pkg.frame.cloud, lp->r_wl(), lp->t_wl());
    out.has_new_pose = true;
    return true;
}

bool FastLio2Algorithm::getGlobalMap(PointVec& out_points) {
    if (!m_builder || m_builder->status() != BuilderStatus::MAPPING) return false;
    m_builder->lidar_processor()->collectGlobalMap(out_points);
    return !out_points.empty();
}

} // namespace rosiwit_slam
```

> **命名冲突提示**:`rosiwit_slam::SyncPackage`(slam_types 层)与全局 `::SyncPackage`(commons.h)同名。实现里用 `::SyncPackage` 显式指代全局版,`rosiwit_slam::SyncPackage` 指 SLAM 层版。`Vec`/`IMUData` 同理用 `::Vec`/`::IMUData`。若嫌脆弱,可在 commons.h 类型外包 `namespace fast_lio2 {}`,但属可选优化,本计划保持最小改动。

**Step 3: 在工厂注册**
`src/slam_core/slam_factory.cpp`:
```cpp
#include "slam_core/slam_factory.h"
#include "algorithms/fast_lio2/fast_lio2_algorithm.h"

namespace rosiwit_slam {
std::unique_ptr<ISlamAlgorithm> SlamFactory::create(const std::string& name) {
    if (name == "fast_lio2") return std::make_unique<FastLio2Algorithm>();
    return nullptr;
}
std::vector<std::string> SlamFactory::listNames() { return { "fast_lio2" }; }
}
```

**Step 4: 验证编译 + 测试**
Run: `colcon build --packages-select rosiwit_slam && colcon test --packages-select rosiwit_slam`
Expected: SUCCESS;`test_slam_factory` 的 `ListContainsFastLio2` 现在全绿(若 A3 保留该用例)。

**Step 5: Commit**
```bash
git add -A
git commit -m "feat(fast_lio2): add FastLio2Algorithm adapter implementing ISlamAlgorithm"
```

---

## Phase D — SlamNode 极薄 ROS 层 + 切换 main

### Task D1: 写 SlamNode

**Files:**
- Create: `include/ros_interface/slam_node.h`
- Create: `src/ros_interface/slam_node.cpp`

**Step 1: 写头文件**

```cpp
// include/ros_interface/slam_node.h
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <mutex>
#include "slam_core/i_slam_algorithm.h"
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

class SlamNode : public rclcpp::Node {
public:
    explicit SlamNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SlamNode() override = default;

private:
    struct NodeConfig {
        std::string imu_topic   = "/imu";
        std::string lidar_topic = "/velodyne_points";
        std::string body_frame  = "base_link";
        std::string world_frame = "odom";
        int    lidar_filter_num = 3;
        double lidar_min_range  = 0.5;
        double lidar_max_range  = 100.0;
        bool   print_time_cost  = false;
    };

    void loadParameters();
    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg);
    void lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void timerCB();          // 调 algo->tryPopAndProcess + 发布最新输出
    void mapTimerCB();
    void onOutput(const SlamOutput& out);
    void publish(const SlamOutput& out);

    // 工具(沿用 Utils,Task B1 后路径在 algorithms/fast_lio2/map_builder/utils.h)
    builtin_interfaces::msg::Time toRosTime(double sec);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          m_imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  m_lidar_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub, m_world_cloud_pub, m_global_map_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr     m_path_pub;
    rclcpp::TimerBase::SharedPtr m_timer, m_map_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf;

    std::unique_ptr<ISlamAlgorithm> m_algo;
    NodeConfig m_cfg;
    std::mutex m_out_mutex;
    SlamOutput m_latest;
    bool m_have_output = false;
    nav_msgs::msg::Path m_path;
};

} // namespace rosiwit_slam
```

**Step 2: 写实现**

```cpp
// src/ros_interface/slam_node.cpp
#include "ros_interface/slam_node.h"
#include "slam_core/slam_factory.h"
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include "utils.h"   // Utils::ros2PCL / getSec / getTime

using namespace std::chrono_literals;

namespace rosiwit_slam {

SlamNode::SlamNode(const rclcpp::NodeOptions& options) : rclcpp::Node("rosiwit_slam", options) {
    loadParameters();

    std::string algo_name = "fast_lio2";
    this->declare_parameter<std::string>("slam_algorithm", algo_name);
    this->get_parameter("slam_algorithm", algo_name);

    std::string config_path;
    this->declare_parameter<std::string>("config_file", "");
    this->declare_parameter<std::string>("config_path", "");
    this->get_parameter("config_file", config_path);
    if (config_path.empty()) this->get_parameter("config_path", config_path);

    m_algo = SlamFactory::create(algo_name);
    if (!m_algo) {
        RCLCPP_FATAL(this->get_logger(), "Unknown SLAM algorithm: %s", algo_name.c_str());
        throw std::runtime_error("unknown slam algorithm");
    }
    if (!m_algo->init(config_path)) {
        RCLCPP_FATAL(this->get_logger(), "Algorithm init failed for config: %s", config_path.c_str());
        throw std::runtime_error("slam init failed");
    }
    m_algo->setOutputCallback([this](const SlamOutput& o){ onOutput(o); });

    m_imu_sub  = create_subscription<sensor_msgs::msg::Imu>(m_cfg.imu_topic, 10,
                  std::bind(&SlamNode::imuCB, this, std::placeholders::_1));
    m_lidar_sub= create_subscription<sensor_msgs::msg::PointCloud2>(m_cfg.lidar_topic, 10,
                  std::bind(&SlamNode::lidarCB, this, std::placeholders::_1));
    m_body_cloud_pub  = create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
    m_world_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
    m_odom_pub        = create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
    m_path_pub        = create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
    m_global_map_pub  = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_map", 10);
    m_tf = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    m_timer     = create_wall_timer(20ms, std::bind(&SlamNode::timerCB, this));
    m_map_timer = create_wall_timer(2s,   std::bind(&SlamNode::mapTimerCB, this));
    m_path.header.frame_id = m_cfg.world_frame;
    RCLCPP_INFO(get_logger(), "SlamNode ready: algo=%s imu=%s lidar=%s",
                algo_name.c_str(), m_cfg.imu_topic.c_str(), m_cfg.lidar_topic.c_str());
}

void SlamNode::loadParameters() {
    // frame/topic 既可来自参数(launch 覆盖),也可来自 yaml
    this->declare_parameter<std::string>("imu_topic",   m_cfg.imu_topic);
    this->declare_parameter<std::string>("lidar_topic", m_cfg.lidar_topic);
    this->declare_parameter<std::string>("body_frame",  m_cfg.body_frame);
    this->declare_parameter<std::string>("world_frame", m_cfg.world_frame);
    this->get_parameter("imu_topic",   m_cfg.imu_topic);
    this->get_parameter("lidar_topic", m_cfg.lidar_topic);
    this->get_parameter("body_frame",  m_cfg.body_frame);
    this->get_parameter("world_frame", m_cfg.world_frame);
}

void SlamNode::imuCB(const sensor_msgs::msg::Imu::SharedPtr msg) {
    IMUSample s;
    s.acc  = V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    s.gyro = V3D(msg->angular_velocity.x,    msg->angular_velocity.y,    msg->angular_velocity.z);
    s.time = Utils::getSec(msg->header);
    m_algo->onImu(s);
}

void SlamNode::lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    LidarFrame f;
    f.cloud = Utils::ros2PCL(msg, m_cfg.lidar_filter_num, m_cfg.lidar_min_range, m_cfg.lidar_max_range);
    std::sort(f.cloud->points.begin(), f.cloud->points.end(),
              [](const PointType& a, const PointType& b){ return a.curvature < b.curvature; });
    f.start_time = Utils::getSec(msg->header);
    f.end_time   = f.start_time + (f.cloud->points.empty() ? 0.0 : f.cloud->points.back().curvature / 1000.0);
    m_algo->onLidar(f);
}

void SlamNode::onOutput(const SlamOutput& out) {
    std::lock_guard<std::mutex> lock(m_out_mutex);
    m_latest = out;
    m_have_output = true;
}

void SlamNode::timerCB() {
    // 轮询触发同步处理(修法 b)
    auto* base = dynamic_cast<SlamBase*>(m_algo.get());
    if (base) base->tryPopAndProcess();

    SlamOutput out; bool have;
    { std::lock_guard<std::mutex> lock(m_out_mutex); have = m_have_output; out = m_latest; }
    if (have && out.has_new_pose) publish(out);
}

void SlamNode::publish(const SlamOutput& out) {
    auto stamp = toRosTime(out.pose.time);
    Eigen::Quaterniond q(out.pose.rot);
    // odom
    if (m_odom_pub->get_subscription_count() > 0) {
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = m_cfg.world_frame; odom.child_frame_id = m_cfg.body_frame;
        odom.header.stamp = stamp;
        odom.pose.pose.position.x = out.pose.trans.x();
        odom.pose.pose.position.y = out.pose.trans.y();
        odom.pose.pose.position.z = out.pose.trans.z();
        odom.pose.pose.orientation.x = q.x(); odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z(); odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = out.pose.vel.x();
        odom.twist.twist.linear.y = out.pose.vel.y();
        odom.twist.twist.linear.z = out.pose.vel.z();
        m_odom_pub->publish(odom);
    }
    // tf
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = m_cfg.world_frame; tf.child_frame_id = m_cfg.body_frame;
        tf.header.stamp = stamp;
        tf.transform.translation.x = out.pose.trans.x();
        tf.transform.translation.y = out.pose.trans.y();
        tf.transform.translation.z = out.pose.trans.z();
        tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
        m_tf->sendTransform(tf);
    }
    auto pub_cloud = [&](const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& p,
                         const CloudType::Ptr& c, const std::string& fid){
        if (!p || p->get_subscription_count() <= 0 || !c) return;
        sensor_msgs::msg::PointCloud2 m; pcl::toROSMsg(*c, m);
        m.header.frame_id = fid; m.header.stamp = stamp; p->publish(m);
    };
    pub_cloud(m_body_cloud_pub,  out.body_cloud,  m_cfg.body_frame);
    pub_cloud(m_world_cloud_pub, out.world_cloud, m_cfg.world_frame);
    // path
    if (m_path_pub->get_subscription_count() > 0) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = m_cfg.world_frame; ps.header.stamp = stamp;
        ps.pose.position.x = out.pose.trans.x(); ps.pose.position.y = out.pose.trans.y();
        ps.pose.position.z = out.pose.trans.z();
        ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
        m_path.poses.push_back(ps);
        m_path_pub->publish(m_path);
    }
}

void SlamNode::mapTimerCB() {
    if (m_global_map_pub->get_subscription_count() <= 0) return;
    PointVec pts;
    if (!m_algo->getGlobalMap(pts) || pts.empty()) return;
    CloudType cloud; for (auto& p : pts) cloud.push_back(p);
    cloud.width = cloud.size(); cloud.height = 1;
    sensor_msgs::msg::PointCloud2 m; pcl::toROSMsg(cloud, m);
    m.header.frame_id = m_cfg.world_frame; m.header.stamp = now();
    m_global_map_pub->publish(m);
}

builtin_interfaces::msg::Time SlamNode::toRosTime(double sec) { return Utils::getTime(sec); }

} // namespace rosiwit_slam
```

**Step 3: 验证编译**
Run: `colcon build --packages-select rosiwit_slam`
Expected: SUCCESS(此时 main.cpp 还指向旧 FastLio2Node,SlamNode 尚未被引用,但应编译通过)

**Step 4: Commit**
```bash
git add -A
git commit -m "feat(ros): add thin SlamNode (format conversion + publish only)"
```

---

### Task D2: 切换 main.cpp + 删除旧 Node

**Files:**
- Modify: `src/main.cpp`
- Delete: `src/ros_interface/fast_lio2_node.cpp`
- Delete: `include/fast_lio2_slam/ros_interface/fast_lio2_node.h`

**Step 1: 重写 main.cpp**

```cpp
// src/main.cpp
#include <rclcpp/rclcpp.hpp>
#include "ros_interface/slam_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(std::make_shared<rosiwit_slam::SlamNode>());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
```

**Step 2: 删除旧 Node**
```bash
git rm src/ros_interface/fast_lio2_node.cpp
git rm include/fast_lio2_slam/ros_interface/fast_lio2_node.h
```

**Step 3: 验证编译 + 运行**
Run: `colcon build --packages-select rosiwit_slam`
Expected: SUCCESS
Run: `ros2 launch rosiwit_slam fast_lio2.launch.py`(launch 尚未改名,下一步处理)
Expected: 节点启动,日志 `SlamNode ready: algo=fast_lio2 ...`,话题 `lio_odom`/`lio_path`/`body_cloud`/`world_cloud`/`cloud_map` 正常发布(若手头无数据,至少确认无崩溃、TF 广播)。

**Step 4: Commit**
```bash
git add -A
git commit -m "refactor(ros): switch main to SlamNode, remove legacy FastLio2Node"
```

---

## Phase E — 收尾(归档、命名、配置)

### Task E1: 归档遗留代码到 legacy/

**Files:**
- Move 到 `legacy/`: `include/fast_lio2_slam/common/*`、`src/common/*`、`src/data_preprocessor/*`、`src/map_manager/*`、`src/localization/*`、`src/loop_closure/*`、`src/odom_fusion/*`、对应 `include/fast_lio2_slam/{data_preprocessor,map_manager,localization,loop_closure,odom_fusion}/*`
- Delete/Move: `test/test_types.cpp`(测遗留 types.h,随遗留归档)

**Step 1: 移动目录**(WSL bash,`legacy/` 在 `src/` 外,GLOB 不扫)
```bash
cd ~/rosiwit_ws/src/rosiwit_slam
mkdir -p legacy/include legacy/src legacy/test
git mv include/fast_lio2_slam/common      legacy/include/fast_lio2_slam_common
git mv src/common                         legacy/src/common
git mv src/data_preprocessor              legacy/src/data_preprocessor
git mv src/map_manager                    legacy/src/map_manager
git mv src/localization                   legacy/src/localization
git mv src/loop_closure                   legacy/src/loop_closure
git mv src/odom_fusion                    legacy/src/odom_fusion
git mv include/fast_lio2_slam/data_preprocessor legacy/include/data_preprocessor
git mv include/fast_lio2_slam/map_manager       legacy/include/map_manager
git mv include/fast_lio2_slam/localization      legacy/include/localization
git mv include/fast_lio2_slam/loop_closure      legacy/include/loop_closure
git mv include/fast_lio2_slam/odom_fusion       legacy/include/odom_fusion
git mv test/test_types.cpp                legacy/test/test_types.cpp
```
> 若 `include/fast_lio2_slam/` 移空,删除空目录。`include/fast_lio2_slam/pch.hpp` 保留(Task E3 处理)。

**Step 2: 更新 CMakeLists — 移除 test_types**
`CMakeLists.txt` 的 `BUILD_TESTING` 块里删除 `test_types` 相关 4 行(`ament_add_gtest(test_types ...)` 及其 target/link/install)。保留 `test_slam_factory`、`test_slam_base_sync`。

**Step 3: 验证编译 + 测试**
Run: `colcon build --packages-select rosiwit_slam && colcon test --packages-select rosiwit_slam`
Expected: SUCCESS;`test_slam_factory` + `test_slam_base_sync` PASS。

**Step 4: Commit**
```bash
git add -A
git commit -m "chore: archive unused/legacy modules out of build (common, data_preprocessor, map_manager, localization, loop_closure, odom_fusion)"
```

---

### Task E2: config 加 slam_algorithm + launch 重命名

**Files:**
- Modify: `config/default.yaml`、`config/velodyne_vlp16.yaml`、`config/ouster_os1.yaml`、`config/ouster_os1_16.yaml`、`config/livox_avia.yaml`、`config/simulation.yaml`、`config/optimized.yaml`
- Rename: `launch/fast_lio2.launch.py` → `launch/slam.launch.py`
- Modify: `launch/livox_avia.launch.py`

**Step 1: 每个 config yaml 顶部加一行**
```yaml
slam_algorithm: fast_lio2
```
(放在文件首行注释之后、`imu_topic:` 之前)

**Step 2: 重命名 launch 并参数化算法名**
```bash
git mv launch/fast_lio2.launch.py launch/slam.launch.py
```
编辑 `launch/slam.launch.py`:
- 增加 `slam_algorithm_arg = DeclareLaunchArgument('slam_algorithm', default_value='fast_lio2')`
- 在 `slam_node` 的 `parameters` 里加 `'slam_algorithm': LaunchConfiguration('slam_algorithm')`
- `config_file` 参数名与现有保持(`config_path`),确保和 `SlamNode::loadParameters` 一致
- `LaunchDescription` 里加入 `slam_algorithm_arg`

`launch/livox_avia.launch.py` 同样加 `slam_algorithm` 参数。

**Step 3: 验证运行**
Run: `ros2 launch rosiwit_slam slam.launch.py`
Expected: 日志 `SlamNode ready: algo=fast_lio2 ...`

**Step 4: Commit**
```bash
git add -A
git commit -m "feat(config): add slam_algorithm field; rename launch to slam.launch.py"
```

---

### Task E3: PCH 路径 + CMake target 命名 + 命名清理

**Files:**
- Move: `include/fast_lio2_slam/pch.hpp` → `include/pch.hpp`
- Modify: `CMakeLists.txt`(target 名 + PCH 路径)

**Step 1: 移动 PCH**
```bash
git mv include/fast_lio2_slam/pch.hpp include/pch.hpp
rmdir include/fast_lio2_slam 2>/dev/null || true   # 清空目录
```

**Step 2: CMakeLists 更新**
- `CMakeLists.txt:135` `set(NODE_NAME fast_lio2_node)` → `set(NODE_NAME slam_node)`
- `CMakeLists.txt:143-145` PCH 路径改为 `${CMAKE_CURRENT_SOURCE_DIR}/include/pch.hpp`
- 确认 `set_target_properties(${NODE_NAME} PROPERTIES OUTPUT_NAME "rosiwit_slam")` 仍在(可执行名不变)

**Step 3: 验证**
Run: `colcon build --packages-select rosiwit_slam && colcon test --packages-select rosiwit_slam`
Expected: SUCCESS;全部测试 PASS。

**Step 4: Commit**
```bash
git add -A
git commit -m "refactor(build): rename target to slam_node, move pch.hpp, drop fast_lio2_slam dir"
```

---

### Task E4: README + 行为一致性最终验证

**Files:**
- Modify: `README.md`(更新架构图、命名、launch 命令)

**Step 1: 更新 README**
把 README 里的 `fast_lio2.launch.py` → `slam.launch.py`;`FastLio2Node` → `SlamNode`;补一段三层架构说明(可摘自设计文档 §4)。移除对 `data_preprocessor`/`map_manager` 等未集成模块的描述(或注明已归档到 `legacy/`)。

**Step 2: 完整行为验证**
Run: `colcon build --packages-select rosiwit_slam --symlink-install`
Run: `colcon test --packages-select rosiwit_slam`
Expected: 编译无警告新增;`test_slam_factory`、`test_slam_base_sync` PASS。
Run(若有 rosbag 或 Gazebo 数据源): `ros2 launch rosiwit_slam slam.launch.py`
Expected: `lio_odom`/`lio_path`/`body_cloud`/`world_cloud`/`cloud_map`/TF 与重构前一致(对比话题频率 `ros2 topic hz /lio_odom` 与位姿轨迹 RViz 显示)。

**Step 3: Commit**
```bash
git add README.md
git commit -m "docs: update README for three-layer architecture"
```

---

## 验收清单

- [ ] `colcon build` 成功,无新增警告
- [ ] `colcon test` 全绿(`test_slam_factory`、`test_slam_base_sync`)
- [ ] `ros2 launch rosiwit_slam slam.launch.py` 启动正常,FastLIO2 行为与重构前一致
- [ ] 源码全局搜索无 `FastLio2Node`、`fast_lio2_slam/` 残留(`grep -rn "FastLio2Node\|fast_lio2_slam" src include` 应只在 `legacy/` 命中)
- [ ] 新增第二算法只需:在 `algorithms/<name>/` 实现 `ISlamAlgorithm` + 工厂 `create()` 加一个 `if` 分支 + config 改 `slam_algorithm` 字段,Node 与 ROS 层零改动

## 非目标(YAGNI)

- 不接入第二个真实算法
- 不重新设计 loop_closure/map_manager 接入方式
- 不引入跨进程通信
