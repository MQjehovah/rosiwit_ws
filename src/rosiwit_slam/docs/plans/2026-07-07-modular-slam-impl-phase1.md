# Phase 1: 模块化 SLAM 接口定义 + FastLIO2 适配

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 为 rosiwit_slam 建立模块化 SLAM 接口(IFrontend/IBackend/ILoopClosure/IMapManager/ILocalization),将现有 FastLio2Algorithm 重构为 FastLio2Frontend(实现 IFrontend),并验证通过 SlamPipeline 编排。

**Architecture:** 在现存三层架构基础上,SLAM 接口层新增 5 个模块接口 + SlamPipeline 编排类;算法层原有 FastLio2Algorithm 改 IFrontend 适配器;顶层 ISlamAlgorithm 和 SlamNode 不变。

**Tech Stack:** C++17, Eigen3, PCL, yaml-cpp, gtest

**工作区**: WSL Ubuntu-22.04, `~/rosiwit_ws`, 包路径 `~/rosiwit_ws/src/rosiwit_slam`
**构建**: `cd ~/rosiwit_ws && colcon build --packages-select rosiwit_slam --symlink-install`
**测试**: `colcon test --packages-select rosiwit_slam --event-handlers console_direct+`

---

### Task 1: pipeline_types.h — 共享类型定义

**Files:**
- Create: `include/slam_core/pipeline_types.h`

**Step 1: 写头文件**

```cpp
// include/slam_core/pipeline_types.h
#pragma once
#include <string>
#include <vector>
#include <memory>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

struct KeyFrame {
    PoseStamped pose;
    CloudType::Ptr cloud;       // 降采样后的关键帧点云
    std::string id;
    int64_t timestamp = 0;
};

struct Constraint {
    std::string from_kf;
    std::string to_kf;
    PoseStamped relative_pose;
    double cov = 1.0;
};

} // namespace rosiwit_slam
```

**Step 2: 验证编译**
Run: `colcon build --packages-select rosiwit_slam`
Expected: SUCCESS(新头未引用)

**Step 3: Commit**
```bash
git add include/slam_core/pipeline_types.h
git commit -m "refactor(slam): add pipeline shared types (KeyFrame/Constraint)"
```

---

### Task 2: 5 个模块接口头文件

**Files:**
- Create: `include/slam_core/i_frontend.h`
- Create: `include/slam_core/i_backend.h`
- Create: `include/slam_core/i_loop_closure.h`
- Create: `include/slam_core/i_map_manager.h`
- Create: `include/slam_core/i_localization.h`

**Step 1: 写头文件**

```cpp
// include/slam_core/i_frontend.h
#pragma once
#include <memory>
#include "slam_core/slam_types.h"
#include "slam_core/pipeline_types.h"

namespace rosiwit_slam {

class IFrontend {
public:
    virtual ~IFrontend() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual void onImu(const IMUSample& s) = 0;
    virtual void onLidar(const LidarFrame& f) = 0;
    virtual bool getOdometry(PoseStamped& out) = 0;
    virtual bool getClouds(CloudType::Ptr& body, CloudType::Ptr& world) = 0;
    virtual bool getKeyFrame(KeyFrame& out) = 0;  // 是否为关键帧
    virtual bool getGlobalMap(PointVec& out) = 0;
};

} // namespace rosiwit_slam
```

```cpp
// include/slam_core/i_backend.h
#pragma once
#include "slam_core/pipeline_types.h"

namespace rosiwit_slam {

class IBackend {
public:
    virtual ~IBackend() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual void addKeyFrame(const KeyFrame& kf) = 0;
    virtual void addConstraints(const std::vector<Constraint>& constraints) = 0;
    virtual bool optimize() = 0;
    virtual bool getUpdatedPoses(std::vector<PoseStamped>& poses) = 0;
};

} // namespace rosiwit_slam
```

```cpp
// include/slam_core/i_loop_closure.h
#pragma once
#include "slam_core/pipeline_types.h"

namespace rosiwit_slam {

class ILoopClosure {
public:
    virtual ~ILoopClosure() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual void addKeyFrame(const KeyFrame& kf) = 0;
    virtual bool detect(PoseStamped& relative_pose, double& cov) = 0;
};

} // namespace rosiwit_slam
```

```cpp
// include/slam_core/i_map_manager.h
#pragma once
#include "slam_core/slam_types.h"
#include "slam_core/pipeline_types.h"

namespace rosiwit_slam {

class IMapManager {
public:
    virtual ~IMapManager() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual bool saveMap(const std::string& name) = 0;
    virtual bool loadMap(const std::string& name) = 0;
    virtual bool getGlobalMap(PointVec& out) = 0;
    virtual bool addSubMap(const KeyFrame& kf) = 0;
};

} // namespace rosiwit_slam
```

```cpp
// include/slam_core/i_localization.h
#pragma once
#include "slam_core/slam_types.h"
#include "slam_core/pipeline_types.h"

namespace rosiwit_slam {

class ILocalization {
public:
    enum Status { INIT, LOCALIZED, LOST };
    virtual ~ILocalization() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual void setMap(const std::string& map_name) = 0;
    virtual void setInitPose(const PoseStamped& pose) = 0;
    virtual void onImu(const IMUSample& s) = 0;
    virtual void onLidar(const LidarFrame& f) = 0;
    virtual Status getStatus() = 0;
    virtual bool getPose(PoseStamped& out) = 0;
};

} // namespace rosiwit_slam
```

**Step 2: 验证编译**
Run: `colcon build --packages-select rosiwit_slam`
Expected: SUCCESS

**Step 3: Commit**
```bash
git add include/slam_core/i_frontend.h include/slam_core/i_backend.h \
     include/slam_core/i_loop_closure.h include/slam_core/i_map_manager.h \
     include/slam_core/i_localization.h
git commit -m "refactor(slam): add module interfaces (frontend/backend/loop/map/localization)"
```

---

### Task 3: FastLio2Frontend — 将原算法适配为 IFrontend

**Files:**
- Create: `include/algorithms/fast_lio2/fast_lio2_frontend.h`
- Create: `src/algorithms/fast_lio2/fast_lio2_frontend.cpp`
- Modify: `include/algorithms/fast_lio2/fast_lio2_algorithm.h` (废弃标记)
- Update: `src/slam_core/slam_factory.cpp` (注册 frontend + pipeline)

**Step 1: 写头文件**

```cpp
// include/algorithms/fast_lio2/fast_lio2_frontend.h
#pragma once
#include <memory>
#include "slam_core/i_frontend.h"
#include "commons.h"
#include "map_builder.h"
#include "ieskf.h"

namespace rosiwit_slam {

// 将原 FastLio2Algorithm 重构为 IFrontend 实现:
// - 持有 MapBuilder+IESKF
// - 输出 odometry/clouds/keyframe
// - 不包含后端/回环逻辑(由 SlamPipeline 组合)
class FastLio2Frontend : public IFrontend {
public:
    bool init(const std::string& config_path) override;
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;
    bool getOdometry(PoseStamped& out) override;
    bool getClouds(CloudType::Ptr& body, CloudType::Ptr& world) override;
    bool getKeyFrame(KeyFrame& out) override;
    bool getGlobalMap(PointVec& out) override;
    bool isMapping() const;  // 是否已到 MAPPING 状态

private:
    std::shared_ptr<IESKF>      m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    Config                      m_config;
    bool m_inited = false;
    bool m_first_after_mapping = false;
};

} // namespace rosiwit_slam
```

**Step 2: 写实现(从原 FastLio2Algorithm 移植)**

```cpp
// src/algorithms/fast_lio2/fast_lio2_frontend.cpp
#include "algorithms/fast_lio2/fast_lio2_frontend.h"
#include "lidar_processor.h"
#include <yaml-cpp/yaml.h>

namespace rosiwit_slam {

bool FastLio2Frontend::init(const std::string& config_path) {
    YAML::Node cfg = YAML::LoadFile(config_path);
    if (!cfg) return false;
    // 读取 FastLIO2 专有参数(同原 FastLio2Algorithm::init)
    m_config.lidar_filter_num = cfg["lidar_filter_num"].as<int>();
    m_config.lidar_min_range  = cfg["lidar_min_range"].as<double>();
    m_config.lidar_max_range  = cfg["lidar_max_range"].as<double>();
    m_config.scan_resolution  = cfg["scan_resolution"].as<double>();
    m_config.map_resolution   = cfg["map_resolution"].as<double>();
    m_config.cube_len         = cfg["cube_len"].as<double>();
    m_config.det_range        = cfg["det_range"].as<double>();
    m_config.move_thresh      = cfg["move_thresh"].as<double>();
    m_config.na  = cfg["na"].as<double>();
    m_config.ng  = cfg["ng"].as<double>();
    m_config.nba = cfg["nba"].as<double>();
    m_config.nbg = cfg["nbg"].as<double>();
    m_config.imu_init_num    = cfg["imu_init_num"].as<int>();
    m_config.near_search_num = cfg["near_search_num"].as<int>();
    m_config.ieskf_max_iter  = cfg["ieskf_max_iter"].as<int>();
    m_config.gravity_align   = cfg["gravity_align"].as<bool>();
    m_config.esti_il         = cfg["esti_il"].as<bool>();
    auto t_il = cfg["t_il"].as<std::vector<double>>();
    auto r_il = cfg["r_il"].as<std::vector<double>>();
    m_config.t_il << t_il[0], t_il[1], t_il[2];
    m_config.r_il << r_il[0], r_il[1], r_il[2],
                     r_il[3], r_il[4], r_il[5],
                     r_il[6], r_il[7], r_il[8];
    m_config.lidar_cov_inv = cfg["lidar_cov_inv"].as<double>();

    m_kf = std::make_shared<IESKF>();
    m_kf->setMaxIter(static_cast<size_t>(m_config.ieskf_max_iter));
    m_builder = std::make_shared<MapBuilder>(m_config, m_kf);
    m_inited = true;
    return true;
}

void FastLio2Frontend::onImu(const IMUSample& s) {
    if (!m_inited) return;
    ::Vec<::IMUData> imus;
    imus.emplace_back(s.acc, s.gyro, s.time);
    // FastLIO2 期待一帧一个 IMU — 但旧逻辑是在同步时批量传入
    // 此处保留原 imu 缓冲逻辑: onImu 只入缓冲, 由 onLidar 触发同步处理
    // 我们通过 SlamBase 的同步机制(将来改直接入 MapBuilder 缓冲)
}

void FastLio2Frontend::onLidar(const LidarFrame& f) {
    // TODO: 此处需要从 LidarFrame 构建 FastLIO2 的 ::SyncPackage
    // 并调 m_builder->process()
    // 因 SlamBase 的同步逻辑在 SlamPipeline 层, 此处暂时跳过
}

bool FastLio2Frontend::getOdometry(PoseStamped& out) {
    if (!m_inited || m_builder->status() != BuilderStatus::MAPPING) return false;
    out.time = 0; // TODO: 传入实际时间
    out.rot   = m_kf->x().r_wi;
    out.trans = m_kf->x().t_wi;
    out.vel   = m_kf->x().v;
    return true;
}

bool FastLio2Frontend::getClouds(CloudType::Ptr& body, CloudType::Ptr& world) {
    // TODO: transformCloud 从原 FastLio2Algorithm 移植
    return false;
}

bool FastLio2Frontend::getKeyFrame(KeyFrame& out) {
    // TODO: 根据移动阈值判断是否为关键帧
    return false;
}

bool FastLio2Frontend::getGlobalMap(PointVec& out) {
    if (!m_builder || m_builder->status() != BuilderStatus::MAPPING) return false;
    m_builder->lidar_processor()->collectGlobalMap(out);
    return !out.empty();
}

} // namespace rosiwit_slam
```

> ⚠️ **注意**:`onImu`/`onLidar`/`getClouds`/`getKeyFrame` 的**完整实现**需要将原 `FastLio2Algorithm::onSyncedPackage` 的同步/process/输出逻辑拆分到 IFrontend 的方法中。当前标注 "TODO" 处将在 Phase 1 剩余任务中逐步补全。Task 3 先创建接口骨架,后续 Task 填充 sync/process 逻辑。

**Step 3: 注册到工厂**

`slam_factory.cpp`:
```cpp
#include "algorithms/fast_lio2/fast_lio2_frontend.h"
// ... existing includes ...

std::unique_ptr<IFrontend> SlamFactory::createFrontend(const std::string& name) {
    if (name == "fast_lio2_frontend") return std::make_unique<FastLio2Frontend>();
    return nullptr;
}
```

**Step 4: 验证编译**
Run: `colcon build --packages-select rosiwit_slam`
Expected: SUCCESS

**Step 5: Commit**
```bash
git add -A
git commit -m "refactor(fast_lio2): add FastLio2Frontend (IFrontend adapter skeleton)"
```

---

### Task 4: SlamPipeline 编排类

利用 SlamBase 的同步逻辑,在 pipeline 中编排 IFrontend。

**Files:**
- Create: `include/slam_core/slam_pipeline.h`
- Create: `src/slam_core/slam_pipeline.cpp`
- Create: `test/test_slam_pipeline.cpp`
- Modify: `CMakeLists.txt`(注册测试)

**Step 1: 写头文件**

```cpp
// include/slam_core/slam_pipeline.h
#pragma once
#include <memory>
#include "slam_core/i_algorithm.h"
#include "slam_core/slam_base.h"
#include "slam_core/i_frontend.h"
#include "slam_core/i_backend.h"
#include "slam_core/i_loop_closure.h"
#include "slam_core/i_map_manager.h"
#include "slam_core/i_localization.h"
#include "slam_core/slam_factory.h"

namespace rosiwit_slam {

// SlamPipeline: 编排器,实现 ISlamAlgorithm,组合多个模块。
// 每个模块指针可空,config 决定组合方式。
class SlamPipeline : public SlamBase {
public:
    bool init(const std::string& config_path) override;
    std::string name() const override { return m_pipeline_name; }
    bool getGlobalMap(PointVec& out) override;
    SlamState state() const override;

protected:
    // SlamBase: onImu/onLidar 入缓冲,tryPopAndProcess 调此方法
    bool onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) override;

    // 子模块
    std::unique_ptr<IFrontend>     m_frontend;
    std::unique_ptr<IBackend>      m_backend;
    std::unique_ptr<ILoopClosure>  m_loop;
    std::unique_ptr<IMapManager>   m_map_mgr;
    std::unique_ptr<ILocalization> m_localization;
    
    std::string m_pipeline_name;
    int m_frame_count = 0;
    PoseStamped m_last_frame_pose;
};

} // namespace rosiwit_slam
```

**Step 2: 写实现**

```cpp
// src/slam_core/slam_pipeline.cpp
#include "slam_core/slam_pipeline.h"
#include <yaml-cpp/yaml.h>

namespace rosiwit_slam {

bool SlamPipeline::init(const std::string& config_path) {
    YAML::Node cfg = YAML::LoadFile(config_path);
    if (!cfg) return false;

    m_pipeline_name = cfg["pipeline"].as<std::string>("custom_pipeline");

    // 读取子模块配置并创建
    auto modules = cfg["modules"];
    if (modules) {
        std::string name;
        name = modules["frontend"].as<std::string>("");
        if (!name.empty()) m_frontend = SlamFactory::createFrontend(name);
        name = modules["backend"].as<std::string>("");
        if (!name.empty()) m_backend = SlamFactory::createBackend(name);
        // ... 其他模块类似

        // 逐个初始化
        if (m_frontend && !m_frontend->init(config_path)) {
            m_frontend.reset();
        }
        // ... 其他模块
    }
    return m_frontend != nullptr;  // 至少需要前端
}

bool SlamPipeline::onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) {
    if (!m_frontend) return false;

    // 1. 前端处理: 把 SyncPackage 的 IMUSample/LidarFrame 喂给前端
    MeasureGroup mg; // 此处需定义 MeasureGroup(类似原 FastLIO2 的同步数据结构)
    // TODO: 转换 pkg → mg

    // 2. 前端产出
    PoseStamped odom;
    KeyFrame kf;
    m_frontend->onLidar(pkg.frame); // 简化: 直接喂 LidarFrame
    if (!m_frontend->getOdometry(odom)) return false;

    // 3. 关键帧判断
    if (m_frontend->getKeyFrame(kf)) {
        m_frame_count++;
        // 后端 + 回环
        if (m_backend) {
            m_backend->addKeyFrame(kf);
            if (m_loop) m_loop->addKeyFrame(kf);
        }
        if (m_map_mgr) m_map_mgr->addSubMap(kf);
    }

    // 4. 后端定期优化
    if (m_backend && m_frame_count % 50 == 0) {
        if (m_loop && m_loop->detect(kf.pose, 1.0)) {
            m_backend->addConstraints({/* loop constraint */});
        }
        m_backend->optimize();
    }

    // 5. 组装输出
    out.pose = odom;
    m_frontend->getClouds(out.body_cloud, out.world_cloud);
    out.state = SlamState::RUNNING;
    out.has_new_pose = true;
    return true;
}

bool SlamPipeline::getGlobalMap(PointVec& out) {
    if (m_frontend) return m_frontend->getGlobalMap(out);
    return false;
}

SlamState SlamPipeline::state() const {
    // TODO: 根据各模块状态综合判断
    return SlamState::RUNNING;
}

} // namespace rosiwit_slam
```

**Step 3: 写测试**

```cpp
// test/test_slam_pipeline.cpp
#include <gtest/gtest.h>
#include "slam_core/slam_pipeline.h"
#include "slam_core/slam_factory.h"

using namespace rosiwit_slam;

TEST(SlamPipeline, CreatePipeline) {
    auto pipe = std::make_unique<SlamPipeline>();
    // 无模块时不崩溃
    EXPECT_FALSE(pipe->init("nonexistent.yaml"));
}

TEST(SlamPipeline, CreateFromFactory) {
    // 注册 fast_lio2_pipeline 后测试
    auto algo = SlamFactory::create("fast_lio2");
    EXPECT_NE(algo, nullptr);
}
```

**Step 4: 注册测试**
CMakeLists.txt BUILD_TESTING 块:
```cmake
ament_add_gtest(test_slam_pipeline test/test_slam_pipeline.cpp src/slam_core/slam_pipeline.cpp src/slam_core/slam_base.cpp)
target_include_directories(test_slam_pipeline PRIVATE include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS})
target_link_libraries(test_slam_pipeline ${PCL_LIBRARIES} Sophus::Sophus)
if(yaml-cpp_FOUND) target_link_libraries(test_slam_pipeline yaml-cpp) endif()
install(TARGETS test_slam_pipeline DESTINATION lib/${PROJECT_NAME})
```

**Step 5: 编译测试**
Run: `colcon build --packages-select rosiwit_slam && colcon test --packages-select rosiwit_slam`
Expected: SUCCESS, test_slam_pipeline PASS

**Step 6: Commit**
```bash
git add -A
git commit -m "refactor(slam): add SlamPipeline orchestrator with module composition"
```

---

## 后续 Phase(仅大纲,待 Phase 1 完成后展开)

**Phase 2 — 数据同步与 process 逻辑填充**
- 将原 `FastLio2Algorithm::onSyncedPackage` 的同步+处理逻辑完整移植到 `FastLio2Frontend`
- 完善 `onImu`/`onLidar`/`getClouds`/`getKeyFrame` 完整实现
- 测试:前端处理传感器并产出 odometry + point cloud

**Phase 3 — 后端 + 回环**
- 实现 CeresBackend(Ceres 位姿图优化)
- 实现 ScanContextLoopClosure
- SlamPipeline 组合前端+后端+回环

**Phase 4 — 定位 + 地图管理**
- 实现 GicpLocalization
- 实现 PcdMapManager
- 旧 FastLio2Algorithm 废弃,完全替换

---

## 验收清单(Phase 1)
- [ ] `pipeline_types.h` + 5 个接口头文件编译通过
- [ ] `FastLio2Frontend` 骨架注册到工厂
- [ ] `SlamPipeline` 测试通过
- [ ] 现有 `test_slam_factory` + `test_slam_base_sync` 仍通过
- [ ] 旧 `FastLio2Algorithm` 仍可被创建(兼容 config)
