# rosiwit_slam 模块化 SLAM 架构扩展设计

- **日期**: 2026-07-07
- **基础架构**: `docs/plans/2026-07-07-layered-architecture-design.md`(三层架构)
- **设计范围**: 在现有三层架构基础上,将算法层拆分为可组合的模块(前端/后端/回环/定位/地图管理)

---

## 1. 动机与分析

### 现状
当前 rosiwit_slam 的 `ISlamAlgorithm` 接口面向"整包算法"(FastLIO2),仅支持单一算法。用户希望在架构中集成多种 SLAM 组件,包括前端(里程计)、后端(位姿图优化)、回环检测、定位、地图管理等,并支持不同模块的自由组合。

### 参考项目(位于 `E:\workspace_slam`)
三个现有 SLAM 项目的架构分析:

| 项目 | 通信方式 | 架构特点 |
|---|---|---|
| `3d_slam` | ROS1 + 方法调用 | XZSlamInterFace(ROS 层)→MappingInterFace/LocalizationInterFace(SLAM 层),多定位策略(NANO_GICP/ESEKF)可切换 |
| `slam` | ROS2 Node | MappingController/LocalizationController 内拆为独立模块(Odometry/PoseGraph/Loop/Submap),多匹配器(GICP/NDT/Ceres/粒子滤波)可插拔 |
| `slam_3d` | ROS2 + SHM | 类似 3d_slam 但 SHM 零拷贝传输 |

### 关键设计决策(与维护者确认)
1. **功能范围**:全功能(前+后+回环+定位+地图),模块可独立替换和组合
2. **模块间通信**:同进程方法调用(非 ROS 话题)
3. **模块组织**:模块化 + 工厂 + 编排器(Orchestrator)模式
4. **顶层接口不变**:`ISlamAlgorithm` 保持以兼容 `SlamNode`

---

## 2. 整体架构(扩展三层)

```
ROS 接口层 (ros_interface/)
  SlamNode (rclcpp::Node) — 完全不变,只调 ISlamAlgorithm*

SLAM 接口层 (slam_core/)
  ISlamAlgorithm     — 顶层抽象(不变)
  SlamPipeline       — 编排器(实现 ISlamAlgorithm,组合子模块)
  IFrontend          — 前端: 里程计/IMU 处理/点云匹配
  IBackend           — 后端: 位姿图优化/约束管理
  ILoopClosure       — 回环检测与纠正
  IMapManager        — 地图保存/加载/查询/更新
  ILocalization      — 定位(给定地图,持续定位)
  SlamFactory        — 运行时工厂(Pipeline + 子模块均注册)

算法层 (algorithms/)
  fast_lio2/
    FastLio2Frontend     — 原 MapBuilder+IESKF → IFrontend
    FastLio2Pipeline     — 紧耦合版本(前端+后端一体,旧 FastLio2Algorithm)
  ceres_backend/         — Ceres 位姿图优化(从 slam 项目移植)
  gicp_localization/     — GICP/NDT 定位(从 3d_slam 项目移植)
  scan_context_lc/       — ScanContext 回环检测
  pcd_map_manager/       — PCD 文件地图管理
```

---

## 3. 模块接口

### 共享类型(pipeline_types.h)

```cpp
namespace rosiwit_slam {

struct KeyFrame {
    PoseStamped pose;
    CloudType::Ptr cloud;   // 降采样后的关键帧点云
    std::string id;
    int64_t timestamp;
};

struct Constraint {
    std::string from_kf, to_kf;
    PoseStamped relative_pose;
    double cov = 1.0;
};

struct MapInfo {
    std::string name;
    PoseStamped origin;
    size_t point_count;
};

} // namespace rosiwit_slam
```

### 5 个模块接口

```cpp
class IFrontend {
public:
    virtual ~IFrontend() = default;
    virtual bool init(const std::string& config) = 0;
    virtual void onImu(const IMUSample&) = 0;
    virtual void onLidar(const LidarFrame&) = 0;
    virtual bool getOdometry(PoseStamped& out) = 0;
    virtual bool getClouds(CloudType::Ptr& body, CloudType::Ptr& world) = 0;
    virtual bool getKeyFrame(KeyFrame& out) = 0;  // 是否为关键帧
};

class IBackend {
public:
    virtual ~IBackend() = default;
    virtual bool init(const std::string& config) = 0;
    virtual void addKeyFrame(const KeyFrame&) = 0;
    virtual void addConstraints(const std::vector<Constraint>&) = 0;
    virtual bool optimize() = 0;
    virtual bool getUpdatedPoses(std::vector<PoseStamped>&) = 0;
};

class ILoopClosure {
public:
    virtual ~ILoopClosure() = default;
    virtual bool init(const std::string& config) = 0;
    virtual void addKeyFrame(const KeyFrame&) = 0;
    virtual bool detect(PoseStamped& relative, double& cov) = 0;
};

class IMapManager {
public:
    virtual ~IMapManager() = default;
    virtual bool init(const std::string& config) = 0;
    virtual bool saveMap(const std::string& name) = 0;
    virtual bool loadMap(const std::string& name) = 0;
    virtual bool getGlobalMap(PointVec& out) = 0;
    virtual bool addSubMap(const KeyFrame&) = 0;
};

class ILocalization {
public:
    enum Status { INIT, LOCALIZED, LOST };
    virtual ~ILocalization() = default;
    virtual bool init(const std::string& config) = 0;
    virtual void setMap(const std::string& name) = 0;
    virtual void setInitPose(const PoseStamped&) = 0;
    virtual void onImu(const IMUSample&) = 0;
    virtual void onLidar(const LidarFrame&) = 0;
    virtual Status getStatus() = 0;
    virtual bool getPose(PoseStamped& out) = 0;
};
```

---

## 4. SlamPipeline 编排器

```cpp
class SlamPipeline : public ISlamAlgorithm {
public:
    bool init(const std::string& config_path) override;
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;
    SlamState state() const override;
    std::string name() const override { return m_pipeline_name; }
    void setOutputCallback(OutputCallback cb) override;
    bool getGlobalMap(PointVec& out) override;

protected:
    virtual void processFrame(const LidarFrame& frame);  // 可重写编排逻辑

    std::unique_ptr<IFrontend>     m_frontend;
    std::unique_ptr<IBackend>      m_backend;
    std::unique_ptr<ILoopClosure>  m_loop;
    std::unique_ptr<IMapManager>   m_map_mgr;
    std::unique_ptr<ILocalization> m_localization;
    OutputCallback m_cb;
    std::string m_pipeline_name;
};
```

**编排流程**:
```
onImu → frontend->onImu ( + localization->onImu )
onLidar → frontend->onLidar
  → frontend->getKeyFrame → 是? → backend->addKeyFrame + map_mgr->addSubMap
  → backend->optimize (定期)
  → loop->detect → 检测到? → backend->addConstraints + optimize
  → getOdometry + getClouds → 组装 SlamOutput → emitOutput
```

**组合模式(config yaml)**:
```yaml
pipeline: "full_slam"
modules:
  frontend: "fast_lio2_frontend"
  backend: "ceres_pose_graph"
  loop_closure: "scan_context"
  map_manager: "pcd_map"
```

所有模块都是可选的(指针可为空),config 决定哪些组合:

| 配置 | 行为 |
|---|---|
| frontend only | 纯里程计(等价旧 FastLIO2) |
| frontend + backend | 里程计 + 后端优化 |
| frontend + backend + loop | 完整 SLAM 带回环 |
| localization only | 纯定位(需加载地图) |
| frontend + map_manager | 建图(无优化) |

---

## 5. 工厂扩展

```cpp
class SlamFactory {
public:
    // Pipeline 工厂
    static std::unique_ptr<ISlamAlgorithm> create(const std::string& name);
    
    // 子模块工厂
    static std::unique_ptr<IFrontend>     createFrontend(const std::string& name);
    static std::unique_ptr<IBackend>      createBackend(const std::string& name);
    static std::unique_ptr<ILoopClosure>  createLoopClosure(const std::string& name);
    static std::unique_ptr<IMapManager>   createMapManager(const std::string& name);
    static std::unique_ptr<ILocalization> createLocalization(const std::string& name);
};
```

submodule factory 的实现类似:
```cpp
// slam_factory.cpp
std::unique_ptr<IFrontend> SlamFactory::createFrontend(const std::string& name) {
    if (name == "fast_lio2_frontend") return std::make_unique<FastLio2Frontend>();
    return nullptr;
}
std::unique_ptr<IBackend> SlamFactory::createBackend(const std::string& name) {
    if (name == "ceres_pose_graph") return std::make_unique<CeresBackend>();
    return nullptr;
}
// ...
```

---

## 6. 目录结构

```
src/rosiwit_slam/
├── src/
│   ├── ros_interface/slam_node.cpp        # 不变
│   ├── slam_core/
│   │   ├── slam_pipeline.cpp
│   │   ├── i_frontend.cpp                 # 可选:默认/桩实现
│   │   ├── i_backend.cpp
│   │   ├── i_loop_closure.cpp
│   │   ├── i_map_manager.cpp
│   │   ├── i_localization.cpp
│   │   └── slam_factory.cpp               # 扩展注册表
│   ├── algorithms/
│   │   ├── fast_lio2/
│   │   │   ├── fast_lio2_frontend.cpp     # 原算法 → IFrontend
│   │   │   └── map_builder/               # 原有核心不变
│   │   ├── ceres_backend/                 # 后端位姿图优化(新)
│   │   ├── scan_context_lc/               # 回环检测(新)
│   │   ├── gicp_localization/             # GICP/NDT 定位(新)
│   │   └── pcd_map_manager/               # PCD 地图管理(新)
│   └── main.cpp
├── include/
│   ├── ros_interface/slam_node.h          # 不变
│   ├── slam_core/
│   │   ├── slam_pipeline.h
│   │   ├── i_frontend.h / i_backend.h
│   │   ├── i_loop_closure.h
│   │   ├── i_map_manager.h / i_localization.h
│   │   ├── slam_factory.h
│   │   └── pipeline_types.h
│   └── algorithms/
│       ├── fast_lio2/fast_lio2_frontend.h
│       ├── ceres_backend/
│       └── gicp_localization/
├── config/
│   ├── default.yaml              # 旧(兼容)
│   ├── frontend.yaml             # 纯里程计
│   └── full_slam_with_loop.yaml  # 完整 SLAM
└── test/
    ├── test_slam_factory.cpp     # 扩展测试
    ├── test_slam_base_sync.cpp
    └── test_slam_pipeline.cpp    # 新增
```

---

## 7. 分阶段迁移计划

### Phase 1: 接口定义 + FastLIO2 适配
1. 创建 `pipeline_types.h`(KeyFrame/Constraint)
2. 创建 5 个模块接口头文件
3. 创建 `SlamPipeline` 编排类(仅支持 frontend)
4. 创建 `FastLio2Frontend`(封装原 FastLio2Algorithm 逻辑,实现 IFrontend)
5. 扩展工厂注册
6. 测试:创建前端 pipeline + 验证输出

### Phase 2: 后端 + 回环(从 slam/3d_slam 移植)
1. 实现 `CeresBackend`(Ceres 求解器位姿图优化)
2. 实现 `ScanContextLoopClosure`
3. SlamPipeline 组合前端+后端+回环
4. 测试:完整 pipeline 建图

### Phase 3: 定位 + 地图管理
1. 实现 `GicpLocalization`(GICP 配准定位)
2. 实现 `PcdMapManager`
3. SlamPipeline 支持定位模式
4. 测试:加载地图 + 定位

### Phase 4: 清理 + 文档
1. 删除旧 FastLio2Algorithm(完全替换为 FastLio2Frontend + SlamPipeline)
2. 多样化组合配置
3. 更新 README/docs

---

## 8. 兼容性

- **SlamNode 完全不变**:ISlamAlgorithm 接口不变,SlamPipeline 实现它
- **旧 config 兼容**:`slam_algorithm: fast_lio2` → 创建后向兼容的 FastLio2Pipeline(模拟旧行为)
- **测试兼容**:现有 test_slam_factory/test_slam_base_sync 完全不变
- **ros2 launch / rviz**:话题名、参数、框架完全不变

---

## 9. 非目标(YAGNI)

- 不做跨进程/分布式 SLAM(Ros2 topic 通信方式暂不引入)
- 不做多传感器融合的其他类型(视觉、雷达-camera 融合)
- 不做在线/离线 SLAM 分离
- 本设计不包含具体算法实现(如 Ceres 后端、ScanContext 回环)的细节,只定义接口和容错
