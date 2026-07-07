# rosiwit_slam 三层分层架构重构设计

- **日期**: 2026-07-07
- **状态**: 已批准,待实现
- **范围**: `src/rosiwit_slam/` 包内重构

## 1. 背景与现状问题

当前 `rosiwit_slam` 包虽然名为 "rosiwit_slam",但内部完全是为 FAST-LIO2 单一算法硬编码的结构,层次混乱,无法切换其他 SLAM 算法。核心问题:

1. **ROS 层与算法层深度耦合**。`FastLio2Node`(`src/ros_interface/fast_lio2_node.cpp`)直接持有 `IESKF` 和 `MapBuilder`,并直接读取算法内部细节来发布数据:
   - `m_kf->x().t_wi` / `m_kf->x().r_wi` / `m_kf->x().v`(IESKF 状态)
   - `m_builder->lidar_processor()` / `m_builder->lidar_processor()->r_wl()` / `t_wl()`
   - `LidarProcessor::transformCloud(...)` / `collectGlobalMap(...)`
   - 换算法等于重写整个 Node。
2. **没有 SLAM 接口层**。`MapBuilder` 是具体类,无抽象基类,无法多态切换。
3. **命名混乱**。类名 `FastLio2*`、include 路径 `include/fast_lio2_slam/`,但包名 `rosiwit_slam`、节点名 `rosiwit_slam`、可执行名 `rosiwit_slam`,自相矛盾。
4. **职责越界**。IMU/LiDAR 缓冲与时间同步(`syncPackage()`)混在 Node 里,本应属于 SLAM 层。`Config` 结构也是 FastLIO2 专用。
5. **大量未集成代码**。`data_preprocessor/`、`map_manager/`、`localization/`、`loop_closure/`、`odom_fusion/` 等模块当前**完全没有被 `FastLio2Node` 引用**,是半成品/历史遗留,却仍在 CMake 编译范围内。

## 2. 目标

- 建立层次分明的三层架构:**ROS 接口层 / SLAM 接口层 / 算法层**。
- 支持运行时切换多种 SLAM 算法(本次仅落地 FastLIO2 一个实现,但接口为多算法预留)。
- 彻底统一命名为 `rosiwit_slam`,消除 `fast_lio2` 残留(算法名本身保留为合理的算法标识)。
- 保证重构后 FastLIO2 的运行行为与重构前一致。

## 3. 关键设计决策(已与维护者确认)

| 决策点 | 选择 | 理由 |
|---|---|---|
| 重构范围 | **彻底重命名 + 重构** | 项目早期,架构最干净,不留命名债 |
| 接口契约 | **接口吃原始 sensor 帧,内部做同步** | Node 极薄,算法可脱离 ROS 独立测试 |
| 切换机制 | **运行时工厂**(按字符串创建) | 一个二进制,配置即可切换 |
| 交付范围 | **本次只做 FastLIO2**,接口尽量通用 | 控制风险,第二算法后续接入 |
| 目录布局 | **扁平,include 不带包名前缀** | 维护者偏好简洁 |
| 未集成模块 | **归档到 `legacy/`,不编译** | 不丢代码,不污染新架构 |

> **已知权衡(目录布局)**:include 不带包名前缀(`#include "slam_core/..."`)不符合 ROS2 `include/<pkg>/` 严格规范,通用目录名 `slam_core` 存在轻微冲突风险。当前包是终端节点、无被依赖,可接受;后续若被其他包依赖头文件需重新评估。

## 4. 三层架构

```
┌──────────────────────── ROS 接口层 (ros_interface/) ────────────────────────┐
│  SlamNode (rclcpp::Node)                                                     │
│  · 订阅 imu/lidar → 转成 IMUSample/LidarFrame → algo->onImu()/onLidar()      │
│  · 注册 output 回调:算法产出时加锁存 m_latest_output                         │
│  · timer(20ms):发 odom/path/body_cloud/world_cloud/tf                       │
│  · map timer(2s):algo->getGlobalMap() → 发 cloud_map                       │
│  · 不感知任何算法内部细节(无 IESKF/LidarProcessor 依赖)                      │
└───────────────────────────────┬──────────────────────────────────────────────┘
                                │ ISlamAlgorithm* (unique_ptr)
┌───────────────────────────────▼──────────────────────────────────────────────┐
│  SLAM 接口层 (slam_core/)                                                     │
│  · ISlamAlgorithm:抽象基类                                                   │
│  · SlamBase:内置 IMU/LiDAR 缓冲 + 时间同步,子类只填 onSyncedPackage()        │
│  · SlamFactory:运行时按字符串创建                                             │
│  · slam_types.h:IMUSample / LidarFrame / SlamOutput / PoseStamped / SlamState│
└───────────────────────────────┬──────────────────────────────────────────────┘
                                │ 继承 SlamBase
┌───────────────────────────────▼──────────────────────────────────────────────┐
│  算法层 (algorithms/)                                                         │
│  · fast_lio2/FastLio2Algorithm:适配器,持有 MapBuilder + IESKF               │
│    - onSyncedPackage() → m_builder->process(),组装 SlamOutput                │
│    - 内部封装 IESKF 状态 → 统一 PoseStamped 转换                              │
│  · fast_lio2/map_builder/:现有核心原样保留(ieskf/imu/lidar/ikd/commons/utils)│
└──────────────────────────────────────────────────────────────────────────────┘
```

**核心解耦点**:Node 只通过 `SlamOutput`(纯位姿+点云)拿结果,永远看不到 `IESKF`/`Config`/`LidarProcessor`。`Config`、`IESKF`、`IMUData` 全部收进 `algorithms/fast_lio2/`。

## 5. 目录结构(方案 B:扁平)

```
src/rosiwit_slam/
├── src/
│   ├── ros_interface/
│   │   └── slam_node.cpp                      # 极薄 ROS 层(替换 fast_lio2_node.cpp)
│   ├── slam_core/
│   │   ├── slam_factory.cpp                   # 工厂(显式注册 fast_lio2)
│   │   └── slam_base.cpp                      # 同步缓冲基类实现
│   ├── algorithms/fast_lio2/
│   │   ├── fast_lio2_algorithm.cpp            # ISlamAlgorithm 适配器
│   │   └── map_builder/                       # 现有核心原样搬入
│   │       ├── ieskf.{h,cpp}
│   │       ├── imu_processor.{h,cpp}
│   │       ├── lidar_processor.{h,cpp}
│   │       ├── ikd_Tree.{h,cpp}
│   │       ├── commons.{h,cpp}
│   │       ├── utils.{h,cpp}
│   │       └── map_builder.{h,cpp}
│   ├── common/                                # thread_pool, profiler(跨层共享)
│   └── main.cpp
├── include/                                   # 扁平,无包名前缀
│   ├── ros_interface/slam_node.h
│   ├── slam_core/
│   │   ├── i_slam_algorithm.h
│   │   ├── slam_base.h
│   │   ├── slam_factory.h
│   │   └── slam_types.h
│   ├── algorithms/fast_lio2/
│   │   ├── fast_lio2_algorithm.h
│   │   └── fast_lio2_config.h
│   └── pch.hpp
├── legacy/                                    # 未集成模块归档(CMake 不编译)
│   ├── data_preprocessor/
│   ├── map_manager/
│   ├── localization/
│   ├── loop_closure/
│   ├── odom_fusion/
│   └── (对应 include 头文件)
├── launch/slam.launch.py                      # (原 fast_lio2.launch.py)
├── config/                                    # 每个 yaml 顶部加 slam_algorithm 字段
├── rviz/, msg/, srv/, test/
└── CMakeLists.txt
```

## 6. 核心接口

### `slam_core/slam_types.h`
```cpp
namespace rosiwit_slam {
struct IMUSample  { V3D acc, gyro; double time; };
struct LidarFrame { CloudType::Ptr cloud; double start_time, end_time; };
enum  class SlamState { INITIALIZING, READY, RUNNING, LOST };
struct PoseStamped { double time; M3D rot; V3D trans; V3D vel; };
struct SlamOutput  {
    SlamState state = SlamState::INITIALIZING;
    PoseStamped pose;
    CloudType::Ptr body_cloud, world_cloud;
    bool has_new_pose = false;
};
}
```

### `slam_core/i_slam_algorithm.h`
```cpp
class ISlamAlgorithm {
public:
    virtual ~ISlamAlgorithm() = default;
    virtual bool   init(const std::string& config_path) = 0;
    virtual void   onImu(const IMUSample& s) = 0;
    virtual void   onLidar(const LidarFrame& f) = 0;
    virtual SlamState state() const = 0;
    virtual std::string name() const = 0;
    using OutputCallback = std::function<void(const SlamOutput&)>;
    virtual void   setOutputCallback(OutputCallback cb) = 0;
    virtual bool   getGlobalMap(PointVec& out) { return false; }
};
```

### `slam_core/slam_base.h`(代码复用基类)
```cpp
class SlamBase : public ISlamAlgorithm {
public:
    void onImu(const IMUSample&) override;         // 入缓冲
    void onLidar(const LidarFrame&) override;       // 入缓冲 + 尝试同步
    void setOutputCallback(OutputCallback cb) override;
    SlamState state() const override;
protected:
    virtual bool onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) = 0;
    void emitOutput(const SlamOutput& out);
private:
    // 沿用现有 syncPackage() 逻辑,基于 IMUSample 的 SyncPackage
    std::deque<IMUSample> m_imu_buf;
    std::deque<LidarFrame> m_lidar_buf;
    std::mutex m_buf_mutex;
    OutputCallback m_cb;
};
```

### `algorithms/fast_lio2/fast_lio2_algorithm.h`(适配器)
```cpp
class FastLio2Algorithm : public SlamBase {
public:
    bool init(const std::string& config_path) override;   // 读 yaml → m_builder_config
    std::string name() const override { return "fast_lio2"; }
protected:
    bool onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) override;
    bool getGlobalMap(PointVec& out) override;
private:
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    Config m_builder_config;
};
```
- `onSyncedPackage` 把基于 `IMUSample` 的 `SyncPackage` 转成 FastLIO2 的 `IMUData` 后调 `m_builder->process()`,再从 `m_kf->x()` / `m_lidar_processor` 组装 `SlamOutput`。
- `commons.h` / `Config` / `IMUData` 搬到 `algorithms/fast_lio2/map_builder/`,对 SLAM 层不可见。

### `slam_core/slam_factory.cpp`(显式注册,避免静态注册的链接坑)
```cpp
std::unique_ptr<ISlamAlgorithm> SlamFactory::create(const std::string& name) {
    if (name == "fast_lio2") return std::make_unique<FastLio2Algorithm>();
    return nullptr;  // 未知算法
}
```
> 后续算法增多时,可改为宏静态注册 + `--whole-archive`,但当前单算法显式注册最简单可靠。

## 7. 数据流

```
imu msg  → SlamNode::imuCB   → IMUSample  → algo->onImu()   ┐
lidar msg→ SlamNode::lidarCB → LidarFrame → algo->onLidar() ┴→ SlamBase 同步
    → FastLio2Algorithm::onSyncedPackage() → MapBuilder.process() → 组装 SlamOutput
    → emitOutput() → SlamNode::onOutput()(加锁存 m_latest_output)
    → SlamNode timer(20ms): 发 odom/path/body_cloud/world_cloud/tf
    → map timer(2s): algo->getGlobalMap() → 发 cloud_map
```

## 8. 命名规范

| 维度 | 现状 | 新 |
|---|---|---|
| 包名 | `rosiwit_slam` | `rosiwit_slam`(不变) |
| C++ namespace | 无(全局) | `rosiwit_slam` |
| include 根 | `include/fast_lio2_slam/` | `include/`(扁平) |
| ROS 层类 | `FastLio2Node` | `SlamNode` |
| CMake target | `fast_lio2_node` | `slam_node` |
| 可执行输出名 | `rosiwit_slam` | `rosiwit_slam`(不变) |
| 算法适配类 | — | `FastLio2Algorithm`(算法名保留) |

## 9. config / launch 改动

- `config/default.yaml` 顶部新增 `slam_algorithm: fast_lio2`;其余 FastLIO2 键原样保留(由 `FastLio2Algorithm::init` 自行读取)。
- 其余传感器配置(`velodyne_vlp16.yaml`、`ouster_os1*.yaml`、`livox_avia.yaml`、`simulation.yaml`、`optimized.yaml`)各自补一行 `slam_algorithm`。
- `launch/fast_lio2.launch.py` → 重命名 `launch/slam.launch.py`,新增 `slam_algorithm` 参数(默认 `fast_lio2`),透传给节点。`executable='rosiwit_slam'` 不变。
- `launch/livox_avia.launch.py` 同步重命名/参数化。

## 10. CMakeLists.txt 改动要点

- `include_directories` 中 `src/map_builder` → `src/algorithms/fast_lio2/map_builder`。
- `NODE_NAME fast_lio2_node` → `slam_node`;`set_target_properties(... OUTPUT_NAME "rosiwit_slam")` 保留。
- PCH 路径 `include/fast_lio2_slam/pch.hpp` → `include/pch.hpp`。
- `legacy/` 位于 `src/` 之外,GLOB_RECURSE 不会扫到,无需特殊排除。
- 源文件 `GLOB_RECURSE "src/*.cpp"` 自动覆盖新结构。

## 11. 测试范围

- 更新 `test/test_types.cpp`:include 路径从 `map_builder/commons.h` 更新到新位置。
- 新增 `test/test_slam_factory.cpp`:验证 `create("fast_lio2")` 返回非空、未知名返回 `nullptr`。
- 新增 `test/test_slam_base_sync.cpp`:喂入构造的 IMU/LiDAR 序列,验证 `SlamBase` 同步触发时机与现状 `syncPackage()` 行为一致。
- `test_imu_processor` / `test_ikd_tree` / `test_map_manager` 等历史遗留测试保持注释/移除状态(CMakeLists 现状已说明)。

## 12. 迁移步骤(供实现计划展开)

1. 建 `include/slam_core/`,写 `slam_types.h` / `i_slam_algorithm.h` / `slam_base.h` / `slam_factory.h`。
2. `src/map_builder/*` 与对应 include 整体平移到 `src/algorithms/fast_lio2/map_builder/`。
3. 写 `FastLio2Algorithm`(搬现有同步+发布逻辑为 `onSyncedPackage`)。
4. 写 `SlamNode`(极薄 ROS 层,替换 `FastLio2Node`)。
5. 改 `main.cpp`:读 `slam_algorithm` 参数 → `SlamFactory::create()` → 交给 `SlamNode`。
6. 未集成模块移 `legacy/`;更新 CMake / launch / config / 测试。
7. `colcon build` + `ros2 launch rosiwit_slam slam.launch.py` 验证 FastLIO2 行为与重构前一致。

## 13. 非目标(YAGNI)

- 本次不接入第二个真实 SLAM 算法(LIO-SAM / Point-LIO 等)。
- 不重新设计 `loop_closure` / `map_manager` 等的接入方式(仅归档,后续单独设计)。
- 不引入跨进程通信或多算法并行。
- 不做在线重定位、纯视觉 VIO 等扩展。
