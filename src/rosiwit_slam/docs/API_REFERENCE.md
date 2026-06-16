# rosiwit_slam API 参考文档

**版本**: 1.2.0
**更新日期**: 2026-05-05
**作者**: AI Development Team

> **包名**: `rosiwit_slam` | **可执行文件**: `rosiwit_slam` | **ROS2 Distro**: Humble

---

## 目录

1. [核心数据结构](#核心数据结构)
2. [模块API](#模块api)
   - [FastLio2Node - ROS2节点主类](#fastlio2node---ros2节点主类)
   - [IekfEstimator - IEKF状态估计器](#iekfestimator---iekf状态估计器)
   - [MapManager - 地图管理](#mapmanager---地图管理)
   - [MapServer - 地图服务](#mapserver---地图服务)
   - [ImuProcessor - IMU处理](#imuprocessor---imu处理)
   - [ScanContext - 闭环检测](#scancontext---闭环检测)
3. [配置参数](#配置参数)
4. [ROS2接口](#ros2接口)

---

## 核心数据结构

### State - 系统状态向量

**文件**: `include/fast_lio2_slam/common/types.h`

24维状态向量，包含位置、姿态、速度、IMU偏置、重力和外参。

```cpp
struct State {
    // 位姿状态
    Vector3d position;          // 位置 (世界坐标系)
    Quaterniond rotation;       // 姿态四元数
    Vector3d velocity;          // 速度

    // IMU偏置
    Vector3d acc_bias;          // 加速度计偏置
    Vector3d gyro_bias;         // 陀螺仪偏置

    // 重力向量
    Vector3d gravity;          // 重力 (默认 [0, 0, -9.81])

    // LiDAR-IMU外参
    Vector3d ext_R;            // LiDAR-IMU旋转 (欧拉角)
    Vector3d ext_T;            // LiDAR-IMU平移

    double timestamp;          // 时间戳
    Eigen::Matrix<double, 24, 24> covariance;  // 协方差矩阵

    // 方法
    SE3d toSE3() const;        // 转换为SE3姿态
    Matrix4d toMatrix() const; // 转换为4x4齐次变换矩阵
};
```

---

### ImuData - IMU测量数据

```cpp
struct ImuData {
    double timestamp;           // 时间戳 (秒)
    Vector3d acc;              // 加速度 (m/s^2)
    Vector3d gyro;            // 角速度 (rad/s)
};
```

---

### PointCloudData - 点云数据

```cpp
struct PointCloudData {
    double timestamp;          // 扫描开始时间戳
    PointCloudPtr cloud;      // 点云指针 (pcl::PointCloud<PointType>)
};
```

---

### ImuBuffer - 线程安全IMU缓冲区

```cpp
class ImuBuffer {
public:
    ImuBuffer(size_t max_size = 1000);

    void addImu(const ImuData& imu);           // 添加IMU数据
    std::vector<ImuData> getImuBetween(double t_start, double t_end);  // 获取时间区间数据
    void clear();                              // 清空缓冲区
    size_t size() const;                       // 获取数据量
    bool empty() const;                        // 是否为空
};
```

---

## 模块API

### FastLio2Node - ROS2节点主类

**文件**: `include/fast_lio2_slam/ros_interface/fast_lio2_node.h`

ROS2节点主类，整合所有模块。

#### 构造函数

```cpp
explicit FastLio2Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
```

#### 初始化方法

| 方法 | 说明 |
|------|------|
| `void initialize()` | 初始化所有组件 |
| `void loadParameters()` | 加载配置参数 |
| `void createSubscribers()` | 创建订阅者 |
| `void createPublishers()` | 创建发布者 |
| `void createServices()` | 创建ROS2服务 |
| `void initializeModules()` | 初始化核心模块 |

#### 数据回调

| 方法 | 说明 |
|------|------|
| `void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)` | 点云数据回调 |
| `void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)` | IMU数据回调 |
| `void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)` | 里程计数据回调 |

#### 核心处理

| 方法 | 说明 |
|------|------|
| `void processLidarData(const PointCloudData& data)` | 处理点云数据 |
| `void runStateEstimation()` | 运行状态估计 |
| `void publishOdometry()` | 发布里程计 |
| `void publishMap()` | 发布地图 |
| `void publishPath()` | 发布轨迹 |
| `void publishTF()` | 发布TF变换 |

---

### IekfEstimator - IEKF状态估计器

**文件**: `include/fast_lio2_slam/fast_lio2_core/iekf_estimator.h`

迭代扩展卡尔曼滤波状态估计器，实现FAST-LIO2核心算法。

#### 配置结构

```cpp
struct IekfConfig {
    // 迭代参数
    int max_iterations = 5;           // 最大迭代次数
    double converge_threshold = 1e-4; // 收敛阈值

    // 测量噪声
    double point_noise = 0.02;        // 点云测量噪声 (m)
    double position_noise = 0.01;     // 位置噪声
    double rotation_noise = 0.005;    // 旋转噪声 (rad)

    // IMU噪声参数
    double acc_noise = 0.1;
    double gyro_noise = 0.01;
    double acc_bias_noise = 0.0001;
    double gyro_bias_noise = 0.00001;

    // 地图匹配参数
    double max_correspondence_dist = 1.0;
    int min_valid_points = 100;
};
```

#### 主要方法

| 方法 | 说明 |
|------|------|
| `void initialize(const IekfConfig& config)` | 初始化估计器 |
| `void setInitialState(const State& state)` | 设置初始状态 |
| `State getState() const` | 获取当前状态 |
| `void predict(const ImuData& imu)` | IMU预测步 |
| `bool update(const PointCloudPtr& cloud, const IKdTree::Ptr& kd_tree)` | LiDAR更新步 |
| `void reset()` | 重置估计器 |

---

### MapManager - 地图管理

**文件**: `include/fast_lio2_slam/map_manager/map_manager.h`

点云地图存储、子地图管理和持久化。

#### 配置结构

```cpp
struct MapManagerConfig {
    double resolution = 0.2;           // 地图分辨率
    double submap_size = 50.0;          // 子地图大小 (米)
    int max_submap_points = 50000;     // 子地图最大点数
    std::string map_path = "./map";    // 地图保存路径
    bool enable_pcd_save = true;       // 启用PCD保存
    bool enable_submap = true;         // 启用子地图
};
```

#### 主要方法

| 方法 | 说明 |
|------|------|
| `void initialize(const MapManagerConfig& config)` | 初始化地图管理器 |
| `void addPointCloud(const PointCloudPtr& cloud, const SE3d& pose, int frame_id)` | 添加点云到地图 |
| `void updateSubmaps(const SE3d& current_pose)` | 更新子地图 |
| `Submap* getActiveSubmap()` | 获取当前活跃子地图 |
| `PointCloudPtr getFullMap()` | 获取完整地图 |
| `PointCloudPtr getLocalMap(const SE3d& pose, double radius)` | 获取局部地图 |
| `bool saveMap(const std::string& path)` | 保存地图 |
| `bool loadMap(const std::string& path)` | 加载地图 |
| `void clearMap()` | 清空地图 |

#### 新增接口（建图功能）

| 方法 | 说明 |
|------|------|
| `PointCloudPtr getVisualizationCloud(double voxel_size = 0.5)` | 获取可视化点云 |
| `bool saveMap(const std::string& path, const std::string& format = "pcd")` | 保存地图到指定格式 |
| `bool loadMap(const std::string& path, bool merge = false)` | 加载地图（可选合并） |
| `void reset()` | 重置地图管理器 |
| `MapMetadata getMetadata() const` | 获取地图元数据 |
| `MapStatistics getStatistics() const` | 获取地图统计信息 |

---

### MapServer - 地图服务

**文件**: `include/fast_lio2_slam/map_manager/map_server.h`

提供ROS2服务接口用于地图的保存、加载、查询等操作。

#### 配置结构

```cpp
struct MapServerConfig {
    // 服务名称
    std::string save_map_service = "/save_map";
    std::string load_map_service = "/load_map";
    std::string clear_map_service = "/clear_map";
    std::string get_map_service = "/get_map";
    std::string save_pcd_service = "/save_pcd";
    std::string get_metadata_service = "/get_map_metadata";

    // 发布话题
    std::string global_map_topic = "/global_map";
    std::string local_map_topic = "/local_map";
    std::string submap_marker_topic = "/submap_markers";
    std::string map_info_topic = "/map_info";

    // 发布频率
    double map_publish_rate = 0.5;     // Hz
    double marker_publish_rate = 1.0;  // Hz

    // 地图保存
    std::string default_save_path = "./map";
    std::string default_format = "pcd";
    bool auto_save_on_shutdown = true;
    double auto_save_interval = 60.0;  // 秒
};
```

#### ROS2服务接口

| 服务名称 | 类型 | 说明 |
|----------|------|------|
| `/save_map` | `std_srvs/srv/Trigger` | 保存地图到默认路径 |
| `/load_map` | `rosiwit_slam/srv/LoadMap` | 从文件加载地图 |
| `/clear_map` | `std_srvs/srv/Trigger` | 清空当前地图 |
| `/get_map` | `rosiwit_slam/srv/GetMap` | 获取当前地图点云 |
| `/save_pcd` | `rosiwit_slam/srv/SavePcd` | 保存PCD文件 |

---

### ImuProcessor - IMU处理

**文件**: `include/fast_lio2_slam/data_preprocessor/imu_processor.h`

IMU数据处理与预积分。

#### 配置结构

```cpp
struct ImuProcessorConfig {
    double acc_noise = 0.1;           // 加速度计噪声 (m/s^2)
    double gyro_noise = 0.01;          // 陀螺仪噪声 (rad/s)
    double acc_bias_noise = 0.0001;   // 加速度计偏置噪声
    double gyro_bias_noise = 0.00001; // 陀螺仪偏置噪声
    double gravity = 9.81;            // 重力加速度

    int static_init_count = 200;       // 静止初始化帧数
    bool estimate_initial_bias = true; // 是否估计初始偏置
};
```

#### 主要方法

| 方法 | 说明 |
|------|------|
| `void initialize(const ImuProcessorConfig& config)` | 初始化处理器 |
| `void setInitialState(const State& state)` | 设置初始状态 |
| `void setBias(const Vector3d& acc_bias, const Vector3d& gyro_bias)` | 设置偏置 |
| `void addImuData(const ImuData& imu)` | 添加IMU数据 |
| `State predict(double t_start, double t_end, const State& state_init)` | IMU状态预测 |
| `void undistortPointCloud(PointCloudPtr cloud, double t_start, double t_end, const std::vector<ImuData>& imu_data)` | 点云去畸变 |
| `bool estimateInitialBias(Vector3d& acc_bias, Vector3d& gyro_bias)` | 估计初始偏置 |

---

### ScanContext - 闭环检测

**文件**: `include/fast_lio2_slam/loop_closure/scan_context.h`

Scan Context闭环检测算法实现。

#### 配置结构

```cpp
struct ScanContextConfig {
    int ring_num = 20;           // 环数
    int sector_num = 60;         // 扇区数
    double max_range = 80.0;     // 最大距离
    double ring_height = 2.0;    // 环高度

    // 匹配参数
    double threshold = 0.3;      // 匹配阈值
    int min_match_count = 3;     // 最小匹配数

    // 搜索参数
    int exclude_near_scan = 50;  // 排除近邻帧
};
```

#### 主要方法

| 方法 | 说明 |
|------|------|
| `void initialize(const ScanContextConfig& config)` | 初始化检测器 |
| `ScanContextDescriptor makeDescriptor(const PointCloudPtr& cloud, double timestamp, int scan_id, const SE3d& pose)` | 生成描述子 |
| `void addKeyframe(const ScanContextDescriptor& desc)` | 添加关键帧 |
| `bool detectLoop(const ScanContextDescriptor& query, LoopConstraint& constraint)` | 检测闭环 |

---

## 配置参数

### ConfigParams - 系统配置汇总

**文件**: `include/fast_lio2_slam/common/config.h`

```cpp
struct ConfigParams {
    // IMU参数
    ImuParams imu;

    // LiDAR参数
    LidarParams lidar;

    // IEKF参数
    IekfParams iekf;

    // 闭环检测参数
    LoopClosureParams loop_closure;

    // 里程计融合参数
    OdomFusionParams odom_fusion;

    // ROS2接口参数
    struct RosParams {
        std::string lidar_topic = "/lidar_points";
        std::string imu_topic = "/imu/data";
        std::string odom_topic = "/odom";
        // ... 更多参数
    } ros;

    // 地图管理参数
    struct MapParams {
        double resolution = 0.2;
        double submap_size = 50.0;
        // ... 更多参数
    } map;

    // 从YAML文件加载配置
    static ConfigParams fromYaml(const std::string& config_file);
};
```

---

## ROS2接口

### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/lidar_points` | `sensor_msgs/msg/PointCloud2` | 点云数据输入 |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU数据输入 |
| `/odom` | `nav_msgs/msg/Odometry` | 外部里程计数据（可选） |

### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/odom_estimated` | `nav_msgs/msg/Odometry` | 估计的里程计 |
| `/path_estimated` | `nav_msgs/msg/Path` | 估计的轨迹 |
| `/cloud_map` | `sensor_msgs/msg/PointCloud2` | 地图点云 |
| `/global_map` | `sensor_msgs/msg/PointCloud2` | 全局地图 |
| `/local_map` | `sensor_msgs/msg/PointCloud2` | 局部地图 |

### 服务接口

| 服务 | 类型 | 说明 |
|------|------|------|
| `/save_map` | `std_srvs/srv/Trigger` | 保存地图 |
| `/load_map` | `rosiwit_slam/srv/LoadMap` | 加载地图 |
| `/clear_map` | `std_srvs/srv/Trigger` | 清空地图 |
| `/save_pcd` | `rosiwit_slam/srv/SavePcd` | 保存PCD文件 |

### TF变换

| 父坐标系 | 子坐标系 | 说明 |
|----------|----------|------|
| `world` | `map` | 世界到地图 |
| `map` | `odom` | 地图到里程计 |
| `odom` | `base_link` | 里程计到机器人本体 |
| `base_link` | `lidar` | 本体到激光雷达 |
| `base_link` | `imu` | 本体到IMU |

---

## 类型别名

```cpp
namespace fast_lio2_slam {
// 基础类型
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Quaterniond = Eigen::Quaterniond;
using SE3d = Sophus::SE3d;
using SO3d = Sophus::SO3d;

// PCL类型
using PointType = pcl::PointXYZINormal;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;
using PointCloudConstPtr = pcl::PointCloud<PointType>::ConstPtr;
}
```

---

## 使用示例

### 初始化节点

```cpp
#include "fast_lio2_slam/ros_interface/fast_lio2_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<fast_lio2_slam::FastLio2Node>(
        rclcpp::NodeOptions()
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 直接使用IEKF估计器

```cpp
#include "fast_lio2_slam/fast_lio2_core/iekf_estimator.h"

// 创建估计器
fast_lio2_slam::IekfConfig config;
config.max_iterations = 5;
config.point_noise = 0.02;

fast_lio2_slam::IekfEstimator estimator(config);

// 设置初始状态
fast_lio2_slam::State init_state;
init_state.position = Eigen::Vector3d::Zero();
init_state.rotation = Eigen::Quaterniond::Identity();
estimator.setInitialState(init_state);

// IMU预测
fast_lio2_slam::ImuData imu_data;
imu_data.timestamp = 0.0;
imu_data.acc = Eigen::Vector3d(0, 0, 9.81);
imu_data.gyro = Eigen::Vector3d::Zero();
estimator.predict(imu_data);

// LiDAR更新
estimator.update(cloud, kd_tree);
```

---

## 错误处理

所有模块遵循统一的错误处理机制：

1. **参数验证**: 初始化时检查配置参数有效性
2. **状态检查**: 关键操作前检查模块初始化状态
3. **异常捕获**: 使用 `try-catch` 处理可能的异常

```cpp
try {
    estimator.initialize(config);
} catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "初始化失败: %s", e.what());
}
```

---

## 版本历史

| 版本 | 日期 | 变更 |
|------|------|------|
| 1.0.0 | 2026-04-24 | 初始版本，包含核心SLAM功能和建图增强 |

---

## 参考资料

- [FAST-LIO2论文](https://arxiv.org/abs/2107.06829)
- [FAST-LIO2代码库](https://github.com/hku-mars/FAST_LIO)
- [Scan Context](https://github.com/irapkaist/SC-LeGO-LOAM)
- [GTSAM](https://gtsam.org/)

---

## 集成 Launch 文件

> 以下 launch 文件位于 `rosiwit_simulator` 包中，用于集成仿真器与 SLAM。

### simulator_slam_demo.launch.py

**位置**: `rosiwit_simulator/launch/simulator_slam_demo.launch.py`
**功能**: 一键启动 Gazebo 仿真器 + SLAM 节点 + 自动运动，用于端到端建图演示

#### 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `use_sim_time` | bool | `true` | 使用仿真时间 |
| `gui` | bool | `false` | Gazebo GUI (WSL2 下建议关闭) |
| `world` | string | `house.world` | Gazebo 世界文件 |
| `motion_pattern` | string | `circle` | 自动运动模式 (`circle` / `figure8` / `square`) |
| `linear_speed` | double | `0.3` | 线速度 (m/s) |
| `angular_speed` | double | `0.2` | 角速度 (rad/s) |
| `slam_config_file` | string | `velodyne_vlp16.yaml` | SLAM 配置文件 |
| `autonomous_motion` | bool | `true` | 是否启动自动运动节点 |

#### 启动节点列表

| 节点 | 包 | 说明 |
|------|------|------|
| `gazebo` | `gazebo_ros` | Gazebo 物理仿真器 (headless) |
| `robot_state_publisher` | `robot_state_publisher` | 机器人 TF 发布 |
| `spawn_entity` | `gazebo_ros` | 在 Gazebo 中生成机器人 |
| `rosiwit_slam` | `rosiwit_slam` | SLAM 节点 |
| `auto_motion_node` | Python script | 自动轨迹运动节点 |

#### 话题匹配

```
rosiwit_simulator                    rosiwit_slam
┌──────────────────┐                ┌──────────────────┐
│ /velodyne_points ├───────────────►│ /velodyne_points  │
│ /imu             ├───────────────►│ /imu              │
│ /cmd_vel         │◄───────────────│ (auto_motion)     │
└──────────────────┘                └──────────────────┘
```

#### 用法

```bash
# 完整集成演示
ros2 launch rosiwit_simulator simulator_slam_demo.launch.py

# 8 字形轨迹
ros2 launch rosiwit_simulator simulator_slam_demo.launch.py motion_pattern:=figure8

# 启用 Gazebo GUI (需要 X11 显示)
ros2 launch rosiwit_simulator simulator_slam_demo.launch.py gui:=true
```
