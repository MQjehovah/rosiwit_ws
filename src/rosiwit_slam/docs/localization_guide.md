# 全局定位功能使用指南

## 功能概述

rosiwit_slam 项目现已支持全局定位（重定位）功能，可以在已有地图中进行全局定位。该功能采用三阶段定位流程：

1. **粗定位阶段**：利用 Scan Context 进行全局描述子匹配
2. **精配准阶段**：使用 NDT 或 ICP 进行精细配准
3. **验证阶段**：配准质量评估，确保定位可靠性

## 模块架构

### 文件结构
```
include/fast_lio2_slam/localization/
    global_localizer.h     - 全局定位核心实现

srv/
    GlobalLocalize.srv     - 全局定位服务定义
    SetInitialPose.srv     - 设置初始位姿服务定义
    GetLocalizationStatus.srv - 获取定位状态服务定义

msg/
    LocalizationStatus.msg - 定位状态消息定义
```

### 核心类

```cpp
class GlobalLocalizer {
    // 全局定位接口
    bool localize(const PointCloudPtr& scan, SE3d& estimated_pose);

    // 设置目标地图
    void setMap(const PointCloudPtr& map_cloud);

    // 设置初始位姿估计
    void setInitialPose(const SE3d& pose);

    // 获取定位状态
    LocalizationState getState() const;
};
```

## 配置参数

在 `config/default.yaml` 中配置定位参数：

```yaml
localization:
  enable: true                # 启用全局定位
  mode: "manual"              # 定位模式: "auto"(自动) 或 "manual"(手动触发)

  # Scan Context 参数（粗定位阶段）
  scan_context:
    ring_num: 20              # 环形分割数
    sector_num: 60            # 扇形分割数
    max_range: 80.0           # 最大距离范围 (米)
    dist_threshold: 0.3       # 距离阈值
    candidate_count: 5        # 候选帧数量

  # 精配准参数
  fine_alignment:
    method: "ndt"             # 配准方法: "ndt" 或 "icp"
    max_iterations: 50        # 最大迭代次数
    convergence_threshold: 0.01  # 收敛阈值
    resolution: 1.0           # NDT 分辨率 (米)
    voxel_size: 0.5           # 下采样体素大小

  # 验证参数
  validation:
    min_fitness_score: 0.7     # 最小匹配得分
    min_inlier_ratio: 0.5      # 最小内点比例
    max_position_error: 2.0    # 最大位置误差 (米)
    max_rotation_error: 0.5    # 最大旋转误差 (弧度)
```

## ROS2 服务接口

### 1. 全局定位服务 `/global_localize`

触发全局定位：

```bash
ros2 service call /global_localize std_srvs/srv/Trigger
```

### 2. 初始位姿订阅 `/initial_pose`

设置初始位姿估计（用于辅助定位）：

```bash
ros2 topic pub /initial_pose geometry_msgs/msg/Pose "{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

### 3. 加载地图服务 `/load_map`

加载已有地图进行定位：

```bash
ros2 service call /load_map std_srvs/srv/Trigger
```

## 定位状态

定位状态通过 `LocalizationState` 枚举表示：

| 状态 | 值 | 描述 |
|------|----|----|
| `UNINITIALIZED` | 0 | 未初始化 |
| `LOCALIZING` | 1 | 正在定位 |
| `LOCALIZED` | 2 | 已定位 |
| `LOST` | 3 | 丢失定位 |

## 使用流程

### 标准定位流程

1. **启动节点并加载地图**
   ```bash
   ros2 run rosiwit_slam rosiwit_slam --ros-args \
       --params-file config/default.yaml \
       -p map.load_on_startup:=true \
       -p map.map_path:=/path/to/your/map
   ```

2. **（可选）设置初始位姿估计**
   ```bash
   ros2 topic pub /initial_pose geometry_msgs/msg/Pose \
       "{position: {x: 10.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}"
   ```

3. **触发全局定位**
   ```bash
   ros2 service call /global_localize std_srvs/srv/Trigger
   ```

4. **检查定位状态**
   定位成功后，系统会自动更新 IEKF 估计器的初始位姿。

### 自动定位模式

在配置文件中设置 `mode: "auto"`，系统会在启动时自动尝试定位：

```yaml
localization:
  mode: "auto"  # 启动时自动定位
```

## 性能指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 定位精度 | < 0.5m | 位置误差 |
| 定位耗时 | < 2s | 完整定位流程时间 |
| 匹配得分 | > 0.7 | fitness_score |

## 与现有模块集成

### 与 Scan Context 集成

GlobalLocalizer 复用现有的 `ScanContext` 类进行粗定位：
- 从地图的关键帧中获取描述子
- 匹配当前扫描的描述子
- 返回候选关键帧和位姿

### 与 GTSAM 后端集成

定位成功后，可以通过 GTSAM 后端进行位姿优化：
- 定位结果作为新的约束添加到因子图中
- 支持全局位姿图优化

### 与地图管理器集成

定位模块与 `MapManager` 和 `MapPersistence` 集成：
- 通过 `setMap()` 接口加载地图点云
- 支持从持久化文件加载地图

## 故障处理

### 定位失败原因

1. **地图未加载**：确保调用 `/load_map` 服务或配置自动加载
2. **无关键帧**：地图需要包含关键帧数据（Scan Context 描述子）
3. **匹配得分低**：检查扫描数据质量，调整 `min_fitness_score` 阈值
4. **定位漂移**：定期触发重定位，或在显著位置变化时重新定位

### 调试建议

1. 检查地图质量：确保地图点云覆盖完整
2. 检查扫描数据：确保 LiDAR 数据正确输入
3. 调整参数：根据环境调整 Scan Context 和 NDT 参数
4. 使用初始位姿：在已知大概位置时提供初始估计

## API 参考

### GlobalLocalizer 主要方法

```cpp
// 初始化
void initialize(const GlobalLocalizerConfig& config);

// 设置地图
void setMap(const PointCloudPtr& map_cloud,
            const std::vector<ScanContextDescriptor>& keyframes);

// 执行定位
LocalizationResult localize(const PointCloudPtr& scan,
                            const SE3d& initial_pose = SE3d());

// 设置初始位姿
void setInitialPose(const SE3d& pose);

// 获取状态
LocalizationState getState() const;
bool isInitialized() const;
bool hasMap() const;

// 添加关键帧
void addKeyframe(const PointCloudPtr& cloud,
                 const SE3d& pose,
                 double timestamp);
```

### LocalizationResult 结构

```cpp
struct LocalizationResult {
    bool success;                // 定位是否成功
    SE3d estimated_pose;         // 估计的位姿
    double fitness_score;        // 配准得分
    double inlier_ratio;         // 内点比例
    int matched_keyframe_id;     // 匹配的关键帧ID
    LocalizationState state;     // 定位状态
    std::string error_message;   // 错误信息

    // 时间信息
    double coarse_time_ms;       // 粗定位耗时
    double fine_time_ms;         // 精配准耗时
    double total_time_ms;        // 总耗时
};
```

## 注意事项

1. **线程安全**：定位过程使用互斥锁保护，确保并发安全
2. **性能优化**：大规模地图建议使用适当的下采样参数
3. **IEKF 集成**：定位成功后需正确更新 IEKF 状态，避免状态冲突
4. **关键帧管理**：建议在建图时保存关键帧数据，便于后续定位

## 更新日志

- 2026-04-29: 初始版本发布，实现三阶段全局定位功能