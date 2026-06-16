# ROS2 Coverage Planner API 参考文档

> **版本**: v2.0
> **最后更新**: 2026-04-29
> **来源**: 从源码头文件自动提取
> **修复**: 转弯优化合并条件已放宽（v2.0）

---

## 目录

1. [命名空间](#命名空间)
2. [核心结构体](#核心结构体)
3. [接口类](#接口类)
4. [规划器类](#规划器类)
5. [分区规划器](#分区规划器)
6. [转弯优化器](#转弯优化器)
7. [地图预处理器](#地图预处理器)
8. [扫描方向优化器](#扫描方向优化器)
9. [工具类](#工具类)
10. [ROS2节点](#ros2节点)
11. [枚举类型](#枚举类型)

---

## 命名空间

所有类型定义在 `coverage_planner` 命名空间中：

```cpp
namespace coverage_planner
```

---

## 核心结构体

### PlannerResult

规划结果结构体，包含路径规划的全部输出信息。

```cpp
struct PlannerResult
{
    bool success;                                          // 规划是否成功
    std::vector<geometry_msgs::msg::PoseStamped> path;    // 规划路径点序列
    double coverage_rate;                                  // 覆盖率 (0.0-1.0)
    double path_length;                                    // 路径长度 (米)
    int turn_count;                                        // 转弯次数
    std::string error_message;                             // 错误信息
};
```

**字段说明**:
| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 规划是否成功完成 |
| `path` | vector<PoseStamped> | 规划出的路径点序列 |
| `coverage_rate` | double | 覆盖率百分比 (0.0-1.0) |
| `path_length` | double | 路径总长度（米） |
| `turn_count` | int | 路径中的转弯次数 |
| `error_message` | string | 失败时的错误描述 |

---

### PlannerConfig

规划器配置参数结构体。

```cpp
struct PlannerConfig
{
    double robot_radius;           // 机器人半径 (米)
    double coverage_resolution;    // 覆盖路径分辨率 (米)
    double inflation_radius;       // 障碍物膨胀半径 (米)
    bool enable_optimization;      // 是否启用路径优化
    int direction_optimization;    // 扫描方向优化 (0: 水平, 1: 垂直, 2: 自动)
};
```

**字段说明**:
| 字段 | 类型 | 说明 | 默认值 |
|------|------|------|--------|
| `robot_radius` | double | 机器人物理半径 | 0.3 |
| `coverage_resolution` | double | 路径点间隔 | 0.05 |
| `inflation_radius` | double | 障碍物膨胀距离 | robot_radius |
| `enable_optimization` | bool | 启用路径优化 | true |
| `direction_optimization` | int | 扫描方向选择 | 2 (自动) |

---

### Point2D

2D栅格坐标点结构。

```cpp
struct Point2D
{
    int x;  // X坐标
    int y;  // Y坐标

    Point2D() : x(0), y(0) {}
    Point2D(int px, int py) : x(px), y(py) {}

    bool operator==(const Point2D & other) const;
    bool operator!=(const Point2D & other) const;
    double distanceTo(const Point2D & other) const;
};
```

**方法**:
- `operator==`: 判断两点是否相等
- `operator!=`: 判断两点是否不等
- `distanceTo`: 计算到另一点的欧氏距离

---

### CoverageStats

覆盖统计信息结构。

```cpp
struct CoverageStats
{
    int total_cells;           // 总栅格数
    int free_cells;            // 空闲栅格数
    int covered_cells;         // 已覆盖栅格数
    double coverage_rate;      // 覆盖率 (0.0-1.0)
    double path_length;        // 路径长度 (米)
    int turn_count;            // 转弯次数
};
```

---

### ScanLine

扫描线段结构（ZigzagPlanner使用）。

```cpp
struct ScanLine
{
    int y;              // Y坐标（水平扫描）或X坐标（垂直扫描）
    int x_start;        // 起始X坐标
    int x_end;          // 结束X坐标
    bool is_forward;    // 扫描方向

    ScanLine() : y(0), x_start(0), x_end(0), is_forward(true) {}
    ScanLine(int y_coord, int x_s, int x_e, bool forward = true);
};
```

---

### RegionBoundary

区域边界结构（SpiralPlanner使用）。

```cpp
struct RegionBoundary
{
    int x_min;
    int x_max;
    int y_min;
    int y_max;

    RegionBoundary() : x_min(0), x_max(0), y_min(0), y_max(0) {}
    RegionBoundary(int xmin, int xmax, int ymin, int ymax);

    int width() const;
    int height() const;
    bool contains(const Point2D & p) const;
};
```

---

## 接口类

### IPlanner

规划器抽象接口，定义统一API（策略模式）。

```cpp
class IPlanner
{
public:
    virtual ~IPlanner() = default;

    /**
     * @brief 规划全覆盖路径
     * @param map 栅格地图
     * @param start_pose 起始位姿
     * @param config 规划配置参数
     * @return PlannerResult 规划结果
     */
    virtual PlannerResult plan(
        const nav_msgs::msg::OccupancyGrid & map,
        const geometry_msgs::msg::Pose & start_pose,
        const PlannerConfig & config) = 0;

    /**
     * @brief 获取规划器名称
     * @return std::string 规划器名称
     */
    virtual std::string getName() const = 0;

    /**
     * @brief 重置规划器状态
     */
    virtual void reset() = 0;
};
```

**虚函数**:
| 方法 | 说明 |
|------|------|
| `plan()` | 执行路径规划，返回完整结果 |
| `getName()` | 返回规划器名称标识 |
| `reset()` | 清空内部状态，准备新规划 |

---

## 规划器类

### ZigzagPlanner

弓字形路径规划器，实现BSA扫描线算法。

```cpp
class ZigzagPlanner : public IPlanner
{
public:
    ZigzagPlanner();
    ~ZigzagPlanner() override = default;

    PlannerResult plan(
        const nav_msgs::msg::OccupancyGrid & map,
        const geometry_msgs::msg::Pose & start_pose,
        const PlannerConfig & config) override;

    std::string getName() const override { return "ZigzagPlanner"; }
    void reset() override;
};
```

**算法特性**:
- BSA扫描线分割
- 障碍物打断恢复
- 自动选择最优扫描方向（减少转弯）
- 水平/垂直扫描模式切换

**适用场景**:
- 开阔空间、矩形房间
- 需要最少转弯次数
- 路径可预测性高

---

### SpiralPlanner

回字形路径规划器，实现螺旋覆盖算法。

```cpp
class SpiralPlanner : public IPlanner
{
public:
    SpiralPlanner();
    ~SpiralPlanner() override = default;

    PlannerResult plan(
        const nav_msgs::msg::OccupancyGrid & map,
        const geometry_msgs::msg::Pose & start_pose,
        const PlannerConfig & config) override;

    std::string getName() const override { return "SpiralPlanner"; }
    void reset() override;
};
```

**算法特性**:
- 从外向内螺旋生成
- 非凸区域分解
- 降级策略（失败时切换到Zigzag）
- 顺时针/逆时针方向选择

**适用场景**:
- 复杂障碍物环境
- 不规则形状区域
- 需要平滑转弯路径
- 减少机器人磨损

---

### PlannerContext

策略上下文类，管理规划器实例和算法切换。

```cpp
class PlannerContext
{
public:
    PlannerContext();
    ~PlannerContext() = default;

    /**
     * @brief 选择规划器
     * @param mode 规划模式枚举
     * @return IPlanner* 规划器实例指针
     */
    IPlanner* selectPlanner(PlannerMode mode);

    /**
     * @brief 通过字符串选择规划器
     * @param mode_str 规划模式字符串 ("zigzag" 或 "spiral")
     * @return IPlanner* 规划器实例指针
     */
    IPlanner* selectPlanner(const std::string & mode_str);

    /**
     * @brief 获取当前规划器
     * @return IPlanner* 当前规划器实例指针
     */
    IPlanner* getCurrentPlanner() const;

    /**
     * @brief 获取当前规划模式
     * @return PlannerMode 当前模式
     */
    PlannerMode getCurrentMode() const;

    /**
     * @brief 重置所有规划器状态
     */
    void resetAll();
};
```

---

## 工具类

### MapUtils

地图处理工具类，提供静态方法。

```cpp
class MapUtils
{
public:
    /**
     * @brief 检查点是否在地图范围内
     */
    static bool isInBounds(const nav_msgs::msg::OccupancyGrid & map, int x, int y);

    /**
     * @brief 检查点是否为障碍物
     * @param threshold 阜碍物阈值 (默认50)
     */
    static bool isObstacle(const nav_msgs::msg::OccupancyGrid & map, int x, int y, int threshold = 50);

    /**
     * @brief 检查点是否为空闲区域
     * @param threshold 阜碍物阈值 (默认50)
     */
    static bool isFree(const nav_msgs::msg::OccupancyGrid & map, int x, int y, int threshold = 50);

    /**
     * @brief 地图膨胀（障碍物膨胀）
     * @param robot_radius 膨胀半径
     */
    static nav_msgs::msg::OccupancyGrid inflateMap(
        const nav_msgs::msg::OccupancyGrid & map,
        double robot_radius);

    /**
     * @brief BFS可达性检查
     * @param start 起点栅格坐标
     */
    static std::vector<Point2D> getReachableCells(
        const nav_msgs::msg::OccupancyGrid & map,
        const Point2D & start);

    /**
     * @brief 获取所有空闲栅格
     */
    static std::vector<Point2D> getFreeCells(
        const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 世界坐标转栅格坐标
     */
    static Point2D worldToGrid(
        const nav_msgs::msg::OccupancyGrid & map,
        double world_x, double world_y);

    /**
     * @brief 栅格坐标转世界坐标
     */
    static geometry_msgs::msg::Point gridToWorld(
        const nav_msgs::msg::OccupancyGrid & map,
        int grid_x, int grid_y);

    /**
     * @brief 选择最优扫描方向
     * @return 0: 水平最优, 1: 垂直最优
     */
    static int getOptimalScanDirection(
        const nav_msgs::msg::OccupancyGrid & map);
};
```

---

### PathUtils

路径处理工具类。

```cpp
class PathUtils
{
public:
    /**
     * @brief 计算路径长度
     */
    static double calculatePathLength(
        const std::vector<geometry_msgs::msg::PoseStamped> & path);

    /**
     * @brief 计算转弯次数
     * @param angle_threshold 转弯角度阈值 (默认45度)
     */
    static int calculateTurnCount(
        const std::vector<geometry_msgs::msg::PoseStamped> & path,
        double angle_threshold = 45.0);

    /**
     * @brief 路径平滑
     * @param smoothing_factor 平滑因子 (0.0-1.0)
     */
    static std::vector<geometry_msgs::msg::PoseStamped> smoothPath(
        const std::vector<geometry_msgs::msg::PoseStamped> & path,
        double smoothing_factor = 0.5);

    /**
     * @brief 计算覆盖率统计
     * @param visited_mask 覆盖标记矩阵
     */
    static CoverageStats calculateCoverage(
        const nav_msgs::msg::OccupancyGrid & map,
        const std::vector<geometry_msgs::msg::PoseStamped> & path,
        const std::vector<std::vector<bool>> & visited_mask);
};
```

---

## ROS2节点

### CoveragePlannerNode

ROS2全覆盖路径规划节点主类。

```cpp
class CoveragePlannerNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param options ROS2节点选项
     */
    explicit CoveragePlannerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    ~CoveragePlannerNode() override;
};
```

**ROS2接口**:

| 接口类型 | 名称 | 消息类型 | 说明 |
|----------|------|----------|------|
| Subscription | `/map` | nav_msgs/OccupancyGrid | 栅格地图输入 |
| Subscription | `/initialpose` | PoseWithCovarianceStamped | 起始位置输入 |
| Publisher | `/coverage_path` | nav_msgs/Path | 规划路径输出 |
| Service | `/plan_coverage` | std_srvs/Trigger | 触发规划服务 |

**ROS2参数**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `coverage_mode` | string | "zigzag" | 规划模式 (zigzag/spiral) |
| `robot_radius` | double | 0.3 | 机器人半径（米） |
| `coverage_resolution` | double | 0.05 | 路径分辨率（米） |
| `max_map_width` | int | 1000 | 最大地图宽度限制 |
| `max_map_height` | int | 1000 | 最大地图高度限制 |

---

## 分区规划器

### ZoneDecomposer

分区规划器，负责将复杂地图分解为多个简单区域。

```cpp
class ZoneDecomposer
{
public:
    /**
     * @brief 构造函数
     * @param config 分区配置参数
     */
    explicit ZoneDecomposer(const ZoneDecomposerConfig & config);

    /**
     * @brief 执行分区分解
     * @param map 输入地图
     * @return std::vector<Zone> 分区列表
     */
    std::vector<Zone> decompose(const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 查找区域连接通道
     * @param zones 分区列表
     * @param map 输入地图
     * @return std::vector<ConnectionChannel> 连接通道列表
     */
    std::vector<ConnectionChannel> findConnectionChannels(
        std::vector<Zone> & zones,
        const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 设置配置参数
     * @param config 新配置
     */
    void setConfig(const ZoneDecomposerConfig & config);

private:
    ZoneDecomposerConfig config_;
    // ...
};
```

#### ZoneDecomposerConfig 配置参数

| 字段 | 类型 | 说明 | 默认值 |
|------|------|------|--------|
| `min_zone_area` | int | 最小区域面积（栅格数） | 100 |
| `enable_rectangular_split` | bool | 是否启用矩形分割 | true |
| `max_zone_count` | int | 最大分区数量 | 20 |
| `connection_search_radius` | int | 连接点搜索半径 | 5 |

#### Zone 分区结构

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | int | 区域ID |
| `type` | ZoneType | 区域类型（矩形/走廊/复杂/障碍物） |
| `bounding_box` | cv::Rect | 外接矩形 |
| `free_cells` | vector<Point> | 可通行栅格点 |
| `area` | int | 区域面积（栅格数） |
| `centroid` | cv::Point | 质心 |
| `optimal_scan_direction` | int | 最优扫描方向 |
| `neighbor_zones` | vector<int> | 相邻区域ID列表 |

---

## 转弯优化器

### TurnOptimizer

转弯优化器，负责减少路径转弯次数。

```cpp
class TurnOptimizer
{
public:
    /**
     * @brief 构造函数
     * @param config 优化配置参数
     */
    explicit TurnOptimizer(const TurnOptimizerConfig & config);

    /**
     * @brief 优化路径转弯
     * @param path 原始路径
     * @return TurnOptimizeResult 优化结果
     */
    TurnOptimizeResult optimize(
        const std::vector<geometry_msgs::msg::PoseStamped> & path);

    /**
     * @brief 识别转弯点
     * @param path 输入路径
     * @return std::vector<TurnPoint> 转弯点列表
     */
    std::vector<TurnPoint> identifyTurns(
        const std::vector<geometry_msgs::msg::PoseStamped> & path);

    /**
     * @brief 合并相邻转弯点
     * @param turns 转弯点列表
     * @return std::vector<TurnPoint> 合并后的转弯点
     */
    std::vector<TurnPoint> mergeTurns(const std::vector<TurnPoint> & turns);

private:
    TurnOptimizerConfig config_;
    // ...
};
```

#### TurnOptimizerConfig 配置参数

| 字段 | 类型 | 说明 | 默认值 | v2.0修复 |
|------|------|------|--------|---------|
| `angle_threshold` | double | 转弯检测角度阈值（弧度） | 0.1 (约5.7°) | - |
| `merge_distance_threshold` | double | 转弯点合并距离阈值 | **50.0** | 原10.0→50.0 |
| `merge_angle_threshold` | double | 合并角度阈值（弧度） | 0.35 (约20°) | - |

**v2.0修复说明**：
- `merge_distance_threshold`从10栅格增加到50栅格，适配zigzag路径分辨率
- 合并条件放宽：GENTLE和MEDIUM类型均可合并（覆盖约51%转弯点）
- SHARP转弯特殊处理：距离限制20栅格
| `merge_angle_threshold` | double | 转弯合并角度阈值 | 0.35 (约20°) |
| `enable_merge` | bool | 是否启用转弯合并 | true |
| `enable_smooth` | bool | 是否启用转弯平滑 | false |

#### TurnPoint 转弯点结构

| 字段 | 类型 | 说明 |
|------|------|------|
| `index` | size_t | 在路径中的索引 |
| `type` | TurnType | 转弯类型 |
| `angle` | double | 转弯角度（弧度） |
| `can_merge` | bool | 是否可合并 |
| `position` | Point | 转弯位置 |

#### TurnType 转弯类型枚举

```cpp
enum class TurnType
{
    NONE,           // 无转弯
    SHARP,          // 急转弯 (>90°)
    MEDIUM,         // 中等转弯 (45°-90°)
    GENTLE,         // 缓转弯 (<45°)
    U_TURN,         // U形转弯 (180°)
    SCANLINE_END    // 扫描线末端转弯
};
```

---

## 地图预处理器

### MapPreprocessor

地图预处理器，提供形态学处理和障碍物简化功能。

```cpp
class MapPreprocessor
{
public:
    /**
     * @brief 默认构造函数
     */
    MapPreprocessor();

    /**
     * @brief 预处理地图
     * @param map 输入地图
     * @param config 预处理配置
     * @return nav_msgs::msg::OccupancyGrid 处理后的地图
     */
    nav_msgs::msg::OccupancyGrid preprocess(
        const nav_msgs::msg::OccupancyGrid & map,
        const PreprocessConfig & config);

    /**
     * @brief 形态学处理
     * @param map 输入地图
     * @param config 配置参数
     * @return nav_msgs::msg::OccupancyGrid 处理后的地图
     */
    nav_msgs::msg::OccupancyGrid applyMorphology(
        const nav_msgs::msg::OccupancyGrid & map,
        const PreprocessConfig & config);

    /**
     * @brief 合并邻近障碍物
     * @param map 输入地图
     * @param merge_distance 合并距离阈值
     * @return nav_msgs::msg::OccupancyGrid 处理后的地图
     */
    nav_msgs::msg::OccupancyGrid mergeObstacles(
        const nav_msgs::msg::OccupancyGrid & map,
        double merge_distance);

    /**
     * @brief 填充空洞
     * @param map 输入地图
     * @param max_hole_size 最大空洞尺寸
     * @return nav_msgs::msg::OccupancyGrid 处理后的地图
     */
    nav_msgs::msg::OccupancyGrid fillHoles(
        const nav_msgs::msg::OccupancyGrid & map,
        int max_hole_size);
};
```

#### PreprocessConfig 配置参数

| 字段 | 类型 | 说明 | 默认值 |
|------|------|------|--------|
| `enable_morphology` | bool | 是否启用形态学处理 | true |
| `morphology_kernel_size` | int | 形态学核大小 | 3 |
| `opening_iterations` | int | 开运算迭代次数 | 1 |
| `closing_iterations` | int | 闭运算迭代次数 | 1 |
| `enable_obstacle_merge` | bool | 是否启用障碍物合并 | true |
| `obstacle_merge_distance` | double | 障碍物合并距离 | 3.0 |
| `min_obstacle_size` | int | 最小障碍物尺寸 | 2 |
| `enable_hole_filling` | bool | 是否启用空洞填充 | true |
| `max_hole_size` | int | 最大空洞尺寸 | 5 |

---

## 扫描方向优化器

### ScanDirectionOptimizer

扫描方向优化器，实现长边优先策略。

```cpp
class ScanDirectionOptimizer
{
public:
    /**
     * @brief 默认构造函数
     */
    ScanDirectionOptimizer();

    /**
     * @brief 分析最优扫描方向
     * @param map 输入地图
     * @param config 优化配置
     * @return ScanDirectionResult 分析结果
     */
    ScanDirectionResult analyze(
        const nav_msgs::msg::OccupancyGrid & map,
        const ScanDirectionConfig & config);

    /**
     * @brief PCA主方向检测
     * @param map 输入地图
     * @return double 主方向角度（弧度）
     */
    double detectPCADirection(const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 最小外接矩形分析
     * @param map 输入地图
     * @return double 长边方向角度（弧度）
     */
    double analyzeMBR(const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 计算地图长宽比
     * @param map 输入地图
     * @return double 长宽比
     */
    double calculateAspectRatio(const nav_msgs::msg::OccupancyGrid & map);

private:
    ScanDirectionConfig config_;
    // ...
};
```

#### ScanDirectionConfig 配置参数

| 字段 | 类型 | 说明 | 默认值 |
|------|------|------|--------|
| `enable_pca` | bool | 是否启用PCA方向检测 | true |
| `enable_mbr` | bool | 是否启用最小外接矩形 | true |
| `aspect_ratio_threshold` | double | 长宽比阈值 | 2.0 |
| `pca_threshold` | double | PCA主方向置信度阈值 | 0.1 |
| `fallback_to_scanline` | bool | 失败时回退到扫描线统计 | true |

#### ScanDirectionResult 分析结果

| 字段 | 类型 | 说明 |
|------|------|------|
| `direction` | int | 推荐扫描方向 (0:水平, 1:垂直) |
| `confidence` | double | 推荐置信度 (0.0-1.0) |
| `principal_angle` | double | PCA主方向角度（弧度） |
| `aspect_ratio` | double | 地图长宽比 |
| `pca_variance_ratio` | double | PCA方差比 |
| `method_used` | string | 使用的方法名称 |

---

## 枚举类型

### PlannerMode

规划模式枚举。

```cpp
enum class PlannerMode
{
    ZIGZAG,    // 弓字形模式
    SPIRAL,    // 回字形模式
};
```

---

### SpiralDirection

螺旋方向枚举。

```cpp
enum class SpiralDirection
{
    CLOCKWISE,          // 顺时针
    COUNTER_CLOCKWISE,  // 逆时针
};
```

---

## 使用示例

### 基本使用

```cpp
#include "coverage_planner/planner_context.hpp"
#include "coverage_planner/i_planner.hpp"

// 创建策略上下文
coverage_planner::PlannerContext context;

// 选择规划器
auto planner = context.selectPlanner("zigzag");

// 配置参数
coverage_planner::PlannerConfig config;
config.robot_radius = 0.3;
config.coverage_resolution = 0.05;
config.enable_optimization = true;

// 执行规划
auto result = planner->plan(map, start_pose, config);

if (result.success) {
    std::cout << "覆盖率: " << result.coverage_rate * 100 << "%" << std::endl;
    std::cout << "路径长度: " << result.path_length << " 米" << std::endl;
    std::cout << "转弯次数: " << result.turn_count << std::endl;
}
```

### 算法切换

```cpp
// 切换到回字形算法
auto spiral_planner = context.selectPlanner("spiral");

// 或者使用枚举
auto zigzag_planner = context.selectPlanner(coverage_planner::PlannerMode::ZIGZAG);
```

---

## 文件位置

| 文件 | 路径 |
|------|------|
| i_planner.hpp | include/coverage_planner/i_planner.hpp |
| zigzag_planner.hpp | include/coverage_planner/zigzag_planner.hpp |
| spiral_planner.hpp | include/coverage_planner/spiral_planner.hpp |
| planner_context.hpp | include/coverage_planner/planner_context.hpp |
| coverage_utils.hpp | include/coverage_planner/coverage_utils.hpp |
| coverage_planner.hpp | include/coverage_planner/coverage_planner.hpp |
| **zone_decomposer.hpp** | include/coverage_planner/zone_decomposer.hpp |
| **turn_optimizer.hpp** | include/coverage_planner/turn_optimizer.hpp |
| **map_preprocessor.hpp** | include/coverage_planner/map_preprocessor.hpp |
| **scan_direction_optimizer.hpp** | include/coverage_planner/scan_direction_optimizer.hpp |

---

## 相关文档

- [架构说明](architecture.md) - 系统架构和设计决策
- [变更记录](changelog.md) - 版本历史和变更日志
- [README](../README.md) - 项目概述和使用指南
- [安全审计报告](security_report.md) - 安全漏洞修复记录