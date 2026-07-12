// ============================================================
// Diffbot Navigation - A* 规划器
// A*路径规划算法实现
// ============================================================

#ifndef ROSIWIT_NAVIGATION__PLANNERS__ASTAR_PLANNER_HPP_
#define ROSIWIT_NAVIGATION__PLANNERS__ASTAR_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

#include "rosiwit_navigation/nav_core/i_planner.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"
#include "rosiwit_navigation/nav_core/logger.hpp"

namespace rosiwit_navigation
{
namespace planners
{

/**
 * @brief 简化占据栅格地图（用于直接调用场景）
 */
struct OccupancyGridInfo
{
    unsigned int width = 0;
    unsigned int height = 0;
    double resolution = 0.05;

    struct Origin
    {
        double x = 0.0;
        double y = 0.0;
    } origin;
};

struct OccupancyGrid
{
    OccupancyGridInfo info;
    std::vector<int8_t> data;
};

/**
 * @brief 路径点（简化）
 */
struct PathPoint
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

/**
 * @brief A*节点结构
 */
struct AStarNode
{
    int x, y;                   // 地图坐标
    double g_cost;              // 从起点到当前节点的代价
    double h_cost;              // 从当前节点到目标的启发式代价
    double f_cost;              // f = g + h
    int parent_x, parent_y;     // 父节点坐标

    AStarNode() : x(0), y(0), g_cost(0), h_cost(0), f_cost(0),
                  parent_x(-1), parent_y(-1) {}

    bool operator<(const AStarNode& other) const {
        return f_cost > other.f_cost;  // 小顶堆
    }
};

// ---------------------------------------------------------------------------
// A* 规划器常量
// ---------------------------------------------------------------------------
namespace AStarConstants {
    constexpr double kStraightCost = 1.0;                    // 直线移动代价
    constexpr double kDiagonalCost = 1.414;                  // 对角线移动代价 (√2)
    constexpr unsigned char kObstacleThreshold = 254;        // 障碍物判定阈值
    constexpr double kCostFactorScale = 3.0;                 // 梯度代价缩放，越大路径越倾向远离障碍物
    constexpr unsigned char kMaxCost = 255;                  // 最大代价值
    constexpr int kDefaultMaxIterations = 50000;             // 默认最大迭代次数（50K 确保大网格<2s）
    constexpr double kDefaultTimeoutSeconds = 2.0;           // 默认规划超时（秒）
    constexpr double kDefaultGridResolution = 0.05;          // 默认栅格分辨率（米）
    constexpr double kDefaultHeuristicWeight = 1.0;          // 默认启发式权重
    constexpr double kEpsilon = 1e-9;                        // 浮点比较容差
}

/**
 * @brief A* 规划结果（简化接口返回值）
 */
struct AStarResult
{
    bool success = false;
    std::vector<PathPoint> path;
    int iterations = 0;
    double elapsed_ms = 0.0;
};

/**
 * @class AStarPlanner
 * @brief A*规划器实现
 *
 * 使用A*算法进行最优路径规划
 * 继承 IPlannerStrategy 抽象接口，支持策略模式
 */
class AStarPlanner : public core::IPlannerStrategy
{
public:
    /**
     * @brief 简化的规划器配置（用于直接调用场景）
     */
    struct Config
    {
        double grid_resolution = 0.05;
        int max_iterations = 50000;
        double timeout_seconds = 2.0;
        double heuristic_weight = 1.0;
        bool use_diagonal_moves = true;
        int obstacle_cost_threshold = 254;
    };

    AStarPlanner();
    ~AStarPlanner() override;

    // ===================================================================
    // 简化接口（用于非ROS直接调用场景）
    // ===================================================================

    /**
     * @brief 配置规划器（简化接口）
     * @param config 简化配置参数
     */
    void configure(const Config& config);

    /**
     * @brief 执行路径规划（简化接口）
     * @param grid 占据栅格地图
     * @param start 起点（网格坐标）
     * @param goal 终点（网格坐标）
     * @return 规划结果（包含路径、迭代次数、耗时）
     *
     * 边界处理：
     * - grid 为空 → 返回 success=false
     * - start/goal 坐标越界 → 返回 success=false
     * - start == goal → 返回单点路径
     * - 超时 → 返回当前最优或空路径
     */
    AStarResult plan(
        const std::shared_ptr<OccupancyGrid>& grid,
        const PathPoint& start,
        const PathPoint& goal);

    // ===================================================================
    // IPlannerStrategy接口实现
    // ===================================================================
    bool initialize(const core::PlannerConfig& config) override;
    void setCostmap(const core::Costmap& costmap) override;
    void setInflationRadius(double radius_meters);
    void setRobotRadius(double radius) { robot_radius_ = radius; }
    core::Result<core::Path> plan(const core::Pose2D& start, const core::Pose2D& goal) override;
    void planAsync(const core::Pose2D& start, const core::Pose2D& goal,
                   std::function<void(const core::Result<core::Path>&)> callback) override;
    void cancel() override;
    bool isPlanning() const override;
    std::string getName() const override;
    std::string getVersion() const override;
    void reset() override;

private:
    std::string planner_name_;
    core::Logger logger_;
    core::PlannerConfig config_;
    Config simple_config_;
    std::shared_ptr<core::Costmap> costmap_;
    std::vector<unsigned char> costmap_data_;
    unsigned int nx_, ny_;
    bool planning_active_;

    // A*核心数据
    std::vector<AStarNode> nodes_;                     // 预分配的节点数组 [nx_ * ny_]
    std::priority_queue<AStarNode> open_list_;         // 开放列表（优先队列）
    std::vector<uint8_t> closed_list_;                 // 关闭列表（O(1)数组访问，缓存友好）
    std::vector<int> closed_indices_;                  // 已关闭节点索引（用于快速重置）
    bool use_weighted_heuristic_;                      // 大网格时启用加权启发式
    double inflation_radius_ = 0.5;                    // 障碍物膨胀半径（米）
    double robot_radius_ = 0.21;                        // 机器人半径（米）

    // 性能控制
    int max_iterations_;                               // 最大迭代次数（防止无限循环）
    double planning_timeout_seconds_;                  // 规划超时时间（秒）
    std::chrono::steady_clock::time_point planning_start_time_; // 规划开始时间
    int iterations_;                                   // 当前迭代计数

    // 内部方法
    void inflateCostmap();                               // 障碍物膨胀
    double heuristic(int x1, int y1, int x2, int y2) const;
    bool isValid(int x, int y) const;
    bool isObstacle(int x, int y) const;
    unsigned char getCost(int x, int y) const;
    void reconstructPath(const AStarNode& goal_node, core::Path& path);
    void clearSearchData();
};

}  // namespace planners
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__PLANNERS__ASTAR_PLANNER_HPP_