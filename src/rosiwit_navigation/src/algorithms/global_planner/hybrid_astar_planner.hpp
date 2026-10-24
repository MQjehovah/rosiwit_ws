// ============================================================
// Hybrid A* —— 满足非完整约束的 SE(2) 全局规划器
//
// 与 A* 的栅格图搜索不同，这里在每个 (x, y, theta) 离散节点上
// 扩展前进/后退 + 转向的运动基元，因此输出路径本身满足车辆转弯半径，
// 不会出现 A* 那种"折线贴墙"、控制器无法跟踪的问题。
// ============================================================
#ifndef ROSIWIT_NAVIGATION__PLANNERS__HYBRID_ASTAR_PLANNER_HPP_
#define ROSIWIT_NAVIGATION__PLANNERS__HYBRID_ASTAR_PLANNER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosiwit_navigation/nav_core/i_planner.hpp"
#include "rosiwit_navigation/nav_core/logger.hpp"
#include "rosiwit_navigation/nav_core/footprint.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace planners
{

class HybridAStarPlanner : public core::IPlannerStrategy
{
public:
  HybridAStarPlanner();
  ~HybridAStarPlanner() override = default;

  bool initialize(const core::PlannerConfig & config) override;
  void setCostmap(const core::Costmap & costmap) override;
  void setInflationRadius(double radius_meters) override;
  void setRobotRadius(double radius) override { robot_radius_ = radius; }
  void setRobotFootprint(const core::Footprint & footprint) override;
  bool supportsFootprint() const override { return true; }
  core::Result<core::Path> plan(
    const core::Pose2D & start, const core::Pose2D & goal) override;
  void planAsync(
    const core::Pose2D & start, const core::Pose2D & goal,
    std::function<void(const core::Result<core::Path> &)> callback) override;
  void cancel() override;
  bool isPlanning() const override { return planning_active_; }
  std::string getName() const override { return "hybrid_astar"; }
  std::string getVersion() const override { return "1.0"; }
  void reset() override;

private:
  struct MotionPrimitive
  {
    double distance = 0.0;   // 有符号，负值表示倒车
    double curvature = 0.0;  // 1/R，0 表示直线
    double dtheta = 0.0;     // 原地旋转量(rad); distance==0 时生效
  };
  struct Node
  {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double g = 0.0;
    double f = 0.0;
    int parent = -1;
    int ix = 0;
    int iy = 0;
    int itheta = 0;
    MotionPrimitive motion;   // edge from parent to this node
  };

  struct QueueItem
  {
    double f;
    int index;
    bool operator<(const QueueItem & other) const { return f > other.f; }
  };

  bool worldToGrid(double x, double y, int & ix, int & iy) const;
  int headingIndex(double theta) const;
  int stateKey(int ix, int iy, int itheta) const;
  bool isObstacle(int ix, int iy) const;
  unsigned char costAt(int ix, int iy) const;
  unsigned char inflationCostAt(int ix, int iy) const;
  void buildInflationField();
  bool checkPose(const core::Pose2D & pose) const;
  bool checkArc(const core::Pose2D & from, const MotionPrimitive & motion,
                core::Pose2D & to) const;
  double heuristic(const core::Pose2D & a, const core::Pose2D & b) const;
  core::Path reconstruct(const std::vector<Node> & nodes, int goal_index) const;
  void buildPrimitives();

  std::string planner_name_ = "hybrid_astar";
  core::Logger logger_;
  core::PlannerConfig config_;
  std::shared_ptr<core::Costmap> costmap_;
  std::vector<unsigned char> costmap_data_;
  unsigned int nx_ = 0;
  unsigned int ny_ = 0;
  double resolution_ = 0.05;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double robot_radius_ = 0.21;
  double inflation_radius_ = 0.5;
  double step_length_ = 0.35;
  double max_curvature_ = 1.5;
  int heading_bins_ = 72;
  core::Footprint footprint_;
  std::vector<std::pair<double, double>> footprint_samples_;
  // 0.05 对 0.9m 车体 = ~60 采样点; Hybrid A* 每次扩展都要查, 改成 0.08 减 40%.
  double footprint_sample_spacing_ = 0.08;
  std::vector<unsigned char> inflation_field_;
  // recovery-only: tolerate the current footprint overlapping the map so a
  // reverse escape path can be generated; normal planning keeps this at 0.
  core::Pose2D start_pose_;
  bool start_relaxed_ = false;
  std::vector<MotionPrimitive> primitives_;
  bool planning_active_ = false;
  bool cancel_requested_ = false;
};

}  // namespace planners
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__PLANNERS__HYBRID_ASTAR_PLANNER_HPP_
