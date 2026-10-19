#include "hybrid_astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <thread>

namespace rosiwit_navigation
{
namespace planners
{
namespace
{
constexpr double kObstacleThreshold = 250.0;  // 254 未知 / 255 致命
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double a)
{
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}
}  // namespace

HybridAStarPlanner::HybridAStarPlanner()
  : logger_("hybrid_astar_planner")
{
  buildPrimitives();
}

bool HybridAStarPlanner::initialize(const core::PlannerConfig & config)
{
  config_ = config;
  if (!config_.name.empty()) planner_name_ = config_.name;
  if (config_.resolution > 0.0) resolution_ = config_.resolution;
  buildPrimitives();
  LOG_INFO(logger_, "Hybrid A* initialized (res=%.3f, step=%.3f, bins=%d)",
           resolution_, step_length_, heading_bins_);
  return true;
}

void HybridAStarPlanner::buildPrimitives()
{
  primitives_.clear();
  // 前进/后退各 3 种曲率，覆盖直线、左转、右转。
  const double distances[] = {0.35, 0.60, -0.35};
  const double curvatures[] = {-1.2, 0.0, 1.2};
  for (double d : distances) {
    for (double k : curvatures) {
      primitives_.push_back({d, k});
    }
  }
}

void HybridAStarPlanner::setCostmap(const core::Costmap & costmap)
{
  costmap_ = std::make_shared<core::Costmap>(costmap);
  if (!costmap_->grid) {
    LOG_ERROR(logger_, "Invalid costmap");
    return;
  }
  nx_ = costmap_->grid->info.width;
  ny_ = costmap_->grid->info.height;
  resolution_ = costmap_->grid->info.resolution;
  origin_x_ = costmap_->grid->info.origin.position.x;
  origin_y_ = costmap_->grid->info.origin.position.y;

  costmap_data_.assign(static_cast<size_t>(nx_) * ny_, 0);
  if (costmap_->data.size() == costmap_data_.size()) {
    for (size_t i = 0; i < costmap_data_.size(); ++i) {
      costmap_data_[i] = static_cast<unsigned char>(costmap_->data[i]);
    }
  } else {
    for (size_t i = 0; i < costmap_->grid->data.size() && i < costmap_data_.size(); ++i) {
      const int8_t v = costmap_->grid->data[i];
      if (v < 0) costmap_data_[i] = 254;
      else if (v >= 100) costmap_data_[i] = 255;
      else costmap_data_[i] = static_cast<unsigned char>(v * 255 / 100);
    }
  }

  step_length_ = std::max(0.25, 8.0 * resolution_);
  LOG_INFO(logger_, "Costmap set: %ux%u res=%.3f step=%.3f",
           nx_, ny_, resolution_, step_length_);
}

bool HybridAStarPlanner::worldToGrid(double x, double y, int & ix, int & iy) const
{
  ix = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  iy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return ix >= 0 && iy >= 0 &&
         ix < static_cast<int>(nx_) && iy < static_cast<int>(ny_);
}

int HybridAStarPlanner::headingIndex(double theta) const
{
  int idx = static_cast<int>(std::floor((normalizeAngle(theta) + kPi) /
                                        (2.0 * kPi) * heading_bins_));
  if (idx < 0) idx = 0;
  if (idx >= heading_bins_) idx = heading_bins_ - 1;
  return idx;
}

int HybridAStarPlanner::stateKey(int ix, int iy, int itheta) const
{
  return (iy * static_cast<int>(nx_) + ix) * heading_bins_ + itheta;
}

bool HybridAStarPlanner::isObstacle(int ix, int iy) const
{
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(nx_) || iy >= static_cast<int>(ny_)) {
    return true;
  }
  return costAt(ix, iy) >= kObstacleThreshold;
}

unsigned char HybridAStarPlanner::costAt(int ix, int iy) const
{
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(nx_) || iy >= static_cast<int>(ny_) ||
      costmap_data_.empty()) {
    return 254;
  }
  return costmap_data_[static_cast<size_t>(iy) * nx_ + ix];
}

bool HybridAStarPlanner::checkPose(const core::Pose2D & pose) const
{
  int ix = 0, iy = 0;
  if (!worldToGrid(pose.x, pose.y, ix, iy)) return false;

  // 用机器人外接半径检查一圈栅格，而不是只看中心点。
  const int r = std::max(1, static_cast<int>(std::ceil(robot_radius_ / resolution_)));
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (dx * dx + dy * dy > r * r) continue;
      if (isObstacle(ix + dx, iy + dy)) return false;
    }
  }
  return true;
}

bool HybridAStarPlanner::checkArc(
  const core::Pose2D & from, const MotionPrimitive & motion, core::Pose2D & to) const
{
  const double sign = motion.distance >= 0.0 ? 1.0 : -1.0;
  const double distance = std::fabs(motion.distance);
  const int samples = std::max(2, static_cast<int>(std::ceil(distance / (resolution_ * 0.8))));
  const double ds = distance / samples;

  double x = from.x;
  double y = from.y;
  double th = from.theta;
  for (int i = 0; i < samples; ++i) {
    const double dtheta = motion.curvature * ds * sign;
    // 小步圆弧积分，比一次弦长更贴近真实车辆轨迹。
    const double mid_th = th + 0.5 * dtheta;
    x += std::cos(mid_th) * ds * sign;
    y += std::sin(mid_th) * ds * sign;
    th = normalizeAngle(th + dtheta);
    core::Pose2D p{x, y, th};
    if (!checkPose(p)) return false;
  }
  to = {x, y, th};
  return true;
}

double HybridAStarPlanner::heuristic(
  const core::Pose2D & a, const core::Pose2D & b) const
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy) + 0.25 * std::fabs(normalizeAngle(a.theta - b.theta));
}

core::Path HybridAStarPlanner::reconstruct(
  const std::vector<Node> & nodes, int goal_index) const
{
  core::Path path;
  int index = goal_index;
  while (index >= 0) {
    const Node & n = nodes[index];
    core::PathPoint pt;
    pt.pose = core::Pose2D(n.x, n.y, n.theta);
    path.points.push_back(pt);
    index = n.parent;
  }
  std::reverse(path.points.begin(), path.points.end());
  return path;
}

core::Result<core::Path> HybridAStarPlanner::plan(
  const core::Pose2D & start, const core::Pose2D & goal)
{
  if (!costmap_ || !costmap_->grid || costmap_data_.empty()) {
    return core::Result<core::Path>::error(
      core::ErrorCode::NOT_INITIALIZED, "Costmap not set");
  }

  planning_active_ = true;
  cancel_requested_ = false;
  const auto begin = std::chrono::steady_clock::now();
  const int max_iterations = config_.max_iterations > 0 ? config_.max_iterations : 200000;
  const double timeout = config_.timeout > 0.0 ? config_.timeout : 2.0;

  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!worldToGrid(start.x, start.y, sx, sy)) {
    planning_active_ = false;
    return core::Result<core::Path>::error(
      core::ErrorCode::START_IN_OBSTACLE, "Start outside costmap");
  }
  if (!worldToGrid(goal.x, goal.y, gx, gy)) {
    planning_active_ = false;
    return core::Result<core::Path>::error(
      core::ErrorCode::GOAL_IN_OBSTACLE, "Goal outside costmap");
  }
  if (isObstacle(sx, sy) || !checkPose(start)) {
    planning_active_ = false;
    return core::Result<core::Path>::error(
      core::ErrorCode::START_IN_OBSTACLE, "Start in obstacle");
  }
  if (isObstacle(gx, gy) || !checkPose(goal)) {
    planning_active_ = false;
    return core::Result<core::Path>::error(
      core::ErrorCode::GOAL_IN_OBSTACLE, "Goal in obstacle");
  }

  std::vector<Node> nodes;
  nodes.reserve(4096);
  std::unordered_map<int, double> best_g;
  std::priority_queue<QueueItem> open;

  Node start_node;
  start_node.x = start.x;
  start_node.y = start.y;
  start_node.theta = start.theta;
  start_node.g = 0.0;
  start_node.f = heuristic(start, goal);
  start_node.ix = sx;
  start_node.iy = sy;
  start_node.itheta = headingIndex(start.theta);
  nodes.push_back(start_node);
  best_g[stateKey(sx, sy, start_node.itheta)] = 0.0;
  open.push({start_node.f, 0});

  int goal_index = -1;
  int iterations = 0;
  while (!open.empty() && iterations++ < max_iterations) {
    if (cancel_requested_) break;
    if ((iterations & 0x3f) == 0) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin).count();
      if (elapsed > timeout) break;
    }

    const QueueItem item = open.top();
    open.pop();
    const Node current = nodes[item.index];
    if (current.f > item.f + 1e-9) continue;

    const double dx = current.x - goal.x;
    const double dy = current.y - goal.y;
    const double dtheta = std::fabs(normalizeAngle(current.theta - goal.theta));
    if (std::hypot(dx, dy) <= std::max(0.18, 3.0 * resolution_) &&
        dtheta <= 0.45) {
      goal_index = item.index;
      break;
    }

    for (const auto & primitive : primitives_) {
      core::Pose2D next;
      if (!checkArc({current.x, current.y, current.theta}, primitive, next)) continue;
      const int ix = static_cast<int>(std::floor((next.x - origin_x_) / resolution_));
      const int iy = static_cast<int>(std::floor((next.y - origin_y_) / resolution_));
      const int ith = headingIndex(next.theta);
      const int key = stateKey(ix, iy, ith);

      const double travel = std::fabs(primitive.distance);
      const double reversing_penalty = primitive.distance < 0.0 ? 1.35 : 1.0;
      const double heading_penalty = std::fabs(primitive.curvature) * 0.08;
      const double cost_factor = 1.0 + static_cast<double>(costAt(ix, iy)) / 255.0 * 1.5;
      const double g = current.g + travel * cost_factor * reversing_penalty + heading_penalty;

      auto it = best_g.find(key);
      if (it != best_g.end() && it->second <= g) continue;
      best_g[key] = g;

      Node node;
      node.x = next.x;
      node.y = next.y;
      node.theta = next.theta;
      node.g = g;
      node.f = g + heuristic(next, goal);
      node.parent = item.index;
      node.ix = ix;
      node.iy = iy;
      node.itheta = ith;
      nodes.push_back(node);
      open.push({node.f, static_cast<int>(nodes.size()) - 1});
    }

    if (iterations % 2000 == 0) {
    }
  }

  planning_active_ = false;
  if (goal_index < 0) {
    LOG_INFO(logger_, "No goal: iter=%d nodes=%zu open=%zu", iterations, nodes.size(), open.size());
    return core::Result<core::Path>::error(
      cancel_requested_ ? core::ErrorCode::PLANNING_FAILED : core::ErrorCode::NO_VALID_PATH,
      cancel_requested_ ? "Planning cancelled" : "No valid path found");
  }

  core::Path path = reconstruct(nodes, goal_index);
  core::PathPoint last;
  last.pose = goal;
  path.points.push_back(last);
  if (path.points.size() < 2) path.points.push_back(last);
  LOG_INFO(logger_, "Hybrid A* path: %zu points, %d iterations", path.points.size(), iterations);
  return core::Result<core::Path>::ok(path);
}

void HybridAStarPlanner::planAsync(
  const core::Pose2D & start, const core::Pose2D & goal,
  std::function<void(const core::Result<core::Path> &)> callback)
{
  std::thread([this, start, goal, callback]() {
    callback(plan(start, goal));
  }).detach();
}

void HybridAStarPlanner::cancel()
{
  cancel_requested_ = true;
  planning_active_ = false;
}

void HybridAStarPlanner::reset()
{
  cancel_requested_ = false;
  planning_active_ = false;
}

}  // namespace planners
}  // namespace rosiwit_navigation
