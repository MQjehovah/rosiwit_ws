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
  const double forward[] = {0.35, 0.60};
  const double curvatures[] = {-1.2, 0.0, 1.2};
  if (!config_.reverse_only) {
    for (double d : forward) {
      for (double k : curvatures) primitives_.push_back({d, k});
    }
  }
  if (config_.allow_reverse) {
    const double reverse = -std::min(
      std::fabs(0.35), config_.max_reverse_distance > 0.0
                           ? config_.max_reverse_distance : 0.35);
    for (double k : curvatures) primitives_.push_back({reverse, k});
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
  buildInflationField();
  LOG_INFO(logger_, "Costmap set: %ux%u res=%.3f step=%.3f",
           nx_, ny_, resolution_, step_length_);
}

void HybridAStarPlanner::setInflationRadius(double radius_meters)
{
  inflation_radius_ = std::max(0.0, radius_meters);
  if (costmap_ && costmap_data_.size() == static_cast<size_t>(nx_) * ny_)
    buildInflationField();
}

void HybridAStarPlanner::setRobotFootprint(const core::Footprint & footprint)
{
  footprint_ = footprint;
  footprint_samples_ = core::sampleFootprint(footprint_, footprint_sample_spacing_);
  LOG_INFO(logger_, "Robot footprint: %zu vertices, %zu samples",
           footprint_.size(), footprint_samples_.size());
}

void HybridAStarPlanner::buildInflationField()
{
  const size_t n = static_cast<size_t>(nx_) * ny_;
  inflation_field_.assign(n, 0);
  if (inflation_radius_ <= 0.0 || n == 0 || costmap_data_.size() != n) return;

  const int radius_cells = std::max(1, static_cast<int>(
    std::ceil(inflation_radius_ / std::max(1e-6, resolution_))));
  std::vector<float> dist(n, std::numeric_limits<float>::max());
  std::queue<std::pair<int, int>> q;
  for (int y = 0; y < static_cast<int>(ny_); ++y) {
    for (int x = 0; x < static_cast<int>(nx_); ++x) {
      const size_t idx = static_cast<size_t>(y) * nx_ + x;
      // Unknown (254) is a blocked boundary, not a physical obstacle;
      // only real occupied cells define the soft clearance field.
      if (costmap_data_[idx] >= 255) {
        dist[idx] = 0.0f;
        q.push({x, y});
      }
    }
  }
  const int dx4[4] = {-1, 0, 1, 0};
  const int dy4[4] = {0, -1, 0, 1};
  while (!q.empty()) {
    const auto c = q.front();
    q.pop();
    const float cd = dist[static_cast<size_t>(c.second) * nx_ + c.first];
    if (cd >= radius_cells) continue;
    for (int i = 0; i < 4; ++i) {
      const int x = c.first + dx4[i];
      const int y = c.second + dy4[i];
      if (x < 0 || y < 0 || x >= static_cast<int>(nx_) ||
          y >= static_cast<int>(ny_)) continue;
      const size_t idx = static_cast<size_t>(y) * nx_ + x;
      if (dist[idx] > cd + 1.0f) {
        dist[idx] = cd + 1.0f;
        q.push({x, y});
      }
    }
  }
  for (size_t i = 0; i < n; ++i) {
    if (dist[i] >= radius_cells) continue;
    const double ratio = 1.0 - static_cast<double>(dist[i]) /
                               static_cast<double>(radius_cells);
    inflation_field_[i] = static_cast<unsigned char>(
      std::max(0.0, std::min(255.0, ratio * 90.0)));
  }
}

unsigned char HybridAStarPlanner::inflationCostAt(int ix, int iy) const
{
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(nx_) ||
      iy >= static_cast<int>(ny_) ||
      inflation_field_.size() != static_cast<size_t>(nx_) * ny_)
    return 0;
  return inflation_field_[static_cast<size_t>(iy) * nx_ + ix];
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
  if (!costmap_ || !costmap_->grid) return false;
  int ix = 0, iy = 0;
  if (!worldToGrid(pose.x, pose.y, ix, iy)) return false;

  // 快速早退: 先查中心格 (单次查表). 大量候选在这一步就能否掉,
  // 不必为每个候选做 ~60 次 footprint 采样 (Hybrid A* 扩展成本的主因).
  if (isObstacle(ix, iy)) return false;

  // Orientation-aware: check the actual polygon when supplied.  The radius
  // circle remains the fallback for callers that only know a robot radius.
  if (!footprint_samples_.empty()) {
    if (core::footprintCost(*costmap_->grid, pose, footprint_samples_) < 100)
      return true;
    // Recovery-only start clearance: the robot is already standing inside the
    // occupied cells (stale map / charger / self-projection).  Ignore both the
    // footprint and the centre cell inside start_relax_radius so a reverse path
    // can leave; outside that radius the normal costmap applies.  Normal
    // planning sets start_relax_radius=0 and never takes this branch.
    if (start_relaxed_ && config_.start_relax_radius > 0.0 &&
        std::hypot(pose.x - start_pose_.x, pose.y - start_pose_.y) <=
          config_.start_relax_radius) {
      return true;
    }
    return false;
  }

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
  }
  // 碰撞检查只取弧中点和端点: footprint 检查是 Hybrid A* 的主要开销
  // (~50 采样点/次). 0.35~0.6m 的小弧用两点采样足够, 角点擦碰由控制器
  // 的 footprint 代价和 safety 层兜底.
  {
    const double half_th = from.theta;
    const double mid_dtheta = motion.curvature * (distance * 0.5) * sign;
    core::Pose2D mid;
    mid.x = from.x + std::cos(half_th + 0.5 * mid_dtheta) * (distance * 0.5) * sign;
    mid.y = from.y + std::sin(half_th + 0.5 * mid_dtheta) * (distance * 0.5) * sign;
    mid.theta = normalizeAngle(half_th + mid_dtheta);
    if (!checkPose(mid)) return false;
  }
  to = {x, y, th};
  return checkPose(to);
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
  // Reconstruct the actual arcs, not just the node centers.  A straight line
  // between two arc endpoints can cut a footprint corner even though both
  // endpoints are collision-free.
  std::vector<int> chain;
  for (int index = goal_index; index >= 0; index = nodes[index].parent)
    chain.push_back(index);
  std::reverse(chain.begin(), chain.end());

  core::Path path;
  if (chain.empty()) return path;
  auto add_pose = [&path](double x, double y, double theta, double direction) {
    core::PathPoint pt;
    pt.pose = core::Pose2D(x, y, theta);
    pt.velocity_x = direction;
    path.points.push_back(pt);
  };
  add_pose(nodes[chain.front()].x, nodes[chain.front()].y,
           nodes[chain.front()].theta, 0.0);

  for (size_t k = 1; k < chain.size(); ++k) {
    const Node & from = nodes[chain[k - 1]];
    const Node & to = nodes[chain[k]];
    const MotionPrimitive & m = to.motion;
    const double sign = m.distance >= 0.0 ? 1.0 : -1.0;
    const double distance = std::fabs(m.distance);
    const int samples = std::max(2, static_cast<int>(
      std::ceil(distance / std::max(1e-6, resolution_ * 0.8))));
    const double ds = distance / static_cast<double>(samples);
    double x = from.x;
    double y = from.y;
    double th = from.theta;
    for (int i = 0; i < samples; ++i) {
      const double dtheta = m.curvature * ds * sign;
      const double mid_th = th + 0.5 * dtheta;
      x += std::cos(mid_th) * ds * sign;
      y += std::sin(mid_th) * ds * sign;
      th = normalizeAngle(th + dtheta);
      add_pose(x, y, th, sign);
    }
  }
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
  if (isObstacle(sx, sy)) {
    planning_active_ = false;
    return core::Result<core::Path>::error(
      core::ErrorCode::START_IN_OBSTACLE, "Start in obstacle");
  }
  start_pose_ = start;
  // Recovery planner (start_relax_radius > 0) keeps footprint relaxation active
  // even when the start pose is free after the caller cleared the cells under
  // the current footprint.  As soon as the robot moves, its footprint sweeps
  // cells that were not under the original footprint; without this the reverse
  // escape would still be rejected a few centimetres from the start.
  // Normal planner sets start_relax_radius=0, so its behaviour is unchanged.
  start_relaxed_ = config_.start_relax_radius > 0.0;
  if (!checkPose(start) && start_relaxed_) {
    LOG_WARN(logger_, "Start footprint overlaps costmap; recovery start clearance=%.2fm",
             config_.start_relax_radius);
  }
  if (!checkPose(start)) {
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
      // 膨胀代价加权: 只查硬障碍会让路径贴墙 (0 余量), 控制器的 footprint +
      // 实时 scan 跟不进去 -> 卡死. 提高膨胀权重让路径在通道里自然居中,
      // 但窄口仍是"更贵"而不是"不可通行".
      const double cost_factor =
        1.0 + static_cast<double>(costAt(ix, iy)) / 255.0 * 1.5 +
        8.0 * static_cast<double>(inflationCostAt(ix, iy)) / 255.0;
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
      node.motion = primitive;
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
  {
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count();
    LOG_INFO(logger_, "Hybrid A* plan time: %.1f ms", ms);
  }
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
