#include "hybrid_astar_planner.hpp"
#include "rosiwit_navigation/nav_core/costmap_layer.hpp"
#include "rosiwit_navigation/nav_core/distance_field.hpp"

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
constexpr double kObstacleThreshold = 100.0;  // nav_msgs 语义: >=100 不可通行
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
  heuristic_weight_ = std::max(1.0, config_.heuristic_weight);
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
      for (double k : curvatures) primitives_.push_back({d, k, 0.0});
    }
  }
  if (config_.allow_reverse) {
    const double reverse = -std::min(
      std::fabs(0.35), config_.max_reverse_distance > 0.0
                           ? config_.max_reverse_distance : 0.35);
    for (double k : curvatures) primitives_.push_back({reverse, k, 0.0});
  }
  // 原地旋转基元 (distance=0): 前进-only 模式(清洁导航, 禁止倒车)下靠它
  // 掉头/对正航向, 是"不需要倒车也能走出窄空间"的关键.
  // 步长可配 (默认 30°)。窄处 30° 一步的扫掠面太大, 会被 footprint 检查整段否掉;
  // 15° 时同样的空间能"小步挪", 搜索成功率高得多。
  const double kTurn = std::max(1.0, config_.turn_step_deg) * M_PI / 180.0;
  primitives_.push_back({0.0, 0.0, kTurn});
  primitives_.push_back({0.0, 0.0, -kTurn});
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
  // 全链路统一语义 (nav_msgs/OccupancyGrid): -1 unknown / 0..99 代价 / 100 lethal.
  // 不做换算: 层、Costmap、消息、planner 内部数组是同一套数值.
  const bool use_data = costmap_->data.size() == costmap_data_.size();
  for (size_t i = 0; i < costmap_data_.size(); ++i) {
    if (use_data) {
      costmap_data_[i] = costmap_->data[i];
    } else if (i < costmap_->grid->data.size()) {
      costmap_data_[i] = costmap_->grid->data[i];
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
  coarse_footprint_samples_ = core::sampleFootprint(
    footprint_, std::max(0.20, footprint_sample_spacing_));
  // 外接圆 = 原点到最远顶点. 中心离障碍 >= 它 => **任何朝向**都无碰撞,
  // 可以跳过 footprint 采样 (保守充分, 与内切圆相反: 内切只能用来判"必然碰撞").
  circumscribed_radius_ = 0.0;
  for (const auto & v : footprint_) {
    circumscribed_radius_ = std::max(circumscribed_radius_, std::hypot(v.first, v.second));
  }
  LOG_INFO(logger_, "Robot footprint: %zu vertices, %zu samples (search %zu), circumscribed r=%.3f",
             footprint_.size(), footprint_samples_.size(), coarse_footprint_samples_.size(), circumscribed_radius_);
}

void HybridAStarPlanner::buildInflationField()
{
  const size_t n = static_cast<size_t>(nx_) * ny_;
  inflation_field_.assign(n, 0);
  obstacle_dist_m_.assign(n, 1e3f);
  if (n == 0 || costmap_data_.size() != n) return;

  const int radius_cells = std::max(1, static_cast<int>(
    std::ceil(inflation_radius_ / std::max(1e-6, resolution_))));
  // 精确欧氏距离 (单位=格). 以前是 4 邻域 BFS = 曼哈顿距离, 45° 方向偏大 √2,
  // 导致外接圆快速放行把 0.585m 的判据放宽到 0.41m, 且膨胀场是菱形。
  // unknown(-1) 是"阻断边界", 不是实体障碍; 只有真实占据(100)产生膨胀场.
  std::vector<uint8_t> lethal(n, 0);
  for (size_t i = 0; i < n; ++i) {
    if (costmap_data_[i] >= core::kCostLethal) { lethal[i] = 1; }
  }
  const std::vector<float> dist = core::euclideanDistanceCells(
    lethal, static_cast<unsigned int>(nx_), static_cast<unsigned int>(ny_));
  for (size_t i = 0; i < n; ++i) {
    obstacle_dist_m_[i] = dist[i] * static_cast<float>(resolution_);
  }
  if (inflation_radius_ <= 0.0) return;   // 只建距离场 (外接圆快速接受用)

  for (size_t i = 0; i < n; ++i) {
    if (dist[i] >= radius_cells) continue;
    const double ratio = 1.0 - static_cast<double>(dist[i]) /
                               static_cast<double>(radius_cells);
    inflation_field_[i] = static_cast<int8_t>(
      std::max(0.0, std::min(100.0, ratio * 100.0)));
  }
}

int8_t HybridAStarPlanner::inflationCostAt(int ix, int iy) const
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
  const int8_t c = costAt(ix, iy);
  return c < 0 || c >= kObstacleThreshold;   // -1 unknown: 阻断但不膨胀
}

int8_t HybridAStarPlanner::costAt(int ix, int iy) const
{
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(nx_) || iy >= static_cast<int>(ny_) ||
      costmap_data_.empty()) {
    return -1;   // 界外 = 不可通行
  }
  return costmap_data_[static_cast<size_t>(iy) * nx_ + ix];
}

// footprint 覆盖区域内的最大膨胀代价 (0..100): 让规划代价真正考虑车体轮廓.
int8_t HybridAStarPlanner::footprintInflationCost(const core::Pose2D & pose) const
{
  if (footprint_samples_.empty() || inflation_field_.empty()) {
    int ix = 0, iy = 0;
    if (!worldToGrid(pose.x, pose.y, ix, iy)) return 100;
    return inflationCostAt(ix, iy);
  }
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  int8_t worst = 0;
  for (const auto & lp : footprint_samples_) {
    const double wx = pose.x + c * lp.first - s * lp.second;
    const double wy = pose.y + s * lp.first + c * lp.second;
    const int ix = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    const int iy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
    const int8_t v = inflationCostAt(ix, iy);
    if (v > worst) worst = v;
  }
  return worst;
}

bool HybridAStarPlanner::checkPose(const core::Pose2D & pose) const
{
  if (!costmap_ || !costmap_->grid) return false;
  int ix = 0, iy = 0;
  if (!worldToGrid(pose.x, pose.y, ix, iy)) return false;

  // 快速早退: 先查中心格 (单次查表). 大量候选在这一步就能否掉,
  // 不必为每个候选做 ~60 次 footprint 采样 (Hybrid A* 扩展成本的主因).
  if (isObstacle(ix, iy)) return false;

  // 外接圆快速接受 (保守充分): 中心到最近 lethal >= 外接圆半径 (0.585) 时,
  // 车体**任何朝向**都不可能碰 -> O(1) 放行, 省掉一次 footprint 采样.
  // 开阔区(绝大多数扩展)都命中; 走廊/门口(余量 0.40 < 0.585) 不命中,
  // 继续走精确采样, 所以不会放宽窄道判据.
  if (circumscribed_radius_ > 0.0 && !obstacle_dist_m_.empty()) {
    const size_t didx = static_cast<size_t>(iy) * nx_ + ix;
    if (didx < obstacle_dist_m_.size() &&
        obstacle_dist_m_[didx] >= static_cast<float>(circumscribed_radius_)) {
      return true;
    }
  }

  // Orientation-aware: check the actual polygon when supplied.  The radius
  // circle remains the fallback for callers that only know a robot radius.
  if (!footprint_samples_.empty()) {
    // 搜索期用粗采样: 43 点/pose x 8 基元 是 Hybrid A* 的主要开销
    // (0.8m 门洞实测 4s). 粗采样 0.20m 仍有 ~15 点, 墙体必然被跨到;
    // 最后一段由调用方用细采样复核, 不会放过窄门.
    if (core::footprintCost(*costmap_->grid, pose, coarse_footprint_samples_) < 100)
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

  // 原地旋转: 位置不变, 只转航向. 必须按中间姿态采样检查 —— 车头/车尾是绕着
  // 中心扫过去的, 只查终点会把"旋转过程中扫到墙"判成可行 (门洞/贴墙处 30° 旋转
  // 的中间姿态最危险, 安全层的旋转停障圆 r≈0.71 事后又会把这条路径否掉)。
  if (distance < 1e-9) {
    const double span = std::fabs(motion.dtheta);
    const int steps = std::max(2, static_cast<int>(
      std::ceil(span / (10.0 * M_PI / 180.0))));
    core::Pose2D p = from;
    for (int i = 1; i <= steps; ++i) {
      p.theta = normalizeAngle(
        from.theta + motion.dtheta * (static_cast<double>(i) / steps));
      if (!checkPose(p)) return false;
    }
    to = p;
    return true;
  }
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
    if (std::fabs(m.distance) < 1e-9) {
      // 原地旋转段: 位置不变, 朝向变化; velocity_x=0 供执行器识别
      add_pose(from.x, from.y, normalizeAngle(from.theta + m.dtheta), 0.0);
      continue;
    }
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

      double travel = std::fabs(primitive.distance);
      if (travel < 1e-9) travel = 0.08;   // 原地旋转的基础代价 (避免零代价循环)
      const double reversing_penalty = primitive.distance < 0.0 ? 1.35 : 1.0;
      const double heading_penalty = std::fabs(primitive.curvature) * 0.08 +
                                     std::fabs(primitive.dtheta) * 0.5;
      // 膨胀代价加权: 只查硬障碍会让路径贴墙 (0 余量), 控制器的 footprint +
      // 实时 scan 跟不进去 -> 卡死. 提高膨胀权重让路径在通道里自然居中,
      // 但窄口仍是"更贵"而不是"不可通行".
      // 代价必须按 footprint 算, 不能只看中心格: 否则车体一半悬在代价区也和
      // 居中一样便宜 (用户反馈"旁边有空地却贴着代价区走").
      const double fp_infl = footprintInflationCost(next);
      // 全部 0..100 刻度: costAt/100 与 fp_infl/100 都是 0..1 的归一化代价.
      // 膨胀权重 2.82 = 旧 8.0*(90/255), 保持已验证的贴边行为不变.
      // 膨胀代价封顶 60: 通道里 footprint 长期压在 80~100 的代价上, 不封顶会让
      // 每一小步都乘 3.8, 整片代价面被抹平 -> A* 退化成 Dijkstra (实测 12.8s).
      const double infl_capped = std::min<double>(60.0, static_cast<double>(fp_infl));
      const double cost_factor =
        1.0 + static_cast<double>(std::max<int8_t>(0, costAt(ix, iy))) / 100.0 * 1.5 +
        2.82 * infl_capped / 100.0;
      const double g = current.g + travel * cost_factor * reversing_penalty + heading_penalty;

      auto it = best_g.find(key);
      if (it != best_g.end() && it->second <= g) continue;
      best_g[key] = g;

      Node node;
      node.x = next.x;
      node.y = next.y;
      node.theta = next.theta;
      node.g = g;
      node.f = g + heuristic_weight_ * heuristic(next, goal);
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
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - begin).count();
    const char * why = cancel_requested_ ? "cancelled"
        : (elapsed > timeout ? "timeout"
        : (iterations >= max_iterations ? "max_iterations" : "exhausted"));
    // 这三种原因的处置完全不同: timeout/迭代上限 -> 加预算或换基元;
    // exhausted -> 真的没解 (被障碍/轮廓判死)。以前统一报 "No valid path found"。
    LOG_WARN(logger_, "No goal: iter=%d nodes=%zu open=%zu elapsed=%.2fs why=%s",
             iterations, nodes.size(), open.size(), elapsed, why);
    const std::string msg = std::string("No valid path (") + why + ", iter=" +
        std::to_string(iterations) + ", nodes=" + std::to_string(nodes.size()) + ")";
    return core::Result<core::Path>::error(
      cancel_requested_ ? core::ErrorCode::PLANNING_FAILED : core::ErrorCode::NO_VALID_PATH,
      cancel_requested_ ? "Planning cancelled" : msg);
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
