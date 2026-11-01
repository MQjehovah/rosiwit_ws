#include "teb_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "rosiwit_navigation/nav_core/banded_solver.hpp"
#include "rosiwit_navigation/nav_core/footprint.hpp"

namespace rosiwit_navigation
{
namespace controllers
{

using core::BlockTridiagonal;
using core::EdgeAccumulator;
using core::EdgeBlocks;
using core::Footprint;
using core::Pose2D;
using core::TebVertex;
using core::VelocityCommand;

namespace
{
inline double normAngle(double a)
{
  while (a > M_PI) { a -= 2.0 * M_PI; }
  while (a < -M_PI) { a += 2.0 * M_PI; }
  return a;
}

inline void addHessTo(std::array<double, 9> & dst, const std::array<double, 9> & src)
{
  for (int i = 0; i < 9; ++i) { dst[i] += src[i]; }
}

/// 把 (i,j) 的交叉块写进 BlockTridiagonal.upper[i] (只存上三角那一块)
inline void addCross(BlockTridiagonal & sys, size_t i, const std::array<double, 9> & h_ab)
{
  for (int k = 0; k < 9; ++k) { sys.upper[i][k] += h_ab[k]; }
}
}  // namespace

TebController::TebController() : name_("teb"), logger_("teb") {}

TebController::~TebController() = default;

bool TebController::initialize(const core::ControllerConfig & config)
{
  cfg_ = config;
  max_vx_ = config.max_linear_velocity > 0.0 ? config.max_linear_velocity : 0.5;
  max_vw_ = config.max_angular_velocity > 0.0 ? config.max_angular_velocity : 1.0;
  max_ax_ = config.max_accel_x > 0.0 ? config.max_accel_x : 0.8;
  goal_tol_ = config.xy_goal_tolerance > 0.0 ? config.xy_goal_tolerance : 0.1;

  weights_.path = config.teb_weight_path;
  weights_.obstacle = config.teb_weight_obstacle;
  weights_.kinematics = config.teb_weight_kinematics;
  weights_.velocity = config.teb_weight_velocity;
  weights_.accel = config.teb_weight_acceleration;
  weights_.time = config.teb_weight_time;
  weights_.shortest = config.teb_weight_shortest;
  weights_.min_obstacle_dist = config.teb_min_obstacle_dist;
  weights_.inflation_dist = config.teb_inflation_dist;
  weights_.max_deviation = config.teb_max_deviation;

  homotopy_classes_ = std::max(1, config.teb_homotopy_classes);
  homotopy_offset_ = config.teb_homotopy_offset;
  max_iterations_ = std::max(1, config.teb_max_iterations);
  max_samples_ = static_cast<size_t>(std::max(5, config.teb_max_samples));
  dt_ref_ = config.teb_dt_ref > 0.0 ? config.teb_dt_ref : 0.3;
  prune_distance_ = 0.3;

  footprint_samples_.clear();
  if (config.footprint.size() >= 3) {
    footprint_samples_ = core::sampleFootprint(config.footprint, 0.05);
  }
  initialized_ = true;
  LOG_INFO(logger_, "TEB initialized: vmax=%.2f wmax=%.2f classes=%d dt_ref=%.2f",
           max_vx_, max_vw_, homotopy_classes_, dt_ref_);
  return true;
}

void TebController::setPath(const core::Path & path)
{
  path_ = path;
  path_idx_ = 0;
  goal_reached_ = false;
  candidates_.clear();
  best_candidate_ = 0;
  // 轻量预热: 后续 computeVelocityCommand 会按"当前位姿 + 偏移"重建
}

void TebController::setLocalCostmap(const core::CostmapGrid & costmap)
{
  obstacle_distance_.update(costmap);
}

void TebController::setObstacles(const core::ObstacleArray & obstacles) { obstacles_ = obstacles; }

void TebController::setVelocityLimit(double vx, double vw)
{
  if (vx > 0.0) { max_vx_ = vx; }
  if (vw > 0.0) { max_vw_ = vw; }
}

TebController::Band TebController::makeBand(const Pose2D & pose, double lateral_offset) const
{
  Band band;
  if (path_.points.size() < 2) {
    band.push_back({pose.x, pose.y, pose.theta, dt_ref_});
    return band;
  }
  const double spacing = std::max(0.10, std::min(0.60, max_vx_ * dt_ref_));
  // 从机器人当前位姿起步 (弹性带必须锚在当前位姿, 否则输出会反向)
  TebVertex start;
  start.x = pose.x;
  start.y = pose.y;
  start.theta = pose.theta;
  start.dt = dt_ref_;
  band.push_back(start);

  double travelled = 0.0;
  for (size_t i = 1; i < path_.points.size() && band.size() + 1 < max_samples_; ++i) {
    const Pose2D & q = path_.points[i].pose;
    const Pose2D & prev = path_.points[i - 1].pose;
    const double seg = std::hypot(q.x - prev.x, q.y - prev.y);
    if (seg < 1e-6) { continue; }
    travelled += seg;
    if (travelled < spacing) { continue; }
    travelled = 0.0;
    // 侧向偏移: 沿路径法向平移 (起点/终点不偏移)
    double nx = 0.0, ny = 0.0;
    if (lateral_offset != 0.0 && i + 1 < path_.points.size()) {
      const double tx = (path_.points[i + 1].pose.x - prev.x);
      const double ty = (path_.points[i + 1].pose.y - prev.y);
      const double n = std::hypot(tx, ty);
      if (n > 1e-6) {
        nx = -ty / n;
        ny = tx / n;
      }
    }
    TebVertex v;
    v.x = q.x + nx * lateral_offset;
    v.y = q.y + ny * lateral_offset;
    v.theta = q.theta;
    v.dt = dt_ref_;
    band.push_back(v);
  }

  // 末端必须落在目标点上 (终点固定)
  const Pose2D & goal = path_.points.back().pose;
  if (band.size() >= 2) {
    TebVertex g;
    g.x = goal.x;
    g.y = goal.y;
    g.theta = goal.theta;
    g.dt = dt_ref_;
    band.back() = g;
  }
  // 初始化 dt: 该段长度 / 允许速度, 并按加速度上限平滑
  for (size_t i = 0; i + 1 < band.size(); ++i) {
    const double ds = std::hypot(band[i + 1].x - band[i].x, band[i + 1].y - band[i].y);
    double dt = std::max(0.05, ds / std::max(0.05, max_vx_));
    if (i > 0) {
      // |Δv| <= max_ax * dt  =>  dt >= |ds_i - ds_{i-1}| / max_ax
      const double ds_prev = std::hypot(band[i].x - band[i - 1].x, band[i].y - band[i - 1].y);
      dt = std::max(dt, std::fabs(ds - ds_prev) / std::max(0.05, max_ax_));
    }
    band[i].dt = std::min(dt, 2.0);
  }
  return band;
}

double TebController::obstacleDistanceAt(const TebVertex & v, double * dir_x, double * dir_y) const
{
  if (!obstacle_distance_.ready()) {
    if (dir_x != nullptr) { *dir_x = 0.0; }
    if (dir_y != nullptr) { *dir_y = 0.0; }
    return -1.0;
  }
  const double d = obstacle_distance_.footprintClearance(
    Pose2D(v.x, v.y, v.theta), footprint_samples_);
  if (dir_x == nullptr && dir_y == nullptr) { return d; }
  // 距离场梯度 -> 指向"距离增大"的方向; 障碍方向取反
  const double eps = std::max(0.02, cfg_.footprint_sample_spacing);
  const double dx = obstacle_distance_.footprintClearance(
                      Pose2D(v.x + eps, v.y, v.theta), footprint_samples_) -
                    obstacle_distance_.footprintClearance(
                      Pose2D(v.x - eps, v.y, v.theta), footprint_samples_);
  const double dy = obstacle_distance_.footprintClearance(
                      Pose2D(v.x, v.y + eps, v.theta), footprint_samples_) -
                    obstacle_distance_.footprintClearance(
                      Pose2D(v.x, v.y - eps, v.theta), footprint_samples_);
  const double n = std::hypot(dx, dy);
  if (n < 1e-6) {
    if (dir_x != nullptr) { *dir_x = 0.0; }
    if (dir_y != nullptr) { *dir_y = 0.0; }
  } else {
    if (dir_x != nullptr) { *dir_x = -dx / n; }
    if (dir_y != nullptr) { *dir_y = -dy / n; }
  }
  return d;
}

void TebController::optimizeBand(Band & band, int iterations) const
{
  const int n = static_cast<int>(band.size());
  if (n < 3) { return; }
  double lambda = 1e-3;
  for (int it = 0; it < iterations; ++it) {
    BlockTridiagonal sys;
    sys.n = n;
    sys.diag.assign(n, {});
    sys.upper.assign(std::max(0, n - 1), {});
    sys.rhs.assign(static_cast<size_t>(3 * n), 0.0);

    for (int i = 0; i < n; ++i) {
      // 路径项
      const EdgeAccumulator ep = core::edgePath(band[i], path_, weights_.path);
      addHessTo(sys.diag[i], ep.hess);
      for (int k = 0; k < 3; ++k) { sys.rhs[3 * i + k] -= ep.grad[k]; }

      // 障碍项 (端点不参与: 起点是当前位姿, 终点是目标)
      if (i > 0 && i + 1 < n) {
        double dx = 0.0, dy = 0.0;
        const double d = obstacleDistanceAt(band[i], &dx, &dy);
        if (d >= 0.0) {
          const EdgeAccumulator eo = core::edgeObstacle(band[i], d, dx, dy, weights_);
          addHessTo(sys.diag[i], eo.hess);
          for (int k = 0; k < 3; ++k) { sys.rhs[3 * i + k] -= eo.grad[k]; }
        }
      }
      // 时间项对位置无梯度 (dt 由闭式更新), 只影响代价统计
    }

    for (int i = 0; i + 1 < n; ++i) {
      const EdgeBlocks ek = core::edgeKinematics(band[i], band[i + 1], weights_.kinematics);
      addHessTo(sys.diag[i], ek.h_aa);
      addHessTo(sys.diag[i + 1], ek.h_aa);   // 对角块用同一份 (GN 对称近似)
      addCross(sys, static_cast<size_t>(i), ek.h_ab);
      for (int k = 0; k < 3; ++k) {
        sys.rhs[3 * i + k] -= ek.g_a[k];
        sys.rhs[3 * (i + 1) + k] -= ek.g_b[k];
      }
    }

    std::vector<double> delta;
    if (!core::solveBlockTridiagonal(sys, lambda, &delta)) {
      lambda *= 10.0;
      if (lambda > 1e6) { break; }
      continue;
    }
    // 限步: 单次 GN 不让顶点跳超过半个采样间距
    double step = 0.0;
    for (int i = 1; i + 1 < n; ++i) {
      step = std::max(step, std::hypot(delta[3 * i], delta[3 * i + 1]));
    }
    const double step_limit = 0.5 * std::max(0.10, max_vx_ * dt_ref_);
    const double scale = (step > step_limit && step > 1e-9) ? step_limit / step : 1.0;
    for (int i = 1; i + 1 < n; ++i) {
      band[i].x += scale * delta[3 * i];
      band[i].y += scale * delta[3 * i + 1];
      band[i].theta = normAngle(band[i].theta + scale * delta[3 * i + 2]);
    }
    // 偏离上限: 超出 max_deviation 的顶点拉回路径
    for (int i = 1; i + 1 < n; ++i) {
      double cx = band[i].x, cy = band[i].y;
      double best_d = 1e18;
      double bx = cx, by = cy;
      for (size_t k = 0; k + 1 < path_.points.size(); ++k) {
        const Pose2D & a = path_.points[k].pose;
        const Pose2D & b = path_.points[k + 1].pose;
        const double vx = b.x - a.x, vy = b.y - a.y;
        const double l2 = vx * vx + vy * vy;
        double t = 0.0;
        if (l2 > 1e-12) {
          t = std::max(0.0, std::min(1.0, ((cx - a.x) * vx + (cy - a.y) * vy) / l2));
        }
        const double px = a.x + t * vx, py = a.y + t * vy;
        const double d = std::hypot(cx - px, cy - py);
        if (d < best_d) { best_d = d; bx = px; by = py; }
      }
      if (best_d > weights_.max_deviation) {
        const double k = (best_d - weights_.max_deviation) / std::max(1e-6, best_d);
        band[i].x += k * (bx - band[i].x);
        band[i].y += k * (by - band[i].y);
      }
    }
  }
  // dt 闭式更新: 段长/允许速度, 受加速度限制
  for (size_t i = 0; i + 1 < band.size(); ++i) {
    const double ds = std::hypot(band[i + 1].x - band[i].x, band[i + 1].y - band[i].y);
    double dt = std::max(0.05, ds / std::max(0.05, max_vx_));
    if (i > 0) {
      const double prev_v = segmentSpeed(band, i - 1);
      const double v = ds / std::max(1e-6, band[i].dt);
      const double need = std::fabs(v - prev_v) / std::max(0.05, max_ax_);
      dt = std::max(dt, need);
    }
    band[i].dt = std::min(std::max(0.05, dt), 2.0);
  }
}

double TebController::segmentSpeed(const Band & band, size_t i) const
{
  if (i + 1 >= band.size()) { return 0.0; }
  const double ds = std::hypot(band[i + 1].x - band[i].x, band[i + 1].y - band[i].y);
  return ds / std::max(1e-6, band[i].dt);
}

double TebController::bandCost(const Band & band) const
{
  double cost = 0.0;
  for (size_t i = 0; i < band.size(); ++i) {
    cost += core::edgePath(band[i], path_, weights_.path).cost;
    if (i > 0 && i + 1 < band.size()) {
      double dx = 0.0, dy = 0.0;
      const double d = obstacleDistanceAt(band[i], &dx, &dy);
      if (d >= 0.0) { cost += core::edgeObstacle(band[i], d, dx, dy, weights_).cost; }
    }
    cost += core::edgeTime(band[i], weights_.time).cost;
  }
  for (size_t i = 0; i + 1 < band.size(); ++i) {
    cost += core::edgeKinematics(band[i], band[i + 1], weights_.kinematics).cost;
    const double over = std::max(0.0, segmentSpeed(band, i) - max_vx_);
    cost += weights_.velocity * over * over;
  }
  return cost;
}

bool TebController::bandFeasible(const Band & band) const
{
  if (!obstacle_distance_.ready()) { return true; }   // 没图时不额外否决
  int judged = 0;
  for (size_t i = 1; i + 1 < band.size(); ++i) {
    const double d = obstacle_distance_.footprintClearance(
      Pose2D(band[i].x, band[i].y, band[i].theta), footprint_samples_);
    if (d < 0.0) { continue; }        // 窗口外: 不参与判定 (带子可以比窗口长)
    ++judged;
    if (d <= 0.0) { return false; }   // 足迹压到 lethal
  }
  // 一个顶点都判不了 (整条带都在窗口外/贴边) 不等于"安全": 保守判为不可行
  return judged > 0;
}

void TebController::rebuildCandidates(const Pose2D & pose)
{
  candidates_.clear();
  const int classes = std::max(1, homotopy_classes_);
  candidates_.push_back(makeBand(pose, 0.0));
  for (int c = 1; c < classes; ++c) {
    const double sign = (c % 2 == 1) ? 1.0 : -1.0;
    const double offset = sign * homotopy_offset_ * static_cast<double>((c + 1) / 2);
    candidates_.push_back(makeBand(pose, offset));
  }
}

size_t TebController::updatePathIndex(const Pose2D & pose)
{
  if (path_.points.empty()) { return 0; }
  size_t best = path_idx_;
  double best_d = std::numeric_limits<double>::max();
  const size_t window_end = std::min(path_.points.size(), path_idx_ + 200);
  for (size_t i = path_idx_; i < window_end; ++i) {
    const double d = std::hypot(path_.points[i].pose.x - pose.x,
                                path_.points[i].pose.y - pose.y);
    if (d < best_d) { best_d = d; best = i; }
  }
  path_idx_ = std::max(path_idx_, best);
  return path_idx_;
}

VelocityCommand TebController::computeVelocityCommand(const Pose2D & pose,
                                                     const VelocityCommand & /*vel*/)
{
  diagnostics_ = "infeasible=0,classes=0,best=0";
  if (!initialized_ || path_.points.size() < 2) { return {0.0, 0.0, 0.0}; }

  const Pose2D & goal = path_.points.back().pose;
  if (std::hypot(goal.x - pose.x, goal.y - pose.y) < goal_tol_) {
    goal_reached_ = true;
    return {0.0, 0.0, 0.0};
  }

  // 与上一位姿差太多 (跳变/换任务) 时重建候选
  if (candidates_.empty() ||
      std::hypot(pose.x - last_pose_.x, pose.y - last_pose_.y) > 0.5) {
    rebuildCandidates(pose);
  }
  last_pose_ = pose;
  updatePathIndex(pose);
  if (candidates_.empty()) { return {0.0, 0.0, 0.0}; }

  // 起点锚定到当前位姿 (弹性带不跟机器人就会输出反向速度)
  for (auto & band : candidates_) {
    // 裁剪: 丢掉离机器人过近的顶点, 保证 band[1] 在**前方**, 否则首段方向会朝后
    size_t drop = 0;
    while (drop + 2 < band.size() &&
           std::hypot(band[drop + 1].x - pose.x, band[drop + 1].y - pose.y) < prune_distance_) {
      ++drop;
    }
    if (drop > 0) { band.erase(band.begin(), band.begin() + static_cast<std::ptrdiff_t>(drop)); }
    band.front().x = pose.x;
    band.front().y = pose.y;
    band.front().theta = pose.theta;
  }

  double best_cost = 1e18;
  int best = -1;
  for (size_t c = 0; c < candidates_.size(); ++c) {
    optimizeBand(candidates_[c], max_iterations_);
    if (!bandFeasible(candidates_[c])) { continue; }
    const double cost = bandCost(candidates_[c]);
    // 迟滞: 上一拍赢家在 15% 以内就继续用它, 避免两族来回跳
    if (static_cast<int>(c) == best_candidate_ && cost <= best_cost * 1.05) { continue; }
    if (cost < best_cost) {
      best_cost = cost;
      best = static_cast<int>(c);
    }
  }
  if (best < 0) {
    // 全不可行: 退回上一拍赢家 (若上一拍赢家仍可行), 否则如实停车并报 infeasible
    if (best_candidate_ >= 0 && static_cast<size_t>(best_candidate_) < candidates_.size() &&
        bandFeasible(candidates_[best_candidate_])) {
      best = best_candidate_;
    } else {
      std::ostringstream os;
      os << "infeasible=1,classes=" << candidates_.size() << ",best=-1";
      diagnostics_ = os.str();
      return {0.0, 0.0, 0.0};
    }
  }
  best_candidate_ = best;
  const Band & band = candidates_[best];

  // 输出带首段速度: 位移方向 * (路径上的方向标记), 角度按 dt
  double sign = 1.0;
  if (!path_.points.empty() && path_idx_ < path_.points.size()) {
    const double flag = path_.points[path_idx_].velocity_x;
    if (flag < 0.0) { sign = -1.0; }
  }
  const double ds = std::hypot(band[1].x - band[0].x, band[1].y - band[0].y);
  const double dt = std::max(0.05, band[0].dt);
  double v_des = sign * std::min(max_vx_, ds / dt);
  // 角速度用前两段加权平均, 避免单段 Δθ/dt 的跳变
  double w_des = normAngle(band[1].theta - band[0].theta) / dt;
  if (band.size() > 2) {
    const double dt2 = std::max(0.05, band[1].dt);
    const double w2 = normAngle(band[2].theta - band[1].theta) / dt2;
    w_des = 0.65 * w_des + 0.35 * w2;
  }
  w_des = std::min(max_vw_, std::max(-max_vw_, w_des));
  // 单拍加速度限幅 (与 rpp 一致): 控制器自己也不该一拍跳 0.3 m/s
  const double dv_max = std::max(0.05, max_ax_) * 0.1;
  const double dw_max = std::max(0.1, max_vw_) * 0.1;
  double v = std::min(last_v_ + dv_max, std::max(last_v_ - dv_max, v_des));
  double w = std::min(last_w_ + dw_max, std::max(last_w_ - dw_max, w_des));
  last_v_ = v;
  last_w_ = w;

  std::ostringstream os;
  os << "infeasible=0,classes=" << candidates_.size() << ",best=" << best
     << ",n=" << band.size() << ",od=" << (obstacle_distance_.ready() ? 1 : 0);
  diagnostics_ = os.str();
  return {v, 0.0, w};
}

bool TebController::isGoalReached(const Pose2D & pose) const
{
  if (path_.points.empty()) { return true; }
  const Pose2D & goal = path_.points.back().pose;
  return std::hypot(goal.x - pose.x, goal.y - pose.y) < goal_tol_;
}

size_t TebController::getCurrentWaypointIndex() const { return path_idx_; }

double TebController::getProgress() const
{
  if (path_.points.size() < 2) { return 1.0; }
  return static_cast<double>(path_idx_) / static_cast<double>(path_.points.size() - 1);
}

void TebController::reset()
{
  candidates_.clear();
  path_idx_ = 0;
  best_candidate_ = 0;
  goal_reached_ = false;
  last_pose_ = Pose2D();
  last_v_ = 0.0;
  last_w_ = 0.0;
}

std::string TebController::getName() const { return name_; }
std::string TebController::getVersion() const { return "2.0.0"; }
std::string TebController::getDiagnostics() const { return diagnostics_; }

}  // namespace controllers
}  // namespace rosiwit_navigation
