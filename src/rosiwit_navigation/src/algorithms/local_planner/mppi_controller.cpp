#include "mppi_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rosiwit_navigation {
namespace controllers {

MppiController::MppiController()
: name_("mppi"), logger_("mppi") {}

bool MppiController::initialize(const core::ControllerConfig& config) {
  cfg_ = config;
  max_vx_ = config.max_linear_velocity > 0.0 ? config.max_linear_velocity : 0.5;
  max_vw_ = config.max_angular_velocity > 0.0 ? config.max_angular_velocity : 1.0;
  max_ax_ = config.max_accel_x > 0.0 ? config.max_accel_x : 0.4;
  max_aw_ = config.max_accel_theta > 0.0 ? config.max_accel_theta : 0.8;
  goal_tol_ = config.xy_goal_tolerance > 0.0 ? config.xy_goal_tolerance : 0.2;
  resetNominal();
  initialized_ = true;
  LOG_INFO(logger_, "MPPI initialized: vmax=%.2f wmax=%.2f H=%d K=%d",
    max_vx_, max_vw_, horizon_, samples_);
  return true;
}

void MppiController::resetNominal() {
  // 不要从全 0 控制开始: 采样噪声在 0 附近对称, 加权平均后的第一个
  // 控制量可能只有 0.01, 低于底盘的静止判定, 机器人会一直 recovery.
  const double seed_v = std::max(0.0, max_vx_ * 0.5);
  nominal_.assign(static_cast<size_t>(horizon_), Control{seed_v, 0.0});
}

void MppiController::setPath(const core::Path& path) {
  bool same_goal = false;
  if (!path_.points.empty() && !path.points.empty()) {
    same_goal = path_.points.back().pose.distanceTo(path.points.back().pose) < 0.30;
  }
  path_ = path;
  if (!same_goal) {
    idx_ = 0;
    resetNominal();
  }
  if (idx_ >= path_.points.size()) idx_ = 0;
}

size_t MppiController::nearestPathIndex(const core::Pose2D& pose) const {
  size_t best = 0;
  double best_d2 = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path_.points.size(); ++i) {
    const double dx = path_.points[i].pose.x - pose.x;
    const double dy = path_.points[i].pose.y - pose.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

int MppiController::localCostAt(const core::Pose2D& pose) const {
  if (!has_local_costmap_) return 0;
  const auto& info = local_costmap_.info;
  const int mx = static_cast<int>((pose.x - info.origin.position.x) / info.resolution);
  const int my = static_cast<int>((pose.y - info.origin.position.y) / info.resolution);
  if (mx < 0 || my < 0 ||
      mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height)) {
    return 100;
  }
  return local_costmap_.data[
    static_cast<size_t>(my) * info.width + static_cast<size_t>(mx)];
}

double MppiController::normAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

core::Pose2D MppiController::integrate(
  const core::Pose2D& pose, double v, double w, double dt)
{
  core::Pose2D out = pose;
  out.x += v * std::cos(pose.theta) * dt;
  out.y += v * std::sin(pose.theta) * dt;
  out.theta = normAngle(pose.theta + w * dt);
  return out;
}

core::VelocityCommand MppiController::computeVelocityCommand(
  const core::Pose2D& pose, const core::VelocityCommand& vel)
{
  if (!initialized_ || path_.points.empty()) return {0.0, 0.0, 0.0};

  if (pose.distanceTo(path_.points.back().pose) < goal_tol_) {
    return {0.0, 0.0, 0.0};
  }

  idx_ = std::max(idx_, nearestPathIndex(pose));
  if (nominal_.size() != static_cast<size_t>(horizon_)) resetNominal();

  std::normal_distribution<double> noise_v(0.0, sigma_v_);
  std::normal_distribution<double> noise_w(0.0, sigma_w_);

  struct Sample {
    std::vector<Control> controls;
    double cost{0.0};
  };
  std::vector<Sample> sampled(static_cast<size_t>(samples_));
  double min_cost = std::numeric_limits<double>::max();

  for (int k = 0; k < samples_; ++k) {
    auto& sample = sampled[static_cast<size_t>(k)];
    sample.controls = nominal_;
    core::Pose2D p = pose;
    double prev_v = vel.linear_x;
    double prev_w = vel.angular_z;
    double cost = 0.0;

    for (int t = 0; t < horizon_; ++t) {
      double v = sample.controls[static_cast<size_t>(t)].v + noise_v(rng_);
      double w = sample.controls[static_cast<size_t>(t)].w + noise_w(rng_);
      v = std::clamp(v, std::max(0.0, prev_v - max_ax_ * dt_),
                        std::min(max_vx_, prev_v + max_ax_ * dt_));
      w = std::clamp(w, prev_w - max_aw_ * dt_, prev_w + max_aw_ * dt_);
      w = std::clamp(w, -max_vw_, max_vw_);
      sample.controls[static_cast<size_t>(t)] = {v, w};

      p = integrate(p, v, w, dt_);

      const size_t ni = nearestPathIndex(p);
      const auto& ref = path_.points[ni].pose;
      const double dx = p.x - ref.x;
      const double dy = p.y - ref.y;
      const double lateral2 = dx * dx + dy * dy;
      // A* 未填路径点朝向; 用相邻点切线作为参考航向.
      double ref_yaw = ref.theta;
      if (ni + 1 < path_.points.size()) {
        const auto& next = path_.points[ni + 1].pose;
        ref_yaw = std::atan2(next.y - ref.y, next.x - ref.x);
      } else if (ni > 0) {
        const auto& prev = path_.points[ni - 1].pose;
        ref_yaw = std::atan2(ref.y - prev.y, ref.x - prev.x);
      }
      const double heading_err = normAngle(p.theta - ref_yaw);
      cost += path_weight_ * lateral2 +
              heading_weight_ * heading_err * heading_err;

      const int cell = localCostAt(p);
      if (cell >= 100) {
        cost += 1000.0;
      } else if (cell > 0) {
        cost += obstacle_weight_ * static_cast<double>(cell);
      }
      cost += control_weight_ * (v * v + w * w);
      cost += 0.02 * ((v - prev_v) * (v - prev_v) +
                      (w - prev_w) * (w - prev_w));
      prev_v = v;
      prev_w = w;
    }

    const double goal_err = p.distanceTo(path_.points.back().pose);
    cost += goal_weight_ * goal_err * goal_err;
    sample.cost = cost;
    min_cost = std::min(min_cost, cost);
  }

  double sum_w = 0.0;
  std::vector<Control> weighted(static_cast<size_t>(horizon_), Control{});
  for (const auto& sample : sampled) {
    const double weight = std::exp(-(sample.cost - min_cost) / lambda_);
    sum_w += weight;
    for (int t = 0; t < horizon_; ++t) {
      weighted[static_cast<size_t>(t)].v += weight * sample.controls[static_cast<size_t>(t)].v;
      weighted[static_cast<size_t>(t)].w += weight * sample.controls[static_cast<size_t>(t)].w;
    }
  }
  if (sum_w <= 0.0) return {0.0, 0.0, vel.angular_z};

  for (int t = 0; t < horizon_; ++t) {
    nominal_[static_cast<size_t>(t)] = {
      weighted[static_cast<size_t>(t)].v / sum_w,
      weighted[static_cast<size_t>(t)].w / sum_w};
  }

  const Control first = nominal_.front();
  // 滚动时域: 平移标称控制序列, 最后一个保持.
  for (size_t t = 0; t + 1 < nominal_.size(); ++t) nominal_[t] = nominal_[t + 1];

  return {std::clamp(first.v, 0.0, max_vx_), 0.0,
          std::clamp(first.w, -max_vw_, max_vw_)};
}

bool MppiController::isGoalReached(const core::Pose2D& pose) const {
  if (path_.points.empty()) return false;
  return pose.distanceTo(path_.points.back().pose) < goal_tol_;
}

size_t MppiController::getCurrentWaypointIndex() const { return idx_; }

double MppiController::getProgress() const {
  if (path_.points.empty()) return 1.0;
  return static_cast<double>(idx_) / static_cast<double>(path_.points.size());
}

void MppiController::reset() {
  idx_ = 0;
  path_.points.clear();
  resetNominal();
}

void MppiController::setVelocityLimit(double linear_x, double angular_z) {
  if (linear_x > 0.0) max_vx_ = linear_x;
  if (angular_z > 0.0) max_vw_ = angular_z;
}

std::string MppiController::getName() const { return name_; }
std::string MppiController::getVersion() const { return "1.0.0"; }

void MppiController::setObstacles(const core::ObstacleArray& obstacles) {
  obstacles_ = obstacles;
}

void MppiController::setLocalCostmap(const core::CostmapGrid& costmap) {
  local_costmap_ = costmap;
  has_local_costmap_ = !local_costmap_.data.empty() &&
    local_costmap_.info.width > 0 && local_costmap_.info.height > 0;
}

}}  // namespace rosiwit_navigation::controllers
