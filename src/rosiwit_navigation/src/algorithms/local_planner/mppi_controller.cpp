#include "mppi_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>
#include <iostream>
#include <string>

namespace rosiwit_navigation {
namespace controllers {

namespace {
core::Path resamplePath(const core::Path& in, double max_step) {
  if (in.points.size() < 2 || max_step <= 1e-4) return in;
  const double step = std::max(0.02, max_step);
  core::Path out;
  out.points.reserve(in.points.size() * 4);
  out.points.push_back(in.points.front());
  for (size_t i = 1; i < in.points.size(); ++i) {
    const auto& a = in.points[i - 1].pose;
    const auto& b = in.points[i].pose;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double dist = std::hypot(dx, dy);
    const int n = std::max(1, static_cast<int>(std::ceil(dist / step)));
    const double tangent = std::atan2(dy, dx);
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      core::PathPoint p;
      p.pose.x = a.x + t * dx;
      p.pose.y = a.y + t * dy;
      // Keep the requested final heading.  For forward paths the tangent is
      // the driving direction; for a reverse segment the robot still faces
      // a.theta/b.theta, so interpolate the supplied heading instead of
      // turning the reference 180 degrees backwards.
      if (i + 1 == in.points.size() && k == n) {
        p.pose.theta = b.theta;
      } else if (in.points[i].velocity_x < -0.1) {
        double da = b.theta - a.theta;
        while (da > M_PI) da -= 2.0 * M_PI;
        while (da < -M_PI) da += 2.0 * M_PI;
        p.pose.theta = a.theta + t * da;
      } else {
        p.pose.theta = tangent;
      }
      p.velocity_x = in.points[i].velocity_x;
      p.velocity_theta = 0.0;
      out.points.push_back(p);
    }
  }
  return out;
}
}  // namespace

MppiController::MppiController()
: name_("mppi"), logger_("mppi") {}

bool MppiController::initialize(const core::ControllerConfig& config) {
  cfg_ = config;
  max_vx_ = config.max_linear_velocity > 0.0 ? config.max_linear_velocity : 0.5;
  max_vw_ = config.max_angular_velocity > 0.0 ? config.max_angular_velocity : 1.0;
  max_ax_ = config.max_accel_x > 0.0 ? config.max_accel_x : 0.4;
  max_aw_ = config.max_accel_theta > 0.0 ? config.max_accel_theta : 0.8;
  goal_tol_ = config.xy_goal_tolerance > 0.0 ? config.xy_goal_tolerance : 0.2;
  allow_reverse_ = config.allow_reverse;
  footprint_ = config.footprint;
  footprint_samples_ = core::sampleFootprint(footprint_, config.footprint_sample_spacing);
  horizon_ = std::max(1, config.mppi_horizon);
  samples_ = std::max(1, config.mppi_samples);
  dt_ = config.mppi_dt > 1e-4 ? config.mppi_dt : 0.1;
  lambda_ = std::max(1e-4, config.mppi_lambda);
  sigma_v_ = std::max(1e-4, config.mppi_sigma_v);
  sigma_w_ = std::max(1e-4, config.mppi_sigma_w);
  path_weight_ = std::max(0.0, config.mppi_path_weight);
  heading_weight_ = std::max(0.0, config.mppi_heading_weight);
  obstacle_weight_ = std::max(0.0, config.mppi_obstacle_weight);
  control_weight_ = std::max(0.0, config.mppi_control_weight);
  speed_weight_ = std::max(0.0, config.mppi_speed_weight);
  goal_weight_ = std::max(0.0, config.mppi_goal_weight);
  resetNominal();
  initialized_ = true;
  LOG_INFO(logger_, "MPPI initialized: vmax=%.2f wmax=%.2f H=%d K=%d footprint=%zu samples=%zu",
    max_vx_, max_vw_, horizon_, samples_, footprint_.size(), footprint_samples_.size());
  return true;
}

void MppiController::resetNominal() {
  // 不要从全 0 控制开始: 采样噪声在 0 附近对称, 加权平均后的第一个
  // 控制量可能只有 0.01, 低于底盘的静止判定, 机器人会一直 recovery.
  const double seed_v = std::max(0.0, max_vx_ * 0.8);
  const double seed = reverse_mode_ ? -seed_v : seed_v;
  nominal_.assign(static_cast<size_t>(horizon_), Control{seed, 0.0});
}

void MppiController::setPath(const core::Path& path) {
  const core::Path path_in = resamplePath(path, cfg_.path_resample_step);
  bool same_goal = false;
  const size_t old_size = path_.points.size();
  const size_t old_idx = idx_;
  if (!path_.points.empty() && !path_in.points.empty()) {
    same_goal = path_.points.back().pose.distanceTo(path_in.points.back().pose) < 0.30;
  }
  path_ = path_in;
  const bool was_reverse = reverse_mode_;
  reverse_mode_ = false;
  if (allow_reverse_) {
    // Use the first directed segment.  A mixed forward/reverse path is
    // supported by computeVelocityCommand(), which re-evaluates direction at
    // the current path index.
    for (const auto& p : path_.points) {
      if (std::fabs(p.velocity_x) > 0.1) {
        reverse_mode_ = p.velocity_x < 0.0;
        break;
      }
    }
  }
  // 重规划后的路径从当前位姿开始. 同一目标下保留进度比例, 不能每秒
  // 重规划都把 idx_ 拉回 0, 否则 MPPI 会在同一段路径上反复起步/抖动.
  if (!path_.points.empty()) {
    if (same_goal && old_size > 1) {
      const double progress = static_cast<double>(old_idx) /
                              static_cast<double>(old_size);
      idx_ = std::min(path_.points.size() - 1,
                      static_cast<size_t>(progress * path_.points.size()));
    } else {
      idx_ = 0;
    }
  } else {
    idx_ = 0;
  }
  if (!same_goal || was_reverse != reverse_mode_) {
    // 新目标/前进倒车方向切换都必须从静止加速度约束重新起步, 不能沿用
    // 上一方向的速度命令.
    last_output_v_ = 0.0;
    last_output_w_ = 0.0;
    resetNominal();
  }
  if (idx_ >= path_.points.size()) idx_ = 0;
}

size_t MppiController::nearestPathIndex(const core::Pose2D& pose) const {
  return nearestPathIndex(pose, 0, path_.points.size());
}

size_t MppiController::nearestPathIndex(
  const core::Pose2D& pose, size_t begin, size_t end) const {
  if (path_.points.empty()) return 0;
  begin = std::min(begin, path_.points.size() - 1);
  end = std::max(begin + 1, std::min(end, path_.points.size()));
  size_t best = begin;
  double best_d2 = std::numeric_limits<double>::max();
  for (size_t i = begin; i < end; ++i) {
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

  // 只从当前进度附近向后搜索, 防止长路径上每个采样点都全表扫描, 也防止
  // 最近点跳到 U 型路径的另一段导致参考航向突变.
  const size_t progress_begin = idx_ > 2 ? idx_ - 2 : 0;
  idx_ = std::max(idx_, nearestPathIndex(pose, progress_begin, path_.points.size()));

  // A Hybrid A* path may contain both forward and reverse segments.  Select
  // the direction of the upcoming segment rather than treating the whole path
  // as reverse just because one later segment is reverse.
  // 近端 2 点 + 迟滞: 以前看后 6 点(0.08m 重采样后 0.48m), 跨过前进/倒车
  // cusp 时每帧来回翻转 -> resetNominal -> 原地不动.
  bool dir_known = false;
  bool dir_reverse = false;
  for (size_t k = idx_; k < std::min(path_.points.size(), idx_ + 3); ++k) {
    if (std::fabs(path_.points[k].velocity_x) > 0.1) {
      const bool r = path_.points[k].velocity_x < 0.0;
      if (!dir_known) { dir_known = true; dir_reverse = r; }
      else if (r != dir_reverse) { dir_known = false; break; }   // 邻近段方向不一致
      if (k >= idx_ + 1) break;
    }
  }
  bool local_reverse = reverse_mode_;
  if (dir_known) local_reverse = dir_reverse;
  if (local_reverse != reverse_mode_) {
    reverse_mode_ = local_reverse;
    last_output_v_ = 0.0;
    last_output_w_ = 0.0;
    resetNominal();
  }
  if (nominal_.size() != static_cast<size_t>(horizon_)) resetNominal();

  std::normal_distribution<double> noise_v(0.0, sigma_v_);
  std::normal_distribution<double> noise_w(0.0, sigma_w_);

  // 近距离自适应采样幅度: 贴障碍时缩小噪声, 让采样集中在标称轨迹附近,
  // 减少"靠运气穿过窄口/蹭墙"的样本; 空旷区保持原幅度以保证通过性.
  const int margin_cost = core::footprintCost(local_costmap_, pose, footprint_samples_);
  const double sigma_scale = margin_cost >= 85 ? 0.5 : (margin_cost >= 60 ? 0.75 : 1.0);
  std::normal_distribution<double> noise_v_scaled(0.0, sigma_v_ * sigma_scale);
  std::normal_distribution<double> noise_w_scaled(0.0, sigma_w_ * sigma_scale);

  struct Sample {
    std::vector<Control> controls;
    double cost{0.0};
  };
  std::vector<Sample> sampled(static_cast<size_t>(samples_));
  double min_cost = std::numeric_limits<double>::max();

  // 当前速度来自里程计, 但本项目的 odom/twist 可能滞后甚至为 0.
  // 把上一周期实际下发的命令也作为加速度状态, 否则第一段控制量会被
  // 永久压到 max_accel*dt (0.04m/s), 机器人看起来一直不走.
  const double vmin = reverse_mode_ ? -max_vx_ : 0.0;
  const double vmax = reverse_mode_ ? 0.0 : max_vx_;
  const double base_v = std::max(vmin, std::min(vmax,
    std::max(vel.linear_x, last_output_v_)));
  const double base_w = std::abs(vel.angular_z) > 0.02 ?
    vel.angular_z : last_output_w_;
  const size_t kLookaheadPoints = 80;

  for (int k = 0; k < samples_; ++k) {
    auto& sample = sampled[static_cast<size_t>(k)];
    sample.controls = nominal_;
    core::Pose2D p = pose;
    double prev_v = base_v;
    double prev_w = base_w;
    size_t ni = idx_;
    double cost = 0.0;

    for (int t = 0; t < horizon_; ++t) {
      double v = sample.controls[static_cast<size_t>(t)].v + noise_v_scaled(rng_);
      double w = sample.controls[static_cast<size_t>(t)].w + noise_w_scaled(rng_);
      v = std::clamp(v, std::max(vmin, prev_v - max_ax_ * dt_),
                        std::min(vmax, prev_v + max_ax_ * dt_));
      w = std::clamp(w, prev_w - max_aw_ * dt_, prev_w + max_aw_ * dt_);
      w = std::clamp(w, -max_vw_, max_vw_);
      sample.controls[static_cast<size_t>(t)] = {v, w};

      p = integrate(p, v, w, dt_);

      ni = nearestPathIndex(
        p, ni, std::min(path_.points.size(), ni + kLookaheadPoints));
      const auto& ref = path_.points[ni].pose;
      const double dx = p.x - ref.x;
      const double dy = p.y - ref.y;
      const double lateral2 = dx * dx + dy * dy;
      // A* 未填路径点朝向; 用相邻点切线作为参考航向.
      // 原地旋转点(位置不变, velocity_x=0)没有切线, 必须用路径自带朝向,
      // 否则 atan2(0,0)=0 会把车往 0° 带, 和执行器的旋转目标打架.
      double ref_yaw = ref.theta;
      if (ni + 1 < path_.points.size()) {
        const auto& next = path_.points[ni + 1].pose;
        const double ddx = next.x - ref.x;
        const double ddy = next.y - ref.y;
        if (std::hypot(ddx, ddy) > 1e-3) ref_yaw = std::atan2(ddy, ddx);
      } else if (ni > 0) {
        const auto& prev = path_.points[ni - 1].pose;
        const double ddx = ref.x - prev.x;
        const double ddy = ref.y - prev.y;
        if (std::hypot(ddx, ddy) > 1e-3) ref_yaw = std::atan2(ddy, ddx);
      }
      // 倒车段: 车头朝向与路径切线相反. 切线只表示"走的方向", 参考航向
      // 要加 180°, 否则控制器先把车头转到前进方向再倒 -> 原地翻转卡死.
      if (path_.points[ni].velocity_x < -0.1) ref_yaw = normAngle(ref_yaw + M_PI);
      const double heading_err = normAngle(p.theta - ref_yaw);
      cost += path_weight_ * lateral2 +
              heading_weight_ * heading_err * heading_err;
      const int cell = core::footprintCost(local_costmap_, p, footprint_samples_);
      if (cell >= 100) {
        cost += 1000.0;
      } else if (cell > 60) {
        // 蓝色膨胀区 1..99 是可通行的软代价. 0.9m 膨胀会让窄走廊
        // 几乎整片都是 40-60, 若每一格都重罚, 控制器的速度项会被
        // 完全压掉, 机器人以 0.05m/s 爬行并频繁触探索超时.
        // 只在进入约 0.5m 以内 (cost>60) 时开始惩罚, 硬边界仍由
        // cell>=100 和 LiDAR 安全层负责.
        cost += obstacle_weight_ * static_cast<double>(cell - 60);
      }
      cost += control_weight_ * (v * v + w * w);
      // 巡航目标就是配置的 max_velocity_x; 实际值由加速度、路径和
      // 障碍代价约束. 旧实现固定 75%, 导致把上限设成 1.0 也只能跑 ~0.75.
      // 巡航目标: 有轨迹时用轨迹参考速度 (cusp/末段自动减速), 否则用 max_velocity_x
      double v_ref = reverse_mode_ ? -max_vx_ : max_vx_;
      if (traj_.valid && traj_.points.size() == path_.points.size()) {
        const double tv = traj_.points[ni].v;
        if (tv >= 0.0) {
          v_ref = std::max(0.02, std::min(max_vx_, tv));
        } else {
          v_ref = -std::max(0.02, std::min(max_vx_, -tv));
        }
      }
      cost += speed_weight_ * (v - v_ref) * (v - v_ref) * dt_;
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

  last_min_cost_ = std::min(min_cost, 1e12);
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
  if (sum_w <= 0.0) return {0.0, 0.0, 0.0};

  for (int t = 0; t < horizon_; ++t) {
    nominal_[static_cast<size_t>(t)] = {
      weighted[static_cast<size_t>(t)].v / sum_w,
      weighted[static_cast<size_t>(t)].w / sum_w};
  }

  const Control first = nominal_.front();
  // 滚动时域: 平移标称控制序列, 最后一个保持.
  for (size_t t = 0; t + 1 < nominal_.size(); ++t) nominal_[t] = nominal_[t + 1];

  last_output_v_ = std::clamp(first.v, vmin, vmax);
  last_output_w_ = std::clamp(first.w, -max_vw_, max_vw_);
  last_nearest_idx_ = nearestPathIndex(pose);
  last_goal_error_ = pose.distanceTo(path_.points.back().pose);
  last_mean_cost_ = 0.0;
  int best_k = 0;
  for (size_t k = 0; k < sampled.size(); ++k) {
    last_mean_cost_ += sampled[k].cost;
    if (sampled[k].cost < sampled[best_k].cost) best_k = static_cast<int>(k);
  }
  last_mean_cost_ = sampled.empty() ? 0.0 :
    last_mean_cost_ / static_cast<double>(sampled.size());
  last_best_v_ = sampled.empty() ? 0.0 : sampled[best_k].controls.front().v;
  last_best_w_ = sampled.empty() ? 0.0 : sampled[best_k].controls.front().w;

  return {last_output_v_, 0.0, last_output_w_};
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
  reverse_mode_ = false;
  last_output_v_ = 0.0;
  last_output_w_ = 0.0;
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

std::string MppiController::getDiagnostics() const {
  char buf[320];
  std::snprintf(buf, sizeof(buf),
    "controller=mppi out_v=%.3f out_w=%.3f best_v=%.3f best_w=%.3f "
    "min_cost=%.3f mean_cost=%.3f goal_err=%.2f idx=%zu",
    last_output_v_, last_output_w_, last_best_v_, last_best_w_,
    last_min_cost_, last_mean_cost_, last_goal_error_, last_nearest_idx_);
  return std::string(buf);
}

void MppiController::setLocalCostmap(const core::CostmapGrid& costmap) {
  local_costmap_ = costmap;
  has_local_costmap_ = !local_costmap_.data.empty() &&
    local_costmap_.info.width > 0 && local_costmap_.info.height > 0;
}

}}  // namespace rosiwit_navigation::controllers
