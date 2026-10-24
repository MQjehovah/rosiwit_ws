#include "rosiwit_navigation/nav_core/rpp_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rosiwit_navigation
{
namespace controllers
{

namespace
{
double normAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

bool RppController::initialize(const core::ControllerConfig & config)
{
  cfg_ = config;
  max_vx_ = config.max_linear_velocity > 0.0 ? config.max_linear_velocity : 0.4;
  max_vw_ = config.max_angular_velocity > 0.0 ? config.max_angular_velocity : 0.5;
  max_ax_ = config.max_accel_x > 0.0 ? config.max_accel_x : 0.5;
  max_aw_ = config.max_accel_theta > 0.0 ? config.max_accel_theta : 0.2;
  goal_tol_ = config.xy_goal_tolerance > 0.0 ? config.xy_goal_tolerance : 0.2;
  return true;
}

int RppController::dirAt(size_t i) const
{
  if (i >= path_.points.size()) return 1;
  const double v = path_.points[i].velocity_x;
  if (std::fabs(v) < 0.1) return 0;   // 原地旋转段
  return v < 0.0 ? -1 : 1;
}

void RppController::setPath(const core::Path & path)
{
  path_ = path;
  rebuild();
}

void RppController::rebuild()
{
  s_.clear();
  idx_ = 0;
  s_cur_ = 0.0;
  last_v_ = 0.0;
  last_w_ = 0.0;
  s_.reserve(path_.points.size());
  double acc = 0.0;
  for (size_t i = 0; i < path_.points.size(); ++i) {
    if (i > 0) {
      const auto & a = path_.points[i - 1].pose;
      const auto & b = path_.points[i].pose;
      acc += std::hypot(b.x - a.x, b.y - a.y);
    }
    s_.push_back(acc);
  }
}

void RppController::reset()
{
  idx_ = 0;
  s_cur_ = 0.0;
  last_v_ = 0.0;
  last_w_ = 0.0;
}

void RppController::setVelocityLimit(double linear_x, double angular_z)
{
  max_vx_ = linear_x > 0.0 ? linear_x : max_vx_;
  max_vw_ = angular_z > 0.0 ? angular_z : max_vw_;
}

double RppController::getProgress() const
{
  if (s_.empty() || s_.back() < 1e-6) return 0.0;
  return std::min(1.0, s_cur_ / s_.back());
}

void RppController::updateProgress(const core::Pose2D & pose)
{
  // 单调推进: 只在前方 3m 窗口内找最近点, 防止 U 形路径跳段.
  if (path_.points.empty()) return;
  const double window = 3.0;
  size_t best = idx_;
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = idx_; i < path_.points.size(); ++i) {
    if (s_[i] - s_cur_ > window) break;
    const auto & q = path_.points[i].pose;
    const double d = std::hypot(q.x - pose.x, q.y - pose.y);
    if (d < best_d) { best_d = d; best = i; }
  }
  if (best > idx_) { idx_ = best; s_cur_ = s_[best]; }
}

size_t RppController::lookaheadIndex(double s) const
{
  size_t i = idx_;
  while (i + 1 < s_.size() && s_[i] < s) ++i;
  return i;
}

core::VelocityCommand RppController::computeVelocityCommand(
    const core::Pose2D & pose, const core::VelocityCommand & current_velocity)
{
  (void)current_velocity;
  if (path_.points.size() < 2) return {0.0, 0.0, 0.0};
  updateProgress(pose);

  // ---- 原地旋转段 (velocity_x==0 且位置不变) ----
  if (dirAt(idx_) == 0) {
    size_t end = idx_;
    const auto & base = path_.points[idx_].pose;
    while (end + 1 < path_.points.size() &&
           std::hypot(path_.points[end + 1].pose.x - base.x,
                      path_.points[end + 1].pose.y - base.y) < 0.03) {
      ++end;
    }
    const double target = path_.points[end].pose.theta;
    const double err = normAngle(target - pose.theta);
    if (std::fabs(err) < 0.05) {
      idx_ = std::min(end + 1, path_.points.size() - 1);
      s_cur_ = s_[idx_];
      align_dir_ = 0;
      return {0.0, 0.0, 0.0};
    }
    // 方向只在旋转开始时锁存一次, 中途绝不反转 (否则过冲后 err 反号 -> 转回去)
    if (align_dir_ == 0) align_dir_ = err > 0.0 ? 1 : -1;
    const double w = align_dir_ * std::min(max_vw_,
                                           std::max(0.15, 2.0 * std::fabs(err)));
    last_v_ = 0.0;
    last_w_ = w;
    return {0.0, 0.0, w};
  }

  // ---- TRACK: 弧长前视 + RPP 调速 ----
  const double L = std::max(0.25, 0.35 + 0.8 * std::fabs(last_v_));
  const size_t la = lookaheadIndex(s_cur_ + L);
  const auto & ref = path_.points[la].pose;
  const double dx = ref.x - pose.x;
  const double dy = ref.y - pose.y;
  const double dist = std::hypot(dx, dy);
  const int dir = dirAt(idx_);
  const double heading_ref = (dist > 1e-3) ? std::atan2(dy, dx) : ref.theta;
  double e_theta = normAngle(heading_ref - pose.theta);
  // 倒车: 机器人朝向与行进方向相反, 参考航向加 π
  if (dir < 0) e_theta = normAngle(e_theta - M_PI);

  // 横向误差 (路径切向叉积)
  const auto & cur = path_.points[idx_].pose;
  const double tang = std::atan2(ref.y - cur.y + 1e-9, ref.x - cur.x + 1e-9);
  const double e_y = std::sin(tang) * (pose.x - cur.x) - std::cos(tang) * (pose.y - cur.y);

  double v_ref = max_vx_;
  v_ref *= std::max(0.3, 1.0 - 1.2 * std::fabs(e_y));          // 横向误差调速
  v_ref *= std::max(0.3, std::cos(e_theta));                    // 航向误差调速
  const double reach = std::hypot(path_.points.back().pose.x - pose.x,
                                  path_.points.back().pose.y - pose.y);
  v_ref = std::min(v_ref, std::max(0.08, 0.9 * reach));         // 末段减速

  const double v_des = dir * v_ref;
  const double v = std::max(last_v_ - max_ax_ * 0.1,
                            std::min(last_v_ + max_ax_ * 0.1, v_des));
  const double w_ff = (L > 1e-3) ? 2.0 * v * std::sin(e_theta) / L : 0.0;
  const double w_des = std::max(-max_vw_, std::min(max_vw_, w_ff + 0.8 * e_y * v));
  const double w = std::max(last_w_ - max_aw_ * 0.1,
                            std::min(last_w_ + max_aw_ * 0.1, w_des));
  last_v_ = v;
  last_w_ = w;
  return {v, 0.0, w};
}

bool RppController::isGoalReached(const core::Pose2D & pose) const
{
  if (path_.points.empty()) return false;
  const auto & g = path_.points.back().pose;
  return std::hypot(g.x - pose.x, g.y - pose.y) < goal_tol_;
}

}  // namespace controllers
}  // namespace rosiwit_navigation
