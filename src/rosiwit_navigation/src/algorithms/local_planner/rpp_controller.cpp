#include "rosiwit_navigation/nav_core/rpp_controller.hpp"

#include "rosiwit_navigation/nav_core/costmap_layer.hpp"   // kCostLethal

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

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

namespace
{
bool insidePolygon(const std::vector<std::pair<double, double>> & poly, double x, double y)
{
  bool in = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const double yi = poly[i].second, yj = poly[j].second;
    if ((yi > y) != (yj > y)) {
      const double xi = poly[i].first, xj = poly[j].first;
      const double xc = xi + (y - yi) * (xj - xi) / (yj - yi + 1e-12);
      if (x < xc) in = !in;
    }
  }
  return in;
}
}  // namespace

void RppController::setLocalCostmap(const core::CostmapGrid & grid)
{
  if (!map_logged_) {
    map_logged_ = true;
    std::cout << "[RPP] footprint-aware costmap: " << grid.info.width << "x"
              << grid.info.height << " res=" << grid.info.resolution
              << " samples=" << fp_samples_.size() << std::endl;
  }
  auto next = std::make_shared<core::CostmapGrid>(grid);
  std::lock_guard<std::mutex> lk(map_mu_);
  costmap_ptr_ = std::move(next);
}

bool RppController::initialize(const core::ControllerConfig & config)
{
  cfg_ = config;
  max_vx_ = config.max_linear_velocity > 0.0 ? config.max_linear_velocity : 0.4;
  max_vw_ = config.max_angular_velocity > 0.0 ? config.max_angular_velocity : 0.5;
  max_ax_ = config.max_accel_x > 0.0 ? config.max_accel_x : 0.5;
  max_aw_ = config.max_accel_theta > 0.0 ? config.max_accel_theta : 0.2;
  goal_tol_ = config.xy_goal_tolerance > 0.0 ? config.xy_goal_tolerance : 0.2;

  // footprint 采样点 (车体系): 边框按 footprint_sample_spacing 等分, 内部再打一层
  // 栅格 —— 只采样边框会漏掉"细杆/桌腿正好落在多边形内部"的情形。
  fp_samples_.clear();
  const auto & fp = config.footprint;
  if (fp.size() >= 3) {
    const double step = config.footprint_sample_spacing > 0.0
        ? std::max(0.02, config.footprint_sample_spacing) : 0.05;
    double minx = fp[0].first, maxx = fp[0].first;
    double miny = fp[0].second, maxy = fp[0].second;
    for (size_t i = 0; i < fp.size(); ++i) {
      const auto & a = fp[i];
      const auto & b = fp[(i + 1) % fp.size()];
      minx = std::min(minx, a.first);  maxx = std::max(maxx, a.first);
      miny = std::min(miny, a.second); maxy = std::max(maxy, a.second);
      const double len = std::hypot(b.first - a.first, b.second - a.second);
      const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
      for (int k = 0; k < n; ++k) {
        const double t = static_cast<double>(k) / static_cast<double>(n);
        fp_samples_.emplace_back(a.first + (b.first - a.first) * t,
                                 a.second + (b.second - a.second) * t);
      }
    }
    for (double x = minx + 0.5 * step; x < maxx; x += step) {
      for (double y = miny + 0.5 * step; y < maxy; y += step) {
        if (insidePolygon(fp, x, y)) fp_samples_.emplace_back(x, y);
      }
    }
  }
  // 现场可观测性: samples=0 表示节点没注入 footprint (会退化成纯跟踪, 检查不生效)。
  std::cout << "[RPP] footprint sweep check: samples=" << fp_samples_.size()
            << " step=" << (config.footprint_sample_spacing > 0.0
                             ? config.footprint_sample_spacing : 0.05)
            << " m" << std::endl;
  return true;
}

double RppController::arcClearanceScale(
    const core::Pose2D & pose, double v, double w) const
{
  std::shared_ptr<const core::CostmapGrid> map;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    map = costmap_ptr_;
  }
  if (!map || map->data.empty() || fp_samples_.empty()) return 1.0;
  const double res = map->info.resolution;
  const int gw = static_cast<int>(map->info.width);
  const int gh = static_cast<int>(map->info.height);
  const double speed = std::fabs(v);
  if (res <= 1e-6 || gw <= 0 || gh <= 0) return 1.0;
  if (speed < 1e-3) return 1.0;   // 原地旋转由安全层的旋转区负责

  const double ox = map->info.origin.position.x;
  const double oy = map->info.origin.position.y;
  // 预测口径与安全层一致: 前视 ~0.4m 的行程 (至少 0.35m, 最多 1.0s)
  const double horizon_dist = std::max(0.35, speed * 0.8);
  const double horizon_t = std::min(1.0, horizon_dist / speed);
  const int steps = std::max(2, static_cast<int>(std::ceil(horizon_t / 0.05)));
  const double dt = horizon_t / static_cast<double>(steps);

  double px = pose.x, py = pose.y, th = pose.theta;
  for (int s = 1; s <= steps; ++s) {
    px += v * std::cos(th) * dt;
    py += v * std::sin(th) * dt;
    th += w * dt;
    const double c = std::cos(th), sn = std::sin(th);
    for (const auto & p : fp_samples_) {
      const double wx = px + c * p.first - sn * p.second;
      const double wy = py + sn * p.first + c * p.second;
      const int gx = static_cast<int>(std::floor((wx - ox) / res));
      const int gy = static_cast<int>(std::floor((wy - oy) / res));
      if (gx < 0 || gy < 0 || gx >= gw || gy >= gh) continue;   // 窗口外由安全层兜底
      // 只认真实障碍 (kCostLethal). raw 层没有 99 膨胀环 —— 用膨胀环判定会让
      // 0.8m 门洞里 0.66m 的车体永远"被挡"。
      if (map->data[static_cast<size_t>(gy) * gw + gx] >= core::kCostLethal) {
        return static_cast<double>(s - 1) / static_cast<double>(steps);
      }
    }
  }
  return 1.0;
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
  // 轨迹前馈 (若已参数化): 用同索引点的参考速度做上限, cusp/末段自动减速
  if (traj_.valid && traj_.points.size() == path_.points.size()) {
    const size_t ti = std::min(idx_, traj_.points.size() - 1);
    v_ref = std::min(v_ref, std::max(0.05, std::fabs(traj_.points[ti].v)));
  }
  const double reach = std::hypot(path_.points.back().pose.x - pose.x,
                                  path_.points.back().pose.y - pose.y);
  v_ref = std::min(v_ref, std::max(0.08, 0.9 * reach));         // 末段减速

  const double v_des = dir * v_ref;
  const double v = std::max(last_v_ - max_ax_ * 0.1,
                            std::min(last_v_ + max_ax_ * 0.1, v_des));
  {
    // [RPPProbe] 倒车段为什么不倒车: 打印方向标记/索引/航向误差/速度
    static int probe_n = 0;
    if (++probe_n % 50 == 0) {   // 10Hz 控制下约 0.2Hz
      std::cout << "[RPPProbe] pts=" << path_.points.size()
                << " idx=" << idx_ << " dir=" << dir
                << " vx=" << path_.points[idx_].velocity_x
                << " e_theta_deg=" << e_theta * 57.2957795
                << " e_y=" << e_y
                << " v_ref=" << v_ref << " v=" << v
                << " pose_yaw_deg=" << pose.theta * 57.2957795
                << " ref_yaw_deg=" << heading_ref * 57.2957795 << std::endl;
    }
  }
  const double w_ff = (L > 1e-3) ? 2.0 * v * std::sin(e_theta) / L : 0.0;
  const double w_des = std::max(-max_vw_, std::min(max_vw_, w_ff + 0.8 * e_y * v));
  const double w = std::max(last_w_ - max_aw_ * 0.1,
                            std::min(last_w_ + max_aw_ * 0.1, w_des));

  // footprint 扫掠检查 —— **只观测, 不动作**。
  // 曾经的实现按"弧上安全比例"降 v: 实测在贴墙/门洞处 scale 会掉到 0.15~0.35,
  // 而本控制器的 w 与 v 成正比 (w_ff = 2*v*sin(e)/L, 修正项 0.8*e_y*v), 于是
  // v→0 时 w 也→0: 车既不前进也不转向, 10s 后被 ActionTask 判 FAILED_STUCK
  // (simulation.yaml movebase_stuck_timeout=10), 表现为"完全走不动"。
  // 而且"减速"并不能解决切内角 (那是前视点几何问题), 所以这里只保留度量,
  // 供现场判断门洞段的切角程度: [RPPClr] scale 越小说明指令弧越贴障碍。
  const double clr = arcClearanceScale(pose, v, w);
  if (clr < 1.0) {
    const int n = ++clr_probe_;
    if (n <= 5 || n % 20 == 0) {
      std::cout << "[RPPClr] arc tight: n=" << n << " scale=" << clr
                << " v=" << v << " w=" << w << " (仅观测, 不降速)" << std::endl;
    }
  }
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
