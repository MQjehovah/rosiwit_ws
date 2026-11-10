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
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    costmap_ptr_ = std::move(next);
  }
  // 新代价地图 -> 净宽可能变了, 重建车道标记
  updateLaneProfile();
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
  updateLaneProfile();
}

int RppController::laneZoneAt(size_t i) const
{
  if (!cfg_.door_lane_enabled || lane_zone_.size() != path_.points.size()) { return 0; }
  return (i < lane_zone_.size()) ? static_cast<int>(lane_zone_[i]) : 0;
}

// 窄口车道标记: 沿路径每个点向左右打短射线, 量净宽; 净宽 < lane_width 视为窄口。
//   1 = 窄口内   (冻结 ω, 限速)
//   2 = 窄口前的收敛段 (只限速, 正常转向: 偏差必须在这里修完)
void RppController::updateLaneProfile()
{
  lane_zone_.assign(path_.points.size(), 0);
  if (!cfg_.door_lane_enabled || path_.points.size() < 3) { return; }
  std::shared_ptr<const core::CostmapGrid> map;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    map = costmap_ptr_;
  }
  if (!map || map->data.empty()) { return; }
  const double res = map->info.resolution;
  const int gw = static_cast<int>(map->info.width);
  const int gh = static_cast<int>(map->info.height);
  const double ox = map->info.origin.position.x;
  const double oy = map->info.origin.position.y;
  if (res <= 1e-6 || gw <= 0 || gh <= 0) { return; }

  const size_t n = path_.points.size();
  // 每个点的"单侧净宽" (m), 打不到墙就记为 lane_width (即"不窄")
  std::vector<double> half_width(n, cfg_.lane_width);
  const double max_ray = std::max(0.3, cfg_.lane_width);
  for (size_t i = 0; i < n; ++i) {
    const auto & p = path_.points[i].pose;
    // 切线/法向
    const size_t ia = (i == 0) ? 0 : i - 1;
    const size_t ib = (i + 1 < n) ? i + 1 : i;
    double tx = path_.points[ib].pose.x - path_.points[ia].pose.x;
    double ty = path_.points[ib].pose.y - path_.points[ia].pose.y;
    const double tn = std::hypot(tx, ty);
    if (tn < 1e-6) { continue; }
    tx /= tn;
    ty /= tn;
    double worst = cfg_.lane_width;
    for (int side = -1; side <= 1; side += 2) {
      for (double d = 0.05; d <= max_ray + 1e-9; d += 0.02) {
        const double wx = p.x + side * d * (-ty);
        const double wy = p.y + side * d * tx;
        const int gx = static_cast<int>(std::floor((wx - ox) / res));
        const int gy = static_cast<int>(std::floor((wy - oy) / res));
        if (gx < 0 || gy < 0 || gx >= gw || gy >= gh) { break; }   // 窗口外: 不算窄
        if (map->data[static_cast<size_t>(gy) * gw + gx] >= core::kCostLethal) {
          worst = std::min(worst, d);
          break;
        }
      }
    }
    half_width[i] = worst;
  }

  // 窄段 = 两侧净宽之和 < lane_width
  std::vector<bool> narrow(n, false);
  for (size_t i = 0; i < n; ++i) { narrow[i] = (2.0 * half_width[i]) < cfg_.lane_width; }

  // 标记: 窄段本身 = 1; 窄段前 lane_engage_dist 内 = 2; 窄段后 lane_exit_dist 内 = 1
  size_t i = 0;
  while (i < n) {
    if (!narrow[i]) { ++i; continue; }
    size_t j = i;
    while (j + 1 < n && narrow[j + 1]) { ++j; }
    for (size_t k = i; k <= j; ++k) { lane_zone_[k] = 1; }
    // 之后
    double acc = 0.0;
    for (size_t k = j + 1; k < n && acc < cfg_.lane_exit_dist; ++k) {
      acc = std::hypot(path_.points[k].pose.x - path_.points[j].pose.x,
                       path_.points[k].pose.y - path_.points[j].pose.y);
      lane_zone_[k] = 1;
    }
    // 之前 (收敛段: 只限速)
    acc = 0.0;
    for (size_t k = i; k-- > 0 && acc < cfg_.lane_engage_dist;) {
      acc = std::hypot(path_.points[k].pose.x - path_.points[i].pose.x,
                       path_.points[k].pose.y - path_.points[i].pose.y);
      if (lane_zone_[k] == 0) { lane_zone_[k] = 2; }
    }
    i = j + 1;
  }
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

  // ---- TRACK: 对齐 nav2_regulated_pure_pursuit_controller ----
  // 前瞻: use_velocity_scaled_lookahead_dist -> L = v*lookahead_time, 夹在 [min,max]
  const double la_min = cfg_.min_lookahead_distance > 0.0 ? cfg_.min_lookahead_distance : 0.3;
  const double la_max = cfg_.max_lookahead_distance > 0.0
      ? std::max(cfg_.max_lookahead_distance, la_min) : 0.9;
  const double la_time = cfg_.lookahead_time > 0.0 ? cfg_.lookahead_time : 1.5;
  double L = cfg_.lookahead_distance > 0.0 ? cfg_.lookahead_distance : 0.6;
  if (cfg_.use_velocity_scaled_lookahead_dist) { L = std::fabs(last_v_) * la_time; }
  L = std::min(std::max(L, la_min), la_max);
  const size_t la = lookaheadIndex(s_cur_ + L);
  const auto & ref = path_.points[la].pose;
  const double dx = ref.x - pose.x;
  const double dy = ref.y - pose.y;
  const double dist = std::hypot(dx, dy);
  const int dir = dirAt(idx_);
  const double heading_ref = (dist > 1e-3) ? std::atan2(dy, dx) : ref.theta;
  double e_theta = normAngle(heading_ref - pose.theta);
  if (dir < 0) e_theta = normAngle(e_theta - M_PI);

  // 横向误差 (路径切向叉积)
  const auto & cur = path_.points[idx_].pose;
  const double tang = std::atan2(ref.y - cur.y + 1e-9, ref.x - cur.x + 1e-9);
  const double e_y = std::sin(tang) * (pose.x - cur.x) - std::cos(tang) * (pose.y - cur.y);

  // rotate_to_heading (等价 nav2 rotation_shim / RPP use_rotate_to_heading):
  // 航向差超过 rotate_to_heading_min_angle 且车还没在走 -> 原地转到路径朝向,
  // 不要用"边转边走"的弧去消化几十度 (那会把车甩出 0.2m, 窄门收不回来)。
  if (cfg_.use_rotate_to_heading && std::fabs(last_v_) < 0.05 &&
      std::fabs(e_theta) > cfg_.rotate_to_heading_min_angle) {
    if (align_dir_ == 0) { align_dir_ = e_theta > 0.0 ? 1 : -1; }
    const double w_max = cfg_.rotate_to_heading_angular_vel > 0.0
        ? cfg_.rotate_to_heading_angular_vel : 1.8;
    const double w = align_dir_ * std::min(w_max, std::max(0.15, 2.0 * std::fabs(e_theta)));
    last_v_ = 0.0;
    last_w_ = w;
    return {0.0, 0.0, w};
  }
  align_dir_ = 0;

  double v_ref = max_vx_;
  // 曲率调速 (use_regulated_linear_velocity_scaling)
  const double curvature = (L > 1e-3) ? 2.0 * std::sin(e_theta) / L : 0.0;
  const double min_radius = cfg_.regulated_linear_scaling_min_radius > 0.0
      ? cfg_.regulated_linear_scaling_min_radius : 0.9;
  const double min_speed = std::max(0.05, cfg_.regulated_linear_scaling_min_speed);
  if (cfg_.use_regulated_linear_velocity_scaling && std::fabs(curvature) > 1e-3) {
    const double radius = 1.0 / std::fabs(curvature);
    const double scale = std::min(1.0, std::max(min_speed / std::max(0.05, max_vx_),
                                                radius / min_radius));
    v_ref *= scale;
  }
  // 轨迹前馈 (若已参数化): 用同索引点的参考速度做上限
  if (traj_.valid && traj_.points.size() == path_.points.size()) {
    const size_t ti = std::min(idx_, traj_.points.size() - 1);
    v_ref = std::min(v_ref, std::max(0.05, std::fabs(traj_.points[ti].v)));
  }
  const double reach = std::hypot(path_.points.back().pose.x - pose.x,
                                  path_.points.back().pose.y - pose.y);
  // 末段调速 (approach_velocity_scaling_dist / min_approach_linear_velocity)
  const double app_dist = cfg_.approach_velocity_scaling_dist > 0.0
      ? cfg_.approach_velocity_scaling_dist : 0.6;
  const double app_min = cfg_.min_approach_linear_velocity > 0.0
      ? cfg_.min_approach_linear_velocity : 0.05;
  if (reach < app_dist) {
    v_ref = std::min(v_ref, std::max(app_min, max_vx_ * reach / app_dist));
  }
  v_ref = std::max(v_ref, min_speed);   // 防止任何调速把车压死

  const double v_des = dir * v_ref;
  double v = std::max(last_v_ - max_ax_ * 0.1,
                      std::min(last_v_ + max_ax_ * 0.1, v_des));

  // ---- 门区直穿 (窄口 lane) ----
  // 窄口前 lane_engage_dist: 只限速 (偏差要在这里收敛完)
  // 窄口内: 冻结横向修正 (|ω| <= lane_max_omega), 保持直穿
  //   * 窄口里横向余量只有几厘米, 边转边走必然把车体斜置撑宽
  //   * 高 ω 还会放大雷达点云与位姿的时间偏斜 (0.8rad/s * 120ms ≈ 5.5°),
  //     在 0.6m 处把门框点"转"进门洞 7cm -> 幻影障碍 -> 停车
  const int lane_zone = laneZoneAt(idx_);
  if (lane_zone != 0 && cfg_.lane_max_speed > 0.0) {
    v = std::min(v, std::max(cfg_.lane_max_speed, 0.05));
  }

  if (cfg_.debug_probe) {
    static int probe_n = 0;
    if (++probe_n % 50 == 0) {
      std::cout << "[RPPProbe] pts=" << path_.points.size()
                << " idx=" << idx_ << " dir=" << dir
                << " e_theta_deg=" << e_theta * 57.2957795
                << " e_y=" << e_y
                << " L=" << L << " v_ref=" << v_ref << " v=" << v
                << " pose_yaw_deg=" << pose.theta * 57.2957795 << std::endl;
    }
  }

  const double w_ff = (L > 1e-3) ? 2.0 * v * std::sin(e_theta) / L : 0.0;
  const double w_des = std::max(-max_vw_, std::min(max_vw_, w_ff + 0.8 * e_y * v));
  double w = std::max(last_w_ - max_aw_ * 0.1,
                      std::min(last_w_ + max_aw_ * 0.1, w_des));

  // 窄口内冻结横向修正 (航向已经基本正了才冻结; 本来就歪的必须先转正)
  if (lane_zone == 1 && cfg_.lane_max_omega >= 0.0 &&
      std::fabs(e_theta) < cfg_.lane_freeze_max_heading) {
    const double wc = cfg_.lane_max_omega;
    w = std::max(-wc, std::min(wc, w));
  }

  // 碰撞检测调速 (nav2 use_collision_detection): 沿指令弧 max_allowed_time_to_collision
  // 内会撞到真实障碍就按"弧上可安全行程比例"减速。(v,w) 同时缩放 -> 曲率不变。
  const double clr = arcClearanceScale(pose, v, w);
  if (cfg_.use_collision_detection && clr < 1.0) {
    double k = std::max(0.0, clr);
    if (std::fabs(v) > 1e-6) { k = std::max(k, min_speed / std::fabs(v)); }
    k = std::min(1.0, k);
    v *= k;
    w *= k;
  }
  if (cfg_.debug_probe && clr < 1.0) {
    const int n = ++clr_probe_;
    if (n <= 5 || n % 20 == 0) {
      std::cout << "[RPPClr] arc tight: n=" << n << " scale=" << clr
                << " v=" << v << " w=" << w << std::endl;
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
