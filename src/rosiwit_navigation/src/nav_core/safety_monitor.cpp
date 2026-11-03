#include "rosiwit_navigation/nav_core/safety_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace rosiwit_navigation
{
namespace core
{

namespace
{

/// 滞回更新 (nav2 的 trigger/release_consecutive_points 等价物).
bool updateHysteresis(
  bool & active, int & trigger, int & release, bool hit,
  int trigger_frames, int release_frames)
{
  const int tf = std::max(1, trigger_frames);
  const int rf = std::max(1, release_frames);
  if (active) {
    if (hit) {
      release = 0;
    } else if (++release >= rf) {
      active = false;
      release = 0;
    }
  } else if (hit) {
    if (++trigger >= tf) {
      active = true;
      trigger = 0;
    }
  } else {
    trigger = 0;
  }
  return active;
}

}  // namespace

bool pointInPolygon(
  double x, double y,
  const std::vector<std::pair<double, double>> & polygon)
{
  if (polygon.size() < 3) return false;
  bool inside = false;
  const size_t n = polygon.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = polygon[i].first;
    const double yi = polygon[i].second;
    const double xj = polygon[j].first;
    const double yj = polygon[j].second;
    const bool crosses = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (crosses) inside = !inside;
  }
  return inside;
}

int countPointsInPolygon(
  const std::vector<std::pair<double, double>> & points,
  const std::vector<std::pair<double, double>> & polygon)
{
  if (polygon.size() < 3) return 0;
  int count = 0;
  for (const auto & p : points) {
    if (pointInPolygon(p.first, p.second, polygon)) ++count;
  }
  return count;
}

/// 按车体几何 + 制动距离生成停车区 (base 系) —— 等价于 nav2_collision_monitor 的
/// velocity_polygon (translation_forward / translation_backward / rotation)：
/// 前/后方向: 车体该侧外沿 + v²/(2a) + v*reaction + stop_margin;
/// 侧向: 半宽 + side_margin + 转向扫掠 (v*|w|*reaction), 上限 turn_margin。
/// extra_time: SLOWDOWN 区用它把"看得更远"的那一段加进去 (同一个多边形, 更长的车头)。
void buildDynamicStopPolygon(
  const VelocityCommand & cmd,
  const std::vector<std::pair<double, double>> & footprint,
  double brake_decel, double reaction_time, double stop_margin,
  double side_margin, double turn_margin,
  std::vector<std::pair<double, double>> & out,
  double extra_time = 0.0)
{
  if (footprint.size() < 3) { out.clear(); return; }

  double min_x = footprint[0].first, max_x = footprint[0].first;
  double min_y = footprint[0].second, max_y = footprint[0].second;
  for (const auto & p : footprint) {
    min_x = std::min(min_x, p.first);
    max_x = std::max(max_x, p.first);
    min_y = std::min(min_y, p.second);
    max_y = std::max(max_y, p.second);
  }

  const double v = std::fabs(cmd.linear_x);
  const double a = std::max(0.05, brake_decel);
  const double brake = v * v / (2.0 * a) +
                       v * (std::max(0.0, reaction_time) + std::max(0.0, extra_time));
  const double half_width = std::max(std::fabs(min_y), std::fabs(max_y));
  const double turn = std::min(std::max(0.0, turn_margin),
                               v * std::fabs(cmd.angular_z) * std::max(0.0, reaction_time));
  const double side = half_width + std::max(0.0, side_margin) + turn;

  // 停区余量只加在**行进方向**那一端: 倒车时车头不该再留 15cm 停区, 否则
  // 前方障碍会让"倒车脱困"每拍被清零 (实车 2026-11-03 21:16: 前方 0.16m 有墙,
  // recovery 倒车 v=-0.03 连续被 STOP, 20s 后 FAILED_NO_PROGRESS)。
  const bool forward = cmd.linear_x >= 0.0;
  const double margin = std::max(0.0, stop_margin);
  double front = max_x + (forward ? margin : 0.0);
  double rear = -min_x + (forward ? 0.0 : margin);
  if (forward) {
    front += brake;
  } else {
    rear += brake;
  }

  out = {{-rear, -side}, {front, -side}, {front, side}, {-rear, side}};
}

double approachScale(  const std::vector<std::pair<double, double>> & points,
  double vx, double vy, double wz,
  const std::vector<std::pair<double, double>> & footprint,
  double max_time, double time_step)
{  if (max_time <= 0.0 || time_step <= 0.0 || footprint.size() < 3) return 1.0;
  if (std::hypot(vx, vy) < 1e-6 && std::abs(wz) < 1e-6) return 1.0;
  if (points.empty()) return 1.0;

  double radius = 0.0;
  for (const auto & p : footprint) {
    radius = std::max(radius, std::hypot(p.first, p.second));
  }
  const double max_range = std::hypot(vx, vy) * max_time + radius + 0.2;

  std::vector<std::pair<double, double>> candidates;
  candidates.reserve(points.size());
  for (const auto & p : points) {
    if (std::hypot(p.first, p.second) <= max_range) candidates.push_back(p);
  }
  if (candidates.empty()) return 1.0;
  if (candidates.size() > 2000) candidates.resize(2000);

  const int steps = std::max(1, static_cast<int>(std::ceil(max_time / time_step)));
  double x = 0.0, y = 0.0, theta = 0.0;
  for (int step = 1; step <= steps; ++step) {
    x += (vx * std::cos(theta) - vy * std::sin(theta)) * time_step;
    y += (vx * std::sin(theta) + vy * std::cos(theta)) * time_step;
    theta += wz * time_step;
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    for (const auto & p : candidates) {
      const double dx = p.first - x;
      const double dy = p.second - y;
      const double bx = dx * c + dy * s;
      const double by = -dx * s + dy * c;
      if (pointInPolygon(bx, by, footprint)) {
        return std::max(0.0, std::min(1.0, (step * time_step) / max_time));
      }
    }
  }
  return 1.0;
}

/// 动态安全区 = 当前 footprint 在未来 t_stop 内的**扫掠区域**(任意 v,w)。
/// 做法: 沿预测轨迹采样 n 个未来位姿, 点落在任一位姿的 footprint 内即算命中。
/// 与"footprint 外扩"(静态安全区)互补: 外扩管静止/贴边余量, 扫掠管制动过程。
/// 预测用**匀速**(不衰减)近似 -> 扫掠区偏大, 属于安全侧。
///
/// 余量必须**各向异性**: 前/后用 long_margin (制动余量), 侧向用 lat_margin。
/// 以前只有一个 margin = max(stop_margin, side_margin) 沿径向均匀外扩, 于是
/// 5cm 的"刹车余量"被同时加到了车身两侧: 0.33+0.05=0.38 > 0.75/2=0.375,
/// 0.75m 门洞在**任何**跟踪精度下都不可能通过 (实测门洞场景必停在门前 0.6m)。
int countSweptHits(const std::vector<std::pair<double, double>> & pts,
                   const std::vector<std::pair<double, double>> & footprint,
                   double v, double w, double brake_decel, double heading_brake_decel,
                   double reaction_time, int samples, double max_time,
                   double long_margin, double lat_margin, double extra_time)
{
  // extra_time: SLOWDOWN 区用它把"看得更远"的那一段加上去 (同样的 footprint, 更长的扫掠)

  if (footprint.size() < 3) { return 0; }
  // 先把 footprint 按静态余量外扩 (前/后 = long_margin, 左右 = lat_margin):
  // 这样"静态安全区(外扩)"与"动态安全区(扫掠)"是叠加的, 而不是二选一。
  // 用包围盒中心的各向异性缩放, 保持车体形状的相对比例 (梯形前脸不会被拉平)。
  std::vector<std::pair<double, double>> grown = footprint;
  {
    double min_x = footprint[0].first, max_x = footprint[0].first;
    double min_y = footprint[0].second, max_y = footprint[0].second;
    for (const auto & q : footprint) {
      min_x = std::min(min_x, q.first);
      max_x = std::max(max_x, q.first);
      min_y = std::min(min_y, q.second);
      max_y = std::max(max_y, q.second);
    }
    const double xmid = 0.5 * (min_x + max_x);
    const double ymid = 0.5 * (min_y + max_y);
    const double hx = std::max(1e-6, 0.5 * (max_x - min_x));
    const double hy = std::max(1e-6, 0.5 * (max_y - min_y));
    const double sx = (hx + std::max(0.0, long_margin)) / hx;
    const double sy = (hy + std::max(0.0, lat_margin)) / hy;
    for (auto & q : grown) {
      q.first = xmid + (q.first - xmid) * sx;
      q.second = ymid + (q.second - ymid) * sy;
    }
  }
  const auto & fp = grown;
  const int n = std::max(2, samples);
  const double a = std::max(0.05, brake_decel);
  const double aw = std::max(0.05, heading_brake_decel);
  double t_stop = std::max(std::fabs(v) / a, std::fabs(w) / aw) +
                  std::max(0.0, reaction_time) + std::max(0.0, extra_time);
  t_stop = std::min(t_stop, std::max(0.05, max_time));

  std::vector<std::vector<std::pair<double, double>>> poses;
  poses.reserve(static_cast<size_t>(n));
  const bool turning = std::fabs(w) > 1e-4;
  for (int k = 1; k <= n; ++k) {
    const double t = t_stop * static_cast<double>(k) / static_cast<double>(n);
    double px = 0.0, py = 0.0, dth = 0.0;
    if (turning) {
      const double r = v / w;
      dth = w * t;
      px = r * std::sin(dth);
      py = r * (1.0 - std::cos(dth));
    } else {
      px = v * t;
    }
    const double c = std::cos(dth), s2 = std::sin(dth);
    std::vector<std::pair<double, double>> poly;
    poly.reserve(fp.size());
    for (const auto & q : fp) {
      poly.push_back({px + c * q.first - s2 * q.second,
                      py + s2 * q.first + c * q.second});
    }
    poses.push_back(std::move(poly));
  }

  int hits = 0;
  for (const auto & pt : pts) {
    for (const auto & poly : poses) {
      if (pointInPolygon(pt.first, pt.second, poly)) { ++hits; break; }
    }
  }
  return hits;
}

SafetyResult SafetyMonitor::filter(
  const VelocityCommand & cmd,
  const std::vector<std::pair<double, double>> & points) const
{
  SafetyResult result;
  result.command = cmd;

  if (!config_.enabled) {
    result.state = "DISABLED";
    return result;
  }

  // 1) 车身排除: 只丢掉车身轮廓内的点 (nav2 exclusion_zones.robot_body).
  //    以前是在 node 里用一个大矩形过滤, 结果把安全区一起吞了.
  const auto & body = config_.body_polygon;
  std::vector<std::pair<double, double>> pts;
  if (body.size() < 3) {
    pts = points;
  } else {
    pts.reserve(points.size());
    for (const auto & p : points) {
      if (!pointInPolygon(p.first, p.second, body)) pts.push_back(p);
    }
  }

  // 动态安全区: 按当前速度选多边形 (nav2 velocity_polygon 语义)。
  // swept_geometry 是**更严格**的可选模式 (旧实现): 把 footprint 沿预测轨迹扫掠,
  // 并且把"制动期间继续以当前 ω 旋转"也算进横向占用 —— nav2 的 collision_monitor
  // 不做这件事, 因为它会与"窄通道可通过"直接冲突: 0.75m 门洞里 0.66m 车体只剩
  // 2~3cm 侧向余量, 任何 ω 在 0.4s 内转出的 5° 都会让扫掠区超宽而必然判停。
  // 默认关 (行业默认), 需要"更保守"时再打开。
  const bool sweep = config_.swept_geometry && config_.heading_aware &&
    config_.dynamic_geometry && config_.footprint.size() >= 3 &&
    (std::fabs(cmd.linear_x) > 1e-3 || std::fabs(cmd.angular_z) > 1e-3);
  int swept_hits = -1;
  int swept_slow_hits = -1;
  if (sweep) {
    const double long_margin = std::max(0.0, config_.stop_margin);
    const double lat_margin = std::max(0.0, config_.side_margin);
    swept_hits = countSweptHits(pts, config_.footprint, cmd.linear_x, cmd.angular_z,
                                config_.brake_decel, config_.heading_brake_decel,
                                config_.reaction_time, config_.sweep_samples,
                                config_.sweep_max_time, long_margin, lat_margin, 0.0);
    const double slow_extra = std::max(0.0, config_.slowdown_lookahead);
    swept_slow_hits = countSweptHits(pts, config_.footprint, cmd.linear_x, cmd.angular_z,
                                     config_.brake_decel, config_.heading_brake_decel,
                                     config_.reaction_time, config_.sweep_samples,
                                     config_.sweep_max_time + slow_extra, long_margin,
                                     lat_margin, slow_extra);
  }

  // nav2 VelocityPolygon 的语义: rotation 区只在"真的在转"时生效
  // (|v| 小 **且** |w| 明显).
  // ⚠ 2026-10-28: 以前**区选择**只看 |v|<0.05, 而半径分支才看 |w|, 于是
  // "命令为 0(停车/等待/规划中)"时也按旋转区判定 —— 侧后方的墙会命中
  // 旋转矩形并锁存 STOP, 2 s 后误判 blocked 触发 recovery, 最终 FAILED_SAFETY_STOP。
  // 现在两处用同一个 predicate。
  const bool rotating = std::fabs(cmd.linear_x) < 0.05 &&
                        std::fabs(cmd.angular_z) > 0.05;

  // 2) 按运动状态选 STOP 区: 原地旋转 -> 旋转区; 后退 -> 后退区; 否则前进区.
  const std::vector<std::pair<double, double>> * stop_poly = &config_.stop_polygon;
  if (rotating && !config_.stop_polygon_rotation.empty()) {
    stop_poly = &config_.stop_polygon_rotation;
  } else if (cmd.linear_x < 0.0 && !config_.stop_polygon_backward.empty()) {
    stop_poly = &config_.stop_polygon_backward;
  }
  std::vector<std::pair<double, double>> dynamic_stop;
  std::vector<std::pair<double, double>> dynamic_slow;
  if (config_.dynamic_geometry && !rotating && config_.footprint.size() >= 3) {
    buildDynamicStopPolygon(cmd, config_.footprint, config_.brake_decel,
                            config_.reaction_time, config_.stop_margin,
                            config_.side_margin, config_.turn_margin,
                            dynamic_stop);
    if (!dynamic_stop.empty()) stop_poly = &dynamic_stop;
    // nav2 的 slowdown 区就是"同一个多边形的更大版本": 车头多算
    // v*slowdown_lookahead 的行程, 侧向不变 (侧向不能靠时间放大, 否则窄口又算窄)
    buildDynamicStopPolygon(cmd, config_.footprint, config_.brake_decel,
                            config_.reaction_time, config_.stop_margin,
                            config_.side_margin, config_.turn_margin,
                            dynamic_slow, std::max(0.0, config_.slowdown_lookahead));
  }
  int stop_count = 0;
  double stop_radius = config_.stop_radius;
  if (rotating && config_.stop_radius_rotation > 0.0) {
    stop_radius = config_.stop_radius_rotation;   // 原地转: 用扫掠圆
  }
  if (rotating || stop_radius <= 0.0) {
    stop_count = countPointsInPolygon(pts, *stop_poly);
  }
  if (rotating && stop_radius > 0.0) {
    stop_count = 0;
    const double r2 = stop_radius * stop_radius;
    for (const auto & p : pts) {
      if (p.first * p.first + p.second * p.second <= r2) ++stop_count;
    }
  }
  if (swept_hits >= 0) {
    stop_count = swept_hits;   // 可选: 更严格的扫掠区
  }

  // 3) 滞回: 连续 trigger 帧才进入, 连续 release 帧才释放.
  const bool stop_hit = stop_count > config_.stop_max_points;
  const bool stop_now = updateHysteresis(
    stop_active_, stop_trigger_, stop_release_, stop_hit,
    config_.stop_trigger_frames, config_.stop_release_frames);
  if (stop_now) {
    last_scale_ = 0.0;
    result.command = VelocityCommand{};
    result.scale = 0.0;
    result.state = "STOP";
    return result;
  }

  // 4) SLOWDOWN 同样带滞回.
  // 动态几何模式下不再用二值 slowdown_ratio (1.0<->0.5 硬切), 改由 approachScale
  // 按 TTC 连续降速 + 下面的变化率限制; STOP 仍然立即生效.
  bool slow_hit;
  if (swept_slow_hits >= 0) {
    slow_hit = swept_slow_hits > config_.slowdown_max_points;
  } else if (!dynamic_slow.empty()) {
    slow_hit = countPointsInPolygon(pts, dynamic_slow) > config_.slowdown_max_points;
  } else {
    slow_hit = countPointsInPolygon(pts, config_.slowdown_polygon) > config_.slowdown_max_points;
  }
  const bool slow_now = updateHysteresis(
    slowdown_active_, slow_trigger_, slow_release_, slow_hit,
    config_.slowdown_trigger_frames, config_.slowdown_release_frames);

  double scale = 1.0;
  std::string state = "PASS";
  if (slow_now) {
    scale = std::min(scale, config_.slowdown_ratio);
    state = "SLOWDOWN";
  }

  if (config_.approach_enabled) {
    const double approach = approachScale(
      pts, cmd.linear_x, cmd.lateral_y, cmd.angular_z, config_.footprint,
      config_.time_before_collision, config_.simulation_time_step);
    if (approach < scale) {
      scale = approach;
      state = "APPROACH";
    }
  }

  if (scale <= 0.0) {
    last_scale_ = 0.0;
    result.command = VelocityCommand{};
    result.scale = 0.0;
    result.state = "STOP";
    return result;
  }

  // 减速比例变化率限制: 允许立即**降速**(安全优先), 只限制**回升**速度,
  // 避免障碍消失后一帧从 0.5 跳回 1.0 造成"窜一下"。STOP 不受此限制。
  if (config_.scale_slew_rate > 0.0) {
    const double dt = std::max(1e-3, config_.scale_dt);
    const double max_delta = config_.scale_slew_rate * dt;
    if (scale > last_scale_) {
      scale = std::min(scale, last_scale_ + max_delta);
    }
  }
  last_scale_ = scale;

  // 减速档不要把线速度压死: 控制器靠 v*e_y 做横向修正, v 太小只剩漂移
  if (scale > 0.0 && scale < 1.0 && config_.slowdown_min_speed > 0.0) {
    const double v_abs = std::fabs(cmd.linear_x);
    if (v_abs > 1e-3) {
      scale = std::max(scale, std::min(1.0, config_.slowdown_min_speed / v_abs));
    }
  }
  result.command.linear_x = cmd.linear_x * scale;
  result.command.lateral_y = cmd.lateral_y * scale;
  result.command.angular_z = cmd.angular_z * scale;
  result.scale = scale;
  result.state = state;
  return result;
}

}  // namespace core
}  // namespace rosiwit_navigation
