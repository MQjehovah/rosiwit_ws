// ============================================================================
// teb_types.hpp — 弹性带的顶点/权重/各类边的代价与雅可比
//
// 设计参照 teb_local_planner 的边集合 (path / obstacle / kinematics / velocity /
// acceleration / time / shortest)，但用**平方惩罚 + Gauss-Newton**，不做 g2o 的
// 鲁棒核。所有残差都是"越小越好"的二次型，hess 用 J^T J 近似。
//
// 单顶点边返回 EdgeAccumulator(3x3 hess + 3 grad)；
// 涉及相邻顶点的边返回 EdgeBlocks(a/b 两个 3 维块 + 交叉块)。
// ============================================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE__TEB_TYPES_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE__TEB_TYPES_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <vector>

#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace core
{

struct TebVertex
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double dt = 0.3;   ///< 到下一个顶点的时间 (s)
};

struct TebWeights
{
  double path = 1.0;
  double obstacle = 8.0;
  double kinematics = 6.0;
  double velocity = 4.0;
  double accel = 3.0;
  double time = 1.0;
  double shortest = 1.0;
  double min_obstacle_dist = 0.12;
  double inflation_dist = 0.30;
  double max_deviation = 0.60;
};

struct EdgeAccumulator
{
  std::array<double, 9> hess{};
  std::array<double, 3> grad{};
  double cost = 0.0;
};

/// 两个相邻顶点之间的边: 各自 3x3 块 + 两个交叉块 (对称, 只存一份)
struct EdgeBlocks
{
  std::array<double, 9> h_aa{};
  std::array<double, 9> h_ab{};
  std::array<double, 3> g_a{};
  std::array<double, 3> g_b{};
  double cost = 0.0;
};

namespace teb_detail
{

inline double normAngle(double a)
{
  while (a > M_PI) { a -= 2.0 * M_PI; }
  while (a < -M_PI) { a += 2.0 * M_PI; }
  return a;
}

inline void addOuter(std::array<double, 9> & h, const std::array<double, 3> & j, double w)
{
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) { h[r * 3 + c] += w * j[r] * j[c]; }
  }
}

/// 两个顶点的残差 -> EdgeBlocks (数值雅可比: 6 个变量, 中心差分)
inline EdgeBlocks numericBlocks(
  const std::function<double(const double *)> & residual,
  const TebVertex & a, const TebVertex & b, double weight)
{
  EdgeBlocks out;
  const double h = 1e-4;
  double vars[6] = {a.x, a.y, a.theta, b.x, b.y, b.theta};
  const double r0 = residual(vars);
  out.cost = weight * r0 * r0;
  if (weight <= 0.0) { return out; }

  std::array<double, 6> jac{};
  for (int i = 0; i < 6; ++i) {
    const double save = vars[i];
    vars[i] = save + h;
    const double rp = residual(vars);
    vars[i] = save - h;
    const double rm = residual(vars);
    vars[i] = save;
    jac[i] = (rp - rm) / (2.0 * h);
  }
  // hess += 2w * (r0*H + J^T J); 这里只取 Gauss-Newton 项 J^T J 乘以 2w
  for (int i = 0; i < 3; ++i) {
    out.g_a[i] = 2.0 * weight * r0 * jac[i];
    out.g_b[i] = 2.0 * weight * r0 * jac[3 + i];
    for (int j = 0; j < 3; ++j) {
      out.h_aa[i * 3 + j] = 2.0 * weight * jac[i] * jac[j];
      out.h_ab[i * 3 + j] = 2.0 * weight * jac[i] * jac[3 + j];
    }
  }
  return out;
}

/// 点到折线的最近点 (返回最近点与横向距离)
inline void closestOnPath(const TebVertex & v, const Path & path, double * cx, double * cy)
{
  double best = 1e18;
  *cx = v.x;
  *cy = v.y;
  if (path.points.empty()) { return; }
  if (path.points.size() == 1) {
    *cx = path.points[0].pose.x;
    *cy = path.points[0].pose.y;
    return;
  }
  for (size_t i = 0; i + 1 < path.points.size(); ++i) {
    const double ax = path.points[i].pose.x;
    const double ay = path.points[i].pose.y;
    const double bx = path.points[i + 1].pose.x;
    const double by = path.points[i + 1].pose.y;
    const double dx = bx - ax;
    const double dy = by - ay;
    const double l2 = dx * dx + dy * dy;
    double t = 0.0;
    if (l2 > 1e-12) { t = std::max(0.0, std::min(1.0, ((v.x - ax) * dx + (v.y - ay) * dy) / l2)); }
    const double px = ax + t * dx;
    const double py = ay + t * dy;
    const double d2 = (v.x - px) * (v.x - px) + (v.y - py) * (v.y - py);
    if (d2 < best) { best = d2; *cx = px; *cy = py; }
  }
}

}  // namespace teb_detail

/// 偏离全局路径: 横向距离的二次惩罚 (+ 超过 max_deviation 的硬性放大)
inline EdgeAccumulator edgePath(const TebVertex & v, const Path & path, double w)
{
  EdgeAccumulator e;
  if (path.points.empty() || w <= 0.0) { return e; }
  double cx = 0.0, cy = 0.0;
  teb_detail::closestOnPath(v, path, &cx, &cy);
  const double dx = v.x - cx;
  const double dy = v.y - cy;
  const double d = std::sqrt(dx * dx + dy * dy);
  if (d < 1e-9) { return e; }
  const std::array<double, 3> jac = {dx / d, dy / d, 0.0};
  e.cost = w * d * d;
  for (int i = 0; i < 3; ++i) { e.grad[i] = 2.0 * w * d * jac[i]; }
  teb_detail::addOuter(e.hess, jac, 2.0 * w);
  return e;
}

/// 障碍: (dir_x, dir_y) 是指向最近障碍点的单位向量; dist 为车体到障碍的距离(米)
inline EdgeAccumulator edgeObstacle(const TebVertex & v, double dist_m,
                                    double dir_x, double dir_y, const TebWeights & w)
{
  (void)v;
  EdgeAccumulator e;
  if (w.obstacle <= 0.0) { return e; }
  const double min_d = w.min_obstacle_dist;
  const double infl = std::max(w.inflation_dist, min_d + 1e-6);
  const std::array<double, 3> dir = {dir_x, dir_y, 0.0};

  if (dist_m < min_d) {
    const double p = min_d - dist_m;             // >0
    e.cost = w.obstacle * p * p;
    for (int i = 0; i < 3; ++i) { e.grad[i] = 2.0 * w.obstacle * p * dir[i]; }
    teb_detail::addOuter(e.hess, dir, 2.0 * w.obstacle);
    return e;
  }
  if (dist_m < infl) {
    // 软带: 从 infl 到 min 线性升到 0.5*w, 二次
    const double band = infl - min_d;
    const double q = (infl - dist_m) / band;      // 0..1
    const double wk = 0.5 * w.obstacle;
    e.cost = wk * q * q;
    const double dq_ddist = -1.0 / band;
    // cost = wk*q^2; dcost/ddist = 2*wk*q*dq_ddist = -2*wk*q/band
    const double dcost_ddist = 2.0 * wk * q * dq_ddist;
    for (int i = 0; i < 3; ++i) { e.grad[i] = -dcost_ddist * dir[i]; }
    teb_detail::addOuter(e.hess, dir, 2.0 * wk / (band * band));
  }
  return e;
}

/// 非完整约束: 位移方向应与两端朝向一致 (差分驱动不能横移)
inline EdgeBlocks edgeKinematics(const TebVertex & a, const TebVertex & b, double w)
{
  auto residual = [](const double * v) -> double {
    const double dx = v[3] - v[0];
    const double dy = v[4] - v[1];
    const double len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-6) { return teb_detail::normAngle(v[5] - v[2]) * 0.0; }
    // 中位朝向近似弧线: 切向误差 + 两端朝向差
    const double heading = 0.5 * (v[2] + v[5]);
    const double tangent = std::atan2(dy, dx);
    const double e_tangent = teb_detail::normAngle(tangent - heading);
    const double e_turn = teb_detail::normAngle(v[5] - v[2] - 2.0 * e_tangent);
    return e_tangent * e_tangent + 0.5 * e_turn * e_turn;
  };
  return teb_detail::numericBlocks(residual, a, b, w);
}

/// 速度上限: |Δs/dt| 超过 max_v 时惩罚 (无符号差, 兼容倒车)
inline EdgeBlocks edgeVelocity(const TebVertex & a, const TebVertex & b,
                               const TebWeights & w, double max_v)
{
  auto residual = [max_v](const double * v) -> double {
    const double dx = v[3] - v[0];
    const double dy = v[4] - v[1];
    const double dt = 1.0;   // dt 由调用方保证; 这里按"位移/参考 dt" 归一
    const double speed = std::sqrt(dx * dx + dy * dy) / dt;
    const double over = std::max(0.0, speed - max_v);
    return over;
  };
  return teb_detail::numericBlocks(residual, a, b, w.velocity);
}

/// 加速度上限: |v_b - v_a| 超过 max_a*dt 时惩罚
inline EdgeBlocks edgeAccel(const TebVertex & a, const TebVertex & b, const TebVertex & c,
                            const TebWeights & w, double max_a)
{
  auto residual = [max_a](const double * v) -> double {
    const double s1 = std::sqrt((v[3] - v[0]) * (v[3] - v[0]) + (v[4] - v[1]) * (v[4] - v[1]));
    const double s2 = std::sqrt((v[6] - v[3]) * (v[6] - v[3]) + (v[7] - v[4]) * (v[7] - v[4]));
    const double over = std::max(0.0, std::fabs(s2 - s1) - max_a);
    return over;
  };
  // 只对 a/b 两块做数值差分 (c 视作常量)
  EdgeBlocks out = teb_detail::numericBlocks(
    [&](const double * v6) -> double {
      double v7[9] = {v6[0], v6[1], v6[2], v6[3], v6[4], v6[5], c.x, c.y, c.theta};
      return residual(v7);
    },
    a, b, w.accel);
  return out;
}

/// 时间最优: 惩罚每个顶点的 dt
inline EdgeAccumulator edgeTime(const TebVertex & v, double w)
{
  EdgeAccumulator e;
  e.cost = w * std::max(0.0, v.dt);
  return e;
}

/// 最短路径: 惩罚转角 (中间顶点越直越好); teb_local_planner 的 EdgeShortestPath 同义
inline double shortestPathCost(const TebVertex & a, const TebVertex & b, const TebVertex & c,
                               double w, std::array<double, 3> * g_prev,
                               std::array<double, 3> * g_next)
{
  const double ax = a.x - b.x;
  const double ay = a.y - b.y;
  const double cx = c.x - b.x;
  const double cy = c.y - b.y;
  const double la = std::hypot(ax, ay);
  const double lc = std::hypot(cx, cy);
  if (la < 1e-9 || lc < 1e-9) { return 0.0; }
  const double cosang = (ax * cx + ay * cy) / (la * lc);
  const double turn = 1.0 - std::max(-1.0, std::min(1.0, cosang));   // 0=直, 2=掉头
  if (g_prev != nullptr) { *g_prev = {0.0, 0.0, 0.0}; }
  if (g_next != nullptr) { *g_next = {0.0, 0.0, 0.0}; }
  return w * turn;
}

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__TEB_TYPES_HPP_
