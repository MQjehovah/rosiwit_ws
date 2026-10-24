#include "trajectory_parameterizer.hpp"

#include <algorithm>
#include <cmath>

namespace rosiwit_navigation
{
namespace planners
{

namespace
{
double normAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
double absAngle(double a) { return std::fabs(normAngle(a)); }
}  // namespace

core::TimedTrajectory parameterizeTrajectory(const core::Path & path,
                                             const core::TrajectoryLimits & limits)
{
  core::TimedTrajectory traj;
  const size_t n = path.points.size();
  if (n < 2) return traj;
  traj.points.resize(n);

  // 1) 弧长
  double s = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const auto & q = path.points[i];
    if (i > 0) {
      const auto & p = path.points[i - 1].pose;
      s += std::hypot(q.pose.x - p.x, q.pose.y - p.y);
    }
    traj.points[i].pose = q.pose;
    traj.points[i].s = s;
  }

  // 2) 速度上限: 曲率限速 + 旋转段 v=0
  std::vector<double> vmax(n, limits.max_v);
  for (size_t i = 0; i < n; ++i) {
    if (std::fabs(path.points[i].velocity_x) < 0.1) { vmax[i] = 0.0; continue; }
    double kappa = 0.0;
    if (i > 0 && i + 1 < n) {
      const auto & a = path.points[i - 1].pose;
      const auto & b = path.points[i].pose;
      const auto & c = path.points[i + 1].pose;
      const double ab = std::hypot(b.x - a.x, b.y - a.y);
      const double bc = std::hypot(c.x - b.x, c.y - b.y);
      const double ca = std::hypot(a.x - c.x, a.y - c.y);
      const double area2 = std::fabs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
      if (ab > 1e-6 && bc > 1e-6 && ca > 1e-6) kappa = 2.0 * area2 / (ab * bc * ca);
    }
    const double v_curve = (kappa > 1e-6)
        ? std::sqrt(std::max(0.01, limits.max_lat_accel) / kappa)
        : limits.max_v;
    vmax[i] = std::min(limits.max_v, v_curve);
  }

  // 3) 双向加速度积分 + cusp 归零
  std::vector<double> v = vmax;
  for (size_t i = 1; i < n; ++i) {
    const double a = path.points[i - 1].velocity_x;
    const double b = path.points[i].velocity_x;
    if (std::fabs(a) > 0.1 && std::fabs(b) > 0.1 && (a > 0.0) != (b > 0.0)) v[i] = 0.0;
  }
  for (size_t i = 1; i < n; ++i) {
    const double ds = traj.points[i].s - traj.points[i - 1].s;
    if (ds < 1e-6) { v[i] = 0.0; continue; }
    const double reach = std::sqrt(v[i - 1] * v[i - 1] + 2.0 * limits.max_accel * ds);
    v[i] = std::min(v[i], reach);
  }
  for (size_t i = n - 1; i-- > 0;) {
    const double ds = traj.points[i + 1].s - traj.points[i].s;
    if (ds < 1e-6) { v[i] = 0.0; continue; }
    const double reach = std::sqrt(v[i + 1] * v[i + 1] + 2.0 * limits.max_accel * ds);
    v[i] = std::min(v[i], reach);
  }
  v[n - 1] = 0.0;

  // 4) 时间 + 参考速度/角速度
  double t = 0.0;
  for (size_t i = 0; i < n; ++i) {
    auto & pt = traj.points[i];
    if (i > 0) {
      const double ds = pt.s - traj.points[i - 1].s;
      const double dtheta = absAngle(pt.pose.theta - traj.points[i - 1].pose.theta);
      if (ds > 1e-6) {
        const double v_avg = std::max(0.02, 0.5 * (v[i] + v[i - 1]));
        t += ds / v_avg;
      } else if (dtheta > 1e-4) {
        const double w = std::max(0.05, std::min(limits.max_w,
            std::sqrt(std::max(0.01, 2.0 * limits.max_alpha * dtheta))));
        t += dtheta / w;
      }
    }
    pt.t = t;
    const double vx = path.points[i].velocity_x;
    pt.v = (std::fabs(vx) < 0.1) ? 0.0 : ((vx < 0.0 ? -1.0 : 1.0) * v[i]);
    if (i > 0) {
      const double dtheta = normAngle(pt.pose.theta - traj.points[i - 1].pose.theta);
      const double dt = pt.t - traj.points[i - 1].t;
      pt.w = (dt > 1e-6)
          ? std::max(-limits.max_w, std::min(limits.max_w, dtheta / dt)) : 0.0;
    }
  }
  traj.duration = traj.points.back().t;
  traj.valid = traj.duration > 1e-6;
  return traj;
}

}  // namespace planners
}  // namespace rosiwit_navigation
