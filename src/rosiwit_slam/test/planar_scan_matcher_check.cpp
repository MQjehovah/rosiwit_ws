// 平面扫描匹配离线验证 (不需要 ROS/PCL, 可单独 g++ 编译)
//
// 关键: "扫描"用**解析几何**生成 (精确的墙线), 不用栅格中心 —— 栅格中心有
// 最多半格 (2.5cm) 的量化偏差, 会把 matcher 的精度测试变成"测噪声".
#include <cmath>
#include <cstdio>
#include <vector>

#include "algorithms/localization/planar_scan_matcher.hpp"

using namespace rosiwit_slam;

namespace
{

constexpr double kRes = 0.05;
constexpr double kOx = -1.0;
constexpr double kOy = -1.0;
constexpr int kW = 280;   // 14.0 m: x in [-1, 13]
constexpr int kH = 240;   // 12.0 m: y in [-1, 11]

struct Segment { double x0, y0, x1, y1; };

PlanarGrid makeGrid(const std::vector<Segment> & segs)
{
  PlanarGrid g;
  g.width = kW; g.height = kH; g.resolution = kRes; g.origin_x = kOx; g.origin_y = kOy;
  g.data.assign(static_cast<size_t>(kW) * kH, 0);
  auto set = [&](double x, double y) {
    const int gx = static_cast<int>(std::floor((x - kOx) / kRes));
    const int gy = static_cast<int>(std::floor((y - kOy) / kRes));
    if (gx < 0 || gy < 0 || gx >= kW || gy >= kH) return;
    g.data[static_cast<size_t>(gy) * kW + gx] = 100;
  };
  for (const auto & s : segs) {
    const double len = std::hypot(s.x1 - s.x0, s.y1 - s.y0);
    const int n = std::max(1, static_cast<int>(len / (kRes * 0.5)));
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / n;
      set(s.x0 + (s.x1 - s.x0) * t, s.y0 + (s.y1 - s.y0) * t);
    }
  }
  return g;
}

std::vector<Segment> roomSegments()
{
  return {
    {0.0, 0.0, 12.0, 0.0},
    {12.0, 0.0, 12.0, 10.0},
    {12.0, 10.0, 0.0, 10.0},
    {0.0, 10.0, 0.0, 0.0},
    {5.0, 4.0, 5.6, 4.0},   // 柱子的四条边 (非对称特征)
    {5.6, 4.0, 5.6, 4.6},
    {5.6, 4.6, 5.0, 4.6},
    {5.0, 4.6, 5.0, 4.0},
    {8.0, 6.0, 8.0, 8.0},   // 隔断
    {8.0, 8.0, 9.5, 8.0},
  };
}

std::vector<Segment> corridorSegments()
{
  return {
    {0.5, 4.4, 11.5, 4.4},
    {0.5, 5.6, 11.5, 5.6},
  };
}

/// 从解析线段生成扫描 (不判可见性: 点都真实落在墙上, 位姿是一致的).
std::vector<PlanarPoint> synthScan(const std::vector<Segment> & segs,
                                   double x, double y, double yaw, double max_range)
{
  std::vector<PlanarPoint> pts;
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (const auto & sg : segs) {
    const double len = std::hypot(sg.x1 - sg.x0, sg.y1 - sg.y0);
    const int n = std::max(1, static_cast<int>(len / 0.10));
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / n;
      const double wx = sg.x0 + (sg.x1 - sg.x0) * t;
      const double wy = sg.y0 + (sg.y1 - sg.y0) * t;
      const double dx = wx - x, dy = wy - y;
      if (dx * dx + dy * dy > max_range * max_range) continue;
      pts.push_back({c * dx + s * dy, -s * dx + c * dy});
    }
  }
  return pts;
}

double normAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

int fails = 0;
void check(bool ok, const char * what)
{
  std::printf("  %s %s\n", ok ? "[ok]" : "[FAIL]", what);
  if (!ok) ++fails;
}

}  // namespace

int main()
{
  // ---------- 1) 一般场景: 0.15m/0.06rad 偏差能收敛 ----------
  {
    const auto segs = roomSegments();
    const PlanarGrid room = makeGrid(segs);
    const double tx = 3.0, ty = 2.5, tyaw = 0.40;
    const auto scan = synthScan(segs, tx, ty, tyaw, 5.0);
    PlanarMatcherConfig cfg;
    {
      // 数据一致性自检: 在真值位姿下, 每个扫描点到最近占据格的距离应 << inlier_dist.
      int occ = 0;
      for (auto v : room.data) { if (v >= 100) ++occ; }
      double worst = 0.0, sum = 0.0;
      const double c = std::cos(tyaw), s = std::sin(tyaw);
      for (const auto & p : scan) {
        const double wx = tx + c * p.x - s * p.y;
        const double wy = ty + s * p.x + c * p.y;
        const int gx = static_cast<int>(std::floor((wx - room.origin_x) / room.resolution));
        const int gy = static_cast<int>(std::floor((wy - room.origin_y) / room.resolution));
        double best = 1e9;
        for (int dy = -3; dy <= 3; ++dy) {
          for (int dx = -3; dx <= 3; ++dx) {
            const int nx = gx + dx, ny = gy + dy;
            if (nx < 0 || ny < 0 || nx >= room.width || ny >= room.height) continue;
            if (room.data[static_cast<size_t>(ny) * room.width + nx] < 100) continue;
            const double ex = room.origin_x + (nx + 0.5) * room.resolution - wx;
            const double ey = room.origin_y + (ny + 0.5) * room.resolution - wy;
            best = std::min(best, std::hypot(ex, ey));
          }
        }
        worst = std::max(worst, best);
        sum += best;
      }
      std::printf("0) data check: occ_cells=%d pts=%zu mean_dist=%.3f worst=%.3f\n",
                  occ, scan.size(), sum / std::max<size_t>(1, scan.size()), worst);

      // 用库里的距离场做同样的双线性查表, 统计"会被判为外点"的点数.
      const auto field = buildPlanarDistanceField(room, 1.0);
      int far = 0;
      double fsum = 0.0;
      for (const auto & p : scan) {
        const double wx = tx + c * p.x - s * p.y;
        const double wy = ty + s * p.x + c * p.y;
        const double fx = (wx - room.origin_x) / room.resolution;
        const double fy = (wy - room.origin_y) / room.resolution;
        const int x0 = static_cast<int>(std::floor(fx));
        const int y0 = static_cast<int>(std::floor(fy));
        if (x0 < 0 || y0 < 0 || x0 + 1 >= room.width || y0 + 1 >= room.height) {
          ++far;
          continue;
        }
        const double ax = fx - x0, ay = fy - y0;
        auto at = [&](int gx, int gy) {
          return static_cast<double>(field[static_cast<size_t>(gy) * room.width + gx]);
        };
        const double v = (1 - ay) * ((1 - ax) * at(x0, y0) + ax * at(x0 + 1, y0)) +
                         ay * ((1 - ax) * at(x0, y0 + 1) + ax * at(x0 + 1, y0 + 1));
        fsum += v;
        if (v > 0.12) ++far;
      }
      std::printf("   field bilinear: mean=%.3f far(>0.12)=%d/%zu\n",
                  fsum / std::max<size_t>(1, scan.size()), far, scan.size());
    }
    const auto r0 = matchPlanarScan(scan, tx, ty, tyaw, room, cfg);
    std::printf("0) at truth: pts=%d inlier=%.2f resid=%.3f err=(%.3f,%.3f,%.3f)\n",
                r0.points, r0.inlier_ratio, r0.mean_residual,
                r0.x - tx, r0.y - ty, normAngle(r0.yaw - tyaw));
    const auto r = matchPlanarScan(scan, tx + 0.15, ty - 0.12, tyaw + 0.06, room, cfg);
    const double ex = std::hypot(r.x - tx, r.y - ty);
    std::printf("1) room: pts=%d inlier=%.2f resid=%.3f valid=%d "
                "err=(%.3f,%.3f,%.3f) sig=(%.3f,%.3f,%.3f) cond=%.1f\n",
                r.points, r.inlier_ratio, r.mean_residual, static_cast<int>(r.valid),
                r.x - tx, r.y - ty, normAngle(r.yaw - tyaw),
                std::sqrt(r.cov_xx), std::sqrt(r.cov_yy),
                std::sqrt(r.cov_yyaw), r.condition);
    check(r.valid, "match valid");
    check(ex < 0.02, "xy error < 2 cm");
    check(std::fabs(normAngle(r.yaw - tyaw)) < 0.01, "yaw error < 0.01 rad");
    check(std::sqrt(r.cov_xx) < 0.05 && std::sqrt(r.cov_yy) < 0.05,
          "well-conditioned covariance small");
  }

  // ---------- 2) 长走廊: 沿走廊方向退化, 协方差被放大 ----------
  {
    const auto segs = corridorSegments();
    const PlanarGrid corr = makeGrid(segs);
    const double tx = 6.0, ty = 5.0, tyaw = 0.0;
    const auto scan = synthScan(segs, tx, ty, tyaw, 5.0);
    PlanarMatcherConfig cfg;
    const auto r = matchPlanarScan(scan, tx + 0.30, ty, tyaw, corr, cfg);
    const double sx = std::sqrt(r.cov_xx), sy = std::sqrt(r.cov_yy);
    std::printf("2) corridor: valid=%d x=%.3f (true %.3f) y=%.3f "
                "sig=(%.3f,%.3f) sigma_x/sigma_y=%.1f cond=%.0f\n",
                static_cast<int>(r.valid), r.x, tx, r.y, sx, sy,
                sx / std::max(1e-9, sy), r.condition);
    check(r.valid, "match valid");
    check(std::fabs(r.y - ty) < 0.05, "cross-corridor y accurate (< 5 cm)");
    check(sx > 3.0 * sy, "degenerate axis covariance inflated (sigma_x >> sigma_y)");
    check(sx > 0.15, "along-corridor sigma not fake-precise (> 15 cm)");
  }

  std::printf("=== %s ===\n", fails == 0 ? "PASS" : "FAIL");
  return fails == 0 ? 0 : 1;
}
