#include "planar_scan_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

#include <Eigen/Dense>

namespace rosiwit_slam
{
namespace
{

struct PoseEval
{
  double cost = std::numeric_limits<double>::max();  // 全体点的截断残差均方 (越小越好)
  double inlier_sq = 0.0;                            // 内点残差平方和
  int inliers = 0;
  Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
};

inline bool inGrid(int gx, int gy, const PlanarGrid & g)
{
  return gx >= 0 && gy >= 0 && gx < g.width && gy < g.height;
}

inline int cellIndex(const PlanarGrid & g, int gx, int gy) { return gy * g.width + gx; }

/// 双线性插值取距离场 (越界返回 cap).
double sampleField(const std::vector<float> & field, const PlanarGrid & g,
                   double x, double y, double cap)
{
  const double fx = (x - g.origin_x) / g.resolution;
  const double fy = (y - g.origin_y) / g.resolution;
  const int x0 = static_cast<int>(std::floor(fx));
  const int y0 = static_cast<int>(std::floor(fy));
  if (!inGrid(x0, y0, g) || !inGrid(x0 + 1, y0 + 1, g)) { return cap; }
  const double tx = fx - x0;
  const double ty = fy - y0;
  const double v00 = field[cellIndex(g, x0, y0)];
  const double v10 = field[cellIndex(g, x0 + 1, y0)];
  const double v01 = field[cellIndex(g, x0, y0 + 1)];
  const double v11 = field[cellIndex(g, x0 + 1, y0 + 1)];
  return (1 - ty) * ((1 - tx) * v00 + tx * v10) + ty * ((1 - tx) * v01 + tx * v11);
}

/// 打分 + 信息矩阵. 残差超过 inlier_dist 的点当外点 (只记 0 贡献).
PoseEval evaluate(const std::vector<PlanarPoint> & pts, double x, double y, double yaw,
                  const std::vector<float> & field, const PlanarGrid & g,
                  const PlanarMatcherConfig & cfg)
{
  PoseEval out;
  out.info.setZero();
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double cap = cfg.inlier_dist;
  double cost_sum = 0.0;
  double inlier_sq = 0.0;
  int n = 0;
  for (const auto & p : pts) {
    const double wx = x + c * p.x - s * p.y;
    const double wy = y + s * p.x + c * p.y;
    const double d = sampleField(field, g, wx, wy, cap + 1.0);
    // 外点也要计入代价: 否则"把点甩到地图外"会得到更小的均值, 搜索会主动丢点.
    const double dc = std::min(d, cap);
    cost_sum += dc * dc;
    if (d > cap) { continue; }
    inlier_sq += d * d;
    ++n;
    // 距离场梯度 ~ 该点处"约束方向"的单位法向.
    const double eps = g.resolution;
    const double gx = (sampleField(field, g, wx + eps, wy, cap) -
                       sampleField(field, g, wx - eps, wy, cap)) / (2.0 * eps);
    const double gy = (sampleField(field, g, wx, wy + eps, cap) -
                       sampleField(field, g, wx, wy - eps, cap)) / (2.0 * eps);
    const double nx = gx;
    const double ny = gy;
    // 平移: 法向贡献; 旋转: 力臂 r x n.
    const double rx = wx - x;
    const double ry = wy - y;
    const double torque = rx * ny - ry * nx;
    Eigen::Vector3d j(nx, ny, torque);
    out.info += j * j.transpose();
  }
  out.inliers = n;
  out.inlier_sq = inlier_sq;
  out.cost = (pts.empty() ? std::numeric_limits<double>::max()
                          : cost_sum / static_cast<double>(pts.size()));
  return out;
}

}  // namespace

std::vector<float> buildPlanarDistanceField(const PlanarGrid & grid, double cap_m)
{
  const size_t n = static_cast<size_t>(grid.width) * grid.height;
  std::vector<float> field(n, static_cast<float>(cap_m));
  if (!grid.ready()) { return field; }

  const int cap_cells = std::max(1, static_cast<int>(std::ceil(cap_m / grid.resolution)));
  std::vector<float> dist(n, static_cast<float>(cap_cells + 1));
  std::queue<std::pair<int, int>> q;
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const size_t i = static_cast<size_t>(y) * grid.width + x;
      if (grid.data[i] >= 100) {
        dist[i] = 0.0f;
        q.push({x, y});
      }
    }
  }
  const int dx4[4] = {-1, 0, 1, 0};
  const int dy4[4] = {0, -1, 0, 1};
  while (!q.empty()) {
    const std::pair<int, int> c = q.front();
    q.pop();
    const float cd = dist[static_cast<size_t>(c.second) * grid.width + c.first];
    if (cd >= static_cast<float>(cap_cells)) { continue; }
    for (int k = 0; k < 4; ++k) {
      const int nx = c.first + dx4[k];
      const int ny = c.second + dy4[k];
      if (!inGrid(nx, ny, grid)) { continue; }
      float & t = dist[static_cast<size_t>(ny) * grid.width + nx];
      if (cd + 1.0f < t) {
        t = cd + 1.0f;
        q.push({nx, ny});
      }
    }
  }
  for (size_t i = 0; i < n; ++i) {
    field[i] = std::min(static_cast<float>(cap_m), dist[i] * static_cast<float>(grid.resolution));
  }
  return field;
}

PlanarMatchResult matchPlanarScan(const std::vector<PlanarPoint> & pts_base,
                                  double init_x, double init_y, double init_yaw,
                                  const PlanarGrid & grid,
                                  const std::vector<float> & dist_field,
                                  const PlanarMatcherConfig & cfg)
{
  PlanarMatchResult res;
  res.points = static_cast<int>(pts_base.size());
  if (!grid.ready() || dist_field.size() != grid.data.size()) { return res; }
  if (res.points < cfg.min_points) { return res; }

  double bx = init_x;
  double by = init_y;
  double byaw = init_yaw;
  double best_score = std::numeric_limits<double>::max();
  int best_inliers = 0;

  double half_xy = std::max(0.0, cfg.search_xy);
  double half_yaw = std::max(0.0, cfg.search_yaw);
  int levels = std::max(1, cfg.levels);
  const double shrink = std::min(0.9, std::max(0.1, cfg.level_shrink));

  for (int level = 0; level < levels; ++level) {
    // 步长: 粗级约 13x13x9 个候选, 细级一直收到栅格的亚格精度.
    const double step_xy = std::max(grid.resolution / 8.0, half_xy / 6.0);
    const double step_yaw = std::max(0.002, half_yaw / 4.0);
    const int nx = static_cast<int>(std::ceil(half_xy / step_xy));
    const int nyaw = static_cast<int>(std::ceil(half_yaw / step_yaw));
    const double cx = bx;
    const double cy = by;
    const double cyaw = byaw;
    for (int i = -nx; i <= nx; ++i) {
      for (int j = -nx; j <= nx; ++j) {
        for (int k = -nyaw; k <= nyaw; ++k) {
          const double tx = cx + i * step_xy;
          const double ty = cy + j * step_xy;
          const double tyaw = cyaw + k * step_yaw;
          const PoseEval ev = evaluate(pts_base, tx, ty, tyaw, dist_field, grid, cfg);
          if (ev.cost < best_score) {
            best_score = ev.cost;
            bx = tx;
            by = ty;
            byaw = tyaw;
            best_inliers = ev.inliers;
          }
        }
      }
    }
    half_xy *= shrink;
    half_yaw *= shrink;
  }

  const PoseEval best = evaluate(pts_base, bx, by, byaw, dist_field, grid, cfg);
  res.x = bx;
  res.y = by;
  res.yaw = byaw;
  res.inliers = best.inliers;
  res.mean_residual = (best.inliers > 0)
    ? std::sqrt(std::max(0.0, best.inlier_sq / best.inliers)) : 0.0;
  res.inlier_ratio = (res.points > 0)
    ? static_cast<double>(best.inliers) / res.points : 0.0;
  res.valid = res.inliers >= cfg.min_points &&
              res.inlier_ratio >= cfg.min_inlier_ratio &&
              res.mean_residual <= cfg.max_residual;
  if (!res.valid) { return res; }

  // ---- 协方差: (H/σ_meas² + P_prior⁻¹)⁻¹ ----
  // 残差是**系统性**误差, 不能按 1/N 平均掉 (点越多 ≠ 绝对越准);
  // 未观测方向由先验兜底, 不会给出"零方差".
  const double quant = 0.25 * grid.resolution;
  const double meas_var = std::max(1e-6,
      res.mean_residual * res.mean_residual + quant * quant);
  const Eigen::Matrix3d Hn = best.info / meas_var;

  // 先验: 未观测方向至多和搜索窗口一样确定 (0.5 m / 0.2 rad).
  Eigen::Vector3d prior_var(0.25, 0.25, 0.04);
  Eigen::Matrix3d P = Hn;
  P.diagonal() += prior_var.cwiseInverse();
  const Eigen::Matrix3d cov = P.inverse();

  // 条件数用**原始**信息矩阵: 它才反映可观性 (先验会掩盖退化).
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(best.info);
  const Eigen::Vector3d ev = es.eigenvalues();
  const double lmax = std::max(0.0, ev.maxCoeff());
  const double lmin = std::max(0.0, ev.minCoeff());
  res.condition = (lmin > 1e-9 * std::max(1e-12, lmax)) ? (lmax / lmin) : 1.0e9;

  res.cov_xx = cov(0, 0);
  res.cov_xy = cov(0, 1);
  res.cov_yy = cov(1, 1);
  res.cov_yyaw = cov(2, 2);
  return res;
}

PlanarMatchResult matchPlanarScan(const std::vector<PlanarPoint> & pts_base,
                                  double init_x, double init_y, double init_yaw,
                                  const PlanarGrid & grid,
                                  const PlanarMatcherConfig & cfg)
{
  const std::vector<float> field = buildPlanarDistanceField(grid, 1.0);
  return matchPlanarScan(pts_base, init_x, init_y, init_yaw, grid, field, cfg);
}

}  // namespace rosiwit_slam
