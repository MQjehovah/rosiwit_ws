// ============================================================
// 平面 (2D 激光) 扫描匹配
//
// 定位里 3D 点云在长走廊/窄通道上沿墙方向几乎不可观测, 3D 匹配会把这个方向的
// 误差"自由漂移"掉. 2D 激光提供同一平面的几何约束, 与地图做粗到精的穷举匹配,
// 输出:
//   - 修正后的位姿 (x, y, yaw)
//   - 信息矩阵 H (3x3): 由每点的距离场梯度累加, **天然**反映可观性 ——
//     长走廊里缺失方向的 H 很小 -> 协方差大, 而不是像固定 sigma 那样假装精确
//   - 内点率/平均残差/条件数 (供健康等级判定)
//
// 只依赖 Eigen, 不依赖 ROS/PCL (便于离线验证).
// ============================================================
#ifndef ROSIWIT_SLAM__ALGORITHMS__LOCALIZATION__PLANAR_SCAN_MATCHER_HPP_
#define ROSIWIT_SLAM__ALGORITHMS__LOCALIZATION__PLANAR_SCAN_MATCHER_HPP_

#include <cstdint>
#include <vector>

namespace rosiwit_slam
{

struct PlanarPoint
{
  double x = 0.0;
  double y = 0.0;
};

/// 2D 占据栅格 (map 系). data: >=100 障碍, <0 unknown, 其它 free.
struct PlanarGrid
{
  int width = 0;
  int height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<int8_t> data;

  bool ready() const
  {
    return width > 0 && height > 0 && resolution > 0.0 &&
           data.size() == static_cast<size_t>(width) * height;
  }
};

struct PlanarMatcherConfig
{
  // 初始搜索窗口 (以传入初值为中心)
  double search_xy = 0.30;        // m
  double search_yaw = 0.15;       // rad
  // 三级细化: 每一级在上一次最优附近缩小窗口
  int levels = 3;
  double level_shrink = 0.4;      // 每级窗口 / 步长缩放
  // 打分
  double inlier_dist = 0.12;      // 点到障碍的截断距离 (m)
  double min_inlier_ratio = 0.55; // 低于此认为匹配失败
  int min_points = 40;
  double max_residual = 0.06;     // 平均残差上限 (m)
};

struct PlanarMatchResult
{
  bool valid = false;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  int points = 0;
  int inliers = 0;
  double inlier_ratio = 0.0;
  double mean_residual = 0.0;   // 内点平均点-障碍距离 (m)
  double condition = 0.0;       // 信息矩阵条件数 (越大越退化)
  // 协方差近似 (由信息矩阵逆 + 残差推出)
  double cov_xx = 0.0;
  double cov_yy = 0.0;
  double cov_yyaw = 0.0;
  double cov_xy = 0.0;
};

/// 构建截断距离场 (米). 障碍格为 0, 其余为到最近障碍的 4 邻域距离;
/// 超过 cap_m 的格子填 cap_m.
std::vector<float> buildPlanarDistanceField(const PlanarGrid & grid, double cap_m = 1.0);

/// 匹配. pts_base 是 base_link 系下的 2D 点 (x 前, y 左).
PlanarMatchResult matchPlanarScan(const std::vector<PlanarPoint> & pts_base,
                                  double init_x, double init_y, double init_yaw,
                                  const PlanarGrid & grid,
                                  const std::vector<float> & dist_field,
                                  const PlanarMatcherConfig & cfg);

/// 便捷重载: 内部构建距离场 (调用方自行缓存以省时间).
PlanarMatchResult matchPlanarScan(const std::vector<PlanarPoint> & pts_base,
                                  double init_x, double init_y, double init_yaw,
                                  const PlanarGrid & grid,
                                  const PlanarMatcherConfig & cfg);

}  // namespace rosiwit_slam

#endif  // ROSIWIT_SLAM__ALGORITHMS__LOCALIZATION__PLANAR_SCAN_MATCHER_HPP_
