// ============================================================
// 分层代价地图的层接口
//
// 设计目标: 规划器 / 控制器 / 安全层看到的是**同一批障碍层**,
// 只是参与合成的层子集与膨胀参数不同 (三个"视图"):
//   planner    : static + keepout            (自身做 footprint 膨胀)
//   controller : static + scan + tracked + voxel + keepout (不膨胀, 由 footprint 负责)
//   safety     : 直接用原始激光点 (不下采样到栅格)
//
// 栅格取值 (严格按 nav_msgs/OccupancyGrid 约定, 8 位有符号):
//   -1        unknown: 阻断规划, 但不作为膨胀源
//    0        free
//    1..98    膨胀代价梯度
//    99       inscribed (安全环, 与历史可视化一致)
//   100       lethal / occupied (膨胀源)
//
// 这样层内部、CostmapGrid、core::Costmap、OccupancyGrid、planner 内部数组
// 全部是同一套数值, 任何位置都不需要转换.
// ============================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE_COSTMAP_LAYER_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE_COSTMAP_LAYER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace rosiwit_navigation
{
namespace core
{

constexpr int8_t kCostUnknown = -1;
constexpr int8_t kCostFree = 0;
constexpr int8_t kCostInscribed = 99;
constexpr int8_t kCostLethal = 100;

/// 一个视图的几何: 整图原点/分辨率 + 视图窗口在整图中的偏移.
struct LayerConfig
{
  unsigned int width = 0;
  unsigned int height = 0;
  double resolution = 0.05;
  double map_origin_x = 0.0;   // 整图 (map frame) 原点
  double map_origin_y = 0.0;
  int window_x = 0;            // 视图左上角对应的整图栅格列
  int window_y = 0;

  bool valid() const { return width > 0 && height > 0 && resolution > 0.0; }
};

class ICostmapLayer
{
public:
  virtual ~ICostmapLayer() = default;

  virtual std::string name() const = 0;

  /// 视图几何变化时调用 (分辨率/窗口), 层可以据此预计算.
  virtual bool configure(const LayerConfig & cfg)
  {
    cfg_ = cfg;
    return true;
  }

  /// 采样频率由外层决定; 层内部可自行节流. now 为 ROS 秒.
  virtual void update(double now) { (void)now; }

  /// 把本层障碍写入 grid. 几何由调用方显式传入:
  /// 同一个层可以被多个视图共用而不串味.
  virtual void stamp(const LayerConfig & cfg, std::vector<int8_t> & grid) const = 0;

  const LayerConfig & config() const { return cfg_; }

protected:
  /// 世界坐标 -> 视图内栅格索引; 不在视图内返回 false.
  static bool worldToIndex(const LayerConfig & cfg, double wx, double wy, int & gx, int & gy);

  LayerConfig cfg_;
};

/// 4 邻域距离变换 + 梯度代价, 取值仍是 -1/0..100 (与层一致).
/// raw 中的 kCostLethal(100) 为膨胀源; kCostUnknown(-1) 原样保留且不产生 halo.
std::vector<int8_t> inflatedCostmap(
  const std::vector<int8_t> & raw,
  unsigned int width, unsigned int height,
  int inscribed_cells, int inflate_cells);

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE_COSTMAP_LAYER_HPP_
