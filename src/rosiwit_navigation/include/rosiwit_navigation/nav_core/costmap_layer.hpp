// ============================================================
// 分层代价地图的层接口
//
// 设计目标: 规划器 / 控制器 / 安全层看到的是**同一批障碍层**,
// 只是参与合成的层子集与膨胀参数不同 (三个"视图"):
//   planner    : static + keepout            (自身做 footprint 膨胀)
//   controller : static + scan + tracked + voxel + keepout (不膨胀, 由 footprint 负责)
//   safety     : 直接用原始激光点 (不下采样到栅格)
//
// 栅格取值约定 (与既有 planner 兼容, 不要改):
//   0        自由
//   1..99    膨胀代价 (仅供可视化/控制器; planner 自己重算)
//   100      lethal (障碍源, 参与膨胀)
//   254      unknown (阻断搜索, 不作为膨胀源) —— 只在 planner 视图出现
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

constexpr int16_t kCostFree = 0;
constexpr int16_t kCostLethal = 100;
constexpr int16_t kCostUnknown = 254;

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

  /// 把本层障碍写入 grid (只升不降, max 合成). 越界/无数据直接跳过.
  /// 几何由调用方显式传入: 同一个层可以被多个视图共用而不串味.
  virtual void stamp(const LayerConfig & cfg, std::vector<int16_t> & grid) const = 0;

  const LayerConfig & config() const { return cfg_; }

protected:
  /// 世界坐标 -> 视图内栅格索引; 不在视图内返回 false.
  static bool worldToIndex(const LayerConfig & cfg, double wx, double wy, int & gx, int & gy);

  LayerConfig cfg_;
};

/// 4 邻域距离变换 + 梯度代价, 与历史实现逐格一致.
/// raw 中的 kCostLethal 为膨胀源; kCostUnknown 原样保留且不产生 halo.
/// 输出: 0 / 1..99 / 100 / 254.
std::vector<int16_t> inflatedCostmap(
  const std::vector<int16_t> & raw,
  unsigned int width, unsigned int height,
  int inscribed_cells, int inflate_cells);

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE_COSTMAP_LAYER_HPP_
