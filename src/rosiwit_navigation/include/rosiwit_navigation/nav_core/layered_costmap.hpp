// ============================================================
// 分层代价地图: 层列表 + 几何 + 合成 (max)
//
// 视图 = 层子集的合成结果. 同一批层可以给 planner / controller 出
// 不同视图, 避免"每个消费者各自维护一份障碍"的漂移.
// ============================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE_LAYERED_COSTMAP_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE_LAYERED_COSTMAP_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosiwit_navigation/nav_core/costmap_layer.hpp"

namespace rosiwit_navigation
{
namespace core
{

class LayeredCostmap
{
public:
  /// default_value: 视图初始值. 窗口超出地图时保持该值 (局部视图用
  /// kCostLethal, 保证窗口外不可穿越; 规划视图用 kCostFree).
  void configure(const LayerConfig & cfg, int8_t default_value = kCostFree);

  bool addLayer(const std::shared_ptr<ICostmapLayer> & layer);
  void clearLayers();
  bool hasLayer(const std::string & name) const;

  /// 依次 update 各层, 然后合成到 master_.
  void update(double now);

  /// 全部层合成 (max).
  const std::vector<int8_t> & combined() const { return master_; }

  /// 指定层子集合成 (不改动 master_).
  std::vector<int8_t> combinedOf(const std::vector<std::string> & names) const;

  const LayerConfig & config() const { return cfg_; }
  std::vector<std::string> layerNames() const;

private:
  void resetMaster();

  LayerConfig cfg_;
  int8_t default_value_ = kCostFree;
  std::vector<std::shared_ptr<ICostmapLayer>> layers_;
  std::vector<int8_t> master_;
};

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE_LAYERED_COSTMAP_HPP_
