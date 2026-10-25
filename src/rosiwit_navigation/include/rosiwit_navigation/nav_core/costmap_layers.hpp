// ============================================================
// 具体代价地图层
//
//   StaticLayer       /map 栅格 (可配置 unknown 是否阻断)
//   PointsLayer       map 系点障碍 (激光 / 体素投影), 单格或圆盘
//   CirclesLayer      带半径的圆形障碍 (跟踪预测目标)
//   KeepoutLayer      禁行区多边形 (来自 /virtual_obstacles), 可热更新
//   SpeedLimitLayer   分区限速 (纯查询, 不写栅格)
// ============================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE_COSTMAP_LAYERS_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE_COSTMAP_LAYERS_HPP_

#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "rosiwit_navigation/nav_core/costmap_layer.hpp"
#include "rosiwit_navigation/nav_core/obstacle_tracker.hpp"

namespace rosiwit_navigation
{
namespace core
{

/// /map 栅格层. unknown_as_blocked:
///   true  -> unknown 写 kCostUnknown (规划阻断, 不产生膨胀 halo)
///   false -> unknown 当自由 (控制器局部视图: 未知区域不额外加墙)
class StaticLayer : public ICostmapLayer
{
public:
  explicit StaticLayer(bool unknown_as_blocked = false)
  : unknown_as_blocked_(unknown_as_blocked) {}

  std::string name() const override { return "static"; }
  void setOccupancy(const std::vector<int8_t> & data, unsigned int width, unsigned int height);
  void setUnknownAsBlocked(bool v) { unknown_as_blocked_ = v; }
  void stamp(const LayerConfig & cfg, std::vector<int8_t> & grid) const override;

private:
  std::vector<int8_t> data_;
  unsigned int src_width_ = 0;
  unsigned int src_height_ = 0;
  bool unknown_as_blocked_ = false;
};

/// 点障碍层 (激光点 / 体素投影). 半径 0 表示只标记所在格.
class PointsLayer : public ICostmapLayer
{
public:
  explicit PointsLayer(std::string layer_name = "points", double radius = 0.0)
  : name_(std::move(layer_name)), radius_(radius) {}

  std::string name() const override { return name_; }

  void setPoints(const std::vector<Point2D> & points) { points_ = points; }
  void clear() { points_.clear(); }
  void setRadius(double r) { radius_ = r; }
  size_t size() const { return points_.size(); }

  void stamp(const LayerConfig & cfg, std::vector<int8_t> & grid) const override;

private:
  std::string name_;
  double radius_ = 0.0;
  std::vector<Point2D> points_;
};

/// 圆形障碍层 (跟踪器预测位置).
class CirclesLayer : public ICostmapLayer
{
public:
  explicit CirclesLayer(std::string layer_name = "circles") : name_(std::move(layer_name)) {}

  std::string name() const override { return name_; }
  void setCircles(const std::vector<TrackedObstacle> & circles) { circles_ = circles; }
  void clear() { circles_.clear(); }
  size_t size() const { return circles_.size(); }

  void stamp(const LayerConfig & cfg, std::vector<int8_t> & grid) const override;

private:
  std::string name_;
  std::vector<TrackedObstacle> circles_;
};

/// 禁行区层: 多边形栅格化为 lethal. 外部传空列表即撤销.
class KeepoutLayer : public ICostmapLayer
{
public:
  std::string name() const override { return "keepout"; }

  void setPolygons(const std::vector<std::vector<Point2D>> & polygons);
  void clear() { polygons_.clear(); ++version_; }
  void setActive(bool active) { active_ = active; }
  bool active() const { return active_; }
  size_t polygonCount() const { return polygons_.size(); }
  unsigned int version() const { return version_; }

  /// 世界点是否落在任一禁行区内.
  bool contains(double wx, double wy) const;

  void stamp(const LayerConfig & cfg, std::vector<int8_t> & grid) const override;

  /// 上一次 stamp 实际标记的栅格数 (诊断用; 0 说明多边形没落在当前视图里).
  unsigned int lastMarkedCells() const { return last_marked_; }
  const std::vector<std::vector<Point2D>> & polygons() const { return polygons_; }

private:
  std::vector<std::vector<Point2D>> polygons_;
  bool active_ = true;
  unsigned int version_ = 0;
  mutable unsigned int last_marked_ = 0;
};

/// 分区限速 (纯查询). 多边形内取 min(max_linear).
class SpeedLimitLayer
{
public:
  struct Zone
  {
    std::vector<Point2D> polygon;
    double max_linear = std::numeric_limits<double>::max();
  };

  void setZones(const std::vector<Zone> & zones) { zones_ = zones; }
  void clear() { zones_.clear(); }
  size_t size() const { return zones_.size(); }

  /// 返回该点的线速度上限; 不在任何区内返回 fallback.
  double maxLinearAt(double wx, double wy, double fallback) const;
  bool contains(double wx, double wy) const;

  /// 解析 "x,y,w,h,v" 矩形 (v 缺省 = fallback_v).
  static bool parseRectZone(const std::string & spec, Zone & zone, double fallback_v = 0.0);

private:
  std::vector<Zone> zones_;
};

/// 多边形内测试 (射线法, 边界不保证, 栅格用途足够).
bool pointInPolygon(const std::vector<Point2D> & polygon, double wx, double wy);

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE_COSTMAP_LAYERS_HPP_
