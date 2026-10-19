// ============================================================
// 2D 动态障碍跟踪 —— 聚类 + 最近邻关联 + 匀速预测
//
// 输入是 map 系的二维障碍点（激光/局部代价地图/3D 投影均可），
// 输出带 id/速度/预测位置的障碍圆。ROS1/ROS2 薄层只负责把点喂进来，
// 并把预测位置叠加到局部 costmap / 可视化。
// ============================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE_OBSTACLE_TRACKER_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE_OBSTACLE_TRACKER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rosiwit_navigation
{
namespace core
{

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

struct TrackedObstacle
{
  int id = -1;
  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double radius = 0.3;
  int hits = 0;
  int misses = 0;
  bool confirmed = false;

  Point2D predict(double dt) const { return {x + vx * dt, y + vy * dt}; }
  double speed() const;
};

struct ObstacleTrackerConfig
{
  double cluster_distance = 0.35;   // 同簇点间距
  double association_distance = 1.0;  // 帧间关联门限
  double dt_default = 0.1;
  double velocity_smoothing = 0.6;  // 新速度权重
  int confirm_hits = 3;
  int max_misses = 5;
  double max_speed = 2.0;           // 过滤离群速度
  size_t max_points = 4000;
};

class ObstacleTracker
{
public:
  explicit ObstacleTracker(const ObstacleTrackerConfig & config = ObstacleTrackerConfig());

  void setConfig(const ObstacleTrackerConfig & config) { config_ = config; }
  const ObstacleTrackerConfig & config() const { return config_; }

  /// 更新一帧 map 系障碍点。dt<=0 时使用上次时间差/默认值。
  void update(const std::vector<Point2D> & points, double stamp, double dt = -1.0);

  /// 当前已确认障碍（可选包含未确认）。
  std::vector<TrackedObstacle> tracked(bool confirmed_only = true) const;

  /// 预测 horizon 秒后的障碍（供局部 costmap 使用）。
  std::vector<TrackedObstacle> predicted(double horizon) const;

  void clear();
  size_t size() const { return tracks_.size(); }

private:
  struct Cluster
  {
    double x = 0.0;
    double y = 0.0;
    double radius = 0.3;
    int points = 0;
  };

  std::vector<Cluster> cluster(const std::vector<Point2D> & points) const;

  ObstacleTrackerConfig config_;
  std::vector<TrackedObstacle> tracks_;
  int next_id_ = 1;
  double last_stamp_ = -1.0;
};

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE_OBSTACLE_TRACKER_HPP_
