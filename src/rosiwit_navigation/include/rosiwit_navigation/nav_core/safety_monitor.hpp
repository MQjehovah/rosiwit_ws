#ifndef ROSIWIT_NAVIGATION__NAV_CORE__SAFETY_MONITOR_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE__SAFETY_MONITOR_HPP_

#include <string>
#include <utility>
#include <vector>

#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace core
{

struct SafetyMonitorConfig
{
  bool enabled = true;
  std::vector<std::pair<double, double>> stop_polygon;
  double stop_radius = 0.0;  // >0: 用圆形安全区, 优先于 polygon
  int stop_max_points = 2;
  std::vector<std::pair<double, double>> slowdown_polygon;
  int slowdown_max_points = 2;
  double slowdown_ratio = 0.35;
  bool approach_enabled = true;
  double time_before_collision = 2.0;
  double simulation_time_step = 0.1;
  std::vector<std::pair<double, double>> footprint;

  // ---- 以下照 nav2_collision_monitor 的机制补齐 ----
  // 车身轮廓 (base 系): 落在里面的点视为机器人自身, 不参与任何判定.
  // 对应 nav2 的 exclusion_zones.robot_body. 以前用“大矩形自过滤”会连安全区一起吞掉.
  std::vector<std::pair<double, double>> body_polygon;
  // 按运动状态切换的 STOP 区 (nav2 的 VelocityPolygon):
  //   原地旋转 (|vx| < 0.05) -> stop_polygon_rotation
  //   后退   (vx < 0)         -> stop_polygon_backward
  //   其它                    -> stop_polygon
  // 留空表示回退到 stop_polygon.
  std::vector<std::pair<double, double>> stop_polygon_backward;
  std::vector<std::pair<double, double>> stop_polygon_rotation;
  // 原地旋转用**圆**判据 (>0 时优先于 stop_polygon_rotation):
  // 车体原地转扫过的是半径 = max|footprint 顶点| 的圆, 用矩形会在地角方向
  // 多判 0.3~0.5m (实测把 0.83m 外、扫掠只到 0.64m 的物体判成障碍 -> 卡死).
  double stop_radius_rotation = 0.0;
  // 滞回: 连续 N 帧满足才进入 / 释放 (nav2 的 trigger/release_consecutive_points).
  // 默认 1 保持旧行为; 生产配置应设 2/3, 避免单帧噪声让 stop/slow 来回跳.
  int stop_trigger_frames = 1;
  int stop_release_frames = 1;
  int slowdown_trigger_frames = 1;
  int slowdown_release_frames = 1;

  // ---- 动态安全区 (按车体 + 制动距离) ----
  // true: 停车区不再用固定多边形, 而是按 footprint 的几何外沿 + 制动距离生成;
  //       速度越快, 前向停车区越远; 侧向按 车半宽 + 余量 + 转向扫掠。
  // false: 使用上面的固定 stop_polygon / slowdown_polygon (兼容旧配置).
  bool dynamic_geometry = false;
  double brake_decel = 0.8;        // 设计制动减速度 (m/s^2), 需按实车制动测试标定
  double reaction_time = 0.2;      // 反应时间 (s)
  double stop_margin = 0.05;       // 停车区前/后余量 (m)
  double side_margin = 0.03;       // 侧向余量 (m)
  double turn_margin = 0.05;       // 转向扫掠附加侧向余量 (m)
  double slowdown_extra = 0.30;    // 保留字段: 动态模式下的减速区外扩 (m)
  // 减速比例连续化: 每 0.1s 最多变化 scale_slew_rate; STOP 仍立即生效.
  double scale_slew_rate = 0.0;    // 1/s; <=0 关闭 (默认关闭, 保持旧行为; 生产配置开启)
  double scale_dt = 0.1;           // s
};

struct SafetyResult
{
  VelocityCommand command;
  double scale = 1.0;
  std::string state = "PASS";
};

/// ROS-independent collision monitor, equivalent to Nav2 Collision Monitor's
/// STOP / SLOWDOWN / APPROACH behaviour. 额外带车身排除与 trigger/release 滞回.
class SafetyMonitor
{
public:
  void configure(const SafetyMonitorConfig & config)
  {
    config_ = config;
    reset();
  }

  const SafetyMonitorConfig & config() const { return config_; }

  /// 清空滞回状态 (切换任务/切图/恢复后调用).
  void reset() const
  {
    stop_active_ = false;
    stop_trigger_ = 0;
    stop_release_ = 0;
    slowdown_active_ = false;
    slow_trigger_ = 0;
    slow_release_ = 0;
    last_scale_ = 1.0;
  }

  bool stopActive() const { return stop_active_; }
  bool slowdownActive() const { return slowdown_active_; }

  SafetyResult filter(
    const VelocityCommand & cmd,
    const std::vector<std::pair<double, double>> & points) const;

private:
  SafetyMonitorConfig config_;
  // 滞回状态 (filter 是 const, 状态用 mutable: 与 nav2 的 monitor 一样是"有记忆的过滤器")
  mutable bool stop_active_ = false;
  mutable int stop_trigger_ = 0;
  mutable int stop_release_ = 0;
  mutable bool slowdown_active_ = false;
  mutable int slow_trigger_ = 0;
  mutable int slow_release_ = 0;
  mutable double last_scale_ = 1.0;
};

bool pointInPolygon(
  double x, double y,
  const std::vector<std::pair<double, double>> & polygon);

int countPointsInPolygon(
  const std::vector<std::pair<double, double>> & points,
  const std::vector<std::pair<double, double>> & polygon);

double approachScale(
  const std::vector<std::pair<double, double>> & points,
  double vx, double vy, double wz,
  const std::vector<std::pair<double, double>> & footprint,
  double max_time, double time_step);

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__SAFETY_MONITOR_HPP_
