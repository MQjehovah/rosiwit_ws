// ============================================================================
// coverage_grid.hpp — 动态覆盖清洁的覆盖状态栅格 (ROS 无关)
//
// 职责:
//   * 把多边形区域栅格化, 维护 uncovered / covered / obstacle / blocked 状态
//   * 按清扫宽度实时标记已覆盖区域 (不是按规划路径)
//   * 用 BFS 找最近的可达未覆盖格 (前沿式动态覆盖的 v1 目标选择)
//   * 区分静态不可达和动态障碍; 输出不可达区域
// 不依赖 ROS, 由 ROS1/ROS2 接口层传入/取出普通结构体。
// ============================================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE__COVERAGE_GRID_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE__COVERAGE_GRID_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace core
{

enum class CoverageCell : int8_t
{
  OUTSIDE = -2,    // 多边形外
  UNKNOWN = -1,    // 地图未知
  UNCOVERED = 0,   // 区域内的自由、未覆盖
  COVERED = 50,    // 已覆盖
  BLOCKED = 80,    // 不可达/被障碍永久挡住
  OBSTACLE = 100,  // 静态障碍
};

struct CoverageConfig
{
  double sweep_width = 0.5;       // 清扫宽度 (覆盖盘直径)
  double obstacle_dilation = 0.10;  // 动态障碍点膨胀半径
  // 车体中心相对区域边界的内缩量. 设成 sweep_width/2 时, 清扫边缘由
  // 工具宽度覆盖, 车体中心不必贴到边界, 避免控制超调跑出区域.
  double center_inset = 0.0;
  // 车体中心相对**静态障碍**的内缩量 (= 车体半宽). 0 = 关闭 (旧行为).
  // 不开时行会落到贴着墙、车体根本站不下的格子 -> 同一行反复规划直到超时.
  double robot_inset = 0.0;
  // 行间掉头方式:
  //   0        = 原地转 90° + 直线横移 + 原地转 90° (差速轮 R=0)
  //   > 0      = 半圆弧掉头, 半径 R = 行距/2, 上限为本值
  // 半圆掉头的弧长 πR, 曲率半径固定, 控制器可以**不停车**地连续跟踪;
  // 代价是行端要留出 R 的转弯余量 (从两侧的行端内缩 R), 以及弧线会扫到
  // 行端外侧 R 的带状区域 (正是 headland 存在的理由).
  double turn_radius = 0.0;
  // headland 环宽度 (m): 沿区域边界内缩 headland_width/2 走一圈, 覆盖边带.
  // 0 = 不做 (旧行为). 一般设成 sweep_width 的整数倍 (1 圈 = sweep_width).
  // 没有它时行线范围永远差最后半条扫宽: 5.55x2.85 房间顶部 0.34m 条带
  // (1.67 m2, 10.5%) 一条行线都不经过.
  double headland_width = 0.0;
  // 门洞宽度阈值 (m): 某个切片上的可通行宽度 <= 该值时, 强制在单元分解处切开.
  // 0 = 关闭. 整屋覆盖时设为 2*robot_inset+0.1 (0.72m), 让门洞成为独立单元,
  // 房间之间按单元贪心排序, 不再一条长行线横跨两个房间.
  double doorway_width = 0.0;
};

struct CoverageStats
{
  size_t total = 0;      // covered + uncovered + blocked
  size_t covered = 0;
  size_t uncovered = 0;
  size_t blocked = 0;
  size_t obstacle = 0;
  size_t unknown = 0;
  size_t outside = 0;

  double coverageRate() const
  {
    return total > 0 ? static_cast<double>(covered) / static_cast<double>(total) : 0.0;
  }
  double coveredArea(double resolution) const
  {
    return static_cast<double>(covered) * resolution * resolution;
  }
};

class CoverageGrid
{
public:
  bool initialize(const CostmapGridInfo & info, const std::vector<Pose2D> & polygon,
                  const CoverageConfig & config);
  bool ready() const { return ready_; }

  // 静态地图: v>=100 -> OBSTACLE, v<0 -> UNKNOWN (不影响已覆盖/已标记的格子)
  void updateStaticObstacles(const std::vector<int8_t> & map_data);

  // 动态障碍: 每帧重置; 点周围 obstacle_dilation 内标记为动态占用.
  void updateDynamicObstacles(const std::vector<Pose2D> & points);

  // 用当前位姿推进覆盖: 把上一帧到这一帧的轨迹按 sweep_width 标成 covered.
  void updateCovered(const Pose2D & pose);
  void resetPose(const Pose2D & pose);

  // 最近的可达未覆盖格 (BFS, 4 邻域). avoid_dynamic=true 时绕开动态障碍.
  bool nearestUncovered(const Pose2D & pose, Pose2D & target, bool avoid_dynamic) const;
  // 在 max_distance 内取 BFS 最远的未覆盖格: 一次走一条, 避免逐格爬行.
  bool farthestUncovered(const Pose2D & pose, Pose2D & target,
                         double max_distance, bool avoid_dynamic) const;
  // 取最近未覆盖格所在的一整行/列连续可通行段 (局部弓字行段).
  // 返回的 segment 从靠近机器人一端到远端, 长度不超过 max_length.
  bool nextCoverageSegment(const Pose2D & pose, double max_length,
                           std::vector<Pose2D> & segment) const;

  // 是否存在可达的未覆盖格.
  bool hasUncoveredReachable(const Pose2D & pose, bool avoid_dynamic) const;
  bool hasAnyUncovered() const;
  bool contains(const Pose2D & pose) const;
  size_t uncoveredCount() const;

  // 静态可达性: 把不可达的 UNCOVERED 标成 BLOCKED.
  void markUnreachable(const Pose2D & pose);
  // 动态障碍超时: 把所有剩余未覆盖标成 BLOCKED.
  void markAllUncoveredBlocked();
  /// 把 pose 附近 radius 内的未覆盖格标成 BLOCKED (单行/单点走不到时用,
  /// 只放弃这一条带, 不是整块区域).
  void markBlockedAround(const Pose2D & pose, double radius);
  /// 区域质心 (多边形顶点平均): 车体被超调带出区域时用它做回家目标.
  Pose2D centroid() const;

  /// 一次生成**剩余全部覆盖路线** (boustrophedon):
  ///   headland 行线 -> passable 裁剪 -> 行间连接 -> 原地转点
  /// 点序即执行序; **位置与前一点相同的点表示"原地转到该点朝向"** (velocity_x=0).
  /// 对齐 fields2cover 的 SwathGenerator + Route / opennav_coverage 的
  /// "一次出完整 route, 再交给控制器" 架构.
  /// 返回 false = 没有剩余需要扫的行段.
  bool buildRoute(const Pose2D & pose, std::vector<Pose2D> & route) const;
  // 把 seed 所在的未覆盖连通块整体标成 BLOCKED (A* 规划失败时用).
  size_t markComponentBlocked(const Pose2D & seed);

  // 不可达/被挡区域的矩形包络 (每个 cluster 一个 4 点多边形).
  std::vector<std::vector<Pose2D>> blockedRegions(size_t min_cells = 4) const;

  CoverageStats stats() const;

  // 给 /clean_map 的编码: -1 区域外/unknown, 0 未覆盖, 50 已覆盖,
  // 80 不可达/被挡, 100 障碍/动态障碍.
  std::vector<int8_t> encode() const;

  // 给覆盖 A* 的规划图: 0 可通行, 100 不可通行.
  // 区域外不再一律当障碍, 而是按真实地图判断 (否则边缘目标会被
  // 区域边界 + 膨胀误判成 Goal in obstacle).
  std::vector<int8_t> plannerCostmap(const std::vector<int8_t> & map_data) const;

  const CostmapGridInfo & info() const { return info_; }
  size_t dynamicCount() const;

private:
  int index(int mx, int my) const { return my * static_cast<int>(info_.width) + mx; }
  bool cellInBounds(int mx, int my) const;
  bool worldToCell(double x, double y, int & mx, int & my) const;
  Pose2D cellToWorld(int mx, int my) const;
  bool passable(int mx, int my, bool avoid_dynamic) const;
  bool pointInPolygon(double x, double y) const;
  void markDisk(double x, double y, double radius, CoverageCell state,
                bool only_uncovered, bool skip_dynamic);
  bool bfsStartCell(const Pose2D & pose, int & mx, int & my) const;
  bool bfsReach(const Pose2D & pose, bool avoid_dynamic,
                std::vector<uint8_t> * visited) const;

  CostmapGridInfo info_;
  CoverageConfig config_;
  std::vector<CoverageCell> cells_;
  std::vector<uint8_t> dynamic_;
  std::vector<uint8_t> inset_;
  /// 静态障碍的车体半径 halo: !=0 表示"车体中心不能到这里" (但清扫宽度可覆盖).
  std::vector<uint8_t> halo_;
  std::vector<Pose2D> polygon_;
  bool ready_ = false;
  bool scan_vertical_ = false;  // true: 沿 y 扫; false: 沿 x 扫
  bool has_last_pose_ = false;
  Pose2D last_pose_;
};

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__COVERAGE_GRID_HPP_
