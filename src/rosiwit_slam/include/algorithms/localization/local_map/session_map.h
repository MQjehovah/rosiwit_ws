// ============================================================================
// session_map.h — 突变场景用的滚动子图（方案 v3 §3 B 阶段）
//
// 解决的场景：机器人进入一个"全是货堆、看不到高可信几何"的区域，
// 全局图的锚（map->odom）不可用。此时用一个**局部子图**维持跟踪：
//
//   * 子图有自己的坐标系 F0（= 创建瞬间的全局位姿），并记下该位姿 T_GS
//   * 只累积 anchor 系下的点；机器人走远后按半径丢弃 -> 内存恒定
//   * T_GS **冻结**：子图更新不会让全局位姿跳动
//   * 全局位姿 = T_GS * pose_s，因此局部连续、全局不崩
//
// 与现有 keyframe_submap 的区别：后者是"从全局图裁一块"（内容可能是旧的），
// 本类是"用进入后的当前观测新建"（内容永远是当下的样子）。
// ============================================================================
#pragma once

#include <cstdint>
#include <unordered_map>

#include <Eigen/Eigen>

#include "slam_core/slam_types.h"

namespace rosiwit_slam {

class SessionMap {
public:
  struct Config {
    double radius = 25.0;      // 机器人周围保留半径 (m)
    double voxel = 0.15;       // 去重体素 (m)
    size_t max_points = 300000;  // 容量上限 (超过后停止插入并告警)
    int clear_stride = 20;       // 自由空间清除: 扫描点抽样步长 (每 N 个点走一条射线)
  };

  void configure(const Config& c) { cfg_ = c; }
  const Config& config() const { return cfg_; }

  /// 以当前全局位姿为锚创建子图（清空）。**必须在位姿可信时调用**，
  /// 否则锚一开始就是错的。
  void reset(const PoseStamped& anchor, double now_s);

  bool active() const { return active_; }
  const PoseStamped& anchor() const { return anchor_; }
  double ageS(double now_s) const { return active_ ? (now_s - created_s_) : 0.0; }
  size_t size() const { return pts_.size(); }

  /// 用一帧机体系点云更新：world = pose*body -> anchor = T_GS^-1*world，
  /// 体素去重；距机器人超过 radius 的点丢弃。
  void update(const CloudType& scan_body, const PoseStamped& pose_world);

  /// 世界系刚性修正: 把整张子图按 (dR, dT) 搬到新位姿解上 (T_GS' = Δ ∘ T_GS)。
  /// 用于"桥接态发生大修正但绝不能清空子图"的场合: 点不动, 只改锚变换, 一次缓存重建。
  void applyWorldCorrection(const Eigen::Matrix3d& dR, const Eigen::Vector3d& dT) {
    if (!active_) return;
    anchor_.rot = dR * anchor_.rot;
    anchor_.trans = dR * anchor_.trans + dT;
    cache_dirty_ = true;
  }

  /// 世界系子图点云（T_GS * pts），供匹配目标使用。带缓存: update/reset 后重建一次。
  CloudType::Ptr pointsWorld() const;

  /// 只取 center 周围 radius 内的世界系点（radius<=0 = 全部）。走同一份缓存, 只做距离筛选,
  /// 用于"存图半径大、参与匹配的半径小"——存 40m 备着, 匹配只喂 25m, 不让目标云膨胀。
  CloudType::Ptr pointsWorld(const Eigen::Vector3d& center, double radius) const;


  /// 关闭并清空（重新看到高可信几何时调用）
  void deactivate() {
    active_ = false;
    pts_.clear();
    have_robot_ = false;
    cache_.reset();
    cache_dirty_ = true;
  }

  /// zip-up 修正: C 为 world 系修正变换 (子图锚系旧值 -> 重锚后的新值),
  /// 作用于子图全部点与锚。桥接结束重锚回 overlay 时调用 (ELite 式)。
  void applyCorrection(const Eigen::Matrix4d& C);

private:
  int64_t key(double x, double y, double z) const;
  void sweepFar();

  Config cfg_;
  bool active_ = false;
  PoseStamped anchor_;
  double created_s_ = 0.0;

  std::unordered_map<int64_t, Eigen::Vector3d> pts_;  // anchor 系
  mutable CloudType::Ptr cache_;      // 世界系缓存 (pointsWorld 复用, 避免每帧重建 30 万点)
  mutable bool cache_dirty_ = true;
  const CloudType::Ptr& cachedWorld() const;
  Eigen::Vector3d robot_anchor_ = Eigen::Vector3d::Zero();  // 机器人在 anchor 系
  Eigen::Vector3d sweep_origin_ = Eigen::Vector3d::Zero();  // 上次清扫时的机器人位置
  bool have_robot_ = false;
  bool capacity_warned_ = false;
};

}  // namespace rosiwit_slam
