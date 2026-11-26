// ============================================================================
// session_map.h — 突变场景用的滚动子图 v2 (ROLL 式段级临时建图)
//
// 参考论文: ROLL (IROS 2022, arXiv:2203.03923) —— Long-Term Robust LiDAR-based
// Localization With Temporary Mapping in Changing Environments.
//
// v1 的问题: 点存进单一哈希表 (anchor 系), 丢失了"这段点是什么时候/以什么位姿
// 注册"的来源信息 —— 导致 (1) 桥接期的漂移只能整图刚性搬移 (zip-up), 段内
// 漂移无法摊平; (2) 掠射 free 证据按帧计数, 静止时误删真实几何。
//
// v2 (ROLL 式):
//   * 桥接期的扫描按【关键帧】注册 (每 kf_min_dist 米 / kf_min_frames 帧一个),
//     每个关键帧带着自己注册时的估计位姿 —— 保留来源信息
//   * 段结束时 (重锚成功), 用可靠匹配位姿作为末端约束, 把整段的漂移按关键帧
//     序号线性摊平 (ROLL Fig.3 两全局因子位姿图的简化闭式解) —— 段级修正
//   * free 证据与 hit 证据对称: 每帧每体素最多一票, 累计 >=free_votes_to_clear
//     才从视图剔除 (不再按帧硬删)
//   * 视图 = 全体关键帧点 变换→合并→体素去重 (缓存), 死亡/自由体素跳过
// ============================================================================
#pragma once

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Eigen>

#include "slam_core/slam_types.h"

namespace rosiwit_slam {

class SessionMap {
public:
  struct Config {
    double radius = 25.0;        // 机器人周围保留半径 (m); 超出的关键帧整体裁剪
    double voxel = 0.15;         // 视图合并时的体素去重 (m)
    size_t max_points = 400000;  // 视图点数上限 (超过后停止插入并告警)
    int clear_stride = 20;       // 自由空间清除: 扫描点抽样步长
    double kf_min_dist = 1.0;    // 关键帧最小间距 (m)
    int kf_min_frames = 15;      // 关键帧最小帧数
  };

  void configure(const Config& c) { cfg_ = c; }
  const Config& config() const { return cfg_; }

  /// 开图 (位姿可信时调用)
  void reset(const PoseStamped& robot, double now_s);
  bool active() const { return active_; }
  const PoseStamped& anchor() const { return anchor_; }
  double ageS(double now_s) const { return active_ ? (now_s - created_s_) : 0.0; }
  size_t size() const { return view_ ? view_->size() : 0; }
  size_t keyframes() const { return m_kfs.size(); }

  /// 桥接注册: 一帧机体系点云 + 当时的估计位姿。
  /// 内部: 变换到世界系 → 追加到当前关键帧 (按距离/帧数滚动切新关键帧) →
  ///       掠射 free 投票 (每帧每体素一票) → 老关键帧裁剪。
  void update(const CloudType& scan_body, const PoseStamped& pose_world);

  /// 段级出口修正 (ROLL Fig.3 简化闭式解): 用重锚得到的可靠位姿作为末端约束,
  /// 把整段关键帧的漂移按序号线性摊平。返回参与修正的关键帧数。
  size_t applyExitCorrection(const PoseStamped& reliable);

  /// zip-up 兼容入口: C 为 world 系修正变换, 作用于全部关键帧位姿。
  void applyCorrection(const Eigen::Matrix4d& C);

  /// 旧接口兼容: (dR, dT) 形式的 zip-up
  void applyWorldCorrection(const Eigen::Matrix3d& dR, const Eigen::Vector3d& dT) {
    if (!active_) return;
    Eigen::Matrix4d C = Eigen::Matrix4d::Identity();
    C.block<3, 3>(0, 0) = dR;
    C.block<3, 1>(0, 3) = dT;
    applyCorrection(C);
  }

  /// 世界系合并视图 (体素去重 + 半径裁剪, 带缓存)。
  /// intensity = 0 (v2 子图是纯几何; w 账本在 maintainer 侧)。
  CloudType::Ptr pointsWorld() const;
  /// 只取 center 周围 radius 内的点 (走同一份缓存, 距离筛选)
  CloudType::Ptr pointsWorld(const Eigen::Vector3d& center, double radius) const;

  /// 关闭并清空 (重新看到高可信几何时调用)
  void deactivate();


private:
  struct EpKf {
    PoseStamped pose;          // 注册时的估计位姿 (world)
    CloudType::Ptr world;      // 世界系点云 (观测时变换, 含 intensity=0)
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  };

  void rebuildView() const;
  void pruneKfs();

  Config cfg_;
  bool active_ = false;
  PoseStamped anchor_;               // 兼容字段 (= 开图位姿)
  double created_s_ = 0.0;
  Eigen::Vector3d robot_trans_ = Eigen::Vector3d::Zero();  // 最近机器人位置 (world)
  Eigen::Matrix3d robot_rot_ = Eigen::Matrix3d::Identity(); // 最近机器人姿态 (world)
  bool have_robot_ = false;

  std::vector<EpKf> m_kfs;           // 段内关键帧
  int m_frames_since_kf_ = 0;
  Eigen::Vector3d m_last_kf_pos_ = Eigen::Vector3d::Zero();

  uint32_t m_frame = 0;

  mutable CloudType::Ptr view_;
  mutable bool view_dirty_ = true;
  mutable bool capacity_warned_ = false;
};

}  // namespace rosiwit_slam
