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
//   * 动态点剔除 (2026-11-28): 体素必须在 >= min_kf_hits 个关键帧里出现过才进视图/
//     融合 —— 行人、门、反光等单帧瞬态只出现在 1 个关键帧, 自然被淘汰; 静态结构在
//     沿途多个关键帧里重复出现. 最新关键帧不过滤, 保证子图刚开/刚移动时视图不为空.
//     (旧版在这里按"扫描点体素"投 free 票, 投错了对象 —— 静止 3 帧就把全部点判死,
//      见 131ee52; 现在的判据只看"跨关键帧重复观测", 不依赖射线.)
//   * 视图 = 全体关键帧点 变换→合并→体素去重 (缓存), 动态瞬态体素跳过
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
    double kf_min_dist = 1.0;    // 关键帧最小间距 (m)
    int kf_min_frames = 15;      // 关键帧最小帧数 (当前只在距离触发建新帧)
    int min_kf_hits = 2;         // 动态点剔除: 体素至少被 N 个关键帧看到才保留 (0/1=关闭)
                                 // 单关键帧出现的点通常是行人/门/反光等瞬态。最新关键帧
                                 // 例外 (不过滤), 保证子图刚开/刚移动时视图不为空。
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


  /// 段内关键帧 (注册位姿 + 世界系点云)。走出覆盖时整体交给 w 账本融合。
  struct EpKf {
    PoseStamped pose;          // 注册时的估计位姿 (world)
    CloudType::Ptr world;      // 世界系点云 (观测时变换)
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  };

  /// 走出子图覆盖的关键帧整体退出 (由管道取走做一次性融合+雕刻)。
  /// 只按 XY 距离判断 (z 由车辆高度决定, 不参与)。
  std::vector<EpKf> popLeavingKfs(const Eigen::Vector3d& cur_xy);

private:
  void rebuildView() const;
  void pruneKfs();
  /// 该体素是否只被过少的关键帧看到 (动态瞬态)。缺记录时按"支持"处理 (不误删)。
  bool voxelTransient(int64_t vk) const;
  /// 关键帧命中表按机器人位置做距离裁剪 (只在明显超上限时做, 防长时间运行无界增长)。
  void maybePruneHits();

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

  // 动态点剔除: 体素(0.15) -> 出现过它的关键帧数 (cap 255)。缺记录 = 按支持处理。
  std::unordered_map<int64_t, uint8_t> kf_hits_;

  uint32_t m_frame = 0;

  mutable CloudType::Ptr view_;
  mutable bool view_dirty_ = true;
  mutable bool capacity_warned_ = false;
};

}  // namespace rosiwit_slam
