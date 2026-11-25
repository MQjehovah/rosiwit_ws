// include/algorithms/localization/maintainer/insert_gate.h
#pragma once
#include <cmath>
#include "slam_core/localization_stage_types.h"

namespace rosiwit_slam {

/// 地图插入门控: 只有"位姿可信 + 本帧观测被接受 + 两路来源不矛盾"时才写叠加层。
///
/// 退化的位姿一旦写进先验地图, 会把错误变成"地图的一部分", 之后每次定位都会
/// 向它收敛 —— 这是比单帧错位更难恢复的污染。因此插入比定位更严格。
///
/// - tracking:        核心状态是否为 LOCALIZED
/// - match_accepted:  本帧 S2M 是否被接受 (不是"匹配成功了", 而是门控/健康判据通过)
/// - lio_s2m_divergence_m: LIO 相对观测与 S2M 的残差 (无 LIO 时传 0)
/// - cfg.max_insert_divergence <= 0 表示关闭分歧门控 (兼容改造前行为)
inline bool allowMapInsert(bool tracking, bool match_accepted,
                           double lio_s2m_divergence_m,
                           const MaintainerConfig& cfg) {
    if (!tracking) return false;
    if (!match_accepted) return false;
    if (!std::isfinite(lio_s2m_divergence_m)) return false;   // NaN 不能放行
    if (cfg.max_insert_divergence > 0.0 &&
        lio_s2m_divergence_m > cfg.max_insert_divergence) {
        return false;
    }
    return true;
}

/// 锚质量输入: 一次配准结果里与"能不能把这块几何当锚/写进地图"相关的量.
struct AnchorQuality {
    bool   converged = false;
    bool   accepted = false;
    int    inliers = 0;            // 内点数
    int    visible = 0;            // 匹配前可见点数(降采样后, 未做动态滤除); <=0 时用 correspondences
    int    correspondences = 0;    // 有效对应数
    bool   has_hessian = false;    // 是否提供 JtJ 特征分解
    double smallest_eig_ratio = 0.0;
    // P4 (2026-11-26): 位姿被"已确认结构(w>=128)"支撑的内点占比 [0,1]。
    // 退化/未确认区域里匹配可以自洽, 但支撑的是不可信几何 —— 加权锚定用它区分。
    double static_support = -1.0;  // <0 = 配准器未填
};

/// 锚质量判据 (Zhang ICRA 2016 的可观测性 + 场景相对的内点率):
///   - 必须收敛且本帧被采纳;
///   - 有 Hessian 时, 最小/最大特征值比 >= min_eig_ratio (退化方向不可信);
///   - 内点数 >= min_inliers (绝对下限, 防 3 个点也当锚);
///   - 内点数 >= min_ratio * 可见点数 (相对判据: 窄过道里可见几何只有百来个体素,
///     绝对 500 永远达不到; 开阔区可见几千点时 0.5 又会拒掉一半都配不上的弱匹配).
/// 注意分母用**滤除前**的可见点数, 避免 dynamic_filter 的"选择偏差"把 fitness 抬高.
struct AnchorCriteria {
    int    min_inliers = 50;    // 绝对下限(防 3 个点也当锚)
    double min_ratio = 0.5;     // inliers / 可见点数(滤除前)
    double min_eig_ratio = 1e-3;
    double min_static_support = 0.0;  // 内点落在 w>=128 体素的加权占比下限 (0=关)
};

inline bool anchorQualityOk(const AnchorQuality& q, const AnchorCriteria& c) {
    if (!q.converged || !q.accepted) return false;
    if (q.has_hessian && q.smallest_eig_ratio < c.min_eig_ratio) return false;
    // P4: 静态支撑不足 = 位姿主要靠未确认几何对上的 -> 不当锚 (配准使用 w 权重)
    if (c.min_static_support > 0.0 && q.static_support >= 0.0 &&
        q.static_support < c.min_static_support) return false;
    if (q.inliers < c.min_inliers) return false;
    const int vis = q.visible > 0 ? q.visible : q.correspondences;
    if (vis > 0 && static_cast<double>(q.inliers) < c.min_ratio * static_cast<double>(vis)) {
        return false;   // 可见几何里有一半以上都配不上 -> 不信任
    }
    return true;
}

}  // namespace rosiwit_slam
