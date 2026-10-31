// include/algorithms/localization/maintainer/insert_gate.h
#pragma once
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
    if (cfg.max_insert_divergence > 0.0 &&
        lio_s2m_divergence_m > cfg.max_insert_divergence) {
        return false;
    }
    return true;
}

}  // namespace rosiwit_slam
