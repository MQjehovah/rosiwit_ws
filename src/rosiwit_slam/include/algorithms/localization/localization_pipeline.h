// include/algorithms/localization/localization_pipeline.h
#pragma once
#include <memory>
#include <string>
#include "slam_core/i_localization.h"
#include "slam_core/i_scan_matcher.h"
#include "slam_core/i_local_map_provider.h"
#include "slam_core/i_motion_prior.h"
#include "slam_core/i_global_localizer.h"
#include "slam_core/i_localization_monitor.h"
#include "slam_core/i_map_maintainer.h"

namespace rosiwit_slam {

/// 可编排定位流水线: 把定位拆成 5 个可插拔阶段, 由配置选择算法.
///   全局重定位 → 运动先验(预测) → 局部地图 → 配准 → 健康监测
/// 输出位姿 = 配准修正 (通过接受判据) 否则 = 先验预测.
class LocalizationPipeline : public ILocalization {
public:
    LocalizationPipeline() = default;
    ~LocalizationPipeline() override;

    bool init(const std::string& config_path) override;
    void setMap(const std::string& map_name) override;
    void setInitPose(const PoseStamped& pose) override;
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;
    Status getStatus() override;
    bool getPose(PoseStamped& out) override;
    bool getPoseWithCov(PoseStamped& out, Eigen::Matrix<double, 6, 6>& cov) override;

    // 供离线评估/调试: 最近一帧配准结果与置信度
    const MatchResult& lastMatch() const { return m_last_match; }
    double confidence() const { return m_last_match.fitness; }
    std::string describe() const;

private:
    bool acceptMatch(const MatchResult& res, const CloudType::Ptr& scan,
                     const PoseStamped& guess);

    // 各阶段 (由配置选择)
    std::unique_ptr<IGlobalLocalizer>     m_relocalizer;
    std::unique_ptr<IMotionPrior>         m_prior;
    std::unique_ptr<ILocalMapProvider>    m_local_map;
    std::unique_ptr<IScanMatcher>         m_matcher;
    std::unique_ptr<ILocalizationMonitor> m_monitor;
    std::unique_ptr<IMapMaintainer>       m_maintainer;   // 第6阶段: 非破坏性叠加地图维护

    // 配置
    ScanMatcherConfig   m_matcher_cfg;
    LocalMapConfig      m_local_map_cfg;
    MotionPriorConfig   m_motion_cfg;
    GlobalLocalizeConfig m_global_cfg;
    MonitorConfig       m_monitor_cfg;
    AcceptConfig        m_accept_cfg;
    MaintainerConfig    m_maintainer_cfg;
    std::string m_relocalizer_name = "none";
    std::string m_prior_name = "constant_velocity";
    std::string m_local_map_name = "radius_submap";
    std::string m_matcher_name = "pp_icp";
    std::string m_monitor_name = "windowed";
    std::string m_maintainer_name = "none";

    // 状态
    CloudType::Ptr m_map;
    std::string m_map_path;
    PoseStamped m_pose;
    Status m_status = INIT;
    bool m_initialized = false;
    MatchResult m_last_match;

    // 局部地图缓存 (位姿小幅移动时复用, 避免每帧重建 KD-tree/体素)
    CloudType::Ptr m_cached_target;
    PoseStamped m_cached_target_pose;
    bool m_has_cached_target = false;

    // lifelong: 用叠加层刷新全局重定位器
    int m_refresh_counter = 0;
    size_t m_last_overlay_size = 0;

    // 过桥(coasting)与重定位节流
    int m_coast_count = 0;
    int m_reloc_cooldown = 0;

    // s2m 残差接受判据的多帧一致性状态
    V3D m_corr_prev = V3D::Zero();
    bool m_corr_prev_valid = false;
    int m_consistent_cnt = 0;
};

} // namespace rosiwit_slam
