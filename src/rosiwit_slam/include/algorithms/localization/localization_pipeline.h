// include/algorithms/localization/localization_pipeline.h
#pragma once
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "slam_core/i_localization.h"
#include "slam_core/i_scan_matcher.h"
#include "slam_core/i_local_map_provider.h"
#include "slam_core/i_motion_prior.h"
#include "slam_core/i_pose_fusion.h"
#include "slam_core/i_global_localizer.h"
#include "slam_core/i_localization_monitor.h"
#include "slam_core/i_map_maintainer.h"
#include "algorithms/localization/constraints/planar_constraint.h"
#include "algorithms/localization/hypothesis/hypothesis_manager.h"
#include "algorithms/localization/smoother/fixed_lag_smoother.h"
#include "slam_core/i_map_store.h"

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
    void setMapData(const MapData& data) override;
    void cancelSetMap() override { m_cancel_set_map.store(true); }
    void setInitPose(const PoseStamped& pose) override;
    void onImu(const IMUSample& s) override;
    void onOdom(const PoseStamped& odom) override;
    void onLidar(const LidarFrame& f) override;
    /// 按需触发全局重定位: 置 LOST 并清冷却, 下一次 onLidar 进入冷启动/重定位分支
    void requestRelocalization() override;
    Status getStatus() override;
    bool getPose(PoseStamped& out) override;
    bool getPoseWithCov(PoseStamped& out, Eigen::Matrix<double, 6, 6>& cov) override;
    /// 先验不确定度 (米/弧度): 由运动先验的打滑权重推出.
    bool priorUncertainty(double& xy_sigma, double& yaw_sigma) const override;
    bool predictAt(double t, PoseStamped& out) override;
    void onAbsolute(const PoseStamped& pose, const Eigen::Matrix<double, 6, 6>& cov) override;

    // 供离线评估/调试: 最近一帧配准结果与置信度 (加锁返回快照, 供 ROS2 多线程读)
    MatchResult lastMatch() const {
        std::lock_guard<std::mutex> lk(m_snapshot_mu);
        return m_snapshot_match;
    }
    double confidence() const {
        std::lock_guard<std::mutex> lk(m_snapshot_mu);
        return m_snapshot_confidence;
    }
    std::string describe() const;
    /// 位姿融合诊断 (direct 模式下计数值无意义)
    FusionDiag fusionDiag() const;
    std::string poseFusionDiagnostics() const override;
private:
    void setMapImpl(const std::string& map_name);
    bool acceptMatch(const MatchResult& res, const CloudType::Ptr& scan,
                     const PoseStamped& guess, const char** reject_reason = nullptr);
    /// 按逐点时间戳把一帧扫描补偿到帧末位姿; 旋转时避免点云在时间上被拉斜.
    CloudType::Ptr deskewScan(const LidarFrame& f, const PoseStamped& end_pose);
    bool tryBoundedRecovery(const CloudType::Ptr& scan);   // 出口重锁(有界局部重定位)
    CloudType::Ptr buildTarget(const PoseStamped& pose);   // 局部地图 + 叠加层(含压制)
    bool ensureRelocalizerMap();   // 全局重定位库就绪检查(后台构建中返回 false, 绝不阻塞)
    void relocBuildWorker();       // 后台构建线程: FPFH 全图特征+建树(~45s, 不进回调)
    void globalRelocWorker();      // 全局候选搜索线程: 不持 m_state_mu
    void updateSnapshotLocked();   // onLidar 持锁时刷新无锁读快照

    // 各阶段 (由配置选择)
    std::unique_ptr<IGlobalLocalizer>     m_relocalizer;
    std::unique_ptr<IMotionPrior>         m_prior;
    std::unique_ptr<ILocalMapProvider>    m_local_map;
    std::unique_ptr<IScanMatcher>         m_matcher;
    std::unique_ptr<IScanMatcher>         m_global_matcher;   // 全局候选精配专用，独立线程使用
    std::unique_ptr<ILocalizationMonitor> m_monitor;
    std::unique_ptr<IMapMaintainer>       m_maintainer;   // 第6阶段: 非破坏性叠加地图维护
    // 第7阶段: 位姿融合 (direct=直通 / eskf=IMU+odom 推进 + 匹配观测限幅修正)
    std::unique_ptr<IPoseFusion>          m_fusion;
    mutable std::mutex                    m_fusion_mu;
    std::atomic<bool> m_cancel_set_map{false};

    // 配置
    ScanMatcherConfig   m_matcher_cfg;
    LocalMapConfig      m_local_map_cfg;
    MotionPriorConfig   m_motion_cfg;
    GlobalLocalizeConfig m_global_cfg;
    MonitorConfig       m_monitor_cfg;
    AcceptConfig        m_accept_cfg;

    // 雷达 -> IMU 外参 (ROS1 override YAML 顶层的 r_il/t_il)。
    // 建图前端在内部做了这个变换, 定位路径没有; 实机雷达前倾 43 度时,
    // 不改会把雷达系点云直接和 IMU/车体系的图做匹配, 永远收敛不了。
    bool m_lidar_to_imu_valid = false;
    Eigen::Matrix4f m_lidar_to_imu = Eigen::Matrix4f::Identity();
    MaintainerConfig    m_maintainer_cfg;
    PlanarConstraintConfig m_planar_cfg;
    std::string m_relocalizer_name = "none";
    std::string m_prior_name = "constant_velocity";
    std::string m_local_map_name = "radius_submap";
    std::string m_matcher_name = "pp_icp";
    std::string m_monitor_name = "windowed";
    std::string m_maintainer_name = "none";
    std::string m_fusion_name = "direct";   // direct | eskf
    PoseFusionConfig m_fusion_cfg;

    // 状态
    // 并发契约:
    //   - onLidar 是唯一的大状态写入者; 它持 m_state_mu 整个执行期.
    //   - setMap 走 m_state_mu；setInitPose/requestRelocalization 只入命令队列，
    //     由 onLidar worker 消费，服务线程不等待长耗时重定位。
    //   - getStatus/getPose/getPoseCov/confidence 走轻量快照，不等待 onLidar 内的
    //     全局重定位/GICP，避免 ROS spinner 被拖住。
    //   - 高频输出 predictAt 只走 m_prior_mu；所有 m_prior->xxx 都走 m_prior_mu.
    mutable std::mutex m_state_mu;
    mutable std::mutex m_prior_mu;
    mutable std::mutex m_snapshot_mu;
    mutable std::mutex m_cmd_mu;
    PoseStamped m_pending_init;
    bool m_pending_init_valid = false;
    bool m_pending_reloc = false;
    PoseStamped m_snapshot_pose;
    Eigen::Matrix<double, 6, 6> m_snapshot_cov = Eigen::Matrix<double, 6, 6>::Identity();
    MatchResult m_snapshot_match;
    bool m_snapshot_valid = false;
    double m_snapshot_confidence = 0.0;
    CloudType::Ptr m_map;
    std::string m_map_path;
    PoseStamped m_pose;
    /// 逸出保护: 最后一次**被接受**的匹配位姿。位姿跑出地图(局部地图为空)时回退到这里,
    /// 避免运动先验在无修正情况下一直积分, 越漂越远(实机曾漂到离图 96m, 从此再也匹配不上)。
    PoseStamped m_last_accepted;
    bool m_have_last_accepted = false;
    bool m_deskew_enabled = true;
    double m_deskew_max_gap = 0.30;       // 上一接受帧到当前帧起始的最大间隔(s)
    double m_deskew_min_angle_deg = 0.2;  // 小于此旋转量不做补偿
    std::atomic<Status> m_status{INIT};
    std::atomic<bool> m_initialized{false};
    MatchResult m_last_match;

    // 局部地图缓存 (位姿小幅移动时复用, 避免每帧重建 KD-tree/体素)
    CloudType::Ptr m_cached_target;
    PoseStamped m_cached_target_pose;
    bool m_has_cached_target = false;
    /// setInitPose 后首帧必须通过 global.reloc_accept_score；失败仍留在 INIT，
    /// 由 lost_patience 连续帧计数决定何时 LOST。
    bool m_seed_pending = false;
    uint64_t m_cached_target_rev = 0;   // 建缓存时的维护层版本号(overlay 变更 -> 失效)

    // 全局重定位库后台构建 (2026-10-17 推行丢失事故: 懒建把 44.6s 的 FPFH 全图特征
    // 构建塞进 onLidar 回调, 单线程 spinner 冻结, 雷达停处理/界面冻住。行业惯例:
    // 重活不进实时路径 -> setMap 后立即后台线程预建, 回调只查结果)
    std::thread m_reloc_thread;
    std::mutex m_reloc_mu;
    std::condition_variable m_reloc_cv;
    // 全局重定位搜索线程：只做 FPFH 候选搜索，不持 m_state_mu；
    // 找到候选后通过 m_pending_init 交给本地 tracking worker。
    std::thread m_global_thread;
    std::mutex m_global_mu;
    std::condition_variable m_global_cv;
    CloudType::Ptr m_global_scan;
    uint64_t m_global_scan_generation = 0;
    bool m_global_scan_valid = false;
    bool m_global_request = false;
    bool m_global_stop = false;
    std::atomic<uint64_t> m_global_generation{0};
    int m_global_cooldown = 0;
    std::atomic<bool> m_reloc_building{false};
    bool m_reloc_request = false;       // setMap 置位, worker 消费
    bool m_reloc_stop = false;
    bool m_reloc_ready = false;         // 构建完成且对应当前地图
    CloudType::Ptr m_reloc_map_snap;    // 构建目标的地图快照(指针比对判断是否过期)
    std::string m_reloc_path_snap;
    // 冷启动消歧: top1-top2 精配分差小于该值 => 保留多假设跟踪(重复结构双解)
    double m_ambiguity_margin = 0.02;
    // 跟踪期重播种可达性门控: 候选离当前位姿超该值不进假设池(31m 瞬移教训)
    double m_reseed_radius = 5.0;
    // 状态机连续帧计数：健康帧累计到 localized_patience -> LOCALIZED；
    // 不健康帧累计到 lost_patience -> LOST。LOCALIZED 不因单帧异常回到 INIT。
    int m_healthy_streak = 0;
    int m_unhealthy_streak = 0;

    // lifelong: 用叠加层刷新全局重定位器
    int m_refresh_counter = 0;
    size_t m_last_overlay_size = 0;

    // 过桥(coasting)计数
    int m_coast_count = 0;
    // 注入位姿后的 Mahalanobis 门限放行帧数: reset 运动先验会丢掉
    // "3D 匹配 vs IMU 先验" 的固有 roll/pitch 偏置, 门限会每帧都拒且永远学不回来.
    int m_maha_bypass_frames = 0;

    // coast 重锚阈值: matcher 连续失败超过该帧数后, 匹配初值改用最后被接受
    // 的位姿(m_last_accepted)而不是跟着漂的先验, 给 ICP 一个干净起点
    int m_coast_reanchor_frames = 5;

    PlanarConstraint m_planar;   // 地面/平面约束 (第7层视觉约束, 可选)

    HypothesisManager m_hyp;     // 多假设跟踪 (aliasing)
    int m_max_hypotheses = 4;
    double m_hyp_margin = 0.25;
    int m_hyp_stale = 10;
    int m_hyp_reseed_frames = 0; // 跟踪期: 连续未采纳 N 帧后重定位重播多假设 (0=关)
    int m_low_fit_frames = 0;

    FixedLagSmoother m_smoother; // 固定滞后平滑器 (L4)
    SmootherConfig m_smoother_cfg;

    // L19 绝对约束融合
    AbsoluteConfig m_abs_cfg;
    PoseStamped m_abs_pose;
    double m_abs_time = -1.0;
    bool m_has_abs = false;

    // L8: 统一地图存储底座 (可选; 算法经 IMapStore 取数据)
    std::unique_ptr<IMapStore> m_store;
    std::string m_store_name = "none";   // none | pcd | voxel
    double m_store_leaf = 0.3;

    // s2m 残差接受判据的多帧一致性状态
    V3D m_corr_prev = V3D::Zero();
    bool m_corr_prev_valid = false;
    int m_consistent_cnt = 0;
};

} // namespace rosiwit_slam
