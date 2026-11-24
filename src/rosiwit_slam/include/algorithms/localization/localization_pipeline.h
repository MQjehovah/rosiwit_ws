// include/algorithms/localization/localization_pipeline.h
#pragma once
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "algorithms/localization/local_map/session_map.h"
#include "slam_core/i_localization.h"
#include "rosiwit_common/executor.hpp"
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
    /// 注入线程池: 重定位建库 / 全局搜索改投池; 不注入则保持自带线程
    void setExecutor(rosiwit_common::IExecutor* ex) override { m_executor = ex; }
    void setMapData(const MapData& data) override;
    /// 先验地图快照 (定位模式下 /global_map 的数据源)
    bool getMapCloud(PointVec& out) override;
    bool getOverlayCloud(PointVec& out) override;
    bool getSessionMapCloud(PointVec& out) override;
    /// 运行时注入安装外参 (来自 URDF/TF/启动参数); 统一驱动扫描变换与偏航轴 up_body.
    void setMountExtrinsic(const Eigen::Matrix3d& r_base_lidar,
                           const Eigen::Matrix3d& r_il,
                           const Eigen::Vector3d& t_il,
                           const Eigen::Vector3d& t_base_lidar) override;
    /// 地面机器人约束: 位姿 = x/y/yaw + 固定安装姿态 (R=Rz(yaw)*R_mount, z=安装高度).
    /// 竖直墙 + 无地面地图下, z/roll/pitch 是弱观测, 漂了会让远处墙点全部错开.
    void planarizePose(PoseStamped& p) const;
    void cancelSetMap() override { m_cancel_set_map.store(true); }
    void setInitPose(const PoseStamped& pose) override;
    void onImu(const IMUSample& s) override;
    void onOdom(const PoseStamped& odom) override;
    void onRelative(const PoseStamped& pose,
                    const Eigen::Matrix<double, 6, 6>& cov) override;
    void resetRelativeAnchor() override;
    std::string poseFusionName() const override { return m_fusion_name; }
    /// 定位链路支持 odom: localization.odom_mode: fuse 时同时喂 motion_prior 和
    /// pose_fusion(eskf); 具体开关见 init() 里的 m_odom_fuse.
    bool supportsOdom() const override { return true; }
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
    /// 锚质量(行业标准, Zhang ICRA 2016): 本帧收敛+被采纳 + 内点数 + JtJ 最小特征值比.
    /// 只看 fitness 会把"假锁"(几何退化但残差小)当成可信, 用它写地图等于把错误固化.
    bool anchorOk(const MatchResult& res, bool accepted) const;
    /// 按逐点时间戳把一帧扫描补偿到帧末位姿; 旋转时避免点云在时间上被拉斜.
    CloudType::Ptr deskewScan(const LidarFrame& f, const PoseStamped& end_pose);
    bool tryBoundedRecovery(const CloudType::Ptr& scan);   // 出口重锁(有界局部重定位)
    CloudType::Ptr buildTarget(const PoseStamped& pose);
    /// 锚定模式目标: 底图 crop + overlay (压制同区域旧点) —— 探测重锚/正常锚定用
    CloudType::Ptr buildAnchoredTarget(const PoseStamped& pose);   // 局部地图 + 叠加层(含压制)
    /// 扫描预处理: 按给定帧末位姿去畸变 + 雷达->IMU + 高度带裁剪。
    /// 匹配用 guess 版本; 写地图必须用"匹配解 res.pose"再算一遍, 否则点云会被
    /// 转差 corr_deg 那几度, 同一面墙写两次就是两层(实车叠图根因之一)。
    CloudType::Ptr prepareScan(const LidarFrame& f, const PoseStamped& end_pose);

    /// 子图 vs 先验图 的一致性观测 (只测不改): 把子图刚性地对齐到先验图,
    /// 输出残余刚体变换 (Δt, Δyaw)。Δt≈0 => 两者一致; 明显非零 => 子图已经偏了。
    void checkSubmapPriorConsistency();
    bool ensureRelocalizerMap();   // 全局重定位库就绪检查(后台构建中返回 false, 绝不阻塞)
    void relocBuildWorker();       // 兼容路径(未注入执行器): 常驻等待请求
    bool runRelocBuildOnce();      // 池任务: 构建一次重定位库
    void kickRelocBuild();         // 投递 (有在途则不重复投)
    void globalRelocWorker();      // 兼容路径(未注入执行器): 常驻等待请求
    bool runGlobalRelocOnce();     // 池任务: 做一次全局搜索
    void kickGlobalReloc();        // 投递 (有在途则不重复投)
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
    Eigen::Matrix3d m_base_r = Eigen::Matrix3d::Identity();  // base_footprint <- lidar 安装旋转
    bool   m_base_r_valid = false;
    double m_fixed_z = -1.0;                                  // 雷达在地图系的目标高度, <0 = 未设
    // 自转中心 (转动瞬心) 在 base 系的 xy, 标定值; 默认 base 原点.
    // 差速底盘的轮速模型假设 base 原点就是瞬心. 标定必须用机器人**自转**的数据
    // (手动推的轨迹瞬心随手法漂移), 方法见 default.yaml 的 rotation_center 注释.
    Eigen::Vector2d m_rot_center = Eigen::Vector2d::Zero();
    double m_planar_z = 0.0;                                   // planar.z: body(车底) 目标高度
    bool   m_fix_planar = true;                               // 地面约束开关
    Eigen::Matrix4f m_lidar_to_imu = Eigen::Matrix4f::Identity();
    MaintainerConfig    m_maintainer_cfg;

    // ---- B 阶段: 突变场景滚动子图 (shadow: 只累积 + 日志, 暂不参与匹配) ----
    // 高可信几何看不到 (persistency_cov 低) 时开子图, 冻住锚; 恢复后关闭。
    SessionMap m_session_map;
    mutable std::mutex m_session_map_mu;   // onLidar 与 setMap/setInitPose 都会碰子图
    bool   m_session_map_enabled = true;
    // 锚质量判据 (行业标准): JtJ 最小特征值比 (Zhang, ICRA 2016) + 内点数
    int    m_anchor_min_inliers = 50;     // 绝对下限(防 3 个点也当锚)
    double m_anchor_min_ratio = 0.5;      // 内点 / 匹配前可见点数
    double m_anchor_min_eig_ratio = 1e-3;
    int    m_anchor_patience = 5;         // 连续弱锚帧 -> 开会话地图
    bool   m_ever_localized = false;      // 至少成功定位过一次 (之后才允许开会话地图/锚定)
    // 先验图-only 匹配度(fitness_base)的状态; KD 树/目标云放在 .cpp 匿名命名空间(避免头文件引 pcl)
    bool   m_base_only_valid = false;
    int    m_base_only_age = 0;           // 帧计数, 每 30 帧重建一次底图 crop
    double m_cov_base = -1.0;             // 最近一帧的底图覆盖率 (<0 = 未算)
    int    m_cov_base_ok_streak = 0;      // cov_base 达标的连续帧数
    // 锚定状态只有 m_on_submap 一个 (m_anchor_on_base 已合并, 见下)
    int    m_consistency_tick = 0;        // 子图↔先验一致性观测的节拍 (机器人侧新增)
    // 软着陆: 种子/重定位刚生效后的前 N 帧限制单帧修正量(1°/5cm),
    // 防止首帧跳变把点云写岔成两层(机器人侧新增, 锚定切换路径由 zip-up 即时搬移, 不再使用)
    int    m_soft_land_frames = 0;
    int    m_anchor_close_patience = 10;  // 连续好锚帧 -> 关
    // ---- 子图桥接模式 ----
    bool m_on_submap = false;             // true = 目标=子图, 边建图边定位
    int  m_probe_countdown = 0;           // 距下一次 overlay 探测重锚的帧数
    int  m_probe_period = 50;             // 探测周期 (帧, 5s @10Hz)
    int    m_anchor_weak_streak = 0;
    int    m_anchor_ok_streak = 0;
    double m_session_map_radius = 25.0;
    double m_session_map_match_radius = 25.0;   // 参与匹配的子图半径(<=0=全部); 存储仍用 radius_m
    // 子图是"瞬时补丁", 写入门槛必须比 overlay 低: 变化区里 fitness(对先验) 天然低,
    // 若还要求和先验匹配得上, 就永远写不进去 -> 桥接时没有跟踪源 -> 必丢(2026-11-24 实机)。
    double m_sm_write_fitness = 0.35;   // 桥接态写子图的最低 fitness
    double m_sm_write_corr_m = 0.25;    // 允许的单帧修正
    double m_sm_write_corr_deg = 3.0;

    double m_session_map_voxel = 0.15;
    size_t m_session_map_max_points = 300000;
    /// 服务/换图线程调用: 加锁清空子图, 并退出子图桥接模式
    void resetSessionMap() {
        std::lock_guard<std::mutex> lk(m_session_map_mu);
        m_session_map.deactivate();
        m_have_submap_anchor = false;   // 换图/重置后子图窗口锚必须重来
        m_on_submap = false;
        m_probe_countdown = 0;
        m_escape_cnt = 0;               // 换图后逃逸候选作废
        m_escape_prev_valid = false;
    }
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
    /// localization.odom_mode: fuse 时为 true. 单一开关: 同时决定运动先验
    /// (motion_prior.use_odom) 与位姿融合层是否接收 odom.
    bool m_odom_fuse = false;
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
    /// 种子观察窗剩余帧数: 进入 LOCALIZED 时置为 monitor.scrutiny_frames,
    /// 期间每帧都要求 fitness >= reloc_accept_score, 否则退回 INIT。
    int m_scrutiny_frames = 0;
    uint64_t m_cached_target_rev = 0;   // 建缓存时的维护层版本号(overlay 变更 -> 失效)
    // 子图窗口锚 (2026-11-25): 只有"已验证帧"才把窗口中心吸附到匹配位姿,
    // 弱帧(贴墙/稀疏/退化)保持旧锚点, 防止弱匹配把子图窗口带偏 -> 自洽锁歪.
    PoseStamped m_submap_anchor;
    bool m_have_submap_anchor = false;
    // 冻结逃逸 (2026-11-25): 连续拒绝期间, 记录"匹配自己是否稳定"。
    // 匹配每帧都指向同一个位姿 = 真值在那边, 门控却不让过 -> 放行一次.
    int  m_escape_cnt = 0;              // 连续一致帧数
    bool m_escape_prev_valid = false;
    PoseStamped m_escape_prev;          // 上一帧候选
    PoseStamped m_escape_first;         // 本轮首个候选(限制累计离散)
    double m_escape_last_fire_t = -1e9;  // 上次逃逸时间(冷却)

    // 全局重定位库后台构建 (2026-10-17 推行丢失事故: 懒建把 44.6s 的 FPFH 全图特征
    // 构建塞进 onLidar 回调, 单线程 spinner 冻结, 雷达停处理/界面冻住。行业惯例:
    // 重活不进实时路径 -> setMap 后立即后台线程预建, 回调只查结果)
    rosiwit_common::IExecutor* m_executor = nullptr;
    std::atomic<bool> m_reloc_job{false};    // 建库任务在途
    std::atomic<bool> m_global_job{false};   // 全局搜索任务在途
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
    // 匹配位姿与融合输出持续分歧帧数 (创新过大但 ESKF 一直拒收 -> 判定定位不可信)
    int m_inconsistent_streak = 0;
    // 阈值来自 localization.monitor.*: 匹配位姿 vs 融合输出的平移/航向分歧超过它算一帧不一致,
    // 连续 lost_patience 帧即判 LOST (现场: GICP 每帧匹配到 ~10m 外, 但 ESKF 一直拒收)
    double m_consensus_trans_m = 1.0;
    double m_consensus_yaw_deg = 20.0;

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
