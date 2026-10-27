#pragma once
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <deque>
#include <memory>
#include "rosiwit_common/channel.hpp"
#include "rosiwit_common/thread.hpp"
#include "slam_core/i_slam_algorithm.h"
#include "slam_core/slam_base.h"
#include "slam_core/i_frontend.h"
#include "slam_core/i_backend.h"
#include "slam_core/i_loop_closure.h"
#include "slam_core/i_map_manager.h"
#include "slam_core/i_localization.h"
#include "algorithms/visual/visual_constraint_source.h"

namespace rosiwit_slam {

enum class PipelineMode { IDLE, MAPPING, LOCALIZATION };

/// 后端任务 (建图线程的输入).
enum class BackendTask { ADD_KF, MAP_SUBMAP, LOOP_CLOSURE, OPTIMIZE, STOP };
struct BackendItem {
    BackendTask task;
    KeyFrame kf;
};

/// 后端优化后回写的前沿锚点. 独立 shared 状态: 旧 worker 即使晚于 pipeline 退出,
/// 也只会写自己那份 state, 不会碰新实例或已析构的 SlamPipeline.
struct AnchorState {
    std::mutex mu;
    PoseStamped anchor;
    bool have = false;
};

/// 自包含的后端线程. 只持有 shared_ptr<IBackend/IMapManager/AnchorState>,
/// 不引用 SlamPipeline 的成员; reset/析构用 stopAsync() 放弃线程即可, 不会 use-after-free.
///
/// 线程与队列都用 rosiwit_common 的基础层:
///   * 主队列 / 回环队列 = 有界 Channel (满时丢最旧, 与历史行为一致)
///   * 两条线程 = Thread (body 捕获 shared_ptr self, 保证放弃后状态仍活)
///   * OPTIMIZE 不占队列: 原子标志 + 一条 "唤醒项", 队列空时才真正跑优化
class BackendWorker : public std::enable_shared_from_this<BackendWorker> {
public:
    BackendWorker(std::shared_ptr<IBackend> backend,
                  std::shared_ptr<IMapManager> map_mgr,
                  std::shared_ptr<ILoopClosure> loop,
                  std::shared_ptr<AnchorState> anchor);
    void start();
    void push(BackendTask task, const KeyFrame& kf = KeyFrame{});
    /// 仍存活的后台线程数 (abandon 之后用它做有界等待)
    int inflight() const { return m_inflight.load(); }
    /// 置停止并 detach; 正在执行的 optimize 会自然结束, 不阻塞调用线程.
    void stopAsync();

private:
    void run(rosiwit_common::CancelToken token);
    void runLoop(rosiwit_common::CancelToken token);
    void requestOptimize();

    void optimizeAndPublishAnchor();

    std::shared_ptr<IBackend>    m_backend;
    std::shared_ptr<IMapManager> m_map_mgr;
    std::shared_ptr<ILoopClosure> m_loop;
    std::shared_ptr<AnchorState> m_anchor;

    std::unique_ptr<rosiwit_common::Thread> m_thread;
    rosiwit_common::Channel<BackendItem> m_queue{500, rosiwit_common::OverflowPolicy::kDropOldest};

    // 回环检测单独线程: ScanContext + GICP 很重, 绝不能和 ADD_KF/MAP_SUBMAP
    // 共用同一个 worker, 否则一次 top-K ICP 会把后端队列堵满并丢关键帧.
    std::unique_ptr<rosiwit_common::Thread> m_loop_thread;
    rosiwit_common::Channel<KeyFrame> m_loop_queue{200, rosiwit_common::OverflowPolicy::kDropOldest};
    int m_loop_lc_keyframes = 0;
    // OPTIMIZE 不占队列: 合并成 pending 标志, 等主队列排空后必跑一次,
    // 避免"后端优化被 queue full 静默吞掉".
    std::atomic<bool> m_optimize_requested{false};
    std::atomic<int> m_inflight{0};     // 两条线程的存活计数 (abandon 后仍可观测)
};

class SlamPipeline : public SlamBase {
public:
    bool init(const std::string& config_path) override;
    void onImu(const IMUSample& s) override;
    void onOdom(const PoseStamped& odom) override;
    void onLidar(const LidarFrame& f) override;
    std::string name() const override { return m_pipeline_name; }
    bool getGlobalMap(PointVec& out) override;
    SlamState state() const override;

    bool saveMap(const std::string& path) override;
    bool loadMap(const std::string& path) override;
    bool loadLocalizationMapData(const MapData& data) override;
    bool exportMapData(MapData& out) override;
    void cancelMapLoad() override;
    bool appendMap(const std::string& path) override;
    bool resetForNewMapping(const std::string& config_path) override;
    bool saveGridMap(const std::string& pgm_path, const std::string& yaml_path, double resolution) override;
    /// 与 saveGridMap 同一条生成路径 (逐帧观测 + 地面处理), 结果给实时 /map 用。
    bool getRealtimeGrid(double resolution, rosiwit_slam::GridInfo& out) override;
    /// 按需触发一次全局重定位 (转发给定位流水线)
    void requestRelocalization() override;
    bool setGridMapGenerator(const std::string& name) override;
    bool setPipelineMode(const std::string& mode) override;
    std::string getPipelineMode() const override;
    ILocalization::Status localizationStatus() const override;
    bool setInitPose(const PoseStamped& pose) override;
    GridInfo getGridInfo() const override;
    uint64_t getRevision() const override;
    bool getHighRatePose(double t, PoseStamped& out) override;
    bool getPoseCov(PoseStamped& out, Eigen::Matrix<double, 6, 6>& cov) override;
    double getMatchingDegree() const override;    void onAbsolute(const PoseStamped& pose, const Eigen::Matrix<double, 6, 6>& cov) override;
    /// 运动先验的等效不确定度 (米/弧度)
    bool priorUncertainty(double& xy_sigma, double& yaw_sigma) const override;

    PipelineMode getMode() const { return m_mode.load(); }
    void setMode(PipelineMode mode);
    bool loadMapForLocalization(const std::string& path, const PoseStamped& init_pose);

    // 视觉松耦合 (route b): 外部 VO 库输出 -> 后端因子
    void onVisualOdometry(double time, const PoseStamped& vo_pose) override;
    void onVisualLoop(const std::string& from_id, const std::string& to_id,
                      const PoseStamped& rel, double cov) override;

public:
    bool onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) override;
    bool handleMapping(const SyncPackage& pkg, SlamOutput& out);

    std::string m_pipeline_name;
    int m_frame_count = 0;

    ~SlamPipeline() override;

private:
    void pushBackendTask(BackendTask task, const KeyFrame& kf = KeyFrame{});
    void handleLocalization(const LidarFrame& f);
    void localizationWorker(rosiwit_common::CancelToken token);
    /// 停止定位 worker (join) 并丢弃残留帧; 幂等
    void stopLocalizationWorker();

    // 定位模式重处理队列: sensor callback 只入队, 独立 worker 消费.
    // 全局重定位/GICP 可能持续数秒，绝不能占用 ROS spinner 或 50Hz 泵。
    // 容量 2 + 丢最旧: GICP 跟不上时宁可处理最新帧 (旧帧对跟踪没用)。
    rosiwit_common::Channel<LidarFrame> m_loc_buf{2, rosiwit_common::OverflowPolicy::kDropOldest};
    std::unique_ptr<rosiwit_common::Thread> m_loc_worker;

    // 核心只保存通用 PCD/PGM 等；xzmap 导出由 ROS1 适配层完成。

    std::unique_ptr<IFrontend>     m_frontend;
    std::shared_ptr<ILoopClosure>  m_loop;
    std::unique_ptr<ILocalization> m_localization;
    std::unique_ptr<VisualConstraintSource> m_visual;   // 视觉松耦合因子源
    VisualConstraintConfig m_visual_cfg;
    std::atomic<PipelineMode> m_mode{PipelineMode::IDLE};
    std::atomic<bool> m_localization_map_loaded{false};
    /// 上次触发后端的 m_frame_count (防止"计数停在 50 的倍数"时每帧都优化)
    size_t m_last_optimize_kf_ = 0;

    // 雷达 -> IMU 外参 (ROS1 override YAML 顶层 r_il/t_il);
    // 定位模式输出点云前需要先把扫描转到 IMU 系。
    bool m_lidar_to_imu_valid = false;
    Eigen::Matrix4f m_lidar_to_imu = Eigen::Matrix4f::Identity();
    // 后端: 主线程与 worker 共享 backend/map_mgr; worker 自带队列与生命周期.
    std::shared_ptr<IBackend>      m_backend;
    std::shared_ptr<IMapManager>   m_map_mgr;
    std::shared_ptr<AnchorState>   m_anchor;
    std::shared_ptr<BackendWorker> m_backend_worker;
    /// 后端两条线程的存活计数 (abandon 之后仍可能 >0)
    std::atomic<int> m_backend_inflight{0};
    bool waitForBackgroundIdle(int timeout_ms) override;
};

} // namespace rosiwit_slam
