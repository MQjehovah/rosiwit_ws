#pragma once
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <deque>
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
/// 不引用 SlamPipeline 的成员; reset/析构用 stopAsync() 分离线程即可, 不会 use-after-free.
class BackendWorker : public std::enable_shared_from_this<BackendWorker> {
public:
    BackendWorker(std::shared_ptr<IBackend> backend,
                  std::shared_ptr<IMapManager> map_mgr,
                  std::shared_ptr<ILoopClosure> loop,
                  std::shared_ptr<AnchorState> anchor);
    void start();
    void push(BackendTask task, const KeyFrame& kf = KeyFrame{});
    /// 置停止并 detach; 正在执行的 optimize 会自然结束, 不阻塞调用线程.
    void stopAsync();

private:
    void run();

    void optimizeAndPublishAnchor();

    std::shared_ptr<IBackend>    m_backend;
    std::shared_ptr<IMapManager> m_map_mgr;
    std::shared_ptr<ILoopClosure> m_loop;
    std::shared_ptr<AnchorState> m_anchor;
    int m_lc_keyframes = 0;

    std::thread m_thread;
    std::mutex m_mu;
    std::condition_variable m_cv;
    std::deque<BackendItem> m_queue;
    bool m_stop = false;
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
    bool loadMapXzmapOnly(const std::string& path) override;
    bool saveMapXzmapOnly(const std::string& path) override;
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
    bool setInitPose(const PoseStamped& pose) override;
    GridInfo getGridInfo() const override;
    uint64_t getRevision() const override;
    bool getHighRatePose(double t, PoseStamped& out) override;
    bool getPoseCov(PoseStamped& out, Eigen::Matrix<double, 6, 6>& cov) override;
    double getMatchingDegree() const override;    void onAbsolute(const PoseStamped& pose, const Eigen::Matrix<double, 6, 6>& cov) override;

    /// 定位模式的 LiDAR 重处理入口: 传感器回调只入队, 由 work timer 轮询.
    /// 避免 GICP/重定位在 sensor callback 里阻塞 IMU 收包.
    bool tryPopLocalization();

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

    // 定位模式重处理队列: sensor callback 只入队, work timer 消费.
    std::mutex m_loc_mu;
    std::deque<LidarFrame> m_loc_buf;

    // saveMap 附带导出 map.xzmap (关键帧+优化位姿交给 io 模块落盘, SLAM 侧不感知设备格式)
    bool exportXzMap(const std::string& pcd_path);

    std::unique_ptr<IFrontend>     m_frontend;
    std::shared_ptr<ILoopClosure>  m_loop;
    std::unique_ptr<ILocalization> m_localization;
    std::unique_ptr<VisualConstraintSource> m_visual;   // 视觉松耦合因子源
    VisualConstraintConfig m_visual_cfg;
    std::atomic<PipelineMode> m_mode{PipelineMode::IDLE};
    std::atomic<bool> m_localization_map_loaded{false};

    // 后端: 主线程与 worker 共享 backend/map_mgr; worker 自带队列与生命周期.
    std::shared_ptr<IBackend>      m_backend;
    std::shared_ptr<IMapManager>   m_map_mgr;
    std::shared_ptr<AnchorState>   m_anchor;
    std::shared_ptr<BackendWorker> m_backend_worker;
};

} // namespace rosiwit_slam
