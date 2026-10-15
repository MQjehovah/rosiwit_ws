#pragma once
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
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
    bool saveGridMap(const std::string& pgm_path, const std::string& yaml_path, double resolution) override;
    bool setGridMapGenerator(const std::string& name) override;
    bool setPipelineMode(const std::string& mode) override;
    std::string getPipelineMode() const override;
    bool setInitPose(const PoseStamped& pose) override;
    GridInfo getGridInfo() const override;
    bool getHighRatePose(double t, PoseStamped& out) override;

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
    // 后端工作队列
    enum class BackendTask { ADD_KF, OPTIMIZE, STOP };
    void backendThreadLoop();
    void pushBackendTask(BackendTask task, KeyFrame kf = {});

    // saveMap 附带导出 map.xzmap (关键帧+优化位姿交给 io 模块落盘, SLAM 侧不感知设备格式)
    bool exportXzMap(const std::string& pcd_path);

    std::unique_ptr<IFrontend>     m_frontend;
    std::unique_ptr<IBackend>      m_backend;
    std::unique_ptr<ILoopClosure>  m_loop;
    std::unique_ptr<IMapManager>   m_map_mgr;
    std::unique_ptr<ILocalization> m_localization;
    std::unique_ptr<VisualConstraintSource> m_visual;   // 视觉松耦合因子源
    VisualConstraintConfig m_visual_cfg;
    std::atomic<PipelineMode> m_mode{PipelineMode::IDLE};

    // 后端线程
    std::thread m_backend_thread;
    std::mutex m_queue_mutex;
    std::condition_variable m_queue_cv;
    bool m_backend_running = false;

    // 实时回环修正: 后端优化后由地图管理器提供的前沿锚点 A, 输出位姿/轨迹用它转到修正系
    std::mutex m_anchor_mutex;
    PoseStamped m_out_anchor_;   // 初始为单位变换
    bool m_have_anchor_ = false;

    struct BackendItem {
        BackendTask task;
        KeyFrame kf;
    };
    std::queue<BackendItem> m_backend_queue;
};

} // namespace rosiwit_slam
