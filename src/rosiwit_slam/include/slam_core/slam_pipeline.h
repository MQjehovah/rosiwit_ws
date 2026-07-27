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

namespace rosiwit_slam {

enum class PipelineMode { IDLE, MAPPING, LOCALIZATION };

class SlamPipeline : public SlamBase {
public:
    bool init(const std::string& config_path) override;
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;
    std::string name() const override { return m_pipeline_name; }
    bool getGlobalMap(PointVec& out) override;
    SlamState state() const override;

    bool saveMap(const std::string& path) override;
    bool loadMap(const std::string& path) override;
    bool saveGridMap(const std::string& pgm_path, const std::string& yaml_path, double resolution) override;
    bool setPipelineMode(const std::string& mode) override;
    std::string getPipelineMode() const override;
    bool setInitPose(const PoseStamped& pose) override;
    GridInfo getGridInfo() const override;

    PipelineMode getMode() const { return m_mode.load(); }
    void setMode(PipelineMode mode);
    bool loadMapForLocalization(const std::string& path, const PoseStamped& init_pose);

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

    std::unique_ptr<IFrontend>     m_frontend;
    std::unique_ptr<IBackend>      m_backend;
    std::unique_ptr<ILoopClosure>  m_loop;
    std::unique_ptr<IMapManager>   m_map_mgr;
    std::unique_ptr<ILocalization> m_localization;
    std::atomic<PipelineMode> m_mode{PipelineMode::IDLE};

    // 后端线程
    std::thread m_backend_thread;
    std::mutex m_queue_mutex;
    std::condition_variable m_queue_cv;
    bool m_backend_running = false;

    struct BackendItem {
        BackendTask task;
        KeyFrame kf;
    };
    std::queue<BackendItem> m_backend_queue;
};

} // namespace rosiwit_slam
