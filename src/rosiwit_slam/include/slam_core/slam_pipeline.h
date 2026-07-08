#pragma once
#include <memory>
#include <atomic>
#include "slam_core/i_slam_algorithm.h"
#include "slam_core/slam_base.h"
#include "slam_core/i_frontend.h"
#include "slam_core/i_backend.h"
#include "slam_core/i_loop_closure.h"
#include "slam_core/i_map_manager.h"
#include "slam_core/i_localization.h"

namespace rosiwit_slam {

// 运行模式
enum class PipelineMode { IDLE, MAPPING, LOCALIZATION };

class SlamPipeline : public SlamBase {
public:
    bool init(const std::string& config_path) override;
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;
    std::string name() const override { return m_pipeline_name; }
    bool getGlobalMap(PointVec& out) override;
    SlamState state() const override;

    // === 模式控制 (供 ROS 服务调用) ===
    PipelineMode getMode() const { return m_mode.load(); }
    void setMode(PipelineMode mode);
    bool loadMapForLocalization(const std::string& path, const PoseStamped& init_pose);

public:
    bool onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) override;
    bool handleMapping(const SyncPackage& pkg, SlamOutput& out);

    // 模块指针 (public 供 SlamNode 服务访问)
    std::unique_ptr<IFrontend>     m_frontend;
    std::unique_ptr<IBackend>      m_backend;
    std::unique_ptr<ILoopClosure>  m_loop;
    std::unique_ptr<IMapManager>   m_map_mgr;
    std::unique_ptr<ILocalization> m_localization;

    std::string m_pipeline_name;
    int m_frame_count = 0;

private:
    std::atomic<PipelineMode> m_mode{PipelineMode::IDLE};
};

} // namespace rosiwit_slam
