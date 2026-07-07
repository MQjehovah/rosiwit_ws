#pragma once
#include <memory>
#include "slam_core/i_slam_algorithm.h"
#include "slam_core/slam_base.h"
#include "slam_core/i_frontend.h"
#include "slam_core/i_backend.h"
#include "slam_core/i_loop_closure.h"
#include "slam_core/i_map_manager.h"
#include "slam_core/i_localization.h"

namespace rosiwit_slam {

class SlamPipeline : public SlamBase {
public:
    bool init(const std::string& config_path) override;
    void onImu(const IMUSample& s) override;   // 重写: local 模式直接喂定位模块
    void onLidar(const LidarFrame& f) override; // 重写: local 模式绕过 SlamBase 同步
    std::string name() const override { return m_pipeline_name; }
    bool getGlobalMap(PointVec& out) override;
    SlamState state() const override;

public:
    bool onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) override;
    bool handleMapping(const SyncPackage& pkg, SlamOutput& out);
    bool handleLocalization(const SyncPackage& pkg, SlamOutput& out);

    std::unique_ptr<IFrontend>     m_frontend;
    std::unique_ptr<IBackend>      m_backend;
    std::unique_ptr<ILoopClosure>  m_loop;
    std::unique_ptr<IMapManager>   m_map_mgr;
    std::unique_ptr<ILocalization> m_localization;

    std::string m_pipeline_name;
    int m_frame_count = 0;
    bool m_is_localization_mode = false;
    PoseStamped m_loc_last_pose;
    bool m_loc_initialized = false;
};

} // namespace rosiwit_slam
