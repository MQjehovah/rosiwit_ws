// src/slam_core/slam_factory.cpp
#include "slam_core/slam_factory.h"
#include "slam_core/slam_pipeline.h"
#include "slam_core/i_backend.h"
#include "algorithms/fast_lio2/fast_lio2_frontend.h"
#include "algorithms/ceres_backend/ceres_backend.h"
#include "algorithms/gicp_localization/gicp_localization.h"
#include "algorithms/pcd_map_manager/pcd_map_manager.h"
// 新增算法
#include "algorithms/point_lio/point_lio_frontend.h"
#include "algorithms/scan_context/scan_context_lc.h"

#ifdef USE_GTSAM
#include "algorithms/gtsam_backend/gtsam_isam2_backend.h"
#endif

namespace rosiwit_slam {

std::unique_ptr<ISlamAlgorithm> SlamFactory::create(const std::string& name) {
    if (name == "fast_lio2_pipeline") return std::make_unique<SlamPipeline>();
    return nullptr;
}

std::vector<std::string> SlamFactory::listNames() {
    return { "fast_lio2_pipeline" };
}

std::unique_ptr<IFrontend> SlamFactory::createFrontend(const std::string& name) {
    if (name == "fast_lio2_frontend") return std::make_unique<FastLio2Frontend>();
    if (name == "point_lio_frontend")  return std::make_unique<PointLioFrontend>();
    return nullptr;
}

std::unique_ptr<IBackend> SlamFactory::createBackend(const std::string& name) {
    if (name == "ceres_pose_graph") return std::make_unique<CeresBackend>();
#ifdef USE_GTSAM
    if (name == "gtsam_isam2")    return std::make_unique<GtsamIsam2Backend>();
#endif
    return nullptr;
}

std::unique_ptr<ILocalization> SlamFactory::createLocalization(const std::string& name) {
    if (name == "gicp_localization") return std::make_unique<GicpLocalization>();
    return nullptr;
}

std::unique_ptr<IMapManager> SlamFactory::createMapManager(const std::string& name) {
    if (name == "pcd_map_manager") return std::make_unique<PcdMapManager>();
    return nullptr;
}

std::unique_ptr<ILoopClosure> SlamFactory::createLoopClosure(const std::string& name) {
    if (name == "scan_context")  return std::make_unique<ScanContextLoopClosure>();
    return nullptr;
}

} // namespace rosiwit_slam
