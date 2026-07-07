// src/slam_core/slam_factory.cpp
#include "slam_core/slam_factory.h"
#include "slam_core/slam_pipeline.h"
#include "slam_core/i_backend.h"
#include "algorithms/fast_lio2/fast_lio2_algorithm.h"
#include "algorithms/fast_lio2/fast_lio2_frontend.h"
#include "algorithms/ceres_backend/ceres_backend.h"

namespace rosiwit_slam {

std::unique_ptr<ISlamAlgorithm> SlamFactory::create(const std::string& name) {
    if (name == "fast_lio2") return std::make_unique<FastLio2Algorithm>();
    if (name == "fast_lio2_pipeline") return std::make_unique<SlamPipeline>();
    return nullptr;
}

std::vector<std::string> SlamFactory::listNames() {
    return { "fast_lio2", "fast_lio2_pipeline" };
}

std::unique_ptr<IFrontend> SlamFactory::createFrontend(const std::string& name) {
    if (name == "fast_lio2_frontend") return std::make_unique<FastLio2Frontend>();
    return nullptr;
}

std::unique_ptr<IBackend> SlamFactory::createBackend(const std::string& name) {
    if (name == "ceres_pose_graph") return std::make_unique<CeresBackend>();
    return nullptr;
}

} // namespace rosiwit_slam
