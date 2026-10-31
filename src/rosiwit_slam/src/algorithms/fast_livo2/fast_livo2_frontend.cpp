// src/algorithms/fast_livo2/fast_livo2_frontend.cpp
#include "algorithms/fast_livo2/fast_livo2_frontend.h"

#include <fstream>

namespace rosiwit_slam {

FastLivo2Frontend::FastLivo2Frontend() = default;
FastLivo2Frontend::~FastLivo2Frontend() = default;

bool FastLivo2Frontend::init(const std::string& config_path) {
    // Full config parsing lands in the next task.
    std::ifstream f(config_path);
    return f.good();
}

void FastLivo2Frontend::onImu(const IMUSample& /*s*/) {}
void FastLivo2Frontend::onLidar(const LidarFrame& /*f*/) {}
void FastLivo2Frontend::onImage(const ImageFrame& /*f*/) {}

bool FastLivo2Frontend::getOdometry(PoseStamped& /*out*/) { return false; }

bool FastLivo2Frontend::getClouds(CloudType::Ptr& /*body*/, CloudType::Ptr& /*world*/) {
    return false;
}

bool FastLivo2Frontend::getKeyFrame(KeyFrame& /*out*/) { return false; }
bool FastLivo2Frontend::getGlobalMap(PointVec& /*out*/) { return false; }

bool FastLivo2Frontend::getRGBCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& /*out*/) {
    return false;
}

}  // namespace rosiwit_slam
