// src/algorithms/fast_livo2/fast_livo2_frontend.cpp
#include "algorithms/fast_livo2/fast_livo2_frontend.h"
#include "algorithms/fast_livo2/parse_config.h"

#include <iostream>

namespace rosiwit_slam {

FastLivo2Frontend::FastLivo2Frontend() = default;
FastLivo2Frontend::~FastLivo2Frontend() = default;

bool FastLivo2Frontend::init(const std::string& config_path) {
    std::string err;
    if (!parseFastLivo2Config(config_path, m_config, err)) {
        std::cerr << "[FastLivo2Frontend] init failed: " << err << std::endl;
        return false;
    }
    m_inited = true;
    return true;
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
