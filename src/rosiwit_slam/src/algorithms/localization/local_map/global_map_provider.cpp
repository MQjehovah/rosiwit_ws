// src/algorithms/localization/local_map/global_map_provider.cpp
#include "algorithms/localization/local_map/global_map_provider.h"
#include "algorithms/localization/localization_utils.h"

namespace rosiwit_slam {

void GlobalMapProvider::setMap(const CloudType::Ptr& global_map) {
    m_map = downsampleCloud(filterFinite(global_map), m_cfg.voxel_resolution);
}

CloudType::Ptr GlobalMapProvider::getLocalMap(const PoseStamped& pose) {
    (void)pose;
    if (!m_map) return CloudType::Ptr(new CloudType());
    return m_map;
}

} // namespace rosiwit_slam
