#pragma once
#include <string>
#include "slam_core/i_map_manager.h"

namespace rosiwit_slam {

class PcdMapManager : public IMapManager {
public:
    PcdMapManager();
    ~PcdMapManager() override = default;

    bool init(const std::string& config_path) override;
    bool saveMap(const std::string& name) override;
    bool loadMap(const std::string& name) override;
    bool getGlobalMap(PointVec& out) override;
    bool addSubMap(const KeyFrame& kf) override;

private:
    CloudType::Ptr global_map_;
    std::string map_dir_;
    bool initialized_ = false;
};

} // namespace rosiwit_slam
