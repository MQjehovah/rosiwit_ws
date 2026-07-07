#pragma once
#include <memory>
#include "slam_core/i_frontend.h"
#include "commons.h"
#include "map_builder.h"
#include "ieskf.h"

namespace rosiwit_slam {

class FastLio2Frontend : public IFrontend {
public:
    bool init(const std::string& config_path) override;
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;
    bool getOdometry(PoseStamped& out) override;
    bool getClouds(CloudType::Ptr& body, CloudType::Ptr& world) override;
    bool getKeyFrame(KeyFrame& out) override;
    bool getGlobalMap(PointVec& out) override;
    bool isMapping() const;

private:
    std::shared_ptr<IESKF>      m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    Config                      m_config;
    bool m_inited = false;
    bool m_first_after_mapping = false;
};

} // namespace rosiwit_slam
