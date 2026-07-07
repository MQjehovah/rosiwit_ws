#pragma once
#include <memory>
#include <deque>
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

    std::deque<IMUSample> m_imu_buffer;
    CloudType::Ptr m_current_cloud;
    double m_last_lidar_time = 0.0;
    PoseStamped m_last_kf_pose;
    bool m_has_last_kf = false;
    size_t m_frame_counter = 0;
    double m_keyframe_trans_thresh = 1.0;
    double m_keyframe_rot_thresh = 0.17;
};

} // namespace rosiwit_slam
