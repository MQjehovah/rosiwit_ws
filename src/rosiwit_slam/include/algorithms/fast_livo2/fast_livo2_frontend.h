// include/algorithms/fast_livo2/fast_livo2_frontend.h
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "algorithms/fast_livo2/fast_livo2_config.h"
#include "slam_core/i_frontend.h"

namespace rosiwit_slam {

class LivoMapper;

/// FAST-LIVO2 前端（LiDAR-惯性-视觉紧耦合，ROS 无关）。
///
/// 输入：IMUSample / LidarFrame（curvature = 逐点时间，ms）/ ImageFrame。
/// 输出：里程计、当前帧 body/world 点云、关键帧、当前帧彩色世界系点云。
class FastLivo2Frontend : public IFrontend {
public:
    FastLivo2Frontend();
    ~FastLivo2Frontend() override;

    bool init(const std::string& config_path) override;
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;
    void onImage(const ImageFrame& f) override;
    bool getOdometry(PoseStamped& out) override;
    bool getClouds(CloudType::Ptr& body, CloudType::Ptr& world) override;
    bool getKeyFrame(KeyFrame& out) override;
    bool getGlobalMap(PointVec& out) override;
    bool getRGBCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out) override;

private:
    bool shouldMakeKeyframe(const PoseStamped& pose);

    FastLivo2Config m_config;
    std::unique_ptr<LivoMapper> m_mapper;
    bool m_inited = false;

    std::mutex m_mtx;
    PoseStamped m_last_keyframe_pose;
    bool m_has_last_keyframe = false;
    std::uint64_t m_keyframe_counter = 0;
};

}  // namespace rosiwit_slam
