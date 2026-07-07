#pragma once
#include <string>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include "slam_core/i_localization.h"

namespace rosiwit_slam {

class GicpLocalization : public ILocalization {
public:
    GicpLocalization();
    ~GicpLocalization() override = default;

    bool init(const std::string& config_path) override;
    void setMap(const std::string& map_name) override;
    void setInitPose(const PoseStamped& pose) override;
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;
    Status getStatus() override;
    bool getPose(PoseStamped& out) override;

private:
    CloudType::Ptr downsampleCloud(const CloudType::Ptr& cloud, float resolution);

    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp_;
    CloudType::Ptr map_cloud_;
    CloudType::Ptr downsampled_map_;
    pcl::VoxelGrid<PointType> map_voxel_;
    pcl::VoxelGrid<PointType> scan_voxel_;

    PoseStamped current_pose_;
    Status status_ = INIT;
    bool first_scan_ = true;

    float voxel_resolution_ = 0.5f;
    float fitness_threshold_ = 10.0f;
    int max_iterations_ = 64;
    double transformation_epsilon_ = 1e-8;
    bool has_map_ = false;
};

} // namespace rosiwit_slam
