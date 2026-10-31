// src/algorithms/fast_livo2/livo_mapper.h
#pragma once

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "IMU_Processing.h"
#include "algorithms/fast_livo2/fast_livo2_config.h"
#include "slam_core/slam_types.h"
#include "vio.h"
#include "voxel_map.h"

namespace rosiwit_slam {

/// FAST-LIVO2 的算法主体（ROS 无关）：LiDAR-惯性-视觉紧耦合。
///
/// 输入通过 addImu/addLidar/addImage 入队，process() 按 FAST-LIVO2 的
/// WAIT→LIO→VIO 状态机推进；输出通过 getter 读取（不发布任何 ROS 消息）。
class LivoMapper {
public:
    LivoMapper() = default;
    ~LivoMapper();

    bool init(const FastLivo2Config& config);

    void addImu(const IMUSample& sample);
    void addLidar(const LidarFrame& frame);
    void addImage(const ImageFrame& frame);

    /// 推进所有可处理的帧；返回 true 表示至少产生了一次位姿更新。
    bool process();

    bool hasPose() const { return has_pose_; }
    PoseStamped pose() const;
    CloudType::Ptr bodyCloud() const;
    CloudType::Ptr worldCloud() const;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud() const;

private:
    bool sync_packages(LidarMeasureGroup& meas);
    void handleFirstFrame();
    void gravityAlignment();
    void processImu();
    void stateEstimationAndMapping();
    void handleVIO();
    void handleLIO();
    void buildOutputs();
    void updatePoseFromState(double stamp);

    void transformLidar(const Eigen::Matrix3d& rot, const Eigen::Vector3d& t,
                        const PointCloudXYZI::Ptr& input_cloud, PointCloudXYZI::Ptr& trans_cloud);
    void RGBpointBodyToWorld(PointType const* const pi, PointType* const po);

    bool createCamera(const CameraIntrinsics& cam);

    FastLivo2Config cfg_;
    bool inited_ = false;

    // buffers
    std::mutex mtx_buffer_;
    std::condition_variable sig_buffer_;
    std::deque<PointCloudXYZI::Ptr> lid_raw_data_buffer_;
    std::deque<double> lid_header_time_buffer_;
    std::deque<rosiwit_slam::IMUSample> imu_buffer_;
    std::deque<cv::Mat> img_buffer_;
    std::deque<double> img_time_buffer_;

    // timing
    double last_timestamp_lidar_ = -1.0;
    double last_timestamp_imu_ = -1.0;
    double last_timestamp_img_ = -1.0;
    double first_lidar_time_ = 0.0;
    double latest_ekf_time_ = 0.0;

    // state
    SLAM_MODE slam_mode_ = ONLY_LIO;
    LidarMeasureGroup LidarMeasures_;
    StatesGroup state_;
    StatesGroup state_propagat_;
    Eigen::Vector3d extT_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d extR_ = Eigen::Matrix3d::Identity();

    bool is_first_frame_ = false;
    bool lidar_map_inited_ = false;
    bool lidar_pushed_ = false;
    bool gravity_align_finished_ = false;
    bool state_update_flg_ = false;
    int feats_down_size_ = 0;
    int frame_num_ = 0;
    double aver_time_consu_ = 0.0;
    double first_lidar_time_offset_ = 0.0;
    Eigen::Vector3d euler_cur_ = Eigen::Vector3d::Zero();
    std::vector<pointWithVar> pv_list_;

    // clouds
    PointCloudXYZI::Ptr feats_undistort_;
    PointCloudXYZI::Ptr feats_down_body_;
    PointCloudXYZI::Ptr feats_down_world_;
    PointCloudXYZI::Ptr pcl_w_wait_pub_;
    PointCloudXYZI::Ptr pcl_wait_pub_;
    PointCloudXYZRGB::Ptr rgb_cloud_;
    bool has_rgb_ = false;

    // outputs
    mutable std::mutex mtx_output_;
    PoseStamped latest_pose_;
    bool has_pose_ = false;
    CloudType::Ptr out_body_;
    CloudType::Ptr out_world_;

    pcl::VoxelGrid<PointType> downSizeFilterSurf_;

    std::shared_ptr<ImuProcess> p_imu_;
    std::shared_ptr<VIOManager> vio_manager_;
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*> voxel_map_;
    std::shared_ptr<VoxelMapManager> voxelmap_manager_;
};

}  // namespace rosiwit_slam
