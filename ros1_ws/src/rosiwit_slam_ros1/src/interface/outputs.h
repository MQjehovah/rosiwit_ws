// ============================================================================
// outputs.cpp 的声明 — 兼容层之外的补充输出
//
// App 只调老接口 (xz_3d_slam 兼容层)。这里只剩"老接口没有、且本仓库内有人消费"的两路:
//   /lio_odom   导航位姿 (nav 的 odom_topic)
//
// /world_cloud 已删除: 导航直接订原始雷达话题并自己做 map<-lidar 变换
// (voxelCallback 本来就有 TF lookup), SLAM 侧不再重复序列化整帧点云。
// 重复项 (odom/map/cloud_map/grid_map)、无消费者的项 (body_cloud/lio_path/
// slam_calibration)、以及 5 个新服务 (save_map/load_map/merge_map/save_grid_map/
// set_slam_mode) 全部删除。
// ============================================================================
#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <atomic>
#include <mutex>

#include "aux_jobs.h"
#include "slam_algorithm.h"
#include "slam_core/pose_filter.h"

namespace rosiwit_slam_ros1 {

class Ros1OutputsInterface {
public:
    Ros1OutputsInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, SlamAlgorithm& engine);
    ~Ros1OutputsInterface();

    /// 析构前等在途池任务收尾 (任务体捕获 this)
    void waitForAuxJobs();

private:
    /// 挂在 engine 的 50Hz 泵上: 与重构前"同一个 processCB 里排空+发布"时序一致
    void onTick();
    void timerTick(const ros::TimerEvent&);   // 20ms, io 线程
    void publishPose(const rosiwit_slam::SlamOutput& out);

    /// 一次核心输出在 ROS 层的统一形态：base_link 位姿 (+ 平滑/时间戳)。
    struct PreparedPose {
        rosiwit_slam::PoseStamped pose;
        ros::Time stamp;
        bool new_pose = false;
    };
    PreparedPose preparePose(const rosiwit_slam::SlamOutput& out);
    void publishOdometry(const PreparedPose& p);
    void scheduleCalibJob();

    ros::NodeHandle nh_, pnh_;
    SlamAlgorithm& engine_;

    // **App 只调老接口** (xz_3d_slam 兼容层)。这里只剩"老接口没有、且本仓库内有人消费"
    // 的输出: /lio_odom 与 /world_cloud (导航)。重复项 (odom/map/cloud_map/grid_map)、
    // 无消费者的项 (body_cloud/lio_path/slam_calibration)、以及 5 个新服务全部删除。
    ros::Publisher odom_pub_;
    ros::Timer tick_timer_;
    ros::Time last_calib_update_;

    // 位姿输出平滑 (alpha-beta 因果滤波), 与 ROS2 节点行为一致:
    // 只平滑发布位姿/轨迹, 不改变核心状态. filter_alpha=0 表示关闭.
    rosiwit_slam::PoseOutputFilter out_filter_;
    double out_filter_alpha_ = 0.0;
    double out_filter_beta_ = 0.2;
    double out_filter_extrap_s_ = 0.3;
    double last_pose_time_ = -1.0;      // 同一核心位姿不重复做点云深拷贝

    std::atomic<bool> calib_busy_{false};
    AuxJobGuard aux_jobs_;
};

} // namespace rosiwit_slam_ros1
