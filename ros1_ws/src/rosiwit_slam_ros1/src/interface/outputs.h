// ============================================================================
// RosiwitInterface —— rosiwit_slam 原有的"新接口"(计划 T4)
//
// 全部从原 rosiwit_slam_ros1_node.cpp 搬过来, 内容逐字保留:
//   发布: lio_odom / lio_path / body_cloud / world_cloud / cloud_map / grid_map
//         (`odom` 和 `/map` 由兼容接口负责)
//   TF:   兼容接口负责 map->odom；本接口不重复发布
//   服务: save_map / load_map / save_grid_map / set_slam_mode
//
// 兼容接口冲突输出抑制:
//   - 不发 `odom` 别名, 不发任何 TF —— 旧接口要自己发 map->odom 动态 TF, 抢同一个
//     父子边会打架; 而 odom->base_link 在旧系统里由 robot_localization 负责。
//   - **不发布 /map** —— /map 归兼容接口独占 (只有一个 /map 发布者,
//     建图=周期实时栅格, 定位=锁存全局栅格)。
// 其它话题 (lio_odom / lio_path / cloud_map / grid_map / body_cloud / world_cloud) 照旧,
// 因为仓库里 perception 等模块订阅的就是 /lio_odom。
// ============================================================================
#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

#include "rosiwit_slam_ros1/LoadMap.h"
#include "rosiwit_slam_ros1/SaveGridMap.h"
#include "rosiwit_slam_ros1/SaveMap.h"
#include "rosiwit_slam_ros1/SetSlamMode.h"

#include "slam_engine.h"
#include "slam_core/pose_filter.h"

namespace rosiwit_slam_ros1 {

class RosiwitInterface {
public:
    RosiwitInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, SlamEngine& engine,
                     bool suppress_conflicts = false);

private:
    /// 挂在 engine 的 50Hz 泵上: 与重构前"同一个 processCB 里排空+发布"时序一致
    void onTick();
    void publishPose(const rosiwit_slam::SlamOutput& out);

    /// 一次核心输出在 ROS 层的统一形态：base_link 位姿 + 可选 body_cloud。
    struct PreparedPose {
        rosiwit_slam::PoseStamped pose;
        rosiwit_slam::CloudType::Ptr body_cloud;
        ros::Time stamp;
        bool new_pose = false;
    };
    PreparedPose preparePose(const rosiwit_slam::SlamOutput& out);
    void publishOdometry(const PreparedPose& p);
    void publishTransform(const PreparedPose& p);
    void publishClouds(const rosiwit_slam::SlamOutput& out, const PreparedPose& p);
    void publishPath(const PreparedPose& p);

    void mapCB(const ros::TimerEvent&);

    bool handleSaveMap(rosiwit_slam_ros1::SaveMap::Request& req,
                       rosiwit_slam_ros1::SaveMap::Response& res);
    bool handleLoadMap(rosiwit_slam_ros1::LoadMap::Request& req,
                       rosiwit_slam_ros1::LoadMap::Response& res);
    bool handleMergeMap(rosiwit_slam_ros1::LoadMap::Request& req,
                        rosiwit_slam_ros1::LoadMap::Response& res);
    bool handleSaveGridMap(rosiwit_slam_ros1::SaveGridMap::Request& req,
                           rosiwit_slam_ros1::SaveGridMap::Response& res);
    bool handleSetMode(rosiwit_slam_ros1::SetSlamMode::Request& req,
                       rosiwit_slam_ros1::SetSlamMode::Response& res);

    ros::NodeHandle nh_, pnh_;
    SlamEngine& engine_;
    const bool suppress_conflicts_;   // 冲突输出抑制开关

    ros::Publisher odom_pub_, odom_nav_pub_, path_pub_, body_cloud_pub_, world_cloud_pub_,
                   global_map_pub_, grid_map_pub_, map_pub_, calib_pub_;
    ros::ServiceServer srv_save_map_, srv_load_map_, srv_merge_map_, srv_save_grid_, srv_set_mode_;
    ros::Timer map_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;

    bool grid_published_ = false;
    double realtime_grid_resolution_ = 0.3;  // 实时栅格分辨率 (= mapping.grid.realtime_resolution)
    // 位姿输出平滑 (alpha-beta 因果滤波), 与 ROS2 节点行为一致:
    // 只平滑发布位姿/轨迹, 不改变核心状态. filter_alpha=0 表示关闭.
    rosiwit_slam::PoseOutputFilter out_filter_;
    double out_filter_alpha_ = 0.0;
    double out_filter_beta_ = 0.2;
    double out_filter_extrap_s_ = 0.3;
    nav_msgs::Path path_;
    double last_pose_time_ = -1.0;      // 同一核心位姿不重复做点云深拷贝/入轨迹
    ros::Time last_path_pub_;           // lio_path 整条重发节流 (50Hz -> 1Hz)
    static constexpr size_t kMaxPathPoses = 10000;
};

} // namespace rosiwit_slam_ros1
