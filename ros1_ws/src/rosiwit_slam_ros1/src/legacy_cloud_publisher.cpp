// ============================================================================
// legacy 点云与栅格发布侧: 输入回调 + 定时发布 + 实时栅格 —— 从 legacy_xz3dslam_interface.cpp 按关注点拆出 (2026-10-18 薄层整合,
// 纯代码搬移, 行为与拆分前逐字一致; 分区方式对照 ros2_ws SlamNode)。
// ============================================================================
#include "legacy_xz3dslam_interface.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "legacy_convert.h"
#include "legacy_xz3dslam_internal.h"

using namespace rosiwit_slam;
using namespace rosiwit_slam_ros1::legacy_internal;

namespace rosiwit_slam_ros1 {
void LegacyXz3dSlamInterface::laserScanCB(const sensor_msgs::LaserScan::ConstPtr&) {
    // 旧实现只用它做雷达频率统计 (legacy.cpp:209 -> laserScanCallBack)
    const ros::Time now = ros::Time::now();
    if (scan_window_start_.isZero()) { scan_window_start_ = now; scan_count_ = 0; return; }
    ++scan_count_;
    const double dt = (now - scan_window_start_).toSec();
    if (dt >= 1.0) {
        scan_hz_ = scan_count_ / dt;
        scan_window_start_ = now;
        scan_count_ = 0;
        std::cout << "[legacy_iface] /scan freq = " << scan_hz_ << " Hz" << std::endl;
    }
}

void LegacyXz3dSlamInterface::chassisOdomCB(const nav_msgs::Odometry::ConstPtr& msg) {
    // 旧实现无条件订阅并把它当运动先验喂给定位 (legacy.cpp:210 chassisOdomCallBack)
    engine_.onOdomMsg(msg);
    // 位姿-里程计一致性监测的输入 (同一定时器线程消费, 无锁)
    last_odom_x_ = msg->pose.pose.position.x;
    last_odom_y_ = msg->pose.pose.position.y;
    last_odom_t_ = msg->header.stamp.toSec();
}

void LegacyXz3dSlamInterface::liftStateCB(const std_msgs::String::ConstPtr& msg) {
    lift_state_ = msg->data;   // 旧实现 localiza_ptr_->setCurrLiftState(lift_state); 核心无此接口
}

void LegacyXz3dSlamInterface::liftTypeCB(const std_msgs::String::ConstPtr& msg) {
    if (lift_type_ != msg->data) {
        lift_type_ = msg->data;
        std::cout << "[legacy_iface] lift_type = " << lift_type_ << std::endl;
    }
}

void LegacyXz3dSlamInterface::onRawInputCloud(const CloudType& raw_cloud, const ros::Time& stamp) {
    CloudType::Ptr snapshot(new CloudType(raw_cloud));
    {
        std::lock_guard<std::mutex> lk(raw_input_mtx_);
        raw_input_ = snapshot;
        raw_input_stamp_ = stamp;
    }
    // origin_point: 每帧雷达都发 (legacy.cpp:3169/3299/3427)。
    // frame_id 用 "map" 是照抄旧实现 (xz_slam_ros1_interface.cpp:3115) —— 点其实是未配准的输入点。
    if (!snapshot->empty()) {
        point_puber_.publish(toPointCloud2(*snapshot, kMapFrame, stamp));
    }
}

// ---------------------------------------------------------------- 定时器

void LegacyXz3dSlamInterface::timerPublishLocalCloudAndSubmap(const ros::TimerEvent&) {
    startPub();   // legacy.cpp:834
}
void LegacyXz3dSlamInterface::timerPublishRealtimeGridMap(const ros::TimerEvent&) {
    if (slam_mode_ == SlamMode::MAPPING) publishRealtimeGridMap();   // legacy.cpp:836-838
}

// ---------------------------------------------------------------- 0.1s 分发

void LegacyXz3dSlamInterface::startPub() {
    updateModeFromCore();   // 核心是模式的唯一权威
    switch (slam_mode_) {
        case SlamMode::LOCALIZATION:
            publishCurrScan();
            if (debug_publish_) {
                if (!debug_map_published_) { publishPointMap(); debug_map_published_ = true; }
                // publishCurrSubMap();  // D2: 注册不发
            }
            break;

        case SlamMode::MAPPING:
            if (have_imu_ && have_laser_) {
                if (debug_publish_) {
                    if (!debug_map_published_) { publishPointMap(); debug_map_published_ = true; }
                    // publishCurrSubMap();  // D2
                }
                publishCurrScan();
                publishMappingPath();
                // publishLoopBetweenKF();  // D2
            }
            break;

        default:
            break;
    }
}

void LegacyXz3dSlamInterface::publishCurrScan() {
    SlamOutput out;
    if (!engine_.latestOutput(out) || !out.world_cloud || out.world_cloud->empty()) return;
    const CloudType& world = *out.world_cloud;   // 核心给的就是 map 系下的当前帧
    const ros::Time stamp = out.pose.time > 0 ? ros::Time(out.pose.time) : ros::Time::now();

    if (slam_mode_ == SlamMode::LOCALIZATION) {
        // scan_f_map: 抽稀到约 2000 + z∈(0.1,2) (legacy.cpp:982-1011 旧带 (0,2) 在
        // "地面=0" 的车体系下成立; 我们 z_ref 有 ~7cm 标定偏差, 下界抬 0.1 防地面泄漏), **无条件**发
        CloudType::Ptr shaped = legacyScanShape(world, kLocZMax, 1, scan_ground_z_, kScanZMin);
        if (shaped->empty()) return;
        ros1_pointcloud_puber_.publish(toOldPointCloud(*shaped, kMapFrame, stamp));
        // curr_scan_point: 同一个点云, 但只有 debug_publish_ 才发 (legacy.cpp:1007-1009)
        if (debug_publish_) pointcloud_puber_.publish(toPointCloud2(*shaped, kMapFrame, stamp));
        // or_curr_scan_point: 原始输入帧, 抽稀倍率 x2, **不过滤 z** (legacy.cpp:1026-1058)。
        // frame_id "map" 照抄旧实现 (xz_slam_ros1_interface.cpp:1053)。
        CloudType::Ptr raw_snapshot;
        ros::Time raw_stamp;
        {
            std::lock_guard<std::mutex> lk(raw_input_mtx_);
            raw_snapshot = raw_input_;
            raw_stamp = raw_input_stamp_;
        }
        if (raw_snapshot && !raw_snapshot->empty()) {
            CloudType::Ptr orc = legacyScanShape(*raw_snapshot, -1.0, 2, scan_ground_z_);
            if (!orc->empty())
                or_pointcloud_puber_.publish(toPointCloud2(*orc, kMapFrame, raw_stamp));
        }
    } else if (slam_mode_ == SlamMode::MAPPING) {
        // scan_f_map: 抽稀到约 2000 + z∈(0.1,1) (legacy.cpp:1071-1096, 下界同理抬 0.1)
        CloudType::Ptr shaped = legacyScanShape(world, kMapZMax, 1, scan_ground_z_, kScanZMin);
        // curr_scan_point: **完整的**去畸变点云 (legacy.cpp:1097 把过滤结果覆盖掉了), 无条件发
        pointcloud_puber_.publish(toPointCloud2(world, kMapFrame, stamp));
        if (!shaped->empty()) ros1_pointcloud_puber_.publish(toOldPointCloud(*shaped, kMapFrame, stamp));
        // or_curr_scan_point: 建图**不发** (legacy.cpp:1102 那行是注释掉的)
    }
}

void LegacyXz3dSlamInterface::publishMappingPath() {
    mapping_path_msg_.header.frame_id = kMapFrame;
    path_puber_.publish(mapping_path_msg_);   // 整条重发 (legacy.cpp:885-887)
}

void LegacyXz3dSlamInterface::publishRealtimeGridMap() {
    GridInfo gi;
    if (realtime_grid_.latest(gi)) {
        realtime_grid_map_ = toLegacyGrid(gi, kMapFrame);
        realtime_grid_map_area_ =
            static_cast<double>(gi.width) * gi.height * gi.resolution * gi.resolution;
        // legacy.cpp:1749 把建图面积塞进 origin.position.z
        realtime_grid_map_.info.origin.position.z = realtime_grid_map_area_ * 0.00001;
        std::cout << "[legacy_iface] realtime grid updated: " << gi.width << "x" << gi.height
                  << " res=" << gi.resolution << " area=" << realtime_grid_map_area_ << std::endl;
    }
    // 无更新也照发上一张 (legacy.cpp:909 无条件 publish)
    realtime_grid_map_puber_.publish(realtime_grid_map_);
}

void LegacyXz3dSlamInterface::publishPointMap() {
    PointVec pts;
    if (engine_.getGlobalMap(pts) && !pts.empty()) {
        CloudType cloud;
        cloud.reserve(pts.size());
        for (auto& p : pts) cloud.push_back(p);
        cloud.width = cloud.size();
        cloud.height = 1;
        point_map_puber_.publish(toPointCloud2(cloud, kMapFrame, ros::Time::now()));
    }
    // global_grid_map: 由 set_grid_map_ 标志驱动, 发一次就清 (legacy.cpp:926-946)
    if (set_grid_map_) {
        grip_map_puber_.publish(global_grid_map_msg_);
        std::cout << "pub once grid map" << std::endl;
        set_grid_map_ = false;
    }
}

// ---------------------------------------------------------------- 0.01s 位姿 + slam/info

} // namespace rosiwit_slam_ros1
