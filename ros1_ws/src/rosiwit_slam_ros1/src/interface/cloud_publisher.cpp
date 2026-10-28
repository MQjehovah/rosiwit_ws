// ============================================================================
// cloud_publisher.cpp — 兼容层的点云 / 栅格输出
//
// 单一职责: 把核心输出变成 legacy 话题 (scan_f_map / curr_scan_point /
// or_curr_scan_point / mapping_path / global_map / global_grid_map / /map)。
// 输入回调在 subscribers.cpp; 位姿/健康在 pose_publisher.cpp。
// ============================================================================
#include "ros1_slam_interface.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "convert.h"
#include "internal.h"
#include "map_adapter.h"

using namespace rosiwit_slam;
using namespace rosiwit_slam_ros1::interface_internal;

namespace rosiwit_slam_ros1 {





// ---------------------------------------------------------------- 定时器

void Ros1SlamInterface::timerPublishLocalCloudAndSubmap(const ros::TimerEvent&) {
    startPub();   // legacy.cpp:834
}
void Ros1SlamInterface::timerPublishRealtimeGridMap(const ros::TimerEvent&) {
    if (slam_mode_ == SlamMode::MAPPING) publishRealtimeGridMap();   // legacy.cpp:836-838
}

// ---------------------------------------------------------------- 0.1s 分发

void Ros1SlamInterface::startPub() {
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

void Ros1SlamInterface::publishCurrScan() {
    SlamOutput out;
    if (!engine_.latestOutput(out) || !out.world_cloud || out.world_cloud->empty()) return;

    const CloudType& world = *out.world_cloud;   // 核心给的就是 map 系下的当前帧
    const ros::Time stamp = out.pose.time > 0 ? ros::Time(out.pose.time) : ros::Time::now();
    if (slam_mode_ == SlamMode::LOCALIZATION) {
        publishLocalizationScan(world, stamp);
        publishRawInputScan();
    } else if (slam_mode_ == SlamMode::MAPPING) {
        publishMappingScan(world, stamp);
    }
}

void Ros1SlamInterface::publishLocalizationScan(const CloudType& world,
                                                const ros::Time& stamp) {
    // scan_f_map: 抽稀到约 2000 + z∈(0.1,2)；无人订阅时不做 shape/旧 PointCloud 转换。
    const bool need_scan = ros1_pointcloud_puber_.getNumSubscribers() > 0;
    const bool need_debug_cloud =
        debug_publish_ && pointcloud_puber_.getNumSubscribers() > 0;
    if (!need_scan && !need_debug_cloud) return;

    CloudType::Ptr shaped = scanShape(world, kLocZMax, 1, scan_ground_z_, kScanZMin);
    if (shaped->empty()) return;
    if (need_scan)
        ros1_pointcloud_puber_.publish(toOldPointCloud(*shaped, kMapFrame, stamp));
    if (need_debug_cloud)
        pointcloud_puber_.publish(toPointCloud2(*shaped, kMapFrame, stamp));
}

void Ros1SlamInterface::publishRawInputScan() {
    // or_curr_scan_point: 原始输入帧，抽稀 x2，不过滤 z。
    if (or_pointcloud_puber_.getNumSubscribers() <= 0) return;

    CloudType::Ptr raw_snapshot;
    ros::Time raw_stamp;
    {
        std::lock_guard<std::mutex> lk(raw_input_mtx_);
        raw_snapshot = raw_input_;
        raw_stamp = raw_input_stamp_;
    }
    if (!raw_snapshot || raw_snapshot->empty()) return;

    CloudType::Ptr orc = scanShape(*raw_snapshot, -1.0, 2, scan_ground_z_);
    if (!orc->empty())
        or_pointcloud_puber_.publish(toPointCloud2(*orc, kMapFrame, raw_stamp));
}

void Ros1SlamInterface::publishMappingScan(const CloudType& world,
                                           const ros::Time& stamp) {
    if (pointcloud_puber_.getNumSubscribers() > 0) {
        pointcloud_puber_.publish(toPointCloud2(world, kMapFrame, stamp));
    }
    if (ros1_pointcloud_puber_.getNumSubscribers() > 0) {
        CloudType::Ptr shaped = scanShape(world, kMapZMax, 1, scan_ground_z_, kScanZMin);
        if (!shaped->empty())
            ros1_pointcloud_puber_.publish(toOldPointCloud(*shaped, kMapFrame, stamp));
    }
}

void Ros1SlamInterface::publishMappingPath() {
    if (!path_puber_.getNumSubscribers()) return;
    // 整条 Path 重发非常贵 (O(轨迹长度))；10Hz 调用只保留 1Hz 实际发送。
    const ros::Time now = ros::Time::now();
    if (!last_mapping_path_pub_.isZero() &&
        (now - last_mapping_path_pub_).toSec() < 1.0) return;
    nav_msgs::Path msg;
    {
        std::lock_guard<std::mutex> lk(path_mu_);
        if (mapping_path_msg_.poses.empty()) return;
        mapping_path_msg_.header.frame_id = kMapFrame;
        msg = mapping_path_msg_;   // 锁内拷贝, 锁外序列化
    }
    last_mapping_path_pub_ = now;
    path_puber_.publish(msg);   // 整条重发 (legacy.cpp:885-887)
}

void Ros1SlamInterface::publishRealtimeGridMap() {
    // latestPtr(): 无锁拿不可变快照, 不再整份拷一份 GridInfo (I8)。
    // (OccupancyGrid 仍要各有一份 —— ROS 消息自己持有 data 缓冲, 这是协议决定的。)
    const auto snap = realtime_grid_.latestPtr();
    if (snap) {
        nav_msgs::OccupancyGrid msg = toRosGrid(*snap, kMapFrame);
        realtime_grid_map_area_ =
            static_cast<double>(snap->width) * snap->height * snap->resolution * snap->resolution;
        // legacy.cpp:1749 把建图面积塞进 origin.position.z
        msg.info.origin.position.z = realtime_grid_map_area_ * 0.00001;
        std::cout << "[ros1_slam] realtime grid updated: " << snap->width << "x"
                  << snap->height << " res=" << snap->resolution
                  << " area=" << realtime_grid_map_area_ << std::endl;
        setStaticMap(msg);                 // tick 线程也要读 (slam/info、兜底重发)
        realtime_grid_map_puber_.publish(msg);
        return;
    }
    // 无更新也照发上一张 (legacy.cpp:909 无条件 publish); 取快照, 避免与 tick 线程抢
    nav_msgs::OccupancyGrid cur = staticMapSnapshot();
    realtime_grid_map_puber_.publish(cur);
}

void Ros1SlamInterface::publishPointMap() {
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
