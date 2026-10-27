// ============================================================================
// Ros1SlamInterface 点云/栅格发布：输入回调、scan_f_map/curr_scan_point、
// mapping_path、/map 与 global_map。
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
void Ros1SlamInterface::laserScanCB(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // 旧实现只用它做雷达频率统计 (legacy.cpp:209 -> laserScanCallBack)
    const ros::Time now = ros::Time::now();

    // C2b: 同时缓存 base 系 2D 点, 供平面匹配约束使用 (只在定位模式下).
    // 过滤与导航侧一致: 丢弃过近的自反射和过远的噪声.
    if (planar_constraint_enabled_ && slam_mode_ == SlamMode::LOCALIZATION &&
        msg && msg->ranges.size() >= 20) {
        geometry_msgs::TransformStamped tf;
        bool have_tf = !msg->header.frame_id.empty() &&
                       msg->header.frame_id == "base_link";
        if (!have_tf && tf_buffer_ && !msg->header.frame_id.empty()) {
            try {
                tf = tf_buffer_->lookupTransform("base_link", msg->header.frame_id,
                                                 msg->header.stamp, ros::Duration(0.03));
                have_tf = true;
            } catch (const tf2::TransformException&) {
                have_tf = false;
            }
        }
        if (have_tf) {
            // 注意: 2D 雷达是**倾斜安装**的, 取旋转的 rz/atan2 当 yaw 会把俯仰
            // 混进航向 (实测造成 ~12° 误差). 必须用完整旋转做 3D 变换再投影 xy.
            const auto & q = tf.transform.rotation;
            const Eigen::Quaterniond rot(q.w, q.x, q.y, q.z);
            const Eigen::Vector3d trans(tf.transform.translation.x,
                                        tf.transform.translation.y,
                                        tf.transform.translation.z);
            std::vector<rosiwit_slam::PlanarPoint> pts;
            pts.reserve(msg->ranges.size());
            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                const float r = msg->ranges[i];
                if (!std::isfinite(r) || r < 0.15f || r > 8.0f) { continue; }
                const double a = msg->angle_min + msg->angle_increment * static_cast<double>(i);
                const Eigen::Vector3d p_laser(r * std::cos(a), r * std::sin(a), 0.0);
                const Eigen::Vector3d p = rot * p_laser + trans;
                pts.push_back({p.x(), p.y()});
            }
            if (pts.size() >= 60) {
                planar_pts_base_.swap(pts);
                planar_pts_stamp_ = msg->header.stamp;
            }
        }
    }
    if (scan_window_start_.isZero()) { scan_window_start_ = now; scan_count_ = 0; return; }
    ++scan_count_;
    const double dt = (now - scan_window_start_).toSec();
    if (dt >= 1.0) {
        scan_hz_ = scan_count_ / dt;
        scan_window_start_ = now;
        scan_count_ = 0;
        std::cout << "[ros1_slam] /scan freq = " << scan_hz_ << " Hz" << std::endl;
    }
}

void Ros1SlamInterface::chassisOdomCB(const nav_msgs::Odometry::ConstPtr& msg) {
    // 旧实现无条件订阅并把它当运动先验喂给定位 (legacy.cpp:210 chassisOdomCallBack)
    engine_.onOdomMsg(msg);
    // 位姿-里程计一致性监测的输入 (同一定时器线程消费, 无锁)
    last_odom_x_ = msg->pose.pose.position.x;
    last_odom_y_ = msg->pose.pose.position.y;
    last_odom_t_ = msg->header.stamp.toSec();
}

void Ros1SlamInterface::liftStateCB(const std_msgs::String::ConstPtr& msg) {
    lift_state_ = msg->data;   // 旧实现 localiza_ptr_->setCurrLiftState(lift_state); 核心无此接口
}

void Ros1SlamInterface::liftTypeCB(const std_msgs::String::ConstPtr& msg) {
    if (lift_type_ != msg->data) {
        lift_type_ = msg->data;
        std::cout << "[ros1_slam] lift_type = " << lift_type_ << std::endl;
    }
}

void Ros1SlamInterface::onRawInputCloud(const CloudType& raw_cloud, const ros::Time& stamp) {
    CloudType::Ptr snapshot(new CloudType(raw_cloud));
    {
        std::lock_guard<std::mutex> lk(raw_input_mtx_);
        raw_input_ = snapshot;
        raw_input_stamp_ = stamp;
    }
    // origin_point: 每帧雷达都发 (legacy.cpp:3169/3299/3427)。
    // frame_id 用 "map" 是照抄旧实现 (xz_slam_ros1_interface.cpp:3115) —— 点其实是未配准的输入点。
    if (!snapshot->empty() && point_puber_.getNumSubscribers() > 0) {
        point_puber_.publish(toPointCloud2(*snapshot, kMapFrame, stamp));
    }
}

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
    if (!path_puber_.getNumSubscribers() || mapping_path_msg_.poses.empty()) return;
    mapping_path_msg_.header.frame_id = kMapFrame;
    // 整条 Path 重发非常贵 (O(轨迹长度))；10Hz 调用只保留 1Hz 实际发送。
    const ros::Time now = ros::Time::now();
    if (!last_mapping_path_pub_.isZero() &&
        (now - last_mapping_path_pub_).toSec() < 1.0) return;
    last_mapping_path_pub_ = now;
    path_puber_.publish(mapping_path_msg_);   // 整条重发 (legacy.cpp:885-887)
}

void Ros1SlamInterface::publishRealtimeGridMap() {
    GridInfo gi;
    if (realtime_grid_.latest(gi)) {
        nav_msgs::OccupancyGrid msg = toRosGrid(gi, kMapFrame);
        realtime_grid_map_area_ =
            static_cast<double>(gi.width) * gi.height * gi.resolution * gi.resolution;
        // legacy.cpp:1749 把建图面积塞进 origin.position.z
        msg.info.origin.position.z = realtime_grid_map_area_ * 0.00001;
        std::cout << "[ros1_slam] realtime grid updated: " << gi.width << "x" << gi.height
                  << " res=" << gi.resolution << " area=" << realtime_grid_map_area_ << std::endl;
        setStaticMap(std::move(msg));
    }
    // 无更新也照发上一张 (legacy.cpp:909 无条件 publish); 发布用快照, 避免与 tick 线程抢
    nav_msgs::OccupancyGrid snap = staticMapSnapshot();
    realtime_grid_map_puber_.publish(snap);
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
