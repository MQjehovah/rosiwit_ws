// ============================================================================
// subscribers.cpp — 兼容层自己订阅的输入回调
//
// 单一职责: 老接口除 SLAM 传感器外还吃几路底盘/升降信号
// (laserScan / chassisOdom / lift_*), 以及一条"原始点云"钩子。
// 这些是**输入**; 输出在 cloud_publisher.cpp。
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

} // namespace rosiwit_slam_ros1
