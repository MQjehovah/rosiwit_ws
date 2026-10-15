// ros1_interface/rosiwit_slam_ros1/src/rosiwit_slam_ros1_node.cpp
// rosiwit_slam 的 ROS1 接口层: 仅做 ROS1 <-> 算法核心(SlamFactory/SlamBase)的桥接.
// 算法核心完全 ROS 无关, 与 ROS2 节点 (src/ros_interface/slam_node.cpp) 共享同一份代码.
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <yaml-cpp/yaml.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <mutex>
#include <string>

#include "slam_core/slam_factory.h"
#include "slam_core/slam_base.h"
#include "slam_core/slam_types.h"

#include "rosiwit_slam_ros1/SaveMap.h"
#include "rosiwit_slam_ros1/LoadMap.h"
#include "rosiwit_slam_ros1/SaveGridMap.h"
#include "rosiwit_slam_ros1/SetSlamMode.h"

using namespace rosiwit_slam;

class RosiwitSlamRos1Node {
public:
    RosiwitSlamRos1Node(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        std::string algo_name = "fast_lio2_pipeline";
        pnh_.param<std::string>("slam_algorithm", algo_name, algo_name);

        std::string config_path;
        pnh_.param<std::string>("config_file", config_path, "");
        if (config_path.empty()) pnh_.param<std::string>("config_path", config_path, "");

        cfg_.imu_topic   = "/imu";
        cfg_.lidar_topic = "/velodyne_points";
        cfg_.body_frame  = "base_link";
        cfg_.world_frame = "odom";
        cfg_.scan_period_ms = 100.0f;
        cfg_.lidar_filter_num = 3;
        cfg_.lidar_min_range = 0.5;
        cfg_.lidar_max_range = 100.0;
        bool vo_enabled = false, odom_enabled = false, absolute_enabled = false;
        if (!config_path.empty()) {
            try {
                YAML::Node c = YAML::LoadFile(config_path);
                if (c) {
                    cfg_.imu_topic   = c["imu_topic"].as<std::string>(cfg_.imu_topic);
                    cfg_.lidar_topic = c["lidar_topic"].as<std::string>(cfg_.lidar_topic);
                    cfg_.body_frame  = c["body_frame"].as<std::string>(cfg_.body_frame);
                    cfg_.world_frame = c["world_frame"].as<std::string>(cfg_.world_frame);
                    cfg_.scan_period_ms = c["scan_period_ms"].as<float>(cfg_.scan_period_ms);
                    cfg_.lidar_filter_num = c["lidar_filter_num"].as<int>(cfg_.lidar_filter_num);
                    cfg_.lidar_min_range = c["lidar_min_range"].as<double>(cfg_.lidar_min_range);
                    cfg_.lidar_max_range = c["lidar_max_range"].as<double>(cfg_.lidar_max_range);
                    if (c["vision"]) vo_enabled = c["vision"]["enabled"].as<bool>(false);
                    odom_enabled = c["enable_odom"].as<bool>(false);
                    absolute_enabled = c["absolute_enabled"].as<bool>(false);
                }
            } catch (const std::exception& e) {
                ROS_WARN("[rosiwit_slam_ros1] config parse error: %s", e.what());
            }
        }

        algo_ = SlamFactory::create(algo_name);
        if (!algo_) { ROS_FATAL("unknown algo: %s", algo_name.c_str()); ros::shutdown(); return; }
        algo_raw_ = algo_.get();
        if (!algo_->init(config_path)) { ROS_FATAL("init failed"); ros::shutdown(); return; }
        algo_->setOutputCallback([this](const SlamOutput& o) { onOutput(o); });

        // 订阅
        imu_sub_  = nh_.subscribe(cfg_.imu_topic, 1000, &RosiwitSlamRos1Node::imuCB, this);
        lidar_sub_ = nh_.subscribe(cfg_.lidar_topic, 100, &RosiwitSlamRos1Node::lidarCB, this);
        init_pose_sub_ = nh_.subscribe("/initialpose", 10, &RosiwitSlamRos1Node::initialPoseCB, this);
        if (odom_enabled) odom_sub_ = nh_.subscribe("/odom", 100, &RosiwitSlamRos1Node::odomCB, this);
        if (vo_enabled) vo_sub_ = nh_.subscribe("/visual_odometry", 100, &RosiwitSlamRos1Node::voCB, this);
        if (absolute_enabled) absolute_sub_ = nh_.subscribe("/absolute_pose", 100, &RosiwitSlamRos1Node::absoluteCB, this);

        // 发布
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("lio_odom", 1000);
        odom_nav_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("lio_path", 1000);
        body_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("body_cloud", 10000);
        world_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("world_cloud", 10000);
        global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_map", 10, true);
        grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
        static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
        geometry_msgs::TransformStamped stf;
        stf.header.stamp = ros::Time::now();
        stf.header.frame_id = "map"; stf.child_frame_id = "odom";
        stf.transform.rotation.w = 1.0;
        static_tf_->sendTransform(stf);

        path_.header.frame_id = cfg_.world_frame;

        // 服务
        srv_save_map_ = nh_.advertiseService("save_map", &RosiwitSlamRos1Node::handleSaveMap, this);
        srv_load_map_ = nh_.advertiseService("load_map", &RosiwitSlamRos1Node::handleLoadMap, this);
        srv_save_grid_ = nh_.advertiseService("save_grid_map", &RosiwitSlamRos1Node::handleSaveGridMap, this);
        srv_set_mode_ = nh_.advertiseService("set_slam_mode", &RosiwitSlamRos1Node::handleSetMode, this);

        // 定时器
        process_timer_ = nh_.createTimer(ros::Duration(0.02), &RosiwitSlamRos1Node::processCB, this);
        map_timer_ = nh_.createTimer(ros::Duration(2.0), &RosiwitSlamRos1Node::mapCB, this);

        ROS_INFO("rosiwit_slam_ros1 ready: algo=%s imu=%s lidar=%s",
                 algo_name.c_str(), cfg_.imu_topic.c_str(), cfg_.lidar_topic.c_str());
    }

private:
    struct Cfg {
        std::string imu_topic = "/imu";
        std::string lidar_topic = "/velodyne_points";
        std::string body_frame = "base_link";
        std::string world_frame = "odom";
        float scan_period_ms = 100.0f;
        int lidar_filter_num = 3;
        double lidar_min_range = 0.5, lidar_max_range = 100.0;
    };

    static double getSec(const ros::Time& t) { return t.toSec(); }

    static CloudType::Ptr ros2Pcl(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                  int filter_num, double min_r, double max_r) {
        CloudType::Ptr cloud(new CloudType());
        pcl::fromROSMsg(*msg, *cloud);
        filter_num = std::max(1, filter_num);
        CloudType::Ptr out(new CloudType());
        out->reserve(cloud->size() / filter_num + 1);
        const double min2 = min_r * min_r, max2 = max_r * max_r;
        for (size_t i = 0; i < cloud->size(); ++i) {
            if (i % filter_num != 0) continue;
            const auto& p = cloud->points[i];
            if (!pcl::isFinite(p)) continue;
            const double d2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y +
                              static_cast<double>(p.z) * p.z;
            if (d2 < min2 || d2 > max2) continue;
            out->push_back(p);
        }
        return out;
    }

    void imuCB(const sensor_msgs::Imu::ConstPtr& msg) {
        IMUSample s;
        s.acc = V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        s.gyro = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        s.time = getSec(msg->header.stamp);
        algo_->onImu(s);
    }

    void lidarCB(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        LidarFrame f;
        f.cloud = ros2Pcl(msg, cfg_.lidar_filter_num, cfg_.lidar_min_range, cfg_.lidar_max_range);
        if (!f.cloud || f.cloud->empty()) return;
        f.start_time = getSec(msg->header.stamp);
        f.end_time = f.start_time + cfg_.scan_period_ms / 1000.0;
        algo_->onLidar(f);
    }

    void initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        if (algo_raw_->getPipelineMode() != "localization") return;
        PoseStamped p;
        p.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        p.rot = q.toRotationMatrix();
        p.time = getSec(msg->header.stamp);
        algo_raw_->setInitPose(p);
    }

    void odomCB(const nav_msgs::Odometry::ConstPtr& msg) {
        PoseStamped p;
        p.time = getSec(msg->header.stamp);
        p.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        p.rot = q.toRotationMatrix();
        algo_->onOdom(p);
    }

    void voCB(const nav_msgs::Odometry::ConstPtr& msg) {
        PoseStamped p;
        p.time = getSec(msg->header.stamp);
        p.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        p.rot = q.toRotationMatrix();
        algo_->onVisualOdometry(p.time, p);
    }

    void absoluteCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        PoseStamped p;
        p.time = getSec(msg->header.stamp);
        p.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        p.rot = q.toRotationMatrix();
        Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e2;
        auto mi = [](int r) { return r < 3 ? r + 3 : r - 3; };
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
                cov(r, c) = msg->pose.covariance[mi(r) * 6 + mi(c)];
        algo_->onAbsolute(p, cov);
    }

    void onOutput(const SlamOutput& out) {
        std::lock_guard<std::mutex> lk(out_mutex_);
        latest_ = out;
        have_output_ = true;
    }

    void processCB(const ros::TimerEvent&) {
        auto* base = dynamic_cast<SlamBase*>(algo_.get());
        if (base) { while (base->tryPopAndProcess()) {} }
        SlamOutput out; bool have;
        { std::lock_guard<std::mutex> lk(out_mutex_); have = have_output_; out = latest_; }
        if (have && out.has_new_pose) publish(out);
    }

    void publish(const SlamOutput& out) {
        ros::Time stamp;
        if (out.pose.time > 0) stamp = ros::Time(out.pose.time); else stamp = ros::Time::now();
        Eigen::Quaterniond q(out.pose.rot);

        nav_msgs::Odometry odom;
        odom.header.frame_id = cfg_.world_frame;
        odom.child_frame_id = cfg_.body_frame;
        odom.header.stamp = stamp;
        odom.pose.pose.position.x = out.pose.trans.x();
        odom.pose.pose.position.y = out.pose.trans.y();
        odom.pose.pose.position.z = out.pose.trans.z();
        odom.pose.pose.orientation.x = q.x(); odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z(); odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = out.pose.vel.x();
        odom.twist.twist.linear.y = out.pose.vel.y();
        odom.twist.twist.linear.z = out.pose.vel.z();
        odom_pub_.publish(odom);
        if (odom_nav_pub_.getNumSubscribers() > 0) odom_nav_pub_.publish(odom);

        geometry_msgs::TransformStamped tf;
        tf.header.frame_id = cfg_.world_frame; tf.child_frame_id = cfg_.body_frame;
        tf.header.stamp = stamp;
        tf.transform.translation.x = out.pose.trans.x();
        tf.transform.translation.y = out.pose.trans.y();
        tf.transform.translation.z = out.pose.trans.z();
        tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf);

        auto pub_cloud = [&](ros::Publisher& p, const CloudType::Ptr& c, const std::string& fid) {
            if (p.getNumSubscribers() <= 0 || !c) return;
            sensor_msgs::PointCloud2 m; pcl::toROSMsg(*c, m);
            m.header.frame_id = fid; m.header.stamp = stamp; p.publish(m);
        };
        pub_cloud(body_cloud_pub_, out.body_cloud, cfg_.body_frame);
        pub_cloud(world_cloud_pub_, out.world_cloud, cfg_.world_frame);

        if (path_pub_.getNumSubscribers() > 0) {
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = cfg_.world_frame; ps.header.stamp = stamp;
            ps.pose.position.x = out.pose.trans.x();
            ps.pose.position.y = out.pose.trans.y();
            ps.pose.position.z = out.pose.trans.z();
            ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y();
            ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
            path_.poses.push_back(ps);
            path_.header.stamp = stamp;
            path_pub_.publish(path_);
        }
    }

    void mapCB(const ros::TimerEvent&) {
        const std::string mode = algo_raw_->getPipelineMode();
        const bool is_loc = (mode == "localization");
        // cloud_map
        if (is_loc || global_map_pub_.getNumSubscribers() > 0) {
            PointVec pts;
            if (algo_->getGlobalMap(pts) && !pts.empty()) {
                CloudType cloud; cloud.reserve(pts.size());
                for (auto& p : pts) cloud.push_back(p);
                cloud.width = cloud.size(); cloud.height = 1;
                sensor_msgs::PointCloud2 m; pcl::toROSMsg(cloud, m);
                m.header.frame_id = cfg_.world_frame; m.header.stamp = ros::Time::now();
                global_map_pub_.publish(m);
            }
        }
        // grid_map / map
        if (!grid_published_ && (is_loc || map_pub_.getNumSubscribers() > 0)) {
            GridInfo gi = algo_raw_->getGridInfo();
            if (gi.width > 0 && gi.height > 0) {
                nav_msgs::OccupancyGrid grid;
                grid.header.frame_id = cfg_.world_frame;
                grid.header.stamp = ros::Time::now();
                grid.info.resolution = gi.resolution;
                grid.info.width = gi.width;
                grid.info.height = gi.height;
                grid.info.origin.position.x = gi.origin_x;
                grid.info.origin.position.y = gi.origin_y;
                grid.info.origin.orientation.w = 1.0;
                const int w = gi.width, h = gi.height;
                grid.data.resize(static_cast<size_t>(w) * h);
                for (int row = 0; row < h; ++row) {
                    const int src = h - 1 - row;   // 竖直翻转: 首行=最大 y
                    for (int col = 0; col < w; ++col)
                        grid.data[static_cast<size_t>(row) * w + col] =
                            gi.data[static_cast<size_t>(src) * w + col];
                }
                grid_map_pub_.publish(grid);
                map_pub_.publish(grid);
                grid_published_ = true;
            }
        }
    }

    bool handleSaveMap(rosiwit_slam_ros1::SaveMap::Request& req, rosiwit_slam_ros1::SaveMap::Response& res) {
        res.success = algo_raw_->saveMap(req.path);
        res.message = res.success ? "ok" : "failed";
        return true;
    }
    bool handleLoadMap(rosiwit_slam_ros1::LoadMap::Request& req, rosiwit_slam_ros1::LoadMap::Response& res) {
        res.success = algo_raw_->loadMap(req.path);
        res.message = res.success ? "ok" : "failed";
        return true;
    }
    bool handleSaveGridMap(rosiwit_slam_ros1::SaveGridMap::Request& req, rosiwit_slam_ros1::SaveGridMap::Response& res) {
        res.success = algo_raw_->saveGridMap(req.pgm_path, req.yaml_path, req.resolution);
        res.message = res.success ? "ok" : "failed";
        return true;
    }
    bool handleSetMode(rosiwit_slam_ros1::SetSlamMode::Request& req, rosiwit_slam_ros1::SetSlamMode::Response& res) {
        res.success = algo_raw_->setPipelineMode(req.mode);
        res.message = res.success ? "ok" : "unsupported";
        return true;
    }

    ros::NodeHandle nh_, pnh_;
    Cfg cfg_;
    std::unique_ptr<ISlamAlgorithm> algo_;
    ISlamAlgorithm* algo_raw_ = nullptr;

    ros::Subscriber imu_sub_, lidar_sub_, init_pose_sub_, odom_sub_, vo_sub_, absolute_sub_;
    ros::Publisher odom_pub_, odom_nav_pub_, path_pub_, body_cloud_pub_, world_cloud_pub_,
                   global_map_pub_, grid_map_pub_, map_pub_;
    ros::ServiceServer srv_save_map_, srv_load_map_, srv_save_grid_, srv_set_mode_;
    ros::Timer process_timer_, map_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;

    std::mutex out_mutex_;
    SlamOutput latest_;
    bool have_output_ = false;
    bool grid_published_ = false;
    nav_msgs::Path path_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosiwit_slam_ros1");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    RosiwitSlamRos1Node node(nh, pnh);
    ros::spin();
    return 0;
}
