#pragma once
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <builtin_interfaces/msg/time.hpp>
#include <map>
#include <mutex>
#include "ros_interface/frame_utils.h"
#include "slam_core/i_slam_algorithm.h"
#include "slam_core/slam_types.h"
#include "slam_core/pose_filter.h"
#include "rosiwit_slam_ros2/srv/save_map.hpp"
#include "rosiwit_slam_ros2/srv/load_map.hpp"
#include "rosiwit_slam_ros2/srv/save_grid_map.hpp"
#include "rosiwit_slam_ros2/srv/set_slam_mode.hpp"

namespace rosiwit_slam {

using rosiwit_slam_ros2::RigidTransform;

class SlamBase;    // forward declare for tryPopAndProcess

class SlamNode : public rclcpp::Node {
public:
    explicit SlamNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SlamNode() override;

private:
    struct NodeConfig {
        std::string imu_topic   = "/imu";
        std::string lidar_topic = "/velodyne_points";
        std::string lidar_frame = "lidar";
        std::string imu_frame   = "lidar";     // 核心 body frame (FAST-LIO2 估计 IMU body)
        std::string body_frame  = "base_link";
        std::string world_frame = "odom";
        int    lidar_filter_num = 3;
        double lidar_min_range  = 0.5;
        double lidar_max_range  = 100.0;
        float  scan_period_ms   = 100.0f;   // LiDAR 扫描周期 (毫秒), 用于点时间戳
        bool   vision_enabled    = false;    // 视觉松耦合开关
        std::string vo_topic     = "/visual_odometry";  // VO 库发布的里程计话题
        bool   odom_enabled      = false;    // 轮式里程计开关(供定位运动先验)
        std::string odom_topic   = "/odom";  // 轮式里程计话题
        double output_rate_hz    = 100.0;    // 定位高频输出频率(Hz); <=0 关闭(用扫描帧)
        double output_filter_alpha = 0.5;    // 输出 alpha-beta 滤波 alpha
        double output_filter_beta  = 0.2;    // 输出 alpha-beta 滤波 beta
        double output_extrap_max_s = 0.3;    // 外推上限(s)
        double grid_map_resolution = 0.3;    // 实时 /map 分辨率(m)
        double map_publish_period = 0.2;     // /map 发布周期(s)
        bool   absolute_enabled  = false;    // 绝对约束(GNSS/UWB/反光标/二维码)开关
        std::string absolute_topic = "/absolute_pose";  // geometry_msgs/PoseWithCovarianceStamped
        double datum_x = 0, datum_y = 0, datum_z = 0, datum_yaw_deg = 0;  // 绝对源系 -> 地图系
    };

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg);
    void lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void voCB(const nav_msgs::msg::Odometry::SharedPtr msg);   // 视觉里程计 (可选)
    void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg); // 轮式里程计 (可选)
    void absoluteCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);  // 绝对约束 (可选)
    void timerCB();
    void mapTimerCB();
    void highRateCB();
    void onOutput(const SlamOutput& out);
    void publish(const SlamOutput& out);
    void publishOdomTf(const PoseStamped& pose, const builtin_interfaces::msg::Time& stamp);
    void initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    builtin_interfaces::msg::Time toRosTime(double sec);

    /// 从 TF 取 base_link <- IMU / lidar, 供输入输出换算; 失败限频重试.
    bool ensureBaseFrames();
    /// 查一次 TF target <- source; 同 frame 返回单位变换.
    bool lookupMount(const std::string& target, const std::string& source,
                     RigidTransform& out, double timeout_sec);
    /// odom/VO child frame -> 核心 IMU body; 缓存静态外参.
    bool imuFromChild(const std::string& child_frame, RigidTransform& out);
    /// 把 TF 推导出的 r_il/t_il 写进 override YAML, 供核心 init 使用.
    std::string writeExtrinsicOverride(const std::string& base_yaml,
                                       const RigidTransform& imu_lidar,
                                       const RigidTransform& base_lidar);
    /// 换图/换模式后允许 /map 重新发布.
    void resetPublishState();

    void handleSaveMap(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam_ros2::srv::SaveMap::Request>, std::shared_ptr<rosiwit_slam_ros2::srv::SaveMap::Response>);
    void handleLoadMap(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam_ros2::srv::LoadMap::Request>, std::shared_ptr<rosiwit_slam_ros2::srv::LoadMap::Response>);
    void handleSaveGridMap(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam_ros2::srv::SaveGridMap::Request>, std::shared_ptr<rosiwit_slam_ros2::srv::SaveGridMap::Response>);
    void handleSetSlamMode(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam_ros2::srv::SetSlamMode::Request>, std::shared_ptr<rosiwit_slam_ros2::srv::SetSlamMode::Response>);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          m_imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  m_lidar_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_init_pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_vo_sub;   // 视觉里程计 (开关控制)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub; // 轮式里程计 (开关控制)
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_absolute_sub;  // 绝对约束
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub, m_world_cloud_pub, m_global_map_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr     m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_grid_map_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_nav_pub;
    std::unique_ptr<ISlamAlgorithm> m_algo;
    ISlamAlgorithm* m_algo_raw = nullptr;
    rclcpp::TimerBase::SharedPtr m_process_timer, m_map_timer, m_output_timer, m_mount_timer;
    // 回调分组: 传感器收包与重处理解耦, 配合多线程执行器避免处理阻塞收包导致丢帧
    rclcpp::CallbackGroup::SharedPtr m_sensor_cb_group;
    rclcpp::CallbackGroup::SharedPtr m_work_cb_group;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
    mutable std::mutex m_mount_mu;
    rosiwit_slam_ros2::RigidTransform m_base_imu;    // T_base_link_imu
    rosiwit_slam_ros2::RigidTransform m_base_lidar;  // T_base_link_lidar (BBS 投影用)
    std::map<std::string, rosiwit_slam_ros2::RigidTransform> m_child_to_imu;
    bool m_mount_warned = false;
    rclcpp::Service<rosiwit_slam_ros2::srv::SaveMap>::SharedPtr       m_srv_save_map;
    rclcpp::Service<rosiwit_slam_ros2::srv::LoadMap>::SharedPtr       m_srv_load_map;
    rclcpp::Service<rosiwit_slam_ros2::srv::SaveGridMap>::SharedPtr   m_srv_save_grid_map;
    rclcpp::Service<rosiwit_slam_ros2::srv::SetSlamMode>::SharedPtr   m_srv_set_slam_mode;
    NodeConfig m_cfg;
    std::mutex m_out_mutex;
    SlamOutput m_latest;
    bool m_have_output = false;
    nav_msgs::msg::Path m_path;
    bool m_grid_map_published = false;
    uint64_t m_last_map_revision = 0;

    // EMA 平滑滤波（抑制 LIO 抖动，避免 costmap 中机器人中心跳进 lethal）
    PoseStamped m_smoothed_pose;
    bool m_has_smoothed = false;
    double m_ema_alpha = 0.3;

    // L12: 高频输出滤波 (alpha-beta / 常速), 替代 EMA 用于高频流
    PoseOutputFilter m_out_filter;
    Eigen::Matrix<double, 6, 6> m_pose_cov = Eigen::Matrix<double, 6, 6>::Identity();  // [rot,trans]
    bool m_has_cov = false;
};

} // namespace rosiwit_slam
