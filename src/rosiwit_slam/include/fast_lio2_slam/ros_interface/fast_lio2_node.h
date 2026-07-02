/**
 * @file fast_lio2_node.h
 * @brief FAST-LIO2 SLAM - ROS2节点主类
 * @author AI Development Team
 * @date 2026-04-24
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp>

#include "fast_lio2_slam/common/types.h"
#include "fast_lio2_slam/common/config.h"
#include "fast_lio2_slam/common/thread_pool.h"
#include "fast_lio2_slam/common/profiler.h"
#include "fast_lio2_slam/data_preprocessor/point_cloud_converter.h"
#include "fast_lio2_slam/data_preprocessor/point_cloud_filter.h"
#include "fast_lio2_slam/data_preprocessor/imu_processor.h"
#include "fast_lio2_slam/fast_lio2_core/iekf_estimator.h"
#include "fast_lio2_slam/fast_lio2_core/ikd_tree.h"
#include "fast_lio2_slam/odom_fusion/odom_fusion.h"
#include "fast_lio2_slam/map_manager/map_manager.h"
#include "fast_lio2_slam/map_manager/map_server.h"
#include "fast_lio2_slam/map_manager/map_persistence.h"
#include "fast_lio2_slam/map_manager/map_quality.h"
#include "fast_lio2_slam/localization/global_localizer.h"

#include <mutex>
#include <queue>
#include <atomic>
#include <deque>

namespace fast_lio2_slam {

/**
 * @brief FAST-LIO2 SLAM ROS2节点
 */
class FastLio2Node : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit FastLio2Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~FastLio2Node();

private:
    void initialize();
    void loadParameters();
    void createSubscribers();
    void createPublishers();
    void createServices();
    void initializeModules();

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void processPointCloud(PointCloudData& cloud_data);
    void performPrediction(double t_start, double t_end);
    bool performUpdate(PointCloudPtr& cloud);
    void updateMap(PointCloudPtr& cloud);

    void publishOdometry();
    void publishPath();
    void publishMap();
    void publishTF();

    void saveMapCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void savePcdCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void globalLocalizeCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void setInitialPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

    bool checkDataSync();
    void undistortPointCloud(PointCloudData& cloud_data);
    void saveProjectedMap(const PointCloudPtr& cloud, const std::string& base_path);

private:
    // 配置
    ConfigParams config_;
    std::string config_file_path_;

    // ROS2接口
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_pcd_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr global_localize_srv_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr initial_pose_sub_;

    // 数据缓冲
    ImuBuffer imu_buffer_;
    std::queue<PointCloudData> point_cloud_queue_;
    std::mutex cloud_queue_mutex_;

    // 核心模块
    std::unique_ptr<PointCloudConverter> point_cloud_converter_;
    std::unique_ptr<PointCloudFilter> point_cloud_filter_;
    std::unique_ptr<ImuProcessor> imu_processor_;
    std::unique_ptr<IekfEstimator> iekf_estimator_;
    std::unique_ptr<IKdTree> ikd_tree_;
    std::unique_ptr<OdomFusion> odom_fusion_;
    std::unique_ptr<MapManager> map_manager_;

    // 性能优化模块
    std::unique_ptr<ThreadPool> thread_pool_;
    ThreadPoolConfig thread_pool_config_;

    // 建图增强模块
    std::unique_ptr<MapServer> map_server_;
    std::unique_ptr<MapPersistence> map_persistence_;
    std::unique_ptr<MapQualityEvaluator> map_quality_;

    // 全局定位模块
    std::unique_ptr<GlobalLocalizer> global_localizer_;
    LocalizationState localization_state_;
    std::mutex localization_mutex_;

    // 里程计数据缓冲
    std::deque<OdomData> odom_buffer_;
    std::mutex odom_buffer_mutex_;

    // 状态
    State current_state_;
    std::vector<SE3d> pose_history_;
    nav_msgs::msg::Path path_msg_;

    std::atomic<bool> system_initialized_;
    std::atomic<bool> is_processing_;
    std::atomic<bool> first_scan_received_;

    int scan_count_;
    int keyframe_count_;
    double last_scan_time_;

    // 定时器
    rclcpp::TimerBase::SharedPtr process_timer_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::TimerBase::SharedPtr path_timer_;
};

} // namespace fast_lio2_slam
