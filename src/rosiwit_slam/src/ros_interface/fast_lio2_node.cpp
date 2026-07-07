/**
 * @file fast_lio2_node.cpp
 * @brief FAST-LIO2 SLAM ROS2节点实现 (核心算法来自 FAST-LIO2_ROS2)
 */

#include "fast_lio2_slam/ros_interface/fast_lio2_node.h"

#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

FastLio2Node::FastLio2Node(const rclcpp::NodeOptions &options)
    : Node("rosiwit_slam", options)
{
    RCLCPP_INFO(this->get_logger(), "FAST-LIO2 SLAM Node (FAST-LIO2_ROS2 core) Starting");
    loadParameters();

    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        m_node_config.imu_topic, 10, std::bind(&FastLio2Node::imuCB, this, std::placeholders::_1));
    m_lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        m_node_config.lidar_topic, 10, std::bind(&FastLio2Node::lidarCB, this, std::placeholders::_1));

    m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
    m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
    m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
    m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
    m_global_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_map", 10);
    m_map_timer = this->create_wall_timer(2s, std::bind(&FastLio2Node::publishGlobalMap, this));
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    m_state_data.path.poses.clear();
    m_state_data.path.header.frame_id = m_node_config.world_frame;

    m_kf = std::make_shared<IESKF>();
    m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
    m_timer = this->create_wall_timer(20ms, std::bind(&FastLio2Node::timerCB, this));

    RCLCPP_INFO(this->get_logger(), "FAST-LIO2 SLAM Node initialized: imu=%s lidar=%s",
                m_node_config.imu_topic.c_str(), m_node_config.lidar_topic.c_str());
}

void FastLio2Node::loadParameters()
{
    this->declare_parameter<std::string>("config_file", "");
    std::string config_path;
    this->get_parameter<std::string>("config_file", config_path);

    if (config_path.empty())
    {
        RCLCPP_WARN(this->get_logger(), "config_file parameter is empty, using defaults");
        // 兜底默认值 (Gazebo Velodyne + IMU)
        m_node_config.imu_topic = "/imu";
        m_node_config.lidar_topic = "/velodyne_points";
        m_node_config.body_frame = "base_link";
        m_node_config.world_frame = "odom";
        m_node_config.print_time_cost = false;
        m_builder_config.lidar_max_range = 100.0;
        return;
    }

    YAML::Node config = YAML::LoadFile(config_path);
    if (!config)
    {
        RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE: %s", config_path.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

    m_node_config.imu_topic = config["imu_topic"].as<std::string>();
    m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
    m_node_config.body_frame = config["body_frame"].as<std::string>();
    m_node_config.world_frame = config["world_frame"].as<std::string>();
    m_node_config.print_time_cost = config["print_time_cost"].as<bool>();

    m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
    m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
    m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
    m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
    m_builder_config.map_resolution = config["map_resolution"].as<double>();
    m_builder_config.cube_len = config["cube_len"].as<double>();
    m_builder_config.det_range = config["det_range"].as<double>();
    m_builder_config.move_thresh = config["move_thresh"].as<double>();
    m_builder_config.na = config["na"].as<double>();
    m_builder_config.ng = config["ng"].as<double>();
    m_builder_config.nba = config["nba"].as<double>();
    m_builder_config.nbg = config["nbg"].as<double>();

    m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
    m_builder_config.near_search_num = config["near_search_num"].as<int>();
    m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
    m_builder_config.gravity_align = config["gravity_align"].as<bool>();
    m_builder_config.esti_il = config["esti_il"].as<bool>();
    std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
    std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
    m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
    m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2],
        r_il_vec[3], r_il_vec[4], r_il_vec[5],
        r_il_vec[6], r_il_vec[7], r_il_vec[8];
    m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

    m_kf->setMaxIter(static_cast<size_t>(m_builder_config.ieskf_max_iter));
}

void FastLio2Node::imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
    double timestamp = Utils::getSec(msg->header);
    if (timestamp < m_state_data.last_imu_time)
    {
        RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
        std::deque<IMUData>().swap(m_state_data.imu_buffer);
    }
    // 注意: Gazebo IMU 已输出 m/s^2, 此处不再 *10 缩放
    m_state_data.imu_buffer.emplace_back(
        V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
        V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
        timestamp);
    m_state_data.last_imu_time = timestamp;
}

void FastLio2Node::lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    CloudType::Ptr cloud = Utils::ros2PCL(msg, m_builder_config.lidar_filter_num,
                                          m_builder_config.lidar_min_range,
                                          m_builder_config.lidar_max_range);
    std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
    double timestamp = Utils::getSec(msg->header);
    if (timestamp < m_state_data.last_lidar_time)
    {
        RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
        std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
    }
    m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
    m_state_data.last_lidar_time = timestamp;
}

bool FastLio2Node::syncPackage()
{
    if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
        return false;
    if (!m_state_data.lidar_pushed)
    {
        m_package.cloud = m_state_data.lidar_buffer.front().second;
        std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(),
                  [](PointType &p1, PointType &p2)
                  { return p1.curvature < p2.curvature; });
        m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
        m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
        m_state_data.lidar_pushed = true;
    }
    if (m_state_data.last_imu_time < m_package.cloud_end_time)
        return false;

    Vec<IMUData>().swap(m_package.imus);
    while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
    {
        m_package.imus.emplace_back(m_state_data.imu_buffer.front());
        m_state_data.imu_buffer.pop_front();
    }
    m_state_data.lidar_buffer.pop_front();
    m_state_data.lidar_pushed = false;
    return true;
}

void FastLio2Node::publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                                CloudType::Ptr cloud, std::string frame_id, const double &time)
{
    if (pub->get_subscription_count() <= 0)
        return;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = Utils::getTime(time);
    pub->publish(cloud_msg);
}

void FastLio2Node::publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub,
                                   std::string frame_id, std::string child_frame, const double &time)
{
    if (odom_pub->get_subscription_count() <= 0)
        return;
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = frame_id;
    odom.header.stamp = Utils::getTime(time);
    odom.child_frame_id = child_frame;
    odom.pose.pose.position.x = m_kf->x().t_wi.x();
    odom.pose.pose.position.y = m_kf->x().t_wi.y();
    odom.pose.pose.position.z = m_kf->x().t_wi.z();
    Eigen::Quaterniond q(m_kf->x().r_wi);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
    odom.twist.twist.linear.x = vel.x();
    odom.twist.twist.linear.y = vel.y();
    odom.twist.twist.linear.z = vel.z();
    odom_pub->publish(odom);
}

void FastLio2Node::publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub,
                               std::string frame_id, const double &time)
{
    if (path_pub->get_subscription_count() <= 0)
        return;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = Utils::getTime(time);
    pose.pose.position.x = m_kf->x().t_wi.x();
    pose.pose.position.y = m_kf->x().t_wi.y();
    pose.pose.position.z = m_kf->x().t_wi.z();
    Eigen::Quaterniond q(m_kf->x().r_wi);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    m_state_data.path.poses.push_back(pose);
    path_pub->publish(m_state_data.path);
}

void FastLio2Node::broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster,
                               std::string frame_id, std::string child_frame, const double &time)
{
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.frame_id = frame_id;
    transformStamped.child_frame_id = child_frame;
    transformStamped.header.stamp = Utils::getTime(time);
    Eigen::Quaterniond q(m_kf->x().r_wi);
    V3D t = m_kf->x().t_wi;
    transformStamped.transform.translation.x = t.x();
    transformStamped.transform.translation.y = t.y();
    transformStamped.transform.translation.z = t.z();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    broad_caster->sendTransform(transformStamped);
}

void FastLio2Node::publishGlobalMap()
{
    if (m_builder->status() != BuilderStatus::MAPPING || m_global_map_pub->get_subscription_count() <= 0)
        return;
    std::vector<PointType, Eigen::aligned_allocator<PointType>> points;
    m_builder->lidar_processor()->collectGlobalMap(points);
    if (points.empty())
        return;
    CloudType global_cloud;
    for (auto &p : points)
        global_cloud.push_back(p);
    global_cloud.width = global_cloud.size();
    global_cloud.height = 1;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(global_cloud, cloud_msg);
    cloud_msg.header.frame_id = m_node_config.world_frame;
    cloud_msg.header.stamp = this->now();
    m_global_map_pub->publish(cloud_msg);
    RCLCPP_INFO(this->get_logger(), "Published global map with %zu points", global_cloud.size());
}

void FastLio2Node::timerCB()
{
    if (!syncPackage())
        return;
    auto t1 = std::chrono::high_resolution_clock::now();
    m_builder->process(m_package);
    auto t2 = std::chrono::high_resolution_clock::now();

    if (m_node_config.print_time_cost)
    {
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
        RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
    }

    if (m_builder->status() != BuilderStatus::MAPPING)
        return;

    broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);
    publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

    CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);
    publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

    CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());
    publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

    publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
}
