// src/livo_replay.cpp
// 离线回放：直接读 rosbag2，顺序喂 FastLivo2Frontend，输出 TUM 位姿。
// 绕过 ROS executor/定时器/回调组，用于验证核心 LIVO 算法。

#include <fstream>
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>

#include <opencv2/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "algorithms/fast_livo2/fast_livo2_frontend.h"
#include "ros_interface/ros_utils.h"

using namespace rosiwit_slam;

namespace {

cv::Mat toCvMat(const sensor_msgs::msg::Image& msg) {
    cv::Mat img;
    auto* data = const_cast<uint8_t*>(msg.data.data());
    if (msg.encoding == "mono8") {
        img = cv::Mat(msg.height, msg.width, CV_8UC1, data, msg.step).clone();
    } else if (msg.encoding == "bgr8") {
        img = cv::Mat(msg.height, msg.width, CV_8UC3, data, msg.step).clone();
    } else if (msg.encoding == "rgb8") {
        cv::Mat tmp(msg.height, msg.width, CV_8UC3, data, msg.step);
        cv::cvtColor(tmp, img, cv::COLOR_RGB2BGR);
    } else if (msg.encoding == "bgra8") {
        cv::Mat tmp(msg.height, msg.width, CV_8UC4, data, msg.step);
        cv::cvtColor(tmp, img, cv::COLOR_BGRA2BGR);
    } else if (msg.encoding == "rgba8") {
        cv::Mat tmp(msg.height, msg.width, CV_8UC4, data, msg.step);
        cv::cvtColor(tmp, img, cv::COLOR_RGBA2BGR);
    }
    return img;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: livo_replay <config.yaml> <bag_dir> [out.txt] "
                     "[lidar_topic imu_topic image_topic]\n";
        return 1;
    }
    rclcpp::init(argc, argv);

    const std::string config_path = argv[1];
    const std::string bag_path = argv[2];
    const std::string out_path = argc > 3 ? argv[3] : "livo_odom.txt";
    const std::string lidar_topic = argc > 4 ? argv[4] : "/os1_cloud_node1/points";
    const std::string imu_topic = argc > 5 ? argv[5] : "/imu/imu";
    const std::string image_topic = argc > 6 ? argv[6] : "/left/image_raw";

    FastLivo2Frontend frontend;
    if (!frontend.init(config_path)) {
        std::cerr << "frontend init failed\n";
        return 2;
    }

    std::ofstream out(out_path);
    out << std::fixed;

    rosbag2_cpp::Reader reader;
    reader.open(bag_path);

    rclcpp::Serialization<sensor_msgs::msg::Imu> ser_imu;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser_cloud;
    rclcpp::Serialization<sensor_msgs::msg::Image> ser_image;

    size_t n_imu = 0, n_lidar = 0, n_img = 0, n_pose = 0;
    auto pose_from_state = [&](const PoseStamped& p) {
        const Eigen::Quaterniond q(p.rot);
        out << p.time << ' ' << p.trans.x() << ' ' << p.trans.y() << ' ' << p.trans.z()
            << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
        ++n_pose;
    };

    while (reader.has_next()) {
        auto bag_msg = reader.read_next();
        rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
        const std::string& topic = bag_msg->topic_name;

        if (topic == imu_topic) {
            sensor_msgs::msg::Imu msg;
            ser_imu.deserialize_message(&serialized, &msg);
            IMUSample s;
            s.time = RosUtils::getSec(msg.header);
            s.acc = V3D(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
            s.gyro = V3D(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
            frontend.onImu(s);
            ++n_imu;
        } else if (topic == lidar_topic) {
            sensor_msgs::msg::PointCloud2 msg;
            ser_cloud.deserialize_message(&serialized, &msg);
            auto ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(msg);
            LidarFrame f;
            f.cloud = RosUtils::ros2PCL(ptr, 1, 0.0, 1e6, 100.0f);
            if (!f.cloud || f.cloud->empty()) continue;
            std::sort(f.cloud->points.begin(), f.cloud->points.end(),
                      [](const PointType& a, const PointType& b) { return a.curvature < b.curvature; });
            f.start_time = RosUtils::getSec(msg.header);
            f.end_time = f.start_time + f.cloud->points.back().curvature / 1000.0;
            frontend.onLidar(f);
            PoseStamped p;
            if (frontend.getOdometry(p)) pose_from_state(p);
            ++n_lidar;
        } else if (topic == image_topic) {
            sensor_msgs::msg::Image msg;
            ser_image.deserialize_message(&serialized, &msg);
            ImageFrame f;
            f.image = toCvMat(msg);
            if (f.image.empty()) continue;
            f.time = RosUtils::getSec(msg.header);
            f.frame_id = msg.header.frame_id;
            frontend.onImage(f);
            ++n_img;
        }
        if ((n_lidar + n_imu + n_img) % 20000 == 0) {
            std::cerr << "imu=" << n_imu << " lidar=" << n_lidar << " img=" << n_img
                      << " poses=" << n_pose << std::endl;
        }
    }

    std::cerr << "done: imu=" << n_imu << " lidar=" << n_lidar << " img=" << n_img
              << " poses=" << n_pose << std::endl;
    rclcpp::shutdown();
    return 0;
}
