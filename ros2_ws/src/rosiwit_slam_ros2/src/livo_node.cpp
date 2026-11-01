// src/livo_node.cpp
// 专用 FAST-LIVO2 管线：直接驱动核心 FastLivo2Frontend，播包出图。
// 与通用 SlamPipeline 解耦，避免 executor/定时器/地图回调的互相干扰。

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <csignal>
#include <ctime>
#include <deque>
#include <execinfo.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>

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

// ---- int64 体素降采样 (pcl::VoxelGrid 用 int32 索引, 大范围小 leaf 时会溢出失效) ----
namespace {

struct VoxelAccum {
    double x = 0.0, y = 0.0, z = 0.0;
    double r = 0.0, g = 0.0, b = 0.0;
    uint32_t n = 0;
};

inline int64_t voxelKey(int64_t ix, int64_t iy, int64_t iz) {
    constexpr int64_t MASK = (1LL << 21) - 1;
    return ((ix & MASK) << 42) | ((iy & MASK) << 21) | (iz & MASK);
}

void voxelDownsampleInt64(const pcl::PointCloud<pcl::PointXYZRGB>& in, double leaf,
                          pcl::PointCloud<pcl::PointXYZRGB>& out) {
    out.clear();
    if (leaf <= 0.0 || in.empty()) {
        out = in;
        return;
    }
    const double inv_leaf = 1.0 / leaf;
    const int64_t limit = (1LL << 20) - 1;
    std::unordered_map<int64_t, VoxelAccum> voxels;
    voxels.reserve(in.size() / 4 + 1);
    for (const auto& p : in.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const int64_t ix = static_cast<int64_t>(std::floor(p.x * inv_leaf));
        const int64_t iy = static_cast<int64_t>(std::floor(p.y * inv_leaf));
        const int64_t iz = static_cast<int64_t>(std::floor(p.z * inv_leaf));
        if (ix < -limit || ix > limit || iy < -limit || iy > limit || iz < -limit || iz > limit) continue;
        VoxelAccum& a = voxels[voxelKey(ix, iy, iz)];
        a.x += p.x;
        a.y += p.y;
        a.z += p.z;
        a.r += p.r;
        a.g += p.g;
        a.b += p.b;
        ++a.n;
    }
    out.reserve(voxels.size());
    for (const auto& kv : voxels) {
        const VoxelAccum& a = kv.second;
        if (a.n == 0) continue;
        pcl::PointXYZRGB p;
        p.x = static_cast<float>(a.x / a.n);
        p.y = static_cast<float>(a.y / a.n);
        p.z = static_cast<float>(a.z / a.n);
        p.r = static_cast<uint8_t>(a.r / a.n);
        p.g = static_cast<uint8_t>(a.g / a.n);
        p.b = static_cast<uint8_t>(a.b / a.n);
        out.push_back(p);
    }
    out.width = static_cast<uint32_t>(out.size());
    out.height = 1;
    out.is_dense = false;
}

}  // namespace

// 崩溃时打印 C 栈回溯（无 gdb 环境下定位 SIGSEGV/SIGFPE）
void installCrashHandler() {
    static auto handler = [](int sig) {
        void* frames[40];
        const int n = ::backtrace(frames, 40);
        char buf[256];
        const int len = std::snprintf(buf, sizeof(buf), "\n[fast_livo2] signal %d, backtrace:\n", sig);
        if (len > 0) ::write(STDERR_FILENO, buf, static_cast<size_t>(len));
        ::backtrace_symbols_fd(frames, n, STDERR_FILENO);
        ::signal(sig, SIG_DFL);
        ::raise(sig);
    };
    ::signal(SIGSEGV, handler);
    ::signal(SIGFPE, handler);
    ::signal(SIGABRT, handler);
    ::signal(SIGBUS, handler);
    ::signal(SIGILL, handler);
}

class LivoNode : public rclcpp::Node {
public:
    LivoNode() : rclcpp::Node("fast_livo2") {
        config_path_ = declare_parameter<std::string>("config_path", "");
        output_pcd_ = declare_parameter<std::string>("output_pcd", "livo_map.pcd");

        if (config_path_.empty()) {
            throw std::runtime_error("config_path is required");
        }
        loadTopics(config_path_);

        frontend_.reset(new FastLivo2Frontend());
        if (!frontend_->init(config_path_)) {
            throw std::runtime_error("FastLivo2Frontend init failed: " + config_path_);
        }

        rclcpp::SubscriptionOptions opts;
        opts.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        // 可靠传输: 播包时 best_effort 会静默丢 ~1/3 的点云帧, 离线播包宁可慢也不能丢帧.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(500)).reliable();

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr m) { onImu(m); }, opts);
        sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { onLidar(m); }, opts);
        if (compressed_) {
            sub_img_comp_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                image_topic_, qos,
                [this](sensor_msgs::msg::CompressedImage::SharedPtr m) { onCompressed(m); }, opts);
        } else {
            sub_img_ = create_subscription<sensor_msgs::msg::Image>(
                image_topic_, qos, [this](sensor_msgs::msg::Image::SharedPtr m) { onImage(m); }, opts);
        }

        pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("lio_odom", 100);
        pub_path_ = create_publisher<nav_msgs::msg::Path>("lio_path", 10);
        pub_body_ = create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10);
        pub_world_ = create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10);
        pub_rgb_ = create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud_rgb", 10);
        pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_map", 2);
        map_pub_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publishMap(); });

        path_.header.frame_id = world_frame_;
        path_.header.stamp = now();

        worker_ = std::thread([this] { workerLoop(); });
        RCLCPP_INFO(get_logger(), "fast_livo2 pipeline: lidar=%s imu=%s image=%s",
                    lidar_topic_.c_str(), imu_topic_.c_str(), image_topic_.c_str());
    }

    ~LivoNode() override { shutdown(); }

    /// 停止收帧 -> 把队列里的帧处理完 -> 存图.
    void shutdown() {
        if (finished_) return;
        finished_ = true;
        stop_ = true;
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
        saveMap();
    }

private:
    void loadTopics(const std::string& path) {
        YAML::Node root = YAML::LoadFile(path);
        YAML::Node in = root["global"]["input"];
        YAML::Node fr = root["global"]["frames"];
        auto get = [&](YAML::Node n, const char* k, const std::string& def) {
            return (n && n[k]) ? n[k].as<std::string>() : def;
        };
        lidar_topic_ = get(in, "lidar_topic", "/os1_cloud_node1/points");
        imu_topic_ = get(in, "imu_topic", "/imu/imu");
        image_topic_ = get(in, "image_topic", "/left/image_raw");
        compressed_ = (in && in["enable_image_processing"]) ? in["enable_image_processing"].as<bool>() : false;
        world_frame_ = get(fr, "world_frame", "map");
        body_frame_ = get(fr, "body_frame", "base_link");
    }

    void onImu(const sensor_msgs::msg::Imu::SharedPtr& m) {
        IMUSample s;
        s.time = RosUtils::getSec(m->header);
        s.acc = V3D(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
        s.gyro = V3D(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
        {
            std::lock_guard<std::mutex> lk(mtx_);
            imu_q_.push_back(s);
        }
        cv_.notify_one();
    }

    void onImage(const sensor_msgs::msg::Image::SharedPtr& m) {
        ImageFrame f;
        f.image = toCvMat(*m);
        if (f.image.empty()) return;
        f.time = RosUtils::getSec(m->header);
        f.frame_id = m->header.frame_id;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            img_q_.push_back(std::move(f));
        }
        cv_.notify_one();
    }

    void onCompressed(const sensor_msgs::msg::CompressedImage::SharedPtr& m) {
        cv::Mat buf(1, static_cast<int>(m->data.size()), CV_8UC1,
                    const_cast<uint8_t*>(m->data.data()));
        cv::Mat img = cv::imdecode(buf, cv::IMREAD_COLOR);
        if (img.empty()) return;
        ImageFrame f;
        f.image = img;
        f.time = RosUtils::getSec(m->header);
        f.frame_id = m->header.frame_id;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            img_q_.push_back(std::move(f));
        }
        cv_.notify_one();
    }

    void onLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& m) {
        LidarFrame f;
        f.cloud = RosUtils::ros2PCL(m, 1, 0.0, 1e6, 100.0f);
        if (!f.cloud || f.cloud->empty()) return;
        std::sort(f.cloud->points.begin(), f.cloud->points.end(),
                  [](const PointType& a, const PointType& b) { return a.curvature < b.curvature; });
        f.start_time = RosUtils::getSec(m->header);
        f.end_time = f.start_time + f.cloud->points.back().curvature / 1000.0;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            lidar_q_.push_back(f);
            ++n_lidar_rx_;
        }
        cv_.notify_one();
    }

    void workerLoop() {
        auto t_start = std::chrono::steady_clock::now();
        while (true) {
            LidarFrame f;
            {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.wait_for(lk, std::chrono::milliseconds(50), [this] {
                    return !lidar_q_.empty() || stop_;
                });
                if (lidar_q_.empty()) {
                    if (stop_) break;   // 队列已清空 -> 收工
                    continue;
                }
                f = lidar_q_.front();
                lidar_q_.pop_front();
                q_lidar_max_ = std::max(q_lidar_max_, lidar_q_.size());

                for (const auto& s : imu_q_) frontend_->onImu(s);
                imu_q_.clear();
                for (const auto& im : img_q_) frontend_->onImage(im);
                img_q_.clear();
            }

            const auto t_a = std::chrono::steady_clock::now();
            frontend_->onLidar(f);
            const auto t_b = std::chrono::steady_clock::now();
            ++n_lidar_done_;
            publishOutputs();
            const auto t_c = std::chrono::steady_clock::now();
            t_core_ += std::chrono::duration<double>(t_b - t_a).count();
            t_pub_ += std::chrono::duration<double>(t_c - t_b).count();
            if (n_lidar_done_ % 200 == 0) {
                const double el = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t_start).count();
                RCLCPP_INFO(get_logger(),
                            "processed %zu/%zu lidar in %.1f s (%.1f ms/fr); core %.1f ms, pub %.1f ms, "
                            "cpu %.1f%%, qmax %zu",
                            n_lidar_done_, n_lidar_rx_, el,
                            1000.0 * el / static_cast<double>(n_lidar_done_),
                            1000.0 * t_core_ / 200.0, 1000.0 * t_pub_ / 200.0,
                            100.0 * (std::clock() - c_start_) / static_cast<double>(CLOCKS_PER_SEC) / el,
                            q_lidar_max_);
                t_core_ = t_pub_ = 0.0;
                q_lidar_max_ = 0;
            }
        }
    }

    void publishOutputs() {
        PoseStamped pose;
        if (!frontend_->getOdometry(pose)) return;

        const builtin_interfaces::msg::Time stamp = RosUtils::getTime(pose.time);
        const Eigen::Quaterniond q(pose.rot);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = world_frame_;
        odom.child_frame_id = body_frame_;
        odom.pose.pose.position.x = pose.trans.x();
        odom.pose.pose.position.y = pose.trans.y();
        odom.pose.pose.position.z = pose.trans.z();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        pub_odom_->publish(odom);

        // 重话题降频: path/点云 1 Hz, rgb 5 Hz, 位姿 10 Hz.
        const auto t_now = std::chrono::steady_clock::now();
        const double since_path = std::chrono::duration<double>(t_now - last_path_).count();
        const double since_cloud = std::chrono::duration<double>(t_now - last_cloud_).count();
        const double since_rgb = std::chrono::duration<double>(t_now - last_rgb_).count();

        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = stamp;
        ps.header.frame_id = world_frame_;
        ps.pose = odom.pose.pose;
        path_.poses.push_back(ps);
        if (since_path >= 1.0 && pub_path_->get_subscription_count() > 0) {
            last_path_ = t_now;
            path_.header.stamp = stamp;
            pub_path_->publish(path_);
        }

        CloudType::Ptr body, world;
        frontend_->getClouds(body, world);
        const bool want_cloud = since_cloud >= 1.0;
        if (want_cloud) {
            last_cloud_ = t_now;
            auto pub = [&](auto& p, const CloudType::Ptr& c) {
                if (!c || c->empty() || p->get_subscription_count() == 0) return;
                sensor_msgs::msg::PointCloud2 m;
                pcl::toROSMsg(*c, m);
                m.header.frame_id = world_frame_;
                m.header.stamp = stamp;
                p->publish(m);
            };
            pub(pub_body_, body);
            pub(pub_world_, world);
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb;
        const bool have_rgb = frontend_->getRGBCloud(rgb) && rgb && !rgb->empty();
        if (have_rgb && since_rgb >= 0.2 && pub_rgb_->get_subscription_count() > 0) {
            last_rgb_ = t_now;
            sensor_msgs::msg::PointCloud2 m;
            pcl::toROSMsg(*rgb, m);
            m.header.frame_id = world_frame_;
            m.header.stamp = stamp;
            pub_rgb_->publish(m);
        }
        {
            std::lock_guard<std::mutex> lk(map_mtx_);
            if (have_rgb) {
                for (const auto& p : rgb->points) {
                    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                    if (std::abs(p.x) > 1000 || std::abs(p.y) > 1000 || std::abs(p.z) > 1000) continue;
                    map_rgb_->push_back(p);
                }
                ++n_rgb_frames_;
            } else if (world && !world->empty()) {
                for (const auto& p : world->points) {
                    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                    if (std::abs(p.x) > 2000 || std::abs(p.y) > 2000 || std::abs(p.z) > 2000) continue;
                    pcl::PointXYZRGB c;
                    c.x = p.x; c.y = p.y; c.z = p.z;
                    c.r = c.g = c.b = 200;
                    map_rgb_->push_back(c);
                }
                ++n_world_frames_;
            } else {
                ++n_empty_frames_;
            }
            ++map_frames_;
        }
    }

    void downsampleMap() {
        pcl::PointCloud<pcl::PointXYZRGB> out;
        voxelDownsampleInt64(*map_rgb_, 0.1, out);
        std::cerr << "[fast_livo2] voxel 0.1 m: " << map_rgb_->size() << " -> " << out.size()
                  << " points (int64 grid)" << std::endl;
        auto ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        ptr->swap(out);
        map_rgb_ = ptr;
    }

    void publishMap() {
        if (pub_map_->get_subscription_count() == 0) return;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        {
            std::lock_guard<std::mutex> lk(map_mtx_);
            if (!map_rgb_ || map_rgb_->empty()) return;
            cloud = map_rgb_;
        }
        sensor_msgs::msg::PointCloud2 m;
        pcl::toROSMsg(*cloud, m);
        m.header.frame_id = world_frame_;
        m.header.stamp = now();
        pub_map_->publish(m);
    }

    void saveMap() {
        bool ok = false;
        size_t n_raw = 0, n_out = 0;
        {
            std::lock_guard<std::mutex> lk(map_mtx_);
            std::cerr << "[fast_livo2] map accumulate: rgb_frames=" << n_rgb_frames_
                      << " gray_frames=" << n_world_frames_ << " empty_frames=" << n_empty_frames_
                      << " raw_points=" << (map_rgb_ ? map_rgb_->size() : 0) << std::endl;
            if (!map_rgb_ || map_rgb_->empty()) return;
            n_raw = map_rgb_->size();
            downsampleMap();
            n_out = map_rgb_->size();
            ok = pcl::io::savePCDFileBinary(output_pcd_, *map_rgb_) == 0;
            pcl::PointCloud<pcl::PointXYZRGB>().swap(*map_rgb_);
        }
        std::cerr << "[fast_livo2] map " << (ok ? "saved" : "save FAILED") << ": " << output_pcd_
                  << " raw=" << n_raw << " points=" << n_out << std::endl;
    }

    std::string config_path_, output_pcd_, lidar_topic_, imu_topic_, image_topic_;
    std::string world_frame_ = "map", body_frame_ = "base_link";
    bool compressed_ = false;

    std::unique_ptr<FastLivo2Frontend> frontend_;

    std::mutex mtx_;
    std::mutex map_mtx_;
    std::condition_variable cv_;
    std::deque<IMUSample> imu_q_;
    std::deque<ImageFrame> img_q_;
    std::deque<LidarFrame> lidar_q_;
    std::thread worker_;
    std::atomic<bool> stop_{false};
    bool finished_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img_comp_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_body_, pub_world_, pub_rgb_, pub_map_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;

    nav_msgs::msg::Path path_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_rgb_{new pcl::PointCloud<pcl::PointXYZRGB>()};
    int map_frames_ = 0;
    size_t n_rgb_frames_ = 0;
    size_t n_world_frames_ = 0;
    size_t n_empty_frames_ = 0;
    std::chrono::steady_clock::time_point last_path_{};
    std::chrono::steady_clock::time_point last_cloud_{};
    std::chrono::steady_clock::time_point last_rgb_{};
    size_t n_lidar_done_ = 0;
    size_t n_lidar_rx_ = 0;
    size_t q_lidar_max_ = 0;
    std::clock_t c_start_ = std::clock();
    double t_core_ = 0.0;
    double t_pub_ = 0.0;
};

static std::atomic<bool> g_signal{false};

static void requestStop(int) { g_signal = true; }

int main(int argc, char** argv) {
    installCrashHandler();
    rclcpp::init(argc, argv);
    // 自己接管 SIGINT/SIGTERM: 先让 worker 把队列处理完, 再存图退出.
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    try {
        rclcpp::executors::MultiThreadedExecutor exec;
        auto node = std::make_shared<LivoNode>();
        exec.add_node(node);
        std::thread spinner([&exec] { exec.spin(); });
        while (rclcpp::ok() && !g_signal) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(node->get_logger(), "stop requested, draining queues...");
        node->shutdown();
        rclcpp::shutdown();
        if (spinner.joinable()) spinner.join();
    } catch (const std::exception& e) {
        std::cerr << "fast_livo2 fatal: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
