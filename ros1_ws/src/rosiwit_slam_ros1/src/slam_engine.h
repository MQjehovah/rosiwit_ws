// ============================================================================
// SlamEngine —— 算法核心的唯一持有者 (计划 T4)
//
// 本类里的内容是从原 `rosiwit_slam_ros1_node.cpp` **原样搬出来**的:
//   核心 YAML 装载 -> SlamFactory::create -> init -> 输入订阅 -> 输出快照 -> 50Hz 泵
//
// 抽出来的唯一理由: 让两套 ROS 接口(rosiwit 新接口 / 旧 3d_slam 接口)共用**同一个**
// 核心实例 —— 一个进程、一个核心、两套接口同时对外, 而不是起两个进程各跑一份 SLAM。
//
// **行为契约**: `input_profile=rosiwit` 下本类行为必须与重构前逐位一致
// (话题名、发布频率、TF、服务名都不变)。`legacy` 只额外做一件事:
// 输入话题名改由旧配置(见 T1 的 LegacyConfig)决定。
// ============================================================================
#pragma once

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "slam_core/i_slam_algorithm.h"
#include "slam_core/slam_base.h"
#include "slam_core/slam_types.h"

#include "legacy_config.h"

namespace rosiwit_slam_ros1 {

/// 旧 3d_slam 的四个配置来源在**本包**里的等价路径。
/// 旧实现用的是编译期宏 PROJECT_PATH (xz_3d_slam/CMakeLists.txt:24/26):
/// 装机后 = /opt/ros/melodic/share/xz_3d_slam, 开发时 = 源码目录。
/// ros::package::getPath 恰好两种情况都能给出正确目录, 所以用它等价替换。
LegacyConfPaths defaultLegacyPaths();

class SlamEngine {
public:
    SlamEngine(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~SlamEngine();

    SlamEngine(const SlamEngine&) = delete;
    SlamEngine& operator=(const SlamEngine&) = delete;

    /// init 成功才为 true。为 false 时 core 可能为 nullptr, 调用方不应再建接口层。
    bool ok() const { return ok_; }
    rosiwit_slam::ISlamAlgorithm* algo() const { return algo_raw_; }
    rosiwit_slam::SlamBase* base() const { return base_; }

    // ---- profile ----
    const std::string& inputProfile() const { return input_profile_; }
    bool legacyProfile() const { return input_profile_ == "legacy"; }
    const std::string& configPath() const { return config_path_; }
    /// 仅 legacy profile 有意义 (否则是默认构造的全默认值)
    const LegacyConfig& legacyConfig() const { return legacy_cfg_; }

    // ---- 帧名 / 话题名 ----
    const std::string& worldFrame() const { return cfg_.world_frame; }
    const std::string& bodyFrame() const { return cfg_.body_frame; }
    const std::string& imuTopic() const { return cfg_.imu_topic; }
    const std::string& lidarTopic() const { return cfg_.lidar_topic; }

    // ---- 输出/地图 快照 (线程安全) ----
    /// 有输出就拷到 out 返回 true。**不过滤** has_new_pose —— 调用方自己看
    /// (与重构前的 processCB 一致: 反复发布最后一份输出)。
    bool latestOutput(rosiwit_slam::SlamOutput& out) const;
    bool getGlobalMap(rosiwit_slam::PointVec& out) const;
    bool getGridInfo(rosiwit_slam::GridInfo& out) const;
    bool getPoseCov(rosiwit_slam::PoseStamped& out, Eigen::Matrix<double, 6, 6>& cov) const;
    std::string pipelineMode() const;

    // ---- 泵 / 钩子 ----
    /// 注册 50Hz tick (在排空核心输出之后按注册顺序调用)。两套接口各挂一个:
    /// 这样"排空 + 发布"仍发生在同一个回调里, 时序与重构前的 processCB 完全一致。
    void addTickHook(std::function<void()> hook);
    /// 每次核心产出时调用。**在核心自己的线程上执行**, 跨线程用要自己同步。
    void setOutputHook(std::function<void(const rosiwit_slam::SlamOutput&)> hook);

    // ---- 输入 (公开给测试/其它接口复用) ----
    void onImuMsg(const sensor_msgs::Imu::ConstPtr& msg);
    void onLidarMsg(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void onOdomMsg(const nav_msgs::Odometry::ConstPtr& msg);
    void onInitialPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

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

    /// 与重构前逐字一致: 抽稀 + 去 NaN + 距离裁剪
    static rosiwit_slam::CloudType::Ptr ros2Pcl(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                               int filter_num, double min_r, double max_r);

    void imuCB(const sensor_msgs::Imu::ConstPtr& msg);
    void lidarCB(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
    void voCB(const nav_msgs::Odometry::ConstPtr& msg);
    void absoluteCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void onOutput(const rosiwit_slam::SlamOutput& out);
    void processCB(const ros::TimerEvent&);

    ros::NodeHandle nh_, pnh_;
    Cfg cfg_;

    std::string input_profile_ = "rosiwit";
    std::string config_path_;
    LegacyConfig legacy_cfg_;

    std::unique_ptr<rosiwit_slam::ISlamAlgorithm> algo_;
    rosiwit_slam::ISlamAlgorithm* algo_raw_ = nullptr;
    rosiwit_slam::SlamBase* base_ = nullptr;
    bool ok_ = false;

    ros::Subscriber imu_sub_, lidar_sub_, init_pose_sub_, odom_sub_, vo_sub_, absolute_sub_;
    ros::Timer process_timer_;

    mutable std::mutex out_mutex_;
    rosiwit_slam::SlamOutput latest_;
    bool have_output_ = false;

    std::vector<std::function<void()>> tick_hooks_;
    std::function<void(const rosiwit_slam::SlamOutput&)> output_hook_;
};

} // namespace rosiwit_slam_ros1
