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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
    /// 实时栅格: 走核心 generateGridMap (逐帧观测 + 地面处理), 与存 pgm 同一条路径。
    bool getRealtimeGrid(double resolution, rosiwit_slam::GridInfo& out);
    /// **唯一的初始位姿入口** (/initialpose 话题与 /switch_map、/relocation 服务都走这里)。
    /// 负责: ① insane 值归零(可选) ② z 归零(可选, 旧实现的 /switch_map 约定)
    ///       ③ 车身(base_link) -> 雷达 的安装变换换算 ④ setInitPose。
    /// zero_if_insane / force_z_zero 是旧实现里**两类调用方**的差异, 作为参数保留,
    /// 但换算逻辑只有这一份。
    void applyInitPose(const geometry_msgs::PoseWithCovarianceStamped& in,
                       bool zero_if_insane, bool force_z_zero = false);
    /// 按需触发一次全局重定位 (转发到核心的定位流水线)
    void requestRelocalization();
    bool getGridInfo(rosiwit_slam::GridInfo& out) const;
    bool getPoseCov(rosiwit_slam::PoseStamped& out, Eigen::Matrix<double, 6, 6>& cov) const;
    /// 定位匹配度 [0,1] (旧接口 slam/info.matching_degree); 非定位/未支持时为 0
    double getMatchingDegree() const;
    /// 丢掉核心的建图状态并重建 (= 旧实现的"删掉建图实例再 new"), 用于进建图模式前清干净。
    bool resetCore();
    std::string pipelineMode() const;

    // ---- 泵 / 钩子 ----
    /// 注册 50Hz tick (在排空核心输出之后按注册顺序调用)。两套接口各挂一个:
    /// 这样"排空 + 发布"仍发生在同一个回调里, 时序与重构前的 processCB 完全一致。
    void addTickHook(std::function<void()> hook);
    /// 每次核心产出时调用。**在核心自己的线程上执行**, 跨线程用要自己同步。
    void setOutputHook(std::function<void(const rosiwit_slam::SlamOutput&)> hook);
    /// [T6] 每收到一帧输入就调用 (`is_lidar=true` 表示雷达帧)。
    /// 旧接口用它复刻 legacy 的 `get_imu_data_`/`get_laser_data_` 门控 (legacy.cpp:351)。
    void setInputSeenHook(std::function<void(bool is_lidar)> hook);
    /// [T6] 每收到一帧雷达点云就调用 —— 旧接口用它发 `origin_point` /
    /// 定位模式下的 `or_curr_scan_point`。
    /// **在 ROS 回调线程上执行**。给的是已按 lidar_filter_num/min/max_range 处理过的、
    /// **仍在雷达自身坐标系**的云 (与交给核心的是同一份)。旧实现这两个话题也用
    /// `frame_id="map"` 发布 (xz_slam_ros1_interface.cpp:1053/3115), 薄层照抄该约定;
    /// 即这两个"原始点"话题的点是**未配准到 map 系**的输入点, frame_id 仅作兼容标记。
    /// 另注: 旧实现的 origin_point 是更原始的 CustomMsg 转换结果 (含 delete/z 过滤差异)。
    void setRawLidarHook(std::function<void(const rosiwit_slam::CloudType&, const ros::Time&)> hook);

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
        // 可选输入话题 (与核心 YAML 的 odom_topic / absolute_topic / vision.vo_topic 对应)
        std::string odom_topic = "/odom";
        std::string vo_topic = "/visual_odometry";
        std::string absolute_topic = "/absolute_pose";
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
    std::function<void(bool)> input_seen_hook_;
    std::function<void(const rosiwit_slam::CloudType&, const ros::Time&)> raw_lidar_hook_;

    // legacy profile: 优先从 TF 推导 r_il/t_il (雷达-IMU 外参), 而不是写死在配置里
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    /// 推导 T_(imu <- lidar). 顺序: ① 两帧同名 -> 恒等; ② TF 里查得到 -> 用 TF;
    /// ③ 都没有 -> 回落到核心 YAML 里的值。source 说明用了哪一种。
    bool computeLegacyExtrinsic(Eigen::Matrix3d& R, Eigen::Vector3d& t, std::string& source);

public:
    /// 车身(base_link) <- 雷达 的安装变换, 来自 TF, 首次用时取一次。
    /// 用于把**上层下发的车身系初始位姿**换算成核心要的雷达系 (见 onInitialPoseMsg)。
    bool lidarToBase(Eigen::Matrix3d* R = nullptr, Eigen::Vector3d* t = nullptr);
    bool computeLidarToBase();                    // 取一次, 失败则保持单位阵

private:
    Eigen::Matrix3d r_lb_ = Eigen::Matrix3d::Identity();   // base <- lidar
    Eigen::Vector3d t_lb_ = Eigen::Vector3d::Zero();
    bool have_lidar_mount_ = false;
    ros::Time mount_last_try_;          // 上次尝试查询安装变换的时刻 (2s 节流)
    std::string mount_lidar_frame_ = "lidar";
    std::string mount_base_frame_ = "base_link";
};

} // namespace rosiwit_slam_ros1
