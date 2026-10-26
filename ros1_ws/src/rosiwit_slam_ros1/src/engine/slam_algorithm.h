// ============================================================================
// SlamAlgorithm —— 算法核心的唯一持有者 (计划 T4)
//
// 本类里的内容是从原 `rosiwit_slam_ros1_node.cpp` **原样搬出来**的:
//   核心 YAML 装载 -> SlamFactory::create -> init -> 输入订阅 -> 输出快照 -> 50Hz 泵
//
// 一个进程、一个核心实例、一个 ROS1 对外接口；输入话题由机器人现有
// sensor_config(见 Xz3dConfig) 决定，避免与底盘现有驱动/launch 配置脱节。
// ============================================================================
#pragma once

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "frame_utils.h"
#include "slam_core/i_slam_algorithm.h"
#include "slam_core/slam_base.h"
#include "slam_core/slam_types.h"
#include "slam_core/online_calibrator.h"
#include "slam_core/calibration_controller.h"

#include "rosiwit_common/channel.hpp"
#include "rosiwit_common/executor.hpp"
#include "rosiwit_common/periodic.hpp"
#include "rosiwit_common/thread.hpp"

#include "config.h"

namespace rosiwit_slam_ros1 {

/// 旧 3d_slam 的四个配置来源在**本包**里的等价路径。
/// 旧实现用的是编译期宏 PROJECT_PATH (xz_3d_slam/CMakeLists.txt:24/26):
/// 装机后 = /opt/ros/melodic/share/xz_3d_slam, 开发时 = 源码目录。
/// ros::package::getPath 恰好两种情况都能给出正确目录, 所以用它等价替换。
Xz3dConfPaths defaultXz3dPaths();
Xz3dConfPaths defaultXz3dPaths(ros::NodeHandle& pnh);

class SlamAlgorithm {
public:
    /// executor: 由组装根注入的通用执行器 (可为 nullptr——只做同步后台任务)。
    /// 约定: 本类不创建线程池, 只用注入的执行器; 自带的两条线程是
    ///       FrontEnd(帧序) 与 Tick(50Hz 发布), 属于"顺序执行器"。
    SlamAlgorithm(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                  rosiwit_common::IExecutor* executor = nullptr);

    /// 启停 (由组装根控制): start() 起 FrontEnd+Tick 线程, shutdown() 逆序关停。
    void start();
    void shutdown();
    ~SlamAlgorithm();

    SlamAlgorithm(const SlamAlgorithm&) = delete;
    SlamAlgorithm& operator=(const SlamAlgorithm&) = delete;

    /// init 成功才为 true。为 false 时 core 可能为 nullptr, 调用方不应再建接口层。
    bool ok() const { return ok_; }
    rosiwit_slam::ISlamAlgorithm* algo() const { return algo_raw_; }
    rosiwit_slam::SlamBase* base() const { return base_; }

    const std::string& configPath() const { return config_path_; }
    const Xz3dConfig& xzConfig() const { return xz_cfg_; }

    // ---- 帧名 / 话题名 ----
    const std::string& worldFrame() const { return cfg_.world_frame; }
    const std::string& bodyFrame() const { return cfg_.body_frame; }
    const std::string& imuTopic() const { return cfg_.imu_topic; }
    const std::string& lidarTopic() const { return cfg_.lidar_topic; }
    /// 可选输入是否启用 (由核心 YAML 决定); 订阅由 SlamRosNode 负责
    bool useVo() const { return vo_enabled_; }
    bool useAbsolute() const { return absolute_enabled_; }
    const std::string& voTopic() const { return cfg_.vo_topic; }
    const std::string& absoluteTopic() const { return cfg_.absolute_topic; }

    // ---- 输出/地图 快照 (线程安全) ----
    /// 有输出就拷到 out 返回 true。**不过滤** has_new_pose —— 调用方自己看
    /// (与重构前的 processCB 一致: 反复发布最后一份输出)。
    bool latestOutput(rosiwit_slam::SlamOutput& out) const;
    /// 丢掉最近一次核心输出快照 (切图时清掉上一张图的位姿，避免旧输出被重发)
    void clearLatestOutput();
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
    /// 飞行记录仪手动落盘; 返回文件路径(空=无数据) —— /slam/dump_recorder 服务用
    std::string dumpFlightRecorder(const std::string& reason);
    bool getGridInfo(rosiwit_slam::GridInfo& out) const;
    bool getPoseCov(rosiwit_slam::PoseStamped& out, Eigen::Matrix<double, 6, 6>& cov) const;
    /// 定位匹配度 [0,1] (旧接口 slam/info.matching_degree); 非定位/未支持时为 0
    double getMatchingDegree() const;
    /// 运动先验的等效不确定度 (米/弧度); false = 未提供
    bool priorUncertainty(double& xy_sigma, double& yaw_sigma) const;
    /// 丢掉核心的建图状态并重建 (= 旧实现的"删掉建图实例再 new"), 用于进建图模式前清干净。
    bool resetCore();
    std::string pipelineMode() const;
    /// 核心定位健康状态（唯一权威），ROS 层只做协议映射。
    rosiwit_slam::ILocalization::Status localizationStatus() const;
    /// 在线时间偏移估计（只读；由上层决定是否写回配置）。
    rosiwit_slam::TimeCalibrationResult onlineTimeCalibration() const;
    /// 在线 yaw 外参估计（IMU 朝向 vs LIO 朝向；只读）。
    rosiwit_slam::YawCalibrationResult onlineYawCalibration() const;
    /// 闭环标定: 过门限/稳定窗口后应用时间偏移；异常回滚。
    rosiwit_slam::CalibrationDecision updateCalibration();

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
    // ---- 输入入口: 由 SlamRosNode 订阅后转发 (本类不再自己 subscribe) ----
    // 只做入队: 真正的处理在 FrontEnd 线程 (frontendLoop -> processImu/processLidar),
    // 这样 ROS spinner 上不再做 PCL 转换与核心计算。
    void onImuMsg(const sensor_msgs::Imu::ConstPtr& msg);
    void onLidarMsg(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void onOdomMsg(const nav_msgs::Odometry::ConstPtr& msg);
    void onInitialPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void onVoMsg(const nav_msgs::Odometry::ConstPtr& msg) { voCB(msg); }
    void onAbsoluteMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    { absoluteCB(msg); }

    /// 把任务投到注入的执行器 (lane 由使用方声明, 例如 "disk_read").
    /// 返回 false = 被拒绝 (池满/已关停/无执行器)。
    bool runOnLane(const std::string& lane, std::string name,
                   std::function<void(rosiwit_common::CancelToken)> job);

    rosiwit_common::IExecutor* executor() const { return executor_; }

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
        // 机身自过滤: 在 base_link 下按包围盒删除机体自身回波.
        bool self_filter_enabled = false;
        double self_x_min = -0.45, self_x_max = 0.75;
        double self_y_min = -0.55, self_y_max = 0.55;
        double self_z_min = 0.05, self_z_max = 1.40;
    };

    static double getSec(const ros::Time& t) { return t.toSec(); }

    /// 与重构前逐字一致: 抽稀 + 去 NaN + 距离裁剪
    /// scan_period_ms > 0 且点云没有逐点 timestamp 字段时, 按方位角推导逐点时间 (去畸变用)
    static rosiwit_slam::CloudType::Ptr ros2Pcl(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                               int filter_num, double min_r, double max_r,
                                               double scan_period_ms = 0.0);
    /// 删除 base_link 机身盒内的点 (输入点云仍为雷达系, 用 base_lidar_ 变换).
    void applySelfFilter(rosiwit_slam::CloudType::Ptr& cloud);

    void voCB(const nav_msgs::Odometry::ConstPtr& msg);
    void absoluteCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void onOutput(const rosiwit_slam::SlamOutput& out);
    /// 真正的帧处理 (只在 FrontEnd 线程上跑)
    void processImu(const sensor_msgs::Imu::ConstPtr& msg);
    void processLidar(const sensor_msgs::PointCloud2::ConstPtr& msg);
    /// FrontEnd: 唯一消费帧的线程 (替代原 50Hz processCB + input_worker)
    void frontendLoop(rosiwit_common::CancelToken token);
    /// 50Hz tick: 只跑接口挂的发布钩子 (原 processCB 的 tick_hooks 部分)
    void tickLoop();

    // 传感器输入队列: ROS 回调只入队, 重处理在独立线程串行执行。
    // 这样 Lidar 的 PCL 转换和核心 onLidar 不再阻塞 spinner/timer。
    enum class InputKind { IMU, LIDAR };
    struct InputItem {
        InputKind kind;
        sensor_msgs::Imu::ConstPtr imu;
        sensor_msgs::PointCloud2::ConstPtr lidar;
    };
    // 有界输入队列: 满时丢最旧 (过期帧对定位/建图没用), 计数在 Channel 里。
    static constexpr size_t kInputCapacity = 512;
    void enqueueInput(InputItem item);
    void logInputStats();

    ros::NodeHandle nh_, pnh_;
    Cfg cfg_;

    std::string config_path_;
    Xz3dConfig xz_cfg_;

    std::unique_ptr<rosiwit_slam::ISlamAlgorithm> algo_;
    rosiwit_slam::ISlamAlgorithm* algo_raw_ = nullptr;
    rosiwit_slam::SlamBase* base_ = nullptr;
    bool ok_ = false;
    bool online_calib_enabled_ = true;
    bool vo_enabled_ = false;
    bool absolute_enabled_ = false;
    mutable std::mutex calib_mutex_;
    rosiwit_slam::OnlineTimeCalibrator online_time_calib_;
    rosiwit_slam::YawExtrinsicCalibrator yaw_calib_;
    rosiwit_slam::CalibrationController calib_controller_;
    std::atomic<double> imu_time_offset_{0.0};
    double last_imu_yaw_ = 0.0;
    double last_imu_stamp_ = -1.0;
    bool have_imu_yaw_ = false;

    rosiwit_common::IExecutor* executor_ = nullptr;

    std::mutex core_mu_;                 // 保护 algo_ 的输入与重建 (reset vs FE)
    rosiwit_common::Channel<InputItem> input_{kInputCapacity,
      rosiwit_common::OverflowPolicy::kDropOldest};
    std::unique_ptr<rosiwit_common::Thread> frontend_;
    std::unique_ptr<rosiwit_common::PeriodicThread> tick_;
    std::atomic<uint64_t> input_imu_in_{0};
    std::atomic<uint64_t> input_lidar_in_{0};
    std::atomic<uint64_t> input_imu_done_{0};
    std::atomic<uint64_t> input_lidar_done_{0};

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
    bool computeXz3dExtrinsic(Eigen::Matrix3d& R, Eigen::Vector3d& t, std::string& source);

public:
    /// base_link <- 核心 body(IMU) 与 base_link <- lidar 的安装变换.
    /// 行业标准: 算法估计 IMU body; ROS1 接口层负责 base_link 的输入/输出换算.
    /// blocking=true: 启动路径最多等 ~3s; false: 周期回调只试一次(2s 节流).
    bool computeBaseFrames(bool blocking = false);
    const RigidTransform& baseImu() const { return base_imu_; }
    const RigidTransform& baseLidar() const { return base_lidar_; }
    /// 把 odom/VO 的 child frame 换到核心 IMU body; 静态外参查询一次后缓存.
    bool imuFromChild(const std::string& child_frame, RigidTransform& out);

private:
    RigidTransform base_imu_;      // T_base_link_imu
    RigidTransform base_lidar_;    // T_base_link_lidar (BBS 投影用)
    std::map<std::string, RigidTransform> child_to_imu_;   // T_imu_child
    ros::Time mount_last_try_;     // 上次尝试查询安装变换的时刻 (2s 节流)
    std::string mount_lidar_frame_ = "lidar";
    std::string mount_imu_frame_ = "lidar";
    std::string mount_base_frame_ = "base_link";
};

} // namespace rosiwit_slam_ros1
