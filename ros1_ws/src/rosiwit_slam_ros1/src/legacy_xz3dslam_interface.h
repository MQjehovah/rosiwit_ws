// ============================================================================
// 旧 xz_3d_slam 接口的**发布侧** (计划 T5 / 设计文档 §5.2/§5.3)
//
// 只做"旧接口对外长什么样", 算法全部由 SlamEngine 里的核心提供。
// 14 个话题的注册与发布条件逐条照抄 legacy.cpp:273-300 (initPub) 与
// 333-1110 (startPub / publishCurrScan / publishMappingPath / publishPointMap /
// publishRealtimeGridMap) 与 1112-1211 (publishCurrPose) 与 1823-1897
// (publishLocationInfo)。行号以 E:\workspace_slam\3d_slam 那份为准。
//
// **本任务只做发布侧**; 订阅侧 T6、告警码 T7、服务 T8、TF T9、丢失上报接线 T11。
// 因此下面留了三处 seam (都标了 [T6]/[T8]/[T9]):
//   - 原始输入点云缓存: T6 的订阅回调往里塞 (or_curr_scan_point / origin_point 的数据源)
//   - 模式切换: T8 的服务调 setModeFromLegacy()
//   - TF: T9 实现 sendTransform()
// ============================================================================
#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>

#include "slam_core/slam_types.h"
#include "xzrobot_ros_dependency/SlamInfo.h"

#include "legacy_config.h"
#include "lost_monitor.h"
#include "realtime_grid.h"
#include "slam_engine.h"

namespace rosiwit_slam_ros1 {

/// 旧 `enum SLAMMODE { FREE, MAPPING, LOCALIZATION, UNKNOWN }`
/// (xz_slam_ros1_interface.h:92-97) —— **顺序照抄**, 旧实现有按整数比较的习惯。
enum class SlamMode { FREE, MAPPING, LOCALIZATION, UNKNOWN };

/// 旧实现的 xzSlamPose3D 里我们真正用到的那几项
struct LegacyPose {
    double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

class LegacyXz3dSlamInterface {
public:
    LegacyXz3dSlamInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, SlamEngine& engine);

    // ---- [T6] 订阅侧往这里喂数据 ----
    /// 每收到一帧**原始输入**点云就调一次(点云需已转到 map 系):
    /// 它是 `origin_point`(每帧雷达) 与定位模式下 `or_curr_scan_point` 的数据源。
    void onRawInputCloud(const rosiwit_slam::CloudType& world_cloud, const ros::Time& stamp);
    /// [T6] 记录"收到过 IMU / 激光" —— 旧实现建图分支要求两者都为真才发布 (legacy.cpp:351)
    void setInputSeen(bool imu, bool laser);

    // ---- [T8] 服务侧改模式 ----
    /// 旧实现改模式就是给 slam_mode_ 赋值, 然后靠 localiza_ptr_/mapping_ptr_ 是否为空决定行为。
    /// 我们只有**一个核心**, 所以这里同时把模式同步给核心 (FREE -> "idle")。
    void setModeFromLegacy(SlamMode m);
    SlamMode slamMode() const { return slam_mode_; }

    // ---- [T8/T11] 地图与定位 latch ----
    /// 地图加载成功时调 (旧实现: localiza_ptr_ 被创建出来)
    void setMapLoaded(bool loaded) { lost_monitor_.setMapLoaded(loaded); }
    /// 换图/进入定位/重定位时调 (旧实现: relocating_ = true)
    void resetLocalizationLatch();
    /// 旧 `curr_map_name_` (slam/info 的 map_name), 初值 "NULL" (legacy.cpp:93)
    void setMapName(const std::string& name) { curr_map_name_ = name; }

    /// [T8] /switch_map 里读完 `<map_base>.pgm/.yaml` 后调
    /// (对应旧实现 LocalizationSetGridMap, legacy.cpp:373-455)。
    /// 值先攒着, 等下一次 publishPointMap 时一次性发出并清标志 (legacy.cpp:926-946)。
    void setGlobalGridMap(const nav_msgs::OccupancyGrid& grid);

    /// 旧的 "重定位进行中" 标志 (legacy.cpp:1843-1853 由 isRelocalization() 驱动)
    bool relocating() const { return relocating_; }

private:
    void initPub();        // legacy.cpp:273-300
    void initTimerPub();   // legacy.cpp:247-271
    void initSub();        // legacy.cpp:193-215 (只补引擎没覆盖的那几个)

    // 定时器回调
    void timerPublishLocalCloudAndSubmap(const ros::TimerEvent&);  // 0.1s -> startPub()
    void timerPublishLocalizationPose(const ros::TimerEvent&);     // 0.01s -> publishCurrPose()
    void timerPublishRealtimeGridMap(const ros::TimerEvent&);      // 1/rt_hz, 仅 MAPPING

    void startPub();                 // legacy.cpp:333-371
    void publishCurrScan();          // legacy.cpp:966-1110
    void publishMappingPath();       // legacy.cpp:840-889
    void publishRealtimeGridMap();   // legacy.cpp:891-910
    void publishPointMap();          // legacy.cpp:912-964
    void publishCurrPose();          // legacy.cpp:1112-1211
    void publishLocationInfo();      // legacy.cpp:1823-1897
    // [T9] void sendTransform();    // legacy.cpp:1899-1936
    // [T7] void timerPublishWarningCode(const ros::TimerEvent&);  // legacy.cpp:457-553

    SlamMode updateModeFromCore();   // 核心 pipeline mode -> 旧 SLAMMODE
    void applyModeToCore(SlamMode m);

    // ---- [T6] 订阅回调 ----
    void laserScanCB(const sensor_msgs::LaserScan::ConstPtr& msg);   // 只做频率统计
    void chassisOdomCB(const nav_msgs::Odometry::ConstPtr& msg);     // -> 核心 onOdom
    void liftStateCB(const std_msgs::String::ConstPtr& msg);         // 存字段(核心无此接口)
    void liftTypeCB(const std_msgs::String::ConstPtr& msg);          // 存字段 + 打日志

    ros::NodeHandle nh_, pnh_;
    SlamEngine& engine_;
    const LegacyConfig& cfg_;        // engine_.legacyConfig()
    const bool debug_publish_;       // 旧 debug_publish_ 开关
    const bool is_debug_;
    const bool use_tf_;

    SlamMode slam_mode_ = SlamMode::FREE;
    std::string curr_map_name_ = "NULL";   // legacy.cpp:93
    LegacyPose curr_pose_;
    double last_pose_stamp_ = 0.0;         // 同帧去重 (legacy.cpp:1116/1121)

    bool have_imu_ = false, have_laser_ = false;    // legacy 的 get_imu_data_/get_laser_data_
    bool debug_map_published_ = false;              // 每进一次模式只发一次 global_map
    double realtime_grid_map_area_ = 0.0;
    bool relocating_ = false;                       // 旧 relocating_ (legacy.cpp:1843-1853)
    bool set_grid_map_ = false;                     // 旧 set_grid_map_ (legacy.cpp:448/940)
    nav_msgs::OccupancyGrid global_grid_map_msg_;   // 旧 global_grid_map_ (latch 一次)
    nav_msgs::Path mapping_path_msg_;               // 建图轨迹 (累积后整条重发)

    LostMonitor lost_monitor_;      // T3
    RealtimeGrid realtime_grid_;    // T2

    // ---- 发布器 (14 个, legacy.cpp:282-297 的名字/类型/queue/latch 逐字照抄) ----
    ros::Publisher point_map_puber_;          // global_map        PointCloud2  1  latch
    ros::Publisher grip_map_puber_;           // global_grid_map   OccupancyGrid 1 latch
    ros::Publisher realtime_grid_map_puber_;  // /map              OccupancyGrid 10
    ros::Publisher pointcloud_puber_;         // curr_scan_point   PointCloud2  10
    ros::Publisher ros1_pointcloud_puber_;    // scan_f_map        PointCloud   10
    ros::Publisher submap_puber_;             // curr_submap_point PointCloud2  10 (D2 不发)
    ros::Publisher or_pointcloud_puber_;      // or_curr_scan_point PointCloud2 10
    ros::Publisher pose_puber_;               // curr_localization_pose PoseStamped 10
    ros::Publisher path_puber_;               // mapping_path      Path         10
    ros::Publisher loop_constraint_puber_;    // loop_constraint   MarkerArray  10 (D2 不发)
    ros::Publisher slam_info_puber_;          // slam/info         SlamInfo     10
    ros::Publisher scan_puber_;               // points_2d         PointCloud2  10 (永不发)
    ros::Publisher warning_code_puber_;       // slam_warning_code String       10 (T7)
    ros::Publisher point_puber_;              // origin_point      PointCloud2  10

    ros::Timer point_cloud_mapping_timer_;    // 0.1s
    ros::Timer localization_pose_timer_;      // 0.01s
    ros::Timer realtime_grid_map_timer_;      // 1/rt_grid_map_publish_hz

    // 旧 /map 的那张"实时栅格"对象 (legacy.cpp 的 realtime_grid_map_):
    // 它是**常驻**的 —— 没有新数据时也照发上一张 (legacy.cpp:909 无条件 publish)
    nav_msgs::OccupancyGrid realtime_grid_map_;

    // 原始输入点云缓存 (T6 填)
    rosiwit_slam::CloudType::Ptr raw_input_;
    ros::Time raw_input_stamp_;

    // ---- [T6] 额外订阅 (引擎已覆盖 imu/lidar/initialpose, 这里补 legacy 剩下的 4 个) ----
    ros::Subscriber scan_sub_, chassis_odom_sub_, lift_state_sub_, lift_type_sub_;
    std::string lift_state_, lift_type_;   // 旧实现存字段; 核心没有对应接口 -> 不生效
    ros::Time scan_window_start_;
    int scan_count_ = 0;
    double scan_hz_ = 0.0;
};

} // namespace rosiwit_slam_ros1
