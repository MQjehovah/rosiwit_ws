// ============================================================================
// 旧 xz_3d_slam 接口的**发布侧** (计划 T5 / 设计文档 §5.2/§5.3)
//
// 只做"旧接口对外长什么样", 算法全部由 SlamEngine 里的核心提供。
// 14 个话题的注册与发布条件逐条照抄 legacy.cpp:273-300 (initPub) 与
// 333-1110 (startPub / publishCurrScan / publishMappingPath / publishPointMap /
// publishRealtimeGridMap) 与 1112-1211 (publishCurrPose) 与 1823-1897
// (publishLocationInfo)。行号以 E:\workspace_slam\3d_slam 那份为准。
//
// 发布侧 / 订阅侧(T6) / 告警码(T7) / 服务(T8) / TF(T9) / 丢失上报(T11) 均已实现;
// 全部算法由 SlamEngine 里的核心提供, 本类只负责复刻旧接口的对外形状与状态粘合。
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

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "slam_core/slam_types.h"
#include "xzrobot_ros_dependency/CommonBoolService.h"
#include "xzrobot_ros_dependency/CommonIntRequestService.h"
#include "xzrobot_ros_dependency/Relocation.h"
#include "xzrobot_ros_dependency/SlamInfo.h"
#include "xzrobot_ros_dependency/SlamModeSwitchService.h"
#include "xzrobot_ros_dependency/StopMapOperation.h"
#include "xzrobot_ros_dependency/SwitchMap.h"
#include "xzrobot_ros_dependency/TagDetect.h"
#include "xzrobot_ros_dependency/UpdateMap.h"

#include <std_srvs/Empty.h>

#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

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
    /// 每收到一帧**原始输入**(未配准、在传感器自身坐标系)点云就调一次:
    /// 它是 `origin_point`(每帧雷达) 与定位模式下 `or_curr_scan_point` 的数据源。
    /// 这两个话题都以 `frame_id="map"` 发布 —— 照抄旧实现 (xz_slam_ros1_interface.cpp:1053/3115),
    /// 即 frame_id 只是兼容标记, 点本身并未转到 map 系, 消费方需知悉。
    void onRawInputCloud(const rosiwit_slam::CloudType& raw_cloud, const ros::Time& stamp);
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

    // ---- [T11] 告警码注入 ----
    /// 把码加进待报列表 (去重后由告警定时器统一发)。T11 用它上报 20003504(pose lost)。
    /// 核心**不暴露**任何告警码通道 (已确认 include/ 下没有 WarningCode 相关接口),
    /// 旧系统那个 20003504 是定位模块自己 push 出来的 (nano_gicp_localization.cpp:9403),
    /// 所以在我们这里只能由 LostMonitor 补报 —— 是"补上", 不是重复。
    void addWarningCode(int code);

    // ---- [T8] 告警抑制窗口 ----
    /// 复刻旧的 `warning_code_start_task_`: 初值 false, **只在 start_task 之后**置 true,
    /// 之后 100 个 tick 不发告警 (legacy.cpp:184 与 459-468)。启动时**不**抑制。
    void setWarningSuppression(bool on);

    // ---- [T7] 3 个 service client (外部服务不在时只 WARN, 不崩) ----
    /// get_charging_point; 失败时 charging_pile_json 为空串
    bool callTagDetect(std::string& charging_pile_json);
    /// set_detect_mode; 0.5s 超时 + 100ms 重试
    bool callSetDetectMode(int mode);
    /// switch_map_pro; 1.0s 超时
    bool callSwitchMapPro(xzrobot_ros_dependency::SwitchMap::Request& req);
    /// 旧的 "重定位进行中" 标志 (legacy.cpp:1843-1853 由 isRelocalization() 驱动)
    bool relocating() const { return relocating_; }

private:
    void initPub();        // legacy.cpp:273-300
    void initTimerPub();   // legacy.cpp:247-271
    void initSub();        // legacy.cpp:193-215 (只补引擎没覆盖的那几个)
    void initClients();    // legacy.cpp:243-244 + /switch_map_pro
    void initServer();     // legacy.cpp:229-245 (8 个 service server)

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

    SlamMode updateModeFromCore();   // 核心 pipeline mode -> 旧 SLAMMODE
    void applyModeToCore(SlamMode m);

    // ---- [T6] 订阅回调 ----
    void laserScanCB(const sensor_msgs::LaserScan::ConstPtr& msg);   // 只做频率统计
    void chassisOdomCB(const nav_msgs::Odometry::ConstPtr& msg);     // -> 核心 onOdom
    void liftStateCB(const std_msgs::String::ConstPtr& msg);         // 存字段(核心无此接口)
    void liftTypeCB(const std_msgs::String::ConstPtr& msg);          // 存字段 + 打日志

    // ---- [T7] 告警 ----
    void loadWarningCodeTable();   // 从**本包** share 读 conf/warning_code.json (M6)
    void timerPublishWarningCode(const ros::TimerEvent&);   // legacy.cpp:457-553
    void tickFreqWindow(bool is_lidar);   // 1s 窗口频率判定 (legacy.cpp:629-637/740-748)

    // ---- [T8] 8 个 service server ----
    bool handleModeSwitch(xzrobot_ros_dependency::SlamModeSwitchService::Request& req,
                          xzrobot_ros_dependency::SlamModeSwitchService::Response& res);
    bool handleSwitchMap(xzrobot_ros_dependency::SwitchMap::Request& req,
                         xzrobot_ros_dependency::SwitchMap::Response& res);
    bool handleRelocation(xzrobot_ros_dependency::Relocation::Request& req,
                          xzrobot_ros_dependency::Relocation::Response& res);
    bool handleStopMap(xzrobot_ros_dependency::StopMapOperation::Request& req,
                       xzrobot_ros_dependency::StopMapOperation::Response& res);
    bool handleGlobalLocalization(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool handleFinishTask(xzrobot_ros_dependency::CommonBoolService::Request& req,
                          xzrobot_ros_dependency::CommonBoolService::Response& res);
    bool handleStartTask(xzrobot_ros_dependency::CommonBoolService::Request& req,
                         xzrobot_ros_dependency::CommonBoolService::Response& res);
    bool handleExpandMap(xzrobot_ros_dependency::UpdateMap::Request& req,
                         xzrobot_ros_dependency::UpdateMap::Response& res);

    /// 把 initial_pose 转成核心的 PoseStamped 并 setInitPose。    /// `zero_if_insane`: /switch_map 才有的 "|x| 或 |y| > 1e4 则全零" 逻辑
    /// (legacy.cpp 里只有换图那条路有; /relocation 没有)。两条路都 **z 强制 0**。
    void applyInitPose(const geometry_msgs::PoseWithCovarianceStamped& in, bool zero_if_insane);

    /// 重建核心 + 同步清掉**本层**的缓存。
    /// 只重建核心是不够的: 我们的 RealtimeGrid 与 /map 的常驻对象/面积都还留着上一轮,
    /// 核心清空后 worker 取不到新栅格就会继续发旧的那张 -> 表现为"切回来还有残留"。
    /// ⚠ **默认关闭**: 实测重建会让建图停摆 (见 .cpp 里的注释), 用
    /// `_reset_on_mode_switch:=true` 打开以便复现/排查。
    void resetCoreAndCaches();

    // ---- [T9] 动态 map->odom TF ----
    void sendTransform();   // legacy.cpp:1899-1936

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
    bool reset_on_mode_switch_ = false;             // 默认关: 重建会让建图停摆 (见 .cpp)
    bool set_grid_map_ = false;                     // 旧 set_grid_map_ (legacy.cpp:448/940)
    double grid_resolution_ = 0.05;                 // 存 pgm/yaml 用的分辨率 (= config 的 map_resolution)
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

    // 原始输入点云缓存 (T6 填)。
    // 写入方 = 雷达回调线程 (engine 的 raw lidar hook); 读取方 = 0.1s/0.01s 定时器线程。
    // 当前 main 用单线程 ros::spin() 本就安全; 加锁是为了换 AsyncSpinner/多线程执行器时也安全。
    mutable std::mutex raw_input_mtx_;
    rosiwit_slam::CloudType::Ptr raw_input_;
    ros::Time raw_input_stamp_;

    // ---- [T6] 额外订阅 (引擎已覆盖 imu/lidar/initialpose, 这里补 legacy 剩下的 4 个) ----
    ros::Subscriber scan_sub_, chassis_odom_sub_, lift_state_sub_, lift_type_sub_;
    std::string lift_state_, lift_type_;   // 旧实现存字段; 核心没有对应接口 -> 不生效
    ros::Time scan_window_start_;
    int scan_count_ = 0;
    double scan_hz_ = 0.0;

    // ---- [T7] 告警 ----
    struct WarningInfo { std::string level, name, content; };
    std::map<int, WarningInfo> warning_code_table_;
    mutable std::mutex warning_code_mtx_;
    std::vector<int> pending_warning_codes_;
    bool warning_code_start_task_ = false;   // 初值 false (xz_slam_ros1_interface.h:552)
    int warning_code_suppress_ticks_ = 0;
    struct FreqWindow { ros::Time start; int count = 0; };
    FreqWindow imu_freq_, lidar_freq_;       // 旧实现 imu/lidar 各一个 1s 窗口
    ros::Timer warning_code_timer_;
    ros::ServiceClient tag_detect_client_, set_detect_mode_client_, switch_map_pro_client_;

    // ---- [T8] service server + 换图状态 ----
    ros::ServiceServer srv_mode_switch_, srv_switch_map_, srv_relocation_, srv_stop_map_,
                       srv_global_loc_, srv_finish_task_, srv_start_task_, srv_expand_map_;
    std::string curr_map_path_;    // 旧 curr_map_path_
    ros::Time last_switch_map_time_;   // /switch_map 的 10s 限流 (legacy.cpp:2121-2137)

    // ---- [T9] TF ----
    // 旧实现: listener_ = new tf2_ros::TransformListener(tfBuffer_) (legacy.cpp:94)。
    // M5: 新接口那边在 legacy profile 下已经不发任何 TF (RosiwitInterface),
    // 所以 map->odom 只有这里一个发布者, 不会打架。
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /// 雷达刚体 -> 车身(`base_link`) 的安装变换 (缓存在 T_lidar_base 里)。
    /// 核心估出来的是**雷达/IMU 刚体**的位姿; 这辆车雷达偏航 90°、俯装 41.67°、抬高 0.9m,
    /// 直接当车身位姿发出去就会**朝向差 90°**且带 42° 俯仰 (旧系统用 conf 里的
    /// roll/pitch/yaw/x/z 做这件事, M13 当初跳过了)。
    /// 这里改成从 TF (`base_link <- ~lidar_frame`) 读, 换车/换装自动正确。
    void computeLidarToBase();                    // 启动/首次用时取一次, 失败则单位阵
    Eigen::Matrix3d r_lb_ = Eigen::Matrix3d::Identity();   // base <- lidar
    Eigen::Vector3d t_lb_ = Eigen::Vector3d::Zero();
    bool have_lidar_mount_ = false;
};

} // namespace rosiwit_slam_ros1
