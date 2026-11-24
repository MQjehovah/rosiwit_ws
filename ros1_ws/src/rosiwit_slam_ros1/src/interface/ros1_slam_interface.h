// ============================================================================
// 旧 xz_3d_slam 接口的**发布侧** (计划 T5 / 设计文档 §5.2/§5.3)
//
// 只做"旧接口对外长什么样", 算法全部由 SlamAlgorithm 里的核心提供。
// 14 个话题的注册与发布条件逐条照抄 legacy.cpp:273-300 (initPub) 与
// 333-1110 (startPub / publishCurrScan / publishMappingPath / publishPointMap /
// publishRealtimeGridMap) 与 1112-1211 (publishCurrPose) 与 1823-1897
// (publishLocationInfo)。行号以 E:\workspace_slam\3d_slam 那份为准。
//
// 发布侧 / 订阅侧(T6) / 告警码(T7) / 服务(T8) / TF(T9) / 丢失上报(T11) 均已实现;
// 全部算法由 SlamAlgorithm 里的核心提供, 本类只负责复刻旧接口的对外形状与状态粘合。
// ============================================================================
#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>

#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <thread>
#include <string>
#include <vector>

#include "aux_jobs.h"
#include "control_loop.h"
#include "slam_core/slam_types.h"
#include "localization/planar_scan_matcher.hpp"
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

#include "lost_monitor.h"  // OdomConsistencyMonitor (LostMonitor 本身已不参与状态判定)
#include "realtime_grid.h"
#include "slam_algorithm.h"

namespace rosiwit_slam_ros1 {

/// 旧 `enum SLAMMODE { FREE, MAPPING, LOCALIZATION, UNKNOWN }`
/// (xz_slam_ros1_interface.h:92-97) —— **顺序照抄**, 旧实现有按整数比较的习惯。
enum class SlamMode { FREE, MAPPING, LOCALIZATION, UNKNOWN };

/// 旧实现的 xzSlamPose3D 里我们真正用到的那几项
struct Xz3dPose {
    double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

class Ros1SlamInterface {
public:
    Ros1SlamInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, SlamAlgorithm& engine);
    ~Ros1SlamInterface();

    // ---- [T6] 订阅侧往这里喂数据 ----
    /// [T6] 记录"收到过 IMU / 激光" —— 旧实现建图分支要求两者都为真才发布 (legacy.cpp:351)
    void setInputSeen(bool imu, bool laser);

    // ---- [T8] 服务侧改模式 ----
    /// 旧实现改模式就是给 slam_mode_ 赋值, 然后靠 localiza_ptr_/mapping_ptr_ 是否为空决定行为。
    /// 我们只有**一个核心**, 所以这里同时把模式同步给核心 (FREE -> "idle")。
    void setModeFromConfig(SlamMode m);
    SlamMode slamMode() const { return slam_mode_; }
    /// 服务专用回调队列 (main.cpp 里单独起一个 AsyncSpinner 消费)
    ros::CallbackQueue* serviceQueue() { return &srv_queue_; }

    // ---- 发布侧 latch 的跨线程复位 (单一写者) ----
    // 切图/切模式/重定位发生在服务线程, 但 last_* / odom_mon_ / 状态位是
    // slam_pub 线程 50Hz 在用的。服务线程只置位掩码, 真正的复位由发布线程
    // 在自己的循环里做 —— 这样这些字段永远只有一个写者, 不会撕裂读。
    static constexpr uint32_t kPubResetPoseLatch    = 1u << 0;  // curr_pose_valid_ + 位姿/高频戳
    static constexpr uint32_t kPubResetLocStatus    = 1u << 1;  // last_core_loc_status_ + core_ever_localized_
    static constexpr uint32_t kPubResetOdomMon      = 1u << 2;  // odom_mon_ + odom_mon_wrong_prev_
    static constexpr uint32_t kPubResetPath         = 1u << 3;  // mapping_path_msg_ + 轨迹节流
    static constexpr uint32_t kPubResetPlanar       = 1u << 4;  // planar_field_/pending/ever_locked
    static constexpr uint32_t kPubResetInfoThrottle = 1u << 5;  // last_location_info_pub_
    static constexpr uint32_t kPubResetDebugFlags   = 1u << 6;  // debug_map_published_

    void requestPublishReset(uint32_t mask) { pub_reset_mask_.fetch_or(mask); }
    /// 只在 slam_pub 线程调用 (publishCurrPose 开头)
    void applyPublishResetIfRequested();

    // ---- [T8/T11] 地图与定位 latch ----
    /// 换图/进入定位/重定位时调 (旧实现: relocating_ = true)
    void resetLocalizationLatch();
    /// 旧 `curr_map_name_` (slam/info 的 map_name), 初值空串 (legacy.cpp:93)
    void setMapName(const std::string& name) {
        std::lock_guard<std::mutex> lk(map_name_mu_);
        curr_map_name_ = name;
    }

    /// [T8] /switch_map 里读完 `<map_base>.pgm/.yaml` 后调
    /// (对应旧实现 LocalizationSetGridMap, legacy.cpp:373-455)。
    /// 值先攒着, 等下一次 publishPointMap 时一次性发出并清标志 (legacy.cpp:926-946)。

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

public:
    /// 接口层析构前主动停掉后台线程: 实时栅格的 worker 会回调进本对象,
    /// 必须比发布者/成员更早停, 否则就是 use-after-free。
    void stopBackgroundWork() { realtime_grid_.stop(); }

private:
    void initPub();        // legacy.cpp:273-300
    void initTimerPub();   // legacy.cpp:247-271
    void initSub();        // legacy.cpp:193-215 (只补引擎没覆盖的那几个)
    void initClients();    // legacy.cpp:243-244 + /switch_map_pro
    void initServer();     // legacy.cpp:229-245 (8 个 service server)

    // 定时器回调
    void timerPublishLocalCloudAndSubmap(const ros::TimerEvent&);  // 0.1s -> startPub()
    void timerPublishRealtimeGridMap(const ros::TimerEvent&);      // 1/rt_hz, 仅 MAPPING
    /// 20ms 心跳: 位姿 / TF / slam/info / 告警码 (都是小消息, 跑在 io 线程)
    void timerHeartbeat(const ros::TimerEvent&);

    // ---- 大消息: 构造 + 发布都在线程池 (见 publish_jobs.cpp) ----
    void startPub();                 // 0.1s 定时器 -> 投递, 不再直接转换
    void scheduleScanJob();
    void scheduleGridJob();
    void scheduleGlobalMapJob();
    void schedulePlanarJob();
    // 池任务体: 构造并就地 publish
    void buildScanMessages();
    void buildGridMessage();
    void buildGlobalMapMessage();
    /// 析构前等在途池任务收尾 (任务体捕获 this)
    void waitForAuxJobs();
    void publishCurrPose();          // 位姿发布总入口 (只做模式分派/顺序)
    /// 定位分支的发布结果：需要丢弃整个 tick / 本 tick 不发 slam/info / 正常发
    enum class PoseTickResult { DropTick, NoInfo, Info };
    PoseTickResult publishLocalizationPose(const rosiwit_slam::SlamOutput& out, bool have_out);
    void publishMappingPose(const rosiwit_slam::SlamOutput& out, bool have_out);
    void publishSlamInfoThrottled();
    void publishCoreStatusIfChanged();
    void publishHighRateLocalizationPose(bool info_published);
    void publishLocationInfo();      // legacy.cpp:1823-1897
    /// LocalizationServer 对外契约: /localization/state (位姿+协方差+健康等级).
    /// C2b: 2D 激光约束. 只在 DEGRADED/LOST 且曾定位成功过时介入,
    /// 连续两次匹配一致才纠偏 (防止单帧误匹配把位姿拉飞).
    void planarConstraintStep();
    /// 切图/未定位阶段的零点位姿 slam/info (1 = INIT, 0,0,0)
    void publishZeroPoseInfo();

    SlamMode updateModeFromCore();   // 核心 pipeline mode -> 旧 SLAMMODE
    void applyModeToCore(SlamMode m);

    // ---- [T6] 订阅回调 ----
    void laserScanCB(const sensor_msgs::LaserScan::ConstPtr& msg);   // 只做频率统计
    void chassisOdomCB(const nav_msgs::Odometry::ConstPtr& msg);     // -> 核心 onOdom
    void liftStateCB(const std_msgs::String::ConstPtr& msg);         // 存字段(核心无此接口)
    void liftTypeCB(const std_msgs::String::ConstPtr& msg);          // 存字段 + 打日志

    // ---- [T7] 告警 ----
    void loadWarningCodeTable();   // 从**本包** share 读 config/warning_code.json
    void timerPublishWarningCode(const ros::TimerEvent&);   // legacy.cpp:457-553
    void tickFreqWindow(bool is_lidar);   // 1s 窗口频率判定 (legacy.cpp:629-637/740-748)

    // ---- [T8] 8 个 service server ----
    bool handleModeSwitch(xzrobot_ros_dependency::SlamModeSwitchService::Request& req,
                          xzrobot_ros_dependency::SlamModeSwitchService::Response& res);
    struct MapSwitchContext {
        std::string base;
        bool detect_ok = false;
        bool pgm_ok = false;
    };
    /// 切图第一阶段：停旧核心/清旧输出/发预览/重置本层状态，服务可以立即返回。
    MapSwitchContext beginMapSwitch(const std::string& base,
                                    const xzrobot_ros_dependency::SwitchMap::Request& req);
    /// 通知 robot_move_base 载入全局地图 (/switch_map_pro)。
    /// nav 的全局地图**不是**订阅 /map, 而是这个服务; map_dir 必须是 `<base>.yaml`
    /// (传目录或 pgm 都会 "读取文件路径异常", 实机验证)。
    void notifyNavSwitchMap(const std::string& base,
                            const xzrobot_ros_dependency::SwitchMap::Request& in);
    /// 切图第二阶段：在**控制线程**上同步装载 (与存图/切模式串行)
    bool runMapLoadInline(const MapSwitchContext& ctx,
                          const geometry_msgs::PoseWithCovarianceStamped& init_pose);
    bool handleSwitchMap(xzrobot_ros_dependency::SwitchMap::Request& req,
                         xzrobot_ros_dependency::SwitchMap::Response& res);
    bool handleRelocation(xzrobot_ros_dependency::Relocation::Request& req,
                          xzrobot_ros_dependency::Relocation::Response& res);
    /// 内部入口: count_as_jump=false 供 2D 平面约束纠偏使用
    /// (位姿连续, 不算重定位; 否则 nav 会把常规纠偏当跳变, 反复清计划/停车)
    bool applyRelocation(xzrobot_ros_dependency::Relocation::Request& req,
                         xzrobot_ros_dependency::Relocation::Response& res,
                         bool count_as_jump);
    bool handleStopMap(xzrobot_ros_dependency::StopMapOperation::Request& req,
                       xzrobot_ros_dependency::StopMapOperation::Response& res);
    /// 存图主体 (跑在 disk_write lane 上)
    bool doStopMap(xzrobot_ros_dependency::StopMapOperation::Request& req,
                   xzrobot_ros_dependency::StopMapOperation::Response& res);
    bool handleGlobalLocalization(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    // 手动落盘飞行记录仪 (诊断, 2026-10-18 日志重构); LOST 翻转会自动落盘
    bool handleDumpRecorder(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool handleFinishTask(xzrobot_ros_dependency::CommonBoolService::Request& req,
                          xzrobot_ros_dependency::CommonBoolService::Response& res);
    bool handleStartTask(xzrobot_ros_dependency::CommonBoolService::Request& req,
                         xzrobot_ros_dependency::CommonBoolService::Response& res);
    bool handleExpandMap(xzrobot_ros_dependency::UpdateMap::Request& req,
                         xzrobot_ros_dependency::UpdateMap::Response& res);

    /// 把 initial_pose 转成核心的 PoseStamped 并 setInitPose。    /// `zero_if_insane`: /switch_map 才有的 "|x| 或 |y| > 1e4 则全零" 逻辑
    /// (legacy.cpp 里只有换图那条路有; /relocation 没有)。两条路都 **z 强制 0**。
    /// count_as_jump=false 供内部 2D 平面约束纠偏使用 (位姿连续, 不算跳变)
    void applyInitPose(const geometry_msgs::PoseWithCovarianceStamped& in,
                       bool zero_if_insane, bool count_as_jump = true);

    /// 重建核心 + 同步清掉**本层**的缓存。
    /// 只重建核心是不够的: 我们的 RealtimeGrid 与 /map 的常驻对象/面积都还留着上一轮,
    /// 核心清空后 worker 取不到新栅格就会继续发旧的那张 -> 表现为"切回来还有残留"。
    /// ⚠ **默认关闭**: 实测重建会让建图停摆 (见 .cpp 里的注释), 用
    /// `_reset_on_mode_switch:=true` 打开以便复现/排查。
    /// 返回 false 表示"核心没重建成功": 调用方必须把服务结果置 0, 不能对外报成功。
    bool resetCoreAndCaches();
    /// 模式切换的执行体 (在 disk_write lane 上跑, 不再占用 ROS 服务线程)
    bool applyModeSwitch(int mode);

    // ---- [T9] 动态 map->odom TF ----
    void sendTransform(const rosiwit_slam::PoseStamped* pred = nullptr);   // legacy.cpp:1899-1936, 更新并广播
    void broadcastMapOdom(); // 定位 LOST 时冻结最后有效 map->odom 并持续广播

    ros::NodeHandle nh_, pnh_;
    // 服务专用回调队列: 服务回调里有有界等待 (切图/切模式/存图), 不能和
    // 话题/定时器共用同一个 spinner 线程, 否则一次切图会让 IMU/雷达回调饿死。
    ros::CallbackQueue srv_queue_;
    ros::NodeHandle srv_nh_;
    SlamAlgorithm& engine_;
    const bool debug_publish_;       // 旧 debug_publish_ 开关
    const bool is_debug_;
    const bool use_tf_;

    // 位姿/TF 发布已挪到 slam_tick 线程 (§C3), 下面这些字段主线程与 tick 线程都会碰,
    // 因此: 标量用 atomic, 字符串/大消息用短锁。tick 独占的状态保持普通成员。
    std::atomic<SlamMode> slam_mode_{SlamMode::FREE};
    Xz3dPose curr_pose_;                    // tick 线程写, 其它线程用 currentPoseSnapshot()
    mutable std::mutex pose_mu_;
    Xz3dPose currentPoseSnapshot() const {
        std::lock_guard<std::mutex> lk(pose_mu_);
        return curr_pose_;
    }
    double last_pose_stamp_ = 0.0;         // 同帧去重 (legacy.cpp:1116/1121)

    // 输入钩子在 FrontEnd 线程写, 0.1s 发布定时器在 io 线程读 -> atomic
    std::atomic<bool> have_imu_{false}, have_laser_{false};   // legacy 的 get_imu_data_/get_laser_data_
    std::atomic<bool> debug_map_published_{false};            // 每进一次模式只发一次 global_map
    std::atomic<double> realtime_grid_map_area_{0.0};   // 主线程写(1Hz), tick 读
    std::atomic<bool> curr_pose_valid_{false};    // 用于定位高频重发
    ros::Time last_new_pose_ros_;
    ros::Time last_high_rate_pub_;
    ros::Time last_high_rate_tf_;
    geometry_msgs::TransformStamped last_map_odom_;  // 最后有效 map->odom
    bool have_map_odom_ = false;
    double next_debug_map_try_ = 0.0;   // global_map 失败重试节流 (墙钟秒)
    ros::Time last_map_odom_pub_;
    std::atomic<bool> relocating_{false};           // 旧 relocating_ (legacy.cpp:1843-1853)
    bool reset_on_mode_switch_ = true;              // 默认开: 与旧实现一致, 切建图即清空上一轮地图
    double grid_resolution_ = 0.05;                 // 存 pgm/yaml 用的分辨率 (= mapping.grid.resolution)
    double realtime_grid_resolution_ = 0.05;        // 实时 /map 分辨率 (= mapping.grid.realtime_resolution)

    // 静态/实时栅格: 生产方 (1Hz 栅格任务 / 切图) 构造一次不可变消息, 读者拿 shared_ptr。
    // **不做整体拷贝**: 一张 1600m 地图的 OccupancyGrid 是 640KB, 原来每读一次就拷一份
    // (兜底重发 1Hz + planar 最高 4Hz -> 最多 ~3MB/s 的无谓 memcpy + 分配)。
    using GridMsgPtr = std::shared_ptr<const nav_msgs::OccupancyGrid>;
    mutable std::mutex static_map_mu_;
    GridMsgPtr staticMapSnapshot() const {
        std::lock_guard<std::mutex> lk(static_map_mu_);
        return static_grid_msg_;
    }
    bool hasStaticMap() const {
        std::lock_guard<std::mutex> lk(static_map_mu_);
        return static_grid_msg_ && !static_grid_msg_->data.empty();
    }
    void setStaticMap(GridMsgPtr msg) {
        std::lock_guard<std::mutex> lk(static_map_mu_);
        static_grid_msg_ = std::move(msg);
    }
    void clearStaticMap() {
        std::lock_guard<std::mutex> lk(static_map_mu_);
        static_grid_msg_.reset();
    }

    // 地图名: 服务线程写 (switch_map), tick 线程读 (slam/info、planar)
    std::string curr_map_name_;   // 只能经 mapName()/setMapName() 访问
    mutable std::mutex map_name_mu_;
    std::string mapName() const {
        std::lock_guard<std::mutex> lk(map_name_mu_);
        return curr_map_name_;
    }
    void setMapNameLocked(const std::string& name) {
        std::lock_guard<std::mutex> lk(map_name_mu_);
        curr_map_name_ = name;
    }
    nav_msgs::Path mapping_path_msg_;               // 建图轨迹 (累积后整条重发)
    mutable std::mutex path_mu_;                    // mapping_path_msg_ 跨线程 (tick 写 / 发布读)
    double last_mapping_path_time_ = -1.0;          // 轨迹去重: 同一个核心位姿不重复入队
    ros::Time last_mapping_path_pub_;               // 轨迹整条重发节流 (10Hz -> 1Hz)
    ros::Time last_location_info_pub_;              // slam/info 节流 (最高 10Hz)
    static constexpr size_t kMaxMappingPathPoses = 10000;

    // 定位状态唯一来源是核心 localizationStatus()；这里只保存上一帧用于 LOST 边沿告警。
    rosiwit_slam::ILocalization::Status last_core_loc_status_ =
        rosiwit_slam::ILocalization::INIT;
    bool core_ever_localized_ = false;
    // confidently-wrong 防线: 激光位姿位移 vs 轮式里程计位移 的窗口一致性
    OdomConsistencyMonitor odom_mon_;
    bool odom_mon_wrong_prev_ = false;            // tick 独占
    // /odom 回调在主线程写, tick 读 -> 原子
    std::atomic<double> last_odom_x_{0.0}, last_odom_y_{0.0};
    std::atomic<double> last_odom_t_{-1.0};
    RealtimeGrid realtime_grid_;    // T2

    // ---- 发布器 (legacy.cpp:282-297 的名字/类型/queue/latch 逐字照抄) ----
    ros::Publisher point_map_puber_;          // global_map        PointCloud2  1  latch
    ros::Publisher grip_map_puber_;
    ros::Publisher overlay_map_puber_;        // debug/overlay_map  地图自更新叠加层 (latched, 1Hz)
    ros::Publisher overlay_confirmed_puber_;  // debug/overlay_map_confirmed 只含 w>=anchor 的叠加层
    ros::Publisher session_map_puber_;        // debug/session_map  会话地图       (latched, 1Hz)
    double overlay_anchor_ = 128.0;           // maintainer.persistency_anchor (w 阈值)
    ros::Time last_aux_map_pub_;           // global_grid_map   OccupancyGrid 1 latch (/map 兼容别名, debug 才发一帧)
    ros::Publisher realtime_grid_map_puber_;  // /map              OccupancyGrid 10
    ros::Publisher pointcloud_puber_;         // curr_scan_point   PointCloud2  10
    ros::Publisher ros1_pointcloud_puber_;    // scan_f_map        PointCloud   10
    ros::Publisher submap_puber_;             // curr_submap_point PointCloud2  10 (D2 不发)
    ros::Publisher pose_puber_;               // curr_localization_pose PoseStamped 10
    ros::Publisher path_puber_;               // mapping_path      Path         10
    ros::Publisher slam_info_puber_;
    double loc_sigma_xy_ = 0.05;               // 兜底位姿不确定度 (无 2D 匹配时)
    double loc_sigma_yaw_ = 0.03;
    float loc_degrade_matching_ = 0.65f;       // 低于此匹配度进入 DEGRADED

    // ---- C2b: 2D 激光平面约束 ----
    std::vector<rosiwit_slam::PlanarPoint> planar_pts_base_;  // base 系 2D 点
    ros::Time planar_pts_stamp_;
    // planar 状态: 激光回调 (io 线程) 写点集, slam_pub 线程跑匹配/纠偏 -> 一把锁
    mutable std::mutex planar_mu_;
    ros::Time planar_last_try_;
    rosiwit_slam::PlanarGrid planar_grid_;
    std::vector<float> planar_field_;
    std::vector<int8_t> planar_grid_src_;      // 用于判断地图是否变化
    // 最近一次有效 2D 匹配 (供 /localization/state 用真实协方差)
    ros::Time planar_cov_stamp_;
    double planar_cov_xx_ = 0.0, planar_cov_yy_ = 0.0, planar_cov_yaw_ = 0.0;
    // 待确认的纠偏候选 (连续两帧一致才执行)
    bool planar_pending_ = false;
    double planar_px_ = 0.0, planar_py_ = 0.0, planar_pyaw_ = 0.0;
    ros::Time planar_pending_stamp_;
    // "本张地图上曾经定位成功过": 与 core_ever_localized_ 不同, 它不会被
    // /relocation 清掉 —— 手动拉偏后正需要 2D 约束把位姿拉回来.
    bool planar_ever_locked_ = false;
    // C2b 默认关闭: 核心的 seed 门限/accept 机制目前不接受外部位姿跳变
    // (apply 后下一帧被推回原处, 见计划文档 C2b 记录). 打开会变成反复纠偏.
    bool planar_constraint_enabled_ = false;
    ros::Time last_planar_applied_;
    double last_planar_x_ = 0.0, last_planar_y_ = 0.0;
    int planar_apply_repeats_ = 0;
    ros::Publisher warning_code_puber_;       // slam_warning_code String       10 (T7)

    // "最新优先": 上一份还在算就丢掉本帧 (点云/栅格都是最新状态才有意义)
    std::atomic<bool> scan_job_busy_{false};
    std::atomic<bool> grid_job_busy_{false};
    std::atomic<bool> global_map_job_busy_{false};
    std::atomic<bool> planar_job_busy_{false};
    std::atomic<double> last_path_build_s_{0.0};   // 轨迹的 1Hz 构造节流
    AuxJobGuard aux_jobs_;                          // 析构时等在途任务
    ControlLoop control_;                           // 管理命令: 载图/切模式/存图

    ros::Timer point_cloud_mapping_timer_;    // 0.1s
    ros::Timer heartbeat_timer_;              // 20ms: 位姿/TF/slam_info

    ros::Timer realtime_grid_map_timer_;      // 1/rt_grid_map_publish_hz

    // 旧 /map 的那张"实时栅格"对象 (legacy.cpp 的 realtime_grid_map_):
    // 它是**常驻**的 —— 没有新数据时也照发上一张 (legacy.cpp:909 无条件 publish)
    GridMsgPtr static_grid_msg_;    // 不可变快照 (shared_ptr 交换, 零拷贝)

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
    // start_task (服务线程) 写, 1s 告警定时器 (io 线程) 读/清 -> atomic
    std::atomic<bool> warning_code_start_task_{false};   // 初值 false (xz_slam_ros1_interface.h:552)
    std::atomic<int> warning_code_suppress_ticks_{0};
    struct FreqWindow { ros::Time start; int count = 0; };
    FreqWindow imu_freq_, lidar_freq_;       // 旧实现 imu/lidar 各一个 1s 窗口
    ros::Timer warning_code_timer_;
    ros::ServiceClient tag_detect_client_, set_detect_mode_client_, switch_map_pro_client_;

    // ---- [T8] service server + 换图状态 ----
    ros::ServiceServer srv_mode_switch_, srv_switch_map_, srv_relocation_, srv_stop_map_,
                       srv_global_loc_, srv_finish_task_, srv_start_task_, srv_expand_map_,
                       srv_dump_recorder_;
    // 地图路径: 服务线程写 (switch_map), tick 线程读 (/slam/info) -> 必须加锁
    mutable std::mutex map_path_mu_;
    std::string curr_map_path_;   // 只能经 currentMapPath()/setCurrentMapPath() 访问
    std::string currentMapPath() const {
        std::lock_guard<std::mutex> lk(map_path_mu_);
        return curr_map_path_;
    }
    void setCurrentMapPath(const std::string& p) {
        std::lock_guard<std::mutex> lk(map_path_mu_);
        curr_map_path_ = p;
    }
    std::atomic<double> loaded_map_mtime_{0.0};   // 同名地图重存检测: 加载时 mtime
    // 保存前质量门限
    double map_quality_free_min_m2_ = 1.0;
    double map_quality_unknown_max_ = 0.60;    // 旧 curr_map_path_
    std::atomic<uint32_t> pub_reset_mask_{0};   // 见 kPubReset* 位定义 (服务线程置位)
    uint64_t pub_reset_seen_ = 0;               // 仅 slam_pub 线程访问
    // /switch_map 就绪信号 (后台加载任务写, 服务线程读)
    // /slam/mode_switch 异步执行状态 (服务线程只投递, lane 线程执行)
    // ROS 层兼容：大图 xzmap 解析/关键帧重建在 aarch64 上可达 25s，
    // 不能让 /switch_map 服务线程被拖到 App 超时。PGM 先同步发布，
    // 完整定位地图在后台串行加载。
    // 完整定位地图加载投到执行器的 "disk_read" lane (threads=1, 天然串行),
    // 不再自己 std::async —— 线程由组装根持有。
    // 在途计数用于析构等待: 任务体捕获了 this, 绝不能析构后还在跑。
    std::atomic<bool> map_switch_busy_{false};

    /// 切图任务收尾 (在途计数--, 归零时唤醒析构)。任务体与提交失败路径都要调。

    // ---- [T9] TF ----
    // 旧实现: listener_ = new tf2_ros::TransformListener(tfBuffer_) (legacy.cpp:94)。
    // M5: 新接口那边在 legacy profile 下已经不发任何 TF (RosiwitInterface),
    // 所以 map->odom 只有这里一个发布者, 不会打架。
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /// 核心估计 IMU body; 对外位姿在 ROS1 层由 base_link <- IMU 转出.
    /// 本函数只刷新 scan_ground_z_ (由 SlamAlgorithm::computeBaseFrames 提供).
    void computeLidarToBase();
    /// 地面在地图系里的 z。地图系原点=起始 IMU -> 地面 ≈ -IMU 安装高度。
    /// 发布 scan_f_map / curr_scan_point 时用它做 z 参考, 保证与栅格 0.1~2.0m(相对地面)同带。
    double scan_ground_z_ = -0.898;
};

} // namespace rosiwit_slam_ros1
