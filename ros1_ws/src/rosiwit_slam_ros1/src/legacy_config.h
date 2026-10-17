// ============================================================================
// 旧 xz_3d_slam 的配置装载 —— 与旧实现逐条对齐; 本头文件无 ROS 依赖, 实现只用
// yaml-cpp 解析 + ROS 日志宏 (ROS_WARN)。
//
// JSON 用 yaml-cpp 解析: JSON 是 YAML 的子集, 因此不必额外引 json 库。
//
// 对齐来源 (E:\workspace_slam\3d_slam\src\xz_3d_slam\xz_slam_ros1_interface.cpp, 下称 legacy.cpp):
//   构造函数 32-43        : 装载顺序 + use_delete_pts 与机型的与运算
//   loadConfigJson 1373-1407    : sensor_config.json -> laser_type / 话题名
//   loadConfFromJsonFile 1409-1708 : 主 slam_conf -> /opt 覆盖 -> laser_params 子树 -> 逐键默认值
//   xz_slam_ros1_node.cpp 23-44 : xz_log_path / print_xz_log_to_file (在 node main, 不在接口类)
//
// 原则: 任何文件缺失 / 解析失败只告警并回落到内置默认值, **绝不抛异常**
//       (旧实现在文件缺失时会直接抛, 这里按设计文档 D14 降级为告警)。
// ============================================================================
#pragma once

#include <string>

namespace rosiwit_slam_ros1 {

/// 旧实现的四个配置来源。都会逐个尝试, 缺失只告警。
struct LegacyConfPaths {
    std::string proj_conf_json;  // PROJECT_PATH/conf/slam_conf.json      (优先)
    std::string pkg_conf_json;   // 本包 conf/slam_conf.json              (兜底)
    std::string opt_sensor_json; // /opt/xzrobot/configs/sensor_config.json
    std::string opt_slam_json;   // /opt/xzrobot/configs/3d_slam/slam_conf.json (深合并覆盖)
};

/// 旧配置的装载结果。字段名去掉旧代码的下划线后缀, 语义一一对应。
struct LegacyConfig {
    // --- 来自 sensor_config.json + 机型 (legacy.cpp:1373-1407) ---
    // mid360 | new_mid360 | vanjee | new_vanjee | new_robosense | ""(都不命中)
    std::string laser_type = "";
    std::string laser_sub_name = "rslidar_points";
    std::string imu_sub_name = "zyz_imu_data";

    // --- 主配置 (legacy.cpp:1454-1701), 括号内是旧实现"键缺失"时的回落值 ---
    std::string slam_init_type = "localization";
    bool is_debug = false;
    bool debug_publish = false; // 键缺失时回落 is_debug (legacy.cpp:1470-1474)

    double roll_new = 0;
    double pitch_new = 0; // 注意: 不是文件里的 45, 是旧代码的默认
    double yaw_new = 90;
    double x_new = 0;
    double z_new = 0;
    double imu_in_car_frame_yaw_livox = 65;
    double imu_in_car_frame_yaw_wj = 60;

    int publish_hz = 20;            // 解析出来但发布周期硬编码 0.1s (legacy.cpp:254-255)
    int rt_grid_map_publish_hz = 1;
    double yaw_calib = 0; // 读入时已按 *M_PI/180 转弧度 (legacy.cpp:1563-1565); 缺键时不乘
    double x_calib = 0;
    double y_calib = 0;

    bool use_tf = true;
    bool use_filter_odom = false; // 旧实现**从不读此键**(只存在于 conf/slam_conf.json), 读出仅供启动日志
    bool use_2d_undistort = true;
    double th_2d_range_min = 0.5;
    double th_2d_range_max = 3.0;
    int test_wait_time = 30;

    bool use_delete_pts = true; // 装载后还要 && isSkywalker50(model) (legacy.cpp:43)
    double delete_pt_in_car_max_x = 0.5;
    double delete_pt_in_car_min_x = -1.2;
    double delete_pt_in_car_max_y = 0.5;
    double delete_pt_in_car_min_y = -0.5;
    double delete_pt_in_car_max_z = 0.8;
    double delete_pt_in_car_min_z = -0.5; // 注意: 旧代码默认 -0.5, 而文件里写的是 -10

    double use_3d_z_min = 0.0;
    double use_3d_z_max = 20.0;
    double mapping_z_min = 0.0; // 缺省取 use_3d_z_min
    double mapping_z_max = 20.0; // 缺省取 use_3d_z_max
    double loc_z_min = 0.0;      // 缺省取 use_3d_z_min
    double loc_z_max = 20.0;     // 缺省取 use_3d_z_max

    bool lidar_pts_monitor = true;
    int lidar_pts_monitor_interval = 10; // 读入后 <1 强制为 10

    // --- 来自 node main 的日志配置 (xz_slam_ros1_node.cpp:32-44) ---
    // 旧代码的回落值; 随包发行的 conf/slam_conf.json 会把它们覆盖成
    // /opt/xzrobot/logs/xzrobot_3d_slam/xz_3d_slam.log + true。
    // 本进程不用 xz_log, 读出仅供启动日志与溯源。
    std::string xz_log_path = "/home/xzrobot/slam_logs/xz_3d_slam.log";
    bool print_xz_log_to_file = false;

    /// 按旧顺序装载: sensor_config.json -> 主 slam_conf -> /opt 覆盖深合并 -> laser_params.<base> 子树合并
    /// 任何文件缺失/解析失败只告警, 用内置默认值。
    /// 返回后在内部完成: use_delete_pts &&= isSkywalker50(machine_model)
    /// 并把生效值打到启动日志。
    static LegacyConfig load(const LegacyConfPaths& paths, const std::string& machine_model);
};

/// legacy.cpp:12-20 原样
bool isSkywalker50(const std::string& model);
bool isSkywalkerGT(const std::string& model);

/// 旧地图路径契约 (设计文档 §5.10 A): base = (map_dir 去掉尾斜杠) + "/map"
/// 基名恒为 "map" (map.pcd / map.xzmap / map.pgm / map.yaml), map_name 只是显示标签。
std::string legacyMapBase(const std::string& map_dir);

} // namespace rosiwit_slam_ros1
