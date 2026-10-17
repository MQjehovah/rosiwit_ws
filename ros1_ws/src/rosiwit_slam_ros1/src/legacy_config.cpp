// ============================================================================
// 旧 xz_3d_slam 配置装载的实现 —— 逐条照抄旧行为, 不允许"顺手改正"
// 对齐: legacy.cpp:32-43 / 1373-1407 / 1409-1708, xz_slam_ros1_node.cpp:32-44
// ============================================================================
#include "legacy_config.h"

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

namespace rosiwit_slam_ros1 {
namespace {

bool fileReadable(const std::string& path) {
    if (path.empty()) return false;
    std::ifstream ifs(path);
    return ifs.good();
}

/// nlohmann::json::merge_patch 的等价物 (legacy.cpp:1429 / 1449):
/// 两边都是映射节点 -> 按 key 递归合并; 否则 src 整体覆盖。
/// 返回合并后的节点, 调用方必须用返回值回写 (yaml-cpp 的 Node 是句柄, 未挂载节点
/// 在函数内赋值不会传播出去)。
/// 注: 不模拟 merge_patch 的 "值为 null 时删除该 key" 语义 —— 配置文件不会出现 null。
YAML::Node deepMerge(YAML::Node dst, const YAML::Node& src) {
    if (!src.IsMap()) return YAML::Clone(src);
    if (!dst.IsMap()) dst = YAML::Node(YAML::NodeType::Map);
    for (auto it = src.begin(); it != src.end(); ++it) {
        const std::string key = it->first.as<std::string>();
        const YAML::Node cur = dst[key];
        if (it->second.IsMap() && cur.IsMap())
            dst[key] = deepMerge(cur, it->second);
        else
            dst[key] = YAML::Clone(it->second);
    }
    return dst;
}

/// 读一个键; 缺失就回落 fallback (不告警, 与旧实现的逐键 else 分支等价)
template <typename T>
T getOr(const YAML::Node& conf, const std::string& key, const T& fallback) {
    if (!conf || !conf.IsMap() || !conf[key]) return fallback;
    try {
        return conf[key].as<T>();
    } catch (const std::exception& e) {
        ROS_WARN_STREAM("[legacy_config] key '" << key << "' has unexpected type (" << e.what()
                                               << "), falling back to default");
        return fallback;
    }
}

/// legacy.cpp:1373-1407 —— sensor_config.json + 机型 决定 laser_type 与两个话题名
std::string loadSensorConfig(const std::string& path, const std::string& model,
                             std::string& laser_sub, std::string& imu_sub) {
    std::string config_type = "Livox_Mid360";  // type_3d 缺失时的旧默认
    if (fileReadable(path)) {
        try {
            const YAML::Node conf = YAML::LoadFile(path);
            if (conf && conf.IsMap() && conf["type_3d"]) config_type = conf["type_3d"].as<std::string>();
            if (config_type == "Livox") config_type = "Livox_Mid360";
        } catch (const std::exception& e) {
            ROS_WARN_STREAM("[legacy_config] failed to parse sensor_config (" << e.what()
                            << "), falling back to type_3d=Livox_Mid360: " << path);
        }
    } else {
        ROS_WARN_STREAM(
            "[legacy_config] sensor_config not readable, falling back to type_3d=Livox_Mid360: "
            << path);
    }
    std::cout << "config_type=" << config_type << std::endl;

    std::string laser_type;  // 旧实现初值空串 (legacy.h:368) —— 都不命中时保持空
    if (config_type == "WLR_722" && isSkywalkerGT(model)) laser_type = "new_vanjee";
    if (config_type == "WLR_722" && isSkywalker50(model)) laser_type = "vanjee";
    if ((config_type == "Livox_Mid360" || config_type == "Livox_Mid360s") && isSkywalkerGT(model))
        laser_type = "new_mid360";
    if ((config_type == "Livox_Mid360" || config_type == "Livox_Mid360s") && isSkywalker50(model))
        laser_type = "mid360";
    if (config_type == "Robosense" && isSkywalkerGT(model)) {
        laser_type = "new_robosense";
        // legacy.cpp:1402-1403: Robosense 在 sensor_config 阶段就强制这两个话题名
        laser_sub = "rslidar_points";
        imu_sub = "rslidar_imu";
    }
    return laser_type;
}

/// legacy.cpp:1419-1452 —— 主配置 (优先 proj, 兜底 pkg) + /opt 深合并覆盖 + laser_params.<base> 子树合并
YAML::Node loadMainConf(const LegacyConfPaths& paths, const std::string& laser_type) {
    YAML::Node conf;
    bool loaded = false;

    if (fileReadable(paths.proj_conf_json)) {
        try {
            conf = YAML::LoadFile(paths.proj_conf_json);
            loaded = true;
            std::cout << "[legacy_config] 主配置: " << paths.proj_conf_json << std::endl;
        } catch (const std::exception& e) {
            ROS_WARN_STREAM("[legacy_config] failed to parse main conf (" << e.what()
                            << "): " << paths.proj_conf_json);
        }
    }
    if (!loaded && fileReadable(paths.pkg_conf_json)) {
        try {
            conf = YAML::LoadFile(paths.pkg_conf_json);
            loaded = true;
            std::cout << "[legacy_config] 主配置(兜底): " << paths.pkg_conf_json << std::endl;
        } catch (const std::exception& e) {
            ROS_WARN_STREAM("[legacy_config] failed to parse fallback conf (" << e.what()
                             << "): " << paths.pkg_conf_json);
        }
    }
    if (!loaded) {
        ROS_WARN_STREAM("[legacy_config] main conf missing, using built-in defaults (proj="
                        << paths.proj_conf_json << " pkg=" << paths.pkg_conf_json << ")");
    }

    // /opt 覆盖: 深合并 (嵌套子树按 key 合并, 不是整块替换)
    if (fileReadable(paths.opt_slam_json)) {
        try {
            conf = deepMerge(conf, YAML::LoadFile(paths.opt_slam_json));
            std::cout << "[legacy_config] 已深合并覆盖配置: " << paths.opt_slam_json << std::endl;
        } catch (const std::exception& e) {
            ROS_WARN_STREAM("[legacy_config] failed to parse override conf (" << e.what()
                            << "): " << paths.opt_slam_json);
        }
    }

    // laser_params.<base> 子树合并到根 (仅当 laser_type 非空)
    if (conf.IsMap() && !laser_type.empty() && conf["laser_params"]) {
        std::string base_type = laser_type;
        if (base_type == "new_mid360") base_type = "mid360";
        else if (base_type == "new_vanjee") base_type = "vanjee";
        else if (base_type == "new_robosense") base_type = "robosense";

        const YAML::Node lp = conf["laser_params"];
        if (lp.IsMap() && lp[base_type]) {
            conf = deepMerge(conf, lp[base_type]);
            std::cout << "[legacy_config] laser_params 已合并: " << base_type << std::endl;
        }
    }
    return conf;
}

}  // namespace

bool isSkywalker50(const std::string& model) { return model == "XZ-SW50"; }

bool isSkywalkerGT(const std::string& model) { return model == "XZ-SWGT"; }

std::string legacyMapBase(const std::string& map_dir) {
    std::string p = map_dir;
    // 1) 去掉结尾的 '/'
    while (p.size() > 1 && p.back() == '/') p.pop_back();

    // 2) **实机实测: UI 传进来的 map_dir 是文件路径**(如 ".../map.yaml"), 不是目录!
    //    旧实现是 `request.map_dir.substr(0, 32) + "map"` (legacy.cpp:2298) ——
    //    "/opt/xzrobot/maps/<13 位数字 id>/" 正好 32 字符, 所以那个 truncate 恰好把
    //    "/map.yaml" 切掉、留下目录再拼 "map"。这里**泛化**成"识别并剥掉地图文件后缀",
    //    对任意目录名长度都对 (也因此不必像旧实现那样截断路径)。
    static const char* kSuffixes[] = {"/map.yaml", "/map.pgm", "/map.xzmap",
                                      "/map.pcd",  "/map.jpg", "/map"};
    for (const char* s : kSuffixes) {
        const size_t n = std::string(s).size();
        if (p.size() >= n && p.compare(p.size() - n, n, s) == 0) {
            p.erase(p.size() - n);
            while (p.size() > 1 && p.back() == '/') p.pop_back();
            break;
        }
    }

    if (p.empty() || p == "/") return "/map";  // 根目录 "/" (或空串) -> /map, 避免拼成 "//map"
    return p + "/map";
}

LegacyConfig LegacyConfig::load(const LegacyConfPaths& paths, const std::string& machine_model) {
    LegacyConfig c;

    // --- 1. sensor_config.json + 机型 (legacy.cpp:35-37) ---
    c.laser_type = loadSensorConfig(paths.opt_sensor_json, machine_model, c.laser_sub_name,
                                    c.imu_sub_name);

    // --- 2~4. 主配置 -> /opt 覆盖 -> laser_params 子树 (legacy.cpp:1419-1452) ---
    const YAML::Node conf = loadMainConf(paths, c.laser_type);

    // --- 5. 逐键读取 (legacy.cpp:1454-1701) ---
    c.slam_init_type = getOr<std::string>(conf, "slam_init_type", "localization");
    c.is_debug = getOr<bool>(conf, "is_debug", false);
    c.debug_publish = conf.IsMap() && conf["debug_publish"] ? getOr<bool>(conf, "debug_publish", false)
                                                           : c.is_debug;  // 未配置时回落 is_debug

    c.roll_new = getOr<double>(conf, "roll_new", 0.0);
    c.pitch_new = getOr<double>(conf, "pitch_new", 0.0);
    c.yaw_new = getOr<double>(conf, "yaw_new", 90.0);
    c.x_new = getOr<double>(conf, "x_new", 0.0);
    c.z_new = getOr<double>(conf, "z_new", 0.0);
    c.imu_in_car_frame_yaw_livox = getOr<double>(conf, "imu_in_car_frame_yaw_livox", 65.0);
    c.imu_in_car_frame_yaw_wj = getOr<double>(conf, "imu_in_car_frame_yaw_wj", 60.0);

    // legacy.cpp:1535-1547 —— conf 里的值优先。
    // 与旧实现的一个**有意差别**（D21）: 键缺失时旧实现回落硬编码默认值
    // ("rslidar_points"/"zyz_imu_data"), 会盖掉 sensor_config 阶段为 Robosense
    // 强制写入的话题名; 这里改为保留上一步的结果 (缺失时才用硬编码默认值兜底)。
    // 随包发行的 conf/slam_conf.json 这两个键都在, 因此实际部署行为与旧实现一致。
    c.laser_sub_name = getOr<std::string>(conf, "laser_sub_name", c.laser_sub_name);
    c.imu_sub_name = getOr<std::string>(conf, "imu_sub_name", c.imu_sub_name);

    c.publish_hz = getOr<int>(conf, "publish_hz", 20);
    c.rt_grid_map_publish_hz = getOr<int>(conf, "rt_grid_map_publish_hz", 1);

    // yaw_calib: 仅在存在该键时才 *M_PI/180 (legacy.cpp:1563-1569)
    if (conf.IsMap() && conf["yaw_calib"]) {
        c.yaw_calib = getOr<double>(conf, "yaw_calib", 0.0) * M_PI / 180.0;
    } else {
        c.yaw_calib = 0;
    }
    c.x_calib = getOr<double>(conf, "x_calib", 0.0);
    c.y_calib = getOr<double>(conf, "y_calib", 0.0);

    c.use_tf = getOr<bool>(conf, "use_tf", true);
    // 旧实现从不读此键 (只在 conf 里存在); 读出仅供启动日志溯源
    c.use_filter_odom = getOr<bool>(conf, "use_filter_odom", false);
    c.use_2d_undistort = getOr<bool>(conf, "use_2d_undistort", true);
    c.th_2d_range_min = getOr<double>(conf, "th_2d_range_min", 0.5);
    c.th_2d_range_max = getOr<double>(conf, "th_2d_range_max", 3.0);
    c.test_wait_time = getOr<int>(conf, "test_wait_time", 30);

    c.use_delete_pts = getOr<bool>(conf, "use_delete_pts", true);
    c.delete_pt_in_car_max_x = getOr<double>(conf, "delete_pt_in_car_max_x", 0.5);
    c.delete_pt_in_car_min_x = getOr<double>(conf, "delete_pt_in_car_min_x", -1.2);
    c.delete_pt_in_car_max_y = getOr<double>(conf, "delete_pt_in_car_max_y", 0.5);
    c.delete_pt_in_car_min_y = getOr<double>(conf, "delete_pt_in_car_min_y", -0.5);
    c.delete_pt_in_car_max_z = getOr<double>(conf, "delete_pt_in_car_max_z", 0.8);
    c.delete_pt_in_car_min_z = getOr<double>(conf, "delete_pt_in_car_min_z", -0.5);

    c.use_3d_z_min = getOr<double>(conf, "use_3d_z_min", 0.0);
    c.use_3d_z_max = getOr<double>(conf, "use_3d_z_max", 20.0);
    c.mapping_z_min = getOr<double>(conf, "mapping_z_min", c.use_3d_z_min);
    c.mapping_z_max = getOr<double>(conf, "mapping_z_max", c.use_3d_z_max);
    c.loc_z_min = getOr<double>(conf, "loc_z_min", c.use_3d_z_min);
    c.loc_z_max = getOr<double>(conf, "loc_z_max", c.use_3d_z_max);

    c.lidar_pts_monitor = getOr<bool>(conf, "lidar_pts_monitor", true);
    c.lidar_pts_monitor_interval = getOr<int>(conf, "lidar_pts_monitor_interval", 10);
    if (c.lidar_pts_monitor_interval < 1) c.lidar_pts_monitor_interval = 10;

    // --- 6. node main 的日志配置 (xz_slam_ros1_node.cpp:32-44) ---
    c.xz_log_path = getOr<std::string>(conf, "xz_log_path", "/home/xzrobot/slam_logs/xz_3d_slam.log");
    c.print_xz_log_to_file = getOr<bool>(conf, "print_xz_log_to_file", false);

    // --- 7. 装载之后的与运算 (legacy.cpp:43) ---
    c.use_delete_pts = c.use_delete_pts && isSkywalker50(machine_model);

    std::cout << "[legacy_config] machine_model='" << machine_model << "' laser_type='" << c.laser_type
              << "' laser_sub='" << c.laser_sub_name << "' imu_sub='" << c.imu_sub_name
              << "' slam_init_type='" << c.slam_init_type << "'" << std::endl;
    std::cout << "[legacy_config] effective params: loc_z_min = " << c.loc_z_min
              << ", loc_z_max = " << c.loc_z_max << ", mapping_z_min = " << c.mapping_z_min
              << ", mapping_z_max = " << c.mapping_z_max << ", yaw_calib(rad) = " << c.yaw_calib
              << ", use_delete_pts = " << (c.use_delete_pts ? 1 : 0)
              << ", use_tf = " << (c.use_tf ? 1 : 0)
              << ", rt_grid_map_publish_hz = " << c.rt_grid_map_publish_hz << std::endl;
    return c;
}

}  // namespace rosiwit_slam_ros1
