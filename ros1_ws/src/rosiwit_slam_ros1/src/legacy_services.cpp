// ============================================================================
// legacy 服务侧: 初始位姿 + 全部服务处理器 + 外部服务客户端 —— 从 legacy_xz3dslam_interface.cpp 按关注点拆出 (2026-10-18 薄层整合,
// 纯代码搬移, 行为与拆分前逐字一致; 分区方式对照 ros2_ws SlamNode)。
// ============================================================================
#include "legacy_xz3dslam_interface.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "legacy_convert.h"
#include "legacy_xz3dslam_internal.h"

using namespace rosiwit_slam;
using namespace rosiwit_slam_ros1::legacy_internal;

namespace rosiwit_slam_ros1 {
bool LegacyXz3dSlamInterface::callTagDetect(std::string& charging_pile_json) {
    charging_pile_json.clear();
    for (int i = 0; i < 10; ++i) {   // 1.0s 超时 (100ms x 10)
        xzrobot_ros_dependency::TagDetect srv;
        if (tag_detect_client_.call(srv)) {
            charging_pile_json = srv.response.charging_pile_json;
            return true;
        }
        ros::Duration(0.1).sleep();
    }
    ROS_WARN("[legacy_iface] get_charging_point unavailable");
    return false;   // 旧行为: 失败给空 charging_pile_json (D14)
}

bool LegacyXz3dSlamInterface::callSetDetectMode(int mode) {
    for (int i = 0; i < 5; ++i) {    // 0.5s 超时 + 100ms 重试
        xzrobot_ros_dependency::CommonIntRequestService srv;
        srv.request.mode = static_cast<int8_t>(mode);
        if (set_detect_mode_client_.call(srv)) return srv.response.result;
        ros::Duration(0.1).sleep();
    }
    ROS_WARN("[legacy_iface] set_detect_mode unavailable");
    return false;
}

bool LegacyXz3dSlamInterface::callSwitchMapPro(xzrobot_ros_dependency::SwitchMap::Request& req) {
    for (int i = 0; i < 10; ++i) {   // 1.0s 超时
        xzrobot_ros_dependency::SwitchMap srv;
        srv.request = req;
        if (switch_map_pro_client_.call(srv)) return srv.response.success;
        ros::Duration(0.1).sleep();
    }
    ROS_WARN("[legacy_iface] /switch_map_pro unavailable");
    return false;
}

// ---------------------------------------------------------------- [T8] 8 个 server
void LegacyXz3dSlamInterface::applyInitPose(
    const geometry_msgs::PoseWithCovarianceStamped& in, bool zero_if_insane) {
    // **不在这里重复实现**: /switch_map、/relocation 与 /initialpose 共用
    // SlamEngine::applyInitPose() —— insane 归零、z 归零、车身->雷达安装变换换算、setInitPose
    // 全部只有一份。这里只保留本路径的差异: z 强制归零。
    if (!engine_.algo()) return;
    engine_.applyInitPose(in, zero_if_insane, /*force_z_zero=*/true);
}

bool LegacyXz3dSlamInterface::handleModeSwitch(
    xzrobot_ros_dependency::SlamModeSwitchService::Request& req,
    xzrobot_ros_dependency::SlamModeSwitchService::Response& res) {
    if (req.mode == 0) {
        // 旧 switchToLocalization (legacy.cpp:2060-2115): 先置 FREE, 再调 /set_detect_mode(0),
        // **要求成功** —— 失败就 result=0 + return false。
        // 旧实现同时 DELETE_PTR 掉了建图/定位实例 -> 我们在这里也重建核心(丢上一轮地图)。
        resetCoreAndCaches();
        setModeFromLegacy(SlamMode::FREE);
        const bool ok = callSetDetectMode(0);
        res.result = ok ? 1 : 0;
        return ok;
    }
    if (req.mode == 1) {
        // 旧 switchToMapping (legacy.cpp:1974-2058): 也调 /set_detect_mode(1), 但
        // "if (!success) return false;" 那行在 2026 是**注释掉的** -> 结果不影响返回值;
        // 随后 run() -> warning_code_start_task_ = true (184), 即重新开启抑制窗口。
        // **关键**: 旧实现先 DELETE_PTR(mapping_ptr_) 再 new 一个全新实例, 所以退出建图再
        // 进建图**不会残留上一轮地图**; 我们单核心实例, 必须显式重建。
        resetCoreAndCaches();
        callSetDetectMode(1);
        setModeFromLegacy(SlamMode::MAPPING);
        setWarningSuppression(true);
        res.result = 1;
        return true;
    }
    return false;   // 非法 mode: result 保持默认 0
}

bool LegacyXz3dSlamInterface::handleSwitchMap(
    xzrobot_ros_dependency::SwitchMap::Request& req,
    xzrobot_ros_dependency::SwitchMap::Response& res) {
    // 10s 限流 —— 注意这在 **/switch_map** 里, 不在 /slam/mode_switch (legacy.cpp:2121-2137)
    const ros::Time now = ros::Time::now();
    if (!last_switch_map_time_.isZero() && (now - last_switch_map_time_).toSec() < 10.0) {
        addWarningCode(20003005);   // set mode operation frequent
        res.success = false;
        return false;
    }
    last_switch_map_time_ = now;

    // 分支 1: 参数不合法
    if (req.map_dir.empty() || req.map_name.empty()) {
        res.success = false;
        return true;
    }
    // 分支 2: 同一张图且已在定位 -> 只重设初始位姿 + 转发 /switch_map_pro
    if (req.map_dir == curr_map_path_ && slam_mode_ == SlamMode::LOCALIZATION) {
        applyInitPose(req.initial_pose, true);
        res.success = callSwitchMapPro(req);
        return true;
    }
    // 分支 3: 换图
    relocating_ = true;
    // **切图不清图** (用户要求, 也与旧实现的实际效果一致):
    // 旧实现在这里删的是"建图实例", 定位实例由 loadMap() 原地替换; 我们若在这里重建核心,
    // 会对 /switch_map 引入一次 STOP+join(后端忙时把服务线程卡死, 现场表现"切图卡住"),
    // 还要重新做一次 IMU 初始化(2s), 纯属浪费。地图由下面的 loadMap() 替换即可。
    // 清图只在"进建图"时做(handleModeSwitch -> 清上一轮残留)。
    std::cout << "[legacy_iface] /switch_map: keep core, just load the new map" << std::endl;
    const bool detect_ok = callSetDetectMode(0);
    res.success = detect_ok;   // 旧实现: **该结果单独决定** success

    const std::string base = legacyMapBase(req.map_dir);   // "/dir/map"
    if (engine_.algo() && !engine_.algo()->loadMap(base)) {
        // D13: 旧实现不检查加载结果; 我们检查并报错(同时把 success 拉低)
        res.success = false;
        ROS_ERROR_STREAM("[legacy_iface] /switch_map: loadMap failed: " << base);
    }
    if (engine_.algo()) engine_.algo()->setPipelineMode("localization");
    slam_mode_ = SlamMode::LOCALIZATION;
    curr_map_path_ = req.map_dir;
    curr_map_name_ = req.map_name;
    applyInitPose(req.initial_pose, true);
    lost_monitor_.reset();
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
    lost_monitor_.setMapLoaded(true);
    setWarningSuppression(true);   // 旧实现这里也调 run() (legacy.cpp:2345)
    return true;
}

bool LegacyXz3dSlamInterface::handleRelocation(
    xzrobot_ros_dependency::Relocation::Request& req,
    xzrobot_ros_dependency::Relocation::Response& res) {
    // 旧 relocate (legacy.cpp:2394-2418): relocating_ 置真又立刻置假;
    // setInitPose 的 **z 强制 0**; success 恒 true。
    // 注意这里**没有** |x|,|y| > 1e4 归零 —— 那只在 /switch_map 里。
    relocating_ = true;
    applyInitPose(req.initial_pose, false);
    relocating_ = false;
    // [T11] 进入重定位 -> 清 latch (下次真的定位上之前报 1, 不报 0)
    lost_monitor_.reset();
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
    res.success = true;
    return true;
}

bool LegacyXz3dSlamInterface::handleStopMap(
    xzrobot_ros_dependency::StopMapOperation::Request& req,
    xzrobot_ros_dependency::StopMapOperation::Response& res) {
    // 旧 stopMap (legacy.cpp:2419-2516)
    const std::string folder = req.map_dir;
    struct stat st;
    const bool exists = (!folder.empty() && ::stat(folder.c_str(), &st) == 0 && S_ISDIR(st.st_mode));
    if (!exists) {
        if (::mkdir(folder.c_str(), 0755) != 0) {
            // 旧实现: 建目录失败 -> return false 且 success 留 false
            ROS_ERROR_STREAM("[legacy_iface] /stop_map_operation: cannot create dir: " << folder);
            return false;
        }
    }
    // 旧实现要求 mapping_ptr_ 非空
    if (slam_mode_ != SlamMode::MAPPING || !engine_.algo()) return false;

    // 旧: mapping_ptr_->saveMap(folderPath, "map") -> 等价于 saveMap("<dir>/map")
    const std::string base = legacyMapBase(req.map_dir);
    if (!engine_.algo()->saveMap(base)) {
        // 旧实现的 saveMap 返回 void, **不检查**结果 -> 我们只报错, 不改 success
        ROS_ERROR_STREAM("[legacy_iface] /stop_map_operation: saveMap failed: " << base);
    }

    // 同时补 map.pgm / map.yaml: 旧实现的地图目录里有这两个文件, 且旧
    // LocalizationSetGridMap() (legacy.cpp:373-455) 会 imread("<base>.pgm") 并读
    // "<base>.yaml" 的 resolution/origin/occupied_thresh。核心的 saveGridMap 写的正是
    // 这个格式 (grid_map_io.cpp:7-39: P5, 首行=最大 y; yaml 含 image/resolution/
    // origin/negate/occupied_thresh/free_thresh/mode)。
    if (!engine_.algo()->saveGridMap(base + ".pgm", base + ".yaml", grid_resolution_)) {
        // 与 saveMap 同理: 失败只报错, 不改 success (旧实现也没有这一步可对照)
        ROS_ERROR_STREAM("[legacy_iface] /stop_map_operation: saveGridMap failed: " << base);
    } else {
        std::cout << "[legacy_iface] saved grid map: " << base << ".pgm / " << base
                  << ".yaml (resolution=" << grid_resolution_ << ")" << std::endl;
        // 旧实现还会从同一张灰度图导出 <base>.jpg 与 <dir>/thumb.png (App 的缩略图)
        writeMapThumbnails(base);
    }

    std::string charging_json;
    if (!callTagDetect(charging_json)) {
        std::cout << "XZSlamInterFace::stopMap(), call get_charging_point failed" << std::endl;
        charging_json.clear();   // 旧实现失败时取默认构造的空串
    }

    res.current_pose.header.frame_id = kMapFrame;
    res.current_pose.pose.pose.position.x = curr_pose_.x;
    res.current_pose.pose.pose.position.y = curr_pose_.y;
    res.current_pose.pose.pose.position.z = curr_pose_.z;   // **真实 z**, 不是 -0.07
    res.current_pose.pose.pose.orientation.x = curr_pose_.q.x();
    res.current_pose.pose.pose.orientation.y = curr_pose_.q.y();
    res.current_pose.pose.pose.orientation.z = curr_pose_.q.z();
    res.current_pose.pose.pose.orientation.w = curr_pose_.q.w();
    res.charging_pile_json = charging_json;
    // map_resolution / map_origin / map_negate / *_thresh / error 全部留默认空值:
    // 旧代码里那几行是注释掉的 (legacy.cpp:2506-2515), 照旧留空。
    res.success = true;
    return true;
}

bool LegacyXz3dSlamInterface::handleGlobalLocalization(std_srvs::Empty::Request&,
                                                       std_srvs::Empty::Response&) {
    // 旧实现 legacy.cpp:2634 就一行 `return true` (真正干活的是老定位模块自己)。
    // 我们把它接成**按需触发一次全局重定位**: 核心侧置 LOST 并清冷却, 下一帧进入冷启动分支
    // (localization.pipeline.relocalizer=bbs 时即全局检索; =none 时退化为有界局部重锁)。
    if (!engine_.algo()) return false;
    engine_.requestRelocalization();
    lost_monitor_.reset();
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
    relocating_ = true;
    std::cout << "[legacy_iface] /global_localization -> relocalization requested" << std::endl;
    return true;
}

bool LegacyXz3dSlamInterface::handleDumpRecorder(std_srvs::Trigger::Request&,
                                                 std_srvs::Trigger::Response& res) {
    // 飞行记录仪手动落盘 (诊断; LOST 翻转时已自动落盘, 这里给现场排查主动触发)
    const std::string path = engine_.dumpFlightRecorder("manual");
    res.success = !path.empty();
    res.message = path.empty() ? "no frames recorded" : path;
    return true;
}

bool LegacyXz3dSlamInterface::handleFinishTask(
    xzrobot_ros_dependency::CommonBoolService::Request&,
    xzrobot_ros_dependency::CommonBoolService::Response& res) {
    // 旧 finishTask (legacy.cpp:2601-2609): 有定位实例 -> updateMap() + status=true;
    // 否则 return false 且 status 留 false。降级: 定位模式且有核心 -> true。
    if (slam_mode_ != SlamMode::LOCALIZATION || !engine_.algo()) return false;
    res.status = true;
    return true;
}

bool LegacyXz3dSlamInterface::handleStartTask(
    xzrobot_ros_dependency::CommonBoolService::Request&,
    xzrobot_ros_dependency::CommonBoolService::Response& res) {
    // 旧 startTask (legacy.cpp:2591-2599): setUpdateMapFlag() + status=true
    if (slam_mode_ != SlamMode::LOCALIZATION || !engine_.algo()) return false;
    res.status = true;
    return true;
}

bool LegacyXz3dSlamInterface::handleExpandMap(
    xzrobot_ros_dependency::UpdateMap::Request& req,
    xzrobot_ros_dependency::UpdateMap::Response& res) {
    // **有意降级 (D5)**: 旧实现 (legacy.cpp:2538-2589) 是"重建建图实例 +
    // setIncreaseMapFlag() + loadCurrentMap(map_dir + \"/map.xzmap\") +
    // setInitPose(pos, rot * AngleAxis(angle,Z))", 核心没有"增量扩图后写回"的能力。
    ROS_WARN("[legacy_iface] update_map_operation: 降级实现 (D5) —— 用 loadMap + 切定位代替 "
             "旧实现的增量扩图写回; 行为不等价");
    const std::string base = legacyMapBase(req.map_dir);
    bool ok = false;
    if (engine_.algo()) {
        ok = engine_.algo()->loadMap(base);
        if (ok) {
            engine_.algo()->setPipelineMode("localization");
            slam_mode_ = SlamMode::LOCALIZATION;
            applyInitPose(req.initial_pose, false);
            lost_monitor_.reset();
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
            lost_monitor_.setMapLoaded(true);
            curr_map_path_ = req.map_dir;
        } else {
            ROS_ERROR_STREAM("[legacy_iface] update_map_operation: loadMap failed: " << base);
        }
    }
    res.status = ok;
    return true;
}

// ---------------------------------------------------------------- [T9] TF map->odom

} // namespace rosiwit_slam_ros1
