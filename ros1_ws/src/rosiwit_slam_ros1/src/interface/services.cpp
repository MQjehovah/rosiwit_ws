// ============================================================================
// Ros1SlamInterface 服务与初始位姿：/switch_map、/relocation、模式切换、
// 地图保存/停止/任务等服务处理器。
// ============================================================================

#include "ros1_slam_interface.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "convert.h"
#include "internal.h"
#include "map_adapter.h"

using namespace rosiwit_slam;
using namespace rosiwit_slam_ros1::interface_internal;

namespace rosiwit_slam_ros1 {
bool Ros1SlamInterface::callTagDetect(std::string& charging_pile_json) {
    charging_pile_json.clear();
    for (int i = 0; i < 10; ++i) {   // 1.0s 超时 (100ms x 10)
        xzrobot_ros_dependency::TagDetect srv;
        if (tag_detect_client_.call(srv)) {
            charging_pile_json = srv.response.charging_pile_json;
            return true;
        }
        ros::Duration(0.1).sleep();
    }
    ROS_WARN("[ros1_slam] get_charging_point unavailable");
    return false;   // 旧行为: 失败给空 charging_pile_json (D14)
}

bool Ros1SlamInterface::callSetDetectMode(int mode) {
    for (int i = 0; i < 5; ++i) {    // 0.5s 超时 + 100ms 重试
        xzrobot_ros_dependency::CommonIntRequestService srv;
        srv.request.mode = static_cast<int8_t>(mode);
        if (set_detect_mode_client_.call(srv)) return srv.response.result;
        ros::Duration(0.1).sleep();
    }
    ROS_WARN("[ros1_slam] set_detect_mode unavailable");
    return false;
}

bool Ros1SlamInterface::callSwitchMapPro(xzrobot_ros_dependency::SwitchMap::Request& req) {
    for (int i = 0; i < 10; ++i) {   // 1.0s 超时
        xzrobot_ros_dependency::SwitchMap srv;
        srv.request = req;
        if (switch_map_pro_client_.call(srv)) return srv.response.success;
        ros::Duration(0.1).sleep();
    }
    ROS_WARN("[ros1_slam] /switch_map_pro unavailable");
    return false;
}

// ---------------------------------------------------------------- [T8] 8 个 server
void Ros1SlamInterface::applyInitPose(
    const geometry_msgs::PoseWithCovarianceStamped& in, bool zero_if_insane) {
    // **不在这里重复实现**: /switch_map、/relocation 与 /initialpose 共用
    // SlamEngine::applyInitPose() —— insane 归零、z 归零、车身->雷达安装变换换算、setInitPose
    // 全部只有一份。这里只保留本路径的差异: z 强制归零。
    if (!engine_.algo()) return;
    engine_.applyInitPose(in, zero_if_insane, /*force_z_zero=*/true);
}

bool Ros1SlamInterface::handleModeSwitch(
    xzrobot_ros_dependency::SlamModeSwitchService::Request& req,
    xzrobot_ros_dependency::SlamModeSwitchService::Response& res) {
    if (req.mode == 0) {
        // 旧 switchToLocalization (legacy.cpp:2060-2115): 先置 FREE, 再调 /set_detect_mode(0),
        // **要求成功** —— 失败就 result=0 + return false。
        // 旧实现同时 DELETE_PTR 掉了建图/定位实例 -> 我们在这里也重建核心(丢上一轮地图)。
        resetCoreAndCaches();
        setModeFromConfig(SlamMode::FREE);
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
        setModeFromConfig(SlamMode::MAPPING);
        setWarningSuppression(true);
        res.result = 1;
        return true;
    }
    return false;   // 非法 mode: result 保持默认 0
}

bool Ros1SlamInterface::handleSwitchMap(
    xzrobot_ros_dependency::SwitchMap::Request& req,
    xzrobot_ros_dependency::SwitchMap::Response& res) {
    // 切图不限流：允许用户连续切换，最后一次请求胜出。
    // 后台加载旧图时若来了新请求，旧 job 完成后发现 generation 已过期，
    // 不再应用旧图的定位状态；新 job 继续加载新图。

    // 分支 1: 参数不合法
    if (req.map_dir.empty() || req.map_name.empty()) {
        res.success = false;
        return true;
    }
    // 分支 2: 同一张图且**核心确实在定位** -> 只重设初始位姿 + 转发 /switch_map_pro。
    // 不能只看 legacy 自己的 slam_mode_: set_slam_mode idle 后核心已 idle，
    // 但 timer 尚未刷新 slam_mode_ 时会误走这条分支，导致“切了但没定位输出”。
    const bool core_is_localization = (engine_.pipelineMode() == "localization");
    if (req.map_dir == curr_map_path_ && slam_mode_ == SlamMode::LOCALIZATION &&
        core_is_localization) {
        applyInitPose(req.initial_pose, true);
        // /switch_map_pro 由外部感知/地图管理节点提供；实测它可能阻塞数秒并
        // 返回 false，不能让它拖延/否决核心已经完成的同图切换。这里不再同步调用。
        res.success = true;
        return true;
    }
    // 分支 3: 换图
    // 先"停用并清空"上一张图：核心置 idle（不再产出旧图位姿/不再消费定位帧），
    // 清掉最近一次输出快照。随后后台加载新图，加载完成再切回 localization。
    // 这是行业常见的 deactivate -> swap -> activate，而不是继续让旧图输出再靠 10Hz 重发掩盖。
    if (engine_.algo()) {
        engine_.algo()->cancelMapLoad();
        engine_.algo()->setPipelineMode("idle");
    }
    engine_.clearLatestOutput();
    const uint64_t switch_gen = ++map_switch_generation_;
    relocating_ = true;
    map_switch_busy_.store(true);
    const bool detect_ok = callSetDetectMode(0);
    const std::string base = xzMapBase(req.map_dir);   // "/dir/map"

    // 1) 先在 ROS 层把老地图的 PGM 发布出去。App/RViz 依赖 /map，
    //    不能等核心解析完 25s 的 xzmap 才更新；PGM 的 127/128 会保持灰色未知区。
    rosiwit_slam::GridInfo static_grid;
    const bool pgm_ok = interface_internal::loadGridPgm(base, static_grid) &&
        static_grid.width > 0 && static_grid.height > 0;
    if (pgm_ok) {
        realtime_grid_map_ = toRosGrid(static_grid, kMapFrame);
        realtime_grid_map_area_ =
            static_cast<double>(static_grid.width) * static_grid.height *
            static_grid.resolution * static_grid.resolution;
        realtime_grid_map_.info.origin.position.z = realtime_grid_map_area_ * 0.00001;
        realtime_grid_map_puber_.publish(realtime_grid_map_);
        std::cout << "[ros1_slam] /switch_map: published PGM map "
                  << static_grid.width << "x" << static_grid.height
                  << " unknown=" << std::count(static_grid.data.begin(), static_grid.data.end(), -1)
                  << std::endl;
    }

    static_map_valid_ = pgm_ok;
    // 切图默认显示地图零点 (0,0,0)，不保留上一张图的位姿。
    // 核心已 idle + 输出快照已清空；这里同步清本层缓存，只发一次零点 slam/info，
    // 之后等新图加载/定位结果，不再周期性重发。
    curr_pose_ = Xz3dPose();
    curr_pose_valid_ = false;
    last_new_pose_ros_ = ros::Time();
    last_high_rate_pub_ = ros::Time();
    last_high_rate_tf_ = ros::Time();
    last_pose_stamp_ = 0.0;

    // 2) 服务立即返回，避免 App 5s 超时；完整定位地图在后台串行加载。
    res.success = detect_ok || pgm_ok;
    slam_mode_ = SlamMode::LOCALIZATION;
    curr_map_path_ = req.map_dir;
    curr_map_name_ = req.map_name;
    lost_monitor_.reset();
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
    lost_monitor_.setMapLoaded(true);
    setWarningSuppression(true);
    publishZeroPoseInfo();   // 立即发布零位姿，App 不必等核心加载完

    const geometry_msgs::PoseWithCovarianceStamped init_pose = req.initial_pose;
    {
        std::lock_guard<std::mutex> jobs_lk(map_switch_jobs_mtx_);
        // 回收已完成 job，避免连续切图时 future 列表无限增长。
        for (auto it = map_switch_jobs_.begin(); it != map_switch_jobs_.end();) {
            if (it->valid() && it->wait_for(std::chrono::seconds(0)) ==
                                   std::future_status::ready) {
                try { it->get(); } catch (...) {}
                it = map_switch_jobs_.erase(it);
            } else {
                ++it;
            }
        }
        map_switch_jobs_.push_back(std::async(std::launch::async,
            [this, base, init_pose, switch_gen]() {
                std::lock_guard<std::mutex> lk(map_switch_mtx_);
                map_switch_busy_.store(true);
                rosiwit_slam::MapData map_data;
                const bool loaded = map_adapter::loadMapData(base, map_data) &&
                                    engine_.algo() &&
                                    engine_.algo()->loadLocalizationMapData(map_data);
                if (switch_gen != map_switch_generation_.load()) {
                    std::cout << "[ros1_slam] /switch_map: discard stale map load: "
                              << base << std::endl;
                    map_switch_busy_.store(false);
                    return;
                }
                if (!loaded) {
                    ROS_ERROR_STREAM("[ros1_slam] /switch_map background loadMap failed: "
                                     << base);
                    relocating_ = false;
                    map_switch_busy_.store(false);
                    return;
                }
                engine_.algo()->setPipelineMode("localization");
                applyInitPose(init_pose, true);
                relocating_ = false;
                map_switch_busy_.store(false);
                std::cout << "[ros1_slam] /switch_map background localization map ready: "
                          << base << std::endl;
            }));
    }
    return true;
}

bool Ros1SlamInterface::handleRelocation(
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

bool Ros1SlamInterface::handleStopMap(
    xzrobot_ros_dependency::StopMapOperation::Request& req,
    xzrobot_ros_dependency::StopMapOperation::Response& res) {
    // 旧 stopMap (legacy.cpp:2419-2516)
    const std::string folder = req.map_dir;
    struct stat st;
    const bool exists = (!folder.empty() && ::stat(folder.c_str(), &st) == 0 && S_ISDIR(st.st_mode));
    if (!exists) {
        if (::mkdir(folder.c_str(), 0755) != 0) {
            // 旧实现: 建目录失败 -> return false 且 success 留 false
            ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: cannot create dir: " << folder);
            return false;
        }
    }
    // 旧实现要求 mapping_ptr_ 非空
    if (slam_mode_ != SlamMode::MAPPING || !engine_.algo()) return false;

    // 旧: mapping_ptr_->saveMap(folderPath, "map") -> 等价于 saveMap("<dir>/map")
    const std::string base = xzMapBase(req.map_dir);
    rosiwit_slam::MapData map_data;
    if (!engine_.algo()->exportMapData(map_data) ||
        !map_adapter::saveMapData(map_data, base)) {
        ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: xzmap export failed: " << base);
    }

    // 同时补 map.pgm / map.yaml: 旧实现的地图目录里有这两个文件, 且旧
    // LocalizationSetGridMap() (legacy.cpp:373-455) 会 imread("<base>.pgm") 并读
    // "<base>.yaml" 的 resolution/origin/occupied_thresh。核心的 saveGridMap 写的正是
    // 这个格式 (grid_map_io.cpp:7-39: P5, 首行=最大 y; yaml 含 image/resolution/
    // origin/negate/occupied_thresh/free_thresh/mode)。
    if (!engine_.algo()->saveGridMap(base + ".pgm", base + ".yaml", grid_resolution_)) {
        // 与 saveMap 同理: 失败只报错, 不改 success (旧实现也没有这一步可对照)
        ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: saveGridMap failed: " << base);
    } else {
        std::cout << "[ros1_slam] saved grid map: " << base << ".pgm / " << base
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

bool Ros1SlamInterface::handleGlobalLocalization(std_srvs::Empty::Request&,
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
    std::cout << "[ros1_slam] /global_localization -> relocalization requested" << std::endl;
    return true;
}

bool Ros1SlamInterface::handleDumpRecorder(std_srvs::Trigger::Request&,
                                                 std_srvs::Trigger::Response& res) {
    // 飞行记录仪手动落盘 (诊断; LOST 翻转时已自动落盘, 这里给现场排查主动触发)
    const std::string path = engine_.dumpFlightRecorder("manual");
    res.success = !path.empty();
    res.message = path.empty() ? "no frames recorded" : path;
    return true;
}

bool Ros1SlamInterface::handleFinishTask(
    xzrobot_ros_dependency::CommonBoolService::Request&,
    xzrobot_ros_dependency::CommonBoolService::Response& res) {
    // 旧 finishTask (legacy.cpp:2601-2609): 有定位实例 -> updateMap() + status=true;
    // 否则 return false 且 status 留 false。降级: 定位模式且有核心 -> true。
    if (slam_mode_ != SlamMode::LOCALIZATION || !engine_.algo()) return false;
    res.status = true;
    return true;
}

bool Ros1SlamInterface::handleStartTask(
    xzrobot_ros_dependency::CommonBoolService::Request&,
    xzrobot_ros_dependency::CommonBoolService::Response& res) {
    // 旧 startTask (legacy.cpp:2591-2599): setUpdateMapFlag() + status=true
    if (slam_mode_ != SlamMode::LOCALIZATION || !engine_.algo()) return false;
    res.status = true;
    return true;
}

bool Ros1SlamInterface::handleExpandMap(
    xzrobot_ros_dependency::UpdateMap::Request& req,
    xzrobot_ros_dependency::UpdateMap::Response& res) {
    // **有意降级 (D5)**: 旧实现 (legacy.cpp:2538-2589) 是"重建建图实例 +
    // setIncreaseMapFlag() + loadCurrentMap(map_dir + \"/map.xzmap\") +
    // setInitPose(pos, rot * AngleAxis(angle,Z))", 核心没有"增量扩图后写回"的能力。
    ROS_WARN("[ros1_slam] update_map_operation: 降级实现 (D5) —— 用 loadMap + 切定位代替 "
             "旧实现的增量扩图写回; 行为不等价");
    const std::string base = xzMapBase(req.map_dir);
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
            ROS_ERROR_STREAM("[ros1_slam] update_map_operation: loadMap failed: " << base);
        }
    }
    res.status = ok;
    return true;
}

// ---------------------------------------------------------------- [T9] TF map->odom

} // namespace rosiwit_slam_ros1
