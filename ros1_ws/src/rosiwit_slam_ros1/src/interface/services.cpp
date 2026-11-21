// ============================================================================
// services.cpp — 兼容层的服务处理
//
// 单一职责: 17 个 ROS 服务的入参校验、状态迁移与结果填写
// (/switch_map /relocation /stop_map_operation /slam_mode_switch ...),
// 以及它们需要的"重活投递"(切图/存图走 lane, 见 core_gateway.cpp)。
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

/// 控制线程与调用方之间传递"存图结果"(超时后闭包仍可能写它, 所以用 shared_ptr)
struct StopResult {
    bool ret = false;      // 处理函数返回值 (与旧实现一致)
    bool success = false;  // response.success
    std::string error;
};
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
    const geometry_msgs::PoseWithCovarianceStamped& in, bool zero_if_insane,
    bool count_as_jump) {
    // **不在这里重复实现**: /switch_map、/relocation 与 /initialpose 共用
    // SlamAlgorithm::applyInitPose() —— insane 归零、z 归零、车身->雷达安装变换换算、setInitPose
    // 全部只有一份。这里只保留本路径的差异: z 强制归零。
    if (!engine_.algo()) return;
    engine_.applyInitPose(in, zero_if_insane, /*force_z_zero=*/true, count_as_jump);
}

bool Ros1SlamInterface::applyModeSwitch(int mode) {
    if (mode == 0) {
        // 旧 switchToLocalization (legacy.cpp:2060-2115): 先置 FREE, 再调 /set_detect_mode(0),
        // **要求成功** —— 失败就 result=0 + return false。
        // 旧实现同时 DELETE_PTR 掉了建图/定位实例 -> 我们在这里也重建核心(丢上一轮地图)。
        const bool rebuilt = resetCoreAndCaches();
        setModeFromConfig(SlamMode::FREE);
        // 不短路: 即使核心重建失败也要把检测模式切回 0, 否则 SLAM 侧状态与
        // 本层 FREE 不一致 (外层只关心整体成败)。
        const bool detect_ok = callSetDetectMode(0);
        return rebuilt && detect_ok;
    }
    if (mode == 1) {
        // 旧 switchToMapping (legacy.cpp:1974-2058): 也调 /set_detect_mode(1), 但
        // "if (!success) return false;" 那行在 2026 是**注释掉的** -> 结果不影响返回值;
        // 随后 run() -> warning_code_start_task_ = true (184), 即重新开启抑制窗口。
        // **关键**: 旧实现先 DELETE_PTR(mapping_ptr_) 再 new 一个全新实例, 所以退出建图再
        // 进建图**不会残留上一轮地图**; 我们单核心实例, 必须显式重建。
        const bool rebuilt = resetCoreAndCaches();
        callSetDetectMode(1);
        setModeFromConfig(SlamMode::MAPPING);
        setWarningSuppression(true);
        // 核心没重建成功时不能对外报 MAPPING: 否则"新一帧在旧图上配准"会静默发生。
        return rebuilt;
    }
    return false;
}

bool Ros1SlamInterface::handleModeSwitch(
    xzrobot_ros_dependency::SlamModeSwitchService::Request& req,
    xzrobot_ros_dependency::SlamModeSwitchService::Response& res) {
    if (req.mode != 0 && req.mode != 1) {
        return false;   // 非法 mode: result 保持默认 0
    }
    const int mode = req.mode;

    // 投到控制线程: 与载图/存图严格串行 (它们都要改核心对象图), 服务线程只等结果。
    // 结果用 shared_ptr 传递: 超时后闭包还在队列里, 不能捕获栈上的 res。
    auto ok = std::make_shared<std::atomic<bool>>(false);
    const bool done = control_.postAndWait(
        [this, mode, ok] { ok->store(applyModeSwitch(mode)); }, /*timeout_ms=*/15000);
    if (!done) {
        ROS_WARN_STREAM("[ros1_slam] /slam/mode_switch: control thread busy > 15s, "
                        "returning accepted (mode " << mode << ")");
        res.result = 1;
        return true;
    }
    const bool switched = ok->load();
    res.result = switched ? 1 : 0;
    std::cout << "[ros1_slam] /slam/mode_switch: mode " << mode << " -> "
              << (switched ? "ok" : "FAILED") << std::endl;
    return switched;
}

Ros1SlamInterface::MapSwitchContext Ros1SlamInterface::beginMapSwitch(
    const std::string& base, const xzrobot_ros_dependency::SwitchMap::Request& req) {
    MapSwitchContext ctx;
    ctx.base = base;

    // 1. 停用旧定位：核心 idle + 丢弃旧输出，之后任何残留输出都不能进入界面。
    // 结构操作一律走 SlamAlgorithm 的持锁入口 (不能直接拿 algo() 裸指针)
    engine_.cancelCoreMapLoad();
    engine_.setCorePipelineMode("idle");
    engine_.clearLatestOutput();

    relocating_ = true;
    map_switch_busy_.store(true);
    ctx.detect_ok = callSetDetectMode(0);

    // 2. 先发布新图 PGM，App/RViz 不必等 xzmap 解析完成。
    rosiwit_slam::GridInfo static_grid;
    {
        // 读 PGM 与"整段写盘"互斥 (I5 读侧): 等不到就跳过静态图, 服务照常继续
        map_adapter::MapIoGuard pgm_guard(/*wait_ms=*/1000);
        ctx.pgm_ok = pgm_guard.ok() &&
                     interface_internal::loadGridPgm(base, static_grid) &&
                     static_grid.width > 0 && static_grid.height > 0;
    }
    if (ctx.pgm_ok) {
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
    static_map_valid_.store(ctx.pgm_ok);

    // 3. 切图默认显示地图零点，清掉上一张图的位姿/高频状态。
    {
        std::lock_guard<std::mutex> lk(pose_mu_);
        curr_pose_ = Xz3dPose();
    }
    // latch 状态交给 slam_pub 线程复位 (服务线程只置掩码, 避免与 50Hz 发布争写)
    requestPublishReset(kPubResetPoseLatch | kPubResetLocStatus |
                        kPubResetOdomMon | kPubResetDebugFlags);

    slam_mode_ = SlamMode::LOCALIZATION;
    setCurrentMapPath(req.map_dir);
    setMapName(req.map_name);
    setWarningSuppression(true);
    publishZeroPoseInfo();   // 只发一次，之后由核心输出接管
    return ctx;
}

bool Ros1SlamInterface::runMapLoadInline(
    const MapSwitchContext& ctx,
    const geometry_msgs::PoseWithCovarianceStamped& init_pose) {
    // 在**控制线程**上同步装载: 解析大图 (aarch64 实测可达 25s) + 装进核心。
    // 控制线程不承担延迟敏感的工作, 所以在这里阻塞是允许的; 而且它与存图/切模式
    // 天然串行 (同一个队列), 不需要 generation 那种跨线程仲裁。
    map_switch_busy_.store(true);
    rosiwit_slam::MapData map_data;
    const uint64_t core_gen = engine_.coreGeneration();
    const bool parsed = map_adapter::loadMapData(ctx.base, map_data);
    if (!parsed) {
        ROS_ERROR_STREAM("[ros1_slam] /switch_map: parse failed: " << ctx.base);
        relocating_ = false;
        map_switch_busy_.store(false);
        return false;
    }
    if (!engine_.loadLocalizationMapData(map_data, core_gen)) {
        ROS_ERROR_STREAM("[ros1_slam] /switch_map: loadMap failed: " << ctx.base);
        relocating_ = false;
        map_switch_busy_.store(false);
        return false;
    }
    if (!engine_.setCorePipelineMode("localization", /*wait_ms=*/1000)) {
        ROS_ERROR_STREAM("[ros1_slam] /switch_map: setPipelineMode(localization) FAILED");
    }
    applyInitPose(init_pose, true);
    relocating_ = false;
    map_switch_busy_.store(false);
    std::cout << "[ros1_slam] /switch_map: localization map ready: " << ctx.base
              << std::endl;
    return true;
}

bool Ros1SlamInterface::handleSwitchMap(
    xzrobot_ros_dependency::SwitchMap::Request& req,
    xzrobot_ros_dependency::SwitchMap::Response& res) {
    if (req.map_dir.empty() || req.map_name.empty()) {
        res.success = false;
        return true;
    }

    // 上一次装载还在跑: 直接回"已受理", 不排进队列 —— 否则 App 的重试会让控制线程
    // 把同一张大图解析两遍 (25s x2)。
    if (map_switch_busy_.load()) {
        ROS_WARN_STREAM("[ros1_slam] /switch_map: previous load still running, "
                        "accepted without reload: " << req.map_dir);
        res.success = true;
        return true;
    }

    const auto cmd = req;   // 值拷贝: 超时后闭包仍在队列里, 不能引用栈对象
    auto ok = std::make_shared<std::atomic<bool>>(false);
    const bool done = control_.postAndWait(
        [this, cmd, ok] {
            auto applySameMap = [this, &cmd]() {
                // 同一张图 + 核心确实在定位 + 文件没更新: 只重设初始位姿
                const bool core_is_localization = (engine_.pipelineMode() == "localization");
                if (cmd.map_dir != currentMapPath() ||
                    slam_mode_ != SlamMode::LOCALIZATION || !core_is_localization) {
                    return false;
                }
                struct stat st {};
                const std::string base = xzMapBase(cmd.map_dir);
                double mtime = 0.0;
                if (::stat((base + ".xzmap").c_str(), &st) == 0) {
                    mtime = static_cast<double>(st.st_mtime);
                }
                if (mtime > 0.0 && mtime > loaded_map_mtime_.load()) {
                    ROS_WARN_STREAM("[ros1_slam] /switch_map: 同名地图已更新 (mtime "
                                    << mtime << " > " << loaded_map_mtime_.load()
                                    << "), 强制重载");
                    setCurrentMapPath(std::string());
                    return false;   // 需要真的重载
                }
                applyInitPose(cmd.initial_pose, true);
                return true;
            };

            if (applySameMap()) { ok->store(true); return; }

            {
                struct stat st {};
                const std::string base = xzMapBase(cmd.map_dir);
                loaded_map_mtime_.store((::stat((base + ".xzmap").c_str(), &st) == 0)
                    ? static_cast<double>(st.st_mtime) : 0.0);
            }
            MapSwitchContext ctx = beginMapSwitch(xzMapBase(cmd.map_dir), cmd);
            const bool loaded = runMapLoadInline(ctx, cmd.initial_pose);
            // 与旧行为一致: PGM 发布成功即算"换图受理", 完整装载失败另看日志
            ok->store(loaded || ctx.pgm_ok);
            if (!loaded) {
                ROS_ERROR_STREAM("[ros1_slam] /switch_map: load FAILED: " << ctx.base);
            }
        },
        /*timeout_ms=*/12000);
    if (!done) {
        // 大图解析可到 25s: 超过 12s 按"已受理"返回, 装载仍在控制线程继续
        ROS_WARN_STREAM("[ros1_slam] /switch_map: still loading after 12s, "
                        "returning accepted: " << req.map_dir);
        res.success = true;
        return true;
    }
    res.success = ok->load();
    return true;
}

bool Ros1SlamInterface::handleRelocation(
    xzrobot_ros_dependency::Relocation::Request& req,
    xzrobot_ros_dependency::Relocation::Response& res) {
    return applyRelocation(req, res, /*count_as_jump=*/true);
}

bool Ros1SlamInterface::applyRelocation(
    xzrobot_ros_dependency::Relocation::Request& req,
    xzrobot_ros_dependency::Relocation::Response& res, bool count_as_jump) {
    // 旧 relocate (legacy.cpp:2394-2418): relocating_ 置真又立刻置假;
    // setInitPose 的 **z 强制 0**; success 恒 true。
    // 注意这里**没有** |x|,|y| > 1e4 归零 —— 那只在 /switch_map 里。
    relocating_ = true;
    applyInitPose(req.initial_pose, false, count_as_jump);
    relocating_ = false;
    // 契约字段 relocalize_count 由 SlamAlgorithm::poseJumpSeq() 单源提供
    // (applyInitPose 里 +1), 这里不再自己计数。
    // [T11] 进入重定位 -> 清 latch (下次真的定位上之前报 1, 不报 0)
    requestPublishReset(kPubResetLocStatus | kPubResetOdomMon);
    res.success = true;
    return true;
}

bool Ros1SlamInterface::handleStopMap(
    xzrobot_ros_dependency::StopMapOperation::Request& req,
    xzrobot_ros_dependency::StopMapOperation::Response& res) {
    // 存图 = 导出整图 + 写 xzmap/pgm + 缩略图 + 调外部服务, 秒级 (大图上限 180s)。
    // 投到控制线程: 与切图/切模式严格串行 (它们改的是同一份核心地图)。
    // 位姿/TF 由 io 线程的心跳定时器照常发布, 所以等待期间对外输出不受影响。
    const auto cmd = req;   // 值拷贝: 超时后闭包仍在队列里
    auto out = std::make_shared<StopResult>();
    const bool done = control_.postAndWait(
        [this, cmd, out] {
            // lambda 里 cmd 是 const; doStopMap 要非 const 引用 (旧实现会改 req)
            xzrobot_ros_dependency::StopMapOperation::Request r_req = cmd;
            xzrobot_ros_dependency::StopMapOperation::Response r_res;
            out->ret = doStopMap(r_req, r_res);
            out->success = r_res.success;
            out->error = r_res.error;
        },
        /*timeout_ms=*/180000);
    if (!done) {
        res.success = false;
        if (res.error.empty()) res.error = "MAP_SAVE_TIMEOUT";
        ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: save still running after 180s: "
                         << req.map_dir);
        return true;
    }
    res.success = out->success;
    res.error = out->error;
    return out->ret;   // 与旧实现一致: mkdir 失败返回 false, 其余失败用 res.error 表达
}

bool Ros1SlamInterface::doStopMap(
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

    // ---- 保存前质量门限 ----
    // 以前只要点云非空就写盘并返回 success=true, 空图/全 unknown/墙体断裂的图也会
    // "保存成功", 下一次定位直接对不上. 这里先用同一条栅格生成路径体检.
    {
        rosiwit_slam::GridInfo gi;
        if (!engine_.getCoreGridMap(grid_resolution_, gi) || gi.data.empty()) {
            res.success = false;
            res.error = "MAP_QUALITY_NO_GRID";
            ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: 栅格生成失败, 拒绝保存");
            return true;
        }
        size_t free_n = 0;
        size_t occ_n = 0;
        size_t unk_n = 0;
        for (const auto v : gi.data) {
            if (v >= 50) { ++occ_n; }
            else if (v >= 0) { ++free_n; }
            else { ++unk_n; }
        }
        const double cell = std::max(1e-6, gi.resolution * gi.resolution);
        const double free_area = static_cast<double>(free_n) * cell;
        const double total = static_cast<double>(gi.data.size());
        const double unknown_frac = total > 0.0 ? static_cast<double>(unk_n) / total : 1.0;
        const double map_quality_free_min_m2 =
          (ros::param::get("~map_quality_free_min_m2", map_quality_free_min_m2_))
            ? map_quality_free_min_m2_ : map_quality_free_min_m2_;
        const double map_quality_unknown_max =
          (ros::param::get("~map_quality_unknown_max", map_quality_unknown_max_))
            ? map_quality_unknown_max_ : map_quality_unknown_max_;
        std::cout << "[ros1_slam] map quality: free=" << free_area << " m2 occupied="
                  << static_cast<double>(occ_n) * cell << " m2 unknown="
                  << (unknown_frac * 100.0) << "% (+" << gi.resolution << "m, "
                  << gi.width << "x" << gi.height << ")" << std::endl;
        if (free_area < map_quality_free_min_m2 || unknown_frac > map_quality_unknown_max) {
            res.success = false;
            res.error = "MAP_QUALITY_REJECTED";
            ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: 地图质量不达标 (free="
                             << free_area << " m2 < " << map_quality_free_min_m2
                             << " 或 unknown=" << (unknown_frac * 100.0)
                             << "% > " << (map_quality_unknown_max * 100.0)
                             << "%), 拒绝保存");
            return true;
        }
    }

    // 整段写盘 (xzmap + pgm/yaml + 缩略图) 与载图/并发写盘互斥:
    // 之前只有 saveMapData 内部那把锁, 重命名/缩略图/PGM 读取都在锁外。
    map_adapter::MapIoGuard io_guard(/*wait_ms=*/5000);
    if (!io_guard.ok()) {
        res.success = false;
        res.error = "MAP_IO_BUSY";
        ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: map io busy (load in progress): "
                         << base);
        return true;
    }

    rosiwit_slam::MapData map_data;
    if (!engine_.exportCoreMapData(map_data) ||
        !map_adapter::saveMapData(map_data, base)) {
        res.success = false;
        res.error = "MAP_SAVE_XZMAP_FAILED";
        ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: xzmap export failed: " << base);
        return true;
    }

    // 同时补 map.pgm / map.yaml: 旧实现的地图目录里有这两个文件, 且旧
    // LocalizationSetGridMap() (legacy.cpp:373-455) 会 imread("<base>.pgm") 并读
    // "<base>.yaml" 的 resolution/origin/occupied_thresh。核心的 saveGridMap 写的正是
    // 这个格式 (grid_map_io.cpp:7-39: P5, 首行=最大 y; yaml 含 image/resolution/
    // origin/negate/occupied_thresh/free_thresh/mode)。
    // 先写临时文件再 rename: 掉电/进程被杀时不会留下"新 xzmap + 旧 pgm"的混合.
    const std::string tmp_pgm = base + ".tmp.pgm";
    const std::string tmp_yaml = base + ".tmp.yaml";
    if (!engine_.saveCoreGridMap(tmp_pgm, tmp_yaml, grid_resolution_)) {
        res.success = false;
        res.error = "MAP_SAVE_GRID_FAILED";
        ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: saveGridMap failed: " << base);
        return true;
    }
    std::remove((base + ".pgm").c_str());
    std::remove((base + ".yaml").c_str());
    if (std::rename(tmp_pgm.c_str(), (base + ".pgm").c_str()) != 0 ||
        std::rename(tmp_yaml.c_str(), (base + ".yaml").c_str()) != 0) {
        res.success = false;
        res.error = "MAP_SAVE_RENAME_FAILED";
        ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: rename tmp map failed: " << base);
        return true;
    }
    {
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

    // curr_pose_ 由 tick 线程发布时更新, 这里是另一条线程 -> 取快照
    const Xz3dPose pose_snap = currentPoseSnapshot();
    res.current_pose.header.frame_id = kMapFrame;
    res.current_pose.pose.pose.position.x = pose_snap.x;
    res.current_pose.pose.pose.position.y = pose_snap.y;
    res.current_pose.pose.pose.position.z = pose_snap.z;   // **真实 z**, 不是 -0.07
    res.current_pose.pose.pose.orientation.x = pose_snap.q.x();
    res.current_pose.pose.pose.orientation.y = pose_snap.q.y();
    res.current_pose.pose.pose.orientation.z = pose_snap.q.z();
    res.current_pose.pose.pose.orientation.w = pose_snap.q.w();
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
    requestPublishReset(kPubResetOdomMon);
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
    // 核心只认 PCD 文件路径; xzMapBase 返回的是无扩展名的 base (如 "<dir>/map"),
    // 直接传进去 loadMapForLocalization 的 ifstream 探测必然失败。
    const std::string pcd = base + ".pcd";
    bool ok = false;
    if (engine_.algo()) {
        ok = engine_.loadCoreMap(pcd);
        if (!ok) {
            ROS_ERROR_STREAM("[ros1_slam] update_map_operation: 需要 " << pcd
                             << " (核心的扩图/载图只吃 PCD); 若目录里只有 .xzmap, "
                             << "请走 /switch_map");
        }
        if (ok) {
            engine_.setCorePipelineMode("localization", /*wait_ms=*/1000);
            slam_mode_ = SlamMode::LOCALIZATION;
            applyInitPose(req.initial_pose, false);
            requestPublishReset(kPubResetLocStatus | kPubResetOdomMon);
            setCurrentMapPath(req.map_dir);
        } else {
            ROS_ERROR_STREAM("[ros1_slam] update_map_operation: loadMap failed: " << base);
        }
    }
    res.status = ok;
    return true;
}

// ---------------------------------------------------------------- [T9] TF map->odom

} // namespace rosiwit_slam_ros1
