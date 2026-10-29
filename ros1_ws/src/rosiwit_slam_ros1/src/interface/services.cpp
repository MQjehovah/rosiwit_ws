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

bool Ros1SlamInterface::handleModeSwitch(
    xzrobot_ros_dependency::SlamModeSwitchService::Request& req,
    xzrobot_ros_dependency::SlamModeSwitchService::Response& res) {
    if (req.mode == 0) {
        // 旧 switchToLocalization (legacy.cpp:2060-2115): 先置 FREE, 再调 /set_detect_mode(0),
        // **要求成功** —— 失败就 result=0 + return false。
        // 旧实现同时 DELETE_PTR 掉了建图/定位实例 -> 我们在这里也重建核心(丢上一轮地图)。
        const bool rebuilt = resetCoreAndCaches();
        setModeFromConfig(SlamMode::FREE);
        // 不短路: 即使核心重建失败也要把检测模式切回 0, 否则 SLAM 侧状态与
        // 本层 FREE 不一致 (外层只关心整体成败)。
        const bool detect_ok = callSetDetectMode(0);
        const bool ok = rebuilt && detect_ok;
        res.result = ok ? 1 : 0;
        return ok;
    }
    if (req.mode == 1) {
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
        res.result = rebuilt ? 1 : 0;
        return rebuilt;
    }
    return false;   // 非法 mode: result 保持默认 0
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

    ctx.generation = ++map_switch_generation_;
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
    curr_pose_valid_.store(false);
    last_new_pose_ros_ = ros::Time();
    last_high_rate_pub_ = ros::Time();
    last_high_rate_tf_ = ros::Time();
    last_pose_stamp_ = 0.0;

    slam_mode_ = SlamMode::LOCALIZATION;
    curr_map_path_ = req.map_dir;
    setMapName(req.map_name);
    last_core_loc_status_ = rosiwit_slam::ILocalization::INIT;
    core_ever_localized_ = false;
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
    setWarningSuppression(true);
    publishZeroPoseInfo();   // 只发一次，之后由核心输出接管
    return ctx;
}

void Ros1SlamInterface::finishMapLoadJob() {
    std::lock_guard<std::mutex> lk(map_load_mtx_);
    if (map_load_inflight_ > 0) --map_load_inflight_;
    if (map_load_inflight_ == 0) map_load_cv_.notify_all();
}

void Ros1SlamInterface::startMapLoadJob(
    const MapSwitchContext& ctx,
    const geometry_msgs::PoseWithCovarianceStamped& init_pose) {
    // 大图解析在 aarch64 上可达 25s, 不能拖住 /switch_map 服务线程;
    // 投到执行器的 "disk_read" lane: threads=1 => 串行, 天然替代原来的
    // map_switch_mtx_ + future 列表; generation 保证只有最后一次请求生效。
    {
        std::lock_guard<std::mutex> lk(map_load_mtx_);
        ++map_load_inflight_;   // 必须在提交前自增, 否则析构可能漏等
    }

    const std::string base = ctx.base;
    const uint64_t gen = ctx.generation;
    // 在途计数挂在 DoneFn 上: 任务被执行 / 被取消 / 被丢队列三种情况都会回调,
    // 所以析构的等待一定能结束 (I4)。任务体里不再自己收尾。
    const bool accepted = engine_.runOnLaneWithDone("disk_read", "switch_map_load",
        [this, base, init_pose, gen](rosiwit_common::CancelToken token) {
            map_switch_busy_.store(true);
            rosiwit_slam::MapData map_data;
            // 解析 25s 期间核心可能被模式切换重建: 抓解析前的 generation, 装载时校验
            const uint64_t core_gen = engine_.coreGeneration();
            const bool parsed = map_adapter::loadMapData(base, map_data);
            if (token.cancelled()) {
                std::cout << "[ros1_slam] /switch_map: load cancelled after parse: "
                          << base << std::endl;
                map_switch_busy_.store(false);
                relocating_ = false;
                return;
            }
            const bool loaded = parsed && engine_.loadLocalizationMapData(map_data, core_gen);
            if (gen != map_switch_generation_.load()) {
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
            engine_.setCorePipelineMode("localization", /*wait_ms=*/1000);
            applyInitPose(init_pose, true);
            relocating_ = false;
            map_switch_busy_.store(false);
            std::cout << "[ros1_slam] /switch_map background localization map ready: "
                      << base << std::endl;
        },
        [this](bool ok) {
            if (!ok) {
                // 被取消/丢弃/异常: 别把"切图中"和"重定位中"标志留下来
                map_switch_busy_.store(false);
                relocating_ = false;
            }
            finishMapLoadJob();
        });

    if (!accepted) {
        ROS_ERROR_STREAM("[ros1_slam] /switch_map: rejected by executor (lane busy): "
                         << base);
    }
}

bool Ros1SlamInterface::handleSwitchMap(
    xzrobot_ros_dependency::SwitchMap::Request& req,
    xzrobot_ros_dependency::SwitchMap::Response& res) {
    if (req.map_dir.empty() || req.map_name.empty()) {
        res.success = false;
        return true;
    }

    // 同一张图且核心确实在定位：只重设初始位姿，不重载地图。
    // 不能只看 slam_mode_：切 idle 后 timer 尚未刷新时会误走这里。
    const bool core_is_localization = (engine_.pipelineMode() == "localization");
    if (req.map_dir == curr_map_path_ && slam_mode_ == SlamMode::LOCALIZATION &&
        core_is_localization) {
        // 同名重存过 (重新建图覆盖了同目录) 必须重载, 否则 App 看到的还是内存里的旧图.
        struct stat st {};
        const std::string base = xzMapBase(req.map_dir);
        double mtime = 0.0;
        if (::stat((base + ".xzmap").c_str(), &st) == 0) {
            mtime = static_cast<double>(st.st_mtime);
        }
        if (mtime <= 0.0 || mtime <= loaded_map_mtime_) {
            applyInitPose(req.initial_pose, true);
            res.success = true;
            return true;
        }
        ROS_WARN_STREAM("[ros1_slam] /switch_map: 同名地图已更新 (mtime "
                        << mtime << " > " << loaded_map_mtime_ << "), 强制重载");
        curr_map_path_.clear();
    }

    // 换图：阶段一同步完成（服务立即返回），阶段二在后台加载 MapData。
    {
        struct stat st {};
        const std::string base = xzMapBase(req.map_dir);
        loaded_map_mtime_ = (::stat((base + ".xzmap").c_str(), &st) == 0)
          ? static_cast<double>(st.st_mtime) : 0.0;
    }
    MapSwitchContext ctx = beginMapSwitch(xzMapBase(req.map_dir), req);
    startMapLoadJob(ctx, req.initial_pose);
    res.success = ctx.detect_ok || ctx.pgm_ok;
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
    last_core_loc_status_ = rosiwit_slam::ILocalization::INIT;
    core_ever_localized_ = false;
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
    res.success = true;
    return true;
}

bool Ros1SlamInterface::handleStopMap(
    xzrobot_ros_dependency::StopMapOperation::Request& req,
    xzrobot_ros_dependency::StopMapOperation::Response& res) {
    // 存图 = 导出整图 + 写 xzmap + 生成 pgm/yaml + 缩略图 + 充电桩检测 (秒级)。
    // 全部丢到 disk_write lane 上做, 服务在这里等结果 —— 主线程不被 CPU/IO 占住,
    // 而位姿/TF 已经由 slam_tick 发布, 所以等待期间对外输出照常。
    bool do_ok = true;
    const bool ran = engine_.runOnLaneAndWait(
        "disk_write", "stop_map",
        [this, &req, &res, &do_ok](rosiwit_common::CancelToken) { do_ok = doStopMap(req, res); },
        /*timeout_ms=*/180000);
    if (!ran) {
        // 池拒绝/超时: 任务可能还在跑, 结果丢不掉就明确报错
        res.success = false;
        if (res.error.empty()) res.error = "MAP_SAVE_EXECUTOR_FAILED";
        ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: map save job failed/timeout: "
                         << req.map_dir);
        return true;
    }
    return do_ok;   // 与旧实现一致: mkdir 失败返回 false, 其余失败用 res.error 表达
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
            last_core_loc_status_ = rosiwit_slam::ILocalization::INIT;
            core_ever_localized_ = false;
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
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
