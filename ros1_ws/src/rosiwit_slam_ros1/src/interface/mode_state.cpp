// ============================================================================
// mode_state.cpp — 接口层的模式状态机与核心生命周期
//
// 单一职责: 核心模式 <-> 接口模式 (SlamMode) 的同步、切模式时重建核心、
// 定位 latch 复位、以及"地面高度"这类随模式变化的小状态。
// 与 assembly.cpp (启动时跑一次的接线) 分开: 这套状态机在运行期随时被调用。
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

SlamMode Ros1SlamInterface::updateModeFromCore() {
    // 切图过渡：核心先被置 idle 清输出，本层仍保持 LOCALIZATION 语义，
    // 避免 timer 把 slam_mode_ 改成 FREE 后界面状态跳变/地图名被清。
    if (map_switch_busy_.load()) return slam_mode_;
    const std::string m = engine_.pipelineMode();
    if (m == "mapping")           slam_mode_ = SlamMode::MAPPING;
    else if (m == "localization") slam_mode_ = SlamMode::LOCALIZATION;
    else if (m == "idle") {
        slam_mode_ = SlamMode::FREE;
        // 切图过渡期间 core 会短暂 idle；此时不能清掉刚设置的地图状态。
        if (!map_switch_busy_.load() && !relocating_.load()) {
            setMapName(std::string());
            curr_map_path_.clear();
            realtime_grid_map_area_ = 0.0;
            static_map_valid_.store(false);
            curr_pose_valid_.store(false);
            clearStaticMap();
            last_static_map_pub_ = ros::Time();
        }
    } else {
        slam_mode_ = SlamMode::UNKNOWN;
    }
    return slam_mode_;
}

void Ros1SlamInterface::applyModeToCore(SlamMode m) {
    const char* core_mode = (m == SlamMode::MAPPING)      ? "mapping"
                          : (m == SlamMode::LOCALIZATION) ? "localization"
                                                          : "idle";
    // 改模式是结构操作: 走持锁入口, 拿不到 (正在装载地图/重建) 就明确失败
    if (!engine_.setCorePipelineMode(core_mode)) {
        ROS_WARN_STREAM("[ros1_slam] core rejected/deferred pipeline mode '" << core_mode
                        << "' (busy with map load?)");
    }
}

void Ros1SlamInterface::setModeFromConfig(SlamMode m) {
    const bool changed = (slam_mode_ != m);
    if (m != SlamMode::LOCALIZATION) static_map_valid_.store(false);
    slam_mode_ = m;
    applyModeToCore(m);
    if (changed) {
        debug_map_published_ = false;   // 旧实现每进模式重置
        relocating_ = false;
        std::cout << "[ros1_slam] mode -> " << modeName(m) << std::endl;
    }
}

void Ros1SlamInterface::resetCoreAndCaches() {
    // **默认开启** (与旧实现语义一致): 旧 xz_3d_slam 每次切建图都 DELETE_PTR(mapping_ptr_)
    // 再 new, 所以上一轮绝不残留。我们单实例, 只能显式重建内部模块达到同样效果。
    //
    // 为什么必须清: 不清的话新一帧点云会**在上一轮的旧图上做配准**, 旧图(已漂过的那一层)
    // 与新图叠在一起, 表现为"进建图先看到残留, 跑一会儿两层越错越开 → 地图全糊"。
    // 实机已复现: 日志里出现 skip core rebuild 后, 地图 occupied/total 长时间完全不变。
    //
    // 历史说明: 早期曾因"重建后地图不再累积"而默认关闭, 但后续实测发现**关闭时同样不累积**,
    // 该观察不可靠; 现按旧实现语义恢复默认开启, 并保留开关便于 A/B:
    //   _reset_on_mode_switch:=false 可退回不清图的行为。
    if (!reset_on_mode_switch_) {
        std::cout << "[ros1_slam] reset_on_mode_switch=false -> 保留上一轮地图 (会有残留)"
                  << std::endl;
        return;
    }
    std::cout << "[ros1_slam] reset_on_mode_switch=true -> rebuilding core (清空上一轮地图)"
              << std::endl;
    engine_.resetCore();   // 注意: 这里必须是 engine_.resetCore(), 不是本函数自己
    // 本层缓存一并复位 (见头文件注释)
    realtime_grid_.clear();
    clearStaticMap();
    realtime_grid_map_area_ = 0.0;
    curr_pose_ = Xz3dPose();
    curr_pose_valid_.store(false);
    last_new_pose_ros_ = ros::Time();
    last_high_rate_pub_ = ros::Time();
    last_high_rate_tf_ = ros::Time();
    last_pose_stamp_ = 0.0;
    last_mapping_path_time_ = -1.0;
    last_mapping_path_pub_ = ros::Time();
    last_location_info_pub_ = ros::Time();
    mapping_path_msg_ = nav_msgs::Path();
    last_core_loc_status_ = rosiwit_slam::ILocalization::INIT;
    core_ever_localized_ = false;
    debug_map_published_ = false;
    std::cout << "[ros1_slam] core rebuilt + local caches cleared" << std::endl;
}

void Ros1SlamInterface::resetLocalizationLatch() {
    relocating_ = true;
    last_core_loc_status_ = rosiwit_slam::ILocalization::INIT;
    core_ever_localized_ = false;
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
}

void Ros1SlamInterface::setInputSeen(bool imu, bool laser) {
    if (imu) have_imu_ = true;
    if (laser) have_laser_ = true;
}

void Ros1SlamInterface::computeLidarToBase() {
    // 核心 body 是 IMU; 地图系原点=初始 IMU. 地面 z = -T_base_link_imu.t.z()(base 在地面时).
    if (engine_.computeBaseFrames(/*blocking=*/false) && engine_.baseImu().valid) {
        scan_ground_z_ = -engine_.baseImu().t.z();
    }
}

} // namespace rosiwit_slam_ros1
