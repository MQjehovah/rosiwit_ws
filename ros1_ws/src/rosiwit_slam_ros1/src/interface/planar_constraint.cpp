// ============================================================================
// planar_constraint.cpp — 2D 激光平面约束 (定位漂移的连续纠偏)
//
// 这是一个**算法**单元, 不是发布逻辑: 用当前帧与地图做栅格相关, 得到平移/航向
// 修正量, 再走 /relocation 的完整重定位路径。
// 与 pose_publisher.cpp (发布) 分开: 算法改动不会牵动发布链路。
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

namespace {
/// 角度归一到 (-pi, pi]
double planarNormAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
}  // namespace

void Ros1SlamInterface::planarConstraintStep() {
    if (!planar_constraint_enabled_) return;
    if (slam_mode_ != SlamMode::LOCALIZATION) return;
    // 只有"曾经真的定位成功过"才允许 2D 纠偏: 否则启动阶段位姿本来就是 0,
    // 用 2D 匹配去"修"它可能把从未定位的机器人拉到别处.
    if (!planar_ever_locked_ || !curr_pose_valid_) return;
    const ros::Time now = ros::Time::now();
    if (planar_pts_stamp_.isZero() ||
        (now - planar_pts_stamp_).toSec() > 0.6) return;   // 激光不新鲜
    if (static_cast<int>(planar_pts_base_.size()) < 60) return;

    // 校验策略:
    //   TRACKING 且匹配度足够 -> 低频 (1 Hz) 校验, 只在大偏差时纠偏
    //   DEGRADED/LOST         -> 4 Hz 校验 + 更大搜索窗
    const auto st = engine_.localizationStatus();
    const bool healthy = (st == rosiwit_slam::ILocalization::LOCALIZED &&
                          engine_.getMatchingDegree() >= loc_degrade_matching_);
    const double min_period = healthy ? 1.0 : 0.25;
    if (!planar_last_try_.isZero() &&
        (now - planar_last_try_).toSec() < min_period) return;
    planar_last_try_ = now;

    const nav_msgs::OccupancyGrid map_snap = staticMapSnapshot();
    if (map_snap.data.empty() ||
        map_snap.info.width == 0 || map_snap.info.height == 0) {
        return;
    }

    // 地图几何变化时重建栅格 + 距离场 (一次性开销, 不是每帧).
    const bool geom_changed =
        planar_grid_.width != static_cast<int>(map_snap.info.width) ||
        planar_grid_.height != static_cast<int>(map_snap.info.height) ||
        std::fabs(planar_grid_.resolution - realtime_grid_map_.info.resolution) > 1e-9 ||
        planar_grid_.data.size() != realtime_grid_map_.data.size();
    if (geom_changed || planar_field_.empty()) {
        planar_grid_.width = static_cast<int>(realtime_grid_map_.info.width);
        planar_grid_.height = static_cast<int>(realtime_grid_map_.info.height);
        planar_grid_.resolution = realtime_grid_map_.info.resolution;
        planar_grid_.origin_x = realtime_grid_map_.info.origin.position.x;
        planar_grid_.origin_y = realtime_grid_map_.info.origin.position.y;
        planar_grid_.data = realtime_grid_map_.data;
        planar_field_ = buildPlanarDistanceField(planar_grid_, 1.5);
        planar_pending_ = false;
        std::cout << "[planar] distance field built: " << planar_grid_.width << "x"
                  << planar_grid_.height << " res=" << planar_grid_.resolution << std::endl;
    }

    PlanarMatcherConfig cfg;
    const bool lost = (st == rosiwit_slam::ILocalization::LOST);
    // LOST 时给更大的搜索窗 (等价于一次受约束的重定位);
    // healthy 时窗口小一点, 只用来"校验 + 小修正".
    cfg.search_xy = lost ? 1.20 : (healthy ? 0.45 : 0.60);
    cfg.search_yaw = lost ? 0.40 : (healthy ? 0.15 : 0.30);
    cfg.min_points = 80;
    // 这台 2D 雷达是倾斜安装的: 远处光束打到地面/天花, 这类点天然不在地图墙上.
    // 实测墙面点占比 ~0.64, 所以内点率门限按 0.50 设, 真伪靠残差/条件数/连续一致.
    cfg.min_inlier_ratio = 0.50;
    cfg.max_residual = 0.10;   // 地图是 0.05 栅格拼出来的墙面, 留出栅格量化余量

    // 恢复搜索: 失败就放大窗口重试 (最多 3 轮, 覆盖 ~2m 内的错锁).
    // 只在"最优解贴在边界"或"匹配无效"时放大: 拉到 0.6 -> 1.08 -> 1.94m.
    PlanarMatchResult r;
    for (int attempt = 0; attempt < 3; ++attempt) {
        // curr_pose_ 由 tick 线程写、这里在输入/主线程读: 必须走加锁快照,
        // 否则可能拿到 (x,y) 与 yaw 不同源的撕裂值。
        const Xz3dPose cp = currentPoseSnapshot();
        r = matchPlanarScan(planar_pts_base_, cp.x, cp.y, cp.yaw,
                            planar_grid_, planar_field_, cfg);
        if (r.valid) { break; }
        if (attempt + 1 >= 3 || r.points < cfg.min_points) { break; }
        cfg.search_xy *= 1.8;
        cfg.search_yaw *= 1.5;
        std::cout << "[planar] widening search to " << cfg.search_xy << " m (prev ratio="
                  << r.inlier_ratio << " resid=" << r.mean_residual << ")" << std::endl;
    }
    if (!r.valid) {
        planar_pending_ = false;
        std::cout << "[planar] miss: pts=" << r.points << " inliers=" << r.inliers
                  << " ratio=" << r.inlier_ratio << " resid=" << r.mean_residual
                  << " (need ratio>=" << cfg.min_inlier_ratio
                  << " resid<=" << cfg.max_residual << ")" << std::endl;
        return;
    }

    // 有效匹配: 记录真实协方差 (DEGRADED 时写进 /localization/state).
    planar_cov_stamp_ = now;
    planar_cov_xx_ = r.cov_xx;
    planar_cov_yy_ = r.cov_yy;
    planar_cov_yaw_ = r.cov_yyaw;

    const Xz3dPose cp2 = currentPoseSnapshot();
    const double dx = r.x - cp2.x;
    const double dy = r.y - cp2.y;
    const double dyaw = planarNormAngle(r.yaw - cp2.yaw);
    const bool big = std::hypot(dx, dy) > 0.15 || std::fabs(dyaw) > 0.12;
    if (!big) { planar_pending_ = false; return; }
    if (r.condition > 1.0e5) return;   // 退化 (长走廊): 不硬拉, 交给 3D 恢复

    // 连续两次一致才执行: 单帧误匹配 (动态障碍/玻璃) 不能直接把位姿拉飞.
    if (planar_pending_ && (now - planar_pending_stamp_).toSec() < 2.0 &&
        std::hypot(r.x - planar_px_, r.y - planar_py_) < 0.08 &&
        std::fabs(planarNormAngle(r.yaw - planar_pyaw_)) < 0.06) {
        // 防周期: 同一处纠偏短时间内反复出现, 说明核心没有接受 (还在用旧位姿),
        // 继续硬拉会变成死循环 —— 停下来等 3D 恢复或人工介入.
        if (!last_planar_applied_.isZero() &&
            (now - last_planar_applied_).toSec() < 5.0 &&
            std::hypot(r.x - last_planar_x_, r.y - last_planar_y_) < 0.15) {
            ++planar_apply_repeats_;
            if (planar_apply_repeats_ > 3) {
                planar_pending_ = false;
                std::cout << "[planar] same correction repeated " << planar_apply_repeats_
                          << "x -> stop (core not accepting, wait for 3D)" << std::endl;
                return;
            }
        } else {
            planar_apply_repeats_ = 0;
        }
        geometry_msgs::PoseWithCovarianceStamped fix;
        fix.header.stamp = now;
        fix.header.frame_id = kMapFrame;
        fix.pose.pose.position.x = r.x;
        fix.pose.pose.position.y = r.y;
        fix.pose.pose.position.z = cp2.z;
        const double half = 0.5 * r.yaw;
        fix.pose.pose.orientation.z = std::sin(half);
        fix.pose.pose.orientation.w = std::cos(half);
        // 走完整重定位路径: 只调 applyInitPose 不够 —— 车轮里程计监视器的参考
        // 还停在旧位姿上, 核心下一帧就会把位姿推回去 (实测会变成反复纠偏).
        xzrobot_ros_dependency::Relocation::Request req;
        xzrobot_ros_dependency::Relocation::Response res;
        req.map_name = curr_map_name_;
        req.initial_pose = fix;
        // 2D 平面约束是"把位姿拉回十几厘米"的连续纠偏, 不是重定位:
        // 不能让它冒充实位姿跳变, 否则 nav 会在执行中反复清计划/停车 (实测窄通道必挂).
        applyRelocation(req, res, /*count_as_jump=*/false);
        planar_pending_ = false;
        last_planar_applied_ = now;
        last_planar_x_ = r.x;
        last_planar_y_ = r.y;
        std::cout << "[planar] 2D constraint applied: dx=" << dx << " dy=" << dy
                  << " dyaw=" << dyaw << " inlier=" << r.inlier_ratio
                  << " resid=" << r.mean_residual << " cond=" << r.condition << std::endl;
    } else {
        planar_pending_ = true;
        planar_px_ = r.x;
        planar_py_ = r.y;
        planar_pyaw_ = r.yaw;
        planar_pending_stamp_ = now;
        std::cout << "[planar] candidate: dx=" << dx << " dy=" << dy
                  << " dyaw=" << dyaw << " (awaiting agreement)" << std::endl;
    }
}

} // namespace rosiwit_slam_ros1
