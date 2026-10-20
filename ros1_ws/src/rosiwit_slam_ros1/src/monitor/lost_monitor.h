// ============================================================================
// 定位丢失状态机 —— 产出旧 SlamInfo.location_status 的 0/1/2 (设计文档 §5.11 B)
//
// 信号来源: ISlamAlgorithm::getPoseCov() 在核心 m_status == LOST 时返回 false
// (localization_pipeline.cpp:641), m_status 由真正的 MonitorLevel 驱动
// (localization_pipeline.cpp:620-629)。薄层不改核心。
//
// 必须处理的两个副作用:
//  1. **INIT 陷阱**: m_status == INIT(刚加载地图、还没定位上)时 getPoseCov 返回 **true**
//     并给出默认位姿。所以不能"返回 true 就算定位成功" —— 必须用"是否曾经真的定位上过"
//     的 latch 区分, 否则会把 INIT 误报成 SUCCESS(0)。这就是 ever_localized_ 存在的唯一理由,
//     也是 onLocalizedPose() 唯一的职责。
//  2. **DEGRADED 分辨不出**: 核心把 MonitorLevel::DEGRADED 也映射成 LOCALIZED
//     (localization_pipeline.cpp:627), 所以 1(INIT) 只用于"重定位进行中",
//     不用于"降级"(D11)。降级时我们照样报 0。
// ============================================================================
#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

namespace rosiwit_slam_ros1 {

/// 与 xzrobot_ros_dependency/msg/SlamInfo.msg 的常量一一对应
/// (int8 SUCCESS=0 / INIT=1 / FAILED=2)
enum class LocStatus : int8_t { SUCCESS = 0, INIT = 1, FAILED = 2 };

/// 纯逻辑状态机, 无线程、无 ROS 依赖 —— 便于单测。
/// **线程约定**: 所有方法必须在同一线程调用 (T11 里统放在 100Hz 的 pose 定时器里)。
class LostMonitor {
public:
    /// 进入定位/换图/重定位时调用: 回到"还没定位上"的初始态。
    /// 注意**不动** map_loaded_: 换图是 reset() 之后加载成功才 setMapLoaded(true),
    /// 而重定位根本不换地图 —— 若在这里把 map_loaded_ 清掉, 重定位后就再没人把它置回来了。
    void reset();

    /// 核心吐出一次新位姿且处于定位模式时调用 (T11: engine 输出回调)。
    void onLocalizedPose();

    /// 地图是否已加载 —— FAILED 的使能条件之一。
    void setMapLoaded(bool loaded);

    /// 每个 tick 调一次, 参数是 core.getPoseCov() 的返回值(即"当前是否还信得过")。
    LocStatus update(bool pose_cov_ok);

    /// 一次性边沿: 从"非 FAILED"翻到 FAILED 时返回一次 true, 之后为 false 直到再次丢失。
    /// (T11: 为真就把 20003504 加进待报列表)
    bool consumeLostEdge();

private:
    bool map_loaded_     = false;
    bool ever_localized_ = false;   // INIT 陷阱的 latch
    bool last_failed_    = false;   // 上一次 update 是否 FAILED, 用于边沿检测
    bool lost_edge_      = false;
};

/// 位姿-里程计一致性监测 (confidently-wrong 防线, 2026-10-17 推行丢失事故):
/// 激光定位锁到重复结构的错误位置时, 匹配分照样 0.98+, 状态机永远看不到"丢";
/// 但激光位姿的位移量与轮式里程计的位移量会长期背离 —— 这是少数可观测信号之一。
/// 判据: 10s 窗口 |Δslam| 与 |Δodom| 差 > max(0.8, 25%·|Δodom|), 连续 2 个窗口。
/// 轮子没转(|Δodom|<0.3m: 抱死推行/静止)时无法判定, 不计不报(防误报)。
class OdomConsistencyMonitor {
public:
    /// 每次有效定位位姿调用: t 秒, (sx,sy) 激光位姿, (ox,oy) 同时刻轮式里程计
    void onPose(double t, double sx, double sy, double ox, double oy) {
        if (!have_ref_) { anchor(t, sx, sy, ox, oy); return; }
        const double dt = t - t0_;
        if (dt < window_s_) return;                       // 窗口未满, 累计中
        const double ds = std::hypot(sx - sx0_, sy - sy0_);
        const double dodom = std::hypot(ox - ox0_, oy - oy0_);
        anchor(t, sx, sy, ox, oy);
        if (dodom < 0.3) { idle_windows_ = 0; return; }   // 轮子没转: 无法判定
        dev_ = std::fabs(ds - dodom);
        const double lim = std::max(0.8, 0.25 * dodom);
        bad_windows_ = (dev_ > lim) ? (bad_windows_ + 1) : 0;
        wrong_ = (bad_windows_ >= 2);
    }
    bool wrongness() const { return wrong_; }
    double deviation() const { return dev_; }
    void reset() { have_ref_ = false; bad_windows_ = 0; idle_windows_ = 0; wrong_ = false; dev_ = 0.0; }

private:
    void anchor(double t, double sx, double sy, double ox, double oy) {
        t0_ = t; sx0_ = sx; sy0_ = sy; ox0_ = ox; oy0_ = oy; have_ref_ = true;
    }
    double window_s_ = 10.0;
    bool have_ref_ = false;
    double t0_ = 0, sx0_ = 0, sy0_ = 0, ox0_ = 0, oy0_ = 0;
    int bad_windows_ = 0;
    int idle_windows_ = 0;
    double dev_ = 0.0;
    bool wrong_ = false;
};

} // namespace rosiwit_slam_ros1
