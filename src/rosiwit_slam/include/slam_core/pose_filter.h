// include/slam_core/pose_filter.h
// 位姿输出滤波 (L12 方案三): alpha-beta / 常速 (constant-velocity) 因果滤波.
// 用于把高频输出流(IMU 传播)做平滑低延迟处理, 替代简单 EMA(引入滞后).
// 与"固定滞后平滑器(L4)"区别: 本类是因果递归滤波(只用过去+当前), 低延迟; L4 是滑窗优化(更准, 用于扫描帧锚点).
#pragma once
#include <Eigen/Eigen>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

class PoseOutputFilter {
public:
    void configure(double alpha, double beta, double max_extrap_s) {
        m_alpha = alpha; m_beta = beta; m_max_extrap = max_extrap_s;
    }
    void reset(const PoseStamped& p) {
        m_init = false;
        m_pos = p.trans;
        m_q = Eigen::Quaterniond(p.rot).normalized();
        m_vel.setZero();
        m_w.setZero();
        m_t = p.time;
        m_init = true;
    }
    bool initialized() const { return m_init; }

    /// 测量更新 (通常在每个扫描帧/校正帧)
    void update(double t, const PoseStamped& meas);
    /// 取 t 时刻平滑位姿 (超末次更新则按速度外推, 封顶 max_extrap)
    bool poseAt(double t, PoseStamped& out) const;

private:
    double m_alpha = 0.5, m_beta = 0.2, m_max_extrap = 0.3;
    bool m_init = false;
    double m_t = -1.0;
    V3D m_pos = V3D::Zero();
    V3D m_vel = V3D::Zero();
    Eigen::Quaterniond m_q = Eigen::Quaterniond::Identity();
    V3D m_w = V3D::Zero();   // 世界系角速度(轴*角/时间)
};

} // namespace rosiwit_slam
