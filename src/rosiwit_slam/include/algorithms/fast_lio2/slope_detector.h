#pragma once
// 坡度判定: 双阈值迟滞, 避免单帧噪声把 z 锁反复放掉/收回.
//
// 输入优先级:
//   1) 点云地面法向 (ground_ok) —— 唯一可靠来源;
//   2) 无地面时, IMU 倾角 < 10deg 才作为后备 (IMU 斜装/估计异常时会跑到 40~80deg);
//   3) 两者都不可用时保持当前状态, 不计数 —— 绝不因"没有证据"而解锁.
#include <cmath>

namespace rosiwit_slam {

struct SlopeGateConfig {
    double enter_deg = 10.0;   // 进坡阈值
    double exit_deg = 6.0;     // 回平地阈值 (< enter, 形成迟滞)
    int enter_frames = 5;      // 连续多少帧超阈值才进坡
    int exit_frames = 20;      // 连续多少帧低于阈值才回平地
};

class SlopeDetector {
public:
    SlopeDetector() = default;
    explicit SlopeDetector(const SlopeGateConfig& cfg) : m_cfg(cfg) {}

    void setConfig(const SlopeGateConfig& cfg) { m_cfg = cfg; }
    const SlopeGateConfig& config() const { return m_cfg; }

    /// 返回更新后的 onSlope(). 角度单位: rad.
    bool update(bool ground_ok, double ground_tilt_rad, double imu_tilt_rad);

    bool onSlope() const { return m_on_slope; }
    double lastSlopeDeg() const { return m_last_deg; }
    void reset() { m_on_slope = false; m_up = 0; m_down = 0; m_last_deg = 0.0; }

private:
    SlopeGateConfig m_cfg;
    bool m_on_slope = false;
    int m_up = 0;
    int m_down = 0;
    double m_last_deg = 0.0;
};

}  // namespace rosiwit_slam
