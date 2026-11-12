#include "algorithms/fast_lio2/slope_detector.h"

namespace rosiwit_slam {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
// IMU 倾角后备的有效上限: 超过它说明是安装角/估计异常, 不是坡度.
constexpr double kImuFallbackMaxDeg = 10.0;
}  // namespace

bool SlopeDetector::update(bool ground_ok, double ground_tilt_rad, double imu_tilt_rad) {
    double slope_rad = 0.0;
    bool have_evidence = false;
    if (ground_ok && std::isfinite(ground_tilt_rad)) {
        slope_rad = ground_tilt_rad;
        have_evidence = true;
    } else if (std::isfinite(imu_tilt_rad) &&
               imu_tilt_rad < kImuFallbackMaxDeg * kPi / 180.0) {
        slope_rad = imu_tilt_rad;
        have_evidence = true;
    }
    if (!have_evidence) return m_on_slope;  // 无证据: 保持, 不计数

    const double slope_deg = std::fabs(slope_rad) * kRadToDeg;
    m_last_deg = slope_deg;

    if (!m_on_slope) {
        if (slope_deg > m_cfg.enter_deg) {
            if (++m_up >= m_cfg.enter_frames) {
                m_on_slope = true;
                m_up = 0;
                m_down = 0;
            }
        } else {
            m_up = 0;
        }
    } else {
        if (slope_deg < m_cfg.exit_deg) {
            if (++m_down >= m_cfg.exit_frames) {
                m_on_slope = false;
                m_down = 0;
                m_up = 0;
            }
        } else {
            m_down = 0;
        }
    }
    return m_on_slope;
}

}  // namespace rosiwit_slam
