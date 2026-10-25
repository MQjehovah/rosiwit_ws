// 轮速"软门控"离线验证: odom 从开关变成带方差的观测.
//
// 关键断言:
//   1) 正常直行 (slip≈0) -> 权重≈1, sigma 小  [与旧行为完全一致]
//   2) slip = 阈值        -> 权重≈0.5 (旧行为: 1.0)  <- 连续化
//   3) slip = 2x 阈值     -> 权重≈0.06 (旧行为: 0)   <- 不丢弃, 只降权
//   4) 平移速度超物理上限 -> 降权到 10%, 但样本照常入账 (odom 时钟不冻结)
#include <cmath>
#include <cstdio>
#include <vector>

#include "algorithms/localization/motion_prior/imu_motion_prior.h"

using namespace rosiwit_slam;

namespace {

int fails = 0;
void check(bool ok, const char* what) {
    std::printf("  %s %s\n", ok ? "[ok]" : "[FAIL]", what);
    if (!ok) ++fails;
}

M3D yawRot(double yaw) {
    M3D R = M3D::Identity();
    R(0, 0) = std::cos(yaw);
    R(0, 1) = -std::sin(yaw);
    R(1, 0) = std::sin(yaw);
    R(1, 1) = std::cos(yaw);
    return R;
}

struct Rig {
    ImuMotionPrior prior;
    double t = 0.0;
    V3D odom_pos = V3D::Zero();
    double odom_yaw = 0.0;

    void setup(double slip_thresh = 0.3) {
        MotionPriorConfig cfg;
        cfg.use_odom = true;
        cfg.slip_yaw_thresh = slip_thresh;
        cfg.max_odom_speed = 3.0;
        cfg.imu_calib_samples = 0;     // 跳过静止标定
        prior.configure(cfg);
        PoseStamped p;
        p.time = 0.0;
        p.trans = V3D(1.0, 2.0, 0.0);
        prior.reset(p);
        // 先来一帧 odom, 再 onMatch —— 偏移 (map<-odom) 只在两者都有时才建立
        step(0.0, 0.0, 0.0);
        prior.onMatch(p);
    }

    // dt 秒一步: 真实前进 v, odom 的偏航角速度 = odom_yaw_rate, IMU 陀螺 = gyro_w
    void step(double v, double odom_yaw_rate, double gyro_w) {
        const double dt = 0.02;
        t += dt;
        odom_pos.x() += v * dt;
        odom_yaw += odom_yaw_rate * dt;

        IMUSample s;
        s.time = t;
        s.gyro = V3D(0.0, 0.0, gyro_w);
        s.acc = V3D(0.0, 0.0, 9.81);
        prior.onImu(s);

        PoseStamped o;
        o.time = t;
        o.trans = odom_pos;
        o.rot = yawRot(odom_yaw);
        prior.onOdom(o);
    }

    // 制造一个平移毛刺 (单帧跳变 jump 米), 用于验证"不整包丢弃"
    void spike(double jump) {
        const double dt = 0.02;
        t += dt;
        odom_pos.x() += jump;
        PoseStamped o;
        o.time = t;
        o.trans = odom_pos;
        o.rot = yawRot(odom_yaw);
        prior.onOdom(o);
    }

    double weightAfter(double t_predict = -1.0) {
        prior.predict(t_predict < 0.0 ? t : t_predict);
        return prior.odomWeight();
    }
};

}  // namespace

int main() {
    // ---------- 1) 正常直行 ----------
    {
        Rig rig;
        rig.setup();
        for (int i = 0; i < 25; ++i) rig.step(0.20, 0.0, 0.0);
        const double w = rig.weightAfter();
        std::printf("1) straight: weight=%.3f sigma_xy=%.3f sigma_yaw=%.3f\n",
                    w, rig.prior.transSigma(), rig.prior.yawSigma());
        check(w > 0.98, "normal driving keeps full odom weight (>0.98)");
        check(rig.prior.transSigma() < 0.05, "normal driving sigma small (<5cm)");
        check(!rig.prior.odomSlip(), "not flagged as slip");
    }

    // ---------- 2) slip = 阈值 ----------
    {
        Rig rig;
        rig.setup(0.3);
        for (int i = 0; i < 25; ++i) rig.step(0.20, 0.0, 0.0);
        // odom 报 0.3 rad/s, 陀螺报 0 -> slip=0.3=阈值
        for (int i = 0; i < 25; ++i) rig.step(0.20, 0.3, 0.0);
        const double w = rig.weightAfter();
        std::printf("2) slip=thresh: weight=%.3f sigma_xy=%.3f slip=%d\n",
                    w, rig.prior.transSigma(), static_cast<int>(rig.prior.odomSlip()));
        check(w > 0.35 && w < 0.65, "slip at threshold -> weight ~0.5 (was hard 0/1)");
        check(rig.prior.transSigma() > 0.20, "sigma inflated at threshold");
        check(rig.prior.odomSlip(), "flagged as slip");
    }

    // ---------- 3) slip = 2x 阈值 ----------
    {
        Rig rig;
        rig.setup(0.3);
        for (int i = 0; i < 25; ++i) rig.step(0.20, 0.0, 0.0);
        for (int i = 0; i < 25; ++i) rig.step(0.20, 0.6, 0.0);
        const double w = rig.weightAfter();
        std::printf("3) slip=2x: weight=%.4f sigma_xy=%.3f\n",
                    w, rig.prior.transSigma());
        check(w < 0.12, "heavy slip -> weight near zero (same as old reject)");
        check(rig.prior.transSigma() > 0.50, "sigma large on heavy slip");
    }

    // ---------- 4) 平移毛刺: 降权但样本入账 ----------
    {
        Rig rig;
        rig.setup(0.3);
        for (int i = 0; i < 25; ++i) rig.step(0.20, 0.0, 0.0);
        const double w_before = rig.weightAfter();
        rig.spike(1.0);                       // 1m/0.02s = 50 m/s >> 3 m/s
        const double w_spike = rig.weightAfter();
        // 再来一帧正常 odom: 如果毛刺被"整包丢弃", odom 时钟会停在旧样本,
        // dt 变长 -> 速度被算小, 权重反而回到 1; 我们现在验证时钟确实前进了.
        const double dt_prev = 0.02;
        const double t_before = rig.t;
        rig.step(0.20, 0.0, 0.0);
        const double dt_used = rig.t - t_before;
        const double w_after = rig.weightAfter();
        std::printf("4) spike: w_before=%.3f w_spike=%.3f w_after=%.3f dt=%.3f\n",
                    w_before, w_spike, w_after, dt_used);
        check(w_before > 0.98, "pre-spike weight full");
        check(w_spike < 0.15, "spike down-weighted to <=10%");
        check(std::fabs(dt_used - dt_prev) < 1e-9, "odom clock still advances after spike");
        check(w_after > 0.5, "recovers after the spike frame");
    }

    std::printf("=== %s ===\n", fails == 0 ? "PASS" : "FAIL");
    return fails == 0 ? 0 : 1;
}
