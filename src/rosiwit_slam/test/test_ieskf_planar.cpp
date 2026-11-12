// IESKF::updatePlanar: 平地平面约束只锁 z / roll / pitch, 不动 yaw.
//
// 断言:
//   1) 调用后 t_wi.z == z_ref, v.z == 0
//   2) lock_roll_pitch=true: roll/pitch 对齐 r_flat, yaw 保持
//   3) lock_roll_pitch=false: 姿态完全不变
//   4) P(5,5) 被收紧到 <= 1e-3
#include <cmath>
#include <cstdio>

#include <Eigen/Eigen>
#include <sophus/so3.hpp>

#include "algorithms/fast_lio2/map_builder/ieskf.h"

namespace {
int fails = 0;
void check(bool ok, const char* what) {
    std::printf("  %s %s\n", ok ? "[ok]" : "[FAIL]", what);
    if (!ok) ++fails;
}
double yawOf(const M3D& R) { return std::atan2(R(1, 0), R(0, 0)); }
}

int main() {
    IESKF kf;
    kf.P().setIdentity();
    State& s = kf.x();
    s.t_wi = V3D(10.0, -3.0, 5.0);
    s.v = V3D(0.5, 0.2, -0.7);
    // 当前姿态: yaw=30deg, 另有 5deg roll
    const M3D R_cur = Eigen::AngleAxisd(30.0 * M_PI / 180.0, V3D::UnitZ()).toRotationMatrix() *
                      Eigen::AngleAxisd(5.0 * M_PI / 180.0, V3D::UnitX()).toRotationMatrix();
    s.r_wi = R_cur;
    const M3D r_flat = M3D::Identity();   // 平地参考: 无 roll/pitch

    kf.updatePlanar(0.07, r_flat, true);
    check(std::fabs(kf.x().t_wi.z() - 0.07) < 1e-12, "z 被锁到 z_ref");
    check(std::fabs(kf.x().v.z()) < 1e-12, "v.z 清零");
    check(std::fabs(yawOf(kf.x().r_wi) - 30.0 * M_PI / 180.0) < 1e-9, "yaw 保持不变");
    const M3D dR = r_flat.transpose() * kf.x().r_wi;
    check(std::fabs(yawOf(dR)) > 0.1, "确实保留了非零 yaw 差(仅测试自证)");
    const V3D rp = Sophus::SO3d(dR * Eigen::AngleAxisd(-yawOf(dR), V3D::UnitZ()).toRotationMatrix()).log();
    check(rp.norm() < 1e-6, "roll/pitch 对齐 r_flat");
    check(kf.P()(5, 5) <= 1e-3, "P(5,5) 收紧");

    // lock_roll_pitch = false: 姿态不动
    IESKF kf2;
    kf2.P().setIdentity();
    kf2.x().r_wi = R_cur;
    kf2.x().t_wi = V3D(0, 0, 2.0);
    kf2.updatePlanar(1.0, r_flat, false);
    check((kf2.x().r_wi - R_cur).norm() < 1e-12, "lock_roll_pitch=false 时姿态不变");
    check(std::fabs(kf2.x().t_wi.z() - 1.0) < 1e-12, "lock_roll_pitch=false 时 z 仍锁");

    std::printf(fails == 0 ? "ALL PASS\n" : "FAILED %d\n", fails);
    return fails == 0 ? 0 : 1;
}
