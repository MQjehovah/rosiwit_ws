// test/test_planar_constraint.cpp
// scan_plane: 从扫描里提水平面, 在 map 系把 roll/pitch 转回竖直 (不动 yaw, 不动安装角).
#include <gtest/gtest.h>

#include <cmath>

#include "algorithms/localization/constraints/planar_constraint.h"

using namespace rosiwit_slam;

namespace {

// 生成一个绕 y 轴倾斜 tilt_deg 的天花板平面 (z = h + tan(tilt)*x)
CloudType::Ptr mkTiltedCeiling(double h, double tilt_deg, double half = 2.0) {
    CloudType::Ptr c(new CloudType());
    const double k = std::tan(tilt_deg * M_PI / 180.0);
    for (double x = -half; x <= half; x += 0.1)
        for (double y = -half; y <= half; y += 0.1) {
            PointType p;
            p.x = static_cast<float>(x);
            p.y = static_cast<float>(y);
            p.z = static_cast<float>(h + k * x);
            c->push_back(p);
        }
    c->width = static_cast<uint32_t>(c->size());
    c->height = 1;
    return c;
}

PoseStamped mkPose(double yaw_deg = 0.0) {
    PoseStamped p;
    p.trans = V3D::Zero();
    p.rot = Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, V3D::UnitZ()).toRotationMatrix();
    return p;
}

PlanarConstraintConfig cfg(const std::string& mode) {
    PlanarConstraintConfig c;
    c.mode = mode;
    c.max_tilt_deg = 35.0;
    c.max_correction_deg = 15.0;
    return c;
}

double yawOf(const M3D& R) { return std::atan2(R(1, 0), R(0, 0)); }

}  // namespace

TEST(ScanPlaneConstraint, LevelsTiltedCeilingAndKeepsYaw) {
    PlanarConstraint pc;
    pc.configure(cfg("scan_plane"));
    PoseStamped pose = mkPose(30.0);
    const double yaw0 = yawOf(pose.rot);
    const double k = std::tan(10.0 * M_PI / 180.0);
    const Eigen::Vector3d n_body(-std::sin(10.0 * M_PI / 180.0), 0.0, std::cos(10.0 * M_PI / 180.0));
    (void)k;

    pc.apply(pose, mkTiltedCeiling(3.0, 10.0));

    const Eigen::Vector3d n_map = pose.rot * n_body;
    EXPECT_GT(n_map.z(), 0.999) << "ceiling normal should be vertical after correction";
    EXPECT_NEAR(yawOf(pose.rot), yaw0, 1e-6) << "yaw must not change";
}

TEST(ScanPlaneConstraint, LevelCeilingLeavesPoseAlone) {
    PlanarConstraint pc;
    pc.configure(cfg("scan_plane"));
    PoseStamped pose = mkPose(12.0);
    const M3D R0 = pose.rot;
    pc.apply(pose, mkTiltedCeiling(3.0, 0.0));
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) EXPECT_NEAR(pose.rot(r, c), R0(r, c), 1e-6);
}

TEST(ScanPlaneConstraint, SkipsCorrectionAboveLimit) {
    PlanarConstraint pc;
    pc.configure(cfg("scan_plane"));   // max_correction_deg = 15
    PoseStamped pose = mkPose(0.0);
    const M3D R0 = pose.rot;
    pc.apply(pose, mkTiltedCeiling(3.0, 40.0));   // 40 deg -> 视为跳变
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) EXPECT_NEAR(pose.rot(r, c), R0(r, c), 1e-6);
}

TEST(ScanPlaneConstraint, NoneModeDoesNothing) {
    PlanarConstraint pc;
    pc.configure(cfg("none"));
    PoseStamped pose = mkPose(0.0);
    const M3D R0 = pose.rot;
    pc.apply(pose, mkTiltedCeiling(3.0, 10.0));
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) EXPECT_NEAR(pose.rot(r, c), R0(r, c), 1e-6);
}

// catkin_add_gtest 只链 GTEST_LIBRARIES (不含 gtest_main), 测试须自带 main
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
