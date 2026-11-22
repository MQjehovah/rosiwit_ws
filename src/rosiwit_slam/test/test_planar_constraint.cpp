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

TEST(ScanPlaneConstraint, PullsZToMapGround) {
    PlanarConstraint pc;
    pc.configure(cfg("scan_plane"));

    // 地图地面 z=0 (1m 格)
    CloudType::Ptr map(new CloudType());
    for (double x = -3.0; x <= 3.0; x += 0.1)
        for (double y = -3.0; y <= 3.0; y += 0.1) {
            PointType p;
            p.x = static_cast<float>(x);
            p.y = static_cast<float>(y);
            p.z = 0.0f;
            map->push_back(p);
        }
    map->width = static_cast<uint32_t>(map->size());
    map->height = 1;
    pc.setMap(map);

    // 扫描: 地板在 -0.8 (传感器离地 0.8), 天花板在 +1.5
    CloudType::Ptr scan(new CloudType());
    for (double x = -2.0; x <= 2.0; x += 0.1)
        for (double y = -2.0; y <= 2.0; y += 0.1) {
            PointType pf;
            pf.x = static_cast<float>(x);
            pf.y = static_cast<float>(y);
            pf.z = -0.8f;
            scan->push_back(pf);
            PointType pc2;
            pc2.x = static_cast<float>(x);
            pc2.y = static_cast<float>(y);
            pc2.z = 1.5f;
            scan->push_back(pc2);
        }
    scan->width = static_cast<uint32_t>(scan->size());
    scan->height = 1;

    PoseStamped pose;
    pose.trans = V3D(0.0, 0.0, -1.5);   // 掉到地下
    pose.rot = M3D::Identity();
    for (int i = 0; i < 12; ++i) pc.apply(pose, scan);

    EXPECT_NEAR(pose.trans.z(), 0.8, 0.12) << "z should converge to map ground + sensor height";
}

// 实机场景: 位姿 z 掉到地板以下时, 扫描里的"地板"位于传感器上方(+0.8)。
// 这种情况下"传感器离地高"是负数, 用 gz+h 会算错 —— 必须用"地板平面 == 地图地面"
// 的判据, 收敛结果应让点云地板落在 z=0, 即 pose.z = -0.8。
TEST(ScanPlaneConstraint, PullsZWhenFloorIsAboveSensor) {
    PlanarConstraint pc;
    pc.configure(cfg("scan_plane"));

    CloudType::Ptr map(new CloudType());
    for (double x = -3.0; x <= 3.0; x += 0.1)
        for (double y = -3.0; y <= 3.0; y += 0.1) {
            PointType p;
            p.x = static_cast<float>(x);
            p.y = static_cast<float>(y);
            p.z = 0.0f;
            map->push_back(p);
        }
    map->width = static_cast<uint32_t>(map->size());
    map->height = 1;
    pc.setMap(map);

    CloudType::Ptr scan(new CloudType());
    for (double x = -2.0; x <= 2.0; x += 0.1)
        for (double y = -2.0; y <= 2.0; y += 0.1) {
            PointType pf;
            pf.x = static_cast<float>(x);
            pf.y = static_cast<float>(y);
            pf.z = 0.8f;      // 地板在传感器上方 (位姿过低)
            scan->push_back(pf);
            PointType pc2;
            pc2.x = static_cast<float>(x);
            pc2.y = static_cast<float>(y);
            pc2.z = 3.0f;
            scan->push_back(pc2);
        }
    scan->width = static_cast<uint32_t>(scan->size());
    scan->height = 1;

    PoseStamped pose;
    pose.trans = V3D(0.0, 0.0, -1.5);
    pose.rot = M3D::Identity();
    for (int i = 0; i < 12; ++i) pc.apply(pose, scan);

    // 点云地板 F = pose.z + 0.8 应落到地图地面 0
    EXPECT_NEAR(pose.trans.z() + 0.8, 0.0, 0.12) << "scan floor should land on map ground";
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
