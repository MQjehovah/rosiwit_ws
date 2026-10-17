// 退化点云守卫回归测试
// 背景: 2026-10-17 20:06 实机节点死亡 —— 跟踪/恢复路径把一个降采样后仅 16 点的扫描
// 喂给 PCL GICP, computeCovariances 报 "points (16) < k_correspondences_ (20)"
// 后进程直接消失(无 core dump)。matcher 必须在进入 GICP 之前拒绝退化输入。
#include <gtest/gtest.h>
#include "algorithms/localization/matcher/gicp_matcher.h"

using namespace rosiwit_slam;

namespace {
CloudType::Ptr makeLayer(double step, int nx, int ny, double z) {
    CloudType::Ptr c(new CloudType());
    c->reserve(nx * ny);
    for (int i = 0; i < nx; ++i)
        for (int j = 0; j < ny; ++j) {
            PointType p;   // PCL 1.8 无 (x,y,z) 三参构造, 逐字段赋值
            p.x = i * step;
            p.y = j * step;
            p.z = z;
            c->push_back(p);
        }
    c->width = static_cast<uint32_t>(c->size());
    c->height = 1;
    return c;
}
// 地板+天花板两层, 避免纯平面退化
CloudType::Ptr makeRoom(double step, int nx, int ny) {
    CloudType::Ptr c = makeLayer(step, nx, ny, 0.0);
    CloudType::Ptr top = makeLayer(step, nx, ny, 2.0);
    *c += *top;
    c->width = static_cast<uint32_t>(c->size());
    c->height = 1;
    return c;
}
PoseStamped identityPose(double dx = 0.0) {
    PoseStamped p;
    p.rot = Eigen::Matrix3d::Identity();
    p.trans = V3D(dx, 0.0, 0.0);
    return p;
}
ScanMatcherConfig fastCfg() {
    ScanMatcherConfig cfg;
    cfg.voxel_resolution = 0.05;
    cfg.max_iterations = 20;
    cfg.max_correspondence_dist = 2.0;
    return cfg;
}
} // namespace

// 崩溃现场复刻: 16 点扫描(4x4)进 GICP 必须被拒绝 (guard 不存在时 PCL 打印
// k_correspondences 警告且可能崩掉测试进程)
TEST(GicpMatcherGuard, TinyScanRejected) {
    GicpMatcher m;
    m.configure(fastCfg());
    m.setTarget(makeRoom(0.5, 30, 30));
    MatchResult r;
    EXPECT_FALSE(m.match(makeLayer(0.5, 4, 4, 0.0), identityPose(), r));
}

// 16 点目标同样必须拒绝
TEST(GicpMatcherGuard, TinyTargetRejected) {
    GicpMatcher m;
    m.configure(fastCfg());
    m.setTarget(makeLayer(0.5, 4, 4, 0.0));
    MatchResult r;
    EXPECT_FALSE(m.match(makeRoom(0.5, 30, 30), identityPose(), r));
}

// 守卫不得误伤正常规模点云
TEST(GicpMatcherGuard, NormalCloudsStillMatch) {
    GicpMatcher m;
    m.configure(fastCfg());
    m.setTarget(makeRoom(0.5, 30, 30));
    MatchResult r;
    // 同源点云(略小平移)必须能配准成功
    EXPECT_TRUE(m.match(makeRoom(0.5, 28, 28), identityPose(0.15), r));
    EXPECT_TRUE(r.converged);
}

// catkin_add_gtest 只链 GTEST_LIBRARIES (不含 gtest_main), 所以这里自带 main
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
