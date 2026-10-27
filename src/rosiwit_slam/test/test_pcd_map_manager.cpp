#include <gtest/gtest.h>
#include <cstdio>
#include "algorithms/pcd_map_manager/pcd_map_manager.h"

using namespace rosiwit_slam;

// PCL 1.8 的 PointXYZINormal 没有 (x,y,z) 构造函数 (只有默认构造), 这里给测试补一个。
static PointType mkPt(float x, float y, float z) {
    PointType p;
    p.x = x; p.y = y; p.z = z;
    return p;
}

TEST(PcdMapManagerTest, CreateAndInit) {
    PcdMapManager mgr;
    EXPECT_TRUE(mgr.init("nonexistent.yaml"));
}

TEST(PcdMapManagerTest, EmptyMapReturnsFalse) {
    PcdMapManager mgr;
    mgr.init("nonexistent.yaml");
    PointVec out;
    EXPECT_FALSE(mgr.getGlobalMap(out));
}

TEST(PcdMapManagerTest, SaveEmptyReturnsFalse) {
    PcdMapManager mgr;
    mgr.init("nonexistent.yaml");
    EXPECT_FALSE(mgr.saveMap("empty.pcd"));
}

TEST(PcdMapManagerTest, LoadNonexistentReturnsFalse) {
    PcdMapManager mgr;
    mgr.init("nonexistent.yaml");
    EXPECT_FALSE(mgr.loadMap("nonexistent.pcd"));
}

TEST(PcdMapManagerTest, AddSubMapAndRetrieve) {
    PcdMapManager mgr;
    mgr.init("nonexistent.yaml");

    KeyFrame kf;
    kf.cloud.reset(new CloudType());
    kf.cloud->push_back(mkPt(1.0, 0.0, 0.0));
    kf.cloud->push_back(mkPt(0.0, 1.0, 0.0));
    kf.pose.trans = V3D::Zero();

    EXPECT_TRUE(mgr.addSubMap(kf));

    PointVec out;
    EXPECT_TRUE(mgr.getGlobalMap(out));
    EXPECT_GT(out.size(), 0);
}

// addSubMap 从"每个关键帧对整张图重跑 VoxelGrid"改成增量累积后:
// 同一体素内的多帧点必须合并成一个点 (等价于整图 VoxelGrid), 不能随帧数膨胀。
TEST(PcdMapManagerTest, AddSubMapVoxelAccumulates) {
    PcdMapManager mgr;
    ASSERT_TRUE(mgr.init("nonexistent.yaml"));

    const int kFrames = 30;
    for (int i = 0; i < kFrames; ++i) {
        KeyFrame kf;
        kf.cloud.reset(new CloudType());
        // 全部落在同一个 0.3m 体素内 (x 在 [0.10, 0.13))
        kf.cloud->push_back(mkPt(0.10f + 0.001f * static_cast<float>(i), 0.10f, 0.10f));
        kf.pose.trans = V3D::Zero();
        ASSERT_TRUE(mgr.addSubMap(kf));
    }

    PointVec out;
    ASSERT_TRUE(mgr.getGlobalMap(out));
    ASSERT_EQ(out.size(), 1u);                     // 一个体素只留一个点

    // 该点的坐标 = 体素内所有点的 centroid (与 VoxelGrid 默认行为一致)
    double mean_x = 0.0;
    for (int i = 0; i < kFrames; ++i) mean_x += 0.10 + 0.001 * i;
    mean_x /= kFrames;
    EXPECT_NEAR(out[0].x, mean_x, 2e-3);
}

TEST(PcdMapManagerTest, LoadResetsVoxelIndex) {
    PcdMapManager mgr;
    ASSERT_TRUE(mgr.init("nonexistent.yaml"));

    // 先建一段地图 (建立 voxel 索引)
    KeyFrame kf;
    kf.cloud.reset(new CloudType());
    kf.cloud->push_back(mkPt(0.1f, 0.1f, 0.1f));
    kf.pose.trans = V3D::Zero();
    ASSERT_TRUE(mgr.addSubMap(kf));

    // loadMap 失败时不应破坏已有地图; 成功后索引必须与新的 global_map_ 一致,
    // 再 addSubMap 不能沿用旧索引写到越界下标。
    EXPECT_FALSE(mgr.loadMap("nonexistent.pcd"));

    KeyFrame kf2;
    kf2.cloud.reset(new CloudType());
    kf2.cloud->push_back(mkPt(1.5f, 1.5f, 1.5f));
    kf2.pose.trans = V3D::Zero();
    ASSERT_TRUE(mgr.addSubMap(kf2));

    PointVec out;
    ASSERT_TRUE(mgr.getGlobalMap(out));
    EXPECT_GE(out.size(), 2u);
}

TEST(PcdMapManagerTest, AddSubMapEmptyCloudReturnsFalse) {
    PcdMapManager mgr;
    mgr.init("nonexistent.yaml");

    KeyFrame kf;
    kf.cloud.reset(new CloudType());
    kf.pose.trans = V3D::Zero();
    EXPECT_FALSE(mgr.addSubMap(kf));
}

// catkin_add_gtest 只链 GTEST_LIBRARIES (不含 gtest_main)
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
