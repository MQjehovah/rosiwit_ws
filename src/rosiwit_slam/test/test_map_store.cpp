// test/test_map_store.cpp
// P1/P2 契约测试: 各 IMapStore 实现应给出等价的最近邻/半径查询/统计.
#include <gtest/gtest.h>
#include <cstdio>
#include <memory>
#include <cmath>
#include "algorithms/map_store/map_store_factory.h"
#include "algorithms/map_store/voxel_map_store.h"

using namespace rosiwit_slam;

// PCL 1.8 的 PointXYZINormal 没有 (x,y,z) 构造函数 (只有默认构造), 这里给测试补一个。
static PointType mkPt(float x, float y, float z) {
    PointType p;
    p.x = x; p.y = y; p.z = z;
    return p;
}

namespace {
CloudType::Ptr makeCloud() {
    CloudType::Ptr c(new CloudType());
    for (double x = 0; x <= 8; x += 0.08)
        for (double y = 0; y <= 8; y += 0.08)
            c->push_back(mkPt(static_cast<float>(x), static_cast<float>(y), 0.0f));
    for (double z = 0; z <= 2; z += 0.05)
        for (double t = 0; t <= 8; t += 0.05) {
            c->push_back(mkPt(static_cast<float>(t), 0.0f, static_cast<float>(z)));
            c->push_back(mkPt(static_cast<float>(t), 8.0f, static_cast<float>(z)));
        }
    return c;
}
} // namespace

// ---- 契约: 各存储实现行为一致 ----
class MapStoreContractTest : public ::testing::TestWithParam<std::string> {};

TEST_P(MapStoreContractTest, BuildAndQuery) {
    auto store = MapStoreFactory::create(GetParam(), 0.2);
    ASSERT_NE(store, nullptr);
    ASSERT_TRUE(store->buildFromCloud(makeCloud()));
    EXPECT_GT(store->size(), 0u);

    // 半径查询
    auto sub = store->queryRadius(Eigen::Vector3d(3, 3, 0), 2.0);
    EXPECT_GT(sub->size(), 0u);
    for (const auto& p : sub->points) {
        const double d2 = (p.x - 3) * (p.x - 3) + (p.y - 3) * (p.y - 3) + p.z * p.z;
        EXPECT_LE(d2, 2.0 * 2.0 + 1e-6);
    }

    // 最近邻
    PointType q; q.x = 3.0f; q.y = 3.0f; q.z = 0.0f;
    std::vector<PointType> pts;
    std::vector<float> sq;
    const int n = store->nearestK(q, 5, pts, sq);
    EXPECT_EQ(n, 5);
    for (int i = 1; i < n; ++i) EXPECT_LE(sq[i - 1], sq[i] + 1e-9f);

    // 统计
    VoxelStats st = store->statsAt(Eigen::Vector3d(3, 3, 0), 0.5);
    EXPECT_TRUE(st.valid);
    EXPECT_GT(st.n, 0);
}

INSTANTIATE_TEST_CASE_P(AllStores, MapStoreContractTest,
                         ::testing::ValuesIn(MapStoreFactory::listNames()));

// ---- 等价: KD树存储 vs 体素存储 最近邻距离一致 ----
TEST(MapStoreEquivalenceTest, NearestDistanceMatches) {
    CloudType::Ptr c = makeCloud();
    auto pcd = MapStoreFactory::create("pcd", 0.2);
    auto vox = MapStoreFactory::create("voxel", 0.2);
    ASSERT_TRUE(pcd->buildFromCloud(c));
    ASSERT_TRUE(vox->buildFromCloud(c));

    double worst = 0.0;
    for (double x = 0.5; x <= 7.5; x += 0.7)
        for (double y = 0.5; y <= 7.5; y += 0.7) {
            PointType q; q.x = static_cast<float>(x); q.y = static_cast<float>(y); q.z = 0.0f;
            const double dp = pcd->nearestDist2(q);
            const double dv = vox->nearestDist2(q);
            worst = std::max(worst, std::fabs(std::sqrt(dp) - std::sqrt(dv)));
        }
    EXPECT_LT(worst, 1e-3);   // 最近邻距离等价 (体素扩展应精确)
}

TEST(MapStoreTransform, InsertErase) {
    auto store = MapStoreFactory::create("voxel", 0.2);
    ASSERT_NE(store, nullptr);
    store->buildFromCloud(makeCloud());
    const size_t before = store->size();

    CloudType::Ptr add(new CloudType());
    for (double x = 0; x < 1; x += 0.1) add->push_back(mkPt(static_cast<float>(x), 20.0f, 0.5f));
    const uint64_t r0 = store->revision();
    store->insert(add);
    EXPECT_GT(store->size(), before);
    EXPECT_GT(store->revision(), r0);

    store->erase(add);
    EXPECT_LT(store->size(), before + add->size());
}

// ---- P5: .vxmap 持久化往返 ----
TEST(MapStorePersist, VxMapRoundTrip) {
    auto s = MapStoreFactory::create("voxel", 0.2);
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->buildFromCloud(makeCloud()));
    const size_t n0 = s->size();
    // gtest 1.8 没有 ::testing::TempDir()
    const std::string path = "/tmp/rosiwit_test_map.vxmap";
    ASSERT_TRUE(s->save(path));

    auto s2 = MapStoreFactory::create("voxel", 0.5);   // 不同 leaf, load 应以文件为准
    ASSERT_TRUE(s2->load(path));
    EXPECT_EQ(s2->size(), n0);

    PointType q; q.x = 3.0f; q.y = 3.0f; q.z = 0.0f;
    EXPECT_NEAR(s->nearestDist2(q), s2->nearestDist2(q), 1e-6);
}

// ---- P5: groundAt ----
TEST(MapStoreGround, GroundAtFloor) {
    auto s = MapStoreFactory::create("voxel", 0.2);
    ASSERT_NE(s, nullptr);
    ASSERT_TRUE(s->buildFromCloud(makeCloud()));
    double z = 99.0;
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    ASSERT_TRUE(s->groundAt(3.0, 3.0, z, n));
    EXPECT_NEAR(z, 0.0, 0.05);
    EXPECT_GT(n.z(), 0.9);
}

// ---- 列索引一致性: erase 之后 groundAt 不能再看到已删除的体素 ----
// groundAt 从"遍历全部体素"改成"列索引直查"后, 这里守住索引与 m_cells 的同步。
TEST(MapStoreGround, EraseDropsColumnEntries) {
    VoxelMapStore s(0.2);
    ASSERT_TRUE(s.buildFromCloud(makeCloud()));
    // 必须覆盖 groundAt 会查的整个 3x3 列窗口 (leaf 0.2 -> 列 [2.6,3.4)),
    // 只删正中间那一列时, 周围列的点仍满足">=5 点 + 平面"的判定。
    CloudType::Ptr patch(new CloudType());
    for (double x = 2.60; x <= 3.40; x += 0.02)
        for (double y = 2.60; y <= 3.40; y += 0.02)
            patch->push_back(mkPt(static_cast<float>(x), static_cast<float>(y), 0.0f));
    s.erase(patch);

    double z = 99.0;
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    EXPECT_FALSE(s.groundAt(3.0, 3.0, z, n));      // 该列已被删空
    EXPECT_TRUE(s.groundAt(6.0, 4.0, z, n));       // 其它位置不受影响
}

// ---- P5: decay 移除长期未更新的体素 ----
TEST(MapStoreDecay, RemovesStaleCells) {
    VoxelMapStore s(0.2);
    ASSERT_TRUE(s.buildFromCloud(makeCloud()));   // t=-1: 不参与衰减
    const size_t base = s.size();

    CloudType::Ptr add(new CloudType());
    add->push_back(mkPt(0.0f, 0.0f, 0.5f));
    s.insert(add, 100.0);
    const size_t after_add = s.size();
    EXPECT_GT(after_add, base);

    s.setDecayWindow(10.0);
    s.decay(200.0);                                // 200-100 > 10 -> 该体素移除
    EXPECT_LT(s.size(), after_add);
    EXPECT_GT(s.size(), 0u);                        // 未记时间的底图点仍在
}

// catkin_add_gtest 只链 GTEST_LIBRARIES (不含 gtest_main)
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
