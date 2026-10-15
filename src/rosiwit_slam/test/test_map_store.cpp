// test/test_map_store.cpp
// P1/P2 契约测试: 各 IMapStore 实现应给出等价的最近邻/半径查询/统计.
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include "algorithms/map_store/map_store_factory.h"

using namespace rosiwit_slam;

namespace {
CloudType::Ptr makeCloud() {
    CloudType::Ptr c(new CloudType());
    for (double x = 0; x <= 8; x += 0.08)
        for (double y = 0; y <= 8; y += 0.08)
            c->push_back(PointType(static_cast<float>(x), static_cast<float>(y), 0.0f));
    for (double z = 0; z <= 2; z += 0.05)
        for (double t = 0; t <= 8; t += 0.05) {
            c->push_back(PointType(static_cast<float>(t), 0.0f, static_cast<float>(z)));
            c->push_back(PointType(static_cast<float>(t), 8.0f, static_cast<float>(z)));
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

INSTANTIATE_TEST_SUITE_P(AllStores, MapStoreContractTest,
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
    for (double x = 0; x < 1; x += 0.1) add->push_back(PointType(static_cast<float>(x), 20.0f, 0.5f));
    const uint64_t r0 = store->revision();
    store->insert(add);
    EXPECT_GT(store->size(), before);
    EXPECT_GT(store->revision(), r0);

    store->erase(add);
    EXPECT_LT(store->size(), before + add->size());
}
