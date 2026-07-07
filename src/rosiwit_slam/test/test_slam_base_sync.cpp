// test/test_slam_base_sync.cpp
#include <gtest/gtest.h>
#include "slam_core/slam_base.h"

using namespace rosiwit_slam;

namespace {
// 最小桩:记录被同步的包
class CountingAlgo : public SlamBase {
public:
    int synced_count = 0;
    double last_end_time = -1.0;
    bool init(const std::string&) override { return true; }      // 桩:使类可实例化
    std::string name() const override { return "counting"; }     // 桩:使类可实例化
protected:
    bool onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) override {
        ++synced_count;
        last_end_time = pkg.frame.end_time;
        out.has_new_pose = true;
        return true;
    }
};
}

TEST(SlamBaseSync, NoDataNoSync) {
    CountingAlgo a;
    EXPECT_EQ(a.synced_count, 0);
    EXPECT_FALSE(a.tryPopAndProcess());   // 无数据
}

TEST(SlamBaseSync, SyncsWhenImuCoversLidarSpan) {
    CountingAlgo a;
    // lidar 帧 [10.0, 10.1]
    LidarFrame f; f.start_time = 10.0; f.end_time = 10.1;
    f.cloud.reset(new CloudType);
    f.cloud->push_back(PointType{});
    a.onLidar(f);
    EXPECT_EQ(a.synced_count, 0);         // 还没有覆盖到 end_time 的 IMU

    // IMU 序列覆盖 [9.9 .. 10.1]
    for (int i = 0; i <= 5; ++i) {
        IMUSample s; s.time = 9.9 + i * 0.04; a.onImu(s);
    }
    a.tryPopAndProcess();                 // fix(b): 显式驱动同步
    EXPECT_GE(a.synced_count, 1);
    EXPECT_NEAR(a.last_end_time, 10.1, 1e-9);
}

TEST(SlamBaseSync, ForwardsOutputViaCallback) {
    CountingAlgo a;
    SlamOutput captured;
    bool got = false;
    a.setOutputCallback([&](const SlamOutput& o){ captured = o; got = true; });

    LidarFrame f; f.start_time = 1.0; f.end_time = 1.05;
    f.cloud.reset(new CloudType); f.cloud->push_back(PointType{});
    a.onLidar(f);
    for (int i = 0; i < 4; ++i) { IMUSample s; s.time = 1.0 + i*0.02; a.onImu(s); }
    a.tryPopAndProcess();                 // fix(b): 显式驱动同步
    EXPECT_TRUE(got);
    EXPECT_TRUE(captured.has_new_pose);
}
