#include <gtest/gtest.h>
#include <cstdio>
#include "algorithms/pcd_map_manager/pcd_map_manager.h"

using namespace rosiwit_slam;

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
    kf.cloud->push_back(PointType(1.0, 0.0, 0.0));
    kf.cloud->push_back(PointType(0.0, 1.0, 0.0));
    kf.pose.trans = V3D::Zero();

    EXPECT_TRUE(mgr.addSubMap(kf));

    PointVec out;
    EXPECT_TRUE(mgr.getGlobalMap(out));
    EXPECT_GT(out.size(), 0);
}

TEST(PcdMapManagerTest, AddSubMapEmptyCloudReturnsFalse) {
    PcdMapManager mgr;
    mgr.init("nonexistent.yaml");

    KeyFrame kf;
    kf.cloud.reset(new CloudType());
    kf.pose.trans = V3D::Zero();
    EXPECT_FALSE(mgr.addSubMap(kf));
}
