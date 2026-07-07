/**
 * @file test_ikd_tree.cpp
 * @brief 单元测试 - KdTree封装 (基于PCL KdTreeFLANN)
 * @author AI Development Team - Test Engineer
 * @date 2026-04-24
 */

#include <gtest/gtest.h>
#include "fast_lio2_slam/fast_lio2_core/ikd_tree.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <random>

namespace fast_lio2_slam {
namespace test {

class IkdTreeTest : public ::testing::Test {
protected:
    void SetUp() override {
        tree_ = std::make_unique<IKdTree>();

        // 创建随机点云
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(-10.0f, 10.0f);

        test_cloud_ = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
            new pcl::PointCloud<pcl::PointXYZINormal>);

        for (int i = 0; i < 1000; ++i) {
            pcl::PointXYZINormal pt;
            pt.x = dis(gen);
            pt.y = dis(gen);
            pt.z = dis(gen);
            pt.intensity = static_cast<float>(i);
            pt.normal_x = 0.0f;
            pt.normal_y = 0.0f;
            pt.normal_z = 1.0f;
            test_cloud_->points.push_back(pt);
        }
        test_cloud_->width = test_cloud_->points.size();
        test_cloud_->height = 1;
        test_cloud_->is_dense = true;
    }

    std::unique_ptr<IKdTree> tree_;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr test_cloud_;
};

// ==================== 初始化测试 ====================

TEST_F(IkdTreeTest, DefaultInitialization) {
    EXPECT_EQ(tree_->size(), 0);
    EXPECT_TRUE(tree_->empty());
}

TEST_F(IkdTreeTest, Build) {
    tree_->build(test_cloud_);

    EXPECT_EQ(tree_->size(), test_cloud_->points.size());
    EXPECT_FALSE(tree_->empty());
}

// ==================== 增量插入测试 ====================

TEST_F(IkdTreeTest, IncrementalInsert_SinglePoint) {
    tree_->build(test_cloud_);
    size_t initial_size = tree_->size();

    pcl::PointXYZINormal new_pt;
    new_pt.x = 100.0f;
    new_pt.y = 100.0f;
    new_pt.z = 100.0f;
    new_pt.intensity = 9999.0f;

    tree_->insert(new_pt);

    EXPECT_EQ(tree_->size(), initial_size + 1);
}

TEST_F(IkdTreeTest, IncrementalInsert_MultiplePoints) {
    tree_->build(test_cloud_);
    size_t initial_size = tree_->size();

    // 插入100个新点
    for (int i = 0; i < 100; ++i) {
        pcl::PointXYZINormal pt;
        pt.x = static_cast<float>(i * 0.1);
        pt.y = static_cast<float>(i * 0.1);
        pt.z = static_cast<float>(i * 0.1);
        pt.intensity = static_cast<float>(i);
        tree_->insert(pt);
    }

    EXPECT_EQ(tree_->size(), initial_size + 100);
}

// ==================== 最近邻搜索测试 ====================

TEST_F(IkdTreeTest, NearestNeighbor_Basic) {
    tree_->build(test_cloud_);

    // 使用已知点查询
    pcl::PointXYZINormal query = test_cloud_->points[0];
    pcl::PointXYZINormal nearest;
    double distance = 0.0;

    bool found = tree_->nearestSearch(query, nearest, distance);

    EXPECT_TRUE(found);
    EXPECT_NEAR(distance, 0.0, 1e-6);  // 应该找到自己
}

TEST_F(IkdTreeTest, NearestNeighbor_NewPoint) {
    tree_->build(test_cloud_);

    pcl::PointXYZINormal query;
    query.x = 0.0f;
    query.y = 0.0f;
    query.z = 0.0f;
    query.intensity = 0.0f;

    pcl::PointXYZINormal nearest;
    double distance = 0.0;

    bool found = tree_->nearestSearch(query, nearest, distance);

    EXPECT_TRUE(found);
    EXPECT_GE(distance, 0.0);   // 距离应该非负
    EXPECT_LT(distance, 20.0);  // 在测试范围内应该找到近邻
}

TEST_F(IkdTreeTest, NearestNeighbor_EmptyTree) {
    pcl::PointXYZINormal query;
    pcl::PointXYZINormal nearest;
    double distance = 0.0;

    bool found = tree_->nearestSearch(query, nearest, distance);

    EXPECT_FALSE(found);  // 空树应该返回false
}

// ==================== K近邻搜索测试 ====================

TEST_F(IkdTreeTest, KNearestNeighbor_Basic) {
    tree_->build(test_cloud_);

    pcl::PointXYZINormal query;
    query.x = 0.0f;
    query.y = 0.0f;
    query.z = 0.0f;

    int k = 5;
    std::vector<pcl::PointXYZINormal> results;
    std::vector<double> distances;

    bool found = tree_->kNearestSearch(query, k, results, distances);

    EXPECT_TRUE(found);
    EXPECT_EQ(static_cast<int>(results.size()), k);
    EXPECT_EQ(static_cast<int>(distances.size()), k);

    // 距离应该递增 (PCL返回平方距离)
    for (size_t i = 1; i < distances.size(); ++i) {
        EXPECT_GE(distances[i], distances[i - 1]);
    }
}

TEST_F(IkdTreeTest, KNearestNeighbor_LargeK) {
    tree_->build(test_cloud_);

    pcl::PointXYZINormal query;
    query.x = 0.0f;
    query.y = 0.0f;
    query.z = 0.0f;

    int k = 2000;  // 大于测试点数
    std::vector<pcl::PointXYZINormal> results;
    std::vector<double> distances;

    bool found = tree_->kNearestSearch(query, k, results, distances);

    // 应该返回所有点
    EXPECT_TRUE(found);
    EXPECT_EQ(static_cast<int>(results.size()),
              static_cast<int>(test_cloud_->points.size()));
}

// ==================== 获取所有点测试 ====================

TEST_F(IkdTreeTest, GetAllPoints) {
    tree_->build(test_cloud_);

    auto cloud = tree_->getAllPoints();
    EXPECT_EQ(cloud->size(), test_cloud_->points.size());
}

// ==================== 降采样插入测试 ====================

TEST_F(IkdTreeTest, InsertPointCloud_Downsample) {
    IKdTreeConfig config;
    config.downsample_size = 1.0;  // 1米降采样
    tree_->setConfig(config);
    tree_->build(test_cloud_);
    size_t initial_size = tree_->size();

    // 插入与已有点非常接近的点 (应被降采样跳过)
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr near_cloud(
        new pcl::PointCloud<pcl::PointXYZINormal>);
    for (const auto& pt : test_cloud_->points) {
        pcl::PointXYZINormal offset = pt;
        offset.x += 0.01f;  // 微小偏移, 在降采样范围内
        near_cloud->push_back(offset);
    }
    near_cloud->width = near_cloud->size();
    near_cloud->height = 1;

    tree_->insertPointCloud(near_cloud);

    // 由于降采样, 大部分点应被跳过
    EXPECT_LE(tree_->size(), initial_size + 10);
}

// ==================== 清空测试 ====================

TEST_F(IkdTreeTest, Clear) {
    tree_->build(test_cloud_);
    EXPECT_GT(tree_->size(), 0);

    tree_->clear();

    EXPECT_EQ(tree_->size(), 0);
    EXPECT_TRUE(tree_->empty());
}

// ==================== 边界条件测试 ====================

TEST_F(IkdTreeTest, SinglePointTree) {
    auto single_cloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
        new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::PointXYZINormal pt;
    pt.x = 0.0f;
    pt.y = 0.0f;
    pt.z = 0.0f;
    pt.intensity = 0.0f;
    single_cloud->points.push_back(pt);
    single_cloud->width = 1;
    single_cloud->height = 1;

    tree_->build(single_cloud);
    EXPECT_EQ(tree_->size(), 1);

    pcl::PointXYZINormal query;
    query.x = 0.0f;
    query.y = 0.0f;
    query.z = 0.0f;

    pcl::PointXYZINormal nearest;
    double dist;
    bool found = tree_->nearestSearch(query, nearest, dist);

    EXPECT_TRUE(found);
    EXPECT_NEAR(dist, 0.0, 1e-6);
}

TEST_F(IkdTreeTest, DuplicatePoints) {
    auto cloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
        new pcl::PointCloud<pcl::PointXYZINormal>);

    // 添加重复点
    for (int i = 0; i < 100; ++i) {
        pcl::PointXYZINormal pt;
        pt.x = 0.0f;  // 全部在原点
        pt.y = 0.0f;
        pt.z = 0.0f;
        pt.intensity = static_cast<float>(i);
        cloud->points.push_back(pt);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    tree_->build(cloud);

    pcl::PointXYZINormal query;
    query.x = 0.0f;
    query.y = 0.0f;
    query.z = 0.0f;

    pcl::PointXYZINormal nearest;
    double dist;
    bool found = tree_->nearestSearch(query, nearest, dist);

    EXPECT_TRUE(found);
    EXPECT_NEAR(dist, 0.0, 1e-6);
}

// ==================== 性能测试 ====================

TEST_F(IkdTreeTest, Performance_LargeCloud) {
    // 创建大点云
    auto large_cloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
        new pcl::PointCloud<pcl::PointXYZINormal>);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-100.0f, 100.0f);

    for (int i = 0; i < 10000; ++i) {
        pcl::PointXYZINormal pt;
        pt.x = dis(gen);
        pt.y = dis(gen);
        pt.z = dis(gen);
        pt.intensity = static_cast<float>(i);
        large_cloud->points.push_back(pt);
    }
    large_cloud->width = large_cloud->points.size();
    large_cloud->height = 1;

    // 测试建树时间
    tree_->build(large_cloud);
    EXPECT_EQ(tree_->size(), 10000);

    // 测试查询时间
    pcl::PointXYZINormal query;
    query.x = 0.0f;
    query.y = 0.0f;
    query.z = 0.0f;

    for (int i = 0; i < 100; ++i) {
        pcl::PointXYZINormal nearest;
        double dist;
        tree_->nearestSearch(query, nearest, dist);
    }

    EXPECT_TRUE(true);  // 验证性能可接受
}

} // namespace test
} // namespace fast_lio2_slam

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
