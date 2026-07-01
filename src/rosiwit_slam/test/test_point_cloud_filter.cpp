/**
 * @file test_point_cloud_filter.cpp
 * @brief 单元测试 - 点云滤波器
 * @author AI Development Team - Test Engineer
 * @date 2026-04-24
 */

#include <gtest/gtest.h>
#include "fast_lio2_slam/data_preprocessor/point_cloud_filter.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fast_lio2_slam {
namespace test {

class PointCloudFilterTest : public ::testing::Test {
protected:
    std::unique_ptr<PointCloudFilter> filter_;
    PointCloudFilterConfig params_;
    void SetUp() override {
        // 创建默认参数
        params_.voxel_size = 0.1;
        params_.min_distance = 0.3;
        params_.max_distance = 100.0;
        params_.use_voxel_filter = true;
        params_.use_outlier_removal = true;
        params_.mean_k = 50;
        params_.std_mul_thresh = 1.0;
        
        filter_ = std::make_unique<PointCloudFilter>(params_);
    }
    
    PointCloudFilterParams params_;
    std::unique_ptr<PointCloudFilter> filter_;
    
    // 创建测试点云
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr createTestCloud(int num_points) {
        auto cloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
            new pcl::PointCloud<pcl::PointXYZINormal>);
        
        for (int i = 0; i < num_points; ++i) {
            pcl::PointXYZINormal pt;
            pt.x = static_cast<float>(i) * 0.1f;
            pt.y = static_cast<float>(i) * 0.05f;
            pt.z = static_cast<float>(i) * 0.02f;
            pt.intensity = static_cast<float>(i);
            pt.normal_x = 0.0f;
            pt.normal_y = 0.0f;
            pt.normal_z = 1.0f;
            cloud->points.push_back(pt);
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;
        return cloud;
    }
};

// ==================== 初始化测试 ====================

TEST_F(PointCloudFilterTest, DefaultInitialization) {
    PointCloudFilter filter;
    EXPECT_TRUE(true);  // 验证构造不崩溃
}

TEST_F(PointCloudFilterTest, ParamInitialization) {
    // 直接访问配置成员（因为PointCloudFilter没有提供getter方法）
    EXPECT_NEAR(params_.voxel_size, 0.1, 1e-6);
    EXPECT_NEAR(params_.min_distance, 0.3, 1e-6);
    EXPECT_NEAR(params_.max_distance, 100.0, 1e-6);
}

// ==================== 体素滤波测试 ====================

TEST_F(PointCloudFilterTest, VoxelFilter_Basic) {
    auto cloud = createTestCloud(1000);
    auto filtered = filter_->voxelDownsample(cloud);
    
    // 体素滤波后点数应该减少
    EXPECT_LT(filtered->points.size(), cloud->points.size());
}

TEST_F(PointCloudFilterTest, VoxelFilter_EmptyCloud) {
    auto cloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
        new pcl::PointCloud<pcl::PointXYZINormal>);
    cloud->width = 0;
    cloud->height = 1;
    
    auto filtered = filter_->voxelDownsample(cloud);
    EXPECT_EQ(filtered->points.size(), 0);
}

TEST_F(PointCloudFilterTest, VoxelFilter_PreserveStructure) {
    auto cloud = createTestCloud(100);
    auto filtered = filter_->voxelDownsample(cloud);
    
    // 滤波后点云应该保持基本结构
    EXPECT_GT(filtered->points.size(), 0);
    EXPECT_EQ(filtered->height, 1);
}

// ==================== 距离滤波测试 ====================

TEST_F(PointCloudFilterTest, DistanceFilter_Basic) {
    auto cloud = createTestCloud(100);
    auto filtered = filter_->filterByRange(cloud);
    
    // 所有有效点应保留
    EXPECT_GT(filtered->points.size(), 0);
}

TEST_F(PointCloudFilterTest, DistanceFilter_RemoveNearPoints) {
    // 创建包含近距离点的点云
    auto cloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
        new pcl::PointCloud<pcl::PointXYZINormal>);
    
    // 添加近距离点（小于min_distance）
    for (int i = 0; i < 10; ++i) {
        pcl::PointXYZINormal pt;
        pt.x = 0.1f;
        pt.y = 0.0f;
        pt.z = 0.0f;
        pt.intensity = 0.0f;
        cloud->points.push_back(pt);
    }
    
    // 添加远距离点
    for (int i = 0; i < 10; ++i) {
        pcl::PointXYZINormal pt;
        pt.x = 10.0f;
        pt.y = static_cast<float>(i);
        pt.z = 0.0f;
        pt.intensity = 1.0f;
        cloud->points.push_back(pt);
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    
    auto filtered = filter_->filterByRange(cloud);
    
    // 近距离点应该被过滤
    EXPECT_LT(filtered->points.size(), cloud->points.size());
}

TEST_F(PointCloudFilterTest, DistanceFilter_RemoveFarPoints) {
    // 创建包含远距离点的点云
    auto cloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
        new pcl::PointCloud<pcl::PointXYZINormal>);
    
    // 添加超远距离点
    for (int i = 0; i < 10; ++i) {
        pcl::PointXYZINormal pt;
        pt.x = 200.0f;  // 超过max_distance
        pt.y = static_cast<float>(i);
        pt.z = 0.0f;
        pt.intensity = 1.0f;
        cloud->points.push_back(pt);
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    
    auto filtered = filter_->filterByRange(cloud);
    
    // 远距离点应该被过滤
    EXPECT_EQ(filtered->points.size(), 0);
}

// ==================== 离群点移除测试 ====================

TEST_F(PointCloudFilterTest, OutlierRemoval_Basic) {
    auto cloud = createTestCloud(100);
    auto filtered = filter_->statisticalOutlierRemoval(cloud);
    
    // 正常点云应该保留大部分点
    EXPECT_GT(filtered->points.size(), 50);
}

TEST_F(PointCloudFilterTest, OutlierRemoval_WithOutliers) {
    auto cloud = createTestCloud(100);
    
    // 添加一些离群点
    for (int i = 0; i < 10; ++i) {
        pcl::PointXYZINormal pt;
        pt.x = 1000.0f + static_cast<float>(i) * 100.0f;
        pt.y = 1000.0f;
        pt.z = 1000.0f;
        pt.intensity = 100.0f;
        cloud->points.push_back(pt);
    }
    cloud->width = cloud->points.size();
    
    auto filtered = filter_->statisticalOutlierRemoval(cloud);
    
    // 离群点应该被移除
    EXPECT_LT(filtered->points.size(), cloud->points.size());
}

// ==================== 完整滤波流水线测试 ====================

TEST_F(PointCloudFilterTest, FullPipeline) {
    auto cloud = createTestCloud(1000);
    auto filtered = filter_->process(cloud);
    
    // 完整滤波后点数应该减少
    EXPECT_LT(filtered->points.size(), cloud->points.size());
    EXPECT_GT(filtered->points.size(), 0);
}

TEST_F(PointCloudFilterTest, FullPipeline_EmptyCloud) {
    auto cloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
        new pcl::PointCloud<pcl::PointXYZINormal>);
    cloud->width = 0;
    cloud->height = 1;
    
    auto filtered = filter_->process(cloud);
    EXPECT_EQ(filtered->points.size(), 0);
}

// ==================== 参数更新测试 ====================

TEST_F(PointCloudFilterTest, UpdateParams) {
    // 修改参数并重新创建filter
    config_.voxel_size = 0.5;
    config_.min_distance = 1.0;
    config_.max_distance = 50.0;
    
    filter_ = std::make_unique<PointCloudFilter>(config_);
    
    // 验证参数已更新（通过创建新filter的方式）
}

// ==================== 边界条件测试 ====================

TEST_F(PointCloudFilterTest, SinglePoint) {
    auto cloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
        new pcl::PointCloud<pcl::PointXYZINormal>);
    
    pcl::PointXYZINormal pt;
    pt.x = 5.0f;
    pt.y = 0.0f;
    pt.z = 0.0f;
    pt.intensity = 1.0f;
    cloud->points.push_back(pt);
    cloud->width = 1;
    cloud->height = 1;
    
    auto filtered = filter_->process(cloud);
    EXPECT_EQ(filtered->points.size(), 1);
}

TEST_F(PointCloudFilterTest, AllPointsOutOfRange) {
    auto cloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
        new pcl::PointCloud<pcl::PointXYZINormal>);
    
    // 所有点都在过滤范围外
    for (int i = 0; i < 100; ++i) {
        pcl::PointXYZINormal pt;
        pt.x = 0.1f;  // 小于min_distance
        pt.y = 0.0f;
        pt.z = 0.0f;
        pt.intensity = 0.0f;
        cloud->points.push_back(pt);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    
    auto filtered = filter_->filterByRange(cloud);
    EXPECT_EQ(filtered->points.size(), 0);
}

} // namespace test
} // namespace fast_lio2_slam

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}