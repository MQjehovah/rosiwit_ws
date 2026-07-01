/**
 * @file test_types_only.cpp
 * @brief 类型定义和基础功能单元测试
 * @author AI Development Team
 * @date 2026-04-24
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <iostream>
#include <memory>
#include <filesystem>

// 简化的类型定义测试（不依赖完整项目头文件）

// 定义测试用的PointType
using PointType = pcl::PointXYZI;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;
using SE3d = Sophus::SE3<double>;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3<double>;

// ==================== Sophus测试 ====================
TEST(SophusTest, SE3IdentityTest) {
    SE3d identity;
    
    // 默认构造应该是单位变换
    EXPECT_NEAR(identity.translation().x(), 0.0, 1e-6);
    EXPECT_NEAR(identity.translation().y(), 0.0, 1e-6);
    EXPECT_NEAR(identity.translation().z(), 0.0, 1e-6);
    
    // 验证旋转矩阵为单位阵
    auto rotation_matrix = identity.rotationMatrix();
    EXPECT_NEAR(rotation_matrix(0, 0), 1.0, 1e-6);
    EXPECT_NEAR(rotation_matrix(1, 1), 1.0, 1e-6);
    EXPECT_NEAR(rotation_matrix(2, 2), 1.0, 1e-6);
}

TEST(SophusTest, SE3TransformTest) {
    // 使用SO3和Vector3d构造SE3
    auto rotation = Sophus::SO3<double>::rotX(0.1);
    SE3d pose(rotation, Vector3d(1.0, 2.0, 3.0));
    
    EXPECT_NEAR(pose.translation().x(), 1.0, 1e-6);
    EXPECT_NEAR(pose.translation().y(), 2.0, 1e-6);
    EXPECT_NEAR(pose.translation().z(), 3.0, 1e-6);
}

// ==================== PCL点云测试 ====================
TEST(PCLTest, PointCloudCreationTest) {
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    
    // 空点云
    EXPECT_EQ(cloud->size(), static_cast<size_t>(0));
    
    // 添加点
    for (int i = 0; i < 100; ++i) {
        PointType point;
        point.x = static_cast<float>(i * 0.1);
        point.y = static_cast<float>(i * 0.1);
        point.z = static_cast<float>(i * 0.1);
        point.intensity = static_cast<float>(i);
        cloud->push_back(point);
    }
    
    EXPECT_EQ(cloud->size(), static_cast<size_t>(100));
}

TEST(PCLTest, PointCloudClearTest) {
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    
    for (int i = 0; i < 50; ++i) {
        PointType point;
        point.x = static_cast<float>(i);
        point.y = static_cast<float>(i);
        point.z = static_cast<float>(i);
        cloud->push_back(point);
    }
    
    EXPECT_EQ(cloud->size(), static_cast<size_t>(50));
    
    cloud->clear();
    EXPECT_EQ(cloud->size(), static_cast<size_t>(0));
}

TEST(PCLTest, PointIntensityTest) {
    PointType point;
    point.x = 1.0f;
    point.y = 2.0f;
    point.z = 3.0f;
    point.intensity = 100.0f;
    
    EXPECT_FLOAT_EQ(point.x, 1.0f);
    EXPECT_FLOAT_EQ(point.y, 2.0f);
    EXPECT_FLOAT_EQ(point.z, 3.0f);
    EXPECT_FLOAT_EQ(point.intensity, 100.0f);
}

// ==================== Eigen矩阵测试 ====================
TEST(EigenTest, Vector3dTest) {
    Vector3d v(1.0, 2.0, 3.0);
    
    EXPECT_DOUBLE_EQ(v.x(), 1.0);
    EXPECT_DOUBLE_EQ(v.y(), 2.0);
    EXPECT_DOUBLE_EQ(v.z(), 3.0);
    
    // 测试范数
    EXPECT_NEAR(v.norm(), std::sqrt(1.0 + 4.0 + 9.0), 1e-6);
}

TEST(EigenTest, Matrix3dIdentityTest) {
    Matrix3d I = Matrix3d::Identity();
    
    EXPECT_DOUBLE_EQ(I(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(I(1, 1), 1.0);
    EXPECT_DOUBLE_EQ(I(2, 2), 1.0);
    EXPECT_DOUBLE_EQ(I(0, 1), 0.0);
}

TEST(EigenTest, VectorBlockTest) {
    Vector3d v(1.0, 2.0, 3.0);
    
    // 测试block操作
    auto block2 = v.head<2>();
    EXPECT_DOUBLE_EQ(block2(0), 1.0);
    EXPECT_DOUBLE_EQ(block2(1), 2.0);
}

// ==================== 地图数据结构测试 ====================

// 会话信息结构
struct TestSessionInfo {
    std::string session_id;
    std::string name;
    int frame_count = 0;
    bool is_merged = false;
    std::string start_time;
    std::string end_time;
    
    TestSessionInfo(const std::string& id, const std::string& n)
        : session_id(id), name(n), frame_count(0), is_merged(false) {}
};

TEST(MapTypesTest, SessionInfoTest) {
    TestSessionInfo session("test_session", "Test Session");
    EXPECT_EQ(session.session_id, "test_session");
    EXPECT_EQ(session.name, "Test Session");
    EXPECT_EQ(session.frame_count, 0);
    EXPECT_FALSE(session.is_merged);
}

// 地图元数据结构
struct TestMapMetadata {
    std::string map_name;
    size_t total_points = 0;
    size_t total_submaps = 0;
    Vector3d min_bound;
    Vector3d max_bound;
    std::string created_time;
    std::string modified_time;
    
    double computeCoverageArea() const {
        return (max_bound.x() - min_bound.x()) * 
               (max_bound.y() - min_bound.y());
    }
};

TEST(MapTypesTest, MapMetadataTest) {
    TestMapMetadata metadata;
    metadata.map_name = "test_map";
    metadata.total_points = 1000;
    metadata.total_submaps = 5;
    metadata.min_bound = Vector3d(0.0, 0.0, 0.0);
    metadata.max_bound = Vector3d(10.0, 10.0, 5.0);
    
    EXPECT_EQ(metadata.map_name, "test_map");
    EXPECT_EQ(metadata.total_points, 1000);
    EXPECT_DOUBLE_EQ(metadata.computeCoverageArea(), 100.0);
}

// 地图统计结构
struct TestMapStatistics {
    size_t total_points = 0;
    size_t total_frames = 0;
    size_t total_submaps = 0;
    double memory_usage_mb = 0.0;
    double memory_limit_mb = 2048.0;
    double avg_point_density = 0.0;
    
    double memoryUsagePercent() const {
        return (memory_usage_mb / memory_limit_mb) * 100.0;
    }
    
    bool isMemoryCritical() const {
        return memoryUsagePercent() > 80.0;
    }
};

TEST(MapTypesTest, MapStatisticsTest) {
    TestMapStatistics stats;
    stats.memory_usage_mb = 1024.0;
    stats.memory_limit_mb = 2048.0;
    
    EXPECT_DOUBLE_EQ(stats.memoryUsagePercent(), 50.0);
    EXPECT_FALSE(stats.isMemoryCritical());
    
    stats.memory_usage_mb = 1800.0;
    EXPECT_TRUE(stats.isMemoryCritical());
}

// 子地图信息结构
struct TestSubmapInfo {
    int id = -1;
    int point_count = 0;
    SE3d center_pose;
    double timestamp_start = 0.0;
    double timestamp_end = 0.0;
    Vector3d min_bound;
    Vector3d max_bound;
    
    TestSubmapInfo() : center_pose(), min_bound(Vector3d::Zero()), max_bound(Vector3d::Zero()) {}
};

TEST(MapTypesTest, SubmapInfoTest) {
    TestSubmapInfo info;
    info.id = 1;
    info.point_count = 1000;
    
    EXPECT_EQ(info.id, 1);
    EXPECT_EQ(info.point_count, 1000);
    EXPECT_NEAR(info.center_pose.translation().x(), 0.0, 1e-6);
}

// ==================== 简化版MapManager测试 ====================

class SimpleMapManager {
public:
    SimpleMapManager(double resolution = 0.2) : resolution_(resolution), total_points_(0) {}
    
    void addPointCloud(const PointCloudPtr& cloud) {
        // 简化实现：直接计数
        total_points_ += cloud->size();
        if (total_cloud_) {
            *total_cloud_ += *cloud;
        } else {
            total_cloud_ = cloud;
        }
    }
    
    size_t pointCount() const { return total_points_; }
    
    void clear() {
        total_points_ = 0;
        if (total_cloud_) {
            total_cloud_->clear();
        }
    }
    
    PointCloudPtr getCloud() const { return total_cloud_; }
    
private:
    double resolution_;
    size_t total_points_;
    PointCloudPtr total_cloud_;
};

TEST(SimpleMapManagerTest, InitializationTest) {
    SimpleMapManager manager;
    EXPECT_EQ(manager.pointCount(), static_cast<size_t>(0));
}

TEST(SimpleMapManagerTest, AddPointCloudTest) {
    SimpleMapManager manager;
    
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < 100; ++i) {
        PointType point;
        point.x = static_cast<float>(i * 0.1);
        point.y = static_cast<float>(i * 0.1);
        point.z = static_cast<float>(i * 0.1);
        cloud->push_back(point);
    }
    
    manager.addPointCloud(cloud);
    EXPECT_EQ(manager.pointCount(), static_cast<size_t>(100));
}

TEST(SimpleMapManagerTest, ClearTest) {
    SimpleMapManager manager;
    
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < 50; ++i) {
        PointType point;
        point.x = static_cast<float>(i);
        point.y = static_cast<float>(i);
        point.z = static_cast<float>(i);
        cloud->push_back(point);
    }
    
    manager.addPointCloud(cloud);
    EXPECT_EQ(manager.pointCount(), static_cast<size_t>(50));
    
    manager.clear();
    EXPECT_EQ(manager.pointCount(), static_cast<size_t>(0));
}

// ==================== 文件系统测试 ====================
TEST(FileSystemTest, CreateDirectoryTest) {
    std::string test_path = "/tmp/test_map_manager_fs";
    
    if (std::filesystem::exists(test_path)) {
        std::filesystem::remove_all(test_path);
    }
    
    std::filesystem::create_directories(test_path);
    EXPECT_TRUE(std::filesystem::exists(test_path));
    EXPECT_TRUE(std::filesystem::is_directory(test_path));
    
    // 清理
    std::filesystem::remove_all(test_path);
    EXPECT_FALSE(std::filesystem::exists(test_path));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}