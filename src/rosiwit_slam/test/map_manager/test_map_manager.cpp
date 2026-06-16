/**
 * @file test_map_manager.cpp
 * @brief 地图管理模块单元测试
 * @author AI Development Team
 * @date 2026-04-24
 */

#include <gtest/gtest.h>
#include "fast_lio2_slam/map_manager/map_manager.h"
#include "fast_lio2_slam/map_manager/map_persistence.h"
#include "fast_lio2_slam/map_manager/map_quality.h"
#include "fast_lio2_slam/common/types.h"

#include <filesystem>
#include <fstream>

using namespace fast_lio2_slam;
using Sophus::SE3d;
using Sophus::SO3d;

// ==================== MapManager 测试 ====================

class MapManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        MapManagerConfig config;
        config.resolution = 0.2;
        config.submap_size = 50.0;
        config.max_submap_points = 50000;
        config.map_path = "/tmp/test_map";
        config.enable_pcd_save = true;
        config.enable_submap = true;
        
        map_manager_ = std::make_unique<MapManager>(config);
        
        // 创建测试目录
        std::filesystem::create_directories("/tmp/test_map");
        std::filesystem::create_directories("/tmp/test_persistence");
    }
    
    void TearDown() override {
        map_manager_.reset();
    }
    
    std::unique_ptr<MapManager> map_manager_;
};

TEST_F(MapManagerTest, InitializationTest) {
    EXPECT_NE(map_manager_, nullptr);
    EXPECT_EQ(map_manager_->pointCount(), static_cast<size_t>(0));
    EXPECT_EQ(map_manager_->submapCount(), static_cast<size_t>(0));
}

TEST_F(MapManagerTest, AddPointCloudTest) {
    // 创建测试点云
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < 100; ++i) {
        PointType point;
        point.x = static_cast<float>(i * 0.1);
        point.y = static_cast<float>(i * 0.1);
        point.z = static_cast<float>(i * 0.1);
        point.intensity = static_cast<float>(i);
        cloud->push_back(point);
    }
    
    SE3d pose = SE3d::Identity();
    
    map_manager_->addPointCloud(cloud, pose, 0);
    
    EXPECT_EQ(map_manager_->pointCount(), static_cast<size_t>(100));
}

TEST_F(MapManagerTest, SaveLoadMapTest) {
    // 创建测试点云
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < 50; ++i) {
        PointType point;
        point.x = static_cast<float>(i);
        point.y = static_cast<float>(i);
        point.z = static_cast<float>(i);
        point.intensity = static_cast<float>(i);
        cloud->push_back(point);
    }
    
    SE3d pose;  // 默认构造创建单位变换
    map_manager_->addPointCloud(cloud, pose, 0);
    
    // 保存地图 (PCD格式)
    bool save_result = map_manager_->saveMap("/tmp/test_map/test_map", "pcd");
    EXPECT_TRUE(save_result);
    
    // 检查文件是否存在
    EXPECT_TRUE(std::filesystem::exists("/tmp/test_map/test_map.pcd"));
    
    // 创建新的MapManager加载地图
    MapManagerConfig config;
    config.map_path = "/tmp/test_map";
    MapManager new_manager(config);
    
    bool load_result = new_manager.loadMap("/tmp/test_map/test_map.pcd", false);
    EXPECT_TRUE(load_result);
    
    // 验证点数
    EXPECT_EQ(new_manager.pointCount(), static_cast<size_t>(50));
}

TEST_F(MapManagerTest, ClearTest) {
    // 创建测试点云
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < 30; ++i) {
        PointType point;
        point.x = static_cast<float>(i);
        point.y = static_cast<float>(i);
        point.z = static_cast<float>(i);
        cloud->push_back(point);
    }
    
    SE3d pose;  // 默认构造创建单位变换
    map_manager_->addPointCloud(cloud, pose, 0);
    
    EXPECT_EQ(map_manager_->pointCount(), static_cast<size_t>(30));
    
    // 清空地图
    map_manager_->clear();
    
    EXPECT_EQ(map_manager_->pointCount(), static_cast<size_t>(0));
}

TEST_F(MapManagerTest, GetVisualizationCloudTest) {
    // 创建测试点云
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < 1000; ++i) {
        PointType point;
        point.x = static_cast<float>(i * 0.01);
        point.y = static_cast<float>(i * 0.01);
        point.z = static_cast<float>(i * 0.01);
        cloud->push_back(point);
    }
    
    SE3d pose;  // 默认构造创建单位变换
    map_manager_->addPointCloud(cloud, pose, 0);
    
    // 获取可视化点云
    PointCloudPtr viz_cloud = map_manager_->getVisualizationCloud(0.1);
    
    EXPECT_NE(viz_cloud, nullptr);
    // 体素滤波后点数应该减少
    EXPECT_LT(viz_cloud->size(), cloud->size());
}

TEST_F(MapManagerTest, MetadataTest) {
    // 添加一些点
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < 100; ++i) {
        PointType point;
        point.x = static_cast<float>(i * 0.1);
        point.y = static_cast<float>(i * 0.1);
        point.z = static_cast<float>(i * 0.1);
        cloud->push_back(point);
    }
    
    SE3d pose;  // 默认构造创建单位变换
    map_manager_->addPointCloud(cloud, pose, 0);
    
    // 获取元数据
    MapMetadata metadata = map_manager_->getMetadata();
    
    EXPECT_EQ(metadata.total_points, 100);
}

TEST_F(MapManagerTest, StatisticsTest) {
    // 添加一些点
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < 100; ++i) {
        PointType point;
        point.x = static_cast<float>(i * 0.1);
        point.y = static_cast<float>(i * 0.1);
        point.z = static_cast<float>(i * 0.1);
        cloud->push_back(point);
    }
    
    SE3d pose;  // 默认构造创建单位变换
    map_manager_->addPointCloud(cloud, pose, 0);
    
    // 获取统计信息
    MapStatistics stats = map_manager_->getStatistics();
    
    EXPECT_EQ(stats.total_points, 100);
}

// ==================== MapPersistence 测试 ====================

class MapPersistenceTest : public ::testing::Test {
protected:
    void SetUp() override {
        persistence_ = std::make_unique<MapPersistence>();
        std::filesystem::create_directories("/tmp/test_persistence");
    }
    
    void TearDown() override {
        persistence_.reset();
    }
    
    std::unique_ptr<MapPersistence> persistence_;
};

TEST_F(MapPersistenceTest, SaveLoadPointCloudTest) {
    // 创建测试点云
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < 50; ++i) {
        PointType point;
        point.x = static_cast<float>(i);
        point.y = static_cast<float>(i);
        point.z = static_cast<float>(i);
        point.intensity = static_cast<float>(i);
        cloud->push_back(point);
    }
    
    persistence_->savePointCloud(cloud, "/tmp/test_persistence/load_test.pcd");
    
    // 加载
    PointCloudPtr loaded = persistence_->loadPointCloud("/tmp/test_persistence/load_test.pcd");
    EXPECT_NE(loaded, nullptr);
    EXPECT_EQ(loaded->size(), static_cast<size_t>(50));
}

TEST_F(MapPersistenceTest, PLYFormatTest) {
    // 创建测试点云
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < 30; ++i) {
        PointType point;
        point.x = static_cast<float>(i);
        point.y = static_cast<float>(i);
        point.z = static_cast<float>(i);
        cloud->push_back(point);
    }
    
    // PLY格式保存和加载
    persistence_->savePointCloud(cloud, "/tmp/test_persistence/test.ply");
    PointCloudPtr loaded = persistence_->loadPointCloud("/tmp/test_persistence/test.ply");
    
    EXPECT_NE(loaded, nullptr);
    EXPECT_EQ(loaded->size(), static_cast<size_t>(30));
}

// ==================== 类型定义测试 ====================

TEST(TypesTest, SessionInfoTest) {
    SessionInfo session("test_session", "Test Session");
    EXPECT_EQ(session.session_id, "test_session");
    EXPECT_EQ(session.name, "Test Session");
    EXPECT_EQ(session.frame_count, 0);
    EXPECT_FALSE(session.is_merged);
}

TEST(TypesTest, MapMetadataTest) {
    MapMetadata metadata;
    metadata.map_name = "test_map";
    metadata.total_points = 1000;
    metadata.total_submaps = 5;
    
    EXPECT_EQ(metadata.map_name, "test_map");
    EXPECT_EQ(metadata.total_points, 1000);
    EXPECT_DOUBLE_EQ(metadata.computeCoverageArea(), 0.0);
}

TEST(TypesTest, MapStatisticsTest) {
    MapStatistics stats;
    stats.memory_usage_mb = 1024.0;
    stats.memory_limit_mb = 2048.0;
    
    EXPECT_DOUBLE_EQ(stats.memoryUsagePercent(), 50.0);
    EXPECT_FALSE(stats.isMemoryCritical());
    
    stats.memory_usage_mb = 1800.0;
    EXPECT_TRUE(stats.isMemoryCritical());
}

TEST(TypesTest, SubmapInfoTest) {
    SubmapInfo info;
    info.id = 1;
    info.point_count = 1000;
    info.center_pose = SE3d();
    
    EXPECT_EQ(info.id, 1);
    EXPECT_EQ(info.point_count, 1000);
}

// ==================== MapQualityEvaluator 测试 ====================

class MapQualityTest : public ::testing::Test {
protected:
    void SetUp() override {
        evaluator_ = std::make_unique<MapQualityEvaluator>();
    }
    
    void TearDown() override {
        evaluator_.reset();
    }
    
    std::unique_ptr<MapQualityEvaluator> evaluator_;
};

TEST_F(MapQualityTest, EvaluateEmptyCloud) {
    PointCloudPtr empty_cloud(new pcl::PointCloud<PointType>());
    
    QualityReport result = evaluator_->evaluatePointCloud(empty_cloud);
    
    // 空点云评估应该返回有效结果
    EXPECT_GE(result.overall_score, 0.0);
    EXPECT_LE(result.overall_score, 1.0);
}

TEST_F(MapQualityTest, EvaluateValidCloud) {
    // 创建均匀分布的点云
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    for (int x = 0; x < 10; ++x) {
        for (int y = 0; y < 10; ++y) {
            for (int z = 0; z < 10; ++z) {
                PointType point;
                point.x = static_cast<float>(x * 0.5);
                point.y = static_cast<float>(y * 0.5);
                point.z = static_cast<float>(z * 0.5);
                cloud->push_back(point);
            }
        }
    }
    
    QualityReport result = evaluator_->evaluatePointCloud(cloud);
    
    EXPECT_GE(result.overall_score, 0.0);
    EXPECT_LE(result.overall_score, 1.0);
    EXPECT_GT(result.density_score, 0.0);
}

TEST_F(MapQualityTest, DensityScoreTest) {
    // 稀疏点云
    PointCloudPtr sparse_cloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < 10; ++i) {
        PointType point;
        point.x = static_cast<float>(i * 10.0);
        point.y = static_cast<float>(i * 10.0);
        point.z = static_cast<float>(i * 10.0);
        sparse_cloud->push_back(point);
    }
    
    // 密集点云
    PointCloudPtr dense_cloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < 1000; ++i) {
        PointType point;
        point.x = static_cast<float>((i % 10) * 0.1);
        point.y = static_cast<float>((i / 10 % 10) * 0.1);
        point.z = static_cast<float>((i / 100) * 0.1);
        dense_cloud->push_back(point);
    }
    
    double sparse_density = evaluator_->computeDensityScore(sparse_cloud);
    double dense_density = evaluator_->computeDensityScore(dense_cloud);
    
    // 密集点云应该有更高的密度分数
    EXPECT_GT(dense_density, sparse_density);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}