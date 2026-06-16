/**
 * @file global_localizer.h
 * @brief FAST-LIO2 SLAM - Global Localization Module
 * @author AI Development Team
 * @date 2026-04-29
 *
 * Implements three-stage global localization:
 * 1. Coarse localization using Scan Context for global descriptor matching
 * 2. Fine alignment using NDT/ICP for precise registration
 * 3. Validation for localization reliability assessment
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include <atomic>
#include <memory>
#include <vector>

#include "fast_lio2_slam/common/types.h"
#include "fast_lio2_slam/loop_closure/scan_context.h"

namespace fast_lio2_slam {

// LocalizationState and LocalizationResult are defined in common/types.h

/**
 * @brief Global Localizer Configuration
 */
struct GlobalLocalizerConfig {
    // Enable/disable localization
    bool enable = true;

    // Mode: "auto" (automatic on startup) or "manual" (triggered by service)
    std::string mode = "manual";

    // Scan Context parameters for coarse localization
    struct ScanContextParams {
        int ring_num = 20;
        int sector_num = 60;
        double max_range = 80.0;
        double dist_threshold = 0.3;
        int candidate_count = 5;
        int exclude_recent_frames = 30;
    } scan_context;

    // Fine alignment parameters
    struct FineAlignmentParams {
        std::string method = "ndt";  // "ndt" or "icp"
        int max_iterations = 50;
        double convergence_threshold = 0.01;
        double transformation_epsilon = 0.01;
        double step_size = 0.1;          // NDT step size
        double resolution = 1.0;          // NDT resolution
        double max_correspondence_dist = 2.0;  // ICP max correspondence distance
        double voxel_size = 0.5;          // Downsample voxel size
    } fine_alignment;

    // Validation parameters
    struct ValidationParams {
        double min_fitness_score = 0.7;
        double min_inlier_ratio = 0.5;
        double max_position_error = 2.0;   // Maximum position error (meters)
        double max_rotation_error = 0.5;   // Maximum rotation error (radians)
    } validation;

    // Search parameters
    struct SearchParams {
        bool use_keyframe_poses = true;   // Use known keyframe poses as prior
        double search_radius = 10.0;       // Search radius when using prior
        int max_search_candidates = 10;    // Maximum candidates to search
    } search;
};

/**
 * @brief Candidate pose from coarse localization
 */
struct LocalizationCandidate {
    SE3d pose;
    int keyframe_id = -1;
    double score = 0.0;
    double yaw_diff = 0.0;
    PointCloudPtr keyframe_cloud;
};

/**
 * @brief Global Localizer Class
 *
 * Implements multi-stage global localization:
 * - Stage 1: Coarse localization using Scan Context
 * - Stage 2: Fine alignment using NDT/ICP
 * - Stage 3: Validation for reliability
 */
class GlobalLocalizer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GlobalLocalizer();
    explicit GlobalLocalizer(const GlobalLocalizerConfig& config);
    ~GlobalLocalizer() = default;

    /**
     * @brief Initialize the localizer with configuration
     */
    void initialize(const GlobalLocalizerConfig& config);

    /**
     * @brief Set the target map for localization
     * @param map_cloud Map point cloud
     * @param keyframes Optional keyframe descriptors from Scan Context
     */
    void setMap(const PointCloudPtr& map_cloud,
                const std::vector<ScanContextDescriptor>& keyframes = {});

    /**
     * @brief Set the target map from file
     * @param map_path Path to map file
     * @return true if successful
     */
    bool loadMap(const std::string& map_path);

    /**
     * @brief Perform global localization
     * @param scan Current scan point cloud
     * @param initial_pose Optional initial pose estimate
     * @return Localization result
     */
    LocalizationResult localize(const PointCloudPtr& scan,
                                 const SE3d& initial_pose = SE3d());

    /**
     * @brief Perform global localization with multiple candidates
     * @param scan Current scan point cloud
     * @param candidates Output candidate poses
     * @return Localization result
     */
    LocalizationResult localizeWithCandidates(
        const PointCloudPtr& scan,
        std::vector<LocalizationCandidate>& candidates);

    /**
     * @brief Set initial pose estimate (for localization assistance)
     * @param pose Initial pose estimate
     */
    void setInitialPose(const SE3d& pose);

    /**
     * @brief Get current localization state
     */
    LocalizationState getState() const { return state_; }

    /**
     * @brief Check if localizer is initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Check if map is loaded
     */
    bool hasMap() const { return map_loaded_; }

    /**
     * @brief Get current localized pose
     */
    SE3d getCurrentPose() const { return current_pose_; }

    /**
     * @brief Reset the localizer state
     */
    void reset();

    /**
     * @brief Update configuration
     */
    void setConfig(const GlobalLocalizerConfig& config);

    /**
     * @brief Get configuration
     */
    const GlobalLocalizerConfig& getConfig() const { return config_; }

    /**
     * @brief Add a keyframe to the database
     * @param cloud Keyframe point cloud
     * @param pose Keyframe pose
     * @param timestamp Keyframe timestamp
     */
    void addKeyframe(const PointCloudPtr& cloud,
                     const SE3d& pose,
                     double timestamp);

    /**
     * @brief Get keyframe count
     */
    size_t keyframeCount() const;

private:
    // ==================== Core Localization Steps ====================

    /**
     * @brief Stage 1: Coarse localization using Scan Context
     * @param scan Current scan
     * @param candidates Output candidate poses
     * @return true if candidates found
     */
    bool coarseLocalize(const PointCloudPtr& scan,
                        std::vector<LocalizationCandidate>& candidates);

    /**
     * @brief Stage 2: Fine alignment using NDT or ICP
     * @param scan Current scan
     * @param candidate Candidate pose from coarse localization
     * @param refined_pose Output refined pose
     * @return Alignment fitness score
     */
    double fineAlign(const PointCloudPtr& scan,
                     const LocalizationCandidate& candidate,
                     SE3d& refined_pose);

    /**
     * @brief Stage 3: Validate localization result
     * @param result Localization result to validate
     * @return true if validation passes
     */
    bool validateResult(LocalizationResult& result);

    // ==================== Helper Functions ====================

    /**
     * @brief Preprocess scan for localization
     */
    PointCloudPtr preprocessScan(const PointCloudPtr& scan);

    /**
     * @brief Compute NDT alignment
     */
    double computeNDTAlignment(const PointCloudPtr& source,
                                const PointCloudPtr& target,
                                const SE3d& initial_guess,
                                SE3d& final_pose);

    /**
     * @brief Compute ICP alignment
     */
    double computeICPAlignment(const PointCloudPtr& source,
                               const PointCloudPtr& target,
                               const SE3d& initial_guess,
                               SE3d& final_pose);

    /**
     * @brief Compute fitness score
     */
    double computeFitnessScore(const PointCloudPtr& source,
                               const PointCloudPtr& target,
                               const SE3d& transform);

    /**
     * @brief Compute inlier ratio
     */
    double computeInlierRatio(const PointCloudPtr& source,
                              const PointCloudPtr& target,
                              const SE3d& transform,
                              double max_distance = 0.5);

    /**
     * @brief Select best candidate from multiple candidates
     */
    int selectBestCandidate(const std::vector<LocalizationCandidate>& candidates,
                            const PointCloudPtr& scan);

private:
    // Configuration
    GlobalLocalizerConfig config_;

    // State
    std::atomic<LocalizationState> state_;
    std::atomic<bool> initialized_;
    std::atomic<bool> map_loaded_;

    // Map data
    PointCloudPtr map_cloud_;
    PointCloudPtr map_downsampled_;  // Downsampled map for faster search

    // Scan Context for coarse localization
    std::unique_ptr<ScanContext> scan_context_;
    std::vector<ScanContextDescriptor> keyframe_descriptors_;
    std::vector<SE3d> keyframe_poses_;
    std::vector<PointCloudPtr> keyframe_clouds_;

    // Current pose
    SE3d current_pose_;
    SE3d initial_pose_;
    bool has_initial_pose_ = false;

    // Thread safety
    mutable std::mutex mutex_;

    // Statistics
    int localization_attempts_ = 0;
    int successful_localizations_ = 0;
};

// ==================== Implementation ====================

inline GlobalLocalizer::GlobalLocalizer()
    : state_(LocalizationState::UNINITIALIZED),
      initialized_(false),
      map_loaded_(false) {
    scan_context_ = std::make_unique<ScanContext>();
}

inline GlobalLocalizer::GlobalLocalizer(const GlobalLocalizerConfig& config)
    : config_(config),
      state_(LocalizationState::UNINITIALIZED),
      initialized_(false),
      map_loaded_(false) {
    scan_context_ = std::make_unique<ScanContext>();
    initialize(config);
}

inline void GlobalLocalizer::initialize(const GlobalLocalizerConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;

    // Initialize Scan Context
    ScanContextConfig sc_config;
    sc_config.ring_num = config.scan_context.ring_num;
    sc_config.sector_num = config.scan_context.sector_num;
    sc_config.max_range = config.scan_context.max_range;
    sc_config.threshold = config.scan_context.dist_threshold;
    scan_context_->initialize(sc_config);

    initialized_ = true;
    state_ = LocalizationState::UNINITIALIZED;
}

inline void GlobalLocalizer::setMap(const PointCloudPtr& map_cloud,
                                     const std::vector<ScanContextDescriptor>& keyframes) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!map_cloud || map_cloud->empty()) {
        return;
    }

    map_cloud_ = map_cloud;

    // Downsample map for faster search
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setInputCloud(map_cloud_);
    voxel_filter.setLeafSize(config_.fine_alignment.voxel_size,
                             config_.fine_alignment.voxel_size,
                             config_.fine_alignment.voxel_size);
    map_downsampled_.reset(new pcl::PointCloud<PointType>());
    voxel_filter.filter(*map_downsampled_);

    // Store keyframes
    keyframe_descriptors_ = keyframes;

    map_loaded_ = true;

    RCLCPP_INFO(rclcpp::get_logger("global_localizer"),
                "Map loaded: %zu points, %zu keyframes",
                map_cloud_->size(), keyframe_descriptors_.size());
}

inline bool GlobalLocalizer::loadMap(const std::string& map_path) {
    // Note: Actual map loading should be done by MapManager/MapPersistence
    // This is a placeholder that would call those modules
    RCLCPP_WARN(rclcpp::get_logger("global_localizer"),
                "loadMap() should be implemented with MapPersistence module");
    return false;
}

inline LocalizationResult GlobalLocalizer::localize(const PointCloudPtr& scan,
                                                      const SE3d& initial_pose) {
    std::lock_guard<std::mutex> lock(mutex_);

    LocalizationResult result;
    auto total_start = std::chrono::high_resolution_clock::now();

    localization_attempts_++;

    // Check preconditions
    if (!initialized_) {
        result.success = false;
        result.state = LocalizationState::UNINITIALIZED;
        result.error_message = "Localizer not initialized";
        return result;
    }

    if (!map_loaded_ || !map_cloud_) {
        result.success = false;
        result.state = LocalizationState::UNINITIALIZED;
        result.error_message = "No map loaded";
        return result;
    }

    if (!scan || scan->empty()) {
        result.success = false;
        result.error_message = "Empty scan";
        return result;
    }

    state_ = LocalizationState::LOCALIZING;

    // Preprocess scan
    PointCloudPtr processed_scan = preprocessScan(scan);

    // Use provided initial pose or stored initial pose
    SE3d working_pose = initial_pose;
    if (has_initial_pose_ && initial_pose.matrix().isIdentity()) {
        working_pose = initial_pose_;
    }

    // Stage 1: Coarse localization
    std::vector<LocalizationCandidate> candidates;
    auto coarse_start = std::chrono::high_resolution_clock::now();
    bool coarse_success = coarseLocalize(processed_scan, candidates);
    auto coarse_end = std::chrono::high_resolution_clock::now();
    result.coarse_time_ms = std::chrono::duration<double, std::milli>(coarse_end - coarse_start).count();

    if (!coarse_success || candidates.empty()) {
        // If no candidates found, try with initial pose as prior
        if (has_initial_pose_) {
            LocalizationCandidate prior_candidate;
            prior_candidate.pose = initial_pose_;
            prior_candidate.score = 1.0;
            candidates.push_back(prior_candidate);
        } else {
            result.success = false;
            result.state = LocalizationState::LOST;
            result.error_message = "Coarse localization failed: no candidates found";
            return result;
        }
    }

    // Stage 2: Fine alignment (try top candidates)
    auto fine_start = std::chrono::high_resolution_clock::now();

    double best_fitness_score = 0.0;
    SE3d best_pose;
    int best_keyframe_id = -1;

    int num_candidates = std::min((int)candidates.size(),
                                   config_.search.max_search_candidates);

    for (int i = 0; i < num_candidates; ++i) {
        SE3d refined_pose;
        double fitness = fineAlign(processed_scan, candidates[i], refined_pose);

        if (fitness > best_fitness_score) {
            best_fitness_score = fitness;
            best_pose = refined_pose;
            best_keyframe_id = candidates[i].keyframe_id;
        }
    }

    auto fine_end = std::chrono::high_resolution_clock::now();
    result.fine_time_ms = std::chrono::duration<double, std::milli>(fine_end - fine_start).count();

    // Check fitness score threshold
    if (best_fitness_score < config_.validation.min_fitness_score) {
        result.success = false;
        result.state = LocalizationState::LOST;
        result.error_message = "Fine alignment failed: low fitness score (" +
                               std::to_string(best_fitness_score) + ")";
        return result;
    }

    // Stage 3: Validation
    result.estimated_pose = best_pose;
    result.fitness_score = best_fitness_score;
    result.matched_keyframe_id = best_keyframe_id;

    bool validation_passed = validateResult(result);

    auto total_end = std::chrono::high_resolution_clock::now();
    result.total_time_ms = std::chrono::duration<double, std::milli>(total_end - total_start).count();

    if (validation_passed) {
        result.success = true;
        result.state = LocalizationState::LOCALIZED;
        current_pose_ = best_pose;
        successful_localizations_++;

        RCLCPP_INFO(rclcpp::get_logger("global_localizer"),
                    "Localization successful! Fitness: %.3f, Time: %.1f ms",
                    result.fitness_score, result.total_time_ms);
    } else {
        result.success = false;
        result.state = LocalizationState::LOST;
    }

    state_ = result.state;
    return result;
}

inline bool GlobalLocalizer::coarseLocalize(const PointCloudPtr& scan,
                                             std::vector<LocalizationCandidate>& candidates) {
    candidates.clear();

    // If we have keyframe descriptors, use Scan Context matching
    if (!keyframe_descriptors_.empty()) {
        // Create descriptor for current scan
        ScanContextDescriptor query = scan_context_->makeDescriptor(
            scan, 0.0, 0, SE3d());

        // Detect matching keyframes
        LoopConstraint constraint;

        // Try multiple times with different candidates
        for (int i = 0; i < config_.scan_context.candidate_count; ++i) {
            if (scan_context_->detectLoop(query, constraint)) {
                LocalizationCandidate candidate;
                candidate.keyframe_id = constraint.to_id;
                candidate.score = constraint.score;
                candidate.yaw_diff = constraint.yaw_diff;

                // Get pose from keyframe
                if (constraint.to_id >= 0 &&
                    constraint.to_id < (int)keyframe_poses_.size()) {
                    candidate.pose = keyframe_poses_[constraint.to_id];

                    // Apply yaw correction from Scan Context
                    // (Scan Context provides relative yaw estimate)
                    Eigen::Matrix3d R_yaw =
                        Eigen::AngleAxisd(constraint.yaw_diff,
                                          Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    candidate.pose.so3() = Sophus::SO3d(R_yaw) * candidate.pose.so3();
                }

                if (constraint.to_id >= 0 &&
                    constraint.to_id < (int)keyframe_clouds_.size()) {
                    candidate.keyframe_cloud = keyframe_clouds_[constraint.to_id];
                }

                candidates.push_back(candidate);
            }
        }
    }

    // If no candidates from Scan Context, use map-based search
    // This is a fallback for cases where keyframes are not available
    if (candidates.empty() && map_downsampled_) {
        // Sample poses across the map
        // This is a simplified approach; more sophisticated methods can be used
        // For now, return empty and rely on initial pose if provided
        RCLCPP_DEBUG(rclcpp::get_logger("global_localizer"),
                     "No Scan Context candidates, map-based search not implemented");
    }

    return !candidates.empty();
}

inline double GlobalLocalizer::fineAlign(const PointCloudPtr& scan,
                                          const LocalizationCandidate& candidate,
                                          SE3d& refined_pose) {
    PointCloudPtr aligned_cloud(new pcl::PointCloud<PointType>());
    double fitness_score = 0.0;

    // Downsample scan for faster alignment
    PointCloudPtr scan_downsampled(new pcl::PointCloud<PointType>());
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setInputCloud(scan);
    voxel_filter.setLeafSize(config_.fine_alignment.voxel_size,
                             config_.fine_alignment.voxel_size,
                             config_.fine_alignment.voxel_size);
    voxel_filter.filter(*scan_downsampled);

    // Select target cloud
    PointCloudPtr target_cloud = map_downsampled_;
    if (candidate.keyframe_cloud && !candidate.keyframe_cloud->empty()) {
        // Use keyframe cloud for local refinement
        target_cloud = candidate.keyframe_cloud;

        // Downsample keyframe cloud too
        PointCloudPtr keyframe_downsampled(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> kf_filter;
        kf_filter.setInputCloud(candidate.keyframe_cloud);
        kf_filter.setLeafSize(config_.fine_alignment.voxel_size * 2,
                              config_.fine_alignment.voxel_size * 2,
                              config_.fine_alignment.voxel_size * 2);
        kf_filter.filter(*keyframe_downsampled);
        target_cloud = keyframe_downsampled;
    }

    if (config_.fine_alignment.method == "ndt") {
        fitness_score = computeNDTAlignment(scan_downsampled, target_cloud,
                                            candidate.pose, refined_pose);
    } else {
        fitness_score = computeICPAlignment(scan_downsampled, target_cloud,
                                            candidate.pose, refined_pose);
    }

    return fitness_score;
}

inline double GlobalLocalizer::computeNDTAlignment(const PointCloudPtr& source,
                                                    const PointCloudPtr& target,
                                                    const SE3d& initial_guess,
                                                    SE3d& final_pose) {
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;

    ndt.setResolution(config_.fine_alignment.resolution);
    ndt.setMaximumIterations(config_.fine_alignment.max_iterations);
    ndt.setTransformationEpsilon(config_.fine_alignment.transformation_epsilon);
    ndt.setStepSize(config_.fine_alignment.step_size);

    ndt.setInputSource(source);
    ndt.setInputTarget(target);

    PointCloudPtr aligned(new pcl::PointCloud<PointType>);

    // Set initial guess
    Eigen::Matrix4f init_matrix = initial_guess.matrix().cast<float>();
    ndt.align(*aligned, init_matrix);

    if (ndt.hasConverged()) {
        Eigen::Matrix4d final_matrix = ndt.getFinalTransformation().cast<double>();
        final_pose = SE3d(final_matrix);
        return ndt.getFitnessScore();
    }

    return 0.0;
}

inline double GlobalLocalizer::computeICPAlignment(const PointCloudPtr& source,
                                                    const PointCloudPtr& target,
                                                    const SE3d& initial_guess,
                                                    SE3d& final_pose) {
    pcl::IterativeClosestPoint<PointType, PointType> icp;

    icp.setMaximumIterations(config_.fine_alignment.max_iterations);
    icp.setTransformationEpsilon(config_.fine_alignment.transformation_epsilon);
    icp.setMaxCorrespondenceDistance(config_.fine_alignment.max_correspondence_dist);

    icp.setInputSource(source);
    icp.setInputTarget(target);

    PointCloudPtr aligned(new pcl::PointCloud<PointType>);

    // Set initial guess
    Eigen::Matrix4f init_matrix = initial_guess.matrix().cast<float>();
    icp.align(*aligned, init_matrix);

    if (icp.hasConverged()) {
        Eigen::Matrix4d final_matrix = icp.getFinalTransformation().cast<double>();
        final_pose = SE3d(final_matrix);
        return 1.0 - (icp.getFitnessScore() / config_.fine_alignment.max_correspondence_dist);
    }

    return 0.0;
}

inline bool GlobalLocalizer::validateResult(LocalizationResult& result) {
    // Check fitness score
    if (result.fitness_score < config_.validation.min_fitness_score) {
        RCLCPP_WARN(rclcpp::get_logger("global_localizer"),
                    "Validation failed: fitness score %.3f < %.3f",
                    result.fitness_score, config_.validation.min_fitness_score);
        return false;
    }

    // Compute inlier ratio
    // Note: This requires map_cloud_ or keyframe_cloud
    // Simplified validation for now

    // Check position bounds (sanity check)
    double pos_norm = result.estimated_pose.translation().norm();
    if (pos_norm > 10000.0) {  // Sanity check for unreasonable positions
        RCLCPP_WARN(rclcpp::get_logger("global_localizer"),
                    "Validation failed: unreasonable position norm %.1f", pos_norm);
        return false;
    }

    return true;
}

inline PointCloudPtr GlobalLocalizer::preprocessScan(const PointCloudPtr& scan) {
    PointCloudPtr filtered(new pcl::PointCloud<PointType>());

    // Remove NaN points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*scan, *filtered, indices);

    // Downsample
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setInputCloud(filtered);
    voxel_filter.setLeafSize(config_.fine_alignment.voxel_size,
                             config_.fine_alignment.voxel_size,
                             config_.fine_alignment.voxel_size);
    PointCloudPtr downsampled(new pcl::PointCloud<PointType>());
    voxel_filter.filter(*downsampled);

    return downsampled;
}

inline void GlobalLocalizer::setInitialPose(const SE3d& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    initial_pose_ = pose;
    has_initial_pose_ = true;
}

inline void GlobalLocalizer::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = LocalizationState::UNINITIALIZED;
    current_pose_ = SE3d();
    has_initial_pose_ = false;
}

inline void GlobalLocalizer::setConfig(const GlobalLocalizerConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;

    // Reinitialize Scan Context with new parameters
    ScanContextConfig sc_config;
    sc_config.ring_num = config.scan_context.ring_num;
    sc_config.sector_num = config.scan_context.sector_num;
    sc_config.max_range = config.scan_context.max_range;
    sc_config.threshold = config.scan_context.dist_threshold;
    scan_context_->initialize(sc_config);
}

inline void GlobalLocalizer::addKeyframe(const PointCloudPtr& cloud,
                                          const SE3d& pose,
                                          double timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);

    int keyframe_id = static_cast<int>(keyframe_poses_.size());

    // Create and store descriptor
    ScanContextDescriptor desc = scan_context_->makeDescriptor(
        cloud, timestamp, keyframe_id, pose);

    scan_context_->addKeyframe(desc);
    keyframe_descriptors_.push_back(desc);
    keyframe_poses_.push_back(pose);
    keyframe_clouds_.push_back(cloud);
}

inline size_t GlobalLocalizer::keyframeCount() const {
    return keyframe_descriptors_.size();
}

inline double GlobalLocalizer::computeFitnessScore(const PointCloudPtr& source,
                                                    const PointCloudPtr& target,
                                                    const SE3d& transform) {
    if (source->empty() || target->empty()) {
        return 0.0;
    }

    // Transform source cloud
    PointCloudPtr transformed(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*source, *transformed, transform.matrix());

    // Build KD-tree for target
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(target);

    double total_distance = 0.0;
    int valid_points = 0;

    for (const auto& point : transformed->points) {
        std::vector<int> indices(1);
        std::vector<float> distances(1);

        if (kdtree.nearestKSearch(point, 1, indices, distances) > 0) {
            total_distance += std::sqrt(distances[0]);
            valid_points++;
        }
    }

    if (valid_points == 0) return 0.0;

    double mean_distance = total_distance / valid_points;
    // Convert to fitness score (lower distance = higher score)
    return 1.0 / (1.0 + mean_distance);
}

inline double GlobalLocalizer::computeInlierRatio(const PointCloudPtr& source,
                                                   const PointCloudPtr& target,
                                                   const SE3d& transform,
                                                   double max_distance) {
    if (source->empty() || target->empty()) {
        return 0.0;
    }

    // Transform source cloud
    PointCloudPtr transformed(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*source, *transformed, transform.matrix());

    // Build KD-tree for target
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(target);

    int inliers = 0;

    for (const auto& point : transformed->points) {
        std::vector<int> indices(1);
        std::vector<float> distances(1);

        if (kdtree.nearestKSearch(point, 1, indices, distances) > 0) {
            if (std::sqrt(distances[0]) <= max_distance) {
                inliers++;
            }
        }
    }

    return static_cast<double>(inliers) / transformed->size();
}

}  // namespace fast_lio2_slam