/**
 * @file global_localizer.cpp
 * @brief FAST-LIO2 SLAM - Global Localization Module Implementation
 */

#include "fast_lio2_slam/localization/global_localizer.h"

#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace fast_lio2_slam {

GlobalLocalizer::GlobalLocalizer()
    : state_(LocalizationState::UNINITIALIZED),
      initialized_(false),
      map_loaded_(false) {
    scan_context_ = std::make_unique<ScanContext>();
}

GlobalLocalizer::GlobalLocalizer(const GlobalLocalizerConfig& config)
    : config_(config),
      state_(LocalizationState::UNINITIALIZED),
      initialized_(false),
      map_loaded_(false) {
    scan_context_ = std::make_unique<ScanContext>();
    initialize(config);
}

void GlobalLocalizer::initialize(const GlobalLocalizerConfig& config) {
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

void GlobalLocalizer::setMap(const PointCloudPtr& map_cloud,
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

bool GlobalLocalizer::loadMap(const std::string& map_path) {
    // Note: Actual map loading should be done by MapManager/MapPersistence
    // This is a placeholder that would call those modules
    RCLCPP_WARN(rclcpp::get_logger("global_localizer"),
                "loadMap() should be implemented with MapPersistence module");
    return false;
}

LocalizationResult GlobalLocalizer::localize(const PointCloudPtr& scan,
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

bool GlobalLocalizer::coarseLocalize(const PointCloudPtr& scan,
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

double GlobalLocalizer::fineAlign(const PointCloudPtr& scan,
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

double GlobalLocalizer::computeNDTAlignment(const PointCloudPtr& source,
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

double GlobalLocalizer::computeICPAlignment(const PointCloudPtr& source,
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

bool GlobalLocalizer::validateResult(LocalizationResult& result) {
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

PointCloudPtr GlobalLocalizer::preprocessScan(const PointCloudPtr& scan) {
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

void GlobalLocalizer::setInitialPose(const SE3d& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    initial_pose_ = pose;
    has_initial_pose_ = true;
}

void GlobalLocalizer::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = LocalizationState::UNINITIALIZED;
    current_pose_ = SE3d();
    has_initial_pose_ = false;
}

void GlobalLocalizer::setConfig(const GlobalLocalizerConfig& config) {
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

void GlobalLocalizer::addKeyframe(const PointCloudPtr& cloud,
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

size_t GlobalLocalizer::keyframeCount() const {
    return keyframe_descriptors_.size();
}

double GlobalLocalizer::computeFitnessScore(const PointCloudPtr& source,
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

double GlobalLocalizer::computeInlierRatio(const PointCloudPtr& source,
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
