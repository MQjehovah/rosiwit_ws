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

}  // namespace fast_lio2_slam
