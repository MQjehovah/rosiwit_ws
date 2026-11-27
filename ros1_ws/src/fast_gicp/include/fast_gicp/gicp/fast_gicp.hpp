#ifndef FAST_GICP_FAST_GICP_HPP
#define FAST_GICP_FAST_GICP_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/search.h>
#include <pcl/registration/registration.h>
#include <fast_gicp/gicp/lsq_registration.hpp>
#include <fast_gicp/gicp/gicp_settings.hpp>

namespace fast_gicp {

// rosiwit local patch (2026-11-27): 目标协方差复用缓存。
// 目标点云 = 地图的体素质心。每次重建目标, 绝大多数点的位置和邻域都没变 (只有新提交的体素和裁剪边缘
// 变化), 它们的协方差与上一次目标里的相同。全量重算 9 万点 (每点 20 近邻 + SVD, 4 线程约 200ms /
// 0.8s CPU) 让账本每次提交后的目标重建都成为一笔重负载: 提交密集区后台线程连续满载, 和跟踪线程抢
// CPU 与内存带宽 (真实数据回放: 求解每轮 10ms -> 14ms, 窗口 p90 280ms)。
// 这里按点位置 (1mm 量化) 记住算过的协方差, 命中就直接用, 只算未命中的。
//  - 只适用于 PLANE 正则化: 协方差 = V diag(1,1,eps) V^T = I - (1-eps) n n^T (n = 最小奇异值方向),
//    每条只存 n (3 个 double)。其它正则化方式旁路缓存 (全量算)。
//  - 进程内所有实例共享: 后台建目标是每次新建实例, 按实例缓存永远命中不了。
//  - 换图必须 clearTargetCovarianceCache(): 位置相同不代表几何相同。
//  - 只有近邻集合确实没变的点才复用: 条目里存着当初第 k 个近邻的距离 d20, 新目标里若有新来的点落在
//    某点 d20 以内, 这点的近邻集合变了, 照常重算 (结果与全量计算一致到 1e-15)。离开裁剪窗口的老点不追溯:
//    它们只影响旧窗口边缘一圈 (离机器人 45m 以外)。
//  - 上限 kCap 条; 超了丢掉最近两代都没被用到的 (裁剪边缘 / 已删除的点)。
namespace detail {
struct TargetCovCache {
  std::mutex mu;
  std::atomic<bool> enabled{true};
  int k = -1;            // 缓存里的协方差是按哪组参数算的 (k 近邻数); 参数变了整表作废
  struct Entry {
    int64_t key = 0;
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    float d20 = 0.f;     // 算这个协方差时第 k 个近邻的距离平方: 新点落在它以内, 近邻集合就变了 -> 必须重算
    uint32_t used = 0;   // 最近一次被用到的代
  };
  std::unordered_map<int64_t, uint32_t> index;   // 位置键 -> entries 下标
  std::vector<Entry> entries;
  uint32_t gen = 0;
  size_t last_hits = 0, last_miss = 0, last_redo = 0;   // 最近一次 setInputTarget 的命中/未命中/因近邻变化重算的点数 (诊断)
  static constexpr size_t kCap = 600000;         // 约 46MB (条目 40B + 哈希节点)
};
}  // namespace detail

template <typename PointT>
inline detail::TargetCovCache& targetCovCache() {
  static detail::TargetCovCache cache;
  return cache;
}

// 位置键: 1mm 量化, 三个分量各 21 bit (有符号范围 ±1048m), 拼成 63 bit
inline int64_t targetCovKey(double x, double y, double z) {
  const int64_t qx = static_cast<int64_t>(std::llround(x * 1000.0)) + (1 << 20);
  const int64_t qy = static_cast<int64_t>(std::llround(y * 1000.0)) + (1 << 20);
  const int64_t qz = static_cast<int64_t>(std::llround(z * 1000.0)) + (1 << 20);
  return qx | (qy << 21) | (qz << 42);
}

/// 换图时调用: 位置相同不代表几何相同
template <typename PointT>
inline void clearTargetCovarianceCache() {
  detail::TargetCovCache& c = targetCovCache<PointT>();
  std::lock_guard<std::mutex> lock(c.mu);
  c.index.clear();
  c.entries.clear();
  c.k = -1;
}

/// 最近一次 setInputTarget 的命中/未命中/因近邻变化重算的点数与缓存条目数 (诊断用)
template <typename PointT>
inline void targetCovCacheStats(size_t& hits, size_t& miss, size_t& entries, size_t* redo = nullptr) {
  detail::TargetCovCache& c = targetCovCache<PointT>();
  std::lock_guard<std::mutex> lock(c.mu);
  hits = c.last_hits;
  miss = c.last_miss;
  entries = c.entries.size();
  if (redo) *redo = c.last_redo;
}

/// 复用开关 (默认开); 关掉后每次 setInputTarget 全量重算, 行为与复用之前完全相同
template <typename PointT>
inline void setTargetCovarianceReuseEnabled(bool on) {
  targetCovCache<PointT>().enabled.store(on);
}

/**
 * @brief Fast GICP algorithm optimized for multi threading with OpenMP
 */
template<typename PointSource, typename PointTarget, typename SearchMethodSource = pcl::search::KdTree<PointSource>, typename SearchMethodTarget = pcl::search::KdTree<PointTarget>>
class FastGICP : public LsqRegistration<PointSource, PointTarget> {
public:
  using Scalar = float;
  using Matrix4 = typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

  using PointCloudSource = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

#if PCL_VERSION >= PCL_VERSION_CALC(1, 10, 0)
  using Ptr = pcl::shared_ptr<FastGICP<PointSource, PointTarget>>;
  using ConstPtr = pcl::shared_ptr<const FastGICP<PointSource, PointTarget>>;
#else
  using Ptr = boost::shared_ptr<FastGICP<PointSource, PointTarget>>;
  using ConstPtr = boost::shared_ptr<const FastGICP<PointSource, PointTarget>>;
#endif

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::reg_name_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::corr_dist_threshold_;

public:
  FastGICP();
  virtual ~FastGICP() override;

  void setNumThreads(int n);
  void setCorrespondenceRandomness(int k);
  void setRegularizationMethod(RegularizationMethod method);

  virtual void swapSourceAndTarget() override;
  virtual void clearSource() override;
  virtual void clearTarget() override;

  virtual void setInputSource(const PointCloudSourceConstPtr& cloud) override;
  virtual void setSourceCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs);
  virtual void setInputTarget(const PointCloudTargetConstPtr& cloud) override;
  virtual void setTargetCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs);

  const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& getSourceCovariances() const {
    return source_covs_;
  }

  const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& getTargetCovariances() const {
    return target_covs_;
  }

protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

  virtual void update_correspondences(const Eigen::Isometry3d& trans);

  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) override;

  virtual double compute_error(const Eigen::Isometry3d& trans) override;

  template<typename PointT>
  bool calculate_covariances(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, pcl::search::Search<PointT>& kdtree, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covariances);
  // rosiwit local patch (2026-11-27): 目标协方差按点位置复用 (见文件头 detail::TargetCovCache)
  template<typename PointT>
  void calculate_target_covariances(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, pcl::search::Search<PointT>& kdtree, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covariances);
  template<typename PointT>
  static Eigen::Matrix3d compute_point_covariance(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, pcl::search::Search<PointT>& kdtree, int k, RegularizationMethod method, int i, float* d_k_sq = nullptr);

protected:
  int num_threads_;
  int k_correspondences_;

  RegularizationMethod regularization_method_;

  std::shared_ptr<SearchMethodSource> search_source_;
  std::shared_ptr<SearchMethodTarget> search_target_;

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> source_covs_;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> target_covs_;

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> mahalanobis_;

  std::vector<int> correspondences_;
  std::vector<float> sq_distances_;
};
}  // namespace fast_gicp

#endif
