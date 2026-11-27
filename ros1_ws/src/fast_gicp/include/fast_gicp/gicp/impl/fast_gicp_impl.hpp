#ifndef FAST_GICP_FAST_GICP_IMPL_HPP
#define FAST_GICP_FAST_GICP_IMPL_HPP

#include <fast_gicp/so3/so3.hpp>

namespace fast_gicp {

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::FastGICP() {
#ifdef _OPENMP
  num_threads_ = omp_get_max_threads();
#else
  num_threads_ = 1;
#endif

  k_correspondences_ = 20;
  reg_name_ = "FastGICP";
  corr_dist_threshold_ = std::numeric_limits<float>::max();

  regularization_method_ = RegularizationMethod::PLANE;
  search_source_.reset(new SearchMethodSource);
  search_target_.reset(new SearchMethodTarget);
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::~FastGICP() {}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
void FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::setNumThreads(int n) {
  num_threads_ = n;

#ifdef _OPENMP
  if (n == 0) {
    num_threads_ = omp_get_max_threads();
  }
#endif
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
void FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::setCorrespondenceRandomness(int k) {
  k_correspondences_ = k;
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
void FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::setRegularizationMethod(RegularizationMethod method) {
  regularization_method_ = method;
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
void FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::swapSourceAndTarget() {
  input_.swap(target_);
  search_source_.swap(search_target_);
  source_covs_.swap(target_covs_);

  correspondences_.clear();
  sq_distances_.clear();
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
void FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::clearSource() {
  input_.reset();
  source_covs_.clear();
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
void FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::clearTarget() {
  target_.reset();
  target_covs_.clear();
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
void FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::setInputSource(const PointCloudSourceConstPtr& cloud) {
  if (input_ == cloud) {
    return;
  }

  pcl::Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);
  search_source_->setInputCloud(cloud);
  source_covs_.clear();
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
void FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::setInputTarget(const PointCloudTargetConstPtr& cloud) {
  if (target_ == cloud) {
    return;
  }
  pcl::Registration<PointSource, PointTarget, Scalar>::setInputTarget(cloud);
  search_target_->setInputCloud(cloud);
  target_covs_.clear();
  // rosiwit local patch (2026-11-27): 目标协方差在这里算完, 不留到第一次 align 里懒算。
  // 9 万点的目标每个点查 20 近邻, 4 线程约 200ms。之前放在 computeTransformation 里, 调用方在后台线程
  // 里建好的"备用"配准器其实只建了一半: 切换后跟踪线程的第一帧要替它算这 200ms —— 每换一次目标
  // 就有一帧 200ms+ (真实数据回放: 目标重建成簇时 100 帧窗口 p50 392ms)。setInputTarget 在哪个线程调,
  // 这笔钱就在哪个线程付; 目标没变 (指针相同) 时不算。
  if (target_ && !target_->empty()) {
    // 按点位置复用上一次目标里算过的协方差, 只算新来的点 (2026-11-27, 见 calculate_target_covariances)
    calculate_target_covariances(target_, *search_target_, target_covs_);
  }
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
void FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::setSourceCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs) {
  source_covs_ = covs;
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
void FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::setTargetCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs) {
  target_covs_ = covs;
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
void FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::computeTransformation(PointCloudSource& output, const Matrix4& guess) {
  if (output.points.data() == input_->points.data() || output.points.data() == target_->points.data()) {
    throw std::invalid_argument("FastGICP: destination cloud cannot be identical to source or target");
  }
  if (source_covs_.size() != input_->size()) {
    calculate_covariances(input_, *search_source_, source_covs_);
  }
  if (target_covs_.size() != target_->size()) {
    calculate_target_covariances(target_, *search_target_, target_covs_);
  }

  LsqRegistration<PointSource, PointTarget>::computeTransformation(output, guess);
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
void FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::update_correspondences(const Eigen::Isometry3d& trans) {
  assert(source_covs_.size() == input_->size());
  assert(target_covs_.size() == target_->size());

  Eigen::Isometry3f trans_f = trans.cast<float>();

  correspondences_.resize(input_->size());
  sq_distances_.resize(input_->size());
  mahalanobis_.resize(input_->size());

  std::vector<int> k_indices(1);
  std::vector<float> k_sq_dists(1);

#pragma omp parallel for num_threads(num_threads_) firstprivate(k_indices, k_sq_dists) schedule(guided, 8)
  for (int i = 0; i < input_->size(); i++) {
    PointTarget pt;
    pt.getVector4fMap() = trans_f * input_->at(i).getVector4fMap();

    search_target_->nearestKSearch(pt, 1, k_indices, k_sq_dists);

    sq_distances_[i] = k_sq_dists[0];
    correspondences_[i] = k_sq_dists[0] < corr_dist_threshold_ * corr_dist_threshold_ ? k_indices[0] : -1;

    if (correspondences_[i] < 0) {
      continue;
    }

    const int target_index = correspondences_[i];
    const auto& cov_A = source_covs_[i];
    const auto& cov_B = target_covs_[target_index];

    Eigen::Matrix4d RCR = cov_B + trans.matrix() * cov_A * trans.matrix().transpose();
    RCR(3, 3) = 1.0;

    mahalanobis_[i] = RCR.inverse();
    mahalanobis_[i](3, 3) = 0.0f;
  }
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
double FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {
  update_correspondences(trans);

  double sum_errors = 0.0;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs(num_threads_);
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs(num_threads_);
  for (int i = 0; i < num_threads_; i++) {
    Hs[i].setZero();
    bs[i].setZero();
  }

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
  for (int i = 0; i < input_->size(); i++) {
    int target_index = correspondences_[i];
    if (target_index < 0) {
      continue;
    }

    const Eigen::Vector4d mean_A = input_->at(i).getVector4fMap().template cast<double>();
    const auto& cov_A = source_covs_[i];

    const Eigen::Vector4d mean_B = target_->at(target_index).getVector4fMap().template cast<double>();
    const auto& cov_B = target_covs_[target_index];

    const Eigen::Vector4d transed_mean_A = trans * mean_A;
    const Eigen::Vector4d error = mean_B - transed_mean_A;

    sum_errors += error.transpose() * mahalanobis_[i] * error;

    if (H == nullptr || b == nullptr) {
      continue;
    }

    Eigen::Matrix<double, 4, 6> dtdx0 = Eigen::Matrix<double, 4, 6>::Zero();
    dtdx0.block<3, 3>(0, 0) = skewd(transed_mean_A.head<3>());
    dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> jlossexp = dtdx0;

    Eigen::Matrix<double, 6, 6> Hi = jlossexp.transpose() * mahalanobis_[i] * jlossexp;
    Eigen::Matrix<double, 6, 1> bi = jlossexp.transpose() * mahalanobis_[i] * error;

    Hs[omp_get_thread_num()] += Hi;
    bs[omp_get_thread_num()] += bi;
  }

  if (H && b) {
    H->setZero();
    b->setZero();
    for (int i = 0; i < num_threads_; i++) {
      (*H) += Hs[i];
      (*b) += bs[i];
    }
  }

  return sum_errors;
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
double FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::compute_error(const Eigen::Isometry3d& trans) {
  double sum_errors = 0.0;

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
  for (int i = 0; i < input_->size(); i++) {
    int target_index = correspondences_[i];
    if (target_index < 0) {
      continue;
    }

    const Eigen::Vector4d mean_A = input_->at(i).getVector4fMap().template cast<double>();
    const auto& cov_A = source_covs_[i];

    const Eigen::Vector4d mean_B = target_->at(target_index).getVector4fMap().template cast<double>();
    const auto& cov_B = target_covs_[target_index];

    const Eigen::Vector4d transed_mean_A = trans * mean_A;
    const Eigen::Vector4d error = mean_B - transed_mean_A;

    sum_errors += error.transpose() * mahalanobis_[i] * error;
  }

  return sum_errors;
}

// rosiwit local patch (2026-11-27): 单点协方差 (原 calculate_covariances 的循环体, 拆出来供复用路径调用)
template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
template <typename PointT>
Eigen::Matrix3d FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::compute_point_covariance(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud, pcl::search::Search<PointT>& kdtree, int k, RegularizationMethod method, int i, float* d_k_sq) {
  std::vector<int> k_indices;
  std::vector<float> k_sq_distances;
  kdtree.nearestKSearch(cloud->at(i), k, k_indices, k_sq_distances);
  if (d_k_sq) *d_k_sq = k_sq_distances.empty() ? 0.f : k_sq_distances.back();

  Eigen::Matrix<double, 4, -1> neighbors(4, k_indices.size());
  for (int j = 0; j < k_indices.size(); j++) {
    neighbors.col(j) = cloud->at(k_indices[j]).getVector4fMap().template cast<double>();
  }
  neighbors.colwise() -= neighbors.rowwise().mean().eval();
  Eigen::Matrix4d cov = neighbors * neighbors.transpose() / k_indices.size();

  if (method == RegularizationMethod::NONE) {
    return cov.block<3, 3>(0, 0);
  } else if (method == RegularizationMethod::FROBENIUS) {
    const double lambda = 1e-3;
    Eigen::Matrix3d C = cov.block<3, 3>(0, 0).cast<double>() + lambda * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d C_inv = C.inverse();
    return (C_inv / C_inv.norm()).inverse();
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector3d values;
  switch (method) {
    default:
      std::cerr << "here must not be reached" << std::endl;
      abort();
    case RegularizationMethod::PLANE:
      values = Eigen::Vector3d(1, 1, 1e-3);
      break;
    case RegularizationMethod::MIN_EIG:
      values = svd.singularValues().array().max(1e-3);
      break;
    case RegularizationMethod::NORMALIZED_MIN_EIG:
      values = svd.singularValues() / svd.singularValues().maxCoeff();
      values = values.array().max(1e-3);
      break;
  }
  return svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
}

template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
template <typename PointT>
bool FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::calculate_covariances(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud, pcl::search::Search<PointT>& kdtree,
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covariances) {
  if (kdtree.getInputCloud() != cloud) {
    kdtree.setInputCloud(cloud);
  }
  covariances.resize(cloud->size());

#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (int i = 0; i < cloud->size(); i++) {
    covariances[i].setZero();
    covariances[i].template block<3, 3>(0, 0) = compute_point_covariance(cloud, kdtree, k_correspondences_, regularization_method_, i);
  }

  return true;
}

// rosiwit local patch (2026-11-27): 目标协方差按点位置复用 (说明见 fast_gicp.hpp 里的 detail::TargetCovCache)。
template <typename PointSource, typename PointTarget, typename SearchMethodSource, typename SearchMethodTarget>
template <typename PointT>
void FastGICP<PointSource, PointTarget, SearchMethodSource, SearchMethodTarget>::calculate_target_covariances(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud, pcl::search::Search<PointT>& kdtree,
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covariances) {
  detail::TargetCovCache& cache = targetCovCache<PointT>();
  if (!cache.enabled.load() || regularization_method_ != RegularizationMethod::PLANE || cloud->empty()) {
    calculate_covariances(cloud, kdtree, covariances);
    return;
  }
  if (kdtree.getInputCloud() != cloud) {
    kdtree.setInputCloud(cloud);
  }
  covariances.resize(cloud->size());
  const double eps = 1e-3;

  // 1) 查表 (单线程, 持锁): 命中的先按缓存值填上, 未命中的记下来。
  //    命中只是"位置见过", 还不能直接用: 得核对这次新来的点有没有落进它的近邻半径 (见 2)。
  std::vector<int> miss;                        // 新位置 (未命中) 的点下标
  std::vector<int> hit_pt;                      // 命中的点下标
  std::vector<float> hit_d20;                   // 命中点当初的第 k 近邻距离平方 (持锁时拷出来, 之后不再碰缓存)
  uint32_t gen = 0;
  {
    std::lock_guard<std::mutex> lock(cache.mu);
    if (cache.k != k_correspondences_) {
      cache.index.clear();
      cache.entries.clear();
      cache.k = k_correspondences_;
    }
    gen = ++cache.gen;
    miss.reserve(cloud->size() / 4);
    hit_pt.reserve(cloud->size());
    hit_d20.reserve(cloud->size());
    for (int i = 0; i < static_cast<int>(cloud->size()); i++) {
      const auto& p = cloud->points[i];
      const auto it = cache.index.find(targetCovKey(p.x, p.y, p.z));
      if (it == cache.index.end()) {
        miss.push_back(i);
        continue;
      }
      detail::TargetCovCache::Entry& e = cache.entries[it->second];
      e.used = gen;
      hit_pt.push_back(i);
      hit_d20.push_back(e.d20);
      covariances[i].setZero();
      covariances[i].block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() - (1.0 - eps) * e.normal * e.normal.transpose();
    }
    cache.last_hits = hit_pt.size();
    cache.last_miss = miss.size();
    cache.last_redo = 0;
  }

  if (miss.empty()) return;                     // 没有新点: 所有近邻集合都不可能变, 全部沿用

  // 2a) 命中点里, 近邻半径内落了新点的挑出来重算 (新点 = 上面未命中的那些点)
  const int nth = std::max(1, num_threads_);
  typename pcl::PointCloud<PointT>::Ptr fresh(new pcl::PointCloud<PointT>());
  fresh->reserve(miss.size());
  for (int i : miss) fresh->push_back(cloud->points[i]);
  pcl::search::KdTree<PointT> fresh_tree;
  fresh_tree.setInputCloud(fresh);
  {
    std::vector<int> warm_i;
    std::vector<float> warm_d;
    fresh_tree.nearestKSearch(fresh->points[0], 1, warm_i, warm_d);   // 索引若懒建, 在单线程里先建好
  }
  std::vector<std::vector<int>> redo_parts(nth);
#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (int j = 0; j < static_cast<int>(hit_pt.size()); j++) {
    const float r = std::sqrt(hit_d20[j]);
    std::vector<int> idx;
    std::vector<float> d2;
    if (r > 0.f && fresh_tree.radiusSearch(cloud->points[hit_pt[j]], r, idx, d2) > 0) {
      redo_parts[std::min(nth - 1, omp_get_thread_num())].push_back(hit_pt[j]);
    }
  }
  std::vector<int> redo;                        // 近邻集合变了的命中点 (点下标)
  for (const auto& v : redo_parts) redo.insert(redo.end(), v.begin(), v.end());

  // 2b) 算: 新位置 + 近邻集合变了的命中点 (并行, 不碰缓存)
  std::vector<float> d_k(cloud->size(), 0.f);
#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (int j = 0; j < static_cast<int>(miss.size()); j++) {
    const int i = miss[j];
    covariances[i].setZero();
    covariances[i].block<3, 3>(0, 0) = compute_point_covariance(cloud, kdtree, k_correspondences_, regularization_method_, i, &d_k[i]);
  }
#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (int j = 0; j < static_cast<int>(redo.size()); j++) {
    const int i = redo[j];
    covariances[i].setZero();
    covariances[i].block<3, 3>(0, 0) = compute_point_covariance(cloud, kdtree, k_correspondences_, regularization_method_, i, &d_k[i]);
  }

  // 3) 入表 (持锁)。条目下标只在持锁期间有效: 一律按位置键现查, 不用之前记下的下标
  //    (并发的另一次构建可能已经插了新条目、甚至触发过容量回收)。
  {
    std::lock_guard<std::mutex> lock(cache.mu);
    cache.last_redo = redo.size();
    cache.entries.reserve(cache.entries.size() + miss.size());
    for (int i : miss) {
      const auto& p = cloud->points[i];
      const int64_t key = targetCovKey(p.x, p.y, p.z);
      if (cache.index.find(key) != cache.index.end()) continue;   // 并发的另一次构建刚插过
      // 法向 = 协方差最小特征值方向 (PLANE: 奇异值 eps 对应的那一列)
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariances[i].block<3, 3>(0, 0));
      detail::TargetCovCache::Entry e;
      e.key = key;
      e.normal = es.eigenvectors().col(0);
      e.d20 = d_k[i];
      e.used = gen;
      cache.index.emplace(key, static_cast<uint32_t>(cache.entries.size()));
      cache.entries.push_back(e);
    }
    for (int i : redo) {
      const auto& p = cloud->points[i];
      const auto it = cache.index.find(targetCovKey(p.x, p.y, p.z));
      if (it == cache.index.end()) continue;
      detail::TargetCovCache::Entry& e = cache.entries[it->second];
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariances[i].block<3, 3>(0, 0));
      e.normal = es.eigenvectors().col(0);
      e.d20 = d_k[i];
      e.used = gen;
    }
    // 4) 容量: 超上限时丢掉最近两代都没用到的条目
    if (cache.entries.size() > detail::TargetCovCache::kCap) {
      std::vector<detail::TargetCovCache::Entry> kept;
      kept.reserve(cloud->size());
      for (const auto& e : cache.entries) {
        if (e.used + 1 >= gen) kept.push_back(e);
      }
      cache.index.clear();
      cache.entries.swap(kept);
      for (uint32_t n = 0; n < cache.entries.size(); ++n) cache.index.emplace(cache.entries[n].key, n);
    }
  }
}

}  // namespace fast_gicp

#endif
