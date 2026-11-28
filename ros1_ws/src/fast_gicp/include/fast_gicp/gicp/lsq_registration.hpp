#ifndef FAST_GICP_LSQ_REGISTRATION_HPP
#define FAST_GICP_LSQ_REGISTRATION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>

namespace fast_gicp {

enum class LSQ_OPTIMIZER_TYPE { GaussNewton, LevenbergMarquardt };

template<typename PointSource, typename PointTarget>
class LsqRegistration : public pcl::Registration<PointSource, PointTarget, float> {
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
  using Ptr = pcl::shared_ptr<LsqRegistration<PointSource, PointTarget>>;
  using ConstPtr = pcl::shared_ptr<const LsqRegistration<PointSource, PointTarget>>;
#else
  using Ptr = boost::shared_ptr<LsqRegistration<PointSource, PointTarget>>;
  using ConstPtr = boost::shared_ptr<const LsqRegistration<PointSource, PointTarget>>;
#endif

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::nr_iterations_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::max_iterations_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::final_transformation_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::transformation_epsilon_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::converged_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LsqRegistration();
  virtual ~LsqRegistration();

  void setRotationEpsilon(double eps);
  void setInitialLambdaFactor(double init_lambda_factor);
  void setDebugPrint(bool lm_debug_print);

  const Eigen::Matrix<double, 6, 6>& getFinalHessian() const;

  // rosiwit local patch: 上一次 align 实际执行的外层迭代次数。pcl 的 nr_iterations_ 是最后一轮的下标
  // (0 基), 所以 +1。节点用它区分"收敛了"和"跑满上限" —— 每轮迭代 = 一遍全量最近邻关联 + 线性化,
  // 是求解器成本的主项 (实测 5000 点每轮约 20ms 单线程)。
  int performedIterations() const { return static_cast<int>(nr_iterations_) + 1; }

  // rosiwit local patch (2026-11-28): 单帧求解时间预算。第一轮总是执行, 之后每轮
  // 开始前看表; 超预算就停并把 time_limited 置位。0 = 关闭 (原行为)。
  void setTimeLimitMs(double ms) { time_limit_ms_ = ms; }
  bool wasTimeLimited() const { return time_limited_; }

  double evaluateCost(const Eigen::Matrix4f& relative_pose, Eigen::Matrix<double, 6, 6>* H = nullptr, Eigen::Matrix<double, 6, 1>* b = nullptr);

  virtual void swapSourceAndTarget() {}
  virtual void clearSource() {}
  virtual void clearTarget() {}

protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

  // rosiwit local patch: 收敛判据在雷达自身位置上量 (见 impl 里的说明)。x0 = 本轮迭代的起点位姿。
  bool is_converged(const Eigen::Isometry3d& delta, const Eigen::Isometry3d& x0) const;

  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H = nullptr, Eigen::Matrix<double, 6, 1>* b = nullptr) = 0;
  virtual double compute_error(const Eigen::Isometry3d& trans) = 0;

  bool step_optimize(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta);
  bool step_gn(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta);
  bool step_lm(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta);

protected:
  double rotation_epsilon_;

  LSQ_OPTIMIZER_TYPE lsq_optimizer_type_;
  int lm_max_iterations_;
  double lm_init_lambda_factor_;
  double lm_lambda_;
  bool lm_debug_print_;
  double time_limit_ms_ = 0.0;   // 单帧求解预算 (ms), 0=off
  bool   time_limited_ = false;  // 上一次 align 是否因预算停止

  Eigen::Matrix<double, 6, 6> final_hessian_;
};
}  // namespace fast_gicp

#endif