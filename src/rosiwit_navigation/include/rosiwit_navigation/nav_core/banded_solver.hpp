// ============================================================================
// banded_solver.hpp — 块三对角 (3x3 块) 线性求解器
//
// 用途: TEB 的 Gauss-Newton/LM 每拍要解
//   (H + lambda*diag(H)) * d = -g
// 其中 H 是对称块三对角 (每个弹性带顶点 3 维: x, y, theta), 顶点数 40~80。
// 稠密解是 O(n^3), 这里用块形式的 Thomas 消元: O(n * 3^3), n=80 时 <0.1ms。
//
// 为什么不用 Eigen/g2o: 本仓库要求不引入新第三方依赖, 且链式结构用不上通用稀疏求解器。
// ============================================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE__BANDED_SOLVER_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE__BANDED_SOLVER_HPP_

#include <array>
#include <cmath>
#include <vector>

namespace rosiwit_navigation
{
namespace core
{

/// 对称块三对角系统: diag[i] 是 (i,i) 的 3x3, upper[i] 是 (i,i+1) 的 3x3 (下三角同理取转置)
struct BlockTridiagonal
{
  int n = 0;
  std::vector<std::array<double, 9>> diag;
  std::vector<std::array<double, 9>> upper;
  std::vector<double> rhs;   ///< 长度 3n, 表示 -g (解出的是增量 d)
};

namespace banded_detail
{

inline void mul33(const std::array<double, 9> & a, const std::array<double, 9> & b,
                  std::array<double, 9> & c)
{
  for (int r = 0; r < 3; ++r) {
    for (int col = 0; col < 3; ++col) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) { s += a[r * 3 + k] * b[k * 3 + col]; }
      c[r * 3 + col] = s;
    }
  }
}

/// c = a^T * b
inline void mulT33(const std::array<double, 9> & a, const std::array<double, 9> & b,
                   std::array<double, 9> & c)
{
  for (int r = 0; r < 3; ++r) {
    for (int col = 0; col < 3; ++col) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) { s += a[k * 3 + r] * b[k * 3 + col]; }
      c[r * 3 + col] = s;
    }
  }
}

inline void mul3(const std::array<double, 9> & m, const double * v, double * out)
{
  for (int r = 0; r < 3; ++r) {
    out[r] = m[r * 3] * v[0] + m[r * 3 + 1] * v[1] + m[r * 3 + 2] * v[2];
  }
}

/// out = m^T * v
inline void mulT3(const std::array<double, 9> & m, const double * v, double * out)
{
  for (int r = 0; r < 3; ++r) {
    out[r] = m[0 * 3 + r] * v[0] + m[1 * 3 + r] * v[1] + m[2 * 3 + r] * v[2];
  }
}

/// 3x3 高斯消元 (部分主元); 奇异返回 false
inline bool solve3(const std::array<double, 9> & in, const double * b, double * x)
{
  double a[3][4];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) { a[r][c] = in[r * 3 + c]; }
    a[r][3] = b[r];
  }
  for (int k = 0; k < 3; ++k) {
    int piv = k;
    for (int r = k + 1; r < 3; ++r) {
      if (std::fabs(a[r][k]) > std::fabs(a[piv][k])) { piv = r; }
    }
    if (std::fabs(a[piv][k]) < 1e-12) { return false; }
    if (piv != k) {
      for (int c = 0; c < 4; ++c) { std::swap(a[k][c], a[piv][c]); }
    }
    for (int r = k + 1; r < 3; ++r) {
      const double f = a[r][k] / a[k][k];
      for (int c = k; c < 4; ++c) { a[r][c] -= f * a[k][c]; }
    }
  }
  for (int r = 2; r >= 0; --r) {
    double s = a[r][3];
    for (int c = r + 1; c < 3; ++c) { s -= a[r][c] * x[c]; }
    x[r] = s / a[r][r];
  }
  return true;
}

}  // namespace banded_detail

/// 解 (H + lambda*diag(H)) d = -rhs; 不修改入参, 结果写入 out (长度 3n)。
/// lambda 用**乘性**阻尼: 对角元乘 (1+lambda), 因此全零对角仍然被判奇异 (不会假装可解)。
inline bool solveBlockTridiagonal(BlockTridiagonal & sys, double lambda,
                                  std::vector<double> * out)
{
  const int n = sys.n;
  if (n <= 0 || out == nullptr) { return false; }
  if (static_cast<int>(sys.diag.size()) != n ||
      static_cast<int>(sys.upper.size()) != std::max(0, n - 1) ||
      static_cast<int>(sys.rhs.size()) != 3 * n) {
    return false;
  }

  // 复制对角块并加阻尼 (入参保持不变, 允许同一系统多次求解)
  std::vector<std::array<double, 9>> a(sys.diag);
  const double damp = 1.0 + std::max(0.0, lambda);
  for (int i = 0; i < n; ++i) {
    a[i][0] *= damp;
    a[i][4] *= damp;
    a[i][8] *= damp;
  }
  std::vector<double> b(sys.rhs.size());
  for (size_t i = 0; i < sys.rhs.size(); ++i) { b[i] = -sys.rhs[i]; }   // 解 H d = -rhs

  // 前向消元: a[i] -= upper[i-1]^T * a'[i-1]^-1 * upper[i-1], b 同理
  for (int i = 1; i < n; ++i) {
    const std::array<double, 9> & up = sys.upper[i - 1];
    double col[3][3];   // a'[i-1]^-1 * upper[i-1] 的列
    for (int c = 0; c < 3; ++c) {
      const double e[3] = {up[0 * 3 + c], up[1 * 3 + c], up[2 * 3 + c]};
      double x[3];
      if (!banded_detail::solve3(a[i - 1], e, x)) { return false; }
      col[0][c] = x[0];
      col[1][c] = x[1];
      col[2][c] = x[2];
    }
    std::array<double, 9> t{};   // t = up^T * col
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        double s = 0.0;
        for (int k = 0; k < 3; ++k) { s += up[k * 3 + r] * col[k][c]; }
        t[r * 3 + c] = s;
      }
    }
    for (int k = 0; k < 9; ++k) { a[i][k] -= t[k]; }

    const double * bprev = &b[3 * (i - 1)];
    double y[3];
    if (!banded_detail::solve3(a[i - 1], bprev, y)) { return false; }
    double adj[3];
    banded_detail::mulT3(up, y, adj);
    for (int r = 0; r < 3; ++r) { b[3 * i + r] -= adj[r]; }
  }

  // 回代
  out->assign(static_cast<size_t>(3 * n), 0.0);
  double x[3];
  if (!banded_detail::solve3(a[n - 1], &b[3 * (n - 1)], x)) { return false; }
  for (int r = 0; r < 3; ++r) { (*out)[3 * (n - 1) + r] = x[r]; }
  for (int i = n - 2; i >= 0; --i) {
    double rhs_i[3] = {b[3 * i], b[3 * i + 1], b[3 * i + 2]};
    double proj[3];
    banded_detail::mul3(sys.upper[i], &(*out)[3 * (i + 1)], proj);
    for (int r = 0; r < 3; ++r) { rhs_i[r] -= proj[r]; }
    if (!banded_detail::solve3(a[i], rhs_i, x)) { return false; }
    for (int r = 0; r < 3; ++r) { (*out)[3 * i + r] = x[r]; }
  }
  return true;
}

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__BANDED_SOLVER_HPP_
