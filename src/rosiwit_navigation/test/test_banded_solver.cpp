#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "rosiwit_navigation/nav_core/banded_solver.hpp"

namespace
{
using rosiwit_navigation::core::BlockTridiagonal;
using rosiwit_navigation::core::solveBlockTridiagonal;

std::array<double, 9> diagBlock(double d)
{
  return {d, 0, 0, 0, d, 0, 0, 0, d};
}

// 生产改动会让这些测试失败的情形:
//   * 消元/回代顺序写错 -> 解与稠密参考不一致 (CoupledMatchesDense)
//   * lambda 没加到对角线上 -> DampingShrinksStep 失败
//   * 奇异矩阵不返回 false -> SingularMatrixReportsFailure 失败
TEST(BandedSolver, DiagonalSystemIsScaled)
{
  BlockTridiagonal sys;
  sys.n = 2;
  sys.diag = {diagBlock(2.0), diagBlock(4.0)};
  sys.upper = {diagBlock(0.0)};
  sys.rhs = {1, 0, 0, 2, 0, 0};

  std::vector<double> d;
  ASSERT_TRUE(solveBlockTridiagonal(sys, 0.0, &d));
  ASSERT_EQ(d.size(), 6u);
  // (H)d = -rhs  ->  x0 = -0.5, x3 = -0.5
  EXPECT_NEAR(d[0], -0.5, 1e-12);
  EXPECT_NEAR(d[3], -0.5, 1e-12);
  EXPECT_NEAR(d[1], 0.0, 1e-12);
}

TEST(BandedSolver, CoupledMatchesDense)
{
  // 3 个顶点, 每个 3 维; H 对称块三对角
  BlockTridiagonal sys;
  sys.n = 3;
  sys.diag = {
    {2.0, 0.1, 0.0, 0.1, 2.5, 0.0, 0.0, 0.0, 1.5},
    {3.0, 0.0, 0.2, 0.0, 3.0, 0.0, 0.2, 0.0, 2.0},
    {2.2, 0.0, 0.0, 0.0, 2.2, 0.0, 0.0, 0.0, 1.8}};
  sys.upper = {
    {-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -0.5},
    {-0.7, 0.0, 0.3, 0.0, -0.7, 0.0, 0.3, 0.0, -0.4}};
  sys.rhs = {1, -2, 3, -1, 1, 0, 2, -3, 1};

  // 稠密参考解 (对称: lower = upper^T)
  const int N = 9;
  std::vector<double> A(N * N, 0.0);
  auto at = [&](int i, int j) -> double & { return A[i * N + j]; };
  auto put3 = [&](int bi, int bj, const std::array<double, 9> & B) {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) { at(bi * 3 + r, bj * 3 + c) = B[r * 3 + c]; }
    }
  };
  for (int i = 0; i < 3; ++i) { put3(i, i, sys.diag[i]); }
  for (int i = 0; i + 1 < 3; ++i) { put3(i, i + 1, sys.upper[i]); }
  for (int i = 0; i + 1 < 3; ++i) {  // lower = upper^T: 元素 (i+1,i)[c][r] = upper[i][r][c]
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) { at((i + 1) * 3 + c, i * 3 + r) = sys.upper[i][r * 3 + c]; }
    }
  }
  std::vector<double> b(N);
  for (int i = 0; i < N; ++i) { b[i] = -sys.rhs[i]; }

  // 稠密高斯消元 (部分主元)
  for (int k = 0; k < N; ++k) {
    int piv = k;
    for (int i = k + 1; i < N; ++i) {
      if (std::fabs(at(i, k)) > std::fabs(at(piv, k))) { piv = i; }
    }
    if (piv != k) {
      for (int j = 0; j < N; ++j) { std::swap(at(k, j), at(piv, j)); }
      std::swap(b[k], b[piv]);
    }
    for (int i = k + 1; i < N; ++i) {
      const double f = at(i, k) / at(k, k);
      for (int j = k; j < N; ++j) { at(i, j) -= f * at(k, j); }
      b[i] -= f * b[k];
    }
  }
  std::vector<double> ref(N, 0.0);
  for (int i = N - 1; i >= 0; --i) {
    double s = b[i];
    for (int j = i + 1; j < N; ++j) { s -= at(i, j) * ref[j]; }
    ref[i] = s / at(i, i);
  }

  std::vector<double> d;
  ASSERT_TRUE(solveBlockTridiagonal(sys, 0.0, &d));
  for (int i = 0; i < N; ++i) {
    EXPECT_NEAR(d[i], ref[i], 1e-9) << "index " << i;
  }
}

TEST(BandedSolver, DampingShrinksStep)
{
  BlockTridiagonal sys;
  sys.n = 2;
  sys.diag = {diagBlock(1.0), diagBlock(1.0)};
  sys.upper = {diagBlock(0.0)};
  sys.rhs = {1, 1, 1, 1, 1, 1};

  std::vector<double> small_lam;
  std::vector<double> big_lam;
  ASSERT_TRUE(solveBlockTridiagonal(sys, 0.0, &small_lam));
  ASSERT_TRUE(solveBlockTridiagonal(sys, 9.0, &big_lam));
  double n1 = 0.0, n2 = 0.0;
  for (size_t i = 0; i < small_lam.size(); ++i) {
    n1 += small_lam[i] * small_lam[i];
    n2 += big_lam[i] * big_lam[i];
  }
  EXPECT_LT(n2, n1);
}

TEST(BandedSolver, SingularMatrixReportsFailure)
{
  BlockTridiagonal sys;
  sys.n = 2;
  sys.diag = {diagBlock(0.0), diagBlock(0.0)};
  sys.upper = {diagBlock(0.0)};
  sys.rhs = {1, 0, 0, 1, 0, 0};
  std::vector<double> d;
  EXPECT_FALSE(solveBlockTridiagonal(sys, 0.0, &d));
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
