// src/nav_core/distance_field.cpp
#include "rosiwit_navigation/nav_core/distance_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rosiwit_navigation
{
namespace core
{

namespace
{

constexpr double kInf = 1e20;

/// 一维平方距离变换: d[q] = min_p ( (q-p)^2 + f[p] )
/// f 为输入 (0 = 该处是源点), n = 长度。内部用 double, 避免 4M 格地图上溢出。
void dt1d(const double * f, double * d, int n, std::vector<int> & v,
          std::vector<double> & z)
{
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + static_cast<double>(q) * q) -
                (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
               (2.0 * q - 2.0 * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + static_cast<double>(q) * q) -
           (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
          (2.0 * q - 2.0 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) { ++k; }
    const double dq = static_cast<double>(q - v[k]);
    d[q] = dq * dq + f[v[k]];
  }
}

}  // namespace

std::vector<float> euclideanDistanceCells(
    const std::vector<uint8_t> & lethal, unsigned int width, unsigned int height)
{
  const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
  std::vector<float> out(n, 0.0f);
  if (n == 0 || lethal.size() != n) { return out; }

  std::vector<double> field(n);
  for (size_t i = 0; i < n; ++i) {
    field[i] = lethal[i] ? 0.0 : kInf;
  }

  const int max_dim = static_cast<int>(std::max(width, height));
  std::vector<double> f(static_cast<size_t>(max_dim));
  std::vector<double> d(static_cast<size_t>(max_dim));
  std::vector<int> v(static_cast<size_t>(max_dim));
  std::vector<double> z(static_cast<size_t>(max_dim) + 1);

  // 先沿 y (列), 再沿 x (行)
  for (unsigned int x = 0; x < width; ++x) {
    for (unsigned int y = 0; y < height; ++y) {
      f[y] = field[static_cast<size_t>(y) * width + x];
    }
    dt1d(f.data(), d.data(), static_cast<int>(height), v, z);
    for (unsigned int y = 0; y < height; ++y) {
      field[static_cast<size_t>(y) * width + x] = d[y];
    }
  }
  for (unsigned int y = 0; y < height; ++y) {
    const size_t row = static_cast<size_t>(y) * width;
    for (unsigned int x = 0; x < width; ++x) { f[x] = field[row + x]; }
    dt1d(f.data(), d.data(), static_cast<int>(width), v, z);
    for (unsigned int x = 0; x < width; ++x) {
      const double sq = d[x];
      out[row + x] = sq >= kInf ? 1e6f : static_cast<float>(std::sqrt(sq));
    }
  }
  return out;
}

}  // namespace core
}  // namespace rosiwit_navigation
