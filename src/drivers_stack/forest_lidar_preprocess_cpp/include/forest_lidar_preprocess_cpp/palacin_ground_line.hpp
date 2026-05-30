// Palacín-style ground line in (x, z) — classification_frame, gravity-aligned.
// z_ground(x) = m*x + b; classify by vertical residual vs band/hole/obstacle thresholds.

#ifndef FOREST_LIDAR_PREPROCESS_CPP__PALACIN_GROUND_LINE_HPP_
#define FOREST_LIDAR_PREPROCESS_CPP__PALACIN_GROUND_LINE_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <utility>
#include <vector>

namespace forest_lidar_preprocess_cpp
{

struct GroundLine2D
{
  double m{0.0};
  double b{0.0};
  bool valid{false};
  std::size_t inliers{0};
};

inline double ground_z_at(const GroundLine2D & line, double x)
{
  return line.m * x + line.b;
}

inline double residual_z(const GroundLine2D & line, double x, double z)
{
  return z - ground_z_at(line, x);
}

inline bool fit_line_from_two_points(
  double x0, double z0, double x1, double z1, double & m, double & b)
{
  const double dx = x1 - x0;
  if (std::abs(dx) < 1e-6) {
    return false;
  }
  m = (z1 - z0) / dx;
  b = z0 - m * x0;
  return true;
}

inline GroundLine2D fit_ground_line_ransac(
  const std::vector<std::pair<double, double>> & xz,
  double inlier_thresh_m,
  int max_iterations,
  std::size_t min_inliers,
  unsigned int rng_seed)
{
  GroundLine2D best;
  if (xz.size() < 2) {
    return best;
  }

  std::mt19937 rng(rng_seed);
  std::uniform_int_distribution<std::size_t> dist(0, xz.size() - 1);

  std::size_t best_count = 0;
  double best_m = 0.0;
  double best_b = 0.0;

  for (int it = 0; it < max_iterations; ++it) {
    const auto & p0 = xz[dist(rng)];
    const auto & p1 = xz[dist(rng)];
    if (p0.first == p1.first && p0.second == p1.second) {
      continue;
    }

    double m = 0.0;
    double b = 0.0;
    if (!fit_line_from_two_points(p0.first, p0.second, p1.first, p1.second, m, b)) {
      continue;
    }

    std::size_t count = 0;
    for (const auto & p : xz) {
      if (std::abs(residual_z({m, b, true, 0}, p.first, p.second)) <= inlier_thresh_m) {
        ++count;
      }
    }

    if (count > best_count) {
      best_count = count;
      best_m = m;
      best_b = b;
    }
  }

  if (best_count < min_inliers) {
    return best;
  }

  // Refine with least squares on inliers
  double sum_x = 0.0;
  double sum_z = 0.0;
  double sum_xx = 0.0;
  double sum_xz = 0.0;
  std::size_t n = 0;
  for (const auto & p : xz) {
    if (std::abs(residual_z({best_m, best_b, true, 0}, p.first, p.second)) > inlier_thresh_m) {
      continue;
    }
    sum_x += p.first;
    sum_z += p.second;
    sum_xx += p.first * p.first;
    sum_xz += p.first * p.second;
    ++n;
  }

  if (n >= 2) {
    const double denom = n * sum_xx - sum_x * sum_x;
    if (std::abs(denom) > 1e-9) {
      best_m = (n * sum_xz - sum_x * sum_z) / denom;
      best_b = (sum_z - best_m * sum_x) / static_cast<double>(n);
    }
  }

  best.m = best_m;
  best.b = best_b;
  best.valid = true;
  best.inliers = best_count;
  return best;
}

inline GroundLine2D fit_ground_line_min_z(
  const std::vector<std::pair<double, double>> & xz,
  double z_offset_m)
{
  GroundLine2D line;
  line.m = 0.0;
  line.b = std::numeric_limits<double>::infinity();
  for (const auto & p : xz) {
    line.b = std::min(line.b, p.second);
  }
  if (!std::isfinite(line.b)) {
    return line;
  }
  line.b += z_offset_m;
  line.valid = true;
  line.inliers = xz.size();
  return line;
}

}  // namespace forest_lidar_preprocess_cpp

#endif  // FOREST_LIDAR_PREPROCESS_CPP__PALACIN_GROUND_LINE_HPP_
