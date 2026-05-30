/**
 * @file cylinder_fit.hpp
 * @brief Vertical-axis cylinder fit for trunk columns (Sprint 4).
 */

#ifndef FOREST_3D_PERCEPTION__CYLINDER_FIT_HPP_
#define FOREST_3D_PERCEPTION__CYLINDER_FIT_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace forest_3d_perception
{

struct CylinderObservation
{
  float cx{0.0f};
  float cy{0.0f};
  float z_base{0.0f};
  float height{0.0f};
  float radius{0.0f};
  float rmse{0.0f};
  float inlier_ratio{0.0f};
  std::size_t n_points{0};
  bool valid{false};
};

enum class CylinderReject
{
  Accepted,
  TooFewPoints,
  TooShort,
  TooWide,
  HighRmse,
  LowInliers,
};

inline bool observation_is_finite(const CylinderObservation & o)
{
  return std::isfinite(o.cx) && std::isfinite(o.cy) && std::isfinite(o.z_base) &&
         std::isfinite(o.height) && std::isfinite(o.radius) && std::isfinite(o.rmse) &&
         o.height > 0.0f && o.radius > 0.0f;
}

inline CylinderReject fit_vertical_cylinder(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const std::vector<std::size_t> & indices,
  CylinderObservation & out,
  double min_height_m,
  double max_radius_m,
  double max_rmse_m,
  double min_inlier_ratio,
  double inlier_dist_m,
  double max_trunk_slice_height_m = 2.5)
{
  if (indices.size() < 5) {
    return CylinderReject::TooFewPoints;
  }

  std::vector<float> zs;
  zs.reserve(indices.size());
  for (std::size_t idx : indices) {
    zs.push_back(cloud.points[idx].z);
  }
  std::sort(zs.begin(), zs.end());
  const float z_base = zs[std::min(zs.size() - 1, zs.size() / 10)];
  const float z_top_cap = z_base + static_cast<float>(max_trunk_slice_height_m);
  float z_max = z_base;

  std::vector<std::size_t> fit_indices;
  fit_indices.reserve(indices.size());
  for (std::size_t idx : indices) {
    const float z = cloud.points[idx].z;
    z_max = std::max(z_max, z);
    if (z <= z_top_cap) {
      fit_indices.push_back(idx);
    }
  }
  if (fit_indices.size() < 5) {
    fit_indices = indices;
  }

  double sum_x = 0.0;
  double sum_y = 0.0;
  for (std::size_t idx : fit_indices) {
    const auto & p = cloud.points[idx];
    sum_x += p.x;
    sum_y += p.y;
  }
  const double inv_n = 1.0 / static_cast<double>(fit_indices.size());
  const float cx = static_cast<float>(sum_x * inv_n);
  const float cy = static_cast<float>(sum_y * inv_n);
  const float height = std::min(z_max, z_top_cap) - z_base;

  if (height < static_cast<float>(min_height_m)) {
    return CylinderReject::TooShort;
  }

  std::vector<float> radii;
  radii.reserve(fit_indices.size());
  for (std::size_t idx : fit_indices) {
    const auto & p = cloud.points[idx];
    const float dx = p.x - cx;
    const float dy = p.y - cy;
    radii.push_back(std::sqrt(dx * dx + dy * dy));
  }
  std::sort(radii.begin(), radii.end());
  const float radius = radii[radii.size() / 2];

  if (radius > static_cast<float>(max_radius_m)) {
    return CylinderReject::TooWide;
  }

  double sum_err2 = 0.0;
  std::size_t inliers = 0;
  for (float r : radii) {
    const double e = static_cast<double>(r - radius);
    sum_err2 += e * e;
    if (std::abs(e) <= inlier_dist_m) {
      ++inliers;
    }
  }
  const float rmse = static_cast<float>(std::sqrt(sum_err2 * inv_n));
  const float inlier_ratio =
    static_cast<float>(inliers) / static_cast<float>(fit_indices.size());

  if (rmse > static_cast<float>(max_rmse_m)) {
    return CylinderReject::HighRmse;
  }
  if (inlier_ratio < static_cast<float>(min_inlier_ratio)) {
    return CylinderReject::LowInliers;
  }

  out.cx = cx;
  out.cy = cy;
  out.z_base = z_base;
  out.height = height;
  out.radius = radius;
  out.rmse = rmse;
  out.inlier_ratio = inlier_ratio;
  out.n_points = indices.size();
  out.valid = observation_is_finite(out);
  return out.valid ? CylinderReject::Accepted : CylinderReject::HighRmse;
}

}  // namespace forest_3d_perception

#endif  // FOREST_3D_PERCEPTION__CYLINDER_FIT_HPP_
