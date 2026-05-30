/**
 * @file ndsm_field.hpp
 * @brief Normalized DSM: height above terrain reference per point (Sprint 2).
 */

#ifndef FOREST_3D_PERCEPTION__NDSM_FIELD_HPP_
#define FOREST_3D_PERCEPTION__NDSM_FIELD_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/terrain_grid_2d.hpp"

namespace forest_3d_perception
{

struct NdsmPoint
{
  std::size_t index{0};
  float h{0.0f};
  float z_ground{0.0f};
};

struct NdsmStats
{
  std::size_t n_valid{0};
  std::size_t n_trunk_band{0};
  std::size_t n_skip_nan_ground{0};
  std::size_t n_skip_height_band{0};
  double mean_h_trunk_band{0.0};
};

/** Per-point height above local ground reference from TerrainGrid2D. */
class NdsmField
{
public:
  static std::vector<NdsmPoint> compute_trunk_band(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const TerrainGrid2D & grid,
    float h_min_m,
    float h_max_m,
    NdsmStats * stats = nullptr)
  {
    std::vector<NdsmPoint> band;
    band.reserve(cloud.size() / 4);
    double sum_h = 0.0;

    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const auto & p = cloud.points[i];
      const float zg = grid.ndsm_ground_reference_at(p.x, p.y);
      if (std::isnan(zg)) {
        if (stats) {
          ++stats->n_skip_nan_ground;
        }
        continue;
      }
      const float h = p.z - zg;
      if (h < h_min_m || h > h_max_m) {
        if (stats) {
          ++stats->n_skip_height_band;
        }
        continue;
      }
      band.push_back({i, h, zg});
      sum_h += static_cast<double>(h);
    }

    if (stats) {
      stats->n_valid = cloud.size();
      stats->n_trunk_band = band.size();
      stats->mean_h_trunk_band =
        band.empty() ? 0.0 : sum_h / static_cast<double>(band.size());
    }
    return band;
  }
};

}  // namespace forest_3d_perception

#endif  // FOREST_3D_PERCEPTION__NDSM_FIELD_HPP_
