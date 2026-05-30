/**
 * @file trunk_column_extractor.hpp
 * @brief 2D column occupancy from nDSM trunk-band points (Sprint 3).
 */

#ifndef FOREST_3D_PERCEPTION__TRUNK_COLUMN_EXTRACTOR_HPP_
#define FOREST_3D_PERCEPTION__TRUNK_COLUMN_EXTRACTOR_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/cylinder_fit.hpp"
#include "forest_3d_perception/ndsm_field.hpp"
#include "forest_3d_perception/terrain_grid_2d.hpp"

namespace forest_3d_perception
{

struct ColumnExtractionParams
{
  float ndsm_min_m{0.28f};
  float ndsm_max_m{5.0f};
  int min_points_per_cell{2};
  int min_cells_per_column{3};
  std::size_t min_points_per_column{8};
  std::size_t max_points_per_column{600};
  std::size_t max_columns_per_frame{20};
};

struct ColumnExtractionStats
{
  std::size_t n_band_points{0};
  std::size_t n_columns_found{0};
  std::size_t n_columns_accepted{0};
  std::size_t reject_sparse{0};
  std::size_t reject_cylinder{0};
  std::size_t reject_rmse{0};
  std::size_t reject_height{0};
  std::size_t reject_radius{0};
};

struct ColumnDetection
{
  std::vector<std::size_t> point_indices;
  CylinderObservation cylinder;
};

class TrunkColumnExtractor
{
public:
  ColumnExtractionParams params;
  double cylinder_min_height_m{0.35};
  double cylinder_max_radius_m{0.70};
  double cylinder_max_rmse_m{0.16};
  double cylinder_min_inlier_ratio{0.40};
  double cylinder_inlier_dist_m{0.12};
  /** Fit cylinder using points only up to this height above column base (excludes canopy). */
  double cylinder_max_slice_height_m{2.5};

  std::vector<ColumnDetection> extract(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const TerrainGrid2D & grid,
    ColumnExtractionStats * stats = nullptr) const
  {
    ColumnExtractionStats local;
    NdsmStats ndsm_stats;
    const auto band = NdsmField::compute_trunk_band(
      cloud, grid, params.ndsm_min_m, params.ndsm_max_m, &ndsm_stats);
    local.n_band_points = band.size();

    if (band.empty()) {
      if (stats) {
        *stats = local;
      }
      return {};
    }

    // Sparse occupancy — só células com hits (evita O(width×height) por frame).
    std::unordered_map<std::size_t, std::vector<std::size_t>> cell_points;
    cell_points.reserve(band.size() / 2 + 1);

    for (const auto & np : band) {
      std::size_t ix = 0;
      std::size_t iy = 0;
      const auto & p = cloud.points[np.index];
      if (!grid.index_for(p.x, p.y, ix, iy)) {
        continue;
      }
      const std::size_t cidx = iy * grid.width + ix;
      cell_points[cidx].push_back(np.index);
    }

    std::unordered_set<std::size_t> visited;
    std::vector<ColumnDetection> detections;

    auto cell_count = [&](std::size_t cidx) -> int {
      const auto it = cell_points.find(cidx);
      return it == cell_points.end() ? 0 : static_cast<int>(it->second.size());
    };

    auto is_seed = [&](std::size_t cidx) {
      return cell_count(cidx) >= params.min_points_per_cell;
    };

    for (const auto & [seed_cidx, _seed_pts] : cell_points) {
      (void)_seed_pts;
      if (visited.count(seed_cidx) || !is_seed(seed_cidx)) {
        continue;
      }

      std::vector<std::size_t> component_cells;
      std::queue<std::size_t> q;
      q.push(seed_cidx);
      visited.insert(seed_cidx);

      while (!q.empty()) {
        const std::size_t cur = q.front();
        q.pop();
        component_cells.push_back(cur);

        const std::size_t ix = cur % grid.width;
        const std::size_t iy = cur / grid.width;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = static_cast<int>(ix) + dx;
            const int ny = static_cast<int>(iy) + dy;
            if (nx < 0 || ny < 0 || nx >= static_cast<int>(grid.width) ||
              ny >= static_cast<int>(grid.height))
            {
              continue;
            }
            const std::size_t nidx =
              static_cast<std::size_t>(ny) * grid.width + static_cast<std::size_t>(nx);
            if (visited.count(nidx) || !is_seed(nidx)) {
              continue;
            }
            visited.insert(nidx);
            q.push(nidx);
          }
        }
      }

      ++local.n_columns_found;
      if (static_cast<int>(component_cells.size()) < params.min_cells_per_column) {
        ++local.reject_sparse;
        continue;
      }

      std::vector<std::size_t> indices;
      indices.reserve(component_cells.size() * 4);
      for (std::size_t cell : component_cells) {
        const auto & pts = cell_points.at(cell);
        indices.insert(indices.end(), pts.begin(), pts.end());
      }
      if (indices.size() < params.min_points_per_column) {
        ++local.reject_sparse;
        continue;
      }
      if (indices.size() > params.max_points_per_column) {
        ++local.reject_sparse;
        continue;
      }

      ColumnDetection det;
      det.point_indices = std::move(indices);
      CylinderObservation cyl;
      const auto rej = fit_vertical_cylinder(
        cloud, det.point_indices, cyl,
        cylinder_min_height_m, cylinder_max_radius_m,
        cylinder_max_rmse_m, cylinder_min_inlier_ratio, cylinder_inlier_dist_m,
        cylinder_max_slice_height_m);

      if (rej != CylinderReject::Accepted) {
        switch (rej) {
          case CylinderReject::TooShort:
            ++local.reject_height;
            break;
          case CylinderReject::TooWide:
            ++local.reject_radius;
            break;
          case CylinderReject::HighRmse:
            ++local.reject_rmse;
            break;
          default:
            ++local.reject_cylinder;
            break;
        }
        continue;
      }

      det.cylinder = cyl;
      detections.push_back(std::move(det));
      ++local.n_columns_accepted;
    }

    if (detections.size() > params.max_columns_per_frame) {
      std::sort(
        detections.begin(), detections.end(),
        [](const ColumnDetection & a, const ColumnDetection & b) {
          return a.point_indices.size() > b.point_indices.size();
        });
      detections.resize(params.max_columns_per_frame);
    }

    if (stats) {
      *stats = local;
    }
    return detections;
  }
};

}  // namespace forest_3d_perception

#endif  // FOREST_3D_PERCEPTION__TRUNK_COLUMN_EXTRACTOR_HPP_
