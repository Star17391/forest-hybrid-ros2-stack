/**
 * @file ground_reference_debug.hpp
 * @brief Per-point and per-cell ground-reference diagnostics (nDSM zg validation).
 */

#ifndef FOREST_3D_PERCEPTION__GROUND_REFERENCE_DEBUG_HPP_
#define FOREST_3D_PERCEPTION__GROUND_REFERENCE_DEBUG_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/terrain_grid_2d.hpp"

namespace forest_3d_perception
{

struct GroundRefPointDiag
{
  float zg_ndsm{std::numeric_limits<float>::quiet_NaN()};
  float h_ndsm{std::numeric_limits<float>::quiet_NaN()};
  float z_surface_cell{std::numeric_limits<float>::quiet_NaN()};
  float z_surface_percentile_raw{std::numeric_limits<float>::quiet_NaN()};
  float z_mesh_cell{std::numeric_limits<float>::quiet_NaN()};
  float z_mesh_observed{std::numeric_limits<float>::quiet_NaN()};
  float zg_surface_neighbor_min{std::numeric_limits<float>::quiet_NaN()};
  TerrainCellProvenance mesh_provenance{TerrainCellProvenance::Unknown};
  bool mesh_has_ground_obs{false};
  float mesh_inpaint_delta_m{0.0f};
  float confidence{0.0f};
};

struct GroundRefFrameStats
{
  std::size_t n_points{0};
  std::size_t n_zg_nan{0};
  std::size_t n_h_below_ndsm_min{0};
  std::size_t n_h_in_ndsm_band{0};
  std::size_t n_h_above_ndsm_max{0};
  std::size_t n_under_inpainted_cell{0};
  std::size_t n_under_no_obs_cell{0};
  double mean_h{0.0};
  double mean_inpaint_delta_m{0.0};
  double mean_zg_minus_z_obs_m{0.0};
  std::size_t n_zg_minus_obs_samples{0};
};

struct GroundRefGridStats
{
  std::size_t mesh_cells_observed{0};
  std::size_t mesh_cells_inpainted{0};
  std::size_t mesh_cells_unknown{0};
  std::size_t surface_cells_percentile{0};
  std::size_t surface_cells_inpainted{0};
};

inline float mesh_confidence_score(TerrainCellProvenance prov, bool has_obs)
{
  if (has_obs) {
    return 1.0f;
  }
  if (prov == TerrainCellProvenance::Inpainted) {
    return 0.35f;
  }
  return 0.0f;
}

/** Per-point diagnostics at (x,y,z) using current TerrainGrid2D state. */
inline GroundRefPointDiag diagnose_point(
  const TerrainGrid2D & grid, float x, float y, float z)
{
  GroundRefPointDiag d;
  std::size_t ix = 0;
  std::size_t iy = 0;
  if (!grid.index_for(x, y, ix, iy)) {
    return d;
  }

  d.zg_ndsm = grid.ndsm_ground_reference_at(ix, iy);
  if (!std::isnan(d.zg_ndsm)) {
    d.h_ndsm = z - d.zg_ndsm;
  }

  d.z_surface_cell = grid.height_at_cell(ix, iy);
  d.z_surface_percentile_raw = grid.surface_percentile_raw_at(ix, iy);
  d.zg_surface_neighbor_min = grid.ground_reference_at(ix, iy);

  if (grid.has_mesh_surface()) {
    d.z_mesh_cell = grid.mesh_height_at_cell(ix, iy);
    d.z_mesh_observed = grid.mesh_observed_z_at(ix, iy);
    d.mesh_provenance = grid.mesh_provenance_at(ix, iy);
    d.mesh_has_ground_obs = grid.mesh_has_ground_observation(ix, iy);
    if (d.mesh_has_ground_obs && !std::isnan(d.z_mesh_cell) && !std::isnan(d.z_mesh_observed)) {
      d.mesh_inpaint_delta_m = d.z_mesh_cell - d.z_mesh_observed;
    }
  }

  d.confidence = mesh_confidence_score(d.mesh_provenance, d.mesh_has_ground_obs);
  return d;
}

inline void accumulate_ground_ref_stats(
  GroundRefFrameStats * stats,
  const GroundRefPointDiag & d,
  float ndsm_min_m,
  float ndsm_max_m)
{
  if (stats == nullptr) {
    return;
  }
  ++stats->n_points;
  if (std::isnan(d.h_ndsm)) {
    ++stats->n_zg_nan;
    return;
  }
  stats->mean_h += static_cast<double>(d.h_ndsm);
  if (d.h_ndsm < ndsm_min_m) {
    ++stats->n_h_below_ndsm_min;
  } else if (d.h_ndsm > ndsm_max_m) {
    ++stats->n_h_above_ndsm_max;
  } else {
    ++stats->n_h_in_ndsm_band;
  }
  if (!d.mesh_has_ground_obs) {
    if (d.mesh_provenance == TerrainCellProvenance::Inpainted) {
      ++stats->n_under_inpainted_cell;
      stats->mean_inpaint_delta_m += static_cast<double>(d.mesh_inpaint_delta_m);
    } else {
      ++stats->n_under_no_obs_cell;
    }
  }
  if (d.mesh_has_ground_obs && !std::isnan(d.zg_ndsm) && !std::isnan(d.z_mesh_observed)) {
    stats->mean_zg_minus_z_obs_m += static_cast<double>(d.zg_ndsm - d.z_mesh_observed);
    ++stats->n_zg_minus_obs_samples;
  }
}

inline void finalize_ground_ref_stats(GroundRefFrameStats * stats)
{
  if (stats == nullptr || stats->n_points == 0) {
    return;
  }
  const double n = static_cast<double>(stats->n_points);
  stats->mean_h /= n;
  if (stats->n_under_inpainted_cell > 0) {
    stats->mean_inpaint_delta_m /=
      static_cast<double>(stats->n_under_inpainted_cell);
  }
  if (stats->n_zg_minus_obs_samples > 0) {
    stats->mean_zg_minus_z_obs_m /=
      static_cast<double>(stats->n_zg_minus_obs_samples);
  }
}

/** Point cloud with intensity = h_ndsm (same XYZ as input). */
inline pcl::PointCloud<pcl::PointXYZI>::Ptr make_height_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const TerrainGrid2D & grid)
{
  auto out = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  out->reserve(cloud.size());
  for (const auto & p : cloud.points) {
    const auto d = diagnose_point(grid, p.x, p.y, p.z);
    if (std::isnan(d.h_ndsm)) {
      continue;
    }
    pcl::PointXYZI q;
    q.x = p.x;
    q.y = p.y;
    q.z = p.z;
    q.intensity = d.h_ndsm;
    out->push_back(q);
  }
  return out;
}

/** Ground reference height at each point: XYZ = (x,y,zg), intensity = confidence. */
inline pcl::PointCloud<pcl::PointXYZI>::Ptr make_zg_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const TerrainGrid2D & grid)
{
  auto out = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  out->reserve(cloud.size());
  for (const auto & p : cloud.points) {
    const auto d = diagnose_point(grid, p.x, p.y, p.z);
    if (std::isnan(d.zg_ndsm)) {
      continue;
    }
    pcl::PointXYZI q;
    q.x = p.x;
    q.y = p.y;
    q.z = d.zg_ndsm;
    q.intensity = d.confidence;
    out->push_back(q);
  }
  return out;
}

/** Cell centers where mesh has no direct ground observation (after inpaint may still have Z). */
inline pcl::PointCloud<pcl::PointXYZI>::Ptr make_mesh_no_obs_cells_cloud(
  const TerrainGrid2D & grid)
{
  auto out = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  if (!grid.has_mesh_surface()) {
    return out;
  }
  const float half_x = static_cast<float>(grid.size_x_m * 0.5);
  const float half_y = static_cast<float>(grid.size_y_m * 0.5);
  const float res = static_cast<float>(grid.resolution_m);
  for (std::size_t iy = 0; iy < grid.height; ++iy) {
    for (std::size_t ix = 0; ix < grid.width; ++ix) {
      if (grid.mesh_has_ground_observation(ix, iy)) {
        continue;
      }
      const float z = grid.mesh_height_at_cell(ix, iy);
      pcl::PointXYZI p;
      p.x = static_cast<float>(ix) * res - half_x + 0.5f * res;
      p.y = static_cast<float>(iy) * res - half_y + 0.5f * res;
      p.z = std::isnan(z) ? 0.0f : z;
      p.intensity = (grid.mesh_provenance_at(ix, iy) == TerrainCellProvenance::Inpainted) ?
        0.35f : 0.0f;
      out->push_back(p);
    }
  }
  return out;
}

/** All mesh cells: center XYZ, intensity = confidence [0,1]. */
inline pcl::PointCloud<pcl::PointXYZI>::Ptr make_mesh_confidence_cells_cloud(
  const TerrainGrid2D & grid)
{
  auto out = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  if (!grid.has_mesh_surface()) {
    return out;
  }
  const float half_x = static_cast<float>(grid.size_x_m * 0.5);
  const float half_y = static_cast<float>(grid.size_y_m * 0.5);
  const float res = static_cast<float>(grid.resolution_m);
  out->reserve(grid.width * grid.height);
  for (std::size_t iy = 0; iy < grid.height; ++iy) {
    for (std::size_t ix = 0; ix < grid.width; ++ix) {
      const float z = grid.mesh_height_at_cell(ix, iy);
      if (std::isnan(z)) {
        continue;
      }
      pcl::PointXYZI p;
      p.x = static_cast<float>(ix) * res - half_x + 0.5f * res;
      p.y = static_cast<float>(iy) * res - half_y + 0.5f * res;
      p.z = z;
      p.intensity = mesh_confidence_score(
        grid.mesh_provenance_at(ix, iy), grid.mesh_has_ground_observation(ix, iy));
      out->push_back(p);
    }
  }
  return out;
}

inline GroundRefGridStats compute_grid_stats(const TerrainGrid2D & grid)
{
  GroundRefGridStats s;
  if (!grid.has_mesh_surface()) {
    return s;
  }
  for (std::size_t iy = 0; iy < grid.height; ++iy) {
    for (std::size_t ix = 0; ix < grid.width; ++ix) {
      switch (grid.mesh_provenance_at(ix, iy)) {
        case TerrainCellProvenance::ObservedGround:
          ++s.mesh_cells_observed;
          break;
        case TerrainCellProvenance::Inpainted:
          ++s.mesh_cells_inpainted;
          break;
        default:
          ++s.mesh_cells_unknown;
          break;
      }
      switch (grid.surface_provenance_at(ix, iy)) {
        case TerrainCellProvenance::ObservedGround:
          ++s.surface_cells_percentile;
          break;
        case TerrainCellProvenance::Inpainted:
          ++s.surface_cells_inpainted;
          break;
        default:
          break;
      }
    }
  }
  return s;
}

}  // namespace forest_3d_perception

#endif  // FOREST_3D_PERCEPTION__GROUND_REFERENCE_DEBUG_HPP_
