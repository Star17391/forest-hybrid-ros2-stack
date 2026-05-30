/**
 * @file terrain_grid_2d.hpp
 * @brief Rolling 2.5D height grid for ground estimation (Fase 1b).
 *
 * Pipeline per frame: bin Z per cell → low percentile → inpaint → spike clamp → inpaint.
 * z_surface is terrain-only (mesh RViz); points above it are non-ground.
 */

#ifndef FOREST_3D_PERCEPTION__TERRAIN_GRID_2D_HPP_
#define FOREST_3D_PERCEPTION__TERRAIN_GRID_2D_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace forest_3d_perception
{

/** How a grid cell obtained its final Z (for debug / confidence). */
enum class TerrainCellProvenance : uint8_t
{
  Unknown = 0,
  ObservedGround = 1,
  Inpainted = 2,
};

struct TerrainGrid2D
{
  double size_x_m{30.0};
  double size_y_m{30.0};
  double resolution_m{0.25};
  double ground_height_thresh_m{0.12};
  double hole_depth_m{0.15};
  int inpaint_max_passes{4};
  /** Low percentile of Z per cell (0.10 = 10th percentile) — robust vs trunks in cell. */
  double height_percentile{0.10};
  /** |z_cell - median(neighbors)| above this → cell invalidated and re-inpainted. */
  double smooth_max_step_m{0.40};
  int smooth_clamp_passes{2};
  int smooth_median_radius_cells{1};
  /** Min-Z in (2r+1)×(2r+1) cells for per-point ground reference. */
  int ground_neighbor_radius_cells{2};

  std::size_t width{0};
  std::size_t height{0};
  std::vector<float> z_surface;  // terrain height per cell (NaN = unknown)
  /** Optional RViz mesh built only from ground-classified points (not full cloud). */
  std::vector<float> z_mesh_surface;
  /** Min-Z from ground-classified points only, before inpaint (NaN = no obs). */
  std::vector<float> mesh_z_observed;
  std::vector<uint8_t> mesh_has_ground_obs;
  std::vector<TerrainCellProvenance> mesh_provenance;
  /** Percentile surface before inpaint/clamp (full cloud). */
  std::vector<float> z_surface_percentile_raw;
  std::vector<TerrainCellProvenance> surface_provenance;
  std::size_t last_cells_clamped{0};

  void configure(double sx, double sy, double res)
  {
    size_x_m = sx;
    size_y_m = sy;
    resolution_m = std::max(0.05, res);
    width = static_cast<std::size_t>(std::ceil(size_x_m / resolution_m));
    height = static_cast<std::size_t>(std::ceil(size_y_m / resolution_m));
    z_surface.assign(width * height, std::numeric_limits<float>::quiet_NaN());
    z_mesh_surface.clear();
    mesh_z_observed.clear();
    mesh_has_ground_obs.clear();
    mesh_provenance.clear();
    z_surface_percentile_raw.clear();
    surface_provenance.clear();
    cell_z_.assign(width * height, {});
    last_cells_clamped = 0;
  }

  bool has_mesh_surface() const
  {
    return z_mesh_surface.size() == width * height;
  }

  float height_for_mesh(std::size_t ix, std::size_t iy) const
  {
    const std::size_t idx = iy * width + ix;
    if (has_mesh_surface()) {
      return z_mesh_surface[idx];
    }
    return z_surface[idx];
  }

  bool index_for(float x, float y, std::size_t & ix, std::size_t & iy) const
  {
    const float half_x = static_cast<float>(size_x_m * 0.5);
    const float half_y = static_cast<float>(size_y_m * 0.5);
    if (x < -half_x || x > half_x || y < -half_y || y > half_y) {
      return false;
    }
    ix = static_cast<std::size_t>((x + half_x) / resolution_m);
    iy = static_cast<std::size_t>((y + half_y) / resolution_m);
    if (ix >= width || iy >= height) {
      return false;
    }
    return true;
  }

  static float percentile_low(const std::vector<float> & samples, double p)
  {
    if (samples.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    if (samples.size() == 1) {
      return samples.front();
    }
    std::vector<float> sorted = samples;
    std::sort(sorted.begin(), sorted.end());
    p = std::clamp(p, 0.0, 1.0);
    const double idx = p * static_cast<double>(sorted.size() - 1);
    const std::size_t i0 = static_cast<std::size_t>(std::floor(idx));
    const std::size_t i1 = std::min(i0 + 1, sorted.size() - 1);
    const double frac = idx - static_cast<double>(i0);
    return sorted[i0] * static_cast<float>(1.0 - frac) + sorted[i1] * static_cast<float>(frac);
  }

  void accumulate(const pcl::PointCloud<pcl::PointXYZ> & cloud)
  {
    for (auto & v : cell_z_) {
      v.clear();
    }
    for (const auto & p : cloud.points) {
      std::size_t ix = 0;
      std::size_t iy = 0;
      if (!index_for(p.x, p.y, ix, iy)) {
        continue;
      }
      cell_z_[iy * width + ix].push_back(p.z);
    }
  }

  void compute_percentile_surface()
  {
    for (std::size_t idx = 0; idx < z_surface.size(); ++idx) {
      if (cell_z_[idx].empty()) {
        z_surface[idx] = std::numeric_limits<float>::quiet_NaN();
      } else {
        z_surface[idx] = percentile_low(cell_z_[idx], height_percentile);
      }
    }
  }

  float median_neighbors(std::size_t ix, std::size_t iy, int radius) const
  {
    std::vector<float> vals;
    vals.reserve(static_cast<std::size_t>((2 * radius + 1) * (2 * radius + 1)));
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        const int nx = static_cast<int>(ix) + dx;
        const int ny = static_cast<int>(iy) + dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) || ny >= static_cast<int>(height)) {
          continue;
        }
        const float z = z_surface[static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx)];
        if (!std::isnan(z)) {
          vals.push_back(z);
        }
      }
    }
    if (vals.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    std::sort(vals.begin(), vals.end());
    return vals[vals.size() / 2];
  }

  void inpaint_empty()
  {
    inpaint_surface_with_provenance(z_surface, surface_provenance);
  }

  void clamp_surface_spikes()
  {
    last_cells_clamped = 0;
    const int r = std::max(1, smooth_median_radius_cells);
    const float max_step = static_cast<float>(smooth_max_step_m);

    for (int pass = 0; pass < smooth_clamp_passes; ++pass) {
      std::vector<float> next = z_surface;
      std::size_t clamped_this_pass = 0;
      for (std::size_t iy = 0; iy < height; ++iy) {
        for (std::size_t ix = 0; ix < width; ++ix) {
          const std::size_t idx = iy * width + ix;
          const float z = z_surface[idx];
          if (std::isnan(z)) {
            continue;
          }
          const float med = median_neighbors(ix, iy, r);
          if (std::isnan(med)) {
            continue;
          }
          if (std::abs(z - med) > max_step) {
            next[idx] = std::numeric_limits<float>::quiet_NaN();
            if (surface_provenance.size() == z_surface.size()) {
              surface_provenance[idx] = TerrainCellProvenance::Unknown;
            }
            ++clamped_this_pass;
          }
        }
      }
      z_surface = next;
      last_cells_clamped += clamped_this_pass;
      if (clamped_this_pass > 0) {
        inpaint_empty();
      }
    }
  }

  float height_at_cell(std::size_t ix, std::size_t iy) const
  {
    if (ix >= width || iy >= height) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return z_surface[iy * width + ix];
  }

  float surface_percentile_raw_at(std::size_t ix, std::size_t iy) const
  {
    if (ix >= width || iy >= height || z_surface_percentile_raw.size() != width * height) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return z_surface_percentile_raw[iy * width + ix];
  }

  float surface_percentile_raw_at(float x, float y) const
  {
    std::size_t ix = 0;
    std::size_t iy = 0;
    if (!index_for(x, y, ix, iy)) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return surface_percentile_raw_at(ix, iy);
  }

  TerrainCellProvenance surface_provenance_at(std::size_t ix, std::size_t iy) const
  {
    if (ix >= width || iy >= height || surface_provenance.size() != width * height) {
      return TerrainCellProvenance::Unknown;
    }
    return surface_provenance[iy * width + ix];
  }

  float mesh_height_at_cell(std::size_t ix, std::size_t iy) const
  {
    if (!has_mesh_surface() || ix >= width || iy >= height) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return z_mesh_surface[iy * width + ix];
  }

  float mesh_observed_z_at(std::size_t ix, std::size_t iy) const
  {
    if (!has_mesh_surface() || mesh_z_observed.size() != width * height || ix >= width || iy >= height) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return mesh_z_observed[iy * width + ix];
  }

  bool mesh_has_ground_observation(std::size_t ix, std::size_t iy) const
  {
    if (!has_mesh_surface() || mesh_has_ground_obs.size() != width * height || ix >= width || iy >= height) {
      return false;
    }
    return mesh_has_ground_obs[iy * width + ix] != 0;
  }

  TerrainCellProvenance mesh_provenance_at(std::size_t ix, std::size_t iy) const
  {
    if (!has_mesh_surface() || mesh_provenance.size() != width * height || ix >= width || iy >= height) {
      return TerrainCellProvenance::Unknown;
    }
    return mesh_provenance[iy * width + ix];
  }

  float height_at(float x, float y) const
  {
    std::size_t ix = 0;
    std::size_t iy = 0;
    if (!index_for(x, y, ix, iy)) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return height_at_cell(ix, iy);
  }

  float ground_reference_at(std::size_t ix, std::size_t iy) const
  {
    return reference_min_in_neighborhood(z_surface, ix, iy);
  }

  float ground_reference_at(float x, float y) const
  {
    std::size_t ix = 0;
    std::size_t iy = 0;
    if (!index_for(x, y, ix, iy)) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return ground_reference_at(ix, iy);
  }

  /**
   * Ground reference for nDSM height: prefers z_mesh_surface (ground-only min-Z
   * per cell + inpaint) so trunk returns in the same XY cell do not inflate zg.
   * Falls back to ground_reference_at when mesh is unavailable.
   */
  float ndsm_ground_reference_at(std::size_t ix, std::size_t iy) const
  {
    if (has_mesh_surface()) {
      const float zm = reference_min_in_neighborhood(z_mesh_surface, ix, iy);
      if (!std::isnan(zm)) {
        return zm;
      }
    }
    return ground_reference_at(ix, iy);
  }

  float ndsm_ground_reference_at(float x, float y) const
  {
    std::size_t ix = 0;
    std::size_t iy = 0;
    if (!index_for(x, y, ix, iy)) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return ndsm_ground_reference_at(ix, iy);
  }

  enum class PointClass { Unknown, Ground, Hole, NonGround };

  PointClass classify_point(float x, float y, float z) const
  {
    const float zg = ground_reference_at(x, y);
    if (std::isnan(zg)) {
      return PointClass::Unknown;
    }
    const float dh = z - zg;
    if (dh < -static_cast<float>(hole_depth_m)) {
      return PointClass::Hole;
    }
    if (dh <= static_cast<float>(ground_height_thresh_m)) {
      return PointClass::Ground;
    }
    return PointClass::NonGround;
  }

  struct SegmentationResult
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground;
    pcl::PointCloud<pcl::PointXYZ>::Ptr holes;
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonground;
    std::size_t cells_filled{0};
    std::size_t cells_total{0};
    double coverage_pct{0.0};
    double mean_abs_dz_ground{0.0};
    std::size_t n_unknown{0};
  };

  SegmentationResult segment_cloud(const pcl::PointCloud<pcl::PointXYZ> & cloud) const
  {
    SegmentationResult out;
    out.ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
    out.holes.reset(new pcl::PointCloud<pcl::PointXYZ>);
    out.nonground.reset(new pcl::PointCloud<pcl::PointXYZ>);

    double sum_abs_dz = 0.0;
    std::size_t n_ground_pts = 0;

    for (const auto & p : cloud.points) {
      switch (classify_point(p.x, p.y, p.z)) {
        case PointClass::Ground:
          out.ground->push_back(p);
          {
            const float zg = ground_reference_at(p.x, p.y);
            sum_abs_dz += std::abs(p.z - zg);
            ++n_ground_pts;
          }
          break;
        case PointClass::Hole:
          out.holes->push_back(p);
          break;
        case PointClass::NonGround:
          out.nonground->push_back(p);
          break;
        case PointClass::Unknown:
          out.nonground->push_back(p);
          ++out.n_unknown;
          break;
      }
    }

    out.cells_total = width * height;
    out.cells_filled = 0;
    for (float z : z_surface) {
      if (!std::isnan(z)) {
        ++out.cells_filled;
      }
    }
    out.coverage_pct =
      out.cells_total > 0 ? 100.0 * static_cast<double>(out.cells_filled) / out.cells_total : 0.0;
    out.mean_abs_dz_ground =
      n_ground_pts > 0 ? sum_abs_dz / static_cast<double>(n_ground_pts) : 0.0;
    return out;
  }

  struct BuildResult
  {
    std::size_t cells_filled{0};
    double coverage_pct{0.0};
    std::size_t cells_clamped{0};
  };

  BuildResult build_from_cloud(const pcl::PointCloud<pcl::PointXYZ> & cloud)
  {
    std::fill(z_surface.begin(), z_surface.end(), std::numeric_limits<float>::quiet_NaN());
    for (auto & v : cell_z_) {
      v.clear();
    }

    accumulate(cloud);
    compute_percentile_surface();
    z_surface_percentile_raw = z_surface;
    surface_provenance.assign(width * height, TerrainCellProvenance::Unknown);
    for (std::size_t idx = 0; idx < z_surface.size(); ++idx) {
      if (!std::isnan(z_surface[idx])) {
        surface_provenance[idx] = TerrainCellProvenance::ObservedGround;
      }
    }
    inpaint_empty();
    clamp_surface_spikes();

    BuildResult br;
    br.cells_clamped = last_cells_clamped;
    br.cells_filled = 0;
    for (float z : z_surface) {
      if (!std::isnan(z)) {
        ++br.cells_filled;
      }
    }
    br.coverage_pct =
      (width * height) > 0 ? 100.0 * static_cast<double>(br.cells_filled) / (width * height) : 0.0;
    return br;
  }

  /**
   * Build ground-only terrain surface (min-Z per cell + inpaint).
   * Used for RViz mesh and as nDSM reference via ndsm_ground_reference_at().
   * Does not alter z_surface used for point classification.
   */
  BuildResult build_mesh_surface_from_cloud(const pcl::PointCloud<pcl::PointXYZ> & ground_cloud)
  {
    const std::size_t ncells = width * height;
    z_mesh_surface.assign(ncells, std::numeric_limits<float>::quiet_NaN());
    mesh_z_observed.assign(ncells, std::numeric_limits<float>::quiet_NaN());
    mesh_has_ground_obs.assign(ncells, 0);
    mesh_provenance.assign(ncells, TerrainCellProvenance::Unknown);
    std::vector<std::vector<float>> mesh_cell_z(ncells);

    for (const auto & p : ground_cloud.points) {
      std::size_t ix = 0;
      std::size_t iy = 0;
      if (!index_for(p.x, p.y, ix, iy)) {
        continue;
      }
      mesh_cell_z[iy * width + ix].push_back(p.z);
    }

    for (std::size_t idx = 0; idx < z_mesh_surface.size(); ++idx) {
      if (mesh_cell_z[idx].empty()) {
        continue;
      }
      const float zmin = *std::min_element(mesh_cell_z[idx].begin(), mesh_cell_z[idx].end());
      z_mesh_surface[idx] = zmin;
      mesh_z_observed[idx] = zmin;
      mesh_has_ground_obs[idx] = 1;
      mesh_provenance[idx] = TerrainCellProvenance::ObservedGround;
    }

    inpaint_surface_with_provenance(z_mesh_surface, mesh_provenance);
    clamp_surface_spikes_on(z_mesh_surface, mesh_provenance);

    BuildResult br;
    br.cells_clamped = 0;
    br.cells_filled = 0;
    for (float z : z_mesh_surface) {
      if (!std::isnan(z)) {
        ++br.cells_filled;
      }
    }
    br.coverage_pct =
      (width * height) > 0 ? 100.0 * static_cast<double>(br.cells_filled) / (width * height) : 0.0;
    return br;
  }

private:
  std::vector<std::vector<float>> cell_z_;

  float reference_min_in_neighborhood(
    const std::vector<float> & surface, std::size_t ix, std::size_t iy) const
  {
    const int r = std::max(0, ground_neighbor_radius_cells);
    float z_min = std::numeric_limits<float>::quiet_NaN();
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        const int nx = static_cast<int>(ix) + dx;
        const int ny = static_cast<int>(iy) + dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) || ny >= static_cast<int>(height)) {
          continue;
        }
        const float z = surface[static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx)];
        if (!std::isnan(z) && (std::isnan(z_min) || z < z_min)) {
          z_min = z;
        }
      }
    }
    return z_min;
  }

  void inpaint_surface_with_provenance(
    std::vector<float> & surface,
    std::vector<TerrainCellProvenance> & provenance) const
  {
    std::vector<float> next = surface;
    for (int pass = 0; pass < inpaint_max_passes; ++pass) {
      bool changed = false;
      for (std::size_t iy = 0; iy < height; ++iy) {
        for (std::size_t ix = 0; ix < width; ++ix) {
          const std::size_t idx = iy * width + ix;
          if (!std::isnan(surface[idx])) {
            continue;
          }
          float sum = 0.0f;
          int count = 0;
          for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
              if (dx == 0 && dy == 0) {
                continue;
              }
              const int nx = static_cast<int>(ix) + dx;
              const int ny = static_cast<int>(iy) + dy;
              if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) ||
                ny >= static_cast<int>(height))
              {
                continue;
              }
              const float nz = surface[static_cast<std::size_t>(ny) * width +
                static_cast<std::size_t>(nx)];
              if (!std::isnan(nz)) {
                sum += nz;
                ++count;
              }
            }
          }
          if (count > 0) {
            next[idx] = sum / static_cast<float>(count);
            if (provenance.size() == surface.size() &&
              provenance[idx] == TerrainCellProvenance::Unknown)
            {
              provenance[idx] = TerrainCellProvenance::Inpainted;
            }
            changed = true;
          }
        }
      }
      surface = next;
      if (!changed) {
        break;
      }
    }
  }

  void clamp_surface_spikes_on(
    std::vector<float> & surface,
    std::vector<TerrainCellProvenance> & provenance) const
  {
    const int r = std::max(1, smooth_median_radius_cells);
    const float max_step = static_cast<float>(smooth_max_step_m);

    for (int pass = 0; pass < smooth_clamp_passes; ++pass) {
      std::vector<float> next = surface;
      for (std::size_t iy = 0; iy < height; ++iy) {
        for (std::size_t ix = 0; ix < width; ++ix) {
          const std::size_t idx = iy * width + ix;
          const float z = surface[idx];
          if (std::isnan(z)) {
            continue;
          }
          const float med = median_neighbors_on(surface, ix, iy, r);
          if (std::isnan(med)) {
            continue;
          }
          if (std::abs(z - med) > max_step) {
            next[idx] = std::numeric_limits<float>::quiet_NaN();
            if (provenance.size() == surface.size()) {
              provenance[idx] = TerrainCellProvenance::Unknown;
            }
          }
        }
      }
      surface = next;
      inpaint_surface_with_provenance(surface, provenance);
    }
  }

  float median_neighbors_on(
    const std::vector<float> & surface, std::size_t ix, std::size_t iy, int radius) const
  {
    std::vector<float> vals;
    vals.reserve(static_cast<std::size_t>((2 * radius + 1) * (2 * radius + 1)));
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        const int nx = static_cast<int>(ix) + dx;
        const int ny = static_cast<int>(iy) + dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) || ny >= static_cast<int>(height)) {
          continue;
        }
        const float z = surface[static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx)];
        if (!std::isnan(z)) {
          vals.push_back(z);
        }
      }
    }
    if (vals.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    std::sort(vals.begin(), vals.end());
    return vals[vals.size() / 2];
  }
};

}  // namespace forest_3d_perception

#endif  // FOREST_3D_PERCEPTION__TERRAIN_GRID_2D_HPP_
