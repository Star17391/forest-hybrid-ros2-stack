/**
 * @file ground_connectivity.hpp
 * @brief Sprint 1 — ground refinement via cell region growing on terrain grid.
 *
 * Height-only ground (h <= tau) is split into:
 *   - connected: same physical surface as robot-adjacent terrain
 *   - suspended: low h but not connected (low foliage, noise)
 */

#ifndef FOREST_3D_PERCEPTION__GROUND_CONNECTIVITY_HPP_
#define FOREST_3D_PERCEPTION__GROUND_CONNECTIVITY_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <limits>
#include <queue>
#include <vector>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/terrain_grid_2d.hpp"

namespace forest_3d_perception
{

struct GroundConnectivityParams
{
  /** Max |z_surface(cell_a) - z_surface(cell_b)| for 4-neighbor merge. */
  double max_surface_step_m{0.15};
  /** Radius around robot (0,0) to place BFS seeds [m]. */
  double seed_radius_m{2.0};
  /**
   * If true, a connected component score is computed only over cells that
   * contain at least one height-ground point.
   *
   * Important: component growth is allowed through cells that only have a
   * valid terrain z_surface estimate (e.g. after inpaint).
   * This prevents splitting the ground surface due to LiDAR occlusions.
   */
  bool require_ground_points_in_cell{true};
};

struct GroundConnectivityResult
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_connected;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_suspended;
  pcl::PointCloud<pcl::PointXYZ>::Ptr holes;
  pcl::PointCloud<pcl::PointXYZ>::Ptr nonground;

  std::size_t n_ground_raw{0};
  std::size_t n_ground_connected{0};
  std::size_t n_suspended{0};
  std::size_t n_holes{0};
  std::size_t n_nonground{0};
  std::size_t n_unknown{0};
  std::size_t n_connected_cells{0};

  double gcr_pct{0.0};
  double suspended_pct_of_raw{0.0};
  double mean_abs_dz_connected{0.0};
  /** Per terrain cell: 1 if in robot-connected ground component (for trunk anchoring). */
  std::vector<uint8_t> cell_connected;
};

class GroundConnectivity
{
public:
  GroundConnectivityParams params;

  GroundConnectivityResult segment(
    const TerrainGrid2D & grid,
    const pcl::PointCloud<pcl::PointXYZ> & cloud) const
  {
    GroundConnectivityResult out;
    out.ground_connected.reset(new pcl::PointCloud<pcl::PointXYZ>);
    out.ground_suspended.reset(new pcl::PointCloud<pcl::PointXYZ>);
    out.holes.reset(new pcl::PointCloud<pcl::PointXYZ>);
    out.nonground.reset(new pcl::PointCloud<pcl::PointXYZ>);

    const std::size_t n_cells = grid.width * grid.height;
    std::vector<bool> cell_has_ground_pt(n_cells, false);
    std::vector<std::size_t> cell_ground_pt_count(n_cells, 0);

    struct PointLabel
    {
      TerrainGrid2D::PointClass cls{TerrainGrid2D::PointClass::Unknown};
    };
    std::vector<PointLabel> labels(cloud.size());

    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const auto & p = cloud.points[i];
      labels[i].cls = grid.classify_point(p.x, p.y, p.z);
      if (labels[i].cls == TerrainGrid2D::PointClass::Ground) {
        ++out.n_ground_raw;
        std::size_t ix = 0;
        std::size_t iy = 0;
        if (grid.index_for(p.x, p.y, ix, iy)) {
          const std::size_t idx = iy * grid.width + ix;
          cell_has_ground_pt[idx] = true;
          ++cell_ground_pt_count[idx];
        }
      }
    }

    // Build connected components on the terrain surface graph:
    // - nodes: cells with valid z_surface
    // - edges: 4-neighbor if |z_a - z_b| <= max_surface_step_m
    // Component identity comes from BFS on the terrain; scoring uses only
    // cells that actually contain "ground" points.
    std::vector<int> comp_id(n_cells, -1);
    std::vector<std::size_t> comp_score;  // sum of ground points per comp
    comp_score.reserve(n_cells);

    auto cell_index = [&](std::size_t ix, std::size_t iy) -> std::size_t {
      return iy * grid.width + ix;
    };

    auto idx_to_xy = [&](std::size_t idx, std::size_t & ix, std::size_t & iy) {
      ix = idx % grid.width;
      iy = idx / grid.width;
    };

    const float max_step = static_cast<float>(params.max_surface_step_m);

    // BFS queue of cell indices
    std::queue<std::size_t> q;
    const int dx4[] = {1, -1, 0, 0};
    const int dy4[] = {0, 0, 1, -1};

    auto is_surface_valid = [&](std::size_t idx) -> bool {
      const std::size_t ix = idx % grid.width;
      const std::size_t iy = idx / grid.width;
      return !std::isnan(grid.height_at_cell(ix, iy));
    };

    // Seed components from ground-evidence cells (and optionally restrict
    // seeds to near-robot for tie-break). Growth is allowed across inpainted
    // valid terrain cells.
    std::vector<bool> visited_seed(n_cells, false);
    if (!params.require_ground_points_in_cell) {
      // Not expected in our use-case; keep semantics.
    }

    std::size_t best_comp = static_cast<std::size_t>(-1);
    std::size_t best_score = 0;

    auto robot_ixiy = [&]() -> std::pair<std::size_t, std::size_t> {
      std::size_t ix = 0;
      std::size_t iy = 0;
      if (!grid.index_for(0.0f, 0.0f, ix, iy)) {
        return {static_cast<std::size_t>(-1), static_cast<std::size_t>(-1)};
      }
      return {ix, iy};
    };
    const auto [robot_ix, robot_iy] = robot_ixiy();

    auto cell_center = [&](std::size_t ix, std::size_t iy) -> std::pair<float, float> {
      const float half_x = static_cast<float>(grid.size_x_m * 0.5);
      const float half_y = static_cast<float>(grid.size_y_m * 0.5);
      const float x = static_cast<float>(ix) * static_cast<float>(grid.resolution_m) - half_x;
      const float y = static_cast<float>(iy) * static_cast<float>(grid.resolution_m) - half_y;
      return {x, y};
    };

    for (std::size_t seed_idx = 0; seed_idx < n_cells; ++seed_idx) {
      if (!cell_has_ground_pt[seed_idx]) {
        continue;
      }
      if (comp_id[seed_idx] != -1) {
        continue;
      }
      if (!is_surface_valid(seed_idx)) {
        continue;
      }

      // Start BFS to assign component id
      const int this_comp = static_cast<int>(comp_score.size());
      comp_score.push_back(0);
      comp_score[this_comp] = 0;

      q.push(seed_idx);
      comp_id[seed_idx] = this_comp;

      while (!q.empty()) {
        const std::size_t cur = q.front();
        q.pop();
        std::size_t cx = 0, cy = 0;
        idx_to_xy(cur, cx, cy);
        const float z0 = grid.height_at_cell(cx, cy);

        // Score accumulation: only cells with ground evidence count
        if (cell_has_ground_pt[cur]) {
          comp_score[this_comp] += cell_ground_pt_count[cur];
        }

        for (int k = 0; k < 4; ++k) {
          const int nx = static_cast<int>(cx) + dx4[k];
          const int ny = static_cast<int>(cy) + dy4[k];
          if (nx < 0 || ny < 0 || nx >= static_cast<int>(grid.width) ||
            ny >= static_cast<int>(grid.height))
          {
            continue;
          }
          const std::size_t nidx = cell_index(static_cast<std::size_t>(nx), static_cast<std::size_t>(ny));
          if (comp_id[nidx] != -1) {
            continue;
          }
          const float zn = grid.height_at_cell(static_cast<std::size_t>(nx), static_cast<std::size_t>(ny));
          if (std::isnan(zn)) {
            continue;
          }
          if (std::abs(z0 - zn) > max_step) {
            continue;
          }
          comp_id[nidx] = this_comp;
          q.push(nidx);
        }
      }
    }

    // If no component spawned due to seed radius, relax: create from any ground-evidence cell.
    if (comp_score.empty()) {
      for (std::size_t seed_idx = 0; seed_idx < n_cells; ++seed_idx) {
        if (!cell_has_ground_pt[seed_idx]) {
          continue;
        }
        if (!is_surface_valid(seed_idx)) {
          continue;
        }
        if (comp_id[seed_idx] != -1) {
          continue;
        }
        const int this_comp = static_cast<int>(comp_score.size());
        comp_score.push_back(0);
        q.push(seed_idx);
        comp_id[seed_idx] = this_comp;
        while (!q.empty()) {
          const std::size_t cur = q.front();
          q.pop();
          std::size_t cx = 0, cy = 0;
          idx_to_xy(cur, cx, cy);
          const float z0 = grid.height_at_cell(cx, cy);
          if (cell_has_ground_pt[cur]) {
            comp_score[this_comp] += cell_ground_pt_count[cur];
          }
          for (int k = 0; k < 4; ++k) {
            const int nx = static_cast<int>(cx) + dx4[k];
            const int ny = static_cast<int>(cy) + dy4[k];
            if (nx < 0 || ny < 0 || nx >= static_cast<int>(grid.width) ||
              ny >= static_cast<int>(grid.height))
            {
              continue;
            }
            const std::size_t nidx = cell_index(static_cast<std::size_t>(nx), static_cast<std::size_t>(ny));
            if (comp_id[nidx] != -1) {
              continue;
            }
            const float zn = grid.height_at_cell(static_cast<std::size_t>(nx), static_cast<std::size_t>(ny));
            if (std::isnan(zn)) {
              continue;
            }
            if (std::abs(z0 - zn) > max_step) {
              continue;
            }
            comp_id[nidx] = this_comp;
            q.push(nidx);
          }
        }
      }
    }

    if (!comp_score.empty()) {
      // Choose the best component as the one with max ground evidence (largest surface)
      best_score = *std::max_element(comp_score.begin(), comp_score.end());
      best_comp = 0;
      for (std::size_t i = 0; i < comp_score.size(); ++i) {
        if (comp_score[i] == best_score) {
          best_comp = i;
          break;
        }
      }
      // Tie-break: if robot cell belongs to some component, prefer it
      if (robot_ix != static_cast<std::size_t>(-1)) {
        const std::size_t ridx = cell_index(robot_ix, robot_iy);
        const int rc = comp_id[ridx];
        if (rc >= 0 && static_cast<std::size_t>(rc) < comp_score.size() &&
          comp_score[static_cast<std::size_t>(rc)] == best_score)
        {
          best_comp = static_cast<std::size_t>(rc);
        }
      }
    }

    out.cell_connected.assign(n_cells, 0);
    if (best_comp != static_cast<std::size_t>(-1)) {
      for (std::size_t idx = 0; idx < n_cells; ++idx) {
        if (comp_id[idx] == static_cast<int>(best_comp)) {
          out.cell_connected[idx] = 1;
          ++out.n_connected_cells;
        }
      }
    }

    double sum_abs_dz_connected = 0.0;
    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const auto & p = cloud.points[i];
      switch (labels[i].cls) {
        case TerrainGrid2D::PointClass::Ground: {
          std::size_t ix = 0;
          std::size_t iy = 0;
          bool connected = false;
          if (grid.index_for(p.x, p.y, ix, iy)) {
            const std::size_t idx = iy * grid.width + ix;
            connected = (best_comp != static_cast<std::size_t>(-1) &&
              comp_id[idx] == static_cast<int>(best_comp));
          }
          if (connected) {
            out.ground_connected->push_back(p);
            ++out.n_ground_connected;
            const float zg = grid.ground_reference_at(p.x, p.y);
            sum_abs_dz_connected += std::abs(p.z - zg);
          } else {
            out.ground_suspended->push_back(p);
            ++out.n_suspended;
          }
          break;
        }
        case TerrainGrid2D::PointClass::Hole:
          out.holes->push_back(p);
          ++out.n_holes;
          break;
        case TerrainGrid2D::PointClass::NonGround:
          out.nonground->push_back(p);
          ++out.n_nonground;
          break;
        case TerrainGrid2D::PointClass::Unknown:
          out.nonground->push_back(p);
          ++out.n_nonground;
          ++out.n_unknown;
          break;
      }
    }

    out.mean_abs_dz_connected =
      out.n_ground_connected > 0 ? sum_abs_dz_connected / static_cast<double>(out.n_ground_connected) : 0.0;

    out.gcr_pct = out.n_ground_raw > 0
      ? 100.0 * static_cast<double>(out.n_ground_connected) / static_cast<double>(out.n_ground_raw)
      : 100.0;
    out.suspended_pct_of_raw = out.n_ground_raw > 0
      ? 100.0 * static_cast<double>(out.n_suspended) / static_cast<double>(out.n_ground_raw)
      : 0.0;
    return out;
  }
};

}  // namespace forest_3d_perception

#endif  // FOREST_3D_PERCEPTION__GROUND_CONNECTIVITY_HPP_
