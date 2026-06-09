/**
 * @file object_on_ground_filter.hpp
 * @brief Recover low objects (fallen rocks) that CSF absorbed into the ground.
 *
 * CSF is a terrain filter: it tends to swallow low, wide objects (fallen rocks)
 * as ground. This post-pass fits a LOCAL TERRAIN PLANE to the CSF ground points
 * and reclassifies any "ground" point sitting above that plane by more than a
 * threshold as non-ground (an object on the ground).
 *
 *   CSF ground -> low-percentile per cell -> fit a tilted plane over a window
 *              -> residual = z - plane(x,y); residual > threshold -> non-ground
 *
 * Why a plane (not a window minimum): a tilted plane follows slopes and the
 * gentle dip of depressions, so the residual is ~0 on real terrain regardless of
 * gradient. Only localized bumps (rocks) produce a large positive residual. A
 * window-minimum reference instead assumes flat terrain and turns every slope or
 * depression edge into a false "object".
 *
 * Robust because rocks are a local minority: the per-cell low percentile looks
 * "under" sparse high points, and a single trim-and-refit pass drops the few
 * rock cells before they can tilt the plane.
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__OBJECT_ON_GROUND_FILTER_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__OBJECT_ON_GROUND_FILTER_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace forest_3d_perception::experimental
{

struct ObjectOnGroundParams
{
  bool enabled{true};
  float cell_m{0.20f};            // DEM grid resolution
  float window_m{1.00f};          // half-window radius for the local plane fit
  float percentile{0.15f};        // low percentile of Z per cell (terrain estimate)
  float height_threshold_m{0.12f};// point above the local plane by more than this = object
  int min_points_per_cell{2};     // ignore near-empty cells when estimating terrain
};

class ObjectOnGroundFilter
{
public:
  ObjectOnGroundParams params;

  /**
   * Split CSF ground into refined ground (real terrain) and recovered objects
   * (rocks). Points are appended to the output clouds (which are cleared first).
   */
  void refine(
    const pcl::PointCloud<pcl::PointXYZ> & csf_ground,
    pcl::PointCloud<pcl::PointXYZ> & ground_out,
    pcl::PointCloud<pcl::PointXYZ> & objects_out) const
  {
    ground_out.clear();
    objects_out.clear();
    if (csf_ground.empty()) {
      return;
    }

    const float inv = 1.0f / params.cell_m;

    // 1. Bucket ground Z per cell.
    std::unordered_map<std::int64_t, std::vector<float>> cell_z;
    cell_z.reserve(csf_ground.size());
    for (const auto & p : csf_ground.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.z)) {
        continue;
      }
      cell_z[key(static_cast<int>(std::floor(p.x * inv)),
                 static_cast<int>(std::floor(p.y * inv)))].push_back(p.z);
    }

    // 2. One representative per cell: low-percentile Z at the cell centre
    //    (robust to a few high rock points inside the cell).
    struct CellRep { float x, y, z; };
    std::unordered_map<std::int64_t, CellRep> cell_rep;
    cell_rep.reserve(cell_z.size());
    for (auto & [k, zs] : cell_z) {
      if (static_cast<int>(zs.size()) < params.min_points_per_cell) {
        continue;  // too sparse to trust
      }
      std::sort(zs.begin(), zs.end());
      const std::size_t i =
        static_cast<std::size_t>(params.percentile * static_cast<float>(zs.size() - 1));
      int ix, iy;
      unkey(k, ix, iy);
      cell_rep[k] = CellRep{
        (static_cast<float>(ix) + 0.5f) * params.cell_m,
        (static_cast<float>(iy) + 0.5f) * params.cell_m,
        zs[i]};
    }

    const int radius = std::max(1, static_cast<int>(std::lround(params.window_m * inv)));

    // 3. Fit one local terrain plane per cell from the representatives in its
    //    window (a tilted plane z = a*x + b*y + c, coords centred on the cell).
    std::unordered_map<std::int64_t, std::array<float, 3>> cell_plane;
    cell_plane.reserve(cell_rep.size());
    for (const auto & [k, rep] : cell_rep) {
      int cx, cy;
      unkey(k, cx, cy);
      std::vector<CellRep> pts;
      pts.reserve(static_cast<std::size_t>((2 * radius + 1) * (2 * radius + 1)));
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          const auto it = cell_rep.find(key(cx + dx, cy + dy));
          if (it != cell_rep.end()) {
            // Centre on the query cell for numerical conditioning.
            pts.push_back(CellRep{it->second.x - rep.x, it->second.y - rep.y, it->second.z});
          }
        }
      }
      std::array<double, 3> coeff;
      if (!fit_plane(pts, coeff)) {
        continue;  // degenerate (too few / collinear) -> no plane for this cell
      }
      // Trim likely rock cells (residual above threshold) and refit once.
      std::vector<CellRep> kept;
      kept.reserve(pts.size());
      for (const auto & q : pts) {
        const double pred = coeff[0] * q.x + coeff[1] * q.y + coeff[2];
        if (q.z - pred <= params.height_threshold_m) {
          kept.push_back(q);
        }
      }
      if (kept.size() >= 4 && kept.size() < pts.size()) {
        std::array<double, 3> refit;
        if (fit_plane(kept, refit)) {
          coeff = refit;
        }
      }
      cell_plane[k] = {static_cast<float>(coeff[0]), static_cast<float>(coeff[1]),
                       static_cast<float>(coeff[2])};
    }

    // 4. Classify each ground point against its cell plane.
    for (const auto & p : csf_ground.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.z)) {
        ground_out.push_back(p);
        continue;
      }
      const int cx = static_cast<int>(std::floor(p.x * inv));
      const int cy = static_cast<int>(std::floor(p.y * inv));
      const std::int64_t k = key(cx, cy);
      const auto pit = cell_plane.find(k);
      const auto rit = cell_rep.find(k);
      if (pit == cell_plane.end() || rit == cell_rep.end()) {
        ground_out.push_back(p);  // no local plane -> keep as ground
        continue;
      }
      // Plane was fit in coords centred on the cell representative.
      const float dx = p.x - rit->second.x;
      const float dy = p.y - rit->second.y;
      const float pred = pit->second[0] * dx + pit->second[1] * dy + pit->second[2];
      if (p.z - pred > params.height_threshold_m) {
        objects_out.push_back(p);  // sits above local terrain plane -> object (rock)
      } else {
        ground_out.push_back(p);
      }
    }

    finalize(ground_out);
    finalize(objects_out);
  }

private:
  static std::int64_t key(int ix, int iy)
  {
    return (static_cast<std::int64_t>(ix) << 32) |
           (static_cast<std::int64_t>(iy) & 0xffffffffLL);
  }

  static void unkey(std::int64_t k, int & ix, int & iy)
  {
    ix = static_cast<int>(static_cast<std::int32_t>(k >> 32));
    iy = static_cast<int>(static_cast<std::int32_t>(k & 0xffffffffLL));
  }

  /**
   * Least-squares fit of z = a*x + b*y + c. Returns false if degenerate.
   */
  template <typename Pt>
  static bool fit_plane(const std::vector<Pt> & pts, std::array<double, 3> & out)
  {
    if (pts.size() < 4) {
      return false;
    }
    double Sxx = 0, Sxy = 0, Sx = 0, Syy = 0, Sy = 0, Sxz = 0, Syz = 0, Sz = 0;
    const double n = static_cast<double>(pts.size());
    for (const auto & p : pts) {
      Sxx += static_cast<double>(p.x) * p.x;
      Sxy += static_cast<double>(p.x) * p.y;
      Sx += p.x;
      Syy += static_cast<double>(p.y) * p.y;
      Sy += p.y;
      Sxz += static_cast<double>(p.x) * p.z;
      Syz += static_cast<double>(p.y) * p.z;
      Sz += p.z;
    }
    const double M[3][3] = {{Sxx, Sxy, Sx}, {Sxy, Syy, Sy}, {Sx, Sy, n}};
    const double r[3] = {Sxz, Syz, Sz};
    const double det =
      M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
      M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
      M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    if (std::abs(det) < 1e-9) {
      return false;  // collinear points -> no unique plane
    }
    const double inv_det = 1.0 / det;
    auto solve_col = [&](int col) {
      double Mc[3][3];
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          Mc[i][j] = (j == col) ? r[i] : M[i][j];
        }
      }
      return (Mc[0][0] * (Mc[1][1] * Mc[2][2] - Mc[1][2] * Mc[2][1]) -
              Mc[0][1] * (Mc[1][0] * Mc[2][2] - Mc[1][2] * Mc[2][0]) +
              Mc[0][2] * (Mc[1][0] * Mc[2][1] - Mc[1][1] * Mc[2][0])) *
             inv_det;
    };
    out[0] = solve_col(0);
    out[1] = solve_col(1);
    out[2] = solve_col(2);
    return true;
  }

  static void finalize(pcl::PointCloud<pcl::PointXYZ> & c)
  {
    c.width = static_cast<std::uint32_t>(c.size());
    c.height = 1;
    c.is_dense = true;
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__OBJECT_ON_GROUND_FILTER_HPP_
