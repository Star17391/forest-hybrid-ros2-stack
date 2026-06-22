/**
 * @file columnar_ground_recovery.hpp
 * @brief Recover object bases (trunk/rock feet) that CSF stole into the ground,
 *        using VERTICAL CONNECTIVITY instead of a fixed height threshold.
 *
 * Motivation: CSF is a terrain filter — the foot of a vertical object lies right
 * on the terrain, so CSF swallows it as ground. The `object_on_ground_filter`
 * recovers it only if it pokes above a local plane by a FIXED margin (~12 cm),
 * which loses thin trunk feet and low rock bases.
 *
 * Idea (forestry literature — columnar / vertical-connectivity recovery):
 *   A CSF-ground point belongs to an OBJECT, not the terrain, if its XY column
 *   contains an above-ground STRUCTURE (non-ground points rising well above the
 *   true floor). In that case any "ground" point in the column that sits above the
 *   true floor is the object's stolen base → reclassify it as non-ground.
 *
 *   per XY cell:  z_floor = min z over (ground ∪ non_ground)   (the real terrain)
 *                 has_object = (max non_ground z − z_floor) > min_object_height
 *   per ground point p in a has_object cell:
 *                 p.z > z_floor + ground_margin  →  recovered (object base)
 *                 else                           →  stays ground
 *
 * Crucially there is NO absolute height cutoff: the base is recovered to whatever
 * height it actually starts, conditioned on a real object being above it. Ground
 * points in columns WITHOUT an object are never touched, so isolated terrain noise
 * is not recovered.
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__COLUMNAR_GROUND_RECOVERY_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__COLUMNAR_GROUND_RECOVERY_HPP_

#include <algorithm>
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

struct ColumnarRecoveryParams
{
  bool enabled{true};
  float cell_m{0.20f};               // XY column resolution
  float ground_margin_m{0.06f};      // a ground point above the LOCAL floor by more than
                                     //   this (in an object column) is the object's base
  float min_object_height_m{0.40f};  // a column "has an object" if non_ground rises
                                     //   this far above the floor (filters low clutter)
  int neighbor_radius_cells{1};      // also count an object in adjacent cells (a trunk
                                     //   foot spreads over a couple of cells)
  int floor_window_cells{2};         // the true ground under a trunk is in the cells
                                     //   AROUND it — take the floor from this window so
                                     //   a trunk cell inherits the surrounding ground
                                     //   (else the stolen base IS the cell's own floor)
  float floor_percentile{0.25f};     // LOW PERCENTILE (not the min) of the window's
                                     //   floors: robust to slopes/rough terrain where
                                     //   the min underestimates the local ground and
                                     //   would falsely recover legit ground points
  float max_recover_m{0.50f};        // only recover the LOW base: a point more than this
                                     //   above the floor is the object itself, not its
                                     //   stolen foot — never recover that high
};

struct ColumnarRecoveryStats
{
  std::size_t n_object_cells{0};
  std::size_t n_recovered{0};
};

class ColumnarGroundRecovery
{
public:
  ColumnarRecoveryParams params;

  /**
   * Split CSF ground into refined ground (real terrain) and recovered object-base
   * points. `recovered_out` should be merged into the non-ground cloud upstream of
   * region growing so the base feeds the trunk/rock clusters. Outputs are cleared.
   */
  ColumnarRecoveryStats refine(
    const pcl::PointCloud<pcl::PointXYZ> & csf_ground,
    const pcl::PointCloud<pcl::PointXYZ> & non_ground,
    pcl::PointCloud<pcl::PointXYZ> & ground_out,
    pcl::PointCloud<pcl::PointXYZ> & recovered_out) const
  {
    ground_out.clear();
    recovered_out.clear();
    ColumnarRecoveryStats stats;
    if (csf_ground.empty()) {
      return stats;
    }

    const float inv = 1.0f / params.cell_m;

    // Per cell: floor (min z over ground+non_ground) and the top of the non-ground
    // structure (max non_ground z, or -inf if none).
    struct Col { float floor; float ng_top; };
    std::unordered_map<std::int64_t, Col> cells;
    cells.reserve(csf_ground.size() + non_ground.size());

    auto touch_floor = [&](float x, float y, float z) {
      if (!std::isfinite(x) || !std::isfinite(z)) {
        return;
      }
      const std::int64_t k = key(
        static_cast<int>(std::floor(x * inv)), static_cast<int>(std::floor(y * inv)));
      auto it = cells.find(k);
      if (it == cells.end()) {
        cells.emplace(k, Col{z, -std::numeric_limits<float>::max()});
      } else {
        it->second.floor = std::min(it->second.floor, z);
      }
    };
    for (const auto & p : csf_ground.points) { touch_floor(p.x, p.y, p.z); }
    for (const auto & p : non_ground.points) { touch_floor(p.x, p.y, p.z); }
    // Non-ground top per cell.
    for (const auto & p : non_ground.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.z)) {
        continue;
      }
      const std::int64_t k = key(
        static_cast<int>(std::floor(p.x * inv)), static_cast<int>(std::floor(p.y * inv)));
      auto it = cells.find(k);
      if (it != cells.end()) {
        it->second.ng_top = std::max(it->second.ng_top, p.z);
      }
    }

    // LOCAL floor: the true ground under a trunk lives in the cells around it (the
    // trunk cell's own min may BE the stolen base). Use a LOW PERCENTILE of the
    // window's floors — robust to slopes (the plain minimum underestimates the
    // local ground on rough/sloped terrain and would falsely recover real ground).
    std::unordered_map<std::int64_t, float> floor_local;
    floor_local.reserve(cells.size());
    {
      const int w = std::max(0, params.floor_window_cells);
      std::vector<float> fs;
      for (const auto & [k, c] : cells) {
        int cx, cy;
        unkey(k, cx, cy);
        fs.clear();
        for (int dy = -w; dy <= w; ++dy) {
          for (int dx = -w; dx <= w; ++dx) {
            const auto it = cells.find(key(cx + dx, cy + dy));
            if (it != cells.end()) {
              fs.push_back(it->second.floor);
            }
          }
        }
        std::sort(fs.begin(), fs.end());
        const std::size_t i = static_cast<std::size_t>(
          params.floor_percentile * static_cast<float>(fs.size() - 1));
        floor_local[k] = fs[i];
      }
    }

    // A cell has an object if its non-ground structure rises far enough above the
    // LOCAL floor.
    std::unordered_map<std::int64_t, char> has_object;
    has_object.reserve(cells.size());
    for (const auto & [k, c] : cells) {
      const bool obj = (c.ng_top > -std::numeric_limits<float>::max()) &&
                       (c.ng_top - floor_local[k] > params.min_object_height_m);
      if (obj) {
        has_object[k] = 1;
        ++stats.n_object_cells;
      }
    }
    // Object presence also via neighbour cells (a foot spans a couple of cells).
    auto object_near = [&](int cx, int cy) -> bool {
      const int r = std::max(0, params.neighbor_radius_cells);
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (has_object.find(key(cx + dx, cy + dy)) != has_object.end()) {
            return true;
          }
        }
      }
      return false;
    };

    // Reclassify ground points: above the column floor + margin, in an object
    // column → recovered base; otherwise real terrain.
    for (const auto & p : csf_ground.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.z)) {
        ground_out.push_back(p);
        continue;
      }
      const int cx = static_cast<int>(std::floor(p.x * inv));
      const int cy = static_cast<int>(std::floor(p.y * inv));
      const auto it = floor_local.find(key(cx, cy));
      const float floor = (it != floor_local.end()) ? it->second : p.z;
      const float dz = p.z - floor;
      // Recover only the LOW base of a real object column: above the floor margin,
      // but not so high it is the object itself, and only where there is an object.
      if (dz > params.ground_margin_m && dz <= params.max_recover_m && object_near(cx, cy)) {
        recovered_out.push_back(p);
        ++stats.n_recovered;
      } else {
        ground_out.push_back(p);
      }
    }

    finalize(ground_out);
    finalize(recovered_out);
    return stats;
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

  static void finalize(pcl::PointCloud<pcl::PointXYZ> & c)
  {
    c.width = static_cast<std::uint32_t>(c.size());
    c.height = 1;
    c.is_dense = true;
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__COLUMNAR_GROUND_RECOVERY_HPP_
