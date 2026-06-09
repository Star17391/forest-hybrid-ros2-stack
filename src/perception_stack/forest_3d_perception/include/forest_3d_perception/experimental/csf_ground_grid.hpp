/**
 * @file csf_ground_grid.hpp
 * @brief Lightweight 2D ground height reference built from CSF ground points (Sprint 3).
 *
 * Each occupied cell stores the minimum Z of all CSF-ground points that fall
 * into it. Used to compute nDSM height (HAG) for non-ground points.
 *
 * INPAINT: CSF ground returns are sparse and, crucially, ABSENT directly under
 * a trunk (the trunk occludes the ground beneath it). Without filling, those
 * cells have no ground reference, so trunk points get NaN HAG and are dropped —
 * starving every downstream stage. After populating observed cells we therefore
 * run a few dilation passes that fill empty cells from the mean of their filled
 * neighbours, so a trunk footprint inherits the surrounding ground height.
 * Source stays the CSF ground points; this only densifies the grid built on top.
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__CSF_GROUND_GRID_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__CSF_GROUND_GRID_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace forest_3d_perception::experimental
{

class CsfGroundGrid
{
public:
  float resolution_m{0.20f};
  int inpaint_passes{6};   // dilation passes to fill empty cells (0 = observed only)

  void build(const pcl::PointCloud<pcl::PointXYZ> & ground_cloud)
  {
    cells_.clear();
    if (ground_cloud.empty()) {
      return;
    }
    const float inv = 1.0f / resolution_m;
    for (const auto & p : ground_cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.z)) {
        continue;
      }
      const int ix = static_cast<int>(std::floor(p.x * inv));
      const int iy = static_cast<int>(std::floor(p.y * inv));
      const auto key = make_key(ix, iy);
      auto it = cells_.find(key);
      if (it == cells_.end()) {
        cells_[key] = p.z;
      } else if (p.z < it->second) {
        it->second = p.z;
      }
    }
    inpaint();
  }

  /** Returns height of `point_z` above the nearest ground reference, or NaN if unknown. */
  float height_above_ground(float x, float y, float point_z) const
  {
    const float inv = 1.0f / resolution_m;
    const int cx = static_cast<int>(std::floor(x * inv));
    const int cy = static_cast<int>(std::floor(y * inv));

    // Search 3×3 neighbourhood for the lowest available ground reference.
    float z_ground = std::numeric_limits<float>::max();
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const auto it = cells_.find(make_key(cx + dx, cy + dy));
        if (it != cells_.end() && it->second < z_ground) {
          z_ground = it->second;
        }
      }
    }
    if (z_ground == std::numeric_limits<float>::max()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return point_z - z_ground;
  }

  bool empty() const { return cells_.empty(); }

private:
  std::unordered_map<std::int64_t, float> cells_;

  static std::int64_t make_key(int ix, int iy)
  {
    return (static_cast<std::int64_t>(ix) << 32) |
           (static_cast<std::int64_t>(iy) & 0xffffffffLL);
  }

  static void decode_key(std::int64_t key, int & ix, int & iy)
  {
    ix = static_cast<int>(static_cast<std::int32_t>(key >> 32));
    iy = static_cast<int>(static_cast<std::int32_t>(key & 0xffffffffLL));
  }

  /**
   * Dilate the observed cells: each pass fills empty 8-neighbour cells with the
   * mean Z of their currently-filled neighbours. Bounded by inpaint_passes, so
   * the fill only reaches that many cells out from observed ground (enough to
   * cover trunk footprints and small gaps, not the whole grid).
   */
  void inpaint()
  {
    for (int pass = 0; pass < inpaint_passes; ++pass) {
      std::unordered_map<std::int64_t, std::pair<double, int>> pending;
      for (const auto & kv : cells_) {
        int ix = 0;
        int iy = 0;
        decode_key(kv.first, ix, iy);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const auto nkey = make_key(ix + dx, iy + dy);
            if (cells_.find(nkey) != cells_.end()) {
              continue;  // already has a ground reference
            }
            auto & acc = pending[nkey];
            acc.first += kv.second;
            acc.second += 1;
          }
        }
      }
      if (pending.empty()) {
        break;
      }
      for (const auto & pk : pending) {
        cells_[pk.first] = static_cast<float>(pk.second.first / pk.second.second);
      }
    }
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__CSF_GROUND_GRID_HPP_
