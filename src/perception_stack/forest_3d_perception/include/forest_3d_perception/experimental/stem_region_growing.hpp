/**
 * @file stem_region_growing.hpp
 * @brief Ground-seeded vertical region growing (experimental, Sprint 3.5 — Option 2).
 *
 * WHY THIS REPLACES THE LINEARITY SPLIT
 * -------------------------------------
 * The per-point linearity split (Option 1) made a LOCAL, per-point decision and
 * had two structural faults: it FRAGMENTED trees (some trunk points linear ->
 * set A, others -> set B, so one tree became two clusters) and it had no notion
 * of the whole object or the ground. That hurt trunk/shrub separation.
 *
 * Option 2 reasons about CONNECTED STRUCTURE anchored to the ground:
 *
 *   1. Compute height-above-ground (HAG) for every non-ground point.
 *   2. SEEDS = points near the base (HAG <= seed_max_hag): ground-connected.
 *   3. Region-grow (BFS, 3D radius connectivity) UPWARD from each seed, capped
 *      at growth_max_hag (skip the high canopy, like the old band did).
 *   4. Each grown region = one ground-anchored object (no fragmentation).
 *   5. Points NOT reachable from any ground seed = floating canopy/branches with
 *      a vertical gap to the ground -> EXCLUDED.
 *
 * This delivers two things the user asked for, by construction:
 *   - GROUND CONNECTION: every cluster (rock, trunk, shrub) is seeded at the
 *     base and grown up, so nothing floats. A "rock up in a tree" is impossible.
 *   - NO FRAGMENTATION: a tree grows as a single connected region, so the
 *     classifier sees the whole trunk instead of two half-objects.
 *
 * Cost: KdTree-bound, same order as Option 1; real-time on the voxelised cloud.
 * This is the pattern robotic-forestry stacks (DigiForest, 3DFin-online) run live.
 *
 * Output reuses PointCluster (point_indices reference the non-ground cloud) so
 * the node's existing classifier / marker / labeled-cloud paths work unchanged.
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__STEM_REGION_GROWING_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__STEM_REGION_GROWING_HPP_

#include <cmath>
#include <cstddef>
#include <queue>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include "forest_3d_perception/experimental/csf_ground_grid.hpp"
#include "forest_3d_perception/experimental/euclidean_clustering.hpp"

namespace forest_3d_perception::experimental
{

struct RegionGrowParams
{
  bool enabled{true};
  float ground_grid_resolution_m{0.20f};  // CSF ground reference resolution
  int ground_inpaint_passes{6};           // densify the sparse CSF ground grid (fill holes
                                          // under trunks) so HAG is available everywhere
  float seed_max_hag_m{0.70f};   // points at/below this HAG seed a region (ground-connected).
                                 // Must clear the CSF class_threshold: CSF absorbs the
                                 // trunk base as ground, so non-ground trunk points only
                                 // start ~class_threshold above the floor.
  float growth_radius_m{0.30f};  // horizontal neighbour radius for connectivity
  float growth_z_scale{0.20f};   // <1 compresses Z in the search so vertical neighbours
                                 // connect more easily. LiDAR trunks have VERTICAL GAPS
                                 // (occlusion / sparse mid-trunk returns); a small z_scale
                                 // bridges them so the whole trunk grows as ONE tall region
                                 // instead of a short base blob (which misclassifies as ROCK).
                                 // Effective vertical reach = growth_radius / growth_z_scale
                                 // (0.30 / 0.20 = 1.5 m). Horizontal reach stays growth_radius,
                                 // so neighbouring trees do not glue.
  float growth_max_hag_m{3.00f}; // do not grow above this HAG (skip the canopy)
  float ground_tolerance_m{0.20f};  // tolerate points slightly below estimated ground
  int min_region_pts{5};
  int max_region_pts{5000};
};

struct RegionGrowResult
{
  std::vector<PointCluster> clusters;                 // point_indices into non-ground
  pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud;  // candidates (HAG in [~0, max])
  pcl::PointCloud<pcl::PointXYZ>::Ptr grown_cloud;    // points assigned to a region
  std::size_t n_working{0};
  std::size_t n_seeds{0};                             // base points eligible to seed a region
  std::size_t n_grown{0};
  // Region-discard diagnostics (root-cause of "seeds -> few clusters"): how many
  // connected components were grown from a seed, and how many were thrown away for
  // being too small (< min_region_pts, i.e. a fragmented/sparse trunk) vs too large
  // (> max_region_pts, an understory bridge). Lets us tell a real recall loss
  // (many small discards) from correct seed collapse (few components, no discards).
  std::size_t n_regions_started{0};
  std::size_t n_discarded_small{0};
  std::size_t n_discarded_large{0};
  std::size_t pts_in_discarded_small{0};              // points lost in small regions
  std::size_t largest_discarded_small{0};             // size of the biggest small discard
};

class StemRegionGrower
{
public:
  RegionGrowParams params;

  RegionGrowResult grow(
    const pcl::PointCloud<pcl::PointXYZ> & ground_cloud,
    const pcl::PointCloud<pcl::PointXYZ> & non_ground_cloud) const
  {
    RegionGrowResult out;
    out.working_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    out.grown_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // Ground reference from CSF ground points. The grid is now INPAINTED (empty
    // cells filled from neighbours) so the trunk footprint — where the trunk
    // occludes the ground directly beneath it, leaving no observed ground cell —
    // still gets a height. Without inpaint, those points returned NaN HAG and
    // were dropped, starving the working set (e.g. 56 of 988 non-ground points).
    CsfGroundGrid grid;
    grid.resolution_m = params.ground_grid_resolution_m;
    grid.inpaint_passes = params.ground_inpaint_passes;
    grid.build(ground_cloud);
    if (grid.empty() || non_ground_cloud.empty()) {
      return out;
    }

    // 1. Working set: non-ground points with HAG in [-tol, growth_max_hag].
    //    Keep the mapping work-index -> original non-ground index, plus HAG.
    std::vector<std::size_t> orig_index;
    std::vector<float> hag;
    orig_index.reserve(non_ground_cloud.size());
    hag.reserve(non_ground_cloud.size());
    out.working_cloud->reserve(non_ground_cloud.size());
    for (std::size_t i = 0; i < non_ground_cloud.size(); ++i) {
      const auto & p = non_ground_cloud.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      const float h = grid.height_above_ground(p.x, p.y, p.z);
      if (std::isnan(h) || h < -params.ground_tolerance_m || h > params.growth_max_hag_m) {
        continue;
      }
      out.working_cloud->push_back(p);
      orig_index.push_back(i);
      hag.push_back(h);
    }
    out.working_cloud->width = static_cast<std::uint32_t>(out.working_cloud->size());
    out.working_cloud->height = 1;
    out.working_cloud->is_dense = true;
    out.n_working = out.working_cloud->size();
    if (out.working_cloud->size() < static_cast<std::size_t>(params.min_region_pts)) {
      return out;
    }

    // 2. KdTree over an ANISOTROPIC copy: Z is compressed by growth_z_scale so
    //    vertical neighbours connect more easily than horizontal ones. This lets
    //    the BFS climb a trunk whose LiDAR rings are spaced vertically by more
    //    than growth_radius, WITHOUT widening horizontal reach (which would glue
    //    neighbouring trees). Effective vertical reach = radius / z_scale.
    pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud(
      new pcl::PointCloud<pcl::PointXYZ>(*out.working_cloud));
    for (auto & p : search_cloud->points) {
      p.z *= params.growth_z_scale;
    }
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(search_cloud);

    // 3. BFS region growing from ground seeds. A region is only ever STARTED
    //    from an unvisited seed (HAG <= seed_max), so connected components with
    //    no ground seed (floating canopy) are never grown -> excluded.
    std::vector<char> visited(out.working_cloud->size(), 0);
    std::vector<int> idx;
    std::vector<float> d2;
    int region_id = 0;

    for (std::size_t s = 0; s < out.working_cloud->size(); ++s) {
      if (hag[s] <= params.seed_max_hag_m) {
        ++out.n_seeds;
      }
      if (visited[s] || hag[s] > params.seed_max_hag_m) {
        continue;
      }
      // Grow the connected component reachable from this ground seed.
      std::vector<std::size_t> region;
      std::queue<std::size_t> q;
      q.push(s);
      visited[s] = 1;
      while (!q.empty()) {
        const std::size_t cur = q.front();
        q.pop();
        region.push_back(cur);
        idx.clear();
        d2.clear();
        tree->radiusSearch(search_cloud->points[cur], params.growth_radius_m, idx, d2);
        for (int j : idx) {
          if (!visited[j]) {
            visited[j] = 1;  // HAG already <= max by working-set construction
            q.push(static_cast<std::size_t>(j));
          }
        }
      }

      ++out.n_regions_started;
      if (region.size() < static_cast<std::size_t>(params.min_region_pts)) {
        ++out.n_discarded_small;
        out.pts_in_discarded_small += region.size();
        out.largest_discarded_small = std::max(out.largest_discarded_small, region.size());
        continue;  // too small (noise OR a fragmented/sparse trunk)
      }
      if (region.size() > static_cast<std::size_t>(params.max_region_pts)) {
        ++out.n_discarded_large;
        continue;  // too large (understory bridge); drop
      }

      PointCluster c;
      c.id = region_id++;
      c.point_indices.reserve(region.size());
      c.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      c.cloud->reserve(region.size());
      for (std::size_t w : region) {
        const std::size_t orig = orig_index[w];
        c.point_indices.push_back(orig);
        c.cloud->push_back(non_ground_cloud.points[orig]);
        out.grown_cloud->push_back(non_ground_cloud.points[orig]);
      }
      c.cloud->width = static_cast<std::uint32_t>(c.cloud->size());
      c.cloud->height = 1;
      c.cloud->is_dense = true;
      out.clusters.push_back(std::move(c));
    }

    out.grown_cloud->width = static_cast<std::uint32_t>(out.grown_cloud->size());
    out.grown_cloud->height = 1;
    out.grown_cloud->is_dense = true;
    out.n_grown = out.grown_cloud->size();
    return out;
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__STEM_REGION_GROWING_HPP_
