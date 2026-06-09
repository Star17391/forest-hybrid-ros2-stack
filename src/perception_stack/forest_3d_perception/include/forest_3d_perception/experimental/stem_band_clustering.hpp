/**
 * @file stem_band_clustering.hpp
 * @brief Stem-band 2D clustering of non-ground points (experimental).
 *
 * Replaces naive Euclidean 3D clustering over the whole non-ground cloud, which
 * merges touching canopies and mixes trunk+canopy+undergrowth into one blob.
 *
 * Robotic-forestry consensus (TreeSLAM, 3DFin, DigiForest): cluster only the
 * TRUNK BAND in 2D, not the whole tree in 3D.
 *
 *   ground (CSF)      → CsfGroundGrid (min-Z per cell)
 *   non-ground (CSF)  → nDSM height above ground
 *                     → keep band [band_min, band_max]   (drops canopy that glues trees)
 *                     → 2D XY Euclidean clustering         (each trunk = compact core)
 *
 * Output reuses PointCluster (id, point_indices into non-ground, cloud) so the
 * node's existing marker/labeled-cloud publishers work unchanged.
 *
 * References (local): docs/perception/references/2024_arxiv_treeslam_summary.md,
 *   2025_isprs_3dfin_pipeline_notes.md, FORESTRY_CLUSTERING_LITERATURE.md §4.4.
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__STEM_BAND_CLUSTERING_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__STEM_BAND_CLUSTERING_HPP_

#include <cmath>
#include <cstddef>
#include <vector>

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "forest_3d_perception/experimental/csf_ground_grid.hpp"
#include "forest_3d_perception/experimental/euclidean_clustering.hpp"

namespace forest_3d_perception::experimental
{

struct StemBandParams
{
  float ground_grid_resolution_m{0.20f};
  float band_min_m{0.30f};
  float band_max_m{3.00f};
  float cluster_tolerance_m{0.20f};
  int cluster_min_pts{6};
  int cluster_max_pts{3000};
};

struct StemBandResult
{
  std::vector<PointCluster> clusters;       // point_indices reference the non-ground cloud
  std::size_t n_non_ground_in{0};
  std::size_t n_band_points{0};             // points surviving the nDSM band
  std::size_t n_ground_cells{0};
  pcl::PointCloud<pcl::PointXYZ>::Ptr band_cloud;  // optional debug: the band points (3D)
};

/**
 * Points surviving the nDSM trunk band. `indices` reference the non-ground
 * cloud; `cloud` holds the matching 3D points (aligned 1:1 with `indices`) so
 * callers can run per-point analysis (e.g. linearity) and split the band before
 * clustering.
 */
struct BandExtraction
{
  std::vector<std::size_t> indices;                // into the non-ground cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;       // 3D band points, aligned with indices
};

class StemBandClusterer
{
public:
  StemBandParams params;

  /**
   * Step 1 — extract the nDSM trunk band: non-ground points whose height above
   * the CSF ground is within [band_min, band_max]. Returns indices into the
   * non-ground cloud plus the matching 3D points, so callers can analyse and
   * split the band before clustering.
   */
  BandExtraction extract_band(
    const pcl::PointCloud<pcl::PointXYZ> & ground_cloud,
    const pcl::PointCloud<pcl::PointXYZ> & non_ground_cloud) const
  {
    BandExtraction band;
    band.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    CsfGroundGrid grid;
    grid.resolution_m = params.ground_grid_resolution_m;
    grid.build(ground_cloud);
    if (grid.empty() || non_ground_cloud.empty()) {
      return band;
    }

    band.indices.reserve(non_ground_cloud.size());
    for (std::size_t i = 0; i < non_ground_cloud.size(); ++i) {
      const auto & p = non_ground_cloud.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.z)) {
        continue;
      }
      const float h = grid.height_above_ground(p.x, p.y, p.z);
      if (std::isnan(h) || h < params.band_min_m || h > params.band_max_m) {
        continue;
      }
      band.indices.push_back(i);
      band.cloud->push_back(p);
    }
    band.cloud->width = static_cast<std::uint32_t>(band.cloud->size());
    band.cloud->height = 1;
    band.cloud->is_dense = true;
    return band;
  }

  /**
   * Step 2 — 2D XY Euclidean clustering of a SUBSET of band points (`subset`
   * holds indices into the non-ground cloud). Z is collapsed so proximity is
   * purely horizontal. Cluster ids start at `id_offset` so two disjoint subsets
   * (e.g. trunk set and the rest) get globally-unique ids when merged.
   */
  std::vector<PointCluster> cluster_band_subset(
    const pcl::PointCloud<pcl::PointXYZ> & non_ground_cloud,
    const std::vector<std::size_t> & subset,
    int id_offset) const
  {
    std::vector<PointCluster> clusters;
    if (subset.size() < static_cast<std::size_t>(params.cluster_min_pts)) {
      return clusters;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr flat(new pcl::PointCloud<pcl::PointXYZ>);
    flat->reserve(subset.size());
    for (std::size_t bi : subset) {
      pcl::PointXYZ p2d;
      p2d.x = non_ground_cloud.points[bi].x;
      p2d.y = non_ground_cloud.points[bi].y;
      p2d.z = 0.0f;
      flat->push_back(p2d);
    }
    flat->width = static_cast<std::uint32_t>(flat->size());
    flat->height = 1;
    flat->is_dense = true;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(flat);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(static_cast<double>(params.cluster_tolerance_m));
    ec.setMinClusterSize(params.cluster_min_pts);
    ec.setMaxClusterSize(params.cluster_max_pts);
    ec.setSearchMethod(tree);
    ec.setInputCloud(flat);
    ec.extract(cluster_indices);

    int id = id_offset;
    clusters.reserve(cluster_indices.size());
    for (const auto & ci : cluster_indices) {
      PointCluster c;
      c.id = id++;
      c.point_indices.reserve(ci.indices.size());
      c.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      c.cloud->reserve(ci.indices.size());
      for (int fi : ci.indices) {
        const std::size_t orig = subset[static_cast<std::size_t>(fi)];
        c.point_indices.push_back(orig);
        c.cloud->push_back(non_ground_cloud.points[orig]);
      }
      c.cloud->width = static_cast<std::uint32_t>(c.cloud->size());
      c.cloud->height = 1;
      c.cloud->is_dense = true;
      clusters.push_back(std::move(c));
    }
    return clusters;
  }

  /** Convenience: extract the full band and cluster it in one shot. */
  StemBandResult cluster(
    const pcl::PointCloud<pcl::PointXYZ> & ground_cloud,
    const pcl::PointCloud<pcl::PointXYZ> & non_ground_cloud) const
  {
    StemBandResult out;
    out.n_non_ground_in = non_ground_cloud.size();

    const BandExtraction band = extract_band(ground_cloud, non_ground_cloud);
    out.band_cloud = band.cloud;
    out.n_band_points = band.indices.size();
    out.clusters = cluster_band_subset(non_ground_cloud, band.indices, 0);
    return out;
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__STEM_BAND_CLUSTERING_HPP_
