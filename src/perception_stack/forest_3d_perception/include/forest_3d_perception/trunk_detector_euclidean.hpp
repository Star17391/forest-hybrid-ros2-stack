/**
 * @file trunk_detector_euclidean.hpp
 * @brief Robust trunk detection for sparse mobile LiDAR.
 *
 * Pipeline: nDSM band → 3D Euclidean clustering → geometry validation
 * → cylinder fit → ground anchor → scored output.
 *
 * Unlike per-slice grid BFS, this handles naturally the 5-15 pts/trunk
 * typical at 3-10 m range on a mobile robot.
 */

#ifndef FOREST_3D_PERCEPTION__TRUNK_DETECTOR_EUCLIDEAN_HPP_
#define FOREST_3D_PERCEPTION__TRUNK_DETECTOR_EUCLIDEAN_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "forest_3d_perception/cylinder_fit.hpp"
#include "forest_3d_perception/ndsm_field.hpp"
#include "forest_3d_perception/terrain_grid_2d.hpp"
#include "forest_3d_perception/trunk_pipeline_audit.hpp"

namespace forest_3d_perception
{

struct EuclideanTrunkParams
{
  float ndsm_min_m{0.20f};
  float ndsm_max_m{2.8f};

  // Euclidean clustering
  float cluster_tolerance_m{0.22f};
  int min_cluster_size{6};
  int max_cluster_size{600};

  // Geometry validation
  float min_height_m{0.40f};
  float max_diameter_m{0.80f};
  float min_verticality{0.55f};

  // Cylinder fit
  float cylinder_max_rmse_m{0.18f};
  float cylinder_min_inlier_ratio{0.35f};
  float cylinder_inlier_dist_m{0.14f};
  float cylinder_max_radius_m{0.45f};
  float cylinder_max_height_m{2.8f};

  // Ground anchor
  float ground_max_base_dz_m{0.80f};

  int max_detections_per_frame{12};
};

struct EuclideanTrunkDetection
{
  CylinderObservation cylinder;
  std::vector<std::size_t> point_indices;
  float verticality{0.0f};
  float score{0.0f};
  bool ground_anchored{false};
};

struct EuclideanDebugClouds
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ndsm_band;
  pcl::PointCloud<pcl::PointXYZ>::Ptr clusters_valid;
  pcl::PointCloud<pcl::PointXYZ>::Ptr rejected;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accepted;
};

class TrunkDetectorEuclidean
{
public:
  EuclideanTrunkParams params;

  std::vector<EuclideanTrunkDetection> detect(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const TerrainGrid2D & grid,
    const std::vector<uint8_t> & cell_connected,
    TrunkPipelineFunnel * funnel = nullptr,
    EuclideanDebugClouds * debug = nullptr) const
  {
    TrunkPipelineFunnel f;
    f.n_nonground_in = cloud.size();

    NdsmStats ndsm_stats;
    const auto band = NdsmField::compute_trunk_band(
      cloud, grid, params.ndsm_min_m, params.ndsm_max_m, &ndsm_stats);
    f.n_band_points = band.size();
    f.n_band_skip_nan_ground = ndsm_stats.n_skip_nan_ground;
    f.n_band_skip_height = ndsm_stats.n_skip_height_band;

    // Build band cloud for Euclidean clustering
    pcl::PointCloud<pcl::PointXYZ>::Ptr band_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    band_cloud->reserve(band.size());
    std::vector<std::size_t> band_to_orig;
    band_to_orig.reserve(band.size());
    for (const auto & np : band) {
      band_cloud->push_back(cloud.points[np.index]);
      band_to_orig.push_back(np.index);
    }

    if (debug) {
      debug->ndsm_band = band_cloud;
      debug->clusters_valid.reset(new pcl::PointCloud<pcl::PointXYZ>);
      debug->rejected.reset(new pcl::PointCloud<pcl::PointXYZ>);
      debug->accepted.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    if (band_cloud->size() < static_cast<std::size_t>(params.min_cluster_size)) {
      if (funnel) { *funnel = f; }
      return {};
    }

    // 3D Euclidean clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(band_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(params.cluster_tolerance_m);
    ec.setMinClusterSize(params.min_cluster_size);
    ec.setMaxClusterSize(params.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(band_cloud);
    ec.extract(cluster_indices);

    f.n_clusters_2d = cluster_indices.size();

    std::vector<EuclideanTrunkDetection> detections;

    for (const auto & ci : cluster_indices) {
      // Compute bounding box and PCA
      pcl::PointXYZ min_pt, max_pt;
      float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
      for (int idx : ci.indices) {
        const auto & p = band_cloud->points[idx];
        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;
      }
      const float inv_n = 1.0f / static_cast<float>(ci.indices.size());
      const float cx = sum_x * inv_n;
      const float cy = sum_y * inv_n;
      const float cz = sum_z * inv_n;

      float z_min = 1e9f, z_max = -1e9f;
      float max_r = 0.0f;
      for (int idx : ci.indices) {
        const auto & p = band_cloud->points[idx];
        z_min = std::min(z_min, p.z);
        z_max = std::max(z_max, p.z);
        const float dx = p.x - cx;
        const float dy = p.y - cy;
        max_r = std::max(max_r, std::sqrt(dx * dx + dy * dy));
      }

      const float height = z_max - z_min;
      const float diameter = max_r * 2.0f;

      // Height check
      if (height < params.min_height_m) {
        ++f.reject_sparse_few_slices;
        push_debug_rejected(debug, band_cloud, ci);
        continue;
      }

      // Diameter check
      if (diameter > params.max_diameter_m) {
        ++f.reject_cylinder_wide;
        push_debug_rejected(debug, band_cloud, ci);
        continue;
      }

      // PCA verticality
      float verticality = compute_verticality(band_cloud, ci);
      if (verticality < params.min_verticality) {
        ++f.reject_cont_drift;
        push_debug_rejected(debug, band_cloud, ci);
        continue;
      }

      // Ground anchor: base of cluster should be near ground
      const float zg = grid.ground_reference_at(cx, cy);
      const float base_dz = std::isnan(zg) ? 0.0f : (z_min - zg);
      if (!std::isnan(zg) && (base_dz < -0.3f || base_dz > params.ground_max_base_dz_m)) {
        ++f.reject_ground_dz;
        push_debug_rejected(debug, band_cloud, ci);
        continue;
      }

      // Ground connectivity check
      if (!cell_connected.empty()) {
        bool near_connected = false;
        for (int dy = -2; dy <= 2 && !near_connected; ++dy) {
          for (int dx = -2; dx <= 2 && !near_connected; ++dx) {
            const float x = cx + static_cast<float>(dx) * grid.resolution_m;
            const float y = cy + static_cast<float>(dy) * grid.resolution_m;
            std::size_t ix = 0, iy = 0;
            if (!grid.index_for(x, y, ix, iy)) { continue; }
            const std::size_t nidx = iy * grid.width + ix;
            if (nidx < cell_connected.size() && cell_connected[nidx] != 0) {
              near_connected = true;
            }
          }
        }
        if (!near_connected) {
          ++f.reject_ground_cell;
          push_debug_rejected(debug, band_cloud, ci);
          continue;
        }
      }

      // Collect original indices for cylinder fit
      std::vector<std::size_t> orig_indices;
      orig_indices.reserve(ci.indices.size());
      for (int idx : ci.indices) {
        orig_indices.push_back(band_to_orig[idx]);
      }

      // Cylinder fit
      CylinderObservation cyl;
      const auto rej = fit_vertical_cylinder(
        cloud, orig_indices, cyl,
        static_cast<double>(params.min_height_m),
        static_cast<double>(params.cylinder_max_radius_m),
        static_cast<double>(params.cylinder_max_rmse_m),
        static_cast<double>(params.cylinder_min_inlier_ratio),
        static_cast<double>(params.cylinder_inlier_dist_m),
        static_cast<double>(params.cylinder_max_height_m));

      if (rej != CylinderReject::Accepted) {
        switch (rej) {
          case CylinderReject::TooShort: ++f.reject_cylinder_short; break;
          case CylinderReject::TooWide: ++f.reject_cylinder_wide; break;
          case CylinderReject::HighRmse: ++f.reject_cylinder_rmse; break;
          case CylinderReject::LowInliers: ++f.reject_cylinder_inliers; break;
          default: break;
        }
        push_debug_rejected(debug, band_cloud, ci);
        continue;
      }

      // Score: combined quality metric
      float score = 0.3f * verticality +
        0.3f * std::min(1.0f, height / 1.5f) +
        0.2f * cyl.inlier_ratio +
        0.2f * std::max(0.0f, 1.0f - cyl.rmse / params.cylinder_max_rmse_m);

      EuclideanTrunkDetection det;
      det.cylinder = cyl;
      det.point_indices = std::move(orig_indices);
      det.verticality = verticality;
      det.score = score;
      det.ground_anchored = !std::isnan(zg);
      detections.push_back(std::move(det));
      ++f.n_accepted_cylinders;

      if (debug && debug->accepted) {
        for (int idx : ci.indices) {
          debug->accepted->push_back(band_cloud->points[idx]);
        }
      }
    }

    // Keep top N by score
    if (detections.size() > static_cast<std::size_t>(params.max_detections_per_frame)) {
      std::sort(detections.begin(), detections.end(),
        [](const EuclideanTrunkDetection & a, const EuclideanTrunkDetection & b) {
          return a.score > b.score;
        });
      detections.resize(params.max_detections_per_frame);
    }

    f.n_stems_finished = cluster_indices.size();
    if (funnel) { *funnel = f; }
    return detections;
  }

private:
  static float compute_verticality(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const pcl::PointIndices & indices)
  {
    if (indices.indices.size() < 4) {
      return 0.0f;
    }

    // Compute covariance manually (avoid PCA crash on degenerate input)
    double sx = 0, sy = 0, sz = 0;
    for (int idx : indices.indices) {
      sx += cloud->points[idx].x;
      sy += cloud->points[idx].y;
      sz += cloud->points[idx].z;
    }
    const double n = static_cast<double>(indices.indices.size());
    const double mx = sx / n, my = sy / n, mz = sz / n;

    double cxx = 0, cyy = 0, czz = 0, cxy = 0, cxz = 0, cyz = 0;
    for (int idx : indices.indices) {
      const double dx = cloud->points[idx].x - mx;
      const double dy = cloud->points[idx].y - my;
      const double dz = cloud->points[idx].z - mz;
      cxx += dx * dx; cyy += dy * dy; czz += dz * dz;
      cxy += dx * dy; cxz += dx * dz; cyz += dy * dz;
    }
    cxx /= n; cyy /= n; czz /= n; cxy /= n; cxz /= n; cyz /= n;

    // For a vertical trunk, Z variance dominates. Verticality = czz / total_variance
    const double total_var = cxx + cyy + czz;
    if (total_var < 1e-8) { return 0.0f; }
    return static_cast<float>(czz / total_var);
  }

  static void push_debug_rejected(
    EuclideanDebugClouds * debug,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const pcl::PointIndices & ci)
  {
    if (!debug || !debug->rejected) { return; }
    for (int idx : ci.indices) {
      debug->rejected->push_back(cloud->points[idx]);
    }
  }
};

}  // namespace forest_3d_perception

#endif  // FOREST_3D_PERCEPTION__TRUNK_DETECTOR_EUCLIDEAN_HPP_
