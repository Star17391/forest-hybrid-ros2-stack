/**
 * @file trunk_slice_detector.hpp
 * @brief Slice-based trunk detection with vertical continuity (TLS / TreeSLAM style).
 *
 * Pipeline: nDSM band → horizontal Z-slices → 2D clustering → inter-slice linking
 * → continuity metrics → ground-connected anchor → robust cylinder fit.
 */

#ifndef FOREST_3D_PERCEPTION__TRUNK_SLICE_DETECTOR_HPP_
#define FOREST_3D_PERCEPTION__TRUNK_SLICE_DETECTOR_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/cylinder_fit.hpp"
#include "forest_3d_perception/ndsm_field.hpp"
#include "forest_3d_perception/terrain_grid_2d.hpp"
#include "forest_3d_perception/trunk_pipeline_audit.hpp"

namespace forest_3d_perception
{

struct SliceCluster2D
{
  int id{0};
  int slice_idx{0};
  float cx{0.0f};
  float cy{0.0f};
  float z_mid{0.0f};
  float radius{0.0f};
  float circularity{0.0f};
  float density{0.0f};
  float elongation{1.0f};
  std::size_t n_points{0};
  std::vector<std::size_t> point_indices;
};

struct StemContinuityMetrics
{
  float vertical_persistence{0.0f};
  float centroid_drift{0.0f};
  float radius_variance{0.0f};
  float slice_occupancy_ratio{0.0f};
  float continuity_score{0.0f};
  int n_slices{0};
  int n_slices_spanned{0};
};

struct SliceTrunkDetection
{
  std::vector<std::size_t> point_indices;
  CylinderObservation cylinder;
  StemContinuityMetrics metrics;
  bool ground_anchored{false};
};

struct SliceDetectionParams
{
  float ndsm_min_m{0.35f};
  float ndsm_max_m{2.4f};
  float slice_height_m{0.18f};
  int max_slices{48};
  float cluster_cell_m{0.14f};
  int min_points_per_cluster{4};
  int min_slices_for_trunk{4};
  float assoc_max_xy_m{0.55f};
  float assoc_max_radius_ratio{0.45f};
  /** Consecutive empty/missed slices before stem is closed (sparse LiDAR). */
  int assoc_max_gap_slices{2};
  float min_continuity_score{0.52f};
  float min_vertical_persistence{0.55f};
  float max_centroid_drift_ratio{1.2f};
  float max_radius_cv{0.35f};
  float ground_anchor_max_xy_m{0.45f};
  float ground_anchor_max_dz_m{0.55f};
  std::size_t max_stems_per_frame{16};
};

/** @deprecated Use TrunkPipelineFunnel — kept for transitional includes. */
using SliceDetectionStats = TrunkPipelineFunnel;

/** Optional RViz debug clouds (intensity = slice index or cluster id). */
struct SliceDebugClouds
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ndsm_band;
  pcl::PointCloud<pcl::PointXYZI>::Ptr clusters_2d;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accepted_points;
  pcl::PointCloud<pcl::PointXYZ>::Ptr rejected_points;
};

class TrunkSliceDetector
{
public:
  SliceDetectionParams params;
  double cylinder_min_height_m{0.40};
  double cylinder_max_radius_m{0.70};
  double cylinder_max_rmse_m{0.16};
  double cylinder_min_inlier_ratio{0.40};
  double cylinder_inlier_dist_m{0.12};
  double cylinder_max_slice_height_m{2.4};

  std::vector<SliceTrunkDetection> detect(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const TerrainGrid2D & grid,
    const std::vector<uint8_t> & cell_connected,
    TrunkPipelineFunnel * funnel = nullptr,
    SliceDebugClouds * debug_clouds = nullptr) const
  {
    TrunkPipelineFunnel local;
    local.n_nonground_in = cloud.size();
    NdsmStats ndsm_stats;
    const auto band = NdsmField::compute_trunk_band(
      cloud, grid, params.ndsm_min_m, params.ndsm_max_m, &ndsm_stats);
    local.n_band_points = band.size();
    local.n_band_skip_nan_ground = ndsm_stats.n_skip_nan_ground;
    local.n_band_skip_height = ndsm_stats.n_skip_height_band;

    SliceDebugClouds dbg_local;
    if (debug_clouds) {
      dbg_local.ndsm_band.reset(new pcl::PointCloud<pcl::PointXYZ>);
      dbg_local.clusters_2d.reset(new pcl::PointCloud<pcl::PointXYZI>);
      dbg_local.accepted_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
      dbg_local.rejected_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    const auto push_ndsm_pt = [&](std::size_t idx) {
      if (dbg_local.ndsm_band) {
        dbg_local.ndsm_band->push_back(cloud.points[idx]);
      }
    };
    for (const auto & np : band) {
      push_ndsm_pt(np.index);
    }

    if (band.size() < static_cast<std::size_t>(params.min_points_per_cluster * 2)) {
      if (funnel) {
        *funnel = local;
      }
      if (debug_clouds) {
        *debug_clouds = dbg_local;
      }
      return {};
    }

    float z_lo = std::numeric_limits<float>::max();
    float z_hi = std::numeric_limits<float>::lowest();
    for (const auto & np : band) {
      const float z = cloud.points[np.index].z;
      z_lo = std::min(z_lo, z);
      z_hi = std::max(z_hi, z);
    }
    if (!(z_hi > z_lo)) {
      if (funnel) {
        *funnel = local;
      }
      if (debug_clouds) {
        *debug_clouds = dbg_local;
      }
      return {};
    }

    const int n_slices = std::min(
      params.max_slices,
      static_cast<int>(std::ceil((z_hi - z_lo) / params.slice_height_m)) + 1);
    local.n_slices_used = static_cast<std::size_t>(n_slices);
    slice_buckets_.assign(static_cast<std::size_t>(n_slices), {});

    std::vector<std::vector<SliceCluster2D>> slices(static_cast<std::size_t>(n_slices));
    int next_cluster_id = 0;

    for (const auto & np : band) {
      const auto & p = cloud.points[np.index];
      const int si = static_cast<int>(
        std::floor((p.z - z_lo) / params.slice_height_m));
      if (si < 0 || si >= n_slices) {
        continue;
      }
      slice_buckets_[static_cast<std::size_t>(si)].push_back(np.index);
    }

    for (int si = 0; si < n_slices; ++si) {
      auto & bucket = slice_buckets_[static_cast<std::size_t>(si)];
      if (bucket.empty()) {
        continue;
      }
      ++local.n_slices_nonempty;
      const auto clusters = cluster_slice_xy(cloud, bucket, si, z_lo, next_cluster_id, &local);
      local.n_clusters_2d += clusters.size();
      local.max_clusters_in_slice = std::max(
        local.max_clusters_in_slice, static_cast<int>(clusters.size()));
      for (const auto & cl : clusters) {
        if (dbg_local.clusters_2d) {
          for (std::size_t idx : cl.point_indices) {
            pcl::PointXYZI pi;
            pi.x = cloud.points[idx].x;
            pi.y = cloud.points[idx].y;
            pi.z = cloud.points[idx].z;
            pi.intensity = static_cast<float>(cl.slice_idx);
            dbg_local.clusters_2d->push_back(pi);
          }
        }
      }
      slices[static_cast<std::size_t>(si)] = clusters;
      bucket.clear();
    }

    struct ActiveStem
    {
      std::vector<int> cluster_ids;
      std::vector<int> slice_indices;
      int last_cluster{-1};
      int last_slice{-1};
      int gap_slices{0};
    };
    std::vector<ActiveStem> active;
    std::vector<ActiveStem> finished;

    for (int si = 0; si < n_slices; ++si) {
      const auto & layer = slices[static_cast<std::size_t>(si)];
      std::vector<bool> layer_used(layer.size(), false);
      std::vector<ActiveStem> next_active;

      for (std::size_t ai = 0; ai < active.size(); ++ai) {
        auto stem = active[ai];
        int best_j = -1;
        float best_cost = std::numeric_limits<float>::max();
        const auto & prev = find_cluster(slices, stem.last_slice, stem.last_cluster);
        if (prev == nullptr) {
          finished.push_back(stem);
          continue;
        }

        for (std::size_t j = 0; j < layer.size(); ++j) {
          if (layer_used[j]) {
            continue;
          }
          const auto & cand = layer[j];
          const float dx = prev->cx - cand.cx;
          const float dy = prev->cy - cand.cy;
          const float dxy = std::hypot(dx, dy);
          if (dxy > params.assoc_max_xy_m) {
            continue;
          }
          const float r_ratio = std::abs(prev->radius - cand.radius) /
            std::max(0.08f, std::max(prev->radius, cand.radius));
          if (r_ratio > params.assoc_max_radius_ratio) {
            continue;
          }
          const float cost = dxy + 0.4f * r_ratio;
          if (cost < best_cost) {
            best_cost = cost;
            best_j = static_cast<int>(j);
          }
        }

        if (best_j >= 0) {
          layer_used[static_cast<std::size_t>(best_j)] = true;
          stem.gap_slices = 0;
          stem.cluster_ids.push_back(layer[static_cast<std::size_t>(best_j)].id);
          stem.slice_indices.push_back(si);
          stem.last_cluster = layer[static_cast<std::size_t>(best_j)].id;
          stem.last_slice = si;
          next_active.push_back(stem);
        } else if (layer.empty()) {
          ++stem.gap_slices;
          if (stem.gap_slices <= params.assoc_max_gap_slices) {
            next_active.push_back(stem);
          } else {
            finished.push_back(stem);
          }
        } else {
          ++stem.gap_slices;
          if (stem.gap_slices <= params.assoc_max_gap_slices) {
            next_active.push_back(stem);
          } else {
            finished.push_back(stem);
          }
        }
      }

      for (std::size_t j = 0; j < layer.size(); ++j) {
        if (layer_used[j]) {
          continue;
        }
        ActiveStem born;
        born.cluster_ids.push_back(layer[j].id);
        born.slice_indices.push_back(si);
        born.last_cluster = layer[j].id;
        born.last_slice = si;
        next_active.push_back(born);
      }
      active.swap(next_active);
    }
    finished.insert(finished.end(), active.begin(), active.end());
    local.n_stems_finished = finished.size();

    auto note_near_miss = [&](int n_sl, const StemContinuityMetrics & m, float bottom_dz) {
      if (n_sl > local.best_rej_slices) {
        local.best_rej_slices = n_sl;
      }
      if (m.continuity_score > local.best_rej_continuity) {
        local.best_rej_continuity = m.continuity_score;
        local.best_rej_drift = m.centroid_drift;
        local.best_rej_persist = m.vertical_persistence;
        local.best_rej_bottom_dz = bottom_dz;
      }
    };

    auto push_rejected_pts = [&](const std::vector<const SliceCluster2D *> & clusts) {
      if (!dbg_local.rejected_points) {
        return;
      }
      for (const auto * c : clusts) {
        for (std::size_t idx : c->point_indices) {
          dbg_local.rejected_points->push_back(cloud.points[idx]);
        }
      }
    };

    auto stem_cxcy = [](const std::vector<const SliceCluster2D *> & clusts) {
      float cx = 0.0f;
      float cy = 0.0f;
      for (const auto * c : clusts) {
        cx += c->cx;
        cy += c->cy;
      }
      if (!clusts.empty()) {
        const float inv = 1.0f / static_cast<float>(clusts.size());
        cx *= inv;
        cy *= inv;
      }
      return std::pair<float, float>{cx, cy};
    };

    std::vector<SliceTrunkDetection> detections;
    for (const auto & stem : finished) {
      std::vector<const SliceCluster2D *> clusters;
      clusters.reserve(stem.slice_indices.size());
      for (std::size_t k = 0; k < stem.slice_indices.size(); ++k) {
        const auto * c = find_cluster(slices, stem.slice_indices[k], stem.cluster_ids[k]);
        if (c != nullptr) {
          clusters.push_back(c);
        }
      }
      const int n_stem_slices = static_cast<int>(stem.slice_indices.size());
      const auto [scx, scy] = stem_cxcy(clusters);

      if (n_stem_slices < params.min_slices_for_trunk ||
        static_cast<int>(clusters.size()) < params.min_slices_for_trunk)
      {
        ++local.reject_sparse_few_slices;
        push_rejected_pts(clusters);
        RejectedStemSample rs;
        rs.threshold = static_cast<float>(params.min_slices_for_trunk);
        rs.measured = static_cast<float>(std::min(n_stem_slices, static_cast<int>(clusters.size())));
        local.record_reject("sparse_slices", n_stem_slices, scx, scy, rs);
        continue;
      }

      const auto metrics = score_stem(clusters, n_slices);
      const float bottom_dz = bottom_slice_dz(clusters, grid);
      note_near_miss(n_stem_slices, metrics, bottom_dz);

      if (metrics.continuity_score < params.min_continuity_score) {
        ++local.reject_cont_score;
        push_rejected_pts(clusters);
        RejectedStemSample rs;
        rs.continuity = metrics.continuity_score;
        rs.threshold = params.min_continuity_score;
        rs.measured = metrics.continuity_score;
        local.record_reject("cont_score", n_stem_slices, scx, scy, rs);
        continue;
      }
      if (metrics.vertical_persistence < params.min_vertical_persistence) {
        ++local.reject_cont_persistence;
        push_rejected_pts(clusters);
        RejectedStemSample rs;
        rs.persistence = metrics.vertical_persistence;
        rs.threshold = params.min_vertical_persistence;
        rs.measured = metrics.vertical_persistence;
        local.record_reject("cont_persist", n_stem_slices, scx, scy, rs);
        continue;
      }
      if (metrics.centroid_drift > params.max_centroid_drift_ratio) {
        ++local.reject_cont_drift;
        push_rejected_pts(clusters);
        RejectedStemSample rs;
        rs.drift = metrics.centroid_drift;
        rs.threshold = params.max_centroid_drift_ratio;
        rs.measured = metrics.centroid_drift;
        local.record_reject("cont_drift", n_stem_slices, scx, scy, rs);
        continue;
      }
      if (metrics.radius_variance > params.max_radius_cv) {
        ++local.reject_cont_radius_cv;
        push_rejected_pts(clusters);
        RejectedStemSample rs;
        rs.radius_cv = metrics.radius_variance;
        rs.threshold = params.max_radius_cv;
        rs.measured = metrics.radius_variance;
        local.record_reject("cont_radius_cv", n_stem_slices, scx, scy, rs);
        continue;
      }

      const auto gnd = ground_anchor_check(clusters, grid, cell_connected);
      if (!gnd.ok) {
        push_rejected_pts(clusters);
        RejectedStemSample rs;
        rs.bottom_dz_m = gnd.bottom_dz;
        switch (gnd.reason) {
          case 1:
            ++local.reject_ground_dz;
            rs.threshold = params.ground_anchor_max_dz_m;
            rs.measured = gnd.bottom_dz;
            local.record_reject("ground_dz", n_stem_slices, scx, scy, rs);
            break;
          case 2:
            ++local.reject_ground_cell;
            local.record_reject("ground_cell", n_stem_slices, scx, scy, rs);
            break;
          case 3:
            ++local.reject_ground_z_nan;
            local.record_reject("ground_z_nan", n_stem_slices, scx, scy, rs);
            break;
          default:
            local.record_reject("ground_other", n_stem_slices, scx, scy, rs);
            break;
        }
        continue;
      }

      std::vector<std::size_t> indices;
      indices.reserve(clusters.size() * 8);
      for (const auto * c : clusters) {
        indices.insert(indices.end(), c->point_indices.begin(), c->point_indices.end());
      }
      if (indices.size() < static_cast<std::size_t>(params.min_points_per_cluster * 2)) {
        ++local.reject_sparse_few_points;
        push_rejected_pts(clusters);
        RejectedStemSample rs;
        rs.threshold = static_cast<float>(params.min_points_per_cluster * 2);
        rs.measured = static_cast<float>(indices.size());
        local.record_reject("sparse_points", n_stem_slices, scx, scy, rs);
        continue;
      }

      CylinderObservation cyl;
      const auto rej = fit_vertical_cylinder(
        cloud, indices, cyl,
        cylinder_min_height_m, cylinder_max_radius_m,
        cylinder_max_rmse_m, cylinder_min_inlier_ratio, cylinder_inlier_dist_m,
        cylinder_max_slice_height_m);
      if (rej != CylinderReject::Accepted) {
        push_rejected_pts(clusters);
        RejectedStemSample rs;
        rs.n_slices = n_stem_slices;
        switch (rej) {
          case CylinderReject::TooShort:
            ++local.reject_cylinder_short;
            rs.threshold = static_cast<float>(cylinder_min_height_m);
            rs.measured = cyl.height;
            local.record_reject("cyl_short", n_stem_slices, scx, scy, rs);
            break;
          case CylinderReject::TooWide:
            ++local.reject_cylinder_wide;
            rs.threshold = static_cast<float>(cylinder_max_radius_m);
            rs.measured = cyl.radius;
            local.record_reject("cyl_wide", n_stem_slices, scx, scy, rs);
            break;
          case CylinderReject::HighRmse:
            ++local.reject_cylinder_rmse;
            rs.threshold = static_cast<float>(cylinder_max_rmse_m);
            rs.measured = cyl.rmse;
            local.record_reject("cyl_rmse", n_stem_slices, scx, scy, rs);
            break;
          case CylinderReject::LowInliers:
            ++local.reject_cylinder_inliers;
            local.record_reject("cyl_inliers", n_stem_slices, scx, scy, rs);
            break;
          default:
            break;
        }
        continue;
      }

      if (dbg_local.accepted_points) {
        for (std::size_t idx : indices) {
          dbg_local.accepted_points->push_back(cloud.points[idx]);
        }
      }

      SliceTrunkDetection det;
      det.point_indices = std::move(indices);
      det.cylinder = cyl;
      det.metrics = metrics;
      det.ground_anchored = true;
      detections.push_back(std::move(det));
      ++local.n_accepted_cylinders;
    }

    if (detections.size() > params.max_stems_per_frame) {
      std::sort(
        detections.begin(), detections.end(),
        [](const SliceTrunkDetection & a, const SliceTrunkDetection & b) {
          return a.metrics.continuity_score > b.metrics.continuity_score;
        });
      detections.resize(params.max_stems_per_frame);
    }

    if (funnel) {
      *funnel = local;
    }
    if (debug_clouds) {
      *debug_clouds = dbg_local;
    }
    return detections;
  }

private:
  mutable std::vector<std::vector<std::size_t>> slice_buckets_;

  const SliceCluster2D * find_cluster(
    const std::vector<std::vector<SliceCluster2D>> & slices,
    int slice_idx, int cluster_id) const
  {
    if (slice_idx < 0 || static_cast<std::size_t>(slice_idx) >= slices.size()) {
      return nullptr;
    }
    for (const auto & c : slices[static_cast<std::size_t>(slice_idx)]) {
      if (c.id == cluster_id) {
        return &c;
      }
    }
    return nullptr;
  }

  std::vector<SliceCluster2D> cluster_slice_xy(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const std::vector<std::size_t> & indices,
    int slice_idx,
    float z_lo,
    int & next_cluster_id,
    TrunkPipelineFunnel * funnel) const
  {
    if (indices.empty()) {
      return {};
    }

    const float inv_cell = 1.0f / params.cluster_cell_m;
    auto grid_key = [](int ix, int iy) -> std::int64_t {
      return (static_cast<std::int64_t>(ix) << 32) |
        (static_cast<std::int64_t>(iy) & 0xffffffffLL);
    };
    std::unordered_map<std::int64_t, std::vector<std::size_t>> cells;
    cells.reserve(indices.size());

    for (std::size_t idx : indices) {
      const auto & p = cloud.points[idx];
      const int ix = static_cast<int>(std::floor(p.x * inv_cell));
      const int iy = static_cast<int>(std::floor(p.y * inv_cell));
      cells[grid_key(ix, iy)].push_back(idx);
    }
    if (funnel) {
      funnel->n_grid_cells_occupied += cells.size();
    }

    std::vector<std::vector<std::size_t>> components;
    std::unordered_map<std::int64_t, bool> visited;
    for (const auto & [key, pts] : cells) {
      if (visited[key]) {
        continue;
      }
      std::vector<std::size_t> comp_pts = pts;
      visited[key] = true;

      std::vector<std::int64_t> queue_keys;
      queue_keys.push_back(key);
      while (!queue_keys.empty()) {
        const std::int64_t cur_key = queue_keys.back();
        queue_keys.pop_back();
        const int cix = static_cast<int>(cur_key >> 32);
        const int ciy = static_cast<int>(cur_key & 0xffffffffLL);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const std::int64_t nkey = grid_key(cix + dx, ciy + dy);
            if (visited.count(nkey)) {
              continue;
            }
            const auto it = cells.find(nkey);
            if (it == cells.end()) {
              continue;
            }
            visited[nkey] = true;
            comp_pts.insert(comp_pts.end(), it->second.begin(), it->second.end());
            queue_keys.push_back(nkey);
          }
        }
      }
      if (comp_pts.size() < static_cast<std::size_t>(params.min_points_per_cluster)) {
        if (funnel) {
          ++funnel->reject_cluster_too_small;
        }
        continue;
      }
      components.push_back(std::move(comp_pts));
    }

    std::vector<SliceCluster2D> out;
    for (auto & comp_pts : components) {
      if (comp_pts.size() < static_cast<std::size_t>(params.min_points_per_cluster)) {
        continue;
      }
      SliceCluster2D c;
      c.id = next_cluster_id++;
      c.slice_idx = slice_idx;
      c.n_points = comp_pts.size();
      c.point_indices = std::move(comp_pts);
      compute_cluster_metrics(cloud, c);
      c.z_mid = z_lo + (static_cast<float>(slice_idx) + 0.5f) * params.slice_height_m;
      out.push_back(std::move(c));
    }
    return out;
  }

  static void compute_cluster_metrics(
    const pcl::PointCloud<pcl::PointXYZ> & cloud, SliceCluster2D & c)
  {
    double sx = 0.0;
    double sy = 0.0;
    for (std::size_t idx : c.point_indices) {
      sx += cloud.points[idx].x;
      sy += cloud.points[idx].y;
    }
    const double inv = 1.0 / static_cast<double>(c.point_indices.size());
    c.cx = static_cast<float>(sx * inv);
    c.cy = static_cast<float>(sy * inv);

    std::vector<float> radii;
    radii.reserve(c.point_indices.size());
    double sxx = 0.0;
    double syy = 0.0;
    double sxy = 0.0;
    for (std::size_t idx : c.point_indices) {
      const auto & p = cloud.points[idx];
      const float dx = p.x - c.cx;
      const float dy = p.y - c.cy;
      radii.push_back(std::hypot(dx, dy));
      sxx += dx * dx;
      syy += dy * dy;
      sxy += dx * dy;
    }
    std::sort(radii.begin(), radii.end());
    c.radius = radii[radii.size() / 2];

    sxx *= inv;
    syy *= inv;
    sxy *= inv;
    const double trace = sxx + syy;
    const double det = sxx * syy - sxy * sxy;
    const double disc = std::max(0.0, 0.25 * trace * trace - det);
    const double root = std::sqrt(disc);
    const double lam_max = 0.5 * trace + root;
    const double lam_min = std::max(1e-6, 0.5 * trace - root);
    c.elongation = static_cast<float>(std::sqrt(lam_max / lam_min));
    c.circularity = static_cast<float>(lam_min / std::max(lam_max, 1e-6));

    const float area = static_cast<float>(M_PI) * c.radius * c.radius;
    c.density = area > 1e-4f ? static_cast<float>(c.n_points) / area : static_cast<float>(c.n_points);
  }

  StemContinuityMetrics score_stem(
    const std::vector<const SliceCluster2D *> & clusters,
    int total_slices) const
  {
    StemContinuityMetrics m;
    m.n_slices = static_cast<int>(clusters.size());
    if (clusters.empty()) {
      return m;
    }

    int z_min_slice = clusters.front()->slice_idx;
    int z_max_slice = clusters.front()->slice_idx;
    float r_sum = 0.0f;
    float r2_sum = 0.0f;
    float mean_cx = 0.0f;
    float mean_cy = 0.0f;
    float max_drift = 0.0f;

    for (const auto * c : clusters) {
      z_min_slice = std::min(z_min_slice, c->slice_idx);
      z_max_slice = std::max(z_max_slice, c->slice_idx);
      r_sum += c->radius;
      r2_sum += c->radius * c->radius;
      mean_cx += c->cx;
      mean_cy += c->cy;
    }
    mean_cx /= static_cast<float>(clusters.size());
    mean_cy /= static_cast<float>(clusters.size());
    float sum_d2 = 0.0f;
    for (const auto * c : clusters) {
      const float dx = c->cx - mean_cx;
      const float dy = c->cy - mean_cy;
      sum_d2 += dx * dx + dy * dy;
      max_drift = std::max(max_drift, std::hypot(dx, dy));
    }

    m.n_slices_spanned = z_max_slice - z_min_slice + 1;
    const float r_mean = r_sum / static_cast<float>(clusters.size());
    const float r_var = std::max(0.0f, r2_sum / static_cast<float>(clusters.size()) - r_mean * r_mean);
    m.radius_variance = r_mean > 1e-3f ? std::sqrt(r_var) / r_mean : 1.0f;
    // RMS XY spread (robust to one bad slice); max kept for debug via drift if needed.
    const float rms_xy = std::sqrt(sum_d2 / static_cast<float>(clusters.size()));
    m.centroid_drift = rms_xy / std::max(0.20f, r_mean);
    (void)max_drift;
    m.slice_occupancy_ratio =
      static_cast<float>(m.n_slices) / static_cast<float>(std::max(1, m.n_slices_spanned));
    // Occupancy along spanned height — NOT n_slices/total_slices (penalizes tall band).
    m.vertical_persistence = m.slice_occupancy_ratio;

    m.continuity_score =
      0.35f * m.slice_occupancy_ratio +
      0.30f * m.vertical_persistence +
      0.20f * std::max(0.0f, 1.0f - m.centroid_drift / params.max_centroid_drift_ratio) +
      0.15f * std::max(0.0f, 1.0f - m.radius_variance / params.max_radius_cv);
    (void)total_slices;
    return m;
  }

  struct GroundAnchorResult
  {
    bool ok{false};
    /** 0=ok 1=dz 2=cell 3=z_nan 4=out_of_grid */
    int reason{0};
    float bottom_dz{0.0f};
  };

  static const SliceCluster2D * lowest_slice_cluster(
    const std::vector<const SliceCluster2D *> & clusters)
  {
    const SliceCluster2D * bottom = clusters.front();
    for (const auto * c : clusters) {
      if (c->slice_idx < bottom->slice_idx) {
        bottom = c;
      }
    }
    return bottom;
  }

  float bottom_slice_dz(
    const std::vector<const SliceCluster2D *> & clusters,
    const TerrainGrid2D & grid) const
  {
    const auto * bottom = lowest_slice_cluster(clusters);
    const float zg = grid.ground_reference_at(bottom->cx, bottom->cy);
    if (std::isnan(zg)) {
      return -1.0f;
    }
    return bottom->z_mid - zg;
  }

  GroundAnchorResult ground_anchor_check(
    const std::vector<const SliceCluster2D *> & clusters,
    const TerrainGrid2D & grid,
    const std::vector<uint8_t> & cell_connected) const
  {
    GroundAnchorResult out;
    const auto * bottom = lowest_slice_cluster(clusters);

    std::size_t ix = 0;
    std::size_t iy = 0;
    if (!grid.index_for(bottom->cx, bottom->cy, ix, iy)) {
      out.reason = 4;
      return out;
    }

    const float zg = grid.ground_reference_at(bottom->cx, bottom->cy);
    if (std::isnan(zg)) {
      out.reason = 3;
      return out;
    }
    out.bottom_dz = bottom->z_mid - zg;
    // Base must sit in trunk nDSM band (not a floating canopy column).
    const float dz_max = std::max(params.ground_anchor_max_dz_m, params.ndsm_max_m);
    if (out.bottom_dz < params.ndsm_min_m || out.bottom_dz > dz_max) {
      out.reason = 1;
      return out;
    }

    bool near_connected = cell_connected.empty();
    if (!cell_connected.empty()) {
      for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
          const float x = bottom->cx + static_cast<float>(dx) * grid.resolution_m;
          const float y = bottom->cy + static_cast<float>(dy) * grid.resolution_m;
          std::size_t nx = 0;
          std::size_t ny = 0;
          if (!grid.index_for(x, y, nx, ny)) {
            continue;
          }
          const std::size_t nidx = ny * grid.width + nx;
          if (nidx < cell_connected.size() && cell_connected[nidx] != 0) {
            near_connected = true;
            break;
          }
        }
        if (near_connected) {
          break;
        }
      }
      if (!near_connected) {
        out.reason = 2;
        return out;
      }
    }
    out.ok = true;
    out.reason = 0;
    return out;
  }
};

}  // namespace forest_3d_perception

#endif  // FOREST_3D_PERCEPTION__TRUNK_SLICE_DETECTOR_HPP_
