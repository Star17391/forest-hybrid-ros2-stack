/**
 * @file stem_point_filter.hpp
 * @brief Per-point linearity filter on non-ground points (experimental, Sprint 3.5).
 *
 * WHY THIS EXISTS
 * ---------------
 * The stem-band clusterer separates trunk from canopy with a blunt height
 * band-pass [band_min, band_max]. Nothing in nature respects a fixed height
 * window: shrubs fill the whole band with foliage, leaning trees drift out of
 * it, and the band keeps canopy points that glue neighbours together. This is
 * the main cause of TRUNK<->SHRUB confusion downstream.
 *
 * This filter attacks the problem at the SOURCE (pre-processing), before
 * clustering: for every non-ground point it runs a LOCAL PCA over a small
 * neighbourhood and keeps only points whose neighbourhood is LINEAR (a thin
 * elongated structure = trunk-like), dropping scattered/volumetric foliage
 * (shrubs, canopy).
 *
 * KEY DESIGN CHOICE — linearity, NOT pure verticality:
 *   We deliberately gate on LINEARITY (is this a thin line?) with only a SOFT
 *   verticality floor, instead of requiring near-vertical. This keeps THIN,
 *   LEANING (diagonal) trunks — which are still valuable landmarks — that a
 *   hard verticality cut would throw away. The soft floor only rejects clearly
 *   horizontal linear clutter (fallen logs lying flat, ground scan lines).
 *
 * RECALL-SAFE on sparse trunks: points with too few neighbours to trust a PCA
 * are KEPT, not dropped — a thin/distant trunk must survive to clustering.
 *
 * FUTURE WORK — Option 2 (planned next): replace/augment this per-point filter
 * with VERTICAL-CONTINUITY REGION GROWING seeded from the ground. That adds a
 * ground anchor (rejects floating canopy branches this filter can still keep)
 * and follows leaning trunks structurally. It is comparably real-time (same
 * KdTree-bound cost on the voxelised cloud) and is what robotic-forestry
 * stacks (DigiForest, 3DFin-online) run live. See discussion in
 * docs/perception/references/FORESTRY_CLUSTERING_LITERATURE.md.
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__STEM_POINT_FILTER_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__STEM_POINT_FILTER_HPP_

#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace forest_3d_perception::experimental
{

struct StemPointFilterParams
{
  bool enabled{true};
  float neighbor_radius_m{0.25f};  // local PCA neighbourhood radius
  int min_neighbors{5};            // below this we can't judge -> keep (recall-safe)
  float linearity_min{0.60f};      // (l2 - l1) / l2 keep threshold (1 = perfect line)
  float verticality_floor{0.30f};  // soft floor on |axis.z|; drops horizontal clutter
};

struct StemPointFilterResult
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  // surviving (trunk-like) points
  std::size_t n_in{0};
  std::size_t n_kept{0};
  std::size_t n_low_neighbors{0};  // kept by the recall-safe rule
};

class StemPointFilter
{
public:
  StemPointFilterParams params;

  /**
   * Per-point trunk-likeness mask, aligned 1:1 with `cloud`.
   *   true  = trunk-like (linear, not clearly horizontal) OR too sparse to judge
   *           (recall-safe: a thin/distant trunk must survive).
   *   false = scattered/volumetric (shrub/canopy foliage) or flat linear clutter.
   * When the filter is disabled, every point is marked true (pass-through).
   *
   * NOTE (subtraction architecture): the caller runs this over the BAND points
   * and SPLITS them — mask=true -> trunk set, mask=false -> the rest (shrubs +
   * rocks). The two sets are disjoint, so trunks are removed from the shrub/rock
   * stream and can't glue to it, while rocks (non-linear) stay in the rest and
   * survive to be clustered & classified as ROCK.
   */
  std::vector<char> linear_mask(const pcl::PointCloud<pcl::PointXYZ> & cloud) const
  {
    std::vector<char> mask(cloud.size(), 1);
    if (!params.enabled || cloud.size() < static_cast<std::size_t>(params.min_neighbors)) {
      return mask;  // disabled or too small: everything passes as candidate
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(in_ptr);

    std::vector<int> idx;
    std::vector<float> dist2;
    const float r = params.neighbor_radius_m;

    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const auto & p = cloud.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        mask[i] = 1;  // can't judge -> keep (recall-safe)
        continue;
      }
      idx.clear();
      dist2.clear();
      const int n = tree.radiusSearch(p, r, idx, dist2);

      // Recall-safe: not enough support to estimate shape -> keep as trunk-like.
      if (n < params.min_neighbors) {
        mask[i] = 1;
        continue;
      }

      // Local covariance over the neighbourhood.
      double cx = 0.0;
      double cy = 0.0;
      double cz = 0.0;
      for (int j : idx) {
        cx += in_ptr->points[j].x;
        cy += in_ptr->points[j].y;
        cz += in_ptr->points[j].z;
      }
      const double inv = 1.0 / static_cast<double>(idx.size());
      cx *= inv;
      cy *= inv;
      cz *= inv;

      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (int j : idx) {
        const double dx = in_ptr->points[j].x - cx;
        const double dy = in_ptr->points[j].y - cy;
        const double dz = in_ptr->points[j].z - cz;
        cov(0, 0) += dx * dx;
        cov(0, 1) += dx * dy;
        cov(0, 2) += dx * dz;
        cov(1, 1) += dy * dy;
        cov(1, 2) += dy * dz;
        cov(2, 2) += dz * dz;
      }
      cov(1, 0) = cov(0, 1);
      cov(2, 0) = cov(0, 2);
      cov(2, 1) = cov(1, 2);
      cov *= inv;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
      if (es.info() != Eigen::Success) {
        mask[i] = 1;  // can't decide -> keep
        continue;
      }
      const Eigen::Vector3d evals = es.eigenvalues();   // ascending
      const double l2 = std::max(evals(2), 1e-9);       // largest
      const double l1 = std::max(evals(1), 0.0);        // middle
      const float linearity = static_cast<float>((l2 - l1) / l2);
      const Eigen::Vector3d axis = es.eigenvectors().col(2).normalized();
      const float verticality = static_cast<float>(std::abs(axis.z()));

      // Trunk-like = thin LINE that is not clearly horizontal. The soft
      // verticality floor lets leaning trunks through while dropping flat
      // linear clutter (fallen logs, ground scan lines).
      mask[i] = (linearity >= params.linearity_min && verticality >= params.verticality_floor)
        ? 1 : 0;
    }
    return mask;
  }

  /** Convenience: return only the trunk-like points (mask applied). */
  StemPointFilterResult filter(const pcl::PointCloud<pcl::PointXYZ> & non_ground) const
  {
    StemPointFilterResult out;
    out.n_in = non_ground.size();
    out.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    out.cloud->reserve(non_ground.size());
    const auto mask = linear_mask(non_ground);
    for (std::size_t i = 0; i < non_ground.size(); ++i) {
      if (mask[i]) {
        out.cloud->push_back(non_ground.points[i]);
      }
    }
    out.cloud->width = static_cast<std::uint32_t>(out.cloud->size());
    out.cloud->height = 1;
    out.cloud->is_dense = true;
    out.n_kept = out.cloud->size();
    return out;
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__STEM_POINT_FILTER_HPP_
