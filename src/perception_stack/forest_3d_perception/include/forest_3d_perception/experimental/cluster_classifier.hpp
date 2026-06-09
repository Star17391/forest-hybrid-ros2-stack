/**
 * @file cluster_classifier.hpp
 * @brief Slice-based geometric classification of clusters (experimental, Sprint 3 / Option B).
 *
 * Replaces the earlier "tall + thin + vertical, else SHRUB" rule, which had two
 * faults the user identified:
 *   1. It capped trunk WIDTH (radius <= 0.35), so WIDE trees failed the trunk
 *      test and fell through to SHRUB. A trunk should be defined by its SHAPE
 *      (a vertical column), not an absolute width.
 *   2. SHRUB was a catch-all ("everything else"), so a tree contaminated by its
 *      lower canopy — wide and non-vertical — was dumped into SHRUB. In a forest
 *      the leftover is not all shrubs.
 *
 * SLICE-BASED TRUNK CORE
 * ----------------------
 * Cut the cluster into horizontal slices of `slice_height_m`. For each slice
 * compute its centre and radius (mean horizontal distance to the centre). A
 * TRUNK is a contiguous vertical RUN of slices whose radius stays compact
 * relative to the narrowest part of the stem (so width is RELATIVE, not capped)
 * and whose centre does not jump (allows leaning trunks). The canopy is exactly
 * where the radius balloons, so it ends the run and is rejected. If that run
 * spans >= `trunk_core_min_height_m`, the cluster is a TRUNK — at any width.
 *
 * TAXONOMY (positive definitions, OBSTACLE is the only catch-all):
 *   TRUNK     has a vertical stem core (slice run).
 *   ROCK      low and flat, no core.
 *   SHRUB     low-to-medium, no core, bushy vegetation.
 *   OBSTACLE  taller than a shrub but with no stem core — a real obstacle, not
 *             a tree and not a bush.
 *   UNKNOWN   too few points to decide.
 *
 * References (local): docs/perception/references/FORESTRY_CLUSTERING_LITERATURE.md
 *   §5 (slice / stem-core stem detection).
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__CLUSTER_CLASSIFIER_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__CLUSTER_CLASSIFIER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/experimental/euclidean_clustering.hpp"

namespace forest_3d_perception::experimental
{

enum class ClusterClass
{
  Unknown = 0,
  Trunk = 1,
  Rock = 2,
  Shrub = 3,
  Obstacle = 4,
};

inline const char * cluster_class_string(ClusterClass c)
{
  switch (c) {
    case ClusterClass::Trunk: return "TRUNK";
    case ClusterClass::Rock: return "ROCK";
    case ClusterClass::Shrub: return "SHRUB";
    case ClusterClass::Obstacle: return "OBSTACLE";
    default: return "UNKNOWN";
  }
}

struct ClassifierParams
{
  // Slice-based trunk-core detection (defines a trunk by SHAPE, not width).
  float slice_height_m{0.20f};
  int slice_min_pts{2};                  // slices thinner than this are ignored
  float trunk_radius_grow_factor{2.2f};  // a slice is "stem" if radius <= factor*ref_radius + margin
  float trunk_radius_abs_margin_m{0.10f};// absolute slack so thin trunks tolerate noise
  float trunk_center_jump_m{0.30f};      // max XY centre drift between stem slices (allows lean)
  float trunk_core_min_height_m{0.80f};  // min vertical span of the stem core to be a TRUNK
  // ROCK: low and flat.
  float rock_max_height_m{0.60f};
  float rock_max_aspect{1.20f};          // height_span / horizontal_size
  // SHRUB up to this height (no core); taller with no core -> OBSTACLE.
  float shrub_max_height_m{1.50f};
  int min_points{4};
};

struct ClusterFeatures
{
  float centroid_x{0.0f};
  float centroid_y{0.0f};
  float centroid_z{0.0f};
  float height_span{0.0f};       // z_max - z_min
  float horizontal_size{0.0f};   // larger XY bbox side
  float verticality{0.0f};       // |v0 . z| from PCA (diagnostic)
  float linearity{0.0f};         // (l0 - l1) / l0 from PCA (diagnostic)
  float trunk_core_height{0.0f}; // vertical span of the detected stem core (0 = none)
  float trunk_ref_radius{0.0f};  // reference stem radius (narrowest part)
  int n_slices{0};
  int n_points{0};
};

struct ClassifiedCluster
{
  ClusterClass cls{ClusterClass::Unknown};
  ClusterFeatures feat;
  float confidence{0.0f};
};

class ClusterClassifier
{
public:
  ClassifierParams params;

  struct Slice
  {
    float z{0.0f};
    float cx{0.0f};
    float cy{0.0f};
    float radius{0.0f};  // mean horizontal distance to slice centre
    int n{0};
  };

  /** Bounding box / PCA features (diagnostic + ROCK/aspect rule). */
  static void basic_features(const PointCluster & c, ClusterFeatures & f)
  {
    const auto & pts = c.cloud->points;
    f.n_points = static_cast<int>(pts.size());

    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    float xmin = pts[0].x;
    float xmax = pts[0].x;
    float ymin = pts[0].y;
    float ymax = pts[0].y;
    float zmin = pts[0].z;
    float zmax = pts[0].z;
    for (const auto & p : pts) {
      sx += p.x;
      sy += p.y;
      sz += p.z;
      xmin = std::min(xmin, p.x);
      xmax = std::max(xmax, p.x);
      ymin = std::min(ymin, p.y);
      ymax = std::max(ymax, p.y);
      zmin = std::min(zmin, p.z);
      zmax = std::max(zmax, p.z);
    }
    const double inv = 1.0 / static_cast<double>(pts.size());
    f.centroid_x = static_cast<float>(sx * inv);
    f.centroid_y = static_cast<float>(sy * inv);
    f.centroid_z = static_cast<float>(sz * inv);
    f.height_span = zmax - zmin;
    f.horizontal_size = std::max(xmax - xmin, ymax - ymin);

    if (pts.size() >= 3) {
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (const auto & p : pts) {
        const double dx = p.x - f.centroid_x;
        const double dy = p.y - f.centroid_y;
        const double dz = p.z - f.centroid_z;
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
      if (es.info() == Eigen::Success) {
        const Eigen::Vector3d evals = es.eigenvalues();
        const double l0 = std::max(evals(2), 1e-9);
        const double l1 = std::max(evals(1), 0.0);
        f.linearity = static_cast<float>((l0 - l1) / l0);
        f.verticality = static_cast<float>(std::abs(es.eigenvectors().col(2).normalized().z()));
      }
    }
  }

  /** Cut the cluster into horizontal slices (ascending Z), with centre+radius. */
  std::vector<Slice> extract_slices(const PointCluster & c) const
  {
    std::vector<Slice> slices;
    const auto & pts = c.cloud->points;
    if (pts.empty()) {
      return slices;
    }
    float zmin = pts[0].z;
    float zmax = pts[0].z;
    for (const auto & p : pts) {
      zmin = std::min(zmin, p.z);
      zmax = std::max(zmax, p.z);
    }
    const float h = std::max(params.slice_height_m, 0.02f);
    const int n_bins = std::max(1, static_cast<int>(std::ceil((zmax - zmin) / h)));

    std::vector<double> sx(n_bins, 0.0);
    std::vector<double> sy(n_bins, 0.0);
    std::vector<int> cnt(n_bins, 0);
    for (const auto & p : pts) {
      int b = static_cast<int>((p.z - zmin) / h);
      b = std::clamp(b, 0, n_bins - 1);
      sx[b] += p.x;
      sy[b] += p.y;
      ++cnt[b];
    }
    // Centres first.
    std::vector<float> cx(n_bins, 0.0f);
    std::vector<float> cy(n_bins, 0.0f);
    for (int b = 0; b < n_bins; ++b) {
      if (cnt[b] > 0) {
        cx[b] = static_cast<float>(sx[b] / cnt[b]);
        cy[b] = static_cast<float>(sy[b] / cnt[b]);
      }
    }
    // Mean radius per bin.
    std::vector<double> rad(n_bins, 0.0);
    for (const auto & p : pts) {
      int b = static_cast<int>((p.z - zmin) / h);
      b = std::clamp(b, 0, n_bins - 1);
      rad[b] += std::hypot(p.x - cx[b], p.y - cy[b]);
    }
    for (int b = 0; b < n_bins; ++b) {
      if (cnt[b] >= params.slice_min_pts) {
        Slice s;
        s.z = zmin + (static_cast<float>(b) + 0.5f) * h;
        s.cx = cx[b];
        s.cy = cy[b];
        s.radius = static_cast<float>(rad[b] / cnt[b]);
        s.n = cnt[b];
        slices.push_back(s);
      }
    }
    return slices;
  }

  /**
   * Longest contiguous vertical run of "stem-like" slices: radius compact
   * relative to the narrowest slice (width is relative, so wide trunks pass),
   * and centre continuous (so leaning trunks are kept but canopy jumps end it).
   * Returns the run's vertical span; sets ref_radius to the stem reference.
   */
  float trunk_core_span(const std::vector<Slice> & slices, float & ref_radius) const
  {
    ref_radius = 0.0f;
    if (slices.size() < 2) {
      return 0.0f;
    }
    // Reference radius = narrowest slice in the lower half (the cleanest stem).
    const std::size_t lower_n = std::max<std::size_t>(1, slices.size() / 2);
    float ref = slices[0].radius;
    for (std::size_t i = 0; i < lower_n; ++i) {
      ref = std::min(ref, slices[i].radius);
    }
    ref_radius = ref;
    const float thresh = params.trunk_radius_grow_factor * ref + params.trunk_radius_abs_margin_m;

    const float h = std::max(params.slice_height_m, 0.02f);
    float best_span = 0.0f;
    std::size_t run_start = 0;
    bool in_run = false;
    for (std::size_t i = 0; i < slices.size(); ++i) {
      bool stem_like = slices[i].radius <= thresh;
      if (stem_like && in_run) {
        // centre continuity vs previous slice in the run
        const float jump = std::hypot(
          slices[i].cx - slices[i - 1].cx, slices[i].cy - slices[i - 1].cy);
        if (jump > params.trunk_center_jump_m) {
          stem_like = false;  // discontinuity: end the run here, start fresh below
        }
      }
      if (stem_like) {
        if (!in_run) {
          in_run = true;
          run_start = i;
        }
        const float span = (slices[i].z - slices[run_start].z) + h;
        best_span = std::max(best_span, span);
      } else {
        in_run = false;
        // allow a new run to start at i if it is itself stem-like
        if (slices[i].radius <= thresh) {
          in_run = true;
          run_start = i;
          best_span = std::max(best_span, h);
        }
      }
    }
    return best_span;
  }

  ClassifiedCluster classify(const PointCluster & c) const
  {
    ClassifiedCluster out;
    if (!c.cloud || c.cloud->empty()) {
      return out;
    }
    basic_features(c, out.feat);
    auto & f = out.feat;

    if (f.n_points < params.min_points) {
      out.cls = ClusterClass::Unknown;
      return out;
    }

    const auto slices = extract_slices(c);
    f.n_slices = static_cast<int>(slices.size());
    f.trunk_core_height = trunk_core_span(slices, f.trunk_ref_radius);

    const float aspect = f.horizontal_size > 1e-3f
      ? f.height_span / f.horizontal_size
      : f.height_span;

    // 1. TRUNK: a vertical stem core tall enough — at any width.
    if (f.trunk_core_height >= params.trunk_core_min_height_m) {
      out.cls = ClusterClass::Trunk;
      out.confidence = std::min(1.0f, f.trunk_core_height / (2.0f * params.trunk_core_min_height_m));
      return out;
    }
    // 2. ROCK: low and flat.
    if (f.height_span <= params.rock_max_height_m && aspect <= params.rock_max_aspect) {
      out.cls = ClusterClass::Rock;
      out.confidence = 0.7f;
      return out;
    }
    // 3. SHRUB: low-to-medium bushy vegetation with no stem core.
    if (f.height_span <= params.shrub_max_height_m) {
      out.cls = ClusterClass::Shrub;
      out.confidence = 0.5f;
      return out;
    }
    // 4. OBSTACLE: taller than a shrub but no stem core — a real obstacle.
    out.cls = ClusterClass::Obstacle;
    out.confidence = 0.5f;
    return out;
  }

  std::vector<ClassifiedCluster> classify_all(
    const std::vector<PointCluster> & clusters) const
  {
    std::vector<ClassifiedCluster> out;
    out.reserve(clusters.size());
    for (const auto & c : clusters) {
      out.push_back(classify(c));
    }
    return out;
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__CLUSTER_CLASSIFIER_HPP_
