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
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
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
  // TRUNK = vertical, via PCA (robust to sparsity: a distant tree is a thin vertical
  // line) OR a dense stem core. The crown is a BONUS, never a requirement.
  float trunk_min_verticality{0.55f};    // |principal axis · z| ≥ this → vertically dominant
  float trunk_min_linearity{0.40f};      // (l0-l1)/l0 ≥ this → thin line (1D), not a surface
  int canopy_min_count{12};              // crown present if ≥ this many contiguous points above
  // ROCK vs SHRUB by SURFACE SMOOTHNESS (not just height): a rock is a well-defined
  // smooth shell (low surface_variation / scatter); a shrub is irregular volumetric
  // scatter (points all over). Rocks can be large, so height is a loose cap only.
  float rock_max_height_m{1.50f};        // rochas/pedregulhos podem ser grandes
  float rock_max_aspect{1.20f};          // height_span / horizontal_size (diagnostic, legacy)
  float rock_max_surface_variation{0.10f}; // ≤ isto = casca smooth → ROCK
  float rock_max_scatter{0.35f};         // ≤ isto = não-volumétrico → ROCK
  float rock_max_local_roughness{0.06f}; // ≤ isto = localmente smooth → ROCK; acima = irregular
  float shrub_min_scatter{0.25f};        // ≥ isto = volumétrico/irregular → SHRUB
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
  // Surface smoothness from PCA eigenvalues (l0≥l1≥l2):
  float scatter{0.0f};           // l2/l0 — ~1 = volumetric (shrub), ~0 = surface (rock)
  float surface_variation{0.0f}; // l2/(l0+l1+l2) — curvature; low = smooth shell (rock)
  float local_roughness{0.0f};   // mean LOCAL surface variation — high = points "all over"
                                 // (irregular vegetation), low = locally smooth (rock)
  float canopy_above{0.0f};      // non-ground points found in the XY column ABOVE this
                                 // cluster's top — a tree has canopy above, a rock does not
                                 // (set by the node, which holds the full non-ground cloud)
  float trunk_core_height{0.0f}; // vertical span of the detected stem core (0 = none)
  float trunk_ref_radius{0.0f};  // reference stem radius (narrowest part)
  int n_slices{0};
  int n_points{0};
};

/** Per-class score indices — must match TreeLandmark.msg class_scores contract. */
constexpr std::size_t kScoreTrunk = 0;
constexpr std::size_t kScoreRock = 1;
constexpr std::size_t kScoreObstacle = 2;
constexpr std::size_t kNumClassScores = 3;

inline float sigmoid(float x)
{
  if (x >= 0.0f) {
    const float z = std::exp(-x);
    return 1.0f / (1.0f + z);
  }
  const float z = std::exp(x);
  return z / (1.0f + z);
}

/** Argmax index into class_scores [tronco, rocha, obstáculo]. */
inline int argmax_class_index(const std::array<float, kNumClassScores> & scores)
{
  int best = 0;
  for (std::size_t i = 1; i < kNumClassScores; ++i) {
    if (scores[i] > scores[static_cast<std::size_t>(best)]) {
      best = static_cast<int>(i);
    }
  }
  return best;
}

inline ClusterClass cluster_class_from_scores(const std::array<float, kNumClassScores> & scores)
{
  switch (argmax_class_index(scores)) {
    case static_cast<int>(kScoreTrunk): return ClusterClass::Trunk;
    case static_cast<int>(kScoreRock): return ClusterClass::Rock;
    case static_cast<int>(kScoreObstacle): return ClusterClass::Obstacle;
    default: return ClusterClass::Unknown;
  }
}

struct ScoredCluster
{
  ClusterFeatures feat;
  std::array<float, kNumClassScores> class_scores{{0.0f, 0.0f, 0.0f}};
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
        const double l0 = std::max(evals(2), 1e-9);  // largest
        const double l1 = std::max(evals(1), 0.0);   // middle
        const double l2 = std::max(evals(0), 0.0);   // smallest
        f.linearity = static_cast<float>((l0 - l1) / l0);
        f.verticality = static_cast<float>(std::abs(es.eigenvectors().col(2).normalized().z()));
        // Smoothness: a rock is a thin shell (l2 tiny vs l0) → low scatter/variation;
        // a shrub fills the volume (l2 ~ l0) → high scatter/variation.
        f.scatter = static_cast<float>(l2 / l0);
        f.surface_variation = static_cast<float>(l2 / (l0 + l1 + l2));
      }
    }
  }

  /**
   * Mean LOCAL surface variation over the cluster. For each sampled point, take its
   * neighbours within a small voxel neighbourhood and PCA them: a rock is locally a
   * smooth surface (small smallest-eigenvalue → low local variation) EVEN if globally
   * curved; irregular vegetation has points "all over" so every local patch is a
   * blob (high local variation). This catches what GLOBAL PCA misses: scattered
   * points that happen to lie in a global plane (read as smooth) but are locally
   * chaotic. Grid hash keeps it O(N).
   */
  static float local_roughness(const PointCluster & c, float cell = 0.20f)
  {
    const auto & pts = c.cloud->points;
    if (pts.size() < 12) {
      return 0.0f;
    }
    using Key = std::array<int, 3>;
    std::map<Key, std::vector<int>> grid;
    auto cell_of = [cell](const pcl::PointXYZ & p) -> Key {
      return {static_cast<int>(std::floor(p.x / cell)),
              static_cast<int>(std::floor(p.y / cell)),
              static_cast<int>(std::floor(p.z / cell))};
    };
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
      grid[cell_of(pts[i])].push_back(i);
    }
    const int step = std::max<int>(1, static_cast<int>(pts.size()) / 200);  // cap work
    double acc = 0.0;
    int cnt = 0;
    for (int i = 0; i < static_cast<int>(pts.size()); i += step) {
      const Key k0 = cell_of(pts[i]);
      std::vector<int> nb;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            auto it = grid.find({k0[0] + dx, k0[1] + dy, k0[2] + dz});
            if (it != grid.end()) {
              nb.insert(nb.end(), it->second.begin(), it->second.end());
            }
          }
        }
      }
      if (nb.size() < 6) {
        continue;
      }
      double mx = 0, my = 0, mz = 0;
      for (int j : nb) { mx += pts[j].x; my += pts[j].y; mz += pts[j].z; }
      const double in = 1.0 / static_cast<double>(nb.size());
      mx *= in; my *= in; mz *= in;
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (int j : nb) {
        const double dx = pts[j].x - mx, dy = pts[j].y - my, dz = pts[j].z - mz;
        cov(0, 0) += dx * dx; cov(0, 1) += dx * dy; cov(0, 2) += dx * dz;
        cov(1, 1) += dy * dy; cov(1, 2) += dy * dz; cov(2, 2) += dz * dz;
      }
      cov(1, 0) = cov(0, 1); cov(2, 0) = cov(0, 2); cov(2, 1) = cov(1, 2);
      cov *= in;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
      if (es.info() != Eigen::Success) {
        continue;
      }
      const Eigen::Vector3d ev = es.eigenvalues();  // ascending: ev(0)≤ev(1)≤ev(2)
      const double sum = ev(0) + ev(1) + ev(2);
      if (sum > 1e-12) {
        acc += ev(0) / sum;  // local surface variation
        ++cnt;
      }
    }
    return cnt > 0 ? static_cast<float>(acc / cnt) : 0.0f;
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

  /**
   * Pure soft scorer: features → normalized [P(tronco), P(rocha), P(obstáculo)].
   * Reusable by the SLAM layer on accumulated multi-view clouds (Fase 3).
   */
  static std::array<float, kNumClassScores> score_class_probs(
    const ClusterFeatures & f, const ClassifierParams & p, float canopy)
  {
    constexpr float s_v = 0.08f;
    constexpr float s_l = 0.08f;
    constexpr float s_c = 0.15f;
    constexpr float s_s = 0.02f;
    constexpr float s_r = 0.015f;
    constexpr float s_h = 0.20f;

    const float ev_vert = sigmoid((f.verticality - p.trunk_min_verticality) / s_v);
    const float ev_line = sigmoid((f.linearity - p.trunk_min_linearity) / s_l);
    const float ev_core = sigmoid(
      (f.trunk_core_height - p.trunk_core_min_height_m) / s_c);
    const float ev_smooth = sigmoid(
      (p.rock_max_surface_variation - f.surface_variation) / s_s);
    const float ev_rough = sigmoid(
      (p.rock_max_local_roughness - f.local_roughness) / s_r);
    const float ev_crown = canopy >= static_cast<float>(p.canopy_min_count) ? 1.0f : 0.0f;
    const float ev_vert_line = ev_vert * ev_line;

    const float s_tronco = std::max(
      1e-6f,
      ev_vert * std::max(ev_line, ev_core) * (1.0f - ev_smooth) * (0.5f + 0.5f * ev_crown));
    const float s_rocha = std::max(
      1e-6f,
      ev_smooth * ev_rough * (1.0f - ev_crown) * (1.0f - ev_vert_line));
    const float s_obstaculo = std::max(
      1e-6f,
      (1.0f - ev_smooth) * (1.0f - ev_vert) *
      sigmoid((f.height_span - p.shrub_max_height_m) / s_h));

    const float sum = s_tronco + s_rocha + s_obstaculo;
    return {s_tronco / sum, s_rocha / sum, s_obstaculo / sum};
  }

  /** Extract features + soft class scores for one cluster. */
  ScoredCluster score_cluster(const PointCluster & c, float canopy = 0.0f) const
  {
    ScoredCluster out;
    if (!c.cloud || c.cloud->empty()) {
      return out;
    }
    basic_features(c, out.feat);
    out.feat.canopy_above = canopy;

    if (out.feat.n_points < params.min_points) {
      return out;
    }

    const auto slices = extract_slices(c);
    out.feat.n_slices = static_cast<int>(slices.size());
    out.feat.trunk_core_height = trunk_core_span(slices, out.feat.trunk_ref_radius);
    out.feat.local_roughness = local_roughness(c);
    out.class_scores = score_class_probs(out.feat, params, canopy);
    return out;
  }

  /** Score every cluster; canopies[i] is crown evidence above cluster i (0 if absent). */
  std::vector<ScoredCluster> score_all(
    const std::vector<PointCluster> & clusters,
    const std::vector<float> & canopies = {}) const
  {
    std::vector<ScoredCluster> out;
    out.reserve(clusters.size());
    for (std::size_t i = 0; i < clusters.size(); ++i) {
      const float cy = (i < canopies.size()) ? canopies[i] : 0.0f;
      out.push_back(score_cluster(clusters[i], cy));
    }
    return out;
  }

  /** Loose gate: cluster is worth emitting as a structural landmark candidate. */
  bool is_structural_candidate(const ScoredCluster & s, int min_emit_points = 10) const
  {
    const auto & f = s.feat;
    if (f.n_points < min_emit_points) {
      return false;
    }
    if (f.height_span < 0.12f || f.horizontal_size < 0.08f) {
      return false;
    }
    // Obvious shrub: volumetric, low, not vertical.
    const bool obvious_shrub =
      f.scatter >= params.shrub_min_scatter &&
      f.verticality < 0.35f &&
      f.height_span < params.shrub_max_height_m;
    if (obvious_shrub) {
      return false;
    }
    const float max_score = std::max(
      {s.class_scores[kScoreTrunk], s.class_scores[kScoreRock], s.class_scores[kScoreObstacle]});
    return max_score >= 0.08f;
  }

  /**
   * P-C API — score a multi-view accumulated cloud (SLAM landmark buffer).
   * Canopy is usually 0: inliers agregados raramente incluem copa acima.
   */
  ScoredCluster score_accumulated_cloud(const PointCluster & accumulated, float canopy = 0.0f) const
  {
    return score_cluster(accumulated, canopy);
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__CLUSTER_CLASSIFIER_HPP_
