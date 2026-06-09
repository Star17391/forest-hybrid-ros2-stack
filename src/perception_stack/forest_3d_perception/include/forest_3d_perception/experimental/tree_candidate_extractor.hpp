/**
 * @file tree_candidate_extractor.hpp
 * @brief Sprint 2: tree/trunk candidates from non-ground Euclidean clusters.
 *
 * Heuristic geometry (height, XY extent, verticality) — no nDSM yet.
 * Active when pipeline_sprint >= 2.
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__TREE_CANDIDATE_EXTRACTOR_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__TREE_CANDIDATE_EXTRACTOR_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include "forest_3d_perception/experimental/euclidean_clustering.hpp"
#include "forest_3d_perception/experimental/tree_candidate_params.hpp"

namespace forest_3d_perception::experimental
{

struct TreeCandidate
{
  int cluster_id{0};
  std::vector<std::size_t> point_indices;
  float height_m{0.0f};
  float xy_extent_m{0.0f};
  float verticality{0.0f};
  float cx{0.0f};
  float cy{0.0f};
  float cz{0.0f};
  float score{0.0f};
};

struct TreeCandidateResult
{
  std::vector<TreeCandidate> tree_candidates;
  std::vector<int> non_tree_cluster_ids;
  std::size_t n_rejected_height{0};
  std::size_t n_rejected_extent{0};
  std::size_t n_rejected_verticality{0};
  std::size_t n_rejected_points{0};
  bool enabled{false};
};

class TreeCandidateExtractor
{
public:
  TreeCandidateParams params;
  bool enabled{false};

  TreeCandidateResult extract(const std::vector<PointCluster> & clusters) const
  {
    TreeCandidateResult out;
    out.enabled = enabled;
    if (!enabled || clusters.empty()) {
      return out;
    }

    std::vector<TreeCandidate> scored;
    scored.reserve(clusters.size());

    for (const auto & cluster : clusters) {
      if (!cluster.cloud || cluster.cloud->size() < static_cast<std::size_t>(params.min_points)) {
        ++out.n_rejected_points;
        out.non_tree_cluster_ids.push_back(cluster.id);
        continue;
      }

      float xmin = cluster.cloud->points[0].x;
      float xmax = xmin;
      float ymin = cluster.cloud->points[0].y;
      float ymax = ymin;
      float zmin = cluster.cloud->points[0].z;
      float zmax = zmin;
      float cx = 0.0f;
      float cy = 0.0f;
      float cz = 0.0f;

      for (const auto & p : cluster.cloud->points) {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
        zmin = std::min(zmin, p.z);
        zmax = std::max(zmax, p.z);
        cx += p.x;
        cy += p.y;
        cz += p.z;
      }

      const float inv = 1.0f / static_cast<float>(cluster.cloud->size());
      cx *= inv;
      cy *= inv;
      cz *= inv;

      const float height = zmax - zmin;
      const float xy_extent = std::max(xmax - xmin, ymax - ymin);
      const float diag = std::sqrt(xy_extent * xy_extent + height * height);
      const float verticality = (diag > 1e-4f) ? (height / diag) : 0.0f;

      if (height < static_cast<float>(params.min_height_m)) {
        ++out.n_rejected_height;
        out.non_tree_cluster_ids.push_back(cluster.id);
        continue;
      }
      if (xy_extent > static_cast<float>(params.max_xy_extent_m)) {
        ++out.n_rejected_extent;
        out.non_tree_cluster_ids.push_back(cluster.id);
        continue;
      }
      if (verticality < static_cast<float>(params.min_verticality)) {
        ++out.n_rejected_verticality;
        out.non_tree_cluster_ids.push_back(cluster.id);
        continue;
      }

      TreeCandidate cand;
      cand.cluster_id = cluster.id;
      cand.point_indices = cluster.point_indices;
      cand.height_m = height;
      cand.xy_extent_m = xy_extent;
      cand.verticality = verticality;
      cand.cx = cx;
      cand.cy = cy;
      cand.cz = cz;
      cand.score = verticality * std::min(height, 3.0f);
      scored.push_back(std::move(cand));
    }

    std::sort(
      scored.begin(), scored.end(),
      [](const TreeCandidate & a, const TreeCandidate & b) {
        return a.score > b.score;
      });

    const std::size_t keep = std::min(
      scored.size(), static_cast<std::size_t>(params.max_candidates_per_frame));
    out.tree_candidates.assign(scored.begin(), scored.begin() + static_cast<std::ptrdiff_t>(keep));

    for (std::size_t i = keep; i < scored.size(); ++i) {
      out.non_tree_cluster_ids.push_back(scored[i].cluster_id);
    }

    return out;
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__TREE_CANDIDATE_EXTRACTOR_HPP_
