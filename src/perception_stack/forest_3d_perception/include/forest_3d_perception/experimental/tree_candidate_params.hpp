/**
 * @file tree_candidate_params.hpp
 * @brief Sprint 2 — geometry filters for trunk-like clusters.
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__TREE_CANDIDATE_PARAMS_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__TREE_CANDIDATE_PARAMS_HPP_

namespace forest_3d_perception::experimental
{

struct TreeCandidateParams
{
  double min_height_m{0.55};
  double max_xy_extent_m{1.2};
  double min_verticality{0.55};
  int min_points{8};
  int max_candidates_per_frame{16};
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__TREE_CANDIDATE_PARAMS_HPP_
