/**
 * @file trunk_pipeline_audit.hpp
 * @brief Per-stage funnel metrics for slice trunk pipeline (debug / validation).
 */

#ifndef FOREST_3D_PERCEPTION__TRUNK_PIPELINE_AUDIT_HPP_
#define FOREST_3D_PERCEPTION__TRUNK_PIPELINE_AUDIT_HPP_

#include <cstddef>
#include <cstring>

namespace forest_3d_perception
{

/** One rejected stem candidate (top-N kept by slice count). */
struct RejectedStemSample
{
  char reason[28]{};
  int n_slices{0};
  float cx{0.0f};
  float cy{0.0f};
  float continuity{0.0f};
  float drift{0.0f};
  float persistence{0.0f};
  float radius_cv{0.0f};
  float bottom_dz_m{0.0f};
  float threshold{0.0f};
  float measured{0.0f};
};

/** Counts at each pipeline stage — published in debug_stats + rosout funnel. */
struct TrunkPipelineFunnel
{
  std::size_t n_nonground_in{0};
  std::size_t n_voxel_context{0};
  std::size_t n_band_points{0};
  std::size_t n_band_skip_nan_ground{0};
  std::size_t n_band_skip_height{0};
  std::size_t n_slices_used{0};
  std::size_t n_slices_nonempty{0};
  std::size_t n_grid_cells_occupied{0};
  std::size_t n_clusters_2d{0};
  std::size_t n_stems_finished{0};
  std::size_t n_accepted_cylinders{0};
  std::size_t n_landmarks_input{0};

  std::size_t reject_sparse_few_slices{0};
  std::size_t reject_sparse_few_points{0};
  std::size_t reject_cont_score{0};
  std::size_t reject_cont_persistence{0};
  std::size_t reject_cont_drift{0};
  std::size_t reject_cont_radius_cv{0};
  std::size_t reject_ground_dz{0};
  std::size_t reject_ground_cell{0};
  std::size_t reject_ground_z_nan{0};
  std::size_t reject_cylinder_short{0};
  std::size_t reject_cylinder_wide{0};
  std::size_t reject_cylinder_rmse{0};
  std::size_t reject_cylinder_inliers{0};
  /** Merged 2D component had fewer than min_points_per_cluster. */
  std::size_t reject_cluster_too_small{0};

  int max_clusters_in_slice{0};
  int best_rej_slices{0};
  float best_rej_continuity{0.0f};
  float best_rej_drift{0.0f};
  float best_rej_persist{0.0f};
  float best_rej_bottom_dz{0.0f};

  static constexpr int kMaxSamples = 3;
  RejectedStemSample reject_samples[kMaxSamples]{};
  int n_reject_samples{0};

  void record_reject(
    const char * reason, int n_slices, float cx, float cy,
    const RejectedStemSample & m)
  {
    RejectedStemSample s = m;
    std::strncpy(s.reason, reason, sizeof(s.reason) - 1);
    s.n_slices = n_slices;
    s.cx = cx;
    s.cy = cy;
    if (n_reject_samples < kMaxSamples) {
      reject_samples[n_reject_samples++] = s;
      return;
    }
    int worst = 0;
    for (int i = 1; i < kMaxSamples; ++i) {
      if (reject_samples[i].n_slices < reject_samples[worst].n_slices) {
        worst = i;
      }
    }
    if (n_slices > reject_samples[worst].n_slices) {
      reject_samples[worst] = s;
    }
  }
};

}  // namespace forest_3d_perception

#endif  // FOREST_3D_PERCEPTION__TRUNK_PIPELINE_AUDIT_HPP_
