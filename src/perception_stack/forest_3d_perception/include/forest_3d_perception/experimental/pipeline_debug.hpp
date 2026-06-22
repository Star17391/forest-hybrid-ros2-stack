/**
 * @file pipeline_debug.hpp
 * @brief Debug stages and per-frame funnel stats (experimental pipeline only).
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__PIPELINE_DEBUG_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__PIPELINE_DEBUG_HPP_

#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/experimental/euclidean_clustering.hpp"

namespace forest_3d_perception::experimental
{

/** 0=full, 1=voxel_only, 2=csf_only, 3=cluster_only(skip CSF), 4=full (alias). */
enum class DebugStageMode : int
{
  Full = 0,
  VoxelOnly = 1,
  CsfOnly = 2,
  ClusterOnly = 3,
};

inline DebugStageMode debug_stage_from_int(int v)
{
  if (v == 1) {
    return DebugStageMode::VoxelOnly;
  }
  if (v == 2) {
    return DebugStageMode::CsfOnly;
  }
  if (v == 3) {
    return DebugStageMode::ClusterOnly;
  }
  return DebugStageMode::Full;
}

enum class PipelineExitStatus : uint8_t
{
  Ok,
  Disabled,
  TfFail,
  RawEmpty,
  CropTooFew,
  VoxelEmpty,
};

inline const char * exit_status_string(PipelineExitStatus s)
{
  switch (s) {
    case PipelineExitStatus::Ok: return "ok";
    case PipelineExitStatus::Disabled: return "disabled";
    case PipelineExitStatus::TfFail: return "tf_fail";
    case PipelineExitStatus::RawEmpty: return "raw_empty";
    case PipelineExitStatus::CropTooFew: return "crop_too_few";
    case PipelineExitStatus::VoxelEmpty: return "voxel_empty";
  }
  return "unknown";
}

struct ClusterDebugSummary
{
  int id{0};
  std::size_t n_points{0};
  float cx{0.0f};
  float cy{0.0f};
  float cz{0.0f};
  float z_min{0.0f};
  float z_max{0.0f};
  float xy_extent{0.0f};
};

inline std::vector<ClusterDebugSummary> summarize_clusters(
  const std::vector<PointCluster> & clusters, std::size_t max_clusters = 12)
{
  std::vector<ClusterDebugSummary> out;
  out.reserve(std::min(clusters.size(), max_clusters));
  for (std::size_t i = 0; i < clusters.size() && i < max_clusters; ++i) {
    const auto & c = clusters[i];
    if (!c.cloud || c.cloud->empty()) {
      continue;
    }
    ClusterDebugSummary s;
    s.id = c.id;
    s.n_points = c.cloud->size();
    float xmin = c.cloud->points[0].x;
    float xmax = xmin;
    float ymin = c.cloud->points[0].y;
    float ymax = ymin;
    s.z_min = c.cloud->points[0].z;
    s.z_max = s.z_min;
    for (const auto & p : c.cloud->points) {
      s.cx += p.x;
      s.cy += p.y;
      s.cz += p.z;
      xmin = std::min(xmin, p.x);
      xmax = std::max(xmax, p.x);
      ymin = std::min(ymin, p.y);
      ymax = std::max(ymax, p.y);
      s.z_min = std::min(s.z_min, p.z);
      s.z_max = std::max(s.z_max, p.z);
    }
    const float inv = 1.0f / static_cast<float>(s.n_points);
    s.cx *= inv;
    s.cy *= inv;
    s.cz *= inv;
    s.xy_extent = std::max(xmax - xmin, ymax - ymin);
    out.push_back(s);
  }
  return out;
}

struct PipelineFunnelStats
{
  PipelineExitStatus status{PipelineExitStatus::Ok};
  std::string input_frame;
  std::string processing_frame;
  std::size_t n_input_msg{0};
  std::size_t n_raw{0};
  std::size_t n_non_finite{0};
  std::size_t n_crop{0};
  std::size_t n_voxel{0};
  std::size_t n_ground{0};
  std::size_t n_non_ground{0};
  // Region-growing funnel (root-cause separation): working set (HAG in band) and
  // ground seeds that actually start a region. A trunk with NO seed this frame is
  // never grown -> flicker. High frame-to-frame CV of n_seeds/n_clusters => the
  // instability is in region growing (Causa C), upstream of classify/cylinder.
  std::size_t n_working{0};
  std::size_t n_seeds{0};
  std::size_t n_clusters{0};
  // Clusters the classifier labelled TRUNK (before the cylinder fit). The gap
  // n_clusters -> n_trunk_classified isolates classify flicker (Causa B); the gap
  // n_trunk_classified -> n_trunk_accept isolates cylinder accept/fallback (Causa A).
  std::size_t n_trunk_classified{0};
  std::size_t n_tree_candidates{0};
  std::size_t n_tree_rejected_height{0};
  std::size_t n_tree_rejected_extent{0};
  std::size_t n_tree_rejected_verticality{0};
  // Trunk cylinder-fit funnel (perception Agente 2): accepted TRUNK landmarks and
  // per-criterion rejections, plus the inter-frame DBH stability proxy (-1 = n/a).
  std::size_t n_trunk_accept{0};
  std::size_t n_trunk_reject_radius{0};
  std::size_t n_trunk_reject_height{0};
  std::size_t n_trunk_reject_verticality{0};
  std::size_t n_trunk_reject_points{0};
  double dbh_stability_pct{-1.0};
  bool gravity_aligned{false};
  // Probabilistic landmark emission (Fase 1 perception).
  std::size_t n_structural_candidates{0};
  std::size_t n_landmarks_emitted{0};
  std::size_t n_dominant_trunk{0};
  std::size_t n_dominant_rock{0};
  std::size_t n_dominant_obstacle{0};
  int pipeline_sprint{1};
  int debug_stage{0};
  double csf_ground_pct{0.0};
  double crop_min_z{0.0};
  double crop_max_z{0.0};
  std::vector<ClusterDebugSummary> cluster_summaries;

  std::string to_json() const
  {
    std::ostringstream oss;
    oss << "{"
        << "\"status\":\"" << exit_status_string(status) << "\""
        << ",\"debug_stage\":" << debug_stage
        << ",\"input_frame\":\"" << input_frame << "\""
        << ",\"processing_frame\":\"" << processing_frame << "\""
        << ",\"n_input_msg\":" << n_input_msg
        << ",\"n_raw\":" << n_raw
        << ",\"n_non_finite\":" << n_non_finite
        << ",\"n_crop\":" << n_crop
        << ",\"n_voxel\":" << n_voxel
        << ",\"n_ground\":" << n_ground
        << ",\"n_non_ground\":" << n_non_ground
        << ",\"n_working\":" << n_working
        << ",\"n_seeds\":" << n_seeds
        << ",\"n_clusters\":" << n_clusters
        << ",\"n_trunk_classified\":" << n_trunk_classified
        << ",\"n_tree_candidates\":" << n_tree_candidates
        << ",\"n_tree_rejected_height\":" << n_tree_rejected_height
        << ",\"n_tree_rejected_extent\":" << n_tree_rejected_extent
        << ",\"n_tree_rejected_verticality\":" << n_tree_rejected_verticality
        << ",\"n_trunk_accept\":" << n_trunk_accept
        << ",\"n_trunk_reject_radius\":" << n_trunk_reject_radius
        << ",\"n_trunk_reject_height\":" << n_trunk_reject_height
        << ",\"n_trunk_reject_verticality\":" << n_trunk_reject_verticality
        << ",\"n_trunk_reject_points\":" << n_trunk_reject_points
        << ",\"dbh_stability_pct\":" << dbh_stability_pct
        << ",\"gravity_aligned\":" << (gravity_aligned ? "true" : "false")
        << ",\"n_structural_candidates\":" << n_structural_candidates
        << ",\"n_landmarks_emitted\":" << n_landmarks_emitted
        << ",\"n_dominant_trunk\":" << n_dominant_trunk
        << ",\"n_dominant_rock\":" << n_dominant_rock
        << ",\"n_dominant_obstacle\":" << n_dominant_obstacle
        << ",\"pipeline_sprint\":" << pipeline_sprint
        << ",\"csf_ground_pct\":" << csf_ground_pct
        << ",\"crop_z_min\":" << crop_min_z
        << ",\"crop_z_max\":" << crop_max_z
        << ",\"clusters\":[";
    for (std::size_t i = 0; i < cluster_summaries.size(); ++i) {
      if (i > 0) {
        oss << ",";
      }
      const auto & c = cluster_summaries[i];
      oss << "{\"id\":" << c.id
          << ",\"n\":" << c.n_points
          << ",\"cx\":" << c.cx << ",\"cy\":" << c.cy << ",\"cz\":" << c.cz
          << ",\"z_min\":" << c.z_min << ",\"z_max\":" << c.z_max
          << ",\"xy_extent\":" << c.xy_extent << "}";
    }
    oss << "]}";
    return oss.str();
  }
};

inline void update_crop_z_bounds(const pcl::PointCloud<pcl::PointXYZ> & cloud, PipelineFunnelStats & s)
{
  if (cloud.empty()) {
    s.crop_min_z = 0.0;
    s.crop_max_z = 0.0;
    return;
  }
  float zmin = cloud.points[0].z;
  float zmax = zmin;
  for (const auto & p : cloud.points) {
    zmin = std::min(zmin, p.z);
    zmax = std::max(zmax, p.z);
  }
  s.crop_min_z = zmin;
  s.crop_max_z = zmax;
}

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__PIPELINE_DEBUG_HPP_
