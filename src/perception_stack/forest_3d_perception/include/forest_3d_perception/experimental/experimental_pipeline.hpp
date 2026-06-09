/**
 * @file experimental_pipeline.hpp
 * @brief Orchestrator for parallel experimental LiDAR perception (Sprints 1–3).
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__EXPERIMENTAL_PIPELINE_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__EXPERIMENTAL_PIPELINE_HPP_

#include "forest_3d_perception/experimental/sprint1_pipeline.hpp"
#include "forest_3d_perception/experimental/tree_candidate_extractor.hpp"
#include "forest_3d_perception/experimental/tree_verification.hpp"

namespace forest_3d_perception::experimental
{

/** 1 = CSF+cluster, 2 = +tree candidates, 3 = +verification */
enum class PipelineSprint : int
{
  GroundClustering = 1,
  TreeCandidates = 2,
  TreeVerification = 3,
};

struct ExperimentalPipelineParams
{
  PipelineSprint sprint{PipelineSprint::GroundClustering};
  Sprint1Params sprint1;
  TreeCandidateParams tree_candidates;
};

struct ExperimentalPipelineResult
{
  Sprint1Result sprint1;
  TreeCandidateResult sprint2;
  TreeVerificationResult sprint3;
};

class ExperimentalPipeline
{
public:
  ExperimentalPipelineParams params;
  Sprint1Pipeline sprint1;
  TreeCandidateExtractor tree_candidates;
  TreeVerification tree_verification;

  void apply_params()
  {
    sprint1.params = params.sprint1;
    sprint1.apply_params();
    tree_candidates.params = params.tree_candidates;
    tree_candidates.enabled =
      static_cast<int>(params.sprint) >= static_cast<int>(PipelineSprint::TreeCandidates);
    tree_verification.enabled =
      static_cast<int>(params.sprint) >= static_cast<int>(PipelineSprint::TreeVerification);
  }

  ExperimentalPipelineResult run(const pcl::PointCloud<pcl::PointXYZ> & cloud) const
  {
    ExperimentalPipelineResult out;
    out.sprint1 = sprint1.run(cloud);
    if (tree_candidates.enabled && !out.sprint1.clusters.clusters.empty()) {
      out.sprint2 = tree_candidates.extract(out.sprint1.clusters.clusters);
    }
    if (tree_verification.enabled && !out.sprint2.tree_candidates.empty()) {
      out.sprint3 = tree_verification.verify(out.sprint2.tree_candidates);
    }
    return out;
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__EXPERIMENTAL_PIPELINE_HPP_
