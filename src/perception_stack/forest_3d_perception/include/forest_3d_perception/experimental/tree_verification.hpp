/**
 * @file tree_verification.hpp
 * @brief Sprint 3 (stub): scoring and validation of tree candidates.
 *
 * Planned: PCA vertical axis, slice consistency, cylinder RMSE, ground attachment.
 * Not active until pipeline_sprint >= 3.
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__TREE_VERIFICATION_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__TREE_VERIFICATION_HPP_

#include <vector>

#include "forest_3d_perception/experimental/tree_candidate_extractor.hpp"

namespace forest_3d_perception::experimental
{

struct VerifiedTree
{
  int candidate_id{0};
  float score{0.0f};
};

struct TreeVerificationResult
{
  std::vector<VerifiedTree> trees;
  std::vector<int> obstacle_cluster_ids;
  bool enabled{false};
};

class TreeVerification
{
public:
  bool enabled{false};

  TreeVerificationResult verify(
    const std::vector<TreeCandidate> & /*candidates*/) const
  {
    TreeVerificationResult out;
    out.enabled = enabled;
    return out;
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__TREE_VERIFICATION_HPP_
