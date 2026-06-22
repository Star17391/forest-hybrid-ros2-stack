/**
 * Unit tests for the soft cluster scorer (P-A).
 */
#include <cmath>
#include <cstdio>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/experimental/cluster_classifier.hpp"
#include "forest_3d_perception/experimental/euclidean_clustering.hpp"

namespace
{

using forest_3d_perception::experimental::ClassifierParams;
using forest_3d_perception::experimental::ClusterClassifier;
using forest_3d_perception::experimental::ClusterFeatures;
using forest_3d_perception::experimental::PointCluster;
using forest_3d_perception::experimental::kScoreRock;
using forest_3d_perception::experimental::kScoreTrunk;

forest_3d_perception::experimental::PointCluster make_vertical_trunk_cluster()
{
  PointCluster c;
  c.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 40; ++i) {
    const float z = 0.2f + 0.05f * static_cast<float>(i);
    c.cloud->push_back(pcl::PointXYZ{0.02f * std::sin(z), 0.02f * std::cos(z), z});
  }
  c.cloud->width = static_cast<std::uint32_t>(c.cloud->size());
  c.cloud->height = 1;
  c.cloud->is_dense = true;
  return c;
}

forest_3d_perception::experimental::PointCluster make_smooth_rock_cluster()
{
  PointCluster c;
  c.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (int ix = 0; ix < 12; ++ix) {
    for (int iy = 0; iy < 12; ++iy) {
      const float x = -0.25f + 0.05f * static_cast<float>(ix);
      const float y = -0.25f + 0.05f * static_cast<float>(iy);
      const float z = 0.15f + 0.01f * std::sin(3.0f * x) * std::cos(3.0f * y);
      c.cloud->push_back(pcl::PointXYZ{x, y, z});
    }
  }
  c.cloud->width = static_cast<std::uint32_t>(c.cloud->size());
  c.cloud->height = 1;
  c.cloud->is_dense = true;
  return c;
}

bool near_one(float sum)
{
  return std::abs(sum - 1.0f) < 0.02f;
}

}  // namespace

int main()
{
  ClusterClassifier clf;
  clf.params = ClassifierParams{};

  const auto trunk = clf.score_cluster(make_vertical_trunk_cluster(), 20.0f);
  const auto rock = clf.score_cluster(make_smooth_rock_cluster(), 0.0f);

  const float trunk_sum = trunk.class_scores[0] + trunk.class_scores[1] + trunk.class_scores[2];
  const float rock_sum = rock.class_scores[0] + rock.class_scores[1] + rock.class_scores[2];

  if (!near_one(trunk_sum) || !near_one(rock_sum)) {
    std::fprintf(stderr, "FAIL: scores must sum to ~1 (trunk=%.3f rock=%.3f)\n", trunk_sum, rock_sum);
    return 1;
  }
  if (trunk.class_scores[kScoreTrunk] <= rock.class_scores[kScoreTrunk]) {
    std::fprintf(
      stderr, "FAIL: trunk cluster should dominate trunk score (T=%.3f R=%.3f)\n",
      trunk.class_scores[kScoreTrunk], rock.class_scores[kScoreTrunk]);
    return 1;
  }
  if (rock.class_scores[kScoreRock] <= trunk.class_scores[kScoreRock]) {
    std::fprintf(
      stderr, "FAIL: rock cluster should dominate rock score (T=%.3f R=%.3f)\n",
      trunk.class_scores[kScoreRock], rock.class_scores[kScoreRock]);
    return 1;
  }

  ClusterFeatures feat{};
  feat.verticality = 0.9f;
  feat.linearity = 0.7f;
  feat.trunk_core_height = 1.2f;
  feat.surface_variation = 0.02f;
  feat.local_roughness = 0.01f;
  feat.height_span = 2.0f;
  const auto probs = ClusterClassifier::score_class_probs(feat, clf.params, 20.0f);
  if (!near_one(probs[0] + probs[1] + probs[2])) {
    std::fprintf(stderr, "FAIL: pure scorer must normalize\n");
    return 1;
  }

  std::printf(
    "PASS soft_cluster_scorer trunk[T=%.2f R=%.2f] rock[T=%.2f R=%.2f]\n",
    trunk.class_scores[kScoreTrunk], trunk.class_scores[kScoreRock],
    rock.class_scores[kScoreTrunk], rock.class_scores[kScoreRock]);
  return 0;
}
