/**
 * P-C prep: scorer sobre nuvem agregada (multi-view simulada).
 */
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
using forest_3d_perception::experimental::PointCluster;
using forest_3d_perception::experimental::kScoreTrunk;

PointCluster arc_cluster(float angle_deg, float radius = 0.12f)
{
  PointCluster c;
  c.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  const float rad = angle_deg * 3.14159265f / 180.0f;
  for (int i = 0; i < 25; ++i) {
    const float a = -0.5f * rad + rad * static_cast<float>(i) / 24.0f;
    const float x = radius * std::cos(a);
    const float y = radius * std::sin(a);
    for (int z = 0; z < 8; ++z) {
      c.cloud->push_back(pcl::PointXYZ{x, y, 0.2f + 0.05f * static_cast<float>(z)});
    }
  }
  c.cloud->width = static_cast<std::uint32_t>(c.cloud->size());
  c.cloud->height = 1;
  c.cloud->is_dense = true;
  return c;
}

PointCluster merge_clusters(const PointCluster & a, const PointCluster & b)
{
  PointCluster out;
  out.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  *out.cloud += *a.cloud;
  *out.cloud += *b.cloud;
  out.cloud->width = static_cast<std::uint32_t>(out.cloud->size());
  out.cloud->height = 1;
  out.cloud->is_dense = true;
  return out;
}

}  // namespace

int main()
{
  ClusterClassifier clf;
  clf.params = ClassifierParams{};

  const auto narrow = clf.score_accumulated_cloud(arc_cluster(40.0f));
  const auto wide = clf.score_accumulated_cloud(arc_cluster(140.0f));
  const auto merged = clf.score_accumulated_cloud(
    merge_clusters(arc_cluster(40.0f), arc_cluster(140.0f, 0.12f)));

  if (merged.class_scores[kScoreTrunk] < narrow.class_scores[kScoreTrunk]) {
    std::fprintf(
      stderr, "FAIL: merged trunk score %.3f < narrow %.3f\n",
      merged.class_scores[kScoreTrunk], narrow.class_scores[kScoreTrunk]);
    return 1;
  }
  if (merged.class_scores[kScoreTrunk] < wide.class_scores[kScoreTrunk]) {
    std::fprintf(
      stderr, "FAIL: merged trunk score %.3f < wide %.3f\n",
      merged.class_scores[kScoreTrunk], wide.class_scores[kScoreTrunk]);
    return 1;
  }

  std::printf(
    "PASS accumulated_cloud_scorer narrow=%.2f wide=%.2f merged=%.2f\n",
    narrow.class_scores[kScoreTrunk], wide.class_scores[kScoreTrunk],
    merged.class_scores[kScoreTrunk]);
  return 0;
}
