#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/experimental/euclidean_clustering.hpp"
#include "forest_3d_perception/experimental/tree_candidate_extractor.hpp"

namespace
{

forest_3d_perception::experimental::PointCluster make_vertical_cluster(int id)
{
  forest_3d_perception::experimental::PointCluster c;
  c.id = id;
  c.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (float z = 0.2f; z <= 2.5f; z += 0.15f) {
    c.cloud->push_back(pcl::PointXYZ{3.0f, 4.0f, z});
    c.point_indices.push_back(c.cloud->size() - 1);
  }
  return c;
}

forest_3d_perception::experimental::PointCluster make_flat_cluster(int id)
{
  forest_3d_perception::experimental::PointCluster c;
  c.id = id;
  c.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (float x = 0.0f; x <= 1.5f; x += 0.1f) {
    for (float y = 0.0f; y <= 1.5f; y += 0.1f) {
      c.cloud->push_back(pcl::PointXYZ{x, y, 0.1f});
      c.point_indices.push_back(c.cloud->size() - 1);
    }
  }
  return c;
}

}  // namespace

int main()
{
  using forest_3d_perception::experimental::TreeCandidateExtractor;

  std::vector<forest_3d_perception::experimental::PointCluster> clusters;
  clusters.push_back(make_vertical_cluster(0));
  clusters.push_back(make_flat_cluster(1));

  TreeCandidateExtractor extractor;
  extractor.enabled = true;
  const auto out = extractor.extract(clusters);

  if (out.tree_candidates.size() != 1u) {
    std::cerr << "FAIL: expected 1 tree candidate, got " << out.tree_candidates.size() << "\n";
    return 1;
  }
  if (out.tree_candidates[0].cluster_id != 0) {
    std::cerr << "FAIL: wrong cluster id accepted\n";
    return 1;
  }
  if (out.non_tree_cluster_ids.empty()) {
    std::cerr << "FAIL: flat cluster should be rejected\n";
    return 1;
  }
  std::cout << "OK: tree candidate extractor accepted vertical cluster\n";
  return 0;
}
