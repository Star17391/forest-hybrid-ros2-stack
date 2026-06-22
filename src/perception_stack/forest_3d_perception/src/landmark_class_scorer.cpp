#include "forest_3d_perception/landmark_class_scorer.hpp"

#include <numeric>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/experimental/cluster_classifier.hpp"

namespace forest_3d_perception
{

LandmarkClassScores score_landmark_points_map(const std::vector<Eigen::Vector3d> & points_map)
{
  LandmarkClassScores out;
  if (points_map.size() < 4) {
    return out;
  }

  experimental::PointCluster cluster;
  cluster.id = 0;
  cluster.cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cluster.cloud->reserve(points_map.size());
  for (const auto & p : points_map) {
    cluster.cloud->push_back(pcl::PointXYZ(
      static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())));
  }
  cluster.point_indices.resize(points_map.size());
  std::iota(cluster.point_indices.begin(), cluster.point_indices.end(), 0);

  experimental::ClusterClassifier clf;
  const experimental::ScoredCluster scored = clf.score_accumulated_cloud(cluster);
  const float sum =
    scored.class_scores[0] + scored.class_scores[1] + scored.class_scores[2];
  if (sum < 1.0e-6F) {
    return out;
  }

  out.scores = scored.class_scores;
  out.valid = true;
  return out;
}

}  // namespace forest_3d_perception
