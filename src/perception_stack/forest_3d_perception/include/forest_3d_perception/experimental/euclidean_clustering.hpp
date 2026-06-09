/**
 * @file euclidean_clustering.hpp
 * @brief PCL Euclidean clustering on non-ground points (experimental Sprint 1).
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__EUCLIDEAN_CLUSTERING_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__EUCLIDEAN_CLUSTERING_HPP_

#include <vector>

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "forest_3d_perception/experimental/clustering_params.hpp"

namespace forest_3d_perception::experimental
{

struct PointCluster
{
  int id{0};
  std::vector<std::size_t> point_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

struct EuclideanClusteringResult
{
  std::vector<PointCluster> clusters;
  std::size_t n_input{0};
  std::size_t n_rejected_small{0};
  std::size_t n_rejected_large{0};
};

class EuclideanClustering
{
public:
  ClusteringParams params;

  EuclideanClusteringResult cluster(const pcl::PointCloud<pcl::PointXYZ> & cloud) const
  {
    EuclideanClusteringResult out;
    out.n_input = cloud.size();
    if (cloud.size() < static_cast<std::size_t>(params.min_cluster_size)) {
      return out;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_ptr);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(params.tolerance);
    ec.setMinClusterSize(params.min_cluster_size);
    ec.setMaxClusterSize(params.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_ptr);
    ec.extract(cluster_indices);

    int id = 0;
    for (const auto & indices : cluster_indices) {
      PointCluster c;
      c.id = id++;
      c.point_indices = std::vector<std::size_t>(
        indices.indices.begin(), indices.indices.end());
      c.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(cloud, indices.indices, *c.cloud);
      out.clusters.push_back(std::move(c));
    }
    return out;
  }

  /** Build XYZI cloud with intensity = cluster_id (0 = unassigned). */
  static pcl::PointCloud<pcl::PointXYZI>::Ptr to_labeled_cloud(
    const pcl::PointCloud<pcl::PointXYZ> & input,
    const std::vector<PointCluster> & clusters)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr labeled(new pcl::PointCloud<pcl::PointXYZI>);
    labeled->resize(input.size());
    for (std::size_t i = 0; i < input.size(); ++i) {
      labeled->points[i].x = input.points[i].x;
      labeled->points[i].y = input.points[i].y;
      labeled->points[i].z = input.points[i].z;
      labeled->points[i].intensity = 0.0f;
    }
    for (const auto & cluster : clusters) {
      for (std::size_t idx : cluster.point_indices) {
        if (idx < labeled->size()) {
          labeled->points[idx].intensity = static_cast<float>(cluster.id + 1);
        }
      }
    }
    labeled->width = static_cast<uint32_t>(labeled->size());
    labeled->height = 1;
    labeled->is_dense = true;
    return labeled;
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__EUCLIDEAN_CLUSTERING_HPP_
