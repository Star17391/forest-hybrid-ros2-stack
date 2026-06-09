/**
 * @file sprint1_pipeline.hpp
 * @brief Experimental LiDAR pipeline Sprint 1: CSF ground + Euclidean clustering.
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__SPRINT1_PIPELINE_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__SPRINT1_PIPELINE_HPP_

#include "forest_3d_perception/experimental/clustering_params.hpp"
#include "forest_3d_perception/experimental/csf_ground_segmentation.hpp"
#include "forest_3d_perception/experimental/csf_params.hpp"
#include "forest_3d_perception/experimental/euclidean_clustering.hpp"

namespace forest_3d_perception::experimental
{

struct Sprint1Params
{
  CsfParams csf;
  ClusteringParams clustering;
};

struct Sprint1Result
{
  CsfGroundResult ground;
  EuclideanClusteringResult clusters;
};

class Sprint1Pipeline
{
public:
  Sprint1Params params;
  CsfGroundSegmentation csf_segmenter;
  EuclideanClustering clusterer;

  void apply_params()
  {
    csf_segmenter.params = params.csf;
    clusterer.params = params.clustering;
  }

  Sprint1Result run(const pcl::PointCloud<pcl::PointXYZ> & cloud) const
  {
    Sprint1Result out;
    out.ground = csf_segmenter.segment(cloud);
    if (out.ground.non_ground && !out.ground.non_ground->empty()) {
      out.clusters = clusterer.cluster(*out.ground.non_ground);
    }
    return out;
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__SPRINT1_PIPELINE_HPP_
