/**
 * @file clustering_params.hpp
 * @brief 3D Euclidean clustering parameters (experimental pipeline Sprint 1).
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__CLUSTERING_PARAMS_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__CLUSTERING_PARAMS_HPP_

namespace forest_3d_perception::experimental
{

struct ClusteringParams
{
  double tolerance{0.25};
  int min_cluster_size{10};
  int max_cluster_size{5000};
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__CLUSTERING_PARAMS_HPP_
