/**
 * @file csf_ground_segmentation.hpp
 * @brief Ground / non-ground split via CSF (Cloth Simulation Filter).
 *
 * Requires CSF library (FetchContent at build time or third_party/csf sources).
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__CSF_GROUND_SEGMENTATION_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__CSF_GROUND_SEGMENTATION_HPP_

#include <cmath>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/experimental/csf_params.hpp"

#include <CSF.h>

namespace forest_3d_perception::experimental
{

struct CsfGroundResult
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground;
  pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground;
  std::size_t n_input{0};
  std::size_t n_finite{0};       ///< points actually fed to CSF (finite only)
  std::size_t n_non_finite{0};   ///< NaN/Inf returns rejected before CSF
  std::size_t n_ground{0};
  std::size_t n_non_ground{0};
};

class CsfGroundSegmentation
{
public:
  CsfParams params;

  CsfGroundResult segment(const pcl::PointCloud<pcl::PointXYZ> & cloud) const
  {
    CsfGroundResult out;
    out.ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
    out.non_ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
    out.n_input = cloud.size();
    if (cloud.empty()) {
      return out;
    }

    // ROOT-CAUSE GUARD: CSF cannot handle non-finite coordinates. A single
    // NaN/Inf return (Gazebo emits these for no-return rays) corrupts the
    // cloth bounding box / grid indexing -> out-of-bounds particle access ->
    // SIGSEGV. Feed CSF a finite-only subset and map indices back to `cloud`.
    std::vector<csf::Point> csf_points;
    std::vector<std::size_t> orig_index;  // CSF index -> original cloud index
    csf_points.reserve(cloud.size());
    orig_index.reserve(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const auto & p = cloud.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      csf::Point pt;
      pt.x = static_cast<float>(p.x);
      pt.y = static_cast<float>(p.y);
      pt.z = static_cast<float>(p.z);
      csf_points.push_back(pt);
      orig_index.push_back(i);
    }
    out.n_finite = csf_points.size();
    out.n_non_finite = out.n_input - out.n_finite;
    if (csf_points.empty()) {
      return out;
    }

    CSF csf;
    csf.params.bSloopSmooth = params.slope_smooth;
    csf.params.cloth_resolution = static_cast<double>(params.cloth_resolution);
    csf.params.rigidness = params.rigidness;
    csf.params.interations = params.iterations;
    csf.params.class_threshold = params.class_threshold;
    csf.params.time_step = params.time_step;
    csf.setPointCloud(csf_points);

    std::vector<int> ground_idx;
    std::vector<int> off_ground_idx;
    csf.do_filtering(ground_idx, off_ground_idx, false);

    out.ground->reserve(ground_idx.size());
    for (int idx : ground_idx) {
      if (idx >= 0 && static_cast<std::size_t>(idx) < orig_index.size()) {
        out.ground->push_back(cloud.points[orig_index[static_cast<std::size_t>(idx)]]);
      }
    }
    out.non_ground->reserve(off_ground_idx.size());
    for (int idx : off_ground_idx) {
      if (idx >= 0 && static_cast<std::size_t>(idx) < orig_index.size()) {
        out.non_ground->push_back(cloud.points[orig_index[static_cast<std::size_t>(idx)]]);
      }
    }

    out.n_ground = out.ground->size();
    out.n_non_ground = out.non_ground->size();
    return out;
  }
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__CSF_GROUND_SEGMENTATION_HPP_
