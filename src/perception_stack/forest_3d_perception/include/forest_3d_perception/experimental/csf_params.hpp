/**
 * @file csf_params.hpp
 * @brief Cloth Simulation Filter parameters (experimental pipeline Sprint 1).
 */

#ifndef FOREST_3D_PERCEPTION__EXPERIMENTAL__CSF_PARAMS_HPP_
#define FOREST_3D_PERCEPTION__EXPERIMENTAL__CSF_PARAMS_HPP_

namespace forest_3d_perception::experimental
{

struct CsfParams
{
  double cloth_resolution{0.5};
  int rigidness{3};
  int iterations{500};
  double class_threshold{0.5};
  double time_step{0.65};
  bool slope_smooth{true};
};

}  // namespace forest_3d_perception::experimental

#endif  // FOREST_3D_PERCEPTION__EXPERIMENTAL__CSF_PARAMS_HPP_
