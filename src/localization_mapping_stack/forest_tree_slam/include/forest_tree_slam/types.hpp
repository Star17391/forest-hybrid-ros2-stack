#pragma once

#include <array>
#include <cstdint>

// Eigen/Core só DECLARA inverse() (MatrixBase); a implementação vive no
// módulo LU. Sem este include, ligações que chamem .inverse() falham com
// "undefined reference" (a declaração existe, a definição não).
#include <Eigen/Dense>

#include "forest_tree_slam/landmark_class.hpp"

namespace forest_tree_slam
{

// Pose 2D do robô (keyframe), SE(2). theta em radianos, [-pi, pi[.
struct Pose2
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

// Deteção crua de um tronco num único frame, já no frame do robô (base_link)
// ou já transladada para `map` por um Δodom de predição — o tracker decide.
// Espelha forest_hybrid_msgs/msg/TreeLandmark (sem o tracking).
struct TreeDetection
{
  double x{0.0};
  double y{0.0};
  double diameter{0.0};        // DBH [m]
  float confidence{0.0F};
  // Covariância 3x3 row-major sobre (x,y,z) da base, em m^2 (vem da perceção).
  // Só a submatriz 2x2 em (x,y) é usada pelo gate de Mahalanobis.
  Eigen::Matrix3d base_covariance{Eigen::Matrix3d::Zero()};
  float diameter_stddev{0.0F};
  std::array<float, kNumClassScores> class_scores{{0.0F, 0.0F, 0.0F}};
  bool has_stem_inliers{false};
};

// uid imutável: nunca recodifica posição (tile). Atribuído monotonicamente na
// 1.ª deteção (FOREST_TREE_SLAM_DESIGN.md §2.2).
using LandmarkUid = std::uint64_t;

}  // namespace forest_tree_slam
