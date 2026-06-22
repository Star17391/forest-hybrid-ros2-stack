#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace forest_3d_perception
{

constexpr std::size_t kLandmarkNumClassScores = 3;

struct LandmarkClassScores
{
  std::array<float, kLandmarkNumClassScores> scores{{0.0F, 0.0F, 0.0F}};
  bool valid{false};
};

/** Scorer suave (P-C) sobre pontos acumulados — reutilizável pelo SLAM (S-F). */
LandmarkClassScores score_landmark_points_map(const std::vector<Eigen::Vector3d> & points_map);

}  // namespace forest_3d_perception
