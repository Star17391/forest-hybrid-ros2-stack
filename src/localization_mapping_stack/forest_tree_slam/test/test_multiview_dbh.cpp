#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "forest_tree_slam/multiview_dbh.hpp"

using forest_tree_slam::DbhRefitResult;
using forest_tree_slam::MultiviewDbhParams;
using forest_tree_slam::MultiviewPointBuffer;

namespace
{
std::vector<Eigen::Vector3d> arc_points(double cx, double cy, double r, double a0, double a1, int n)
{
  std::vector<Eigen::Vector3d> pts;
  for (int i = 0; i < n; ++i) {
    const double t = a0 + (a1 - a0) * static_cast<double>(i) / static_cast<double>(n - 1);
    pts.emplace_back(cx + r * std::cos(t), cy + r * std::sin(t), 1.0);
  }
  return pts;
}
}  // namespace

TEST(MultiviewDbh, VoxelDedupCollapsesDuplicatePoints)
{
  MultiviewPointBuffer buf;
  const Eigen::Vector2d lm(0.0, 0.0);
  const Eigen::Vector2d view(5.0, 0.0);
  std::vector<Eigen::Vector3d> pts = {Eigen::Vector3d(1.0, 0.0, 1.0),
    Eigen::Vector3d(1.005, 0.0, 1.0)};
  EXPECT_TRUE(buf.insert_frame(pts, lm, view));
  EXPECT_EQ(buf.n_voxels(), 1u);
}

TEST(MultiviewDbh, CoverageGrowsWithDifferentViewAngles)
{
  MultiviewPointBuffer buf;
  const Eigen::Vector2d lm(0.0, 0.0);
  const std::vector<Eigen::Vector3d> pts = {Eigen::Vector3d(0.2, 0.0, 1.0)};
  buf.insert_frame(pts, lm, Eigen::Vector2d(5.0, 0.0));
  const double c0 = buf.coverage_ratio();
  buf.insert_frame(pts, lm, Eigen::Vector2d(0.0, 5.0));
  EXPECT_GT(buf.coverage_ratio(), c0);
  EXPECT_GE(buf.n_inlier_frames(), 2u);
}

TEST(MultiviewDbh, MergedArcRefitImprovesRadiusEstimate)
{
  MultiviewDbhParams params;
  params.refit_min_points = 8;
  MultiviewPointBuffer buf(params);
  const Eigen::Vector2d lm(0.0, 0.0);
  const double true_r = 0.20;
  auto narrow = arc_points(0.0, 0.0, true_r, -0.5, 0.5, 20);
  auto wide = arc_points(0.0, 0.0, true_r, -1.2, 1.2, 30);
  buf.insert_frame(narrow, lm, Eigen::Vector2d(4.0, 0.0));
  const DbhRefitResult narrow_fit = buf.refit();
  ASSERT_TRUE(narrow_fit.valid);
  buf.insert_frame(wide, lm, Eigen::Vector2d(0.0, 4.0));
  const DbhRefitResult merged_fit = buf.refit();
  ASSERT_TRUE(merged_fit.valid);
  EXPECT_NEAR(merged_fit.radius, true_r, 0.03);
  EXPECT_GT(merged_fit.arc_coverage, narrow_fit.arc_coverage);
}

// Captura a falha real da Tree2 no sim: um frame com pose derivada deposita um
// anel DESLOCADO no buffer permanente e o Taubin algébrico explodia o raio
// (0.5→1.3). O refit aparado ancorado no prior tem de rejeitar esse anel.
TEST(MultiviewDbh, TrimmedRefitRejectsDisplacedOutlierRing)
{
  MultiviewDbhParams params;
  params.refit_min_points = 8;
  MultiviewPointBuffer buf(params);
  const Eigen::Vector2d lm(0.0, 0.0);
  const double true_r = 0.20;
  // Anel verdadeiro, bem coberto, centrado na origem.
  buf.insert_frame(arc_points(0.0, 0.0, true_r, -1.5, 1.5, 40), lm, Eigen::Vector2d(4.0, 0.0));
  // Anel DESLOCADO (centro a 0.4 m) de um frame mau — minoria.
  buf.insert_frame(arc_points(0.4, 0.0, true_r, -1.0, 1.0, 12), lm, Eigen::Vector2d(0.0, 4.0));

  // Com o prior correto (como o tracker chama), o anel deslocado é aparado.
  const DbhRefitResult fit = buf.refit(0.0, 0.0, true_r, true);
  ASSERT_TRUE(fit.valid);
  EXPECT_NEAR(fit.radius, true_r, 0.04) << "anel deslocado não pode inflar o raio";
  EXPECT_LT(std::hypot(fit.cx, fit.cy), 0.1) << "centro fica no tronco verdadeiro";
}

TEST(MultiviewDbh, SaturatesButRetainsVoxelsForViz)
{
  MultiviewDbhParams params;
  params.coverage_bins = 4;
  params.saturation_coverage = 0.5;
  params.min_inlier_frames = 2;
  params.saturation_max_diameter_var = 1.0;
  MultiviewPointBuffer buf(params);
  const Eigen::Vector2d lm(0.0, 0.0);
  const std::vector<Eigen::Vector3d> pt = {Eigen::Vector3d(0.2, 0.0, 1.0)};
  buf.insert_frame(pt, lm, Eigen::Vector2d(3.0, 0.0));
  buf.insert_frame(pt, lm, Eigen::Vector2d(0.0, 3.0));
  buf.insert_frame(pt, lm, Eigen::Vector2d(-3.0, 0.0));
  EXPECT_TRUE(buf.should_saturate(0.001));
  buf.saturate();
  EXPECT_TRUE(buf.saturated());
  // Novo comportamento: a saturação NÃO limpa os voxels — a nuvem acumulada fica
  // disponível via points() para visualização no RViz.
  EXPECT_GT(buf.n_voxels(), 0u);
  EXPECT_FALSE(buf.points().empty());
  // Saturado => não ingere mais (insert_frame devolve false e não acrescenta voxels).
  const std::size_t n_before = buf.n_voxels();
  EXPECT_FALSE(buf.insert_frame(pt, lm, Eigen::Vector2d(0.0, -3.0)));
  EXPECT_EQ(buf.n_voxels(), n_before);
}
