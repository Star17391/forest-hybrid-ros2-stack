#include <gtest/gtest.h>

#include "forest_tree_slam/camera_projection.hpp"

using forest_tree_slam::CameraIntrinsics;
using forest_tree_slam::pixel_in_bbox;
using forest_tree_slam::project_base_to_image;

namespace
{
// Câmara sintética 640×480, fx=fy=500, principal no centro.
CameraIntrinsics make_k()
{
  CameraIntrinsics k;
  k.fx = 500.0;
  k.fy = 500.0;
  k.cx = 320.0;
  k.cy = 240.0;
  k.width = 640;
  k.height = 480;
  k.valid = true;
  return k;
}
const Eigen::Vector3d kExtrinsic(0.40, 0.0, 0.24);  // base→câmara (RPY=0)
}  // namespace

TEST(CameraProjection, PointStraightAheadHitsImageCentre)
{
  // Ponto no eixo óptico: 5 m à frente da câmara, à mesma altura.
  const Eigen::Vector3d p(5.40, 0.0, 0.24);
  const auto pr = project_base_to_image(p, kExtrinsic, make_k());
  ASSERT_TRUE(pr.valid);
  EXPECT_NEAR(pr.depth, 5.0, 1e-9);
  EXPECT_NEAR(pr.u, 320.0, 1e-6);
  EXPECT_NEAR(pr.v, 240.0, 1e-6);
}

TEST(CameraProjection, PointToTheLeftMapsToSmallerU)
{
  // +y em base_link (esquerda) → lado esquerdo da imagem (u < cx).
  const Eigen::Vector3d p(5.40, 1.0, 0.24);
  const auto pr = project_base_to_image(p, kExtrinsic, make_k());
  ASSERT_TRUE(pr.valid);
  EXPECT_NEAR(pr.u, 500.0 * (-1.0 / 5.0) + 320.0, 1e-6);  // = 220
  EXPECT_LT(pr.u, 320.0);
}

TEST(CameraProjection, PointAboveMapsToSmallerV)
{
  // +z em base_link (acima) → topo da imagem (v < cy).
  const Eigen::Vector3d p(5.40, 0.0, 1.24);
  const auto pr = project_base_to_image(p, kExtrinsic, make_k());
  ASSERT_TRUE(pr.valid);
  EXPECT_NEAR(pr.v, 500.0 * (-1.0 / 5.0) + 240.0, 1e-6);  // = 140
  EXPECT_LT(pr.v, 240.0);
}

TEST(CameraProjection, PointBehindCameraRejected)
{
  // Atrás da câmara (x < extrínseca.x) → inválido.
  const Eigen::Vector3d p(0.30, 0.0, 0.24);
  EXPECT_FALSE(project_base_to_image(p, kExtrinsic, make_k()).valid);
}

TEST(CameraProjection, PointOutsideFovRejected)
{
  // Muito à esquerda e perto → u sai da imagem.
  const Eigen::Vector3d p(1.40, 5.0, 0.24);
  EXPECT_FALSE(project_base_to_image(p, kExtrinsic, make_k()).valid);
}

TEST(CameraProjection, InvalidIntrinsicsGiveNoProjection)
{
  CameraIntrinsics k;  // valid=false
  EXPECT_FALSE(project_base_to_image(Eigen::Vector3d(5.4, 0, 0.24), kExtrinsic, k).valid);
}

TEST(CameraProjection, BboxContainmentMatchesProjectedPixel)
{
  const auto pr = project_base_to_image(
    Eigen::Vector3d(5.40, 0.0, 0.24), kExtrinsic, make_k());
  ASSERT_TRUE(pr.valid);
  // Caixa centrada no pixel projetado contém-no; caixa afastada não.
  EXPECT_TRUE(pixel_in_bbox(pr.u, pr.v, 320.0, 240.0, 60.0, 120.0));
  EXPECT_FALSE(pixel_in_bbox(pr.u, pr.v, 100.0, 240.0, 60.0, 120.0));
}
