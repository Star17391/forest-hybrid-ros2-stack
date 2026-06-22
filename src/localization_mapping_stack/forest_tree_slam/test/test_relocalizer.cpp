#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "forest_tree_slam/relocalizer.hpp"

using forest_tree_slam::LandmarkPoint;
using forest_tree_slam::RelocalizerParams;
using forest_tree_slam::TreeLocRelocalizer;

namespace
{
// Mapa sintético: dispersão ALEATÓRIA de troncos (não grelha) — uma grelha
// regular (mesmo com jitter pequeno) tem triângulos quase-congruentes
// repetidos por translação, o que é adversarial para o descritor de
// triângulo (ambiguidade geométrica que não existe numa floresta real
// irregular). Sorteio com distância mínima entre troncos (~1.2m, plausível).
std::vector<LandmarkPoint> make_synthetic_forest_map()
{
  std::vector<LandmarkPoint> map;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> coord(0.0, 24.0);
  std::uniform_real_distribution<double> diam_dist(0.2, 0.5);
  forest_tree_slam::LandmarkUid uid = 1;
  constexpr double kMinSeparation = 1.2;
  while (map.size() < 64) {
    const double x = coord(rng), y = coord(rng);
    bool ok = true;
    for (const auto & p : map) {
      if (std::hypot(p.x - x, p.y - y) < kMinSeparation) {
        ok = false;
        break;
      }
    }
    if (!ok) {
      continue;
    }
    LandmarkPoint p;
    p.uid = uid++;
    p.x = x;
    p.y = y;
    p.diameter = diam_dist(rng);
    map.push_back(p);
  }
  return map;
}

// Extrai uma vizinhança local do mapa em torno de (cx,cy), e aplica uma
// transformação rígida (dx,dy,dtheta) para simular a observação local do
// robô em coordenadas próprias (frame "query", ~10m de revisita do mapa).
std::vector<LandmarkPoint> make_query_from_map(
  const std::vector<LandmarkPoint> & map, double cx, double cy, double radius,
  double dx, double dy, double dtheta, std::vector<forest_tree_slam::LandmarkUid> * truth = nullptr)
{
  std::vector<LandmarkPoint> query;
  const double c = std::cos(dtheta), s = std::sin(dtheta);
  for (const auto & p : map) {
    if (std::hypot(p.x - cx, p.y - cy) > radius) {
      continue;
    }
    LandmarkPoint q;
    q.uid = 0;  // identidade desconhecida — é isso que o relocalizador resolve
    const double lx = p.x - cx, ly = p.y - cy;
    q.x = c * lx - s * ly + dx;
    q.y = s * lx + c * ly + dy;
    q.diameter = p.diameter;
    query.push_back(q);
    if (truth) {
      truth->push_back(p.uid);
    }
  }
  return query;
}
}  // namespace

TEST(Relocalizer, AcceptsConsistentRevisitAndRecoversTransform)
{
  const auto map = make_synthetic_forest_map();
  // Revisita ~10m do canto inicial do mapa, robô chegou rodado 30 graus.
  const double dtheta = 30.0 * M_PI / 180.0;
  std::vector<forest_tree_slam::LandmarkUid> truth;
  const auto query = make_query_from_map(map, 10.0, 10.0, 6.0, 1.5, -2.0, dtheta, &truth);
  ASSERT_GE(query.size(), 6u);

  TreeLocRelocalizer reloc;
  const auto result = reloc.relocalize(query, map);

  // Sem ruído nesta query (geometria exata), TODAS as correspondências
  // aceites têm de bater com a verdade-terreno — qualquer falha aqui indica
  // um bug de indexação no matching de triângulos, não apenas tolerâncias.
  for (const auto & c : reloc.last_best_inliers()) {
    EXPECT_EQ(c.map_uid, truth[c.query_index])
      << "correspondência errada para a deteção de query #" << c.query_index;
  }

  ASSERT_TRUE(result.accepted);
  EXPECT_GE(result.overlap_ratio, 0.5);
  EXPECT_LT(result.mean_residual_m, 0.4);

  // A transformação recuperada deve aproximar a inversa de (dx,dy,dtheta)
  // aplicada — verificamos indiretamente: transformar um ponto da query
  // pela pose devolvida deve cair perto do ponto correspondente no mapa.
  for (const auto & c : result.correspondences) {
    const auto & q = query[c.query_index];
    const auto map_it = std::find_if(
      map.begin(), map.end(), [&](const LandmarkPoint & p) {return p.uid == c.map_uid;});
    ASSERT_NE(map_it, map.end());
    const double cs = std::cos(result.map_to_query_transform.theta);
    const double sn = std::sin(result.map_to_query_transform.theta);
    const double px = cs * q.x - sn * q.y + result.map_to_query_transform.x;
    const double py = sn * q.x + cs * q.y + result.map_to_query_transform.y;
    EXPECT_NEAR(px, map_it->x, 0.5);
    EXPECT_NEAR(py, map_it->y, 0.5);
  }
}

TEST(Relocalizer, AcceptsQueryWithMixedClassLandmarks)
{
  std::vector<LandmarkPoint> map;
  forest_tree_slam::LandmarkUid uid = 1;
  std::mt19937 rng(99);
  std::uniform_real_distribution<double> coord(2.0, 22.0);
  constexpr double kMinSeparation = 1.2;
  while (map.size() < 48) {
    const double x = coord(rng), y = coord(rng);
    bool ok = true;
    for (const auto & p : map) {
      if (std::hypot(p.x - x, p.y - y) < kMinSeparation) {
        ok = false;
        break;
      }
    }
    if (!ok) {
      continue;
    }
    LandmarkPoint p;
    p.uid = uid++;
    p.x = x;
    p.y = y;
    // Troncos finos (~0.25 m) e rochas grandes (0.6–1.2 m) — classes mistas.
    p.diameter = (map.size() % 3 ==
      0) ? 0.25 + 0.1 * (map.size() % 4) : 0.6 + 0.15 * (map.size() % 5);
    map.push_back(p);
  }

  std::vector<forest_tree_slam::LandmarkUid> truth;
  const auto query = make_query_from_map(map, 12.0, 12.0, 7.0, 0.5, -1.0, 0.25, &truth);
  ASSERT_GE(query.size(), 6u);

  RelocalizerParams params;
  params.diameter_bin_max_m = 1.5;
  TreeLocRelocalizer reloc(params);
  const auto result = reloc.relocalize(query, map);
  EXPECT_TRUE(result.accepted);
  EXPECT_GE(result.overlap_ratio, 0.5);
}

TEST(Relocalizer, RejectsUnrelatedQueryWithNoTrueMatch)
{
  const auto map = make_synthetic_forest_map();
  // Query "aleatória" sem qualquer correspondência geométrica real ao mapa.
  std::vector<LandmarkPoint> query;
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> coord(-50.0, -40.0);  // bem fora do mapa
  std::uniform_real_distribution<double> diam_dist(0.2, 0.5);
  for (int i = 0; i < 6; ++i) {
    LandmarkPoint p;
    p.x = coord(rng);
    p.y = coord(rng);
    p.diameter = diam_dist(rng);
    query.push_back(p);
  }

  TreeLocRelocalizer reloc;
  const auto result = reloc.relocalize(query, map);
  EXPECT_FALSE(result.accepted);
}

TEST(Relocalizer, RejectsTooFewLandmarksToCorrespond)
{
  const auto map = make_synthetic_forest_map();
  std::vector<LandmarkPoint> query(2);  // < min_correspondences
  TreeLocRelocalizer reloc;
  EXPECT_FALSE(reloc.relocalize(query, map).accepted);
}
