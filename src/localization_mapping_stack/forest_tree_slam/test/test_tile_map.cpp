// Testes do endereçamento de tiles fixos (tile_map.hpp).
//
// O que está em jogo: a grelha é FIXA por coordenadas (tile (0,0) centrado na
// origem do mundo), o uid é imutável e o tile é um índice derivado da posição
// otimizada — no loop closure o landmark re-bucketa, o uid nunca muda.

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "forest_tree_slam/tile_map.hpp"

namespace fts = forest_tree_slam;

// ---------------------------------------------------------------- TileGrid --

TEST(TileGrid, OriginTileIsCenteredOnWorldOrigin)
{
  const fts::TileGrid g(20.0);
  // Tile (0,0) cobre [-10, +10[ em x e y.
  EXPECT_EQ(g.index_of({0.0, 0.0}), (fts::TileIndex{0, 0}));
  EXPECT_EQ(g.index_of({9.99, 9.99}), (fts::TileIndex{0, 0}));
  EXPECT_EQ(g.index_of({-10.0, -10.0}), (fts::TileIndex{0, 0}));
  // Fronteira superior é semiaberta: +10 já pertence ao tile seguinte.
  EXPECT_EQ(g.index_of({10.0, 0.0}), (fts::TileIndex{0, 1}));
  EXPECT_EQ(g.index_of({0.0, 10.0}), (fts::TileIndex{1, 0}));
  EXPECT_EQ(g.index_of({-10.01, 0.0}), (fts::TileIndex{0, -1}));
}

TEST(TileGrid, CenterAndBoundsAreConsistentWithIndex)
{
  const fts::TileGrid g(20.0);
  const fts::TileIndex t{-2, 3};
  const Eigen::Vector2d c = g.center_of(t);
  EXPECT_DOUBLE_EQ(c.x(), 60.0);
  EXPECT_DOUBLE_EQ(c.y(), -40.0);
  // O centro de um tile indexa para o próprio tile.
  EXPECT_EQ(g.index_of(c), t);
  Eigen::Vector2d mn, mx;
  g.bounds_of(t, mn, mx);
  EXPECT_DOUBLE_EQ(mn.x(), 50.0);
  EXPECT_DOUBLE_EQ(mx.x(), 70.0);
  EXPECT_DOUBLE_EQ(mn.y(), -50.0);
  EXPECT_DOUBLE_EQ(mx.y(), -30.0);
  // Qualquer ponto dentro dos limites indexa para o tile.
  EXPECT_TRUE(g.contains(t, {55.0, -45.0}));
  EXPECT_FALSE(g.contains(t, {75.0, -45.0}));
}

TEST(TileGrid, EveryPointBelongsToExactlyOneTile)
{
  const fts::TileGrid g(7.5);  // tamanho não-redondo para apanhar erros de floor
  for (double x = -30.0; x <= 30.0; x += 1.3) {
    for (double y = -30.0; y <= 30.0; y += 1.7) {
      const fts::TileIndex t = g.index_of({x, y});
      Eigen::Vector2d mn, mx;
      g.bounds_of(t, mn, mx);
      EXPECT_GE(x, mn.x());
      EXPECT_LT(x, mx.x());
      EXPECT_GE(y, mn.y());
      EXPECT_LT(y, mx.y());
    }
  }
}

TEST(TileGrid, TilesInRadiusIsExactCircleSquareIntersection)
{
  const fts::TileGrid g(20.0);
  // Raio pequeno no centro do tile (0,0): só o próprio.
  auto tiles = g.tiles_in_radius({0.0, 0.0}, 5.0);
  ASSERT_EQ(tiles.size(), 1u);
  EXPECT_EQ(tiles[0], (fts::TileIndex{0, 0}));

  // Centro junto à fronteira direita: o círculo entra no vizinho (0,1) mas
  // NÃO chega aos diagonais (canto a ~sqrt(1^2+9^2) > 5 do ponto (9,1)... o
  // canto (10,10) está a norm(1,9)=9.06 > 5).
  tiles = g.tiles_in_radius({9.0, 1.0}, 5.0);
  ASSERT_EQ(tiles.size(), 2u);

  // Raio a cobrir o anel 3x3 completo a partir do centro.
  tiles = g.tiles_in_radius({0.0, 0.0}, 15.0);
  EXPECT_EQ(tiles.size(), 9u);
}

// --------------------------------------------------------- LandmarkTileMap --

TEST(LandmarkTileMap, AssignBucketsAndReportsChanges)
{
  fts::LandmarkTileMap m(20.0);
  EXPECT_TRUE(m.assign(7, {1.0, 1.0}));    // novo -> mudou
  EXPECT_FALSE(m.assign(7, {2.0, 3.0}));   // mesmo tile -> não mudou
  EXPECT_TRUE(m.assign(7, {25.0, 3.0}));   // atravessou a fronteira -> mudou
  EXPECT_EQ(m.tile_of(7), (fts::TileIndex{0, 1}));
  EXPECT_EQ(m.n_landmarks(), 1u);
  EXPECT_EQ(m.n_tiles(), 1u);  // o tile antigo esvaziou e foi removido
}

TEST(LandmarkTileMap, LoopClosureRebucketMovesUidNotIdentity)
{
  // Cenário: 3 landmarks; o loop closure corrige as posições e um deles
  // atravessa a fronteira do tile. O uid mantém-se; só o bucket muda.
  fts::LandmarkTileMap m(20.0);
  m.assign(1, {5.0, 5.0});      // tile (0,0)
  m.assign(2, {9.5, 0.0});      // tile (0,0), rente à fronteira
  m.assign(3, {28.0, 0.0});     // tile (0,1) — x=30 seria fronteira (0,2)

  // Posições pós-otimização: o 2 foi corrigido +1.0 m em x -> muda de tile.
  const auto optimized = [](fts::LandmarkUid uid) -> Eigen::Vector2d {
      switch (uid) {
        case 1: return {5.2, 4.9};    // continua no (0,0)
        case 2: return {10.5, 0.1};   // atravessou para o (0,1)
        default: return {29.8, 0.3};  // continua no (0,1)
      }
    };
  const std::size_t moved = m.rebucket(optimized);
  EXPECT_EQ(moved, 1u);
  EXPECT_EQ(m.tile_of(1), (fts::TileIndex{0, 0}));
  EXPECT_EQ(m.tile_of(2), (fts::TileIndex{0, 1}));
  EXPECT_EQ(m.tile_of(3), (fts::TileIndex{0, 1}));
  // O uid 2 continua a ser o uid 2 — nenhum landmark nasceu nem morreu.
  EXPECT_EQ(m.n_landmarks(), 3u);
  const auto in_01 = m.uids_in(fts::TileIndex{0, 1});
  EXPECT_EQ(in_01.size(), 2u);
}

TEST(LandmarkTileMap, RadiusQueryReturnsNeighbourhoodUids)
{
  fts::LandmarkTileMap m(20.0);
  m.assign(1, {0.0, 0.0});
  m.assign(2, {25.0, 0.0});    // tile à direita
  m.assign(3, {100.0, 100.0});  // longe

  // Raio 30 a partir da origem apanha os tiles (0,0) e (0,1) mas não o longínquo.
  const auto near = m.uids_in_radius({0.0, 0.0}, 30.0);
  EXPECT_EQ(near.size(), 2u);
  const auto all = m.uids_in_radius({50.0, 50.0}, 200.0);
  EXPECT_EQ(all.size(), 3u);
}

TEST(LandmarkTileMap, RemoveErasesUidAndEmptyTiles)
{
  fts::LandmarkTileMap m(20.0);
  m.assign(1, {0.0, 0.0});
  m.assign(2, {25.0, 0.0});
  m.remove(1);
  EXPECT_FALSE(m.has(1));
  EXPECT_EQ(m.n_landmarks(), 1u);
  EXPECT_EQ(m.n_tiles(), 1u);
  m.remove(42);  // remover uid inexistente é no-op
  EXPECT_EQ(m.n_landmarks(), 1u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
