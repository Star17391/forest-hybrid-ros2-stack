#pragma once

// Partição FIXA do mundo em tiles quadrados (FUTURE_TILED_MAPS.md).
//
// Decisões (fechadas com o utilizador, 2026-07):
//  - A grelha de tiles é FIXA por coordenadas: o tile (0,0) fica centrado na
//    origem do mundo (frame `map`) e os restantes em redor; a divisão existe
//    desde o arranque (é só aritmética — nenhum tile é alocado à cabeça).
//  - Identidade ≠ localização: o `uid` do landmark é imutável; o tile é um
//    ÍNDICE DERIVADO da posição otimizada. Quando o loop closure (iSAM2) move
//    um landmark, `rebucket()` verifica os limites por coordenada e move o
//    uid para o tile correto — o uid nunca muda.
//  - Vizinhança carregada POR RAIO (círculo de raio R), não "8 tiles fixos":
//    mais robusto na borda do tile.
//
// Este módulo é puro (sem ROS, sem GTSAM): recebe posições, devolve índices e
// buckets. O nó de SLAM alimenta-o com as posições otimizadas do backend; o
// mapeamento denso usa a mesma aritmética para os tiles de terreno.

#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <vector>

#include <Eigen/Core>

#include "forest_tree_slam/types.hpp"

namespace forest_tree_slam
{

// Índice inteiro (linha, coluna) de um tile na grelha fixa.
struct TileIndex
{
  std::int32_t r{0};
  std::int32_t c{0};

  bool operator==(const TileIndex & o) const {return r == o.r && c == o.c;}
  bool operator!=(const TileIndex & o) const {return !(*this == o);}
  bool operator<(const TileIndex & o) const
  {
    return r < o.r || (r == o.r && c < o.c);
  }
};

// Aritmética da grelha fixa. O tile (0,0) é o quadrado CENTRADO na origem do
// mundo: cobre [-size/2, +size/2[ em x e y; os vizinhos seguem por translação.
class TileGrid
{
public:
  explicit TileGrid(double tile_size_m)
  : size_(tile_size_m) {}

  double tile_size() const {return size_;}

  // Índice do tile que contém a posição (fronteira: pertence ao tile seguinte,
  // intervalo semiaberto [min, max[ — cada ponto tem exatamente UM tile).
  TileIndex index_of(const Eigen::Vector2d & pos) const
  {
    return TileIndex{
      static_cast<std::int32_t>(std::floor(pos.y() / size_ + 0.5)),
      static_cast<std::int32_t>(std::floor(pos.x() / size_ + 0.5))};
  }

  // Centro do tile em coordenadas do mundo.
  Eigen::Vector2d center_of(const TileIndex & t) const
  {
    return {static_cast<double>(t.c) * size_, static_cast<double>(t.r) * size_};
  }

  // Limites [min, max[ do tile.
  void bounds_of(
    const TileIndex & t, Eigen::Vector2d & min_out, Eigen::Vector2d & max_out) const
  {
    const Eigen::Vector2d c = center_of(t);
    min_out = c - Eigen::Vector2d(size_ / 2.0, size_ / 2.0);
    max_out = c + Eigen::Vector2d(size_ / 2.0, size_ / 2.0);
  }

  bool contains(const TileIndex & t, const Eigen::Vector2d & pos) const
  {
    return index_of(pos) == t;
  }

  // Todos os tiles que INTERSECTAM o círculo (centro, raio) — a "vizinhança
  // ativa" carregada por raio. Interseção exata círculo×quadrado (ponto do
  // quadrado mais próximo do centro), não só os centros.
  std::vector<TileIndex> tiles_in_radius(
    const Eigen::Vector2d & center, double radius_m) const
  {
    std::vector<TileIndex> out;
    const TileIndex c0 = index_of(center);
    const auto span = static_cast<std::int32_t>(std::ceil(radius_m / size_)) + 1;
    for (std::int32_t dr = -span; dr <= span; ++dr) {
      for (std::int32_t dc = -span; dc <= span; ++dc) {
        const TileIndex t{c0.r + dr, c0.c + dc};
        Eigen::Vector2d mn, mx;
        bounds_of(t, mn, mx);
        const Eigen::Vector2d closest = center.cwiseMax(mn).cwiseMin(mx);
        if ((closest - center).norm() <= radius_m) {
          out.push_back(t);
        }
      }
    }
    return out;
  }

private:
  double size_;
};

// Bucketing de landmarks (uid -> tile) sobre a grelha fixa. O tile é derivado
// da posição OTIMIZADA (backend); `rebucket()` re-verifica após loop closure.
class LandmarkTileMap
{
public:
  explicit LandmarkTileMap(double tile_size_m)
  : grid_(tile_size_m) {}

  const TileGrid & grid() const {return grid_;}

  // Insere ou atualiza o landmark; devolve true se o tile MUDOU (ou é novo).
  bool assign(LandmarkUid uid, const Eigen::Vector2d & pos)
  {
    const TileIndex t = grid_.index_of(pos);
    auto it = uid_to_tile_.find(uid);
    if (it == uid_to_tile_.end()) {
      uid_to_tile_[uid] = t;
      tile_to_uids_[t].insert(uid);
      return true;
    }
    if (it->second == t) {
      return false;
    }
    tile_to_uids_[it->second].erase(uid);
    if (tile_to_uids_[it->second].empty()) {
      tile_to_uids_.erase(it->second);
    }
    it->second = t;
    tile_to_uids_[t].insert(uid);
    return true;
  }

  // Re-verifica TODOS os uids contra as posições otimizadas (pós-loop-closure).
  // `lookup` devolve a posição atual de cada uid. Devolve quantos mudaram de tile.
  std::size_t rebucket(
    const std::function<Eigen::Vector2d(LandmarkUid)> & lookup)
  {
    std::size_t moved = 0;
    // Copiar os uids primeiro: assign() muta os mapas.
    std::vector<LandmarkUid> uids;
    uids.reserve(uid_to_tile_.size());
    for (const auto & kv : uid_to_tile_) {
      uids.push_back(kv.first);
    }
    for (const LandmarkUid uid : uids) {
      if (assign(uid, lookup(uid))) {
        ++moved;
      }
    }
    return moved;
  }

  bool has(LandmarkUid uid) const {return uid_to_tile_.count(uid) > 0;}

  TileIndex tile_of(LandmarkUid uid) const {return uid_to_tile_.at(uid);}

  // uids no tile (vazio se o tile não tem landmarks).
  std::vector<LandmarkUid> uids_in(const TileIndex & t) const
  {
    auto it = tile_to_uids_.find(t);
    if (it == tile_to_uids_.end()) {
      return {};
    }
    return {it->second.begin(), it->second.end()};
  }

  // uids em todos os tiles da vizinhança por raio.
  std::vector<LandmarkUid> uids_in_radius(
    const Eigen::Vector2d & center, double radius_m) const
  {
    std::vector<LandmarkUid> out;
    for (const TileIndex & t : grid_.tiles_in_radius(center, radius_m)) {
      auto it = tile_to_uids_.find(t);
      if (it != tile_to_uids_.end()) {
        out.insert(out.end(), it->second.begin(), it->second.end());
      }
    }
    return out;
  }

  void remove(LandmarkUid uid)
  {
    auto it = uid_to_tile_.find(uid);
    if (it == uid_to_tile_.end()) {
      return;
    }
    tile_to_uids_[it->second].erase(uid);
    if (tile_to_uids_[it->second].empty()) {
      tile_to_uids_.erase(it->second);
    }
    uid_to_tile_.erase(it);
  }

  std::size_t n_landmarks() const {return uid_to_tile_.size();}
  std::size_t n_tiles() const {return tile_to_uids_.size();}

  // Tiles atualmente com landmarks (para viz/serialização).
  std::vector<TileIndex> occupied_tiles() const
  {
    std::vector<TileIndex> out;
    out.reserve(tile_to_uids_.size());
    for (const auto & kv : tile_to_uids_) {
      out.push_back(kv.first);
    }
    return out;
  }

private:
  TileGrid grid_;
  std::map<LandmarkUid, TileIndex> uid_to_tile_;
  std::map<TileIndex, std::set<LandmarkUid>> tile_to_uids_;
};

}  // namespace forest_tree_slam
