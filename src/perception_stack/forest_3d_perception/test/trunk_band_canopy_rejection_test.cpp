// Regressão: o cluster de uma árvore é DOMINADO pela copa (um LiDAR inclinado para
// cima vê muito mais copa que tronco). O bug histórico: z_base = percentil-10 de z
// caía DENTRO da copa, a banda do DBH deslocava-se para cima e o DBH explodia
// (medido: tronco de 0.66 m lido como 3.05 m, banda 100% copa).
//
// A correção (âncora no eixo do tronco em cylinder_fit.hpp) tem de:
//   [A] não deixar NENHUM ponto de copa entrar na banda usada para o DBH,
//   [B] recuperar um DBH são (~tronco), e
//   [C] aguentar copa BAIXA colada ao tronco (caso Tree5).
//
// Sem ROS/sim. Exit 0 = passou. `colcon test --packages-select forest_3d_perception`.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "forest_3d_perception/cylinder_fit.hpp"

using namespace forest_3d_perception;

namespace
{
// Constrói tronco fino (semicírculo virado ao sensor) + copa larga densa.
// Devolve a nuvem e marca em is_canopy[] que pontos são copa (ground-truth).
void build_tree(
  pcl::PointCloud<pcl::PointXYZ> & cloud, std::vector<bool> & is_canopy,
  double trunk_r, double trunk_top, double canopy_base, double canopy_r,
  double canopy_top, int canopy_pts_per_ring)
{
  const double cx = 5.0, cy = 0.0;
  // Tronco: semicírculo virado ao sensor (na origem), esparso como um LiDAR real.
  for (double z = 0.05; z <= trunk_top; z += 0.12) {
    for (int i = 0; i < 5; ++i) {
      const double a = M_PI - M_PI / 2 + M_PI * i / 4;
      cloud.push_back({static_cast<float>(cx + trunk_r * std::cos(a)),
                       static_cast<float>(cy + trunk_r * std::sin(a)),
                       static_cast<float>(z)});
      is_canopy.push_back(false);
    }
  }
  // Copa: blob largo e DENSO acima (domina a contagem de pontos).
  for (double z = canopy_base; z <= canopy_top; z += 0.15) {
    const double rr = canopy_r * std::sin(M_PI * (z - canopy_base) /
                                          (canopy_top - canopy_base) * 0.9 + 0.1);
    for (int i = 0; i < canopy_pts_per_ring; ++i) {
      const double a = M_PI - M_PI / 2 + M_PI * i / (canopy_pts_per_ring - 1);
      cloud.push_back({static_cast<float>(cx + rr * std::cos(a)),
                       static_cast<float>(cy + rr * std::sin(a)),
                       static_cast<float>(z)});
      is_canopy.push_back(true);
    }
  }
}

// Corre o fit real e conta pontos de copa na banda. Params = config real do node.
int run_case(const char * name, double trunk_r, double trunk_top,
             double canopy_base, double canopy_r, double canopy_top, int cpr)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<bool> is_canopy;
  build_tree(cloud, is_canopy, trunk_r, trunk_top, canopy_base, canopy_r,
             canopy_top, cpr);

  std::vector<std::size_t> idx(cloud.size());
  for (std::size_t i = 0; i < idx.size(); ++i) { idx[i] = i; }

  std::vector<std::size_t> band;
  CylinderObservation cyl;
  const auto rej = fit_vertical_cylinder(
    cloud, idx, cyl, 0.30, 0.80, 0.05, 0.30, 0.04, 2.50, 0.15, 2.50, 1.8, 0.20,
    &band);

  std::size_t n_canopy = 0;
  for (std::size_t bi : band) {
    if (bi < is_canopy.size() && is_canopy[bi]) { ++n_canopy; }
  }
  const std::size_t n_trunk = cloud.size() - std::count(is_canopy.begin(),
                                                        is_canopy.end(), false);
  (void)n_trunk;
  const double dbh = 2.0 * cyl.radius;
  std::printf(
    "  %-22s n=%zu (copa-domina) | rej=%d Ø=%.3f (GT %.3f) banda=%zu copa-na-banda=%zu\n",
    name, cloud.size(), static_cast<int>(rej), dbh, 2 * trunk_r, band.size(),
    n_canopy);

  int fails = 0;
  // [A] ZERO copa na banda — o requisito central.
  if (n_canopy != 0) {
    std::printf("    FALHOU [A]: %zu pontos de copa contaminaram a banda do DBH\n",
                n_canopy);
    ++fails;
  }
  // [B] DBH são (não inflado pela copa). Tolerância larga: viés de arco parcial.
  if (rej == CylinderReject::Accepted && std::abs(dbh - 2 * trunk_r) > 0.25) {
    std::printf("    FALHOU [B]: DBH %.3f longe do tronco real %.3f\n", dbh,
                2 * trunk_r);
    ++fails;
  }
  return fails;
}
}  // namespace

int main()
{
  int fails = 0;
  std::printf("[1] copa ALTA dominante (tronco 0..3 m, copa 3..8 m, densa):\n");
  // ~125 pontos de tronco vs ~1500 de copa -> percentil-10 de z cairia na copa.
  fails += run_case("copa alta densa", 0.18, 3.0, 3.0, 1.5, 8.0, 50);

  std::printf("[2] copa BAIXA colada ao tronco (tronco fino 0..1.6, copa 1.5..6):\n");
  // Caso Tree5: a copa começa quase à altura do peito.
  fails += run_case("copa baixa (Tree5)", 0.13, 1.6, 1.5, 1.2, 6.0, 60);

  std::printf("[3] copa MUITO densa e larga (stress):\n");
  fails += run_case("copa enorme", 0.15, 2.5, 2.5, 2.5, 9.0, 80);

  std::printf("\n%s (%d falhas)\n", fails == 0 ? "TODOS PASSARAM" : "HOUVE FALHAS",
              fails);
  return fails;
}
