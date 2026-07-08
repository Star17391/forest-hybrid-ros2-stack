#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "forest_tree_slam/types.hpp"

namespace forest_tree_slam
{

// Um tronco observado/no mapa, com identidade conhecida (uid) quando vem do
// mapa, ou desconhecida (uid=0) quando vem da observação local a relocalizar.
struct LandmarkPoint
{
  LandmarkUid uid{0};
  double x{0.0};
  double y{0.0};
  double diameter{0.0};
};

// Correspondência aceite entre uma deteção local (índice em `query`) e um
// landmark do mapa (uid).
struct ReloCorrespondence
{
  std::size_t query_index{0};
  LandmarkUid map_uid{0};
};

struct RelocalizationResult
{
  bool accepted{false};
  Pose2 map_to_query_transform{};   // transforma pontos do frame query -> map
  double overlap_ratio{0.0};
  double mean_residual_m{0.0};
  std::vector<ReloCorrespondence> correspondences;
};

struct RelocalizerParams
{
  // --- TDH (coarse), TreeLoc §coarse ---
  int n_radial_bins{5};
  int n_diameter_bins{8};
  double radial_bin_max_m{15.0};
  double diameter_bin_max_m{1.5};
  int top_n_coarse{8};            // top-N candidatos por chi-quadrado (paper: top-100)

  // --- Triângulos (fine), TreeLoc §fine ---
  double triangle_side_tolerance_m{0.5};
  int top_n_fine{4};              // top-N por nº de triângulos partilhados (paper: top-10)
  double min_triangle_side_m{0.5};  // ignora triângulos degenerados (troncos quase coincidentes)

  // --- Verificação geométrica + supressão de outliers (TreeLoc++) ---
  double planar_residual_threshold_m{0.4};
  double diameter_residual_threshold_m{0.2};
  double min_overlap_ratio{0.6};
  int min_correspondences{5};

  // --- Anti-falso-match (guarda nº5 do plano de robustez) ---
  // Só aceitar se a melhor hipótese bater a 2.ª melhor DISTINTA por uma margem
  // clara de inliers. Em floresta auto-semelhante, duas transformações
  // diferentes com apoio quase igual = match ambíguo → recusar (um match errado
  // é catastrófico: fator ao uid errado puxa o grafo inteiro).
  int accept_margin_inliers{2};
  // Hipóteses contam como DISTINTAS se diferirem mais do que isto (senão são o
  // mesmo cluster da hipótese certa a competir consigo próprio).
  double distinct_transform_translation_m{0.5};
  double distinct_transform_rotation_rad{0.15};
};

// Relocalizador = loop closure unificado (TDH coarse + triângulo fine +
// verificação SVD + supressão de outliers). FOREST_TREE_SLAM_DESIGN.md §5.3.
// Não conhece ROS nem GTSAM: opera em pontos 2D + DBH; o nó traduz o
// resultado em fatores do backend (`add_relocalization_factor`).
class TreeLocRelocalizer
{
public:
  explicit TreeLocRelocalizer(RelocalizerParams params = {})
  : params_(params)
  {
  }

  // `query`: troncos observados localmente (frame do robô na pose corrente,
  // ou já um "prior frame" se houver pose do autopilot — ver mode_manager).
  // `map`: todo o mapa de landmarks conhecido (`/slam/tree_map`).
  RelocalizationResult relocalize(
    const std::vector<LandmarkPoint> & query, const std::vector<LandmarkPoint> & map) const;

  // Melhor conjunto de inliers encontrado na última chamada a `relocalize`,
  // mesmo quando o resultado final foi rejeitado (overlap/correspondências
  // insuficientes) — útil para diagnóstico de porque uma tentativa falhou.
  const std::vector<ReloCorrespondence> & last_best_inliers() const {return last_best_inliers_;}

private:
  mutable std::vector<ReloCorrespondence> last_best_inliers_;

  using Descriptor = std::vector<double>;  // n_radial_bins * n_diameter_bins

  Descriptor compute_tdh(
    const std::vector<LandmarkPoint> & cluster, const std::vector<std::size_t> & members) const;

  double chi_square_distance(const Descriptor & a, const Descriptor & b) const;

  // Triângulo com lados ordenados (a<=b<=c) e os 3 índices originais na
  // mesma ordem dos lados (para reconstruir a correspondência).
  struct Triangle
  {
    std::array<double, 3> sides;       // ordenados
    std::array<std::size_t, 3> indices;  // correspondem aos vértices, MESMA ordem dos `sides`
  };

  std::vector<Triangle> build_triangles(
    const std::vector<LandmarkPoint> & points, const std::vector<std::size_t> & subset) const;

  RelocalizerParams params_;
};

}  // namespace forest_tree_slam
