#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace forest_tree_slam
{

struct MultiviewDbhParams
{
  double voxel_size_m{0.02};
  int coverage_bins{36};
  double saturation_coverage{0.65};
  std::uint32_t min_inlier_frames{3};
  double saturation_max_diameter_var{0.0025};  // (5 cm σ)²
  std::size_t refit_min_points{12};
  double refit_max_radius_m{0.75};
  // Taubin APARADO (IRLS): o buffer é permanente e o Taubin algébrico é destruído
  // por uma minoria de pontos deslocados (frame com pose derivada / ramo). Pontos
  // a mais de trim_tol_m do círculo são ignorados no fit. Ancorado no prior (a
  // estimativa acumulada) p/ evitar que o least-squares inicial seja puxado pelos
  // outliers — sem o viés de raio grande do RANSAC em arcos parciais.
  double trim_tol_m{0.05};
};

struct DbhRefitResult
{
  bool valid{false};
  double cx{0.0};
  double cy{0.0};
  double radius{0.0};
  float arc_coverage{0.0F};
  double rmse{0.0};
};

/** Buffer de inliers multi-view por landmark (tronco) com voxel-dedup e re-fit preguiçoso. */
class MultiviewPointBuffer
{
public:
  explicit MultiviewPointBuffer(MultiviewDbhParams params = {});

  bool saturated() const {return saturated_;}
  double coverage_ratio() const;
  std::uint32_t n_inlier_frames() const {return n_inlier_frames_;}
  std::size_t n_voxels() const {return voxels_.size();}

  /** Insere inliers de um frame (map). Devolve true se houve pontos/bins novos. */
  bool insert_frame(
    const std::vector<Eigen::Vector3d> & points_map, const Eigen::Vector2d & landmark_xy,
    const Eigen::Vector2d & view_xy);

  /** Re-fit do DBH por Taubin APARADO. Se has_prior, ancora a seleção de inliers
   *  no círculo (prior_cx,prior_cy,prior_r) — robusto a um anel deslocado sem o
   *  viés do RANSAC; senão arranca do least-squares de todos os pontos. */
  DbhRefitResult refit(
    double prior_cx = 0.0, double prior_cy = 0.0, double prior_r = 0.0,
    bool has_prior = false) const;

  /** Marca saturado (pára de ingerir). MANTÉM os voxels acumulados para que points()
   *  continue a devolver a nuvem da referência — visualização no RViz e inspeção. */
  void saturate();

  void reset();

  std::vector<Eigen::Vector3d> points() const;

  bool should_saturate(double diameter_var) const;

private:
  std::uint64_t voxel_key(const Eigen::Vector3d & p) const;
  int view_coverage_bin(const Eigen::Vector2d & landmark_xy, const Eigen::Vector2d & view_xy) const;

  MultiviewDbhParams params_;
  std::unordered_map<std::uint64_t, Eigen::Vector3d> voxels_;
  std::uint64_t coverage_bits_{0};
  std::uint32_t n_inlier_frames_{0};
  bool saturated_{false};
};

bool fit_circle_taubin_xy(
  const std::vector<float> & xs, const std::vector<float> & ys, double & cx_out, double & cy_out,
  double & r_out);

float arc_coverage_from_xy(
  const std::vector<float> & xs, const std::vector<float> & ys, double cx, double cy);

}  // namespace forest_tree_slam
