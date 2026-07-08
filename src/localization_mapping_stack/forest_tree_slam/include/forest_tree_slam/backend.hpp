#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "forest_tree_slam/types.hpp"

namespace forest_tree_slam
{

struct BackendParams
{
  // Keyframes por distância/ângulo percorridos (§5.2).
  double keyframe_distance_m{0.75};
  double keyframe_angle_rad{0.35};   // ~20 deg

  // Ruído do prior na 1.ª pose (x,y,theta stddev).
  Eigen::Vector3d prior_pose_sigma{0.1, 0.1, 0.05};

  // Ruído default do between-odom quando o chamador não fornece covariância
  // (fallback; idealmente vem do EKF).
  Eigen::Vector3d default_odom_sigma{0.05, 0.05, 0.03};

  // Ruído default do bearing-range tronco quando a deteção não tem cov
  // (fallback grosseiro; normalmente derivado de TreeDetection::base_covariance).
  double default_bearing_sigma_rad{0.05};
  double default_range_sigma_m{0.15};

  // Ruído do fator de distância entre dois troncos vistos no mesmo scan
  // (rigidez de constelação, Tree-SLAM orchards).
  double constellation_distance_sigma_m{0.1};

  // Ruído do fator GPS fraco (usado só quando a cov reportada é boa).
  Eigen::Vector2d weak_gps_sigma{1.0, 1.0};

  // --- Kernel robusto (Passo 2) ---
  // Envolve os fatores SUJEITOS A OUTLIERS (bearing-range de tronco e rigidez
  // de constelação) num M-estimador de Huber. Sem isto, uma única observação
  // mal-associada (ex.: loop closure a um landmark errado) é uma restrição
  // DURA e arrasta toda a solução de golpe ("o loop closure parte tudo"). O
  // odom/prior/GPS ficam Gaussianos (não são a fonte de outliers; robustecê-los
  // arriscaria ignorar movimento real). `huber_k` em unidades do erro
  // normalizado (1.345 = 95% de eficiência sob ruído Gaussiano).
  bool use_robust_kernels{true};
  double robust_huber_k{1.345};
};

// Wrapper do back-end iSAM2 SE(2): poses-keyframe X_i + landmarks de tronco L_k
// (FOREST_TREE_SLAM_DESIGN.md §5.2). Não conhece ROS; o nó traduz mensagens.
class TreeSlamBackend
{
public:
  explicit TreeSlamBackend(BackendParams params = {});

  // Inicializa o grafo com um prior na pose inicial (chamar uma vez).
  void initialize(const Pose2 & initial_pose);

  bool initialized() const {return initialized_;}

  // Acrescenta uma keyframe ligada à anterior por um fator between-odom.
  // `relative_pose` é o delta SE(2) acumulado desde a última keyframe;
  // `sigma` é o desvio-padrão (x,y,theta) desse delta (do EKF). Devolve o
  // índice da nova keyframe.
  std::size_t add_odom_keyframe(
    const Pose2 & relative_pose, const std::optional<Eigen::Vector3d> & sigma = std::nullopt);

  // Acrescenta a ARESTA SE2 de um salto aéreo (FOREST_TREE_SLAM_DESIGN.md §6):
  // o voo inteiro entra como UM between-factor de covariância grande entre a
  // keyframe pré-takeoff e a pós-aterragem.
  std::size_t add_aerial_hop_edge(const Pose2 & relative_pose, const Eigen::Vector3d & sigma);

  // Observação bearing-range tronco -> landmark (associado pelo tracker, uid
  // já conhecido). Cria o landmark se for a 1.ª vez que este uid aparece.
  void add_tree_observation(
    LandmarkUid uid, std::size_t keyframe_index, double bearing_rad, double range_m,
    const Eigen::Vector2d & initial_guess_world,
    std::optional<double> bearing_sigma_rad = std::nullopt,
    std::optional<double> range_sigma_m = std::nullopt);

  // Fator de rigidez de constelação entre dois troncos vistos no mesmo scan.
  void add_constellation_distance(LandmarkUid uid_a, LandmarkUid uid_b, double distance_m);

  // Fator GPS fraco (prior 2D) na keyframe corrente, quando a cov reportada é boa.
  void add_weak_gps_prior(
    std::size_t keyframe_index, const Eigen::Vector2d & xy_world, const Eigen::Vector2d & sigma);

  // Fator de relocalização/loop-closure: liga a keyframe corrente a um conjunto
  // de landmarks já no mapa (re-observação após TreeLoc aceitar o match).
  // Equivale a `add_tree_observation` mas documenta a intenção (cross-view,
  // arXiv:2409.16680) e é chamado pelo relocalizador, não pelo tracker normal.
  void add_relocalization_factor(
    LandmarkUid uid, std::size_t keyframe_index, double bearing_rad, double range_m);

  // Prior de posição absoluta num landmark (Fase 3). Injeta a posição do ajuste de
  // cilindro à nuvem multi-vista, que é mais fiável que a triangulação bearing×range
  // a >8 m (mal-condicionada). Só atua em landmarks já no grafo.
  void add_landmark_position_prior(
    LandmarkUid uid, const Eigen::Vector2d & xy_world, const Eigen::Vector2d & sigma);

  // Corre uma iteração incremental do iSAM2 com os fatores/valores acumulados
  // desde a última chamada. Devolve false se não havia nada novo.
  bool optimize();

  // --- Acesso ao estado otimizado --------------------------------------
  Pose2 keyframe_pose(std::size_t index) const;
  std::size_t n_keyframes() const {return n_keyframes_;}
  bool has_landmark(LandmarkUid uid) const {return landmark_keys_.count(uid) > 0;}
  Eigen::Vector2d landmark_position(LandmarkUid uid) const;
  Eigen::Matrix3d landmark_covariance(LandmarkUid uid) const;  // (x,y,_) só xy preenchido
  std::vector<LandmarkUid> all_landmark_uids() const;

  // Decide se acumula odom ou abre keyframe nova, dado o delta desde a última
  // keyframe. Uso típico: chamador integra Δodom; quando isto devolve true,
  // chama `add_odom_keyframe` com o delta acumulado e reseta o acumulador.
  bool should_open_keyframe(const Pose2 & accumulated_delta) const;

private:
  gtsam::Key pose_key(std::size_t index) const {return gtsam::Symbol('x', index);}
  gtsam::Key landmark_key(LandmarkUid uid) const;

  // Envolve um modelo de ruído num M-estimador de Huber (se ativo); senão
  // devolve o próprio. Aplica-se aos fatores sujeitos a outliers de associação.
  gtsam::SharedNoiseModel robustify(const gtsam::SharedNoiseModel & base) const;

  BackendParams params_;
  bool initialized_{false};
  std::size_t n_keyframes_{0};

  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph new_factors_;
  gtsam::Values new_values_;
  gtsam::Values current_estimate_;

  std::map<LandmarkUid, gtsam::Key> landmark_keys_;  // só para `has_landmark`/iteração
  bool dirty_{false};
};

}  // namespace forest_tree_slam
