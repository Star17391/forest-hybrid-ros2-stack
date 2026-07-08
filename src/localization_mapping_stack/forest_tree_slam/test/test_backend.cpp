#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "forest_tree_slam/backend.hpp"

using forest_tree_slam::Pose2;
using forest_tree_slam::TreeSlamBackend;

namespace
{
double wrap_angle(double a)
{
  while (a > M_PI) {a -= 2 * M_PI;}
  while (a < -M_PI) {a += 2 * M_PI;}
  return a;
}

// bearing-range exatos (sem ruído) de uma pose para um ponto landmark — usados
// para alimentar o grafo com observações geometricamente consistentes.
void bearing_range(
  const Pose2 & pose, const Eigen::Vector2d & landmark, double & bearing,
  double & range)
{
  const double dx = landmark.x() - pose.x;
  const double dy = landmark.y() - pose.y;
  range = std::hypot(dx, dy);
  bearing = wrap_angle(std::atan2(dy, dx) - pose.theta);
}

double pose_error(const Pose2 & a, const Pose2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
}  // namespace

TEST(Backend, OdomChainMatchesComposedPose)
{
  TreeSlamBackend backend;
  backend.initialize(Pose2{0, 0, 0});
  backend.add_odom_keyframe(Pose2{1.0, 0.0, 0.0});
  backend.add_odom_keyframe(Pose2{1.0, 0.0, M_PI_2});
  backend.add_odom_keyframe(Pose2{1.0, 0.0, 0.0});
  backend.optimize();

  // (0,0,0) -> +1x -> (1,0,0) -> +1x,rot90 -> (2,0,90deg) -> +1x no frame
  // local (que agora é +y global) -> (2,1,90deg)
  const Pose2 p3 = backend.keyframe_pose(3);
  EXPECT_NEAR(p3.x, 2.0, 1e-3);
  EXPECT_NEAR(p3.y, 1.0, 1e-3);
  EXPECT_NEAR(wrap_angle(p3.theta - M_PI_2), 0.0, 1e-3);
}

TEST(Backend, BearingRangeObservationsConvergeLandmarkToTruePosition)
{
  TreeSlamBackend backend;
  backend.initialize(Pose2{0, 0, 0});

  const Eigen::Vector2d true_landmark(3.0, 1.0);
  double b0, r0;
  bearing_range(Pose2{0, 0, 0}, true_landmark, b0, r0);
  backend.add_tree_observation(42, 0, b0, r0, true_landmark + Eigen::Vector2d(0.5, -0.5));

  backend.add_odom_keyframe(Pose2{1.0, 0.0, 0.0});
  double b1, r1;
  bearing_range(Pose2{1.0, 0.0, 0.0}, true_landmark, b1, r1);
  backend.add_tree_observation(42, 1, b1, r1, true_landmark);

  backend.optimize();

  const Eigen::Vector2d est = backend.landmark_position(42);
  EXPECT_NEAR((est - true_landmark).norm(), 0.0, 1e-2);
}

// Fase 3: o prior de posição da nuvem multi-vista deve dominar uma triangulação
// bearing×range ruidosa (mal-condicionada a longa distância) e puxar o landmark
// para a posição da nuvem.
TEST(Backend, MultiviewPositionPriorDominatesNoisyBearingRange)
{
  TreeSlamBackend backend;
  backend.initialize(Pose2{0, 0, 0});

  // Landmark longe (12 m), observado de uma única pose com bearing×range — a
  // posição triangulada é frouxa. Sem mais vistas, fica perto do guess inicial.
  const Eigen::Vector2d true_landmark(12.0, 0.0);
  double b, r;
  bearing_range(Pose2{0, 0, 0}, true_landmark, b, r);
  // Guess inicial deslocado 1.5 m (como um bearing×range ruidoso a 12 m).
  backend.add_tree_observation(7, 0, b, r, true_landmark + Eigen::Vector2d(1.2, 0.9));
  backend.optimize();
  const Eigen::Vector2d before = backend.landmark_position(7);

  // A nuvem multi-vista dá a posição verdadeira com cov pequena → prior forte.
  backend.add_landmark_position_prior(7, true_landmark, Eigen::Vector2d(0.08, 0.08));
  backend.optimize();
  const Eigen::Vector2d after = backend.landmark_position(7);

  EXPECT_LT((after - true_landmark).norm(), (before - true_landmark).norm());
  EXPECT_NEAR((after - true_landmark).norm(), 0.0, 0.15);
}

// Gate (a) do design: ATE 2D vs GT melhor que só-EKF (sem loop). Simula um
// odom com bias sistemático (drift) e confirma que as observações de tronco
// corrigem a trajetória para mais perto da verdade-terreno do que o odom puro.
TEST(Backend, TreeObservationsImproveAteOverRawOdom)
{
  // Trajetória GT: 5 passos de 1m em linha reta; odom tem bias de +5% por passo.
  std::vector<Pose2> gt_poses = {{0, 0, 0}};
  for (int i = 1; i <= 5; ++i) {
    gt_poses.push_back(Pose2{static_cast<double>(i), 0.0, 0.0});
  }
  const std::vector<Eigen::Vector2d> landmarks = {
    Eigen::Vector2d(2.0, 3.0), Eigen::Vector2d(4.0, -3.0)};

  TreeSlamBackend backend;
  backend.initialize(gt_poses[0]);

  std::vector<Pose2> raw_odom_poses = {gt_poses[0]};
  for (std::size_t i = 1; i < gt_poses.size(); ++i) {
    const Pose2 biased_delta{1.05, 0.0, 0.0};  // odom "mede" 1.05m em vez de 1.0m
    backend.add_odom_keyframe(biased_delta);
    raw_odom_poses.push_back(
      Pose2{raw_odom_poses.back().x + biased_delta.x, raw_odom_poses.back().y, 0.0});

    for (std::size_t l = 0; l < landmarks.size(); ++l) {
      double b, r;
      bearing_range(gt_poses[i], landmarks[l], b, r);
      backend.add_tree_observation(100 + l, i, b, r, landmarks[l] + Eigen::Vector2d(0.3, 0.3));
    }
  }
  backend.optimize();

  double ate_raw = 0.0, ate_slam = 0.0;
  for (std::size_t i = 0; i < gt_poses.size(); ++i) {
    ate_raw += pose_error(raw_odom_poses[i], gt_poses[i]);
    ate_slam += pose_error(backend.keyframe_pose(i), gt_poses[i]);
  }
  ate_raw /= static_cast<double>(gt_poses.size());
  ate_slam /= static_cast<double>(gt_poses.size());

  EXPECT_LT(ate_slam, ate_raw);
  EXPECT_LT(ate_slam, 0.1);
}

// Passo 2: kernel robusto (Huber). Constrói um landmark bem fixado por várias
// observações consistentes, depois injeta UMA observação outlier (re-associação
// errada). COM Huber o outlier é atenuado e o landmark quase não se mexe; SEM
// Huber (Gaussiano puro) arrasta de golpe ("o loop closure parte tudo"). O
// mesmo cenário nos dois modos prova que o kernel é a diferença.
namespace
{
double outlier_shift_with(bool use_robust)
{
  forest_tree_slam::BackendParams params;
  params.use_robust_kernels = use_robust;
  TreeSlamBackend backend(params);
  backend.initialize(Pose2{0, 0, 0});

  const Eigen::Vector2d true_landmark(3.0, 0.0);
  double b, r;
  // 4 observações CONSISTENTES de 4 keyframes (landmark bem fixado).
  bearing_range(Pose2{0, 0, 0}, true_landmark, b, r);
  backend.add_tree_observation(7, 0, b, r, true_landmark);
  for (int i = 1; i <= 3; ++i) {
    backend.add_odom_keyframe(Pose2{1.0, 0.0, 0.0});
    bearing_range(Pose2{static_cast<double>(i), 0, 0}, true_landmark, b, r);
    backend.add_tree_observation(7, static_cast<std::size_t>(i), b, r, true_landmark);
  }
  backend.optimize();
  const Eigen::Vector2d before = backend.landmark_position(7);

  // OUTLIER: re-observa o MESMO landmark do último kf como se estivesse a 3 m
  // de lado (bearing 90°, range 3 m). É uma restrição grosseiramente errada.
  backend.add_tree_observation(7, backend.n_keyframes() - 1, M_PI_2, 3.0, true_landmark);
  backend.optimize();
  return (backend.landmark_position(7) - before).norm();
}
}  // namespace

TEST(Backend, RobustKernelLimitsOutlierDragVsGaussian)
{
  const double shift_gauss = outlier_shift_with(false);
  const double shift_robust = outlier_shift_with(true);

  // Sem kernel: o outlier arrasta muito (documenta o "parte tudo").
  EXPECT_GT(shift_gauss, 0.3) << "Gaussiano puro: outlier arrasta " << shift_gauss << " m";
  // Com Huber: o outlier é atenuado -> deslocamento pequeno.
  EXPECT_LT(shift_robust, 0.1) << "Huber: outlier atenuado, desloca " << shift_robust << " m";
  // E claramente melhor que o Gaussiano.
  EXPECT_LT(shift_robust, shift_gauss * 0.5);
}

// Cov POR-MEDIÇÃO: o nó passa agora range_sigma dependente do alcance (antes
// deitava a cov fora e o backend usava sempre 15 cm fixos). Prova que o canal
// funciona: uma observação longe ruidosa injetada com sigma GRANDE arrasta menos
// a pose/landmark do que a MESMA observação injetada com sigma pequeno (otimista).
TEST(Backend, PerMeasurementRangeSigmaAttenuatesNoisyObservation)
{
  auto landmark_shift_with = [](double range_sigma) {
      TreeSlamBackend backend;
      backend.initialize(Pose2{0, 0, 0});
      const Eigen::Vector2d true_lm(3.0, 0.0);
      double b, r;
      // 3 observações consistentes e próximas fixam o landmark.
      bearing_range(Pose2{0, 0, 0}, true_lm, b, r);
      backend.add_tree_observation(5, 0, b, r, true_lm, 0.05, 0.10);
      for (int i = 1; i <= 2; ++i) {
        backend.add_odom_keyframe(Pose2{1.0, 0.0, 0.0});
        bearing_range(Pose2{static_cast<double>(i), 0, 0}, true_lm, b, r);
        backend.add_tree_observation(5, static_cast<std::size_t>(i), b, r, true_lm, 0.05, 0.10);
      }
      backend.optimize();
      const Eigen::Vector2d before = backend.landmark_position(5);
      // Observação RUIDOSA do mesmo landmark (deslocada 1 m no alcance) injetada
      // com o sigma de alcance dado.
      backend.add_tree_observation(
        5, backend.n_keyframes() - 1, b, r + 1.0, true_lm, 0.05, range_sigma);
      backend.optimize();
      return (backend.landmark_position(5) - before).norm();
    };

  const double shift_tight = landmark_shift_with(0.10);  // sigma otimista
  const double shift_loose = landmark_shift_with(0.60);  // sigma realista p/ longe
  EXPECT_LT(shift_loose, shift_tight)
    << "sigma grande devia atenuar a observação ruidosa (loose=" << shift_loose
    << " tight=" << shift_tight << ")";
}

// Gate (c): após a aresta SE2 do salto (com prior errado/grande incerteza),
// re-observar troncos já conhecidos do mapa deve reconvergir a pose para a
// verdade-terreno (a relocalização "fecha o erro", design §6).
TEST(Backend, ReobservingKnownLandmarksAfterAerialHopReconvergesPose)
{
  TreeSlamBackend backend;
  backend.initialize(Pose2{0, 0, 0});

  const Eigen::Vector2d lm_a(2.0, 2.0), lm_b(2.0, -2.0);
  double b, r;
  bearing_range(Pose2{0, 0, 0}, lm_a, b, r);
  backend.add_tree_observation(1, 0, b, r, lm_a);
  bearing_range(Pose2{0, 0, 0}, lm_b, b, r);
  backend.add_tree_observation(2, 0, b, r, lm_b);
  backend.optimize();

  // Salto aéreo: aresta SE2 com erro grande (devia aterrar em (0,5,0) mas a
  // aresta diz (0,5.8,0.1) — erro tipico de GNSS sob dossel) e cov GRANDE.
  const Pose2 true_landing{0.0, 5.0, 0.0};
  const Pose2 noisy_hop_edge{0.0, 5.8, 0.1};
  backend.add_aerial_hop_edge(noisy_hop_edge, Eigen::Vector3d(3.0, 3.0, 0.5));
  backend.optimize();
  const std::size_t landing_kf = backend.n_keyframes() - 1;

  // Pose pós-salto (sem reloc) ainda reflete o erro do salto.
  const Pose2 before_reloc = backend.keyframe_pose(landing_kf);
  EXPECT_GT(pose_error(before_reloc, true_landing), 0.3);

  // Relocalização: re-observa os MESMOS troncos do mapa a partir da pose
  // verdadeira de aterragem -> fatores de loop closure corrigem o grafo.
  bearing_range(true_landing, lm_a, b, r);
  backend.add_relocalization_factor(1, landing_kf, b, r);
  bearing_range(true_landing, lm_b, b, r);
  backend.add_relocalization_factor(2, landing_kf, b, r);
  backend.optimize();

  const Pose2 after_reloc = backend.keyframe_pose(landing_kf);
  EXPECT_LT(pose_error(after_reloc, true_landing), 0.1);
}
