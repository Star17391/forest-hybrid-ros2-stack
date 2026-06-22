// Testes de DIAGNÓSTICO da cola do nó (tree_slam_node.cpp), fora de ROS.
//
// Objetivo: reproduzir de forma deterministra o fluxo de on_landmarks() —
// transformação base_link->mundo, tracker, abertura de keyframes, e fatores
// bearing/range no backend — para isolar QUAL dos problemas suspeitos degrada
// o tracking em modo GROUND, sem depender da latência/ruído da simulação.
//
// Candidatos sob teste:
//   (1) Observações ENTRE keyframes ancoradas à keyframe anterior, mas com
//       bearing/range calculados a partir da pose dead-reckoned atual
//       (descarta o Δodom keyframe->observação). tree_slam_node.cpp:234-263.
//   (2) Odometria não sincronizada com o stamp do scan (lag temporal).
//   (3) Tracker (frame-mundo absoluto) desacoplado da re-otimização do backend.
//
// Estratégia: odometria PERFEITA e SEM ruído de deteção => qualquer erro
// residual no mapa do backend / qualquer duplicação de uid é atribuível à
// lógica de cola, não a ruído. Para a Causa nº1 comparamos o caminho "como o
// nó faz" (BUGGY) contra o caminho geometricamente correto (CORRECT).

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <vector>

#include "forest_tree_slam/backend.hpp"
#include "forest_tree_slam/se2_geometry.hpp"
#include "forest_tree_slam/tracker.hpp"

using forest_tree_slam::BearingRange;
using forest_tree_slam::bearing_range_from;
using forest_tree_slam::between;
using forest_tree_slam::compose;
using forest_tree_slam::interpolate_pose;
using forest_tree_slam::LandmarkTracker;
using forest_tree_slam::LandmarkUid;
using forest_tree_slam::Pose2;
using forest_tree_slam::transform_point;
using forest_tree_slam::TreeDetection;
using forest_tree_slam::TreeSlamBackend;
using forest_tree_slam::wrap_angle;

namespace
{
// Geometria SE(2) partilhada com o nó vive em se2_geometry.hpp — o harness
// abaixo usa exatamente os mesmos helpers que tree_slam_node.cpp.

struct Tree
{
  double x, y, diameter;
};

// Coordenadas de uma árvore-mundo no frame base_link de um robô em `robot`.
void world_to_base(const Pose2 & robot, const Tree & t, double & bx, double & by)
{
  const double c = std::cos(robot.theta), s = std::sin(robot.theta);
  const double dx = t.x - robot.x, dy = t.y - robot.y;
  bx = c * dx + s * dy;
  by = -s * dx + c * dy;
}

// Resultado de uma corrida do harness.
struct RunResult
{
  std::set<LandmarkUid> distinct_uids;     // todos os uids alguma vez nascidos
  std::size_t n_births = 0;
  double max_landmark_error_m = 0.0;       // pior erro de um landmark do backend vs verdade
  double mean_landmark_error_m = 0.0;
};

// Parâmetros do cenário.
struct Scenario
{
  std::vector<Pose2> trajectory;     // pose-verdade do robô em cada scan
  std::vector<Tree> trees;           // árvores-verdade (mundo)
  double sensor_range_m = 12.0;      // só vê árvores dentro deste raio
  bool use_correct_bearing = false;  // false = como o nó faz (BUGGY); true = correto
  int odom_lag_scans = 0;            // Causa nº2: usa a odom de N scans atrás
};

// Reproduz o fluxo de on_landmarks()/on_odom() do nó para um cenário.
RunResult run_node_glue(const Scenario & sc)
{
  TreeSlamBackend backend;
  LandmarkTracker tracker;
  RunResult res;

  // on_odom (1ª): inicializa o grafo na origem e fixa a 1ª keyframe odom.
  backend.initialize(Pose2{0, 0, 0});
  backend.optimize();
  Pose2 last_keyframe_odom_pose = sc.trajectory.front();

  for (std::size_t k = 0; k < sc.trajectory.size(); ++k) {
    // Odometria que o nó "vê" neste scan. Com lag, usa uma pose anterior
    // (reproduz last_odom_pose_ estar atrasada face ao stamp do scan).
    const std::size_t odom_idx =
      (static_cast<int>(k) - sc.odom_lag_scans) > 0 ? k - sc.odom_lag_scans : 0;
    const Pose2 odom_now = sc.trajectory[odom_idx];
    const Pose2 true_now = sc.trajectory[k];  // pose REAL (onde as árvores são vistas)

    const Pose2 delta_since_kf = between(last_keyframe_odom_pose, odom_now);
    const Pose2 last_kf_pose = backend.keyframe_pose(backend.n_keyframes() - 1);
    const Pose2 predicted_world_pose = compose(last_kf_pose, delta_since_kf);

    // Deteções: árvores visíveis, em base_link (a partir da pose REAL), depois
    // transformadas para mundo via predicted_world_pose (igual ao nó:201-217).
    std::vector<TreeDetection> dets_world;
    std::vector<std::size_t> det_tree_idx;
    for (std::size_t ti = 0; ti < sc.trees.size(); ++ti) {
      double bx, by;
      world_to_base(true_now, sc.trees[ti], bx, by);
      if (std::hypot(bx, by) > sc.sensor_range_m) {
        continue;
      }
      const Eigen::Vector2d w = transform_point(predicted_world_pose, bx, by);
      TreeDetection d;
      d.x = w.x();
      d.y = w.y();
      d.diameter = sc.trees[ti].diameter;
      d.confidence = 0.9F;
      dets_world.push_back(d);
      det_tree_idx.push_back(ti);
    }

    const double ang_delta = 0.0;  // (predição do tracker; não relevante p/ isto)
    const auto report = tracker.update(
      dets_world, static_cast<double>(k),
      Eigen::Vector2d(predicted_world_pose.x, predicted_world_pose.y), ang_delta);
    for (auto u : report.births) {
      res.distinct_uids.insert(u);
      ++res.n_births;
    }

    // Keyframe nova? (igual ao nó:234-245)
    std::size_t obs_keyframe = backend.n_keyframes() - 1;
    Pose2 obs_pose = predicted_world_pose;
    bool opened_keyframe = false;
    if (backend.should_open_keyframe(delta_since_kf)) {
      obs_keyframe = backend.add_odom_keyframe(delta_since_kf);
      last_keyframe_odom_pose = odom_now;
      obs_pose = predicted_world_pose;
      opened_keyframe = true;
    }

    // Bearing/range. BUGGY (comportamento antigo): relativo à pose dead-reckoned.
    // CORRECT (o que o nó faz agora, tree_slam_node.cpp): relativo à pose da
    // keyframe de observação. Se a keyframe foi aberta agora, a sua pose ainda
    // não existe em Values; o estimate inicial == predicted (== obs_pose).
    Pose2 ref_pose = obs_pose;  // == predicted_world_pose (caminho BUGGY)
    if (sc.use_correct_bearing && !opened_keyframe) {
      ref_pose = backend.keyframe_pose(obs_keyframe);
    }
    for (std::size_t i = 0; i < dets_world.size(); ++i) {
      const LandmarkUid uid = report.detection_to_uid[i];
      if (uid == 0) {
        continue;
      }
      const BearingRange br = bearing_range_from(ref_pose, dets_world[i].x, dets_world[i].y);
      backend.add_tree_observation(
        uid, obs_keyframe, br.bearing, br.range,
        Eigen::Vector2d(dets_world[i].x, dets_world[i].y));
    }
    backend.optimize();
  }

  // Erro do mapa: empareja cada landmark do backend à árvore-verdade mais
  // próxima e mede o resíduo.
  double sum = 0.0;
  std::size_t n = 0;
  for (const auto uid : backend.all_landmark_uids()) {
    const Eigen::Vector2d p = backend.landmark_position(uid);
    double best = 1e9;
    for (const auto & t : sc.trees) {
      best = std::min(best, std::hypot(p.x() - t.x, p.y() - t.y));
    }
    res.max_landmark_error_m = std::max(res.max_landmark_error_m, best);
    sum += best;
    ++n;
  }
  res.mean_landmark_error_m = n ? sum / static_cast<double>(n) : 0.0;
  return res;
}

// Trajetória em linha reta ao longo de +x, passo `step` por scan.
std::vector<Pose2> straight_line(double step, int n)
{
  std::vector<Pose2> traj;
  for (int i = 0; i < n; ++i) {
    traj.push_back(Pose2{i * step, 0.0, 0.0});
  }
  return traj;
}

// Trajetória de rotação no lugar, passo angular `dtheta` por scan.
std::vector<Pose2> rotate_in_place(double dtheta, int n)
{
  std::vector<Pose2> traj;
  for (int i = 0; i < n; ++i) {
    traj.push_back(Pose2{0.0, 0.0, wrap_angle(i * dtheta)});
  }
  return traj;
}

// Corredor de árvores dos dois lados do eixo x.
std::vector<Tree> corridor()
{
  std::vector<Tree> trees;
  for (int i = 0; i < 12; ++i) {
    trees.push_back(Tree{1.0 + 1.5 * i, 2.5, 0.30});
    trees.push_back(Tree{1.7 + 1.5 * i, -2.2, 0.35});
  }
  return trees;
}
}  // namespace

// ===========================================================================
// CAUSA Nº1 — bearing/range entre keyframes ancorado à keyframe errada.
// Com odom PERFEITA e SEM ruído, o mapa do backend devia reconstruir as
// árvores quase exatamente. Se o caminho BUGGY (como o nó faz) der erro de
// landmark MUITO maior que o CORRECT, a Causa nº1 está confirmada e quantificada.
// ===========================================================================
TEST(NodeGlue_Cause1, StraightLine_BuggyVsCorrect_LandmarkError)
{
  Scenario buggy;
  buggy.trajectory = straight_line(0.10, 60);  // 6 m, keyframes ~cada 0.75 m
  buggy.trees = corridor();
  buggy.use_correct_bearing = false;

  Scenario correct = buggy;
  correct.use_correct_bearing = true;

  const auto rb = run_node_glue(buggy);
  const auto rc = run_node_glue(correct);

  RecordProperty("buggy_max_landmark_error_mm", static_cast<int>(rb.max_landmark_error_m * 1000));
  RecordProperty("correct_max_landmark_error_mm", static_cast<int>(rc.max_landmark_error_m * 1000));

  // O caminho correto deve reconstruir o mapa com erro pequeno (cm).
  EXPECT_LT(rc.max_landmark_error_m, 0.10)
    << "Com odom perfeita, o caminho geometricamente correto devia dar erro ~cm";

  // Se a Causa nº1 for real, o caminho BUGGY tem erro substancialmente maior.
  // (Documenta a magnitude; o fator 3x é um limiar conservador de "claramente pior".)
  EXPECT_GT(rb.max_landmark_error_m, 3.0 * rc.max_landmark_error_m + 0.05)
    << "BUGGY=" << rb.max_landmark_error_m << "m  CORRECT=" << rc.max_landmark_error_m << "m";
}

TEST(NodeGlue_Cause1, Rotation_BuggyVsCorrect_LandmarkError)
{
  Scenario buggy;
  buggy.trajectory = rotate_in_place(0.05, 80);  // 4 rad, keyframes ~cada 0.35 rad
  buggy.trees = corridor();
  buggy.use_correct_bearing = false;

  Scenario correct = buggy;
  correct.use_correct_bearing = true;

  const auto rb = run_node_glue(buggy);
  const auto rc = run_node_glue(correct);

  RecordProperty("buggy_max_landmark_error_mm", static_cast<int>(rb.max_landmark_error_m * 1000));
  RecordProperty("correct_max_landmark_error_mm", static_cast<int>(rc.max_landmark_error_m * 1000));

  EXPECT_LT(rc.max_landmark_error_m, 0.10);
  EXPECT_GT(rb.max_landmark_error_m, 3.0 * rc.max_landmark_error_m + 0.05)
    << "BUGGY=" << rb.max_landmark_error_m << "m  CORRECT=" << rc.max_landmark_error_m << "m";
}

// ===========================================================================
// EFEITO NO TRACKER — duplicação de uid em GROUND com odom perfeita.
// Nº de árvores VISTAS é finito; com odom perfeita um tracker bem alimentado
// não devia criar muito mais uids do que árvores. Excesso => o frame do
// tracker está a saltar (realimentação da Causa nº1 via predicted_world_pose).
// ===========================================================================
TEST(NodeGlue_TrackerSymptom, StraightLine_UidInflation)
{
  Scenario buggy;
  buggy.trajectory = straight_line(0.10, 60);
  buggy.trees = corridor();
  buggy.use_correct_bearing = false;

  Scenario correct = buggy;
  correct.use_correct_bearing = true;

  const auto rb = run_node_glue(buggy);
  const auto rc = run_node_glue(correct);

  RecordProperty("buggy_distinct_uids", static_cast<int>(rb.distinct_uids.size()));
  RecordProperty("correct_distinct_uids", static_cast<int>(rc.distinct_uids.size()));
  RecordProperty("n_trees", static_cast<int>(buggy.trees.size()));

  // Diagnóstico: imprime para o relatório. A asserção compara os dois caminhos
  // — se BUGGY duplica mais uids que CORRECT, o sintoma de tracking está ligado
  // à Causa nº1 e não a ruído (não há ruído aqui).
  EXPECT_GE(rb.distinct_uids.size(), rc.distinct_uids.size());
}

// ===========================================================================
// CAUSA Nº2 — lag odom/scan. Isolada: caminho CORRECT de bearing (remove nº1),
// odom perfeita exceto pelo atraso. Mede quanto a duplicação/erro cresce só
// por causa do lag temporal.
// ===========================================================================
TEST(NodeGlue_Cause2, StraightLine_OdomLagInflatesError)
{
  Scenario no_lag;
  no_lag.trajectory = straight_line(0.10, 60);
  no_lag.trees = corridor();
  no_lag.use_correct_bearing = true;  // remove a Causa nº1
  no_lag.odom_lag_scans = 0;

  Scenario lagged = no_lag;
  lagged.odom_lag_scans = 3;  // ~3 scans de atraso (=30 cm a 0.10 m/scan)

  const auto rn = run_node_glue(no_lag);
  const auto rl = run_node_glue(lagged);

  RecordProperty("nolag_max_landmark_error_mm", static_cast<int>(rn.max_landmark_error_m * 1000));
  RecordProperty("lag_max_landmark_error_mm", static_cast<int>(rl.max_landmark_error_m * 1000));
  RecordProperty("nolag_uids", static_cast<int>(rn.distinct_uids.size()));
  RecordProperty("lag_uids", static_cast<int>(rl.distinct_uids.size()));

  // Documenta o efeito do lag de forma isolada. Sem lag, erro ~cm.
  EXPECT_LT(rn.max_landmark_error_m, 0.10);
  // O lag deve degradar (>=) o erro; a magnitude impressa permite comparar com nº1.
  EXPECT_GE(rl.max_landmark_error_m + 1e-9, rn.max_landmark_error_m);
}

// ===========================================================================
// REGRESSÃO — a lógica CORRIGIDA do nó (bearing/range relativo à keyframe +
// odom sincronizada) reconstrói o mapa quase exatamente, em reta E rotação.
// Guarda contra reintroduzir a Causa nº1.
// ===========================================================================
TEST(NodeGlue_Regression, FixedGlueReconstructsMapExactly)
{
  for (const bool rotating : {false, true}) {
    Scenario sc;
    sc.trajectory = rotating ? rotate_in_place(0.05, 80) : straight_line(0.10, 60);
    sc.trees = corridor();
    sc.use_correct_bearing = true;  // == comportamento atual do nó
    sc.odom_lag_scans = 0;          // == odom sincronizada com o stamp do scan
    const auto r = run_node_glue(sc);
    EXPECT_LT(r.max_landmark_error_m, 0.05)
      << (rotating ? "rotação" : "reta") << ": erro=" << r.max_landmark_error_m << "m";
  }
}

// ===========================================================================
// interpolate_pose — núcleo da correção da Causa nº2 (sincronização odom/scan).
// ===========================================================================
TEST(Se2Geometry, InterpolatePoseMidpoint)
{
  const Pose2 a{0.0, 0.0, 0.0};
  const Pose2 b{2.0, 4.0, 1.0};
  const Pose2 m = interpolate_pose(a, 10.0, b, 12.0, 11.0);  // t no meio
  EXPECT_NEAR(m.x, 1.0, 1e-9);
  EXPECT_NEAR(m.y, 2.0, 1e-9);
  EXPECT_NEAR(m.theta, 0.5, 1e-9);
}

TEST(Se2Geometry, InterpolatePoseClampsOutsideInterval)
{
  const Pose2 a{0.0, 0.0, 0.0};
  const Pose2 b{2.0, 0.0, 0.0};
  EXPECT_NEAR(interpolate_pose(a, 10.0, b, 12.0, 8.0).x, 0.0, 1e-9);   // antes -> a
  EXPECT_NEAR(interpolate_pose(a, 10.0, b, 12.0, 20.0).x, 2.0, 1e-9);  // depois -> b
}

TEST(Se2Geometry, InterpolatePoseWrapsShortestAngle)
{
  // De +170° para -170°: o caminho mais curto passa por +180°, não por 0°.
  const Pose2 a{0.0, 0.0, 170.0 * M_PI / 180.0};
  const Pose2 b{0.0, 0.0, -170.0 * M_PI / 180.0};
  const Pose2 m = interpolate_pose(a, 0.0, b, 1.0, 0.5);
  EXPECT_NEAR(std::abs(m.theta), M_PI, 1e-6);  // ~±180°, não ~0°
}

TEST(Se2Geometry, BearingRangeRoundTrip)
{
  // Um ponto-mundo, visto de uma pose com heading != 0, e reconstruído.
  const Pose2 ref{1.0, -2.0, 0.7};
  const double wx = 4.0, wy = 1.5;
  const BearingRange br = bearing_range_from(ref, wx, wy);
  const Eigen::Vector2d back =
    transform_point(ref, br.range * std::cos(br.bearing), br.range * std::sin(br.bearing));
  EXPECT_NEAR(back.x(), wx, 1e-9);
  EXPECT_NEAR(back.y(), wy, 1e-9);
}
