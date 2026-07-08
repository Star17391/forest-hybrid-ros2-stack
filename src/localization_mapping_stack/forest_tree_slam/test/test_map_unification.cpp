// Testes-espec da UNIFICAÇÃO DOS MAPAS (front-end / back-end).
//
// Alvo da arquitetura: o BACK-END (grafo) é o único dono da POSIÇÃO dos
// landmarks COMMITTED (promovidos, tronco/rocha). O tracker deixa de manter uma
// estimativa de posição paralela para esses — lê a do backend via
// `sync_landmark_anchors`. Mantém tudo o que NÃO é posição (classe, DBH, score)
// e mantém a máquina de COVARIÂNCIA (para o gate de associação continuar são sob
// rotação/deriva). Os CANDIDATOS (ainda não promovidos) mantêm o filtro leve.
//
// Estes testes falham no código pré-refactor (a posição dos committed era
// filtrada das deteções) e passam depois — são a especificação executável.

#include <gtest/gtest.h>

#include <array>
#include <unordered_map>
#include <vector>

#include "forest_tree_slam/landmark_class.hpp"
#include "forest_tree_slam/tracker.hpp"

using forest_tree_slam::LandmarkTracker;
using forest_tree_slam::LandmarkUid;
using forest_tree_slam::TreeDetection;

namespace
{
TreeDetection det(
  double x, double y, double diameter = 0.3, float confidence = 0.9F,
  std::array<float, 3> class_scores = {0.0F, 0.0F, 0.0F})
{
  TreeDetection d;
  d.x = x;
  d.y = y;
  d.diameter = diameter;
  d.confidence = confidence;
  d.class_scores = class_scores;
  return d;
}

std::array<float, 3> trunk() {return {0.85F, 0.10F, 0.05F};}

// Promove um tronco estável em `p` visto de 4 ângulos distintos (paralaxe) →
// committed tronco (entra no grafo). Devolve o uid.
LandmarkUid promote_trunk_at(LandmarkTracker & tracker, double px, double py)
{
  const auto td = trunk();
  tracker.update({det(px, py, 0.3, 0.9F, td)}, 0.0, Eigen::Vector2d(px - 5.0, py));
  tracker.update({det(px, py, 0.3, 0.9F, td)}, 0.1, Eigen::Vector2d(px, py - 5.0));
  tracker.update({det(px, py, 0.3, 0.9F, td)}, 0.2, Eigen::Vector2d(px, py + 5.0));
  tracker.update({det(px, py, 0.3, 0.9F, td)}, 0.3, Eigen::Vector2d(px + 5.0, py));
  return tracker.tracks().front().uid;
}
}  // namespace

// A posição de um landmark committed ATIVO segue o backend, NÃO as deteções.
// (Antes: sync só mexia nos adormecidos e a posição era fundida das deteções.)
TEST(MapUnification, CommittedActivePositionFollowsBackendNotDetection)
{
  LandmarkTracker tracker;
  const auto uid = promote_trunk_at(tracker, 5.0, 0.0);
  ASSERT_TRUE(LandmarkTracker::is_promoted(tracker.tracks().front()));

  // Backend corrige a posição para (5.3, 0) — autoridade do grafo.
  std::unordered_map<LandmarkUid, Eigen::Vector2d> backend_pos;
  backend_pos.emplace(uid, Eigen::Vector2d(5.3, 0.0));
  tracker.sync_landmark_anchors(backend_pos);
  ASSERT_NEAR(tracker.tracks().front().xy.x(), 5.3, 1e-9)
    << "sync tem de ancorar committed ATIVOS (não só adormecidos)";

  // Deteção ligeiramente ao lado (dentro do gate): NÃO deve puxar a posição.
  tracker.update({det(5.5, 0.0, 0.3, 0.9F, trunk())}, 1.0, Eigen::Vector2d(0.0, 0.0));
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_NEAR(tracker.tracks().front().xy.x(), 5.3, 0.02)
    << "posição do committed = grafo; a deteção não a filtra";
  EXPECT_GE(tracker.tracks().front().n_observations, 5u)
    << "mas a observação conta (associou)";
}

// A fusão da PERCEÇÃO (classe log-odds) continua a acumular num committed — o
// refactor tira a posição, não a classe.
TEST(MapUnification, CommittedClassEvidenceStillAccumulates)
{
  LandmarkTracker tracker;
  const auto uid = promote_trunk_at(tracker, 5.0, 0.0);
  std::unordered_map<LandmarkUid, Eigen::Vector2d> backend_pos;
  backend_pos.emplace(uid, Eigen::Vector2d(5.0, 0.0));

  const double post_before = LandmarkTracker::class_posterior(*tracker.find_track(uid))[0];
  for (int k = 0; k < 6; ++k) {
    tracker.sync_landmark_anchors(backend_pos);
    const double ang = k * (M_PI / 3.0);
    tracker.update(
      {det(5.0, 0.0, 0.3, 0.9F, trunk())}, 1.0 + k,
      Eigen::Vector2d(5.0 - 4.0 * std::cos(ang), -4.0 * std::sin(ang)));
  }
  const double post_after = LandmarkTracker::class_posterior(*tracker.find_track(uid))[0];
  EXPECT_GE(post_after, post_before) << "classe continua a acumular no committed";
  EXPECT_GT(post_after, 0.9);
}

// A fusão da CÂMARA (F3) continua a funcionar sobre um committed ancorado ao grafo.
TEST(MapUnification, CameraFusionStillWorksOnCommitted)
{
  LandmarkTracker tracker;
  const auto uid = promote_trunk_at(tracker, 5.0, 0.0);
  std::unordered_map<LandmarkUid, Eigen::Vector2d> backend_pos;
  backend_pos.emplace(uid, Eigen::Vector2d(5.0, 0.0));
  tracker.sync_landmark_anchors(backend_pos);

  const auto before = tracker.find_track(uid)->class_logodds_cam;
  tracker.fuse_camera_class(uid, 0 /*tronco*/, 0.9, 0.0);
  const auto after = tracker.find_track(uid)->class_logodds_cam;
  EXPECT_GT(after[0], before[0]) << "câmara ainda injeta evidência de classe";
}

// Um CANDIDATO (nunca promovido) mantém o filtro de posição — comportamento
// inalterado (guarda de não-regressão do front-end).
TEST(MapUnification, CandidatePositionStillFusesFromDetections)
{
  LandmarkTracker tracker;
  // Sem class_scores → nunca promove → fica candidato.
  tracker.update({det(0.0, 0.0, 0.3)}, 0.0);
  const double x0 = tracker.tracks().front().xy.x();
  for (int i = 0; i < 4; ++i) {
    tracker.update({det(0.4, 0.0, 0.3)}, static_cast<double>(i + 1));
  }
  const double x1 = tracker.tracks().front().xy.x();
  EXPECT_GT(x1, x0 + 0.1) << "candidato ainda filtra a posição das deteções";
  EXPECT_FALSE(LandmarkTracker::is_promoted(tracker.tracks().front()));
}
