#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

#include "forest_tree_slam/landmark_class.hpp"
#include "forest_tree_slam/tracker.hpp"

using forest_tree_slam::LandmarkTracker;
using forest_tree_slam::TreeDetection;

namespace
{
TreeDetection make_det(
  double x, double y, double diameter, float confidence = 0.9F,
  float diameter_stddev = 0.0F, double pos_var_xy = 0.0,
  std::array<float, 3> class_scores = {0.0F, 0.0F, 0.0F})
{
  TreeDetection d;
  d.x = x;
  d.y = y;
  d.diameter = diameter;
  d.confidence = confidence;
  d.diameter_stddev = diameter_stddev;
  d.class_scores = class_scores;
  if (pos_var_xy > 0.0) {
    d.base_covariance.setZero();
    d.base_covariance(0, 0) = pos_var_xy;
    d.base_covariance(1, 1) = pos_var_xy;
  }
  return d;
}

std::array<float, 3> scores_trunk_dominant()
{
  return {0.85F, 0.10F, 0.05F};
}

std::array<float, 3> scores_rock_dominant()
{
  return {0.10F, 0.85F, 0.05F};
}

std::array<float, 3> scores_trunk_flip()
{
  return {0.85F, 0.10F, 0.05F};
}

std::array<float, 3> scores_rock_flip()
{
  return {0.10F, 0.85F, 0.05F};
}
}  // namespace

TEST(Tracker, BirthOnFirstObservation)
{
  LandmarkTracker tracker;
  auto report = tracker.update({make_det(1.0, 2.0, 0.3)}, 0.0);
  EXPECT_EQ(report.births.size(), 1u);
  EXPECT_EQ(tracker.tracks().size(), 1u);
  EXPECT_EQ(tracker.tracks().front().n_observations, 1u);
}

TEST(Tracker, AssociatesRepeatedDetectionNearSamePosition)
{
  LandmarkTracker tracker;
  tracker.update({make_det(5.0, 5.0, 0.3)}, 0.0);
  const auto first_uid = tracker.tracks().front().uid;

  // jitter pequeno, mesma árvore
  auto report = tracker.update({make_det(5.05, 4.97, 0.31)}, 1.0);
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_EQ(tracker.tracks().front().uid, first_uid);
  EXPECT_EQ(tracker.tracks().front().n_observations, 2u);
  EXPECT_TRUE(report.births.empty());
  ASSERT_EQ(report.detection_to_uid.size(), 1u);
  EXPECT_EQ(report.detection_to_uid[0], first_uid);
}

TEST(Tracker, DoesNotAssociateFarDetection)
{
  // Este teste valida o gate de ASSOCIAÇÃO (Mahalanobis), ortogonal ao gate de
  // NASCIMENTO por alcance (birth_max_range_m). Neutraliza-se este último (alcance
  // grande) para que a 2.ª deteção, mesmo longe, nasça como track separado em vez
  // de ser silenciosamente descartada pelo gate de alcance.
  forest_tree_slam::TrackerParams params;
  params.birth_max_range_m = 1000.0;
  LandmarkTracker tracker(params);
  tracker.update({make_det(0.0, 0.0, 0.3)}, 0.0);
  // 14m de distância (10,10), muito fora do gate de Mahalanobis -> novo track
  tracker.update({make_det(10.0, 10.0, 0.3)}, 1.0);
  EXPECT_EQ(tracker.tracks().size(), 2u);
}

TEST(Tracker, UnpromotedCandidateCulledAfterTimeout)
{
  // Um candidato (nunca promovido) é LIXO se ficar muito tempo sem confirmar
  // -> apagado em cull_unpromoted_after_scans (controlo de memória).
  forest_tree_slam::TrackerParams params;
  params.cull_unpromoted_after_scans = 2;
  LandmarkTracker tracker(params);
  tracker.update({make_det(0.0, 0.0, 0.3)}, 0.0);  // sem class_scores -> não promove
  ASSERT_EQ(tracker.tracks().size(), 1u);

  tracker.update({}, 1.0);
  tracker.update({}, 2.0);
  EXPECT_EQ(tracker.tracks().size(), 1u);  // ainda dentro do timeout
  auto report = tracker.update({}, 3.0);
  EXPECT_EQ(tracker.tracks().size(), 0u);  // candidato lixo -> apagado
  EXPECT_EQ(report.deaths.size(), 1u);
}

TEST(Tracker, PromotedLandmarkBecomesDormantNotErased)
{
  // Passo 1: um landmark PROMOVIDO no grafo nunca é apagado; ao perder a vista
  // por > death_age_scans ADORMECE (mapa persistente para loop closure).
  forest_tree_slam::TrackerParams params;
  params.death_age_scans = 2;
  params.cull_unpromoted_after_scans = 2;
  LandmarkTracker tracker(params);
  const auto td = scores_trunk_dominant();
  // 4 vistas de ângulos diferentes (robot_xy varia) -> promove a tronco.
  tracker.update({make_det(5.0, 0.0, 0.3, 0.9F, 0.0F, 0.0, td)}, 0.0, Eigen::Vector2d(0.0, 0.0));
  tracker.update({make_det(5.0, 0.0, 0.3, 0.9F, 0.0F, 0.0, td)}, 0.1, Eigen::Vector2d(0.0, 3.0));
  tracker.update({make_det(5.0, 0.0, 0.3, 0.9F, 0.0F, 0.0, td)}, 0.2, Eigen::Vector2d(0.0, -3.0));
  tracker.update({make_det(5.0, 0.0, 0.3, 0.9F, 0.0F, 0.0, td)}, 0.3, Eigen::Vector2d(3.0, 0.0));
  ASSERT_EQ(tracker.tracks().size(), 1u);
  ASSERT_TRUE(LandmarkTracker::is_promoted(tracker.tracks().front()));
  const auto uid = tracker.tracks().front().uid;

  // Misses muito além de death_age_scans -> adormece, NÃO apaga, mantém uid.
  for (int i = 0; i < 6; ++i) {
    tracker.update({}, 1.0 + static_cast<double>(i));
  }
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_TRUE(tracker.tracks().front().dormant);
  EXPECT_EQ(tracker.tracks().front().uid, uid);
}

TEST(Tracker, GeometricReassociationReawakensDormantInsteadOfBirthing)
{
  // Passo 1 (núcleo): o robô regressa a uma zona já mapeada, mas a odometria
  // derivou -> as deteções caem FORA do gate de posição. A re-associação por
  // geometria de vizinhança (constelação/triângulos) tem de reconhecer o mesmo
  // padrão e RE-ACORDAR os landmarks com os MESMOS uids (loop closure), em vez
  // de dar à luz uids novos (o que faria o erro crescer volta a volta).
  forest_tree_slam::TrackerParams params;
  params.death_age_scans = 2;
  params.enable_geometric_reassoc = true;
  // Neutraliza o gate de NASCIMENTO por alcance (ortogonal a este teste): na
  // constelação abaixo, árvores como (6,5) ficam a >8 m de 3 das 4 vistas e só
  // nasceriam na última vista (n_obs=1, nunca promovidas), mascarando o que aqui
  // se valida — a RE-ASSOCIAÇÃO geométrica de landmarks já promovidos.
  params.birth_max_range_m = 1000.0;
  LandmarkTracker tracker(params);

  // Constelação assimétrica de 5 troncos (sem simetrias -> match não ambíguo).
  const std::array<std::pair<double, double>, 5> pts = {{
    {0.0, 0.0}, {6.0, 1.0}, {1.0, 5.0}, {7.0, 4.0}, {3.0, 9.0}}};  // irregular: sem simetria de 180° (o retângulo antigo criava um 2.º cluster com 4/5 inliers → a guarda de margem recusava, e bem)
  const std::array<double, 5> diam = {0.30, 0.34, 0.28, 0.33, 0.37};
  const auto td = scores_trunk_dominant();
  auto make_scan = [&](double dx, double dy) {
      std::vector<TreeDetection> v;
      for (std::size_t k = 0; k < pts.size(); ++k) {
        v.push_back(make_det(pts[k].first + dx, pts[k].second + dy, diam[k], 0.9F, 0.0F, 0.0, td));
      }
      return v;
    };

  // 8 vistas em órbita -> CONFIRMA os 5 (promovido + paralaxe + score). O mapa
  // de referência da re-associação usa is_confirmed (fantasmas fora); na
  // realidade um landmark de referência tem dezenas de observações (mediana
  // ~134 na corrida real), logo 4 vistas era um fixture sub-observado.
  std::vector<Eigen::Vector2d> views;
  for (int s = 0; s < 8; ++s) {
    const double ang = 2.0 * M_PI * static_cast<double>(s) / 8.0;
    views.push_back(Eigen::Vector2d(3.0 + 8.0 * std::cos(ang), 3.0 + 8.0 * std::sin(ang)));
  }
  for (std::size_t s = 0; s < views.size(); ++s) {
    tracker.update(make_scan(0.0, 0.0), 0.1 * static_cast<double>(s), views[s]);
  }
  ASSERT_EQ(tracker.tracks().size(), 5u);
  std::vector<forest_tree_slam::LandmarkUid> orig_uids;
  for (const auto & t : tracker.tracks()) {
    ASSERT_TRUE(LandmarkTracker::is_promoted(t));
    ASSERT_TRUE(tracker.is_confirmed(t));
    orig_uids.push_back(t.uid);
  }

  // Adormecer todos (vários scans vazios > death_age_scans).
  for (int i = 0; i < 5; ++i) {
    tracker.update({}, 2.0 + i);
  }
  ASSERT_EQ(tracker.tracks().size(), 5u);  // persistem
  for (const auto & t : tracker.tracks()) {
    EXPECT_TRUE(t.dormant);
  }

  // Regresso COM DERIVA: a mesma constelação deslocada 3 m em x — muito além
  // do gate de posição (~1 m). Só a geometria pode re-associar.
  auto report = tracker.update(make_scan(3.0, 0.0), 10.0);

  EXPECT_TRUE(report.births.empty()) << "deriva não pode criar uids novos";
  EXPECT_EQ(tracker.tracks().size(), 5u) << "mapa persistente, sem duplicados";
  EXPECT_EQ(report.reawakened.size(), 5u) << "5 loop closures por geometria";
  // Cada deteção tem de apontar para um uid ORIGINAL (mesma identidade).
  for (auto uid : report.detection_to_uid) {
    EXPECT_NE(uid, 0u);
    EXPECT_NE(std::find(orig_uids.begin(), orig_uids.end(), uid), orig_uids.end());
  }
  // E ninguém ficou adormecido após o revisit.
  for (const auto & t : tracker.tracks()) {
    EXPECT_FALSE(t.dormant);
  }
}

TEST(Tracker, GeometricReassociationPrimaryCatchesActiveDriftedLandmark)
{
  // Associação inter-troncos como via PRIMÁRIA: um landmark ATIVO (nunca
  // adormeceu) cuja deteção deriva para fora do gate de posição tem de ser
  // re-associado por geometria de vizinhança, NÃO gerar um duplicado. Antes, a
  // re-associação geométrica só corria se existissem adormecidos (n_dormant>0),
  // logo este caso nascia um uid novo. Agora corre sempre que há mapa+deteções.
  forest_tree_slam::TrackerParams params;
  params.enable_geometric_reassoc = true;
  params.birth_max_range_m = 1000.0;  // ortogonal (ver teste dos adormecidos).
  LandmarkTracker tracker(params);

  const std::array<std::pair<double, double>, 5> pts = {{
    {0.0, 0.0}, {6.0, 1.0}, {1.0, 5.0}, {7.0, 4.0}, {3.0, 9.0}}};  // irregular: sem simetria de 180° (o retângulo antigo criava um 2.º cluster com 4/5 inliers → a guarda de margem recusava, e bem)
  const std::array<double, 5> diam = {0.30, 0.34, 0.28, 0.33, 0.37};
  const auto td = scores_trunk_dominant();
  auto make_scan = [&](double dx, double dy) {
      std::vector<TreeDetection> v;
      for (std::size_t k = 0; k < pts.size(); ++k) {
        v.push_back(make_det(pts[k].first + dx, pts[k].second + dy, diam[k], 0.9F, 0.0F, 0.0, td));
      }
      return v;
    };

  // 8 vistas em órbita → CONFIRMA os 5 (o mapa da re-associação usa
  // is_confirmed). Ficam ATIVOS (sem scans vazios a seguir).
  std::vector<Eigen::Vector2d> views;
  for (int s = 0; s < 8; ++s) {
    const double ang = 2.0 * M_PI * static_cast<double>(s) / 8.0;
    views.push_back(Eigen::Vector2d(3.0 + 8.0 * std::cos(ang), 3.0 + 8.0 * std::sin(ang)));
  }
  for (std::size_t s = 0; s < views.size(); ++s) {
    tracker.update(make_scan(0.0, 0.0), 0.1 * static_cast<double>(s), views[s]);
  }
  ASSERT_EQ(tracker.tracks().size(), 5u);
  std::vector<forest_tree_slam::LandmarkUid> orig_uids;
  for (const auto & t : tracker.tracks()) {
    ASSERT_TRUE(LandmarkTracker::is_promoted(t));
    ASSERT_TRUE(tracker.is_confirmed(t));
    ASSERT_FALSE(t.dormant) << "ainda ativos (não adormeceram)";
    orig_uids.push_back(t.uid);
  }

  // Scan seguinte COM DERIVA (3 m em x), sem qualquer scan vazio → landmarks
  // continuam ATIVOS. O gate de posição falha; só a geometria re-associa.
  auto report = tracker.update(make_scan(3.0, 0.0), 1.0);

  EXPECT_TRUE(report.births.empty()) << "landmark ativo derivado não pode duplicar";
  EXPECT_EQ(tracker.tracks().size(), 5u) << "sem duplicados";
  for (auto uid : report.detection_to_uid) {
    EXPECT_NE(uid, 0u);
    EXPECT_NE(std::find(orig_uids.begin(), orig_uids.end(), uid), orig_uids.end());
  }
}

TEST(Tracker, SyncLandmarkAnchorsReassociatesDormantAfterBackendCorrection)
{
  // Passo 1 (frame consistente): o backend corrige a posição de um landmark
  // por loop closure, mas o tracker tinha uma cópia desatualizada (congelada ao
  // adormecer). Após sincronizar a âncora com o backend, uma deteção na posição
  // CORRIGIDA tem de re-associar o adormecido (gate de posição), sem birth.
  forest_tree_slam::TrackerParams params;
  params.death_age_scans = 2;
  params.enable_geometric_reassoc = false;  // isola o caminho por posição.
  LandmarkTracker tracker(params);
  const auto td = scores_trunk_dominant();
  // Promove um tronco em (5,0) (4 vistas).
  tracker.update({make_det(5.0, 0.0, 0.3, 0.9F, 0.0F, 0.0, td)}, 0.0, Eigen::Vector2d(0.0, 0.0));
  tracker.update({make_det(5.0, 0.0, 0.3, 0.9F, 0.0F, 0.0, td)}, 0.1, Eigen::Vector2d(0.0, 3.0));
  tracker.update({make_det(5.0, 0.0, 0.3, 0.9F, 0.0F, 0.0, td)}, 0.2, Eigen::Vector2d(0.0, -3.0));
  tracker.update({make_det(5.0, 0.0, 0.3, 0.9F, 0.0F, 0.0, td)}, 0.3, Eigen::Vector2d(3.0, 0.0));
  ASSERT_EQ(tracker.tracks().size(), 1u);
  const auto uid = tracker.tracks().front().uid;

  // Adormece.
  for (int i = 0; i < 5; ++i) {
    tracker.update({}, 1.0 + i);
  }
  ASSERT_TRUE(tracker.tracks().front().dormant);

  // Uma deteção em (8,0) está a 3 m da âncora velha (5,0) -> FORA do gate.
  // Sincroniza a âncora com a posição corrigida do backend (8,0): agora a
  // mesma deteção cai EM CIMA -> re-associa por posição, acorda, sem birth.
  std::unordered_map<forest_tree_slam::LandmarkUid, Eigen::Vector2d> backend_pos;
  backend_pos.emplace(uid, Eigen::Vector2d(8.0, 0.0));
  tracker.sync_landmark_anchors(backend_pos);
  EXPECT_NEAR(tracker.tracks().front().xy.x(), 8.0, 1e-9);

  auto report = tracker.update({make_det(8.0, 0.0, 0.3, 0.9F, 0.0F, 0.0, td)}, 20.0);
  EXPECT_TRUE(report.births.empty()) << "deteção corrigida deve re-associar, não nascer";
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_EQ(report.detection_to_uid.front(), uid);
  EXPECT_FALSE(tracker.tracks().front().dormant);
}

// Existem DUAS quantidades distintas: a confiança de CLASSIFICAÇÃO
// (class_posterior) acumula com vistas diversas e PERSISTE; o campo
// track.confidence (o que publicamos em m.confidence) é a QUALIDADE das deteções
// (média móvel/EMA da confiança por-deteção, que escala com arco/distância/base —
// já NÃO rampa a 1.0 por contagem) e ainda decai com os misses (recência).
TEST(Tracker, ClassificationConfidenceAccumulatesAndPersists_ButTrackConfidenceIsQuality)
{
  LandmarkTracker tracker;
  const auto td = scores_trunk_dominant();
  // Observa o MESMO tronco em (5,0) de 8 ângulos diferentes (robô em órbita).
  double first_post = 0.0, last_post = 0.0;
  for (int k = 0; k < 8; ++k) {
    const double ang = k * (M_PI / 4.0);
    const Eigen::Vector2d robot(5.0 - 4.0 * std::cos(ang), -4.0 * std::sin(ang));
    tracker.update({make_det(5.0, 0.0, 0.3, 0.9F, 0.0F, 0.0, td)}, 0.1 * k, robot);
    const double p = LandmarkTracker::class_posterior(tracker.tracks().front())[0];
    if (k == 0) {first_post = p;}
    last_post = p;
  }
  // (1) Confiança de CLASSIFICAÇÃO sobe com vistas diversas e fica muito alta.
  EXPECT_GT(last_post, 0.95);
  EXPECT_GE(last_post, first_post);
  const auto uid = tracker.tracks().front().uid;
  // track.confidence segue a QUALIDADE da deteção (aqui d.confidence=0.9) via EMA,
  // já não rampa a 1.0 por contagem. Converge para ~0.9 enquanto visto.
  EXPECT_NEAR(tracker.tracks().front().confidence, 0.9, 0.06);

  // (2) Muitos scans SEM ver: track.confidence (recência) decai a ~0, MAS o
  //     posterior de classificação MANTÉM-SE alto (a evidência persiste).
  for (int i = 0; i < 20; ++i) {
    tracker.update({}, 100.0 + i);
  }
  const forest_tree_slam::LandmarkTrack * tt = tracker.find_track(uid);
  ASSERT_NE(tt, nullptr) << "landmark promovido persiste (adormecido), não é apagado";
  EXPECT_LT(tt->confidence, 0.05) << "track.confidence é só recência -> decai a 0";
  EXPECT_GT(LandmarkTracker::class_posterior(*tt)[0], 0.95)
    << "a confiança de CLASSIFICAÇÃO persiste — é a que interessa expor/usar";
}

TEST(Tracker, MergesDuplicateTracksFromTwoIds)
{
  LandmarkTracker tracker;
  // dois nascimentos quase coincidentes no mesmo scan (ex.: deteções
  // duplicadas da perceção) devem fundir-se.
  auto report = tracker.update({make_det(2.0, 2.0, 0.30), make_det(2.05, 1.98, 0.31)}, 0.0);
  EXPECT_EQ(tracker.tracks().size(), 1u);
  EXPECT_EQ(report.merges.size(), 1u);
}

// Passo 3 (de-duplicação): dados do sim mostraram uid 1 (8.62,5.48) e uid 3
// (8.72,5.38) — 0.14 m — a sobreviverem como duplicados porque o DBH inflado
// divergia (0.87 vs 0.60 > merge_diameter_m=0.15). Coincidentes (< 0.25 m) têm
// de fundir INDEPENDENTEMENTE do DBH — é fisicamente a mesma árvore.
TEST(Tracker, CoincidentTracksWithDivergentDbhMerge)
{
  LandmarkTracker tracker;
  // 0.15 m de distância (< merge_coincident_dist_m=0.25) mas diâmetros 0.60 vs
  // 0.90 (|Δ|=0.30 > merge_diameter_m=0.15) -> funde por COINCIDÊNCIA.
  auto report = tracker.update({make_det(2.0, 2.0, 0.60), make_det(2.0, 2.15, 0.90)}, 0.0);
  EXPECT_EQ(tracker.tracks().size(), 1u) << "coincidentes fundem mesmo com DBH divergente";
  EXPECT_EQ(report.merges.size(), 1u);

  // Não-coincidentes (0.35 m) com DBH muito diferente continuam SEPARADOS
  // (podem ser duas árvores distintas próximas) — só a banda de DBH semelhante
  // funde a essa distância.
  LandmarkTracker far;
  far.update({make_det(2.0, 2.0, 0.30), make_det(2.0, 2.35, 0.90)}, 0.0);
  EXPECT_EQ(far.tracks().size(), 2u) << "não-coincidentes com DBH diferente não fundem";
}

TEST(Tracker, RejectsBirthBelowConfidenceThreshold)
{
  forest_tree_slam::TrackerParams params;
  params.birth_confidence = 0.5;
  LandmarkTracker tracker(params);
  tracker.update({make_det(0.0, 0.0, 0.3, 0.1F)}, 0.0);
  EXPECT_EQ(tracker.tracks().size(), 0u);
}

// Gate de ingestão multi-vista (A: resíduo de pose; B: qualidade por-frame).
// O buffer é permanente, logo um frame mau corrompe a referência para sempre.
// O bom frame entra; pose derivada (centro longe), stddev alto e DBH por-frame
// muito fora têm de ser REJEITADOS à entrada (n_voxels não cresce).
TEST(Tracker, MultiviewIngestGateRejectsDriftedAndNoisyFrames)
{
  forest_tree_slam::TrackerParams params;
  // Isola (A)/(B): afrouxa o gate de pose (D) p/ não interferir (testado à parte).
  params.multiview_gate_max_pos_var = 100.0;
  params.multiview_gate_min_obs = 0;
  LandmarkTracker tracker(params);
  const double D = 0.40;

  // Track estável em (5,0), trunk-dominante, várias vistas (n_obs >= promote_min_obs).
  for (int k = 0; k < 5; ++k) {
    const Eigen::Vector2d robot(3.0 * std::cos(k), 3.0 * std::sin(k));
    tracker.update(
      {make_det(5.0, 0.0, D, 0.9F, 0.02F, 0.0, scores_trunk_dominant())},
      0.1 * k, robot);
  }
  ASSERT_EQ(tracker.tracks().size(), 1u);
  const auto uid = tracker.tracks().front().uid;
  ASSERT_GE(tracker.tracks().front().n_observations, params.promote_min_obs);

  auto with_stem = [](TreeDetection d) {
    d.has_stem_inliers = true;
    return d;
  };
  auto n_voxels = [&]() { return tracker.tracks().front().multiview_buffer.n_voxels(); };

  // Nuvem do bom frame: anel perto de (5,0).
  std::vector<Eigen::Vector3d> pts_good;
  for (int i = 0; i < 24; ++i) {
    const double a = 2.0 * M_PI * i / 24.0;
    pts_good.emplace_back(5.0 + 0.20 * std::cos(a), 0.20 * std::sin(a), 1.30);
  }
  // Nuvem distinta (voxels diferentes): se um frame MAU fosse aceite, n_voxels cresceria.
  std::vector<Eigen::Vector3d> pts_other;
  for (int i = 0; i < 24; ++i) {
    const double a = 2.0 * M_PI * i / 24.0;
    pts_other.emplace_back(5.0 + 0.20 * std::cos(a), 2.0 + 0.20 * std::sin(a), 1.30);
  }
  const Eigen::Vector2d robot(0.0, 0.0);

  // BOM frame -> entra.
  tracker.ingest_multiview_inliers(
    uid, pts_good, robot,
    with_stem(make_det(5.0, 0.0, D, 0.9F, 0.02F, 0.0, scores_trunk_dominant())));
  const std::size_t after_good = n_voxels();
  EXPECT_GT(after_good, 0u);

  // (A) pose derivada: centro a 0.5 m (> 0.25) -> rejeitado.
  tracker.ingest_multiview_inliers(
    uid, pts_other, robot,
    with_stem(make_det(5.5, 0.0, D, 0.9F, 0.02F, 0.0, scores_trunk_dominant())));
  EXPECT_EQ(n_voxels(), after_good) << "frame com pose derivada não pode entrar";

  // (B) stddev por-frame alto (0.40 > 0.20) -> rejeitado.
  tracker.ingest_multiview_inliers(
    uid, pts_other, robot,
    with_stem(make_det(5.0, 0.0, D, 0.9F, 0.40F, 0.0, scores_trunk_dominant())));
  EXPECT_EQ(n_voxels(), after_good) << "frame com stddev alto não pode entrar";

  // (B) DBH por-frame ao dobro (rel_dev 1.0 > 0.5) -> rejeitado.
  tracker.ingest_multiview_inliers(
    uid, pts_other, robot,
    with_stem(make_det(5.0, 0.0, D * 2.0, 0.9F, 0.02F, 0.0, scores_trunk_dominant())));
  EXPECT_EQ(n_voxels(), after_good) << "frame com DBH muito fora não pode entrar";
}

// Gate (D): só ingere quando o SLAM já corrigiu a posição (track.cov pequena).
// Antes disso (início, ou meio da órbita ao perder tracking) a árvore "anda" e os
// pontos entrariam em coordenadas-mundo erradas. Calibrado no sim: trace ~0.0046
// não-corrigido vs ~0.0017 corrigido; corte em 0.0025.
TEST(Tracker, MultiviewIngestGateRequiresCorrectedPose)
{
  forest_tree_slam::TrackerParams params;
  params.multiview_gate_max_pos_var = 0.0025;
  params.multiview_gate_min_obs = 6;
  // Isola (D): afrouxa (A)/(B)/(C).
  params.multiview_gate_max_center_residual_m = 100.0;
  params.multiview_gate_max_diameter_stddev_m = 100.0;
  params.multiview_gate_max_diameter_rel_dev = 100.0;
  params.multiview_gate_confident_var = 0.0;  // (C) nunca confiante -> desligado
  LandmarkTracker tracker(params);
  const double D = 0.40;

  auto with_stem = [](TreeDetection d) {
    d.has_stem_inliers = true;
    return d;
  };
  std::vector<Eigen::Vector3d> pts;
  for (int i = 0; i < 24; ++i) {
    const double a = 2.0 * M_PI * i / 24.0;
    pts.emplace_back(5.0 + 0.20 * std::cos(a), 0.20 * std::sin(a), 1.30);
  }
  const Eigen::Vector2d robot(0.0, 0.0);

  // 1.ª observação: nobs=1 < min_obs E cov ainda grande -> bloqueia.
  tracker.update(
    {make_det(5.0, 0.0, D, 0.9F, 0.02F, 0.05, scores_trunk_dominant())}, 0.0, robot);
  ASSERT_EQ(tracker.tracks().size(), 1u);
  const auto uid = tracker.tracks().front().uid;
  auto n_voxels = [&]() {return tracker.tracks().front().multiview_buffer.n_voxels();};
  tracker.ingest_multiview_inliers(
    uid, pts, robot, with_stem(make_det(5.0, 0.0, D, 0.9F, 0.02F, 0.05, scores_trunk_dominant())));
  EXPECT_EQ(n_voxels(), 0u) << "pose não corrigida -> não ingere";

  // Muitas observações apertadas -> EKF encolhe a cov abaixo do limiar.
  for (int k = 0; k < 40; ++k) {
    const Eigen::Vector2d r(3.0 * std::cos(0.3 * k), 3.0 * std::sin(0.3 * k));
    tracker.update(
      {make_det(5.0, 0.0, D, 0.9F, 0.02F, 0.001, scores_trunk_dominant())}, 0.1 * (k + 1), r);
  }
  ASSERT_LT(tracker.tracks().front().cov.trace(), params.multiview_gate_max_pos_var)
    << "pré-condição: cov tem de ter encolhido";
  tracker.ingest_multiview_inliers(
    uid, pts, robot, with_stem(make_det(5.0, 0.0, D, 0.9F, 0.02F, 0.001, scores_trunk_dominant())));
  EXPECT_GT(n_voxels(), 0u) << "pose corrigida -> ingere";
}

TEST(Tracker, FastRotationWithoutPredictionCausesDuplicate)
{
  // Árvore a 8m do robô; entre os dois scans o robô "roda" 0.2 rad (~11.5°)
  // sem que isso seja comunicado ao tracker (angular_delta_rad por defeito
  // = 0). O deslocamento lateral aparente da árvore é 16*sin(0.1) ~= 1.6m,
  // bem acima do gate (innovation cov = cov_track+cov_deteção ~0.08m²,
  // threshold ~0.86m) -> nasce um track duplicado em vez de associar.
  // Reproduz o bug relatado em campo (rotações rápidas geram árvores
  // duplicadas no RViz).
  LandmarkTracker tracker;
  tracker.update({make_det(8.0, 0.0, 0.3)}, 0.0);
  ASSERT_EQ(tracker.tracks().size(), 1u);

  tracker.update({make_det(8.0 * std::cos(0.2), 8.0 * std::sin(0.2), 0.3)}, 1.0);
  EXPECT_EQ(tracker.tracks().size(), 2u) << "sem predição, a rotação rápida devia causar duplicado";
}

TEST(Tracker, FastRotationWithPredictionAssociatesCorrectly)
{
  // Mesmo cenário, mas agora informando o tracker do Δheading entre scans
  // (robot_xy na origem, angular_delta_rad=0.2) -> o passo de predição infla
  // a covariância do track pelo termo de braço de alavanca e o gate volta a
  // aceitar a árvore como a mesma.
  LandmarkTracker tracker;
  tracker.update({make_det(8.0, 0.0, 0.3)}, 0.0);
  const auto first_uid = tracker.tracks().front().uid;

  auto report = tracker.update(
    {make_det(8.0 * std::cos(0.2), 8.0 * std::sin(0.2), 0.3)}, 1.0,
    Eigen::Vector2d::Zero(), 0.2);
  ASSERT_EQ(tracker.tracks().size(), 1u) << "com predição, a mesma rotação não devia duplicar";
  EXPECT_EQ(tracker.tracks().front().uid, first_uid);
  EXPECT_TRUE(report.births.empty());
}

TEST(Tracker, MultiTreeHungarianAssignsClosestPairs)
{
  LandmarkTracker tracker;
  tracker.update({make_det(0.0, 0.0, 0.3), make_det(5.0, 0.0, 0.4)}, 0.0);
  ASSERT_EQ(tracker.tracks().size(), 2u);
  const auto uid_a = tracker.tracks()[0].uid;
  const auto uid_b = tracker.tracks()[1].uid;

  // robô andou; as duas árvores movem-se ligeiramente no frame de trabalho,
  // mas a correspondência geometricamente correta é a mais próxima de cada.
  auto report = tracker.update(
    {make_det(5.1, 0.1, 0.41), make_det(0.1, -0.1, 0.31)}, 1.0);
  ASSERT_EQ(report.detection_to_uid.size(), 2u);
  EXPECT_EQ(report.detection_to_uid[0], uid_b);  // 5.1,0.1 -> árvore B
  EXPECT_EQ(report.detection_to_uid[1], uid_a);  // 0.1,-0.1 -> árvore A
}

TEST(Tracker, HighUncertaintyDbhBarelyMovesTrack)
{
  // Estabelece o track com várias observações de arco largo (σ baixo).
  LandmarkTracker tracker;
  constexpr double kTrueDbh = 0.40;
  for (int i = 0; i < 10; ++i) {
    tracker.update({make_det(5.0, 5.0, kTrueDbh, 0.9F, 0.02F)}, static_cast<double>(i));
  }
  const double before = tracker.tracks().front().diameter;

  // Um arco curto (σ alto) com DBH errado quase não deve mover o track.
  tracker.update({make_det(5.0, 5.0, 0.10, 0.9F, 0.30F)}, 10.0);
  const double after = tracker.tracks().front().diameter;
  EXPECT_NEAR(after, before, 0.02)
    << "arco curto (σ=0.30) não devia puxar o DBH mais de 2 cm";
  EXPECT_NEAR(after, kTrueDbh, 0.03);
}

TEST(Tracker, AlternatingArcCoverageConvergesToTrueDbh)
{
  // Simula multi-view: arcos largos (σ=0.02, valor certo) intercalados com
  // arcos curtos (σ=0.30, valor errado). Com EMA fixo (α=0.2) o DBH derivava
  // ~15–20%; com filtro de informação converge para o valor certo.
  LandmarkTracker tracker;
  constexpr double kTrueDbh = 0.40;
  constexpr double kBadDbh = 0.12;
  for (int i = 0; i < 40; ++i) {
    const bool good = (i % 2) == 0;
    const double d = good ? kTrueDbh : kBadDbh;
    const float sigma = good ? 0.02F : 0.30F;
    tracker.update({make_det(3.0, 3.0, d, 0.9F, sigma)}, static_cast<double>(i));
  }
  EXPECT_NEAR(tracker.tracks().front().diameter, kTrueDbh, 0.04)
    << "DBH fundido devia convergir para o valor dos arcos largos";
}

TEST(Tracker, LowUncertaintyPositionDominatesHighUncertainty)
{
  LandmarkTracker tracker;
  tracker.update({make_det(0.0, 0.0, 0.3, 0.9F, 0.0F, 0.0004)}, 0.0);
  for (int i = 0; i < 5; ++i) {
    tracker.update({make_det(0.02, 0.0, 0.3, 0.9F, 0.02F, 0.0004)}, static_cast<double>(i + 1));
  }
  const double x_after_good = tracker.tracks().front().xy.x();

  tracker.update({make_det(2.0, 2.0, 0.3, 0.9F, 0.0F, 4.0)}, 10.0);
  const double x_after_bad = tracker.tracks().front().xy.x();
  EXPECT_NEAR(x_after_bad, x_after_good, 0.05)
    << "deteção com cov grande (2 m stddev) não devia deslocar a posição >5 cm";
}

TEST(Tracker, OscillatingClassScoresAssociateToSameUid)
{
  LandmarkTracker tracker;
  tracker.update({make_det(2.0, 2.0, 0.4, 0.9F, 0.0F, 0.0, scores_trunk_dominant())}, 0.0);
  const auto first_uid = tracker.tracks().front().uid;

  // Mesma posição, classe por-frame oscila tronco↔rocha — não deve duplicar.
  tracker.update({make_det(2.02, 1.98, 0.41, 0.9F, 0.0F, 0.0, scores_rock_flip())}, 1.0);
  tracker.update({make_det(1.99, 2.01, 0.39, 0.9F, 0.0F, 0.0, scores_trunk_flip())}, 2.0);
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_EQ(tracker.tracks().front().uid, first_uid);
}

TEST(Tracker, ClassLogOddsConvergesDespiteMinorityFrames)
{
  forest_tree_slam::TrackerParams params;
  params.promote_min_obs = 3;
  params.promote_prob = 0.55;
  params.promote_margin = 0.10;
  LandmarkTracker tracker(params);

  // Árvore FIXA em (5,0); o robô ORBITA-a (multi-vista física). Bearings distintos
  // dão vistas novas à acumulação de classe, sem mover a deteção no mundo — assim a
  // posição do committed (dona do grafo) fica estável e não nasce duplicado.
  for (int i = 0; i < 8; ++i) {
    const bool minority = (i % 4) == 3;
    const auto scores = minority ? scores_rock_dominant() : scores_trunk_dominant();
    const double angle = static_cast<double>(i) * 0.4;
    const Eigen::Vector2d robot(5.0 - 4.0 * std::cos(angle), -4.0 * std::sin(angle));
    tracker.update({make_det(5.0, 0.0, 0.35, 0.9F, 0.0F, 0.0, scores)}, static_cast<double>(i),
      robot, 0.4);
  }

  ASSERT_EQ(tracker.tracks().size(), 1u);
  const auto & t = tracker.tracks().front();
  ASSERT_TRUE(LandmarkTracker::is_promoted(t));
  EXPECT_EQ(t.committed_class, forest_tree_slam::kCommittedTrunk);
}

TEST(Tracker, CandidatePromotesOnlyAfterThresholds)
{
  forest_tree_slam::TrackerParams params;
  params.promote_min_obs = 4;
  params.promote_prob = 0.70;
  params.promote_margin = 0.15;
  LandmarkTracker tracker(params);
  const Eigen::Vector2d robot{0.0, 0.0};

  for (int i = 0; i < 3; ++i) {
    const double angle = static_cast<double>(i) * 0.5;
    const double x = 4.0 * std::cos(angle);
    const double y = 4.0 * std::sin(angle);
    tracker.update(
      {make_det(x, y, 0.3, 0.9F, 0.0F, 0.0, scores_trunk_dominant())},
      static_cast<double>(i), robot, 0.5);
    EXPECT_FALSE(LandmarkTracker::is_promoted(tracker.tracks().front()))
      << "não devia promover antes de promote_min_obs";
  }

  const double angle = 1.5;
  tracker.update(
    {make_det(4.0 * std::cos(angle), 4.0 * std::sin(angle), 0.3, 0.9F, 0.0F, 0.0,
      scores_trunk_dominant())},
    3.0, robot, 0.5);
  EXPECT_TRUE(LandmarkTracker::is_promoted(tracker.tracks().front()));
  EXPECT_EQ(tracker.tracks().front().committed_class, forest_tree_slam::kCommittedTrunk);
}

TEST(Tracker, ObstacleClassifiedButNotMapOutput)
{
  EXPECT_FALSE(forest_tree_slam::is_map_output_class(forest_tree_slam::kCommittedObstacle));
  EXPECT_TRUE(forest_tree_slam::is_map_output_class(forest_tree_slam::kCommittedTrunk));
  EXPECT_TRUE(forest_tree_slam::is_map_output_class(forest_tree_slam::kCommittedRock));
}

// Anti-correlação (S-B): ver a mesma árvore parada N vezes do MESMO ângulo não
// deve inflar a evidência tanto como vê-la de N ângulos distintos. Sem isto, um
// robô parado "enche" o log-odds artificialmente (o mesmo erro de contar o mesmo
// arco 2x do DBH). Validamos que, para o mesmo nº de observações, a evidência
// acumulada do caso estático é muito menor que a do caso multi-vista.
TEST(Tracker, StationaryRepeatsDoNotInflateClassEvidence)
{
  constexpr int kObs = 12;
  const std::array<float, 3> scores = scores_trunk_dominant();

  // Caso A — robô parado: bearing constante (mesma vista repetida).
  double static_logodds_spread = 0.0;
  {
    LandmarkTracker tracker;
    const Eigen::Vector2d robot{0.0, 0.0};
    for (int i = 0; i < kObs; ++i) {
      // Δheading=0: nada roda; a árvore fica sempre no mesmo bearing/bin.
      tracker.update({make_det(5.0, 0.0, 0.35, 0.9F, 0.0F, 0.0, scores)},
        static_cast<double>(i), robot, 0.0);
    }
    ASSERT_EQ(tracker.tracks().size(), 1u);
    const auto & lo = tracker.tracks().front().class_logodds;
    static_logodds_spread = lo[0] - lo[1];  // separação tronco vs rocha
  }

  // Caso B — robô orbita uma árvore FIXA em (5,0): cada observação chega de um
  // ângulo distinto (vista nova). É a árvore que fica parada e o robô que se move
  // (multi-vista física) — assim mede-se a evidência por bearings distintos sem
  // deslocar a deteção no mundo.
  double orbit_logodds_spread = 0.0;
  {
    LandmarkTracker tracker;
    for (int i = 0; i < kObs; ++i) {
      const double angle = static_cast<double>(i) * (2.0 * M_PI / kObs);
      const Eigen::Vector2d robot(5.0 - 4.0 * std::cos(angle), -4.0 * std::sin(angle));
      tracker.update({make_det(5.0, 0.0, 0.35, 0.9F, 0.0F, 0.0, scores)},
        static_cast<double>(i), robot, 2.0 * M_PI / kObs);
    }
    ASSERT_EQ(tracker.tracks().size(), 1u);
    const auto & lo = tracker.tracks().front().class_logodds;
    orbit_logodds_spread = lo[0] - lo[1];
  }

  // A 1.ª observação conta sempre cheia (vista inicial); as restantes 11 ficam
  // atenuadas (peso 0.15) no caso estático. A evidência orbital deve ser
  // claramente maior — pelo menos ~2x — para o mesmo nº de frames.
  EXPECT_GT(orbit_logodds_spread, 2.0 * static_logodds_spread)
    << "estático=" << static_logodds_spread << " orbital=" << orbit_logodds_spread;
}

// ── F3: fusão de classe da câmara ──────────────────────────────────────────

TEST(TrackerCameraFusion, NoCameraEvidenceIsExactKillSwitch)
{
  // Sem termo de câmara, a posterior fundida == posterior do LiDAR (o cap nem
  // atua). Garante que fusion desligada reproduz o baseline bit-a-bit.
  LandmarkTracker tracker;
  forest_tree_slam::LandmarkTrack t;
  t.class_logodds = Eigen::Vector3d(2.0, 0.5, 0.1);
  // t.class_logodds_cam fica a zero (default).
  const Eigen::Vector3d fused = tracker.class_posterior_fused(t);
  const Eigen::Vector3d lidar = LandmarkTracker::class_posterior(t);
  EXPECT_TRUE(fused.isApprox(lidar, 1e-12));
}

TEST(TrackerCameraFusion, CameraTrunkEvidenceFlipsAmbiguousLidar)
{
  // LiDAR ambíguo com a rocha ligeiramente à frente; a câmara diz TRONCO com
  // força → a posterior fundida passa a favorecer o tronco.
  LandmarkTracker tracker;
  forest_tree_slam::LandmarkTrack t;
  t.class_logodds = Eigen::Vector3d(1.0, 1.5, 0.0);  // rocha > tronco (LiDAR)
  const Eigen::Vector3d before = LandmarkTracker::class_posterior(t);
  ASSERT_GT(before[1], before[0]);

  t.class_logodds_cam = Eigen::Vector3d(3.0, 0.0, 0.0);  // câmara: tronco
  const Eigen::Vector3d fused = tracker.class_posterior_fused(t);
  EXPECT_GT(fused[0], fused[1]);      // tronco passa a dominar
  EXPECT_GT(fused[0], before[0]);     // e subiu face ao LiDAR-only
}

TEST(TrackerCameraFusion, ConfidentLidarIsCappedWhenCameraPresent)
{
  // LiDAR quase certo em rocha; com câmara (mesmo só a concordar pouco), a
  // componente do LiDAR satura no teto c_lidar (não fica perto de 1).
  forest_tree_slam::TrackerParams params;
  params.fusion_class_c_lidar = 0.85;
  LandmarkTracker tracker(params);
  forest_tree_slam::LandmarkTrack t;
  t.class_logodds = Eigen::Vector3d(0.0, 8.0, 0.0);   // rocha ~1.0 no LiDAR
  ASSERT_GT(LandmarkTracker::class_posterior(t)[1], 0.99);
  t.class_logodds_cam = Eigen::Vector3d(0.0, 0.2, 0.0);  // câmara concorda fraco
  const Eigen::Vector3d fused = tracker.class_posterior_fused(t);
  EXPECT_LT(fused[1], 0.90);  // capado: já não está colado a 1
}

TEST(TrackerCameraFusion, CameraConfirmationPromotesAmbiguousLidar)
{
  // LiDAR vê um objeto AMBÍGUO (tronco/rocha equilibrados) de vários ângulos →
  // n_obs sobe mas NÃO promove (margem insuficiente). A câmara confirma TRONCO
  // de vários ângulos → cruza o limiar e promove a tronco.
  LandmarkTracker tracker;
  const std::array<float, 3> ambiguous = {0.45F, 0.45F, 0.10F};
  forest_tree_slam::LandmarkUid uid = 0;
  for (int i = 0; i < 8; ++i) {
    auto rep = tracker.update(
      {make_det(5.0, 0.0, 0.3, 0.9F, 0.0F, 0.0, ambiguous)}, static_cast<double>(i),
      Eigen::Vector2d(0.0, static_cast<double>(i) * 2.0));  // robô move em Y → Δbearing
    if (!rep.births.empty()) {uid = rep.births[0];}
  }
  ASSERT_NE(uid, 0u);
  ASSERT_NE(tracker.find_track(uid), nullptr);
  EXPECT_EQ(tracker.find_track(uid)->committed_class, forest_tree_slam::kCommittedUnknown)
    << "LiDAR ambíguo sozinho não deve promover";

  for (int i = 0; i < 8; ++i) {
    tracker.fuse_camera_class(uid, 0 /*tronco*/, 0.85, static_cast<double>(i) * 0.3);
  }
  EXPECT_EQ(tracker.find_track(uid)->committed_class, forest_tree_slam::kCommittedTrunk)
    << "a confirmação da câmara deve promover a tronco";
}

TEST(TrackerCameraFusion, WeakCameraDetectionIsIgnored)
{
  // Deteção de câmara abaixo de cam_min_conf não altera o log-odds.
  forest_tree_slam::TrackerParams params;
  params.fusion_class_cam_min_conf = 0.40;
  LandmarkTracker tracker(params);
  auto rep = tracker.update({make_det(3.0, 0.0, 0.3)}, 0.0);
  ASSERT_FALSE(rep.births.empty());
  const auto uid = rep.births[0];
  tracker.fuse_camera_class(uid, 0, 0.30 /*< min_conf*/, 0.0);
  EXPECT_TRUE(tracker.find_track(uid)->class_logodds_cam.isZero(0.0));
}

TEST(TrackerCameraFusion, AntiCorrelationBlocksSameBearing)
{
  // A câmara não deve "encher" o log-odds parada: duas fusões do MESMO bearing
  // só contam uma; um bearing novo volta a acumular.
  LandmarkTracker tracker;
  auto rep = tracker.update({make_det(5.0, 0.0, 0.3)}, 0.0);
  ASSERT_FALSE(rep.births.empty());
  const auto uid = rep.births[0];

  tracker.fuse_camera_class(uid, 0, 0.85, 0.50);            // 1ª: acumula
  const Eigen::Vector3d a = tracker.find_track(uid)->class_logodds_cam;
  EXPECT_GT(a[0], 0.0);

  tracker.fuse_camera_class(uid, 0, 0.85, 0.50);            // mesmo bearing: bloqueia
  const Eigen::Vector3d b = tracker.find_track(uid)->class_logodds_cam;
  EXPECT_TRUE(a.isApprox(b)) << "não devia acumular com o mesmo bearing";

  tracker.fuse_camera_class(uid, 0, 0.85, 1.00);            // bearing novo: acumula
  EXPECT_GT(tracker.find_track(uid)->class_logodds_cam[0], b[0]);
}
