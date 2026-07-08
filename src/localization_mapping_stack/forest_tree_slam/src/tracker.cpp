#include "forest_tree_slam/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "forest_3d_perception/landmark_class_scorer.hpp"
#include "forest_tree_slam/hungarian.hpp"
#include "forest_tree_slam/se2_geometry.hpp"

namespace forest_tree_slam
{

namespace
{
double clamp_prob(float p, float eps)
{
  return std::clamp(static_cast<double>(p), static_cast<double>(eps),
        1.0 - static_cast<double>(eps));
}

// Committed no grafo = promovido E classe de grafo (tronco/rocha). É o conjunto
// cuja POSIÇÃO passa a ser propriedade exclusiva do backend (unificação dos
// mapas). Obstáculos e candidatos ficam de fora (front-end).
bool is_committed_graph(const LandmarkTrack & t)
{
  return LandmarkTracker::is_promoted(t) && is_slam_graph_class(t.committed_class);
}
}  // namespace

Eigen::Vector3d LandmarkTracker::class_posterior(const LandmarkTrack & track)
{
  const double m = track.class_logodds.maxCoeff();
  Eigen::Vector3d expv = (track.class_logodds.array() - m).exp();
  const double sum = expv.sum();
  if (sum < 1e-12) {
    return Eigen::Vector3d::Constant(1.0 / 3.0);
  }
  return expv / sum;
}

Eigen::Vector3d LandmarkTracker::class_posterior_fused(const LandmarkTrack & track) const
{
  // Sem evidência de câmara → idêntico ao LiDAR (kill-switch exato: nem o cap atua).
  if (track.class_logodds_cam.isZero(0.0)) {
    return class_posterior(track);
  }
  const Eigen::Vector3d lidar = class_posterior(track);

  // Cap: limita a componente máxima do LiDAR a `c` e redistribui o excedente pelas
  // outras (proporcional à sua massa) — deixa "espaço" para a câmara cruzar o
  // limiar de promoção, em vez de o LiDAR saturar sozinho perto de 1.
  const double c = params_.fusion_class_c_lidar;
  Eigen::Vector3d capped = lidar;
  Eigen::Index mx = 0;
  const double maxv = lidar.maxCoeff(&mx);
  if (c > 0.0 && c < 1.0 && maxv > c) {
    const double excess = maxv - c;
    capped[mx] = c;
    const double others = 1.0 - maxv;  // massa total das restantes componentes
    for (Eigen::Index i = 0; i < 3; ++i) {
      if (i == mx) {
        continue;
      }
      capped[i] += others > 1e-9 ? excess * (lidar[i] / others) : excess * 0.5;
    }
  }

  // Combina com a câmara em log-odds: softmax(log(capped) + logodds_cam).
  Eigen::Vector3d logits;
  for (Eigen::Index i = 0; i < 3; ++i) {
    logits[i] = std::log(std::max(capped[i], 1e-9)) + track.class_logodds_cam[i];
  }
  const double m = logits.maxCoeff();
  Eigen::Vector3d expv = (logits.array() - m).exp();
  const double sum = expv.sum();
  if (sum < 1e-12) {
    return Eigen::Vector3d::Constant(1.0 / 3.0);
  }
  return expv / sum;
}

bool LandmarkTracker::is_promoted(const LandmarkTrack & track)
{
  return is_promoted_class(track.committed_class);
}

bool LandmarkTracker::is_confirmed(const LandmarkTrack & track) const
{
  if (!is_promoted(track)) {
    return false;
  }
  // Nº de ângulos (bins de bearing) distintos de onde o tronco foi observado.
  const int n_parallax_bins =
    static_cast<int>(__builtin_popcount(track.parallax_bins));
  return n_parallax_bins >= params_.promote_min_parallax_bins &&
         track.score >= params_.promote_score_min;
}

bool LandmarkTracker::has_class_scores(const TreeDetection & d)
{
  const float sum = d.class_scores[0] + d.class_scores[1] + d.class_scores[2];
  return sum > 1.0e-6F;
}

int LandmarkTracker::bearing_bin(double bearing_rad) const
{
  constexpr double kTwoPi = 2.0 * M_PI;
  double b = std::fmod(bearing_rad + kTwoPi, kTwoPi);
  const int bins = std::max(1, params_.class_coverage_bins);
  int bin = static_cast<int>(b / kTwoPi * static_cast<double>(bins));
  if (bin >= bins) {
    bin = bins - 1;
  }
  return bin;
}

void LandmarkTracker::accumulate_class_scores(
  LandmarkTrack & track, const TreeDetection & detection,
  const Eigen::Vector2d & robot_xy) const
{
  if (!has_class_scores(detection)) {
    return;
  }

  const double bearing = std::atan2(track.xy.y() - robot_xy.y(), track.xy.x() - robot_xy.x());
  const int bin = bearing_bin(bearing);
  const std::uint64_t bit = 1ULL << static_cast<unsigned>(bin % 64);

  bool new_view = track.class_coverage_bits == 0;
  if (!new_view && (track.class_coverage_bits & bit) == 0) {
    new_view = true;
  }
  if (!new_view) {
    const double delta = std::abs(wrap_angle(bearing - track.last_obs_bearing_rad));
    new_view = delta >= params_.class_min_bearing_delta_rad;
  }

  const double weight = new_view ? 1.0 : params_.class_correlated_obs_weight;
  const float eps = params_.class_log_eps;
  for (std::size_t i = 0; i < kNumClassScores; ++i) {
    track.class_logodds[static_cast<Eigen::Index>(i)] +=
      weight * std::log(clamp_prob(detection.class_scores[i], eps));
  }

  if (new_view) {
    track.class_coverage_bits |= bit;
    track.last_obs_bearing_rad = bearing;
  }
}

void LandmarkTracker::fuse_camera_class(
  LandmarkUid uid, int class_idx, double conf, double bearing_rad)
{
  if (class_idx < 0 || class_idx >= 3) {
    return;
  }
  if (conf < params_.fusion_class_cam_min_conf) {
    return;  // deteção de câmara fraca: ignora
  }
  for (auto & track : tracks_) {
    if (track.uid != uid) {
      continue;
    }
    // Anti-correlação: não deixar a câmara "encher" o log-odds parada (mesma vista),
    // tal como o LiDAR. Só acumula se a direção do landmark mudou o suficiente.
    if (!std::isnan(track.last_cam_bearing_rad)) {
      const double delta = std::abs(wrap_angle(bearing_rad - track.last_cam_bearing_rad));
      if (delta < params_.class_min_bearing_delta_rad) {
        return;
      }
    }
    const double q = std::clamp(conf, 1.0e-3, 1.0 - 1.0e-3);
    const double logit = std::log(q / (1.0 - q));
    track.class_logodds_cam[class_idx] += params_.fusion_class_w_cam * logit;
    track.last_cam_bearing_rad = bearing_rad;
    try_promote(track);  // a câmara pode cruzar o limiar de promoção mais cedo
    return;
  }
}

std::uint32_t LandmarkTracker::bearing_bin_mask(
  const Eigen::Vector2d & landmark_xy, const Eigen::Vector2d & robot_xy) const
{
  const int nbins = std::clamp(params_.parallax_bin_count, 1, 32);
  const double bearing = std::atan2(
    robot_xy.y() - landmark_xy.y(), robot_xy.x() - landmark_xy.x());  // tronco→robô
  int bin = static_cast<int>(std::floor((bearing + M_PI) / (2.0 * M_PI) * nbins));
  bin = ((bin % nbins) + nbins) % nbins;  // wrap [0, nbins)
  return 1u << bin;
}

void LandmarkTracker::update_score(
  LandmarkTrack & t, const TreeDetection & d, const Eigen::Vector2d & robot_xy) const
{
  // novidade = ver de um ÂNGULO novo (paralaxe). Robô parado → mesmo bin → ~0.1.
  const std::uint32_t mask = bearing_bin_mask(t.xy, robot_xy);
  const bool new_bin = !(t.parallax_bins & mask);
  const double novelty = new_bin ? 1.0 : params_.score_novelty_repeat;

  // consistência = a obs bate na estimativa atual? (resíduo ANTES do fuse)
  const double residual = (Eigen::Vector2d(d.x, d.y) - t.xy).norm();
  const double consistency =
    std::exp(-residual / std::max(1e-3, params_.score_consistency_sigma_m));

  const double quality = std::clamp(static_cast<double>(d.confidence), 0.0, 1.0);
  const double credit = quality * novelty * consistency;

  t.score += params_.score_gain * credit * (1.0 - t.score);  // cresce devagar até 1
  if (residual > params_.score_inconsistent_residual_m) {
    t.score -= params_.score_penalty;  // posição ainda incerta → desce
  }
  t.score = std::clamp(t.score, 0.0, 1.0);
  t.parallax_bins |= mask;
}

void LandmarkTracker::try_promote(LandmarkTrack & track) const
{
  if (track.committed_class != kCommittedUnknown) {
    return;
  }
  if (track.n_observations < params_.promote_min_obs) {
    return;
  }
  // NOTA: promoção = inclusão no BACKEND (fatores que constrangem a pose) → tem de
  // ser FROUXA (n_obs + classe), senão o backend esfomeia e a pose deriva. O gate
  // S + paralaxe NÃO entra aqui; ele decide a CONFIRMAÇÃO (verde/inventário) a
  // jusante (ver is_confirmed()). Separação existência/localização.

  const Eigen::Vector3d posterior = class_posterior_fused(track);
  Eigen::Index best = 0;
  Eigen::Index second = 1;
  if (posterior[second] > posterior[best]) {
    std::swap(best, second);
  }
  for (Eigen::Index i = 2; i < 3; ++i) {
    if (posterior[i] > posterior[best]) {
      second = best;
      best = i;
    } else if (posterior[i] > posterior[second]) {
      second = i;
    }
  }

  if (posterior[best] < params_.promote_prob) {
    return;
  }
  if ((posterior[best] - posterior[second]) < params_.promote_margin) {
    return;
  }
  track.committed_class = committed_from_score_index(static_cast<int>(best));
}

void LandmarkTracker::merge_class_state(LandmarkTrack & kept, const LandmarkTrack & removed) const
{
  kept.class_logodds += removed.class_logodds;
  kept.class_logodds_cam += removed.class_logodds_cam;
  kept.class_coverage_bits |= removed.class_coverage_bits;
  kept.last_multiview_class_coverage_ =
    std::max(kept.last_multiview_class_coverage_, removed.last_multiview_class_coverage_);
  if (kept.committed_class == kCommittedUnknown && removed.committed_class != kCommittedUnknown) {
    kept.committed_class = removed.committed_class;
  } else if (
    kept.committed_class != kCommittedUnknown && removed.committed_class != kCommittedUnknown &&
    kept.committed_class != removed.committed_class)
  {
    // Conflito raro: mantém a classe do track com mais observações (já decidido
    // pelo merge geométrico — aqui só salvaguarda consistência).
    kept.committed_class = kept.n_observations >= removed.n_observations ?
      kept.committed_class :
      removed.committed_class;
  }
}

Eigen::Matrix2d LandmarkTracker::detection_covariance(const TreeDetection & d) const
{
  Eigen::Matrix2d cov = d.base_covariance.topLeftCorner<2, 2>();
  // base_covariance toda a zeros => perceção ainda não preenche incerteza;
  // usa um fallback fixo para não colapsar o gate de Mahalanobis a zero.
  if (cov.isZero(1e-12)) {
    cov = Eigen::Matrix2d::Identity() * params_.fallback_detection_var_xy;
  }
  return cov;
}

double LandmarkTracker::detection_diameter_variance(const TreeDetection & d) const
{
  double sigma = static_cast<double>(d.diameter_stddev);
  if (sigma < 1e-6) {
    sigma = static_cast<double>(params_.fallback_diameter_stddev);
  }
  return sigma * sigma;
}

void LandmarkTracker::fuse_position(LandmarkTrack & t, const TreeDetection & d) const
{
  const Eigen::Matrix2d det_cov = detection_covariance(d);
  const Eigen::Matrix2d prior_prec = t.cov.inverse();
  const Eigen::Matrix2d det_prec = det_cov.inverse();
  const Eigen::Matrix2d post_cov = (prior_prec + det_prec).inverse();
  const Eigen::Vector2d z(d.x, d.y);
  t.xy = post_cov * (prior_prec * t.xy + det_prec * z);
  t.cov = post_cov;
}

void LandmarkTracker::fuse_position_covariance_only(
  LandmarkTrack & t, const TreeDetection & d) const
{
  // Só a covariância (gate). A média (t.xy) é do backend — ver sync_landmark_anchors.
  const Eigen::Matrix2d det_cov = detection_covariance(d);
  const Eigen::Matrix2d prior_prec = t.cov.inverse();
  const Eigen::Matrix2d det_prec = det_cov.inverse();
  t.cov = (prior_prec + det_prec).inverse();
}

void LandmarkTracker::fuse_diameter(LandmarkTrack & t, const TreeDetection & d) const
{
  const double det_var = detection_diameter_variance(d);
  const double prior_var = std::max(t.diameter_var, 1e-8);
  const double post_var = 1.0 / (1.0 / prior_var + 1.0 / det_var);
  t.diameter = post_var * (t.diameter / prior_var + d.diameter / det_var);
  t.diameter_var = post_var;
}

bool LandmarkTracker::mahalanobis_gate_ok(const LandmarkTrack & t, const TreeDetection & d) const
{
  const Eigen::Vector2d delta = Eigen::Vector2d(d.x, d.y) - t.xy;
  const Eigen::Matrix2d innovation_cov = t.cov + detection_covariance(d);
  const Eigen::Matrix2d inv = innovation_cov.inverse();
  const double m2 = delta.transpose() * inv * delta;
  return m2 <= params_.mahalanobis_gate_sq;
}

double LandmarkTracker::cost(const LandmarkTrack & t, const TreeDetection & d) const
{
  const double dxy = (Eigen::Vector2d(d.x, d.y) - t.xy).norm();
  const double dr = std::abs(d.diameter - t.diameter);
  return params_.weight_xy * dxy + params_.weight_radius * dr;
}

TrackerUpdateReport LandmarkTracker::update(
  const std::vector<TreeDetection> & detections_world, double stamp_sec,
  const Eigen::Vector2d & robot_xy, double angular_delta_rad)
{
  TrackerUpdateReport report;
  report.detection_to_uid.assign(detections_world.size(), 0);

  const int n_tracks = static_cast<int>(tracks_.size());
  const int n_dets = static_cast<int>(detections_world.size());

  // 0. Predição: infla a covariância de cada track antes do gate. Termo
  // isotrópico fixo (jitter mínimo) + termo de braço de alavanca
  // (alcance * Δheading)^2 — quanto mais o robô rodou desde o último scan e
  // mais longe está a árvore, maior a incerteza introduzida na posição-mundo
  // prevista, mesmo que a árvore seja estática.
  for (auto & t : tracks_) {
    if (t.dormant) {
      // Adormecido = ponto de mapa conhecido; a incerteza está na pose do robô,
      // não no landmark. Cov congelada para o gate de posição não abrir sem
      // limite (e não casar tudo por proximidade).
      continue;
    }
    const double range = (t.xy - robot_xy).norm();
    const double lever_arm = range * angular_delta_rad;
    t.cov += Eigen::Matrix2d::Identity() * (params_.process_noise_xy_var + lever_arm * lever_arm);
  }

  // 1. Matriz de custo gated. Linhas = tracks, colunas = deteções (mais
  // tracks habitualmente do que deteções por scan; hungarian_assign aceita
  // qualquer formato e transpõe internamente se necessário).
  std::vector<std::vector<double>> cost_matrix(
    n_tracks, std::vector<double>(n_dets, std::numeric_limits<double>::infinity()));
  for (int i = 0; i < n_tracks; ++i) {
    for (int j = 0; j < n_dets; ++j) {
      if (mahalanobis_gate_ok(tracks_[i], detections_world[j])) {
        cost_matrix[i][j] = cost(tracks_[i], detections_world[j]);
      }
    }
  }

  std::vector<int> track_to_det(n_tracks, -1);
  if (n_tracks > 0 && n_dets > 0) {
    track_to_det = hungarian_assign(cost_matrix);
  }

  std::vector<bool> det_matched(n_dets, false);

  // 2. Update dos tracks associados.
  for (int i = 0; i < n_tracks; ++i) {
    const int j = track_to_det[i];
    auto & t = tracks_[i];
    if (j < 0) {
      // Não associado neste scan.
      ++t.scans_since_seen;
      t.confidence = std::max(0.0, t.confidence - params_.confidence_decay_per_miss);
      continue;
    }
    const auto & d = detections_world[j];
    det_matched[j] = true;
    if (t.dormant) {
      // Re-associado por posição (regresso de baixa deriva) → acorda; é um
      // loop closure (o nó liga a observação ao uid existente).
      t.dormant = false;
      report.reawakened.push_back(t.uid);
    }

    // --- SCORER DINÂMICO (antes do fuse: resíduo vs estimativa ATUAL) --------
    update_score(t, d, robot_xy);

    // UNIFICAÇÃO DOS MAPAS: o committed no grafo tem a POSIÇÃO como propriedade do
    // backend (sync_landmark_anchors ancora-a a cada scan). O tracker não a filtra
    // — evita a autoridade dupla que gerava duplicados/fantasmas. Só atualiza a
    // covariância para o gate. Candidatos (ainda fora do grafo) filtram normalmente.
    if (is_committed_graph(t)) {
      fuse_position_covariance_only(t, d);
    } else {
      fuse_position(t, d);
    }
    fuse_diameter(t, d);
    accumulate_class_scores(t, d, robot_xy);
    // Confiança de qualidade (EMA) — mantida como sinal interno; a confiança
    // PUBLICADA e o gate de promoção são agora o `score` (S) acima.
    t.confidence = (1.0 - params_.confidence_gain) * t.confidence +
                   params_.confidence_gain * d.confidence;
    t.n_observations++;
    try_promote(t);
    t.scans_since_seen = 0;
    t.last_seen_stamp_sec = stamp_sec;
    report.detection_to_uid[j] = t.uid;
  }

  // 2b. Re-associação por GEOMETRIA DE VIZINHANÇA (loop closure no solo):
  // antes de dar à luz uids novos, tenta casar as deteções ainda livres ao
  // mapa de landmarks (incl. adormecidos) pelo padrão geométrico local.
  if (params_.enable_geometric_reassoc) {
    geometric_reassociate(detections_world, stamp_sec, robot_xy, det_matched, report);
  }

  // 3. Birth — deteções não associadas com confiança suficiente.
  for (int j = 0; j < n_dets; ++j) {
    if (det_matched[j]) {
      continue;
    }
    const auto & d = detections_world[j];
    if (d.confidence < params_.birth_confidence) {
      continue;
    }
    // Gate de ALCANCE no nascimento: longe, o bearing×range e a deteção de tronco
    // degradam → fantasmas. Além de birth_max_range_m não CRIA landmark (mas uma
    // deteção longe ainda pôde ATUALIZAR um landmark existente acima, na associação).
    if ((Eigen::Vector2d(d.x, d.y) - robot_xy).norm() > params_.birth_max_range_m) {
      continue;
    }
    LandmarkTrack t;
    t.uid = next_uid_++;
    t.xy = Eigen::Vector2d(d.x, d.y);
    t.diameter = d.diameter;
    t.cov = detection_covariance(d);
    t.diameter_var = detection_diameter_variance(d);
    t.confidence = d.confidence;
    // Nasce com S=0 (uma só vista não dá fiabilidade); regista o 1.º ângulo de
    // paralaxe. S cresce devagar nas associações seguintes (vistas novas).
    t.score = 0.0;
    t.parallax_bins = bearing_bin_mask(t.xy, robot_xy);
    t.n_observations = 1;
    t.scans_since_seen = 0;
    t.last_seen_stamp_sec = stamp_sec;
    t.committed_class = kCommittedUnknown;
    t.multiview_buffer = MultiviewPointBuffer(params_.multiview);
    accumulate_class_scores(t, d, robot_xy);
    tracks_.push_back(t);
    report.births.push_back(t.uid);
    report.detection_to_uid[j] = t.uid;
  }

  // 4. Dormência / cull. Landmarks PROMOVIDOS no grafo nunca são apagados:
  // ao passar `death_age_scans` sem associação ADORMECEM (mapa persistente,
  // azul no RViz, elegíveis para loop closure). Apenas CANDIDATOS ainda não
  // promovidos (lixo) são apagados, e mais tarde (cull_unpromoted_after_scans),
  // para controlar memória sem perder o mapa.
  for (auto & t : tracks_) {
    if (!t.dormant && t.scans_since_seen > params_.death_age_scans &&
      is_promoted(t) && is_slam_graph_class(t.committed_class))
    {
      t.dormant = true;
      // Congela a covariância num valor de mapa são (ver dormant_position_var).
      t.cov = Eigen::Matrix2d::Identity() * params_.dormant_position_var;
    }
  }
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [&](const LandmarkTrack & t) {
        const bool graph_landmark = is_promoted(t) && is_slam_graph_class(t.committed_class);
        const bool cull = !graph_landmark &&
        t.scans_since_seen > params_.cull_unpromoted_after_scans;
        if (cull) {
          report.deaths.push_back(t.uid);
        }
        return cull;
      }),
    tracks_.end());

  // 5. Merge — tracks demasiado próximos fundem-se; mantém-se o uid mais antigo
  // (mais observações). Só geometria — sem classe. Duas condições:
  //  (a) COINCIDENTES (< merge_coincident_dist_m): mesma árvore, funde mesmo
  //      com DBH divergente (de-duplicação dominada por posição);
  //  (b) próximos (< merge_dist_m) E com DBH semelhante (< merge_diameter_m).
  for (std::size_t a = 0; a < tracks_.size(); ++a) {
    for (std::size_t b = a + 1; b < tracks_.size(); ) {
      const double dxy = (tracks_[a].xy - tracks_[b].xy).norm();
      const double dr = std::abs(tracks_[a].diameter - tracks_[b].diameter);
      const bool coincident = dxy < params_.merge_coincident_dist_m;
      const bool near_same_dbh = dxy < params_.merge_dist_m && dr < params_.merge_diameter_m;
      if (coincident || near_same_dbh) {
        const bool a_older = tracks_[a].n_observations >= tracks_[b].n_observations;
        LandmarkUid kept = a_older ? tracks_[a].uid : tracks_[b].uid;
        LandmarkUid removed = a_older ? tracks_[b].uid : tracks_[a].uid;
        if (a_older) {
          merge_class_state(tracks_[a], tracks_[b]);
          tracks_[a].n_observations += tracks_[b].n_observations;
          tracks_[a].confidence = std::max(tracks_[a].confidence, tracks_[b].confidence);
          fuse_diameter(tracks_[a], TreeDetection{
              tracks_[a].xy.x(), tracks_[a].xy.y(), tracks_[b].diameter, 1.0F,
              Eigen::Matrix3d::Zero(),
              static_cast<float>(std::sqrt(std::max(tracks_[b].diameter_var, 1e-8)))});
        } else {
          merge_class_state(tracks_[b], tracks_[a]);
          tracks_[b].n_observations += tracks_[a].n_observations;
          tracks_[b].confidence = std::max(tracks_[a].confidence, tracks_[b].confidence);
          fuse_diameter(tracks_[b], TreeDetection{
              tracks_[b].xy.x(), tracks_[b].xy.y(), tracks_[a].diameter, 1.0F,
              Eigen::Matrix3d::Zero(),
              static_cast<float>(std::sqrt(std::max(tracks_[a].diameter_var, 1e-8)))});
          tracks_[a] = tracks_[b];
        }
        tracks_.erase(tracks_.begin() + static_cast<long>(b));
        report.merges.emplace_back(kept, removed);
        // Atualiza uids reportados às deteções deste scan que apontavam para o removido.
        for (auto & uid : report.detection_to_uid) {
          if (uid == removed) {
            uid = kept;
          }
        }
        continue;  // não incrementa b: novo elemento ocupa a posição
      }
      ++b;
    }
  }

  return report;
}

void LandmarkTracker::geometric_reassociate(
  const std::vector<TreeDetection> & detections_world, double stamp_sec,
  const Eigen::Vector2d & robot_xy, std::vector<bool> & det_matched,
  TrackerUpdateReport & report)
{
  const int n_dets = static_cast<int>(detections_world.size());
  if (n_dets < params_.geo_min_query) {
    return;  // triângulos precisam de ≥3; exige uma constelação local mínima.
  }
  int n_unmatched = 0;
  for (int j = 0; j < n_dets; ++j) {
    if (!det_matched[j]) {
      ++n_unmatched;
    }
  }
  if (n_unmatched == 0) {
    return;  // nada por re-associar.
  }

  // Mapa = landmarks promovidos no grafo (ativos + adormecidos). É o conjunto
  // persistente contra o qual casamos o padrão geométrico.
  std::vector<LandmarkPoint> map_points;
  std::size_t n_dormant = 0;
  for (const auto & t : tracks_) {
    if (!is_promoted(t) || !is_slam_graph_class(t.committed_class)) {
      continue;
    }
    map_points.push_back(LandmarkPoint{t.uid, t.xy.x(), t.xy.y(), t.diameter});
    if (t.dormant) {
      ++n_dormant;
    }
  }
  // Associação inter-troncos como via PRIMÁRIA (não só fallback de adormecidos):
  // corre sempre que há mapa suficiente e deteções por associar. Um landmark
  // ATIVO cuja deteção derivou para fora do gate de posição é re-associado por
  // geometria de vizinhança (constelação/triângulos, robusta à deriva) em vez de
  // gerar um duplicado — que o loop closure não desfaz. (n_dormant fica só para
  // diagnóstico.) O early-return `n_unmatched == 0` acima evita o custo quando o
  // gate de posição já casou tudo (tracking contínuo sem deriva).
  if (static_cast<int>(map_points.size()) < params_.ground_reloc.min_correspondences) {
    return;
  }

  // Query = TODAS as deteções deste scan (mais triângulos = match mais robusto);
  // a posição é a do frame de trabalho (com deriva), mas o relocalizador casa
  // por geometria RELATIVA, logo a deriva global é absorvida pela SE2.
  std::vector<LandmarkPoint> query;
  query.reserve(detections_world.size());
  for (const auto & d : detections_world) {
    query.push_back(LandmarkPoint{0, d.x, d.y, d.diameter});
  }

  const auto result = ground_relocalizer_.relocalize(query, map_points);
  if (std::getenv("FOREST_GEO_DEBUG")) {
    std::fprintf(
      stderr,
      "[geo] dets=%d unmatched=%d dormant=%zu map=%zu -> accepted=%d overlap=%.2f "
      "inliers=%zu residual=%.2f\n",
      n_dets, n_unmatched, n_dormant, map_points.size(), result.accepted ? 1 : 0,
      result.overlap_ratio, result.correspondences.size(), result.mean_residual_m);
  }
  if (!result.accepted) {
    return;
  }

  // uids já usados neste scan (stage-A ou births anteriores) — não reatribuir.
  std::vector<LandmarkUid> used_uids = report.detection_to_uid;
  auto already_used = [&](LandmarkUid uid) {
      return std::find(used_uids.begin(), used_uids.end(), uid) != used_uids.end();
    };

  for (const auto & c : result.correspondences) {
    const auto j = static_cast<int>(c.query_index);
    if (j < 0 || j >= n_dets || det_matched[j]) {
      continue;  // só deteções ainda livres.
    }
    if (already_used(c.map_uid)) {
      continue;  // 1 deteção por landmark por scan.
    }
    auto it = std::find_if(
      tracks_.begin(), tracks_.end(),
      [&](const LandmarkTrack & t) {return t.uid == c.map_uid;});
    if (it == tracks_.end()) {
      continue;
    }
    LandmarkTrack & t = *it;
    const auto & d = detections_world[j];
    det_matched[j] = true;
    if (t.dormant) {
      t.dormant = false;
      report.reawakened.push_back(t.uid);
    }
    // NÃO fundir a posição: a deteção tem deriva; a posição do landmark é o
    // estado bom do mapa. A reconciliação geométrica fica a cargo do fator
    // bearing-range que o nó adiciona ao uid existente (loop closure). Diâmetro
    // e classe são invariantes ao frame, logo fundem-se com segurança.
    fuse_diameter(t, d);
    accumulate_class_scores(t, d, robot_xy);
    // Confiança = média móvel (EMA) da QUALIDADE das deteções, não rampa por
    // contagem (que saturava tudo a 1.0). Um landmark visto muitas vezes com
    // deteções fracas (longe/arco parcial/sem base) mantém confiança baixa.
    t.confidence = (1.0 - params_.confidence_gain) * t.confidence +
                   params_.confidence_gain * d.confidence;
    t.n_observations++;
    t.scans_since_seen = 0;
    t.last_seen_stamp_sec = stamp_sec;
    report.detection_to_uid[j] = t.uid;
    used_uids.push_back(t.uid);
  }
}

void LandmarkTracker::sync_landmark_anchors(
  const std::unordered_map<LandmarkUid, Eigen::Vector2d> & backend_positions)
{
  for (auto & t : tracks_) {
    // UNIFICAÇÃO DOS MAPAS: o backend é a autoridade da posição de TODOS os
    // committed no grafo — ativos E adormecidos. Antes só se sincronizavam os
    // adormecidos e os ativos mantinham uma estimativa filtrada própria que
    // derivava da do grafo (autoridade dupla → duplicados/fantasmas no RViz).
    // Agora o tracker não filtra a posição dos committed (ver update()), logo não
    // há estimativa concorrente a proteger; a covariância continua a alimentar o
    // gate (predição + fuse_position_covariance_only), aplicada sobre a posição
    // correta do grafo.
    if (!is_promoted(t) || !is_slam_graph_class(t.committed_class)) {
      continue;
    }
    const auto it = backend_positions.find(t.uid);
    if (it != backend_positions.end()) {
      t.xy = it->second;
    }
  }
}

const LandmarkTrack * LandmarkTracker::find_track(LandmarkUid uid) const
{
  for (const auto & t : tracks_) {
    if (t.uid == uid) {
      return &t;
    }
  }
  return nullptr;
}

bool LandmarkTracker::eligible_for_multiview(
  const LandmarkTrack & track, const TreeDetection & detection) const
{
  if (!detection.has_stem_inliers) {
    return false;
  }
  if (track.committed_class == kCommittedRock || track.committed_class == kCommittedObstacle) {
    return false;
  }
  // Classe: tem de parecer tronco (a menos que já comprometido como tronco).
  if (track.committed_class != kCommittedTrunk) {
    const bool looks_trunk =
      detection.class_scores[kScoreTrunk] >= detection.class_scores[kScoreRock] &&
      detection.class_scores[kScoreTrunk] >= detection.class_scores[kScoreObstacle];
    if (!looks_trunk) {
      return false;
    }
  }

  // (B) Qualidade por-frame absoluta: arco parcial / ramos baixos inflam o
  // diameter_stddev da perceção. Sempre aplicável (não depende do histórico).
  if (detection.diameter_stddev > params_.multiview_gate_max_diameter_stddev_m) {
    return false;
  }

  // Gates relativos: só depois de o landmark estar estável. No arranque não há
  // referência fiável e bloqueá-los-ia (track.xy/diameter ainda enviesados).
  const bool stable = track.n_observations >= params_.promote_min_obs;
  if (stable) {
    // (A) Resíduo de pose: deteção deste scan vs landmark acumulado.
    const double center_residual =
      (Eigen::Vector2d(detection.x, detection.y) - track.xy).norm();
    if (center_residual > params_.multiview_gate_max_center_residual_m) {
      return false;
    }
    // (B) Desvio relativo do DBH por-frame face ao acumulado.
    if (track.diameter > 0.05 && detection.diameter > 0.0) {
      const double rel_dev = std::abs(detection.diameter - track.diameter) / track.diameter;
      if (rel_dev > params_.multiview_gate_max_diameter_rel_dev) {
        return false;
      }
    }
  }

  return true;
}

void LandmarkTracker::apply_multiview_refit(LandmarkTrack & track)
{
  // Ancora o refit aparado no prior (estimativa acumulada) quando o landmark já
  // está estabelecido — assim um anel deslocado de um frame mau é rejeitado pelo
  // fit em vez de o explodir. No arranque (sem prior fiável) arranca de todos.
  const bool has_prior = track.n_observations >= params_.promote_min_obs && track.diameter > 0.05;
  const DbhRefitResult refit = track.multiview_buffer.refit(
    track.xy.x(), track.xy.y(), 0.5 * track.diameter, has_prior);
  if (!refit.valid) {
    return;
  }
  track.xy = Eigen::Vector2d(refit.cx, refit.cy);
  track.diameter = 2.0 * refit.radius;
  const double arc = std::max(0.05, static_cast<double>(refit.arc_coverage));
  const double range = track.xy.norm();
  const double sigma = std::max(0.01, refit.rmse / arc + 0.002 * range);
  track.diameter_var = sigma * sigma;
}

void LandmarkTracker::accumulate_multiview_class_scores(LandmarkTrack & track)
{
  if (track.multiview_buffer.saturated()) {
    return;
  }
  if (track.multiview_buffer.n_voxels() < params_.multiview.refit_min_points) {
    return;
  }

  const double coverage = track.multiview_buffer.coverage_ratio();
  if (coverage < params_.multiview_class_min_coverage) {
    return;
  }
  if (coverage < track.last_multiview_class_coverage_ + params_.multiview_class_coverage_step) {
    return;
  }

  const auto scored =
    forest_3d_perception::score_landmark_points_map(track.multiview_buffer.points());
  if (!scored.valid) {
    return;
  }

  const float eps = params_.class_log_eps;
  const double weight = static_cast<double>(params_.multiview_class_obs_weight);
  for (std::size_t i = 0; i < kNumClassScores; ++i) {
    track.class_logodds[static_cast<Eigen::Index>(i)] +=
      weight * std::log(clamp_prob(scored.scores[i], eps));
  }
  track.last_multiview_class_coverage_ = coverage;
  try_promote(track);
}

void LandmarkTracker::ingest_multiview_inliers(
  LandmarkUid uid, const std::vector<Eigen::Vector3d> & points_map,
  const Eigen::Vector2d & robot_xy, const TreeDetection & detection)
{
  for (auto & track : tracks_) {
    if (track.uid != uid) {
      continue;
    }
    if (!eligible_for_multiview(track, detection) || track.multiview_buffer.saturated()) {
      return;
    }
    if (points_map.empty()) {
      return;
    }
    // (D) Gate de POSE CORRIGIDA: enquanto o SLAM não fixar a posição do landmark
    // (covariância de posição ainda larga, ou poucas observações), a árvore "anda"
    // entre scans e os pontos entrariam em coordenadas-MUNDO erradas — poluindo o
    // buffer permanente. Só ingere quando track.cov já encolheu (EKF convergiu) e
    // há observações suficientes. A estimativa por-frame (fuse_diameter) carrega o
    // DBH durante esta espera; a referência de pontos só começa com pose fiável.
    if (track.n_observations < params_.multiview_gate_min_obs ||
      track.cov.trace() > params_.multiview_gate_max_pos_var)
    {
      return;
    }
    // (C) Gate de consistência geométrica por-frame: quando o fit já é confiável,
    // os pontos deste frame têm de assentar no círculo acumulado. Rejeita frames
    // com pose derivada / contaminação ANTES de entrarem no buffer permanente.
    const bool fit_confident = track.n_observations >= params_.promote_min_obs &&
      track.diameter_var <= params_.multiview_gate_confident_var && track.diameter > 0.05;
    if (fit_confident) {
      const double r = 0.5 * track.diameter;
      std::vector<double> res;
      res.reserve(points_map.size());
      for (const auto & p : points_map) {
        res.push_back(std::abs((Eigen::Vector2d(p.x(), p.y()) - track.xy).norm() - r));
      }
      std::nth_element(res.begin(), res.begin() + res.size() / 2, res.end());
      const double median_res = res[res.size() / 2];
      if (median_res > params_.multiview_gate_point_consistency_tol_m) {
        return;
      }
    }
    const bool changed =
      track.multiview_buffer.insert_frame(points_map, track.xy, robot_xy);
    if (!changed) {
      return;
    }
    apply_multiview_refit(track);
    accumulate_multiview_class_scores(track);
    if (track.multiview_buffer.should_saturate(track.diameter_var)) {
      track.multiview_buffer.saturate();
    }
    return;
  }
}

}  // namespace forest_tree_slam
