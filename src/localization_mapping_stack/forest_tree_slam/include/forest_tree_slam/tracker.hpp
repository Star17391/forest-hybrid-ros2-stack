#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "forest_tree_slam/landmark_class.hpp"
#include "forest_tree_slam/multiview_dbh.hpp"
#include "forest_tree_slam/relocalizer.hpp"
#include "forest_tree_slam/types.hpp"

namespace forest_tree_slam
{

struct TrackerParams
{
  // Gate de Mahalanobis (distância^2) entre deteção e track previsto. Valor
  // qui-quadrado para 2 DoF, ~99% (5.991 a 95%, 9.21 a 99%).
  double mahalanobis_gate_sq{9.21};

  // Custo C_ij = w_xy * ||Δxy|| + w_r * |Δr|  (FOREST_TREE_SLAM_DESIGN.md §5.1).
  double weight_xy{1.0};
  double weight_radius{0.5};

  // Covariância de deteção mínima (m^2) quando a perceção não preenche
  // base_covariance (zeros) — evita gate infinitamente apertado.
  double fallback_detection_var_xy{0.04};      // 20 cm stddev
  float fallback_diameter_stddev{0.03F};        // 3 cm

  // Birth: confiança mínima de uma deteção não associada para abrir um track.
  double birth_confidence{0.3};
  // Birth: NÃO nascer landmark de deteções além deste alcance (m). A longa
  // distância o bearing×range e a deteção de tronco degradam → nasciam fantasmas.
  // Deteções além disto ainda ATUALIZAM landmarks existentes (associação), só não
  // CRIAM novos. (Ver trunk_range_diagnosis: fiável até ~8 m.)
  double birth_max_range_m{8.0};

  // --- SCORER DINÂMICO (S) ---------------------------------------------------
  // S em [0,1]: confiança de fiabilidade que CRESCE DEVAGAR com evidência boa e
  // DESCE com medições fracas/incoerentes. credit = qualidade × novidade ×
  // consistência; S += score_gain·credit·(1−S). A "novidade" depende de ver o
  // tronco de ÂNGULOS NOVOS (paralaxe) → robô parado não satura a 1.
  double score_gain{0.15};                 // α: crescimento lento
  double score_novelty_repeat{0.1};        // crédito de um ângulo já visto (vs 1.0 novo)
  double score_consistency_sigma_m{0.30};  // σ da consistência: exp(-resíduo/σ)
  double score_inconsistent_residual_m{0.5};  // resíduo acima do qual PENALIZA
  double score_penalty{0.10};              // β: quanto S desce numa obs incoerente
  // Paralaxe: bins do bearing tronco→robô (360°/n). Promove só com ≥ N bins
  // distintos (≥ N×passo de cobertura angular).
  int parallax_bin_count{24};              // 24 bins de 15°
  int promote_min_parallax_bins{4};        // ≥4 bins ≈ ≥60° de vistas distintas
  // Promoção: S mínimo (existência) — a localização é garantida pela paralaxe.
  double promote_score_min{0.5};

  // Dormência: nº de scans consecutivos sem associação antes de um track
  // PROMOVIDO (tronco/rocha, no grafo) passar a ADORMECIDO — fica no mapa
  // persistente (azul no RViz), elegível para re-associação por geometria
  // (loop closure), em vez de ser apagado. Mantém o nome histórico.
  int death_age_scans{8};

  // Candidatos NÃO promovidos (ainda kCommittedUnknown) são lixo se ficarem
  // muito tempo sem confirmar — esses SIM são apagados (controlo de memória).
  // Landmarks promovidos no grafo nunca são apagados (mapa persistente).
  int cull_unpromoted_after_scans{20};

  // Ao adormecer, a posição do landmark é "conhecida" (estado do grafo); a
  // incerteza passa a estar na pose do robô, não no landmark. Congela-se a
  // covariância do landmark neste valor para o gate de posição continuar são
  // num regresso de baixa deriva (não cresce sem limite enquanto adormecido).
  double dormant_position_var{0.09};   // (0.3 m)^2

  // --- Re-associação por GEOMETRIA DE VIZINHANÇA (loop closure no solo) ---
  // Quando o gate de posição falha (deriva) mas o robô regressa a uma zona já
  // mapeada, casa-se o padrão geométrico local (constelação/triângulos, via
  // relocalizador) das deteções deste scan ao mapa de landmarks. Robusto à
  // deriva porque usa distâncias RELATIVAS entre landmarks, não a posição
  // absoluta prevista pela odometria.
  bool enable_geometric_reassoc{true};
  // Nº mínimo de troncos visíveis no scan para tentar (triângulos precisam ≥3).
  int geo_min_query{4};
  // Parâmetros do relocalizador para uso no solo (mais permissivos que o
  // pós-salto-aéreo, mas com margem: um match errado é catastrófico).
  RelocalizerParams ground_reloc{
    /*n_radial_bins*/ 5, /*n_diameter_bins*/ 8, /*radial_bin_max_m*/ 15.0,
    /*diameter_bin_max_m*/ 1.5, /*top_n_coarse*/ 8,
    /*triangle_side_tolerance_m*/ 0.4, /*top_n_fine*/ 4, /*min_triangle_side_m*/ 0.5,
    /*planar_residual_threshold_m*/ 0.4, /*diameter_residual_threshold_m*/ 0.2,
    /*min_overlap_ratio*/ 0.5, /*min_correspondences*/ 4};

  // Merge: tracks mais próximos que isto (m) e com DBH semelhante fundem-se.
  double merge_dist_m{0.4};
  double merge_diameter_m{0.15};
  // De-duplicação dominada por POSIÇÃO: tracks COINCIDENTES (mais próximos que
  // isto) fundem-se INDEPENDENTEMENTE do DBH. Sem isto, dois landmarks no mesmo
  // sítio com DBH inchado/divergente (|Δdiam|>merge_diameter_m) sobrevivem como
  // duplicados no mapa persistente (medido no sim: uid 1/3 a 0.14 m, diâmetros
  // 0.87 vs 0.60). A 0.25 m é fisicamente a MESMA árvore.
  double merge_coincident_dist_m{0.25};

  // Incremento de confiança por observação associada; decaimento por scan perdido.
  double confidence_gain{0.15};
  double confidence_decay_per_miss{0.08};

  // Passo de predição (tipo EKF) antes do gate de Mahalanobis: a covariância
  // de cada track cresce entre scans para refletir a incerteza acumulada da
  // odometria. Sem isto, uma rotação rápida do robô desloca lateralmente a
  // posição-mundo prevista de cada árvore (erro ~ alcance * Δheading, efeito
  // de braço de alavanca) sem que o gate saiba — falha o gate, nasce um track
  // duplicado, e o antigo só morre `death_age_scans` scans depois.
  double process_noise_xy_var{0.0005};  // m^2 por scan, mesmo parado (jitter mínimo)

  // --- Acumulação probabilística de classe (log-odds) ---
  float class_log_eps{1.0e-4F};
  double class_min_bearing_delta_rad{0.15};   // ~8.6° — vista nova se Δbearing ≥ isto
  int class_coverage_bins{36};                // bins de 10° para anti-correlação
  double class_correlated_obs_weight{0.15};     // peso atenuado na mesma vista

  // Promoção candidato → comprometido (posterior = softmax(class_logodds)).
  double promote_prob{0.70};
  double promote_margin{0.20};
  std::uint32_t promote_min_obs{4};

  // Fusão de classe da câmara (F3). O LiDAR satura a posterior num teto
  // `fusion_class_c_lidar` (o resto vem da câmara); `fusion_class_w_cam` pesa o
  // termo da câmara em log-odds. Com `fusion_class_w_cam=0`, a posterior fundida
  // é idêntica à do LiDAR (regressão limpa / kill-switch).
  double fusion_class_c_lidar{0.85};
  double fusion_class_w_cam{1.0};
  double fusion_class_cam_min_conf{0.40};

  MultiviewDbhParams multiview{};

  // --- Gate de ingestão multi-vista (decide se um frame ENTRA no buffer) ---
  // O buffer é permanente (saturate() não limpa): um único frame com pose
  // derivada deposita um anel deslocado em map que fica lá para sempre e faz o
  // refit dar um raio gigante (medido: DBH a saltar p/ ~1.5 m a meio da órbita,
  // quando o robô passa os 90° e o SLAM perde a árvore). Por isso filtramos à
  // ENTRADA, não só no fit. Dois critérios:
  //   (A) Resíduo de pose: o centro da deteção deste scan (calculado com a pose
  //       possivelmente derivada) tem de ficar perto do landmark acumulado.
  //   (B) Qualidade por-frame: rejeita arco parcial / contaminação de ramos
  //       (Tree1/Tree5) por stddev alto do DBH e por desvio relativo do DBH.
  // Os gates relativos (A e desvio de DBH) só ligam depois de o landmark ter
  // posição/diâmetro estáveis (>= promote_min_obs); senão bloqueariam o arranque.
  double multiview_gate_max_center_residual_m{0.15};
  double multiview_gate_max_diameter_stddev_m{0.20};
  double multiview_gate_max_diameter_rel_dev{0.5};
  // (D) Gate de POSE corrigida: só ingere quando a posição do landmark já está
  // fixada pelo SLAM. multiview_gate_max_pos_var = traço máx. de track.cov (m²,
  // soma var_x+var_y); multiview_gate_min_obs = chão de observações (warm-up).
  // Calibrado no sim: trace ~0.0046 na fase NÃO-corrigida (início + meio da órbita
  // quando perde tracking, diâmetro enviesado) vs ~0.0017 corrigido (diâmetro=GT).
  // 0.0025 admite só o regime corrigido e rejeita ambas as fases más.
  double multiview_gate_max_pos_var{0.0025};
  std::uint32_t multiview_gate_min_obs{6};
  // (C) Consistência geométrica POR-FRAME: o gate (A) só olha para o CENTRO da
  // deteção, mas a corrupção mede-se na nuvem — um frame com pose derivada traz
  // pontos que não assentam no círculo já convergido. Quando o landmark tem fit
  // confiável (var pequena), exige-se que a mediana de |dist(p, centro) − raio|
  // dos pontos do frame seja ≤ isto; senão o frame é rejeitado ANTES de poluir o
  // buffer permanente. Na fase de arranque (var grande) não gateia (sem círculo
  // fiável). Foi o que apanhou o frame mau que escapava ao (A) e explodia o DBH.
  double multiview_gate_point_consistency_tol_m{0.06};
  double multiview_gate_confident_var{0.0025};  // (5 cm σ)² — fit "confiável"

  // S-F: scorer sobre nuvem acumulada multi-view.
  double multiview_class_min_coverage{0.20};
  double multiview_class_coverage_step{0.05};
  float multiview_class_obs_weight{1.0F};
};

struct LandmarkTrack
{
  LandmarkUid uid{0};
  Eigen::Vector2d xy{Eigen::Vector2d::Zero()};   // posição no frame de trabalho do tracker
  double diameter{0.0};
  double diameter_var{0.01};  // variância acumulada do DBH [m²]
  Eigen::Matrix2d cov{Eigen::Matrix2d::Identity() * 0.1};  // cov 2x2 (x,y) acumulada
  double confidence{0.0};
  // S: scorer dinâmico [0,1] (qualidade × novidade/paralaxe × consistência).
  // É a confiança publicada e o gate de promoção (existência). Não decai com
  // misses (só desce em observações fracas/incoerentes).
  double score{0.0};
  // Bitmask dos bins angulares (bearing tronco→robô) já observados — mede a
  // PARALAXE (vistas de ângulos diferentes) p/ a confiança de LOCALIZAÇÃO.
  std::uint32_t parallax_bins{0};
  std::uint32_t n_observations{0};
  int scans_since_seen{0};
  double last_seen_stamp_sec{0.0};
  // Adormecido: promovido mas fora de vista há > death_age_scans. Persiste no
  // mapa (azul no RViz), com covariância congelada, elegível para loop closure.
  bool dormant{false};

  Eigen::Vector3d class_logodds{Eigen::Vector3d::Zero()};
  // Acumulador SEPARADO da câmara (fusão F3): mantém-se à parte do LiDAR para o
  // cap (o LiDAR satura num teto, a câmara fornece o resto). Índices [tronco,
  // rocha, obstáculo], pós taxonomia 4→3.
  Eigen::Vector3d class_logodds_cam{Eigen::Vector3d::Zero()};
  double last_cam_bearing_rad{std::numeric_limits<double>::quiet_NaN()};
  std::uint8_t committed_class{kCommittedUnknown};
  double last_obs_bearing_rad{0.0};
  std::uint64_t class_coverage_bits{0};

  MultiviewPointBuffer multiview_buffer{MultiviewDbhParams{}};
  double last_multiview_class_coverage_{0.0};
};

// Resultado de uma chamada a `LandmarkTracker::update`: que uids nasceram,
// morreram ou foram fundidos neste scan (para diagnóstico / pose_graph viz).
struct TrackerUpdateReport
{
  std::vector<LandmarkUid> births;
  std::vector<LandmarkUid> deaths;
  std::vector<std::pair<LandmarkUid, LandmarkUid>> merges;  // (mantido, removido)
  // Landmarks adormecidos re-associados neste scan por geometria de vizinhança
  // (= eventos de loop closure no solo; diagnóstico/visualização).
  std::vector<LandmarkUid> reawakened;
  // Por deteção de entrada (mesmo índice de `detections_world`): uid associado,
  // ou 0 se a deteção abriu um novo track (ver `births` para o uid atribuído).
  std::vector<LandmarkUid> detection_to_uid;
};

// Tracker/associação de troncos (FOREST_TREE_SLAM_DESIGN.md §5.1,
// FOREST_SLAM_PERCEPTION_TECHNICAL_ROADMAP.md §3). Agnóstico de ROS: opera em
// coordenadas (x,y) já no frame de trabalho do SLAM (tipicamente a pose
// corrente do grafo); o chamador faz a transformação base_link -> frame.
class LandmarkTracker
{
public:
  explicit LandmarkTracker(TrackerParams params = {})
  : params_(params), ground_relocalizer_(params.ground_reloc)
  {
  }

  // `detections_world`: deteções já transformadas para o frame de trabalho.
  // `stamp_sec`: timestamp do scan (para `last_seen`).
  // `robot_xy`/`angular_delta_rad`: pose e rotação do robô desde o scan
  // anterior (frame de trabalho), usadas só para o passo de predição do gate
  // (braço de alavanca). Omitir mantém o comportamento sem predição.
  TrackerUpdateReport update(
    const std::vector<TreeDetection> & detections_world, double stamp_sec,
    const Eigen::Vector2d & robot_xy = Eigen::Vector2d::Zero(), double angular_delta_rad = 0.0);

  const std::vector<LandmarkTrack> & tracks() const {return tracks_;}

  LandmarkUid next_uid() const {return next_uid_;}

  // Sincroniza a ÂNCORA de associação dos landmarks promovidos (ativos e
  // adormecidos) com as posições OTIMIZADAS do backend — frame global
  // consistente. Sem isto, o tracker associa contra uma cópia desatualizada do
  // mapa (a sua fusão própria, congelada ao adormecer), enquanto o backend já
  // corrigiu as posições por loop closure: no regresso a deteção corrigida cai
  // ao lado da âncora velha, o gate de posição falha e o adormecido não
  // reativa. O backend passa a ser a AUTORIDADE da posição; o tracker mantém
  // classe/DBH/associação. Chamar no início de cada scan, antes de `update`.
  void sync_landmark_anchors(
    const std::unordered_map<LandmarkUid, Eigen::Vector2d> & backend_positions);

  static Eigen::Vector3d class_posterior(const LandmarkTrack & track);

  // Posterior FUNDIDA (F3): satura a do LiDAR num teto `fusion_class_c_lidar` e
  // combina com o log-odds da câmara. Sem termo de câmara, == class_posterior.
  // Não-static porque depende dos params (cap/peso). É esta que decide a promoção.
  Eigen::Vector3d class_posterior_fused(const LandmarkTrack & track) const;

  // Injeta a evidência da câmara para um landmark (chamado pelo nó após associar a
  // deteção LiDAR à caixa do detetor). `class_idx` ∈ {0=tronco,1=rocha,2=obstáculo}
  // (já pós taxonomia 4→3); `conf` é a confiança da câmara; `bearing_rad` é a
  // direção do landmark (anti-correlação: não acumula parado). Re-tenta promover.
  void fuse_camera_class(LandmarkUid uid, int class_idx, double conf, double bearing_rad);

  // PROMOVIDO = entrou no grafo do backend (gate FROUXO: n_obs + classe). Frouxo
  // de propósito para não esfomear o backend (existência).
  static bool is_promoted(const LandmarkTrack & track);
  // CONFIRMADO = elegível para o MAPA/inventário (verde no RViz). Gate EXIGENTE:
  // além de promovido, tem de ter sido visto de ÂNGULOS DISTINTOS (paralaxe ≥
  // promote_min_parallax_bins) e com score de fiabilidade suficiente. Sem isto,
  // uma árvore vista de UM só ângulo (4 frames em linha reta) ia para o mapa.
  bool is_confirmed(const LandmarkTrack & track) const;

  /** Inliers de tree_clusters (map) para um uid associado neste scan. */
  void ingest_multiview_inliers(
    LandmarkUid uid, const std::vector<Eigen::Vector3d> & points_map,
    const Eigen::Vector2d & robot_xy, const TreeDetection & detection);

  const LandmarkTrack * find_track(LandmarkUid uid) const;

private:
  Eigen::Matrix2d detection_covariance(const TreeDetection & d) const;
  double detection_diameter_variance(const TreeDetection & d) const;
  void fuse_position(LandmarkTrack & t, const TreeDetection & d) const;
  // Committed (no grafo): a POSIÇÃO é do backend (sync_landmark_anchors), não se
  // filtra das deteções. Mas a COVARIÂNCIA continua a encolher com a observação
  // (e a inflar com o movimento na predição) para o gate de associação ficar são
  // sob rotação/deriva. Atualiza só t.cov, deixa t.xy intacto.
  void fuse_position_covariance_only(LandmarkTrack & t, const TreeDetection & d) const;
  void fuse_diameter(LandmarkTrack & t, const TreeDetection & d) const;
  // Atualiza o scorer dinâmico S e os bins de paralaxe a partir de uma observação.
  void update_score(
    LandmarkTrack & t, const TreeDetection & d, const Eigen::Vector2d & robot_xy) const;
  // Máscara do bin angular do bearing tronco→robô (paralaxe).
  std::uint32_t bearing_bin_mask(
    const Eigen::Vector2d & landmark_xy, const Eigen::Vector2d & robot_xy) const;
  double cost(const LandmarkTrack & t, const TreeDetection & d) const;
  bool mahalanobis_gate_ok(const LandmarkTrack & t, const TreeDetection & d) const;

  // Re-associação por geometria de vizinhança: para as deteções ainda não
  // associadas pelo gate de posição, casa o padrão geométrico local ao mapa de
  // landmarks (adormecidos + ativos) via relocalizador. Devolve os pares
  // (índice de deteção -> uid existente) aceites com margem. Re-associa
  // (acorda) os tracks no `tracks_` e preenche `report`.
  void geometric_reassociate(
    const std::vector<TreeDetection> & detections_world, double stamp_sec,
    const Eigen::Vector2d & robot_xy, std::vector<bool> & det_matched,
    TrackerUpdateReport & report);

  static bool has_class_scores(const TreeDetection & d);
  int bearing_bin(double bearing_rad) const;
  void accumulate_class_scores(
    LandmarkTrack & track, const TreeDetection & detection,
    const Eigen::Vector2d & robot_xy) const;
  void try_promote(LandmarkTrack & track) const;
  void merge_class_state(LandmarkTrack & kept, const LandmarkTrack & removed) const;
  bool eligible_for_multiview(const LandmarkTrack & track, const TreeDetection & detection) const;
  void apply_multiview_refit(LandmarkTrack & track);
  void accumulate_multiview_class_scores(LandmarkTrack & track);

  TrackerParams params_;
  TreeLocRelocalizer ground_relocalizer_;
  std::vector<LandmarkTrack> tracks_;
  LandmarkUid next_uid_{1};
};

}  // namespace forest_tree_slam
