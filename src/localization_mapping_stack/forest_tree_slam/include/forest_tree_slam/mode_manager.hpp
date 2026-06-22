#pragma once

#include <cstdint>

namespace forest_tree_slam
{

// Espelha forest_hybrid_msgs/msg/SlamStatus::{GROUND,AERIAL,RELOCALIZING,LOST}.
// Não inclui o header do msg para manter este módulo agnóstico de ROS
// (testável offline); o nó faz a tradução.
enum class SlamMode : std::uint8_t
{
  GROUND = 0,
  AERIAL = 1,
  RELOCALIZING = 2,
  LOST = 3,
};

struct ModeManagerParams
{
  // GROUND degradado (congela TF mas continua a publicar) a partir de N scans
  // consecutivos sem nenhuma associação tronco<->track (§8 "floresta densa").
  int degraded_after_scans{15};
  // LOST (deixa de ser autoridade; alerta à missão) — pior que degradado.
  int lost_after_scans{40};
  // Tentativas de relocalização obrigatória antes de desistir e ir para LOST.
  int max_mandatory_relocalization_attempts{3};
};

// Eventos de UMA chamada a `update` — disparados na transição (edge-triggered),
// não a cada tick. O nó reage a estes para orquestrar backend/relocalizer.
struct ModeManagerEvents
{
  bool request_takeoff_snapshot{false};   // GROUND/LOST -> AERIAL: guarda pose pré-salto
  bool request_relocalization{false};     // AERIAL -> RELOCALIZING: corre o TreeLoc
  bool relocalization_mandatory{false};   // válido só quando request_relocalization
};

struct ModeManagerInputs
{
  bool locomotion_aerial{false};      // /system/locomotion_mode == MODE_AERIAL
  bool hop_in_progress{false};        // /forest_gen/hybrid/hop_status == STATE_IN_PROGRESS
  bool hop_done{false};               // hop_status == STATE_DONE (edge tratado internamente)
  bool hop_failed{false};             // hop_status == STATE_FAILED
  bool gnss_good{false};              // cov. GNSS reportada pelo EKF global é baixa (acima do dossel)
  int scans_since_any_association{0};  // do tracker: scans consecutivos com 0 associações
};

// Gestor de modo + transição aérea (FOREST_TREE_SLAM_DESIGN.md §5.4, §6).
// Máquina de estados pura (sem ROS); o nó alimenta `update()` a cada scan/tick
// e chama `notify_relocalization_result` quando o relocalizador termina.
class ModeManager
{
public:
  explicit ModeManager(ModeManagerParams params = {})
  : params_(params)
  {
  }

  ModeManagerEvents update(const ModeManagerInputs & in);

  // Chamado pelo nó quando o `TreeLocRelocalizer` (ou a decisão "opcional,
  // aceita prior GNSS") conclui o ciclo iniciado por `request_relocalization`.
  void notify_relocalization_result(bool accepted);

  SlamMode mode() const {return mode_;}
  // Autoridade do TF map->odom (regra de ouro: nunca dois publishers).
  bool owns_map_to_odom() const {return mode_ == SlamMode::GROUND;}
  // true quando o nó deve continuar a publicar o ÚLTIMO map->odom conhecido
  // sem atualizar com a otimização corrente (congelado, não ausente).
  bool pose_frozen() const {return frozen_;}

private:
  ModeManagerParams params_;
  SlamMode mode_{SlamMode::GROUND};
  bool frozen_{false};
  bool was_aerial_{false};
  bool was_hop_done_consumed_{true};
  bool current_relocalization_mandatory_{false};
  int mandatory_relocalization_attempts_{0};
};

}  // namespace forest_tree_slam
