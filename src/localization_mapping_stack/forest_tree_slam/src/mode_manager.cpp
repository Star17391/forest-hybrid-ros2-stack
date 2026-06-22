#include "forest_tree_slam/mode_manager.hpp"

namespace forest_tree_slam
{

ModeManagerEvents ModeManager::update(const ModeManagerInputs & in)
{
  ModeManagerEvents events;

  // --- LOST: recuperação assim que voltam associações (e não se está no ar) ---
  if (mode_ == SlamMode::LOST && !in.locomotion_aerial &&
    in.scans_since_any_association == 0)
  {
    mode_ = SlamMode::GROUND;
    frozen_ = false;
  }

  // --- Entrada em AERIAL (edge): de qualquer estado de solo -------------
  if (in.locomotion_aerial && !was_aerial_) {
    mode_ = SlamMode::AERIAL;
    frozen_ = true;
    events.request_takeoff_snapshot = true;
    was_hop_done_consumed_ = false;
    mandatory_relocalization_attempts_ = 0;
  }
  was_aerial_ = in.locomotion_aerial;

  if (mode_ == SlamMode::AERIAL) {
    if (in.hop_failed) {
      mode_ = SlamMode::LOST;
      frozen_ = true;
      return events;
    }
    if (in.hop_done && !was_hop_done_consumed_) {
      was_hop_done_consumed_ = true;
      mode_ = SlamMode::RELOCALIZING;
      frozen_ = true;
      events.request_relocalization = true;
      events.relocalization_mandatory = !in.gnss_good;
      current_relocalization_mandatory_ = events.relocalization_mandatory;
      return events;
    }
    // ainda AERIAL/em voo: nada a fazer, map->odom é do EKF (owns_map_to_odom=false).
    return events;
  }

  if (mode_ == SlamMode::RELOCALIZING) {
    // Aguarda `notify_relocalization_result`; só sai daqui por essa via.
    return events;
  }

  // --- GROUND / degradação por falta de associação (§8) ------------------
  if (mode_ == SlamMode::GROUND) {
    if (in.scans_since_any_association >= params_.lost_after_scans) {
      mode_ = SlamMode::LOST;
      frozen_ = true;
    } else if (in.scans_since_any_association >= params_.degraded_after_scans) {
      frozen_ = true;
    } else {
      frozen_ = false;
    }
  }

  return events;
}

void ModeManager::notify_relocalization_result(bool accepted)
{
  if (mode_ != SlamMode::RELOCALIZING) {
    return;
  }
  if (accepted) {
    mode_ = SlamMode::GROUND;
    frozen_ = false;
    mandatory_relocalization_attempts_ = 0;
    return;
  }

  // Rejeitado: se a relocalização era opcional (GNSS bom), aceita-se o prior
  // GNSS como suficiente (design §2.3: "opcional — refina sobretudo o
  // heading") e volta-se a GROUND. Se era obrigatória, tenta-se outra vez até
  // ao limite; depois disso, LOST (alerta à missão).
  if (!current_relocalization_mandatory_) {
    mode_ = SlamMode::GROUND;
    frozen_ = false;
    mandatory_relocalization_attempts_ = 0;
    return;
  }
  ++mandatory_relocalization_attempts_;
  if (mandatory_relocalization_attempts_ >= params_.max_mandatory_relocalization_attempts) {
    mode_ = SlamMode::LOST;
    frozen_ = true;
  }
  // Se ainda há tentativas, fica em RELOCALIZING — o nó deve voltar a pedir
  // `request_relocalization` no próximo scan (não modelado aqui como evento
  // de update() porque depende do nó re-tentar a captura local de troncos).
}

}  // namespace forest_tree_slam
