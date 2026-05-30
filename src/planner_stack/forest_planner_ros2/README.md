# `forest_planner_ros2`

## `mission_manager_node`

FSM de missão: **`/mission/command`**, **`/mission/ack`**, **`/mission/status`**, objetivo **`/planning/mission_goal`** (publicado **uma vez** por perna / waypoint ativo), feedback **`/planning/progress`**, **`/planning/path_blocked`**, **`/planning/goal_reached`**.

- **`MissionCommand.frame_type`**: `FRAME_MAP` (0) ou `FRAME_RELATIVE` (1). **GNSS** deve ser convertido na GUI / *gateway* antes de enviar o comando.
- **Orientação final**: `use_target_yaw` + `target_yaw_rad` (rad, map) para GOTO / RETURN_HOME / TRACK; **PATROL** pode usar `waypoint_yaw[]` (mesmo comprimento que `waypoint_x`) ou deixar vazio para tangente.
- **Emergência**: após `CMD_EMERGENCY_STOP` o fluxo fica **latched** — só aceita `CMD_CLEAR_EMERGENCY_LATCH` (6) até libertar (política de campo: após inspeção / *reset* físico conforme procedimento).
- **Chegada ao objetivo**: subscrição a **`pose_topic`** (defeito `/state/pose_fused`), `header.frame_id` **`map`**, distância 3D ≤ `goal_tolerance_m` e erro de yaw ≤ `goal_tolerance_heading_deg`. `progress` só atualiza UI; não completa missão.
- **`allow_goal_reached_topic_shortcut`**: se `true`, `/planning/goal_reached` também avança perna (testes legados).

Comportamento por comando e camada de trajetória futura: [docs/PLANNING_TRAJECTORY_LAYER.md](../../../docs/PLANNING_TRAJECTORY_LAYER.md).

Contrato: [docs/LAYER_CONTRACTS.md](../../../docs/LAYER_CONTRACTS.md).
