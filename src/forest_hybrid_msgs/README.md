# `forest_hybrid_msgs`

Mensagens ROS 2 partilhadas pelo stack (`OperationMode`, `MissionCommand`, `MissionStatus`, `MissionAck`, …).

**`MissionCommand`:** `frame_type` só **`FRAME_MAP` (0)** ou **`FRAME_RELATIVE` (1)**. GNSS não está na mensagem — converter para `map` no *gateway* / GUI. **`CMD_CLEAR_EMERGENCY_LATCH` (6)** liberta o sistema após **`CMD_EMERGENCY_STOP`**. **Yaw:** `use_target_yaw` + `target_yaw_rad` para alvos simples; **`waypoint_yaw[]`** opcional em PATROL (igual número de pontos).

- Contratos completos por camada: [docs/LAYER_CONTRACTS.md](../../docs/LAYER_CONTRACTS.md)
- O pacote deve exportar `ament_cmake` em `package.xml` para o *overlay* colcon funcionar com a CLI `ros2`.
