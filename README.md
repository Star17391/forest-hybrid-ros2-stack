# Forest Hybrid Robot — ROS 2 stack

Workspace **colcon** para o robô híbrido florestal (lagartas + coaxial quadcopter): drivers, perceção semântica, fusão LiDAR–câmara (futuro), localização/mapeamento, planeamento e *bringup* centralizado.

Treino de redes num repositório à parte (ex.: `forest-semantic-training/`).

## Filosofia de integração

Os **nomes de tópicos e tipos definidos neste repositório são a referência**. Projetos externos (Gazebo, drivers, simuladores tipo ForestGen) fazem **remap** ou *bridges* para cumprir esse contrato — **não** o contrário. Ver [docs/GAZEBO_INTEGRATION.md](docs/GAZEBO_INTEGRATION.md).

## Layout (alinhado ao `fdpo-ros-stack`)

| Pasta | Função |
|--------|--------|
| `src/conf/` | Lançamentos e YAMLs centralizados (`forest_hybrid_conf`) |
| `src/sim_bridge/` | Gazebo-ROS2 bridge (`forest_sim_bridge`): pose, TF, cleanup, panel |
| `src/drivers_stack/` | Câmara, LiDAR (deps), atuadores |
| `src/perception_stack/` | Segmentação semântica ONNX, fusão (futuro) |
| `src/localization_mapping_stack/` | LIO / SLAM (*bringup* futuro) |
| `src/navigation_stack/` | Navegação (Pure Pursuit, trajectory sampler) |
| `src/planner_stack/` | Mission manager + planeamento (extensível) |
| `src/utilities_stack/` | Modo de operação (`/system/locomotion_mode`), utilitários |
| `src/forest_hybrid_msgs/` | Mensagens partilhadas |
| `tools/forest/` | CLI **`forest`** — sim, perfis, testes, diagnósticos (ver secção abaixo) |
| `scripts/` | Legado por tópico (Nicla, LiDAR, missão) — [scripts/README.md](scripts/README.md) |
| `deploy/` | systemd / notas de instalação |
| `docs/` | Contratos por camada, simulação, roadmap |

### README por pacote

| Pacote | README |
|--------|--------|
| `forest_hybrid_msgs` | [src/forest_hybrid_msgs/README.md](src/forest_hybrid_msgs/README.md) |
| `forest_hybrid_conf` | [src/conf/forest_hybrid_conf/README.md](src/conf/forest_hybrid_conf/README.md) |
| `forest_camera_ros2` | [src/drivers_stack/forest_camera_ros2/README.md](src/drivers_stack/forest_camera_ros2/README.md) |
| `forest_lidar_ros2` | [src/drivers_stack/forest_lidar_ros2/README.md](src/drivers_stack/forest_lidar_ros2/README.md) |
| `forest_sensors_cpp` | TF LiDAR + LaserScan→PointCloud2 (C++) |
| Bibliografia SLAM | [docs/FOREST_SLAM_BIBLIOGRAPHY.md](docs/FOREST_SLAM_BIBLIOGRAPHY.md) |
| `forest_actuators_ros2` | [src/drivers_stack/forest_actuators_ros2/README.md](src/drivers_stack/forest_actuators_ros2/README.md) |
| `forest_semantic_segmentation` | [src/perception_stack/forest_semantic_segmentation/README.md](src/perception_stack/forest_semantic_segmentation/README.md) |
| `forest_pointcloud_semantics` | [src/perception_stack/forest_pointcloud_semantics/README.md](src/perception_stack/forest_pointcloud_semantics/README.md) |
| `forest_planner_ros2` | [src/planner_stack/forest_planner_ros2/README.md](src/planner_stack/forest_planner_ros2/README.md) |
| `forest_robot_supervisor` | [src/utilities_stack/forest_robot_supervisor/README.md](src/utilities_stack/forest_robot_supervisor/README.md) |
| `forest_gazebo_bridge` | [src/utilities_stack/forest_gazebo_bridge/README.md](src/utilities_stack/forest_gazebo_bridge/README.md) |
| `forest_navigation_ros2` | [src/navigation_stack/forest_navigation_ros2/README.md](src/navigation_stack/forest_navigation_ros2/README.md) |
| `forest_sim_bridge` | [src/sim_bridge/forest_sim_bridge/](src/sim_bridge/forest_sim_bridge/) |
| `forest_lio_bringup` | [src/localization_mapping_stack/forest_lio_bringup/README.md](src/localization_mapping_stack/forest_lio_bringup/README.md) |

Robô simulado no ForestGen (MARBLE HD2): [docs/FORESTGEN_SIM_ROBOT.md](docs/FORESTGEN_SIM_ROBOT.md).

## Dependências de sistema (Ubuntu + ROS 2 Jazzy exemplo)

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-usb-cam ros-$ROS_DISTRO-camera-ros
```

Na **Raspberry Pi**, se `camera_ros`/`libcamera` falharem, consultar a documentação upstream do `camera_ros` e eventual *fork* Raspberry Pi do `libcamera`.

## Build

```bash
cd /caminho/para/forest-hybrid-ros2-stack
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

**Cada terminal** que use `ros2` sobre este workspace deve fazer `source install/setup.bash` (ou ter isso no `.bashrc`), senão `ros2 interface show forest_hybrid_msgs/...` e `ros2 topic pub` podem falhar.

### `forest_hybrid_msgs` invisível ao `ros2`

O pacote `forest_hybrid_msgs` é **ament_cmake** e tem de exportar `<build_type>ament_cmake</build_type>` em `package.xml`. Sem isso, o *overlay* do colcon não entra no `AMENT_PREFIX_PATH` e a CLI não vê as mensagens.

## Operação com `forest` (simulação e testes)

O dia-a-dia do stack em Gazebo passa pelo CLI **`forest`** em `tools/forest/bin/forest`. Documentação completa: [docs/FOREST_CLI.md](docs/FOREST_CLI.md) · estado e validação: [docs/FOREST_OPERATIONS_STATUS.md](docs/FOREST_OPERATIONS_STATUS.md).

### Instalação (PATH + Tab completion)

```bash
export HYBRID_WS=/caminho/para/forest-hybrid-ros2-stack
cd "$HYBRID_WS"
source install/setup.bash
export FORESTGEN_PATH=~/Projetos/Gazebo/ForestGen   # mundos e modelo MARBLE HD2

bash "$HYBRID_WS/tools/forest/completions/install.sh"
source ~/.bashrc
type forest   # deve resolver para .../tools/forest/bin/forest
```

Em cada shell nova: `source install/setup.bash` e, se o PATH não tiver o CLI, `export PATH="$HYBRID_WS/tools/forest/bin:$PATH"`.

### Fluxo típico

```bash
forest profile list              # perfis em tools/forest/profiles/*.yaml
forest up sim-mvp-nav -d         # Gazebo + nav EKF; -d = detach (PLAY automático)
forest status                    # sessão, perfil, nós
forest test patrol-rect          # PATROL rectângulo (requer PLAY)
forest test goto --assert        # GOTO smoke com goal_reached
forest down                      # shutdown + cleanup
```

**Pose verdade (controlo perfeito, sem drift):** `forest up sim-pose-bridge -d` + `forest test patrol-rect`.

**Teleop:** `forest up sim-teleop -d` → `forest teleop` (cmd_vel para o Gazebo).

**Só sensores / IMU:** `forest up sim-sensors-only -d` → `forest diag imu-analyze`, `forest diag tf-audit`, etc.

### Comandos principais

| Comando | Função |
|---------|--------|
| `forest up <perfil> [-d] [--headless] [--no-rviz] [--timeout SEC]` | Sobe o stack definido no YAML |
| `forest down [--force]` | Termina sessão e limpa processos |
| `forest cleanup [--hybrid]` | Kill Gazebo/RViz/ROS órfãos (sem subir stack) |
| `forest status` | Sessão activa e amostra de nós |
| `forest profile list` / `forest profile validate <nome>` | Perfis |
| `forest test patrol-rect` | Teste PATROL |
| `forest test goto [--assert] [--x X] [--y Y]` | Teste GOTO |
| `forest diag imu-analyze`, `tf-audit`, `lidar`, … | Diagnósticos em `tools/diagnostics/` |
| `forest logs [layer] [-f]` | Logs da sessão (ex. `sim.log`) |
| `forest panel` / `forest teleop` | Painéis (stack já a correr) |
| `forest completion refresh` | Actualizar Tab após mudar perfis/CLI |

Perfis usuais: `sim-minimal`, `sim-pose-bridge`, `sim-mvp-nav`, `sim-sensors-only`, `sim-teleop`.

### Validação rápida (estrutura + sim)

```bash
for p in 1 2 3 4 5 6; do bash tools/forest/tests/phase${p}_validate.sh; done
```

Sessão IMU/TF/nav (Plano B): ver checklist em [docs/FOREST_OPERATIONS_STATUS.md](docs/FOREST_OPERATIONS_STATUS.md).

### CI no GitHub

Por defeito o workflow [`.github/workflows/forest-ci.yml`](.github/workflows/forest-ci.yml) faz **build** + validação das fases 1–6 **sem** Gazebo.

**CI sim live** (opcional): job extra com Gazebo + `forest test goto` — só corre se definires a variável de repositório `FOREST_CI_LIVE=1` e tiveres runner adequado (idealmente self-hosted). Não é necessário para desenvolvimento local.

## Bringup completo (câmara + modo de operação + mission + segmentação)

```bash
source install/setup.bash
ros2 launch forest_hybrid_conf forest_bringup.launch.py
```

Argumentos úteis:

```bash
# Relógio de simulação (Gazebo / relógio publicada)
ros2 launch forest_hybrid_conf forest_bringup.launch.py use_sim_time:=true

# Modo aéreo: segmentação deixa de processar frames (via /system/locomotion_mode)
ros2 launch forest_hybrid_conf forest_bringup.launch.py operation_mode:=aerial

# Desligar publicação de modo (útil se outro nó assumir o tópico)
ros2 launch forest_hybrid_conf forest_bringup.launch.py enable_operation_mode:=false

# Backend de câmara
ros2 launch forest_hybrid_conf forest_bringup.launch.py camera_backend:=csi

# Testes sem hardware de câmara
ros2 launch forest_hybrid_conf forest_bringup.launch.py enable_camera:=false

# Segmentação ONNX (vazio => máscara zero)
ros2 launch forest_hybrid_conf forest_bringup.launch.py \
  onnx_model_path:=/caminho/para/modelo.onnx \
  model_input_width:=512 model_input_height:=384
```

## Apenas camada de missão (testes isolados)

```bash
source install/setup.bash
ros2 launch forest_hybrid_conf mission_layer_only.launch.py
```

## Teste automático (smoke) da mission layer

```bash
source /opt/ros/jazzy/setup.bash   # ou a tua distro
source install/setup.bash
./scripts/run_mission_layer_checks.sh
```

## Teste manual da mission layer (CLI)

Usar **`--once`** (ou `-1`) em `ros2 topic pub` para não inundar a FSM com repetições.

```bash
source install/setup.bash
ros2 launch forest_hybrid_conf forest_bringup.launch.py enable_camera:=false

# Estado
ros2 topic echo /mission/status

# GOTO (command_type 1 = CMD_GOTO_XYZ, frame_type 0 = MAP)
ros2 topic pub --once /mission/command forest_hybrid_msgs/msg/MissionCommand \
  "{command_type: 1, frame_type: 0, command_id: 'goto_1', source: 'cli', target_x: 10.0, target_y: 2.0, target_z: 0.0}"

# Simular pose fundida no goal (frame map) — necessário para COMPLETED
ros2 topic pub --once /state/pose_fused geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

# Opcional: progresso 0–1 (apenas UI / MissionStatus.progress)
ros2 topic pub --once /planning/progress std_msgs/msg/Float32 "{data: 0.5}"

# RETURN_HOME + ACK
ros2 topic pub --once /mission/command forest_hybrid_msgs/msg/MissionCommand \
  "{command_type: 4, frame_type: 0, command_id: 'return_1', source: 'cli'}"
ros2 topic pub --once /mission/ack forest_hybrid_msgs/msg/MissionAck \
  "{command_id: 'return_1', approved: true, reason: 'operator_ok'}"
```

## Contratos por camada (fonte de verdade)

- Documento legível: [**docs/LAYER_CONTRACTS.md**](docs/LAYER_CONTRACTS.md)
- YAML para *parsing* / integração CI: **`src/conf/forest_hybrid_conf/config/layer_contracts.yaml`**
- Índice: [docs/README.md](docs/README.md)

Resumo dos tópicos **já utilizados** no código atual:

| Camada | Tópico | Direção | Tipo | Notas |
|--------|--------|---------|------|--------|
| Utilities | `/system/locomotion_mode` | pub | `forest_hybrid_msgs/OperationMode` | `operation_mode_node`; QoS transient local |
| Mission | `/mission/command` | sub | `forest_hybrid_msgs/MissionCommand` | UI / operador |
| Mission | `/mission/ack` | sub | `forest_hybrid_msgs/MissionAck` | Aprovações (ex.: return home) |
| Mission | `/mission/status` | pub | `forest_hybrid_msgs/MissionStatus` | Estado FSM |
| Mission | `/planning/mission_goal` | pub | `geometry_msgs/PoseStamped` | Objetivo em **`map`**, uma vez por perna |
| Mission | `/state/pose_fused` | sub | `geometry_msgs/PoseStamped` | Pose em **`map`** para fecho de perna (distância + yaw) |
| Mission | `/planning/progress` | sub | `std_msgs/Float32` | 0–1 ao longo do path (UI); **não** completa waypoint |
| Mission | `/planning/path_blocked` | sub | `std_msgs/Bool` | Bloqueio |
| Mission | `/planning/goal_reached` | sub | `std_msgs/Bool` | Só se `allow_goal_reached_topic_shortcut:=true` |
| Câmara | `/camera/image_raw` | pub | `sensor_msgs/Image` | `usb_cam` ou `camera_ros` |
| Câmara | `/camera/camera_info` | pub | `sensor_msgs/CameraInfo` | |
| Segmentação | `/perception/semantic_mask` | pub | `sensor_msgs/Image` | `mono8`, ID de classe |

Tópicos **planeados** (fusão, estado, controlo) estão detalhados em `docs/LAYER_CONTRACTS.md` e no YAML.

## Planeador de baixo nível e floresta sem mapa

O *mission manager* publica **`/planning/mission_goal`** uma vez por perna e subscreve **`/state/pose_fused`** (localizer) para marcar chegada por **tolerância métrica + yaw**. O **navigation stack** (futuro) deve publicar **`/planning/progress`** (percentagem ao longo do path), **`/planning/path_blocked`** e, se quiseres atalho de teste, **`/planning/goal_reached`**. O ROS 2 **não garante** ordenação global de mensagens entre tópicos; cada nó trata o fluxo com timestamps e estado local.

Ideia de **mapas em mosaico / tiles** e o que falta para **missão + navegação** completas: [docs/FUTURE_TILED_MAPS.md](docs/FUTURE_TILED_MAPS.md), [docs/MISSION_AND_NAV_REMAINING.md](docs/MISSION_AND_NAV_REMAINING.md).

## LiDAR

Driver ainda em preparação: `src/drivers_stack/forest_lidar_ros2/DEPENDENCIES.md` e `package.xml`.

## Estado do repositório

- **Operação sim:** CLI `forest` (fases 1–6 concluídas); validação IMU/TF/nav em sim documentada em [docs/FOREST_OPERATIONS_STATUS.md](docs/FOREST_OPERATIONS_STATUS.md).
- **Bringup produto:** C++ (`mission_manager_node`, segmentação ONNX opcional), câmara USB, `operation_mode_node`.
- **Foco actual:** localização/SLAM — Fase 0 fechada · **Fase 1 Palacín v1** em [docs/reports/PHASE1_VERIFICATION.md](docs/reports/PHASE1_VERIFICATION.md) · Fase 2 a seguir.
- **Backlog produto:** PATROL por tile — [docs/MISSION_AND_NAV_REMAINING.md](docs/MISSION_AND_NAV_REMAINING.md).
- **Sensores reais / `tools/hardware/`:** fora de âmbito actual; scripts Nicla/LiDAR em `scripts/` mantidos.

## Documentação extra

- [docs/README.md](docs/README.md) — índice de contratos, sim, CLI, LiDAR, roadmap.
- [deploy/README.md](deploy/README.md) — notas de *deploy* (Raspberry Pi, systemd).
