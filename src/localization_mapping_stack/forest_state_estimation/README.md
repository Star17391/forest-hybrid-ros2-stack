# `forest_state_estimation`

**Camada estimador de estado SE3** — UM EKF `robot_localization` (15-estados, `two_d_mode: false`)
+ autoridade `map→odom` comutada por modo, para o robô híbrido solo/ar.

Ref design: `docs/FOREST_TREE_SLAM_DESIGN.md §4–6`.

## Pipeline SE3

```text
/sensors/wheel_odometry ─────────────────────────────┐
/sensors/imu/data (gyro 3D; accel bloqueada) ────────┤
                                                       ▼
                                       ekf_local (ÚNICO EKF) ──► TF odom→base_link + /state/odometry

/ardupilot/local_position_odom (posição+atitude, publicado pelo hybrid_hop_executor da porta 5760)
        └────────────────────────────────► map_odom_authority_node (fonte aérea)

map_odom_authority_node (autoridade ÚNICA de map→odom):
  └─ GROUND: ground_mode=identity → MANTÉM o último map→odom (identidade até ao 1.º voo,
             correção de aterragem depois);  ground_mode=silent → cede ao Tree-SLAM (Fase 2+)
  └─ AERIAL: map→odom derivado da pose do ArduPilot (T_map_takeoff · T_apHome_base)

TF map→base_link ──► state_contract_node ──► /state/pose_fused
```

NOTA: o antigo `ekf_global` foi removido (re-filtrava o ArduPilot no ar = double-counting; o AP
já tem EKF3). A pose no ar vem direta do AP (design §6). GNSS de campo (`gnss_cov_adapter`+
`navsat_transform`, só com `use_gnss`) voltará a ligar-se no grafo do Tree-SLAM (Fase 2+).

## Nós

| Nó | Função |
|----|--------|
| `ekf_local` | odom→base, wheel+gyro SE3 (`ekf_local.yaml`) — ÚNICO EKF |
| `map_odom_authority_node` | autoridade ÚNICA de map→odom (solo: hold/identidade; ar: ArduPilot direto) |
| `state_contract_node` | publica /state/pose_fused (PoseStamped, frame=map) |
| `gnss_cov_adapter_node` / `navsat_transform` | GNSS (só `use_gnss`; dormentes até ao Tree-SLAM) |

## Comutação de autoridade TF map→odom

| Modo | Publisher | Condição |
|------|-----------|---------|
| GROUND | `map_odom_authority_node` (hold/identidade) ou Tree-SLAM (Fase 2+, `ground_mode=silent`) | `MODE_GROUND` |
| AERIAL | `map_odom_authority_node` (pose do ArduPilot) | `MODE_AERIAL` |

**Regra:** nunca dois publishers em simultâneo. O guard está no `map_odom_authority_node`.

## Covariância GNSS sob dossel

Ref: `docs/perception/references/2024_gnss_under_canopy_degradation.md`
- Recetores comuns sob dossel: **5–20 m** de erro horizontal → cov ≥ 25 m²
- Covariância otimista é a causa nº1 de falha de fusão do EKF global
- O `gnss_cov_adapter_node` aplica o mínimo antes do `navsat_transform`

Configurar `config/gnss_cov_adapter.yaml` para ajustar limiares.

## Dependência

```bash
sudo apt install ros-${ROS_DISTRO}-robot-localization
```

## Launch

```bash
# Sim básico (wheel only, sem GNSS):
ros2 launch forest_state_estimation state_estimation.launch.py \
  use_sim_time:=true use_wheel_odom:=true

# Sim com IMU SE3 (wheel + IMU):
ros2 launch forest_state_estimation state_estimation.launch.py \
  use_sim_time:=true \
  ekf_config:="$(ros2 pkg prefix forest_state_estimation)/share/forest_state_estimation/config/ekf_local.yaml"

# Tree-SLAM como autoridade no solo (autoridade cede o map→odom no solo):
ros2 launch forest_state_estimation state_estimation.launch.py \
  use_sim_time:=true publish_map_odom_identity:=false

# Validação estática (sem Gazebo):
forest test ekf-se3-config

# Teste integrado Gazebo + SITL + salto híbrido:
forest test ekf-se3 [--assert]
```

Ajustar `config/navsat_transform.yaml` (`datum`) ao local de teste.
Ajustar `config/gnss_cov_adapter.yaml` para covariância GNSS realista.
