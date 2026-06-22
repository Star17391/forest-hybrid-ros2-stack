# `forest_tree_slam`

**Tree-SLAM florestal** — pose-graph SE(2) por landmarks de tronco (GTSAM/iSAM2),
autoridade de `map→odom` **no solo**, relocalização através de saltos aéreos
(TreeLoc: TDH coarse + triângulo fine + verificação geométrica/RANSAC + supressão
de outliers). Contributo central da tese — ver `docs/FOREST_TREE_SLAM_DESIGN.md`
(desenho completo) e `docs/LAYER_CONTRACTS.md` (contrato de tópicos/TF).

## Arquitetura interna

```text
tree_slam_node
  ├─ LandmarkTracker      (tracker.hpp/cpp)     — Δodom -> gate Mahalanobis -> Hungarian -> birth/death/merge
  ├─ TreeSlamBackend      (backend.hpp/cpp)     — GTSAM/iSAM2: poses SE2 + landmarks Point2
  ├─ TreeLocRelocalizer   (relocalizer.hpp/cpp) — TDH coarse + triângulo fine + RANSAC + outlier suppression
  └─ ModeManager          (mode_manager.hpp/cpp)— FSM GROUND/AERIAL/RELOCALIZING/LOST
```

Os quatro módulos são **agnósticos de ROS** (compilados em `tree_slam_core`,
testados offline com dados sintéticos em `test/`) — o nó só faz a tradução
mensagens ROS <-> tipos internos (`types.hpp`).

## Subscreve

| Tópico | Tipo | Notas |
|--------|------|-------|
| `/perception/lidar/tree_landmarks` | `TreeLandmarkArray` | base_link, por-frame, com covariância |
| `/state/odometry` | `nav_msgs/Odometry` | EKF SE3 local (`forest_state_estimation`) |
| `/system/locomotion_mode` | `OperationMode` | gatilho solo/ar |
| `/forest_gen/hybrid/hop_status` | `HybridHopStatus` | fases do salto (TAKEOFF/CRUISE/LANDING/TO_GROUND) |
| `/ardupilot/local_position_odom` | `nav_msgs/Odometry` | pose relativa ao takeoff -> aresta SE2 do salto |
| `/sensors/gnss/fix_adapted` | `NavSatFix` | decide relocalização obrigatória vs opcional (design §2.3) |

## Publica

| Tópico | Tipo |
|--------|------|
| `/slam/tree_map` | `TrackedTreeLandmarkArray` (uid, pose@map, DBH, cov, confidence) |
| `/slam/status` | `SlamStatus` (GROUND/AERIAL/RELOCALIZING/LOST + `owns_map_to_odom`) |
| `/slam/pose_graph` | `MarkerArray` (debug: esferas=landmarks, linha=trajetória keyframes) |
| TF `map→odom` | só quando `owns_map_to_odom==true` (GROUND) |

## Testes offline (gates do design §9)

```bash
colcon build --packages-select forest_tree_slam --symlink-install
colcon test --packages-select forest_tree_slam
```

| Teste | Gate |
|-------|------|
| `test_tracker` | birth/death/merge/Hungarian (sub-bloco 1) |
| `test_backend` | (a) ATE com troncos < ATE só-odom; reconvergência pós-salto (gate c) |
| `test_relocalizer` | (b) loop closure correto em revisita; rejeita queries sem correspondência real |
| `test_mode_manager` | transições GROUND/AERIAL/RELOCALIZING/LOST, autoridade TF |

## Correr no stack (CLI `forest`)

```bash
forest up sim-tree-slam -d --world forest_gentle_trees_rocks
forest logs tree_slam -f
ros2 topic hz /slam/tree_map
ros2 topic echo /slam/status
```

O profile `sim-tree-slam` arranca a sim (LiDAR 3D experimental, DBH+cov),
o EKF SE3 com `ground_mode=silent` (cede a autoridade `map→odom` no solo a
este nó) e o `tree_slam_node`. RViz: display "Tree-SLAM pose graph"
(`/slam/pose_graph`) no `.rviz` de LiDAR 3D experimental.

**Sem o GROUND truth do voo/relocalização real ainda integrados** (depende do
Agente 3 publicar a aresta do salto de forma estável em hardware/SITL real;
testado aqui só em SITL básico) — o caminho GROUND-only (sem saltos) é o que
está validado E2E nesta fase.

## Simplificações deliberadas (âmbito de tese, ver design §11)

- Observações de tronco entre keyframes são ligadas à ÚLTIMA keyframe usando
  bearing/range calculados a partir de uma pose dead-reckoned por odom (não
  criam keyframe própria) — padrão "low-rate keyframe, high-rate observation".
- O relocalizador não usa o prior do ArduPilot para restringir a busca
  coarse (faz busca global no mapa) — é um TODO de eficiência, não de
  correção; o gate de correspondências geométricas já garante segurança.
- Fator GPS fraco só documentado/wired para a keyframe de aterragem (via
  `add_weak_gps_prior` após relocalização aceite); a integração contínua de
  GNSS bom acima do dossel durante a marcha no solo fica para quando existir
  uma fonte GNSS em frame `map` ligada a este nó.
