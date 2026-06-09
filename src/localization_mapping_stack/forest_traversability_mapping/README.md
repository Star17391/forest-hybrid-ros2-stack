# `forest_traversability_mapping`

Camada de **mapa de custo / traversabilidade 2.5D** (`grid_map`) entre a perceção
(labels / landmarks) e o planeamento (global / local / NMPC).

Plano completo por sprints: [`docs/TRAVERSABILITY_COSTMAP_PLAN.md`](../../../docs/TRAVERSABILITY_COSTMAP_PLAN.md).

## Estado: Sprint 0 + Sprint 2 (malha de terreno)

`traversability_mapping_node` mantém um `grid_map` local **rolante** centrado na pose do
robô (TF `map_frame` → `base_frame`) e publica:

| Tópico | Tipo |
|--------|------|
| `/mapping/traversability_map` | `grid_map_msgs/GridMap` (camadas `elevation`, `variance`, `cost`, +internas `n_obs`/`m2`) |
| `/mapping/traversability_costmap` | `nav_msgs/OccupancyGrid` (camada `cost`, 0–100) |

- **Sprint 2 (feito):** a camada `elevation` é a **malha de solo 2.5D** que o robô usa —
  funde o ground CSF (`/perception/lidar3d/experimental/ground`, param `ground_topic`)
  por **Welford** (média+variância por célula) em `odom`. Não é uma nuvem; é uma altura
  por célula. Visualiza com o display `grid_map_rviz_plugin/GridMap` (camada `elevation`).
- **Sprint 1 (próximo):** a camada `cost` (hoje 0) recebe os clusters de obstáculos.

## Dependência

Requer `grid_map` (ANYbotics), ainda **não instalado** no ambiente:

```bash
sudo apt install ros-jazzy-grid-map-ros ros-jazzy-grid-map-msgs ros-jazzy-grid-map-core
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Projetos/Tese/forest-hybrid-ros2-stack
colcon build --packages-select forest_traversability_mapping --symlink-install
```

## Run + validação (Sprint 0)

Via CLI `forest` (sim experimental + mapping + RViz, tudo num comando):

```bash
forest up sim-traversability -d --world mvp_empty_flat.sdf
# Gazebo arranca em PLAY; o RViz já traz o display "Traversability costmap".
```

O display lê `/mapping/traversability_costmap` (`OccupancyGrid`) — não precisa do
`grid_map_rviz_plugin`. Para mover o robô e ver a grelha a seguir:

```bash
forest teleop        # ou: forest random-move
```

**Gate Sprint 0** (plumbing, ainda sem custo): a grelha 20×20 m aparece no RViz,
**segue o robô** ao mover, sem saltos de TF. O custo é tudo 0 (cinzento uniforme)
até ao Sprint 1.

Smoke test isolado (sem sim, só confirma que publica): `ros2 run
forest_traversability_mapping traversability_mapping_node` → `ros2 topic hz
/mapping/traversability_costmap` deve dar ~5 Hz.
