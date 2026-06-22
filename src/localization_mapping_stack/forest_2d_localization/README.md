# forest_2d_localization — **LEGACY / CONGELADO**

> **Não usar.** Este pacote integrava `slam_toolbox` (SLAM 2D scan-matching).  
> Foi **substituído** pelo desenho Tree-SLAM (`docs/FOREST_TREE_SLAM_DESIGN.md`).  
> Ver registo completo: [`docs/LEGACY_PATHS.md`](../../../docs/LEGACY_PATHS.md).

O launch `slam_toolbox_online_async.launch.py` **falha por defeito** a menos que  
`FOREST_ALLOW_LEGACY=1` esteja definido.

## Substituto planeado

| Legacy | Novo |
|--------|------|
| `slam_toolbox` → TF `map→odom` | `forest_tree_slam` (SE2 pose-graph) |
| `forest_2d_localization` | `forest_tree_slam` + EKF SE3 |

## Histórico (referência apenas)

Integração Fase 2 (maio 2026): `async_slam_toolbox_node`, input `/perception/lidar/scan_ground`.
