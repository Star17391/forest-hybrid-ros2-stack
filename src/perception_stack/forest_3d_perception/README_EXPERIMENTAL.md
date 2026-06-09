# Experimental LiDAR 3D pipeline (parallel to legacy)

The **legacy** node `lidar3d_segmentation_node` is unchanged. This package adds a
**parallel** pipeline for A/B testing.

## Architecture

| Pipeline | Node | Topics prefix |
|----------|------|----------------|
| Legacy | `lidar3d_segmentation_node` | `/perception/lidar3d/` |
| Experimental | `lidar3d_experimental_node` | `/perception/lidar3d/experimental/` |

Switch at launch:

```bash
ros2 launch forest_3d_perception lidar3d_dual_pipeline.launch.py \
  use_experimental_pipeline:=true use_legacy_pipeline:=true
```

YAML flag file: `config/lidar3d_perception_mode.yaml`.

## Literatura — clustering florestal

Revisão da comunidade científica (CHM, point-cloud, robótica móvel, limitações em copas coladas):

→ [`docs/FORESTRY_CLUSTERING_LITERATURE.md`](docs/FORESTRY_CLUSTERING_LITERATURE.md)

## Sprints

| Sprint | Status | Content |
|--------|--------|---------|
| **1** | Done | CSF ground + Euclidean clustering |
| **2** | **Active** | Tree candidates from clusters (height / verticality / XY extent) |
| **3** | Stub | Tree verification (`tree_verification.hpp`) — PCA, cylinder RMSE |

Default: `pipeline_sprint: 2` in `lidar3d_experimental.yaml`. Set `1` to disable tree stage; `3` when verification is implemented.

## Outputs

**Sprint 1+2 (default):**

- `/perception/lidar3d/experimental/ground`
- `/perception/lidar3d/experimental/non_ground`
- `/perception/lidar3d/experimental/clusters` (intensity = cluster id)
- `/perception/lidar3d/experimental/cluster_markers`
- `/perception/lidar3d/experimental/tree_candidates` (Sprint 2+)
- `/perception/lidar3d/experimental/tree_candidate_markers` (Sprint 2+)
- `/perception/lidar3d/experimental/debug_stats` (JSON, includes `n_tree_candidates`)

## Build (CSF dependency)

CSF is downloaded at `colcon build` via CMake FetchContent, or vendored offline:

```bash
bash scripts/fetch_csf_vendor.sh
colcon build --packages-select forest_3d_perception
```

## Live tuning

```bash
python3 tools/diagnostics/lidar3d_experimental_live_tuning.py
# http://127.0.0.1:8766
```

## Forest CLI (recomendado)

**A/B legacy + experimental** (Gazebo + RViz + ambos os nós):

```bash
forest up sim-lidar3d-experimental -d --world forest_rugged_trees_rocks
```

Ou activar experimental no perfil habitual:

```bash
forest up sim-lidar3d-test -d --lidar3d --lidar3d-experimental --world forest_rugged_trees_rocks
```

**Só CSF experimental** (sem legacy segmentation):

```bash
forest up sim-lidar3d-csf-only -d --world forest_rugged_trees_rocks
# ou: forest up sim-lidar3d-test -d --lidar3d --lidar3d-experimental-only
```

**Live tuning** (terminal 2, com stack a correr):

```bash
forest diag lidar3d-exp-tune
# http://127.0.0.1:8766
```

Legacy tuning continua em `forest diag` → `python3 tools/diagnostics/lidar3d_live_tuning.py` (8765).

## Launch manual (sem forest)

```bash
ros2 launch forest_3d_perception lidar3d_dual_pipeline.launch.py \
  use_sim_time:=true use_experimental_pipeline:=true
```
