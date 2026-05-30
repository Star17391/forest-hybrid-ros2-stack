# `forest_lidar_preprocess_cpp`

Classifica cada retorno do **LaserScan** em **solo** ou **outro**, sem remover pontos.

> **Estado:** protótipo **v0** (`min(z)` + banda). Roadmap e fundamentação científica em  
> [`docs/FOREST_SLAM_BIBLIOGRAPHY.md`](../../../docs/FOREST_SLAM_BIBLIOGRAPHY.md) (secções *Análise solo vs não-solo* e *Estratégia combinada*).

## Tópicos

| Tópico | Tipo | Conteúdo |
|--------|------|----------|
| `/scan` (entrada) | `LaserScan` | Scan bruto |
| `/perception/lidar/points_labeled` | `PointCloud2` | Todos os feixes; campo `label`: 0=inválido, 1=solo, 2=outro |
| `/perception/lidar/scan_ground` | `LaserScan` | Só ranges classificados como solo (resto NaN) |
| `/perception/lidar/scan_other` | `LaserScan` | Troncos/vegetação/obstáculos (resto NaN) |

O scan original em `/scan` não é alterado.

## Método actual (v0)

1. Transforma cada ponto para `classification_frame` (TF ou fallback pitch/altura).
2. `z_ref` = mínimo Z válido no scan.
3. **Solo** se `|z - z_ref| ≤ ground_height_band_m`.

**Limitação:** em floresta com relevo e robô inclinado, isto não equivale ao plano de solo da literatura (Palacín, PRC, LeGO-LOAM). Ver bibliografia para **v1** (linha/plano + buracos) e **v2** (troncos).

## Uso

```bash
ros2 launch forest_lidar_preprocess_cpp lidar_preprocess.launch.py
# hardware:
ros2 launch forest_lidar_preprocess_cpp lidar_preprocess.launch.py classification_frame:=base_link
```

RViz: `PointCloud2` em `/perception/lidar/points_labeled`, colorir por canal `label`.
