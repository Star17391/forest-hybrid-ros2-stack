# Harness offline: isolação tronco/copa (zero copa na medição do DBH)

Valida, **sem ROS/sim**, que `cylinder_fit.hpp::fit_vertical_cylinder` (o ajuste do DBH
do node experimental) não deixa **nenhum** ponto de copa entrar na banda usada para medir
o diâmetro — usando o modelo do robô e as 6 árvores do ForestGen.

O `gpu_lidar` do Gazebo intersecta a mesh **visual** das árvores (a colisão é um casco
simplificado só para a física), por isso o raycasting é feito à visual, que tem tronco real.

## Componentes
- `lidar_model.py` — espelha o `front_laser` do `forest_tracked_robot_lidar3d/model.sdf`
  (32×360 feixes, montado z=0.36 m + pitch 12.5°). **Atualizar se o SDF mudar.**
- `gen_cloud.py` — raycasting (trimesh) à mesh visual; GT por ponto (tronco = geometria
  opaca alpha=255 que começa no solo; copa = folhagem alpha=191).
- `probe.cpp` + `run_probe.sh` — compila e corre o `fit_vertical_cylinder` REAL sobre uma
  nuvem rotulada; conta pontos de copa na banda do DBH (JSON numa linha).
- `batch.py` — bateria 6 árvores × distâncias × azimutes; escreve `out/results.json`.
- `viz.py` — figuras: pontos considerados tronco (medidos) vs descartados + resumo.

## Correr
```bash
PY=/home/star17391/Projetos/Gazebo/ForestGen/venv_forest/bin/python   # tem trimesh/matplotlib
cd tools/canopy_isolation
$PY batch.py          # bateria completa -> out/results.json (objetivo: copa-na-banda = 0)
$PY viz.py            # figuras -> out/isolation_*.png
```

## Resultado de referência (2026-06-21)
96 cenas → **96/96 com 0 pontos de copa na banda**. Regressão equivalente em C++:
`test/trunk_band_canopy_rejection_test.cpp` (corre no `colcon test`).

## Teste no CLI forest (sim real)
```bash
export FORESTGEN_PATH=/home/star17391/Projetos/Gazebo/ForestGen
forest up sim-lidar3d-experimental --lidar3d --headless -w forest_flat_trees
# inspecionar /perception/lidar/tree_clusters (SensorDataQoS): os pontos do fit devem
# ser uma coluna fina à altura do peito (z-span<~1.3 m, raio<~0.5 m), nunca um blob de copa.
forest down --force
```
