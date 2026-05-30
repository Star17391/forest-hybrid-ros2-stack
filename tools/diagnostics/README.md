# Forest diagnostics (`tools/diagnostics`)

Scripts de diagnóstico IMU, TF, LiDAR e pose. Invocação preferida via CLI:

```bash
forest diag imu          # launch isolado IMU
forest diag tf           # launch isolado TF + EKF wheel
forest diag imu-analyze  # estatísticas /sensors/imu/data_raw
forest diag tf-audit     # PDF + tf2_echo
forest diag lidar        # pipeline audit (sim em PLAY)
forest diag pose         # compare GT vs fused (quick 15s)
forest diag pose-benchmark  # Fase 0: metrics.json + CSV + PNG
forest diag ekf-latency  # Fase 0: rates/latency JSON
forest test phase0-benchmark  # workflow: latency + pose benchmark
forest diag world        # audit colisores ForestGen
```

Fase 0 — verificação: [docs/reports/PHASE0_VERIFICATION.md](../docs/reports/PHASE0_VERIFICATION.md).

### RViz stability

| Script | Uso |
|--------|-----|
| `rviz_stability_probe.sh` | GPU, X11/Wayland, validar `.rviz` |
| `rviz_config_validate.py` | Regras Depth/Style/Intensity |
| `parse_rviz_sim_log.py` | Timeline drawable vs `/clock` vs crash |
| `ros_topic_bandwidth.py` | Hz e KB/s dos tópicos |
| `ekf_tf_health.py` | Detecta TF NaN em `odom→base_link` |
| `../forest/workflows/test-rviz-incremental.sh` | Bissecção minimal → sensors → full |

Relatório: [docs/reports/RVIZ_STABILITY_ROOT_CAUSE.md](../docs/reports/RVIZ_STABILITY_ROOT_CAUSE.md).

```bash
python3 tools/diagnostics/parse_rviz_sim_log.py /run/user/1000/forest/sessions/*/sim.log
FOREST_RVIZ_PROFILE=full forest up sim-mvp-nav-imu -d   # RViz completo (nav)
```

Requer `source install/setup.bash` e, para launches, sim a correr ou `forest up` + Gazebo PLAY.

Ver [docs/ROOT_CAUSE_TF_IMU_NAV.md](../docs/ROOT_CAUSE_TF_IMU_NAV.md) e [docs/LIDAR_SIM_PIPELINE.md](../docs/LIDAR_SIM_PIPELINE.md).

### Semantic segmentation/fusion probes

| Script | Uso |
|--------|-----|
| `semantic_runtime_probe.py` | Hz de `/perception/semantic_mask` e `/perception/semantic_points` |
| `semantic_flicker_probe.py` | métrica de flicker temporal da máscara (`avg_switch_ratio`) |
| `../forest/workflows/test-semantic-stack.sh` | corre probes em paralelo para validação rápida |
