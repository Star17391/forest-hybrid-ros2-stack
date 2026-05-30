# Scripts — forest-hybrid-ros2-stack

## CLI `forest` (novo entrypoint operacional)

```bash
export PATH="$HOME/Projetos/Tese/forest-hybrid-ros2-stack/tools/forest/bin:$PATH"
bash tools/forest/completions/install.sh   # Tab completion (uma vez)
source ~/.bashrc
forest up sim-pose-bridge --panel-only   # ou sim-minimal
forest status
forest down
```

Documentação: [docs/FOREST_CLI.md](../docs/FOREST_CLI.md) · defaults Gazebo: [docs/FOREST_LAUNCH_DEFAULTS.md](../docs/FOREST_LAUNCH_DEFAULTS.md)

```bash
forest up sim-mvp-nav -d          # Gazebo em PLAY (política forest)
forest test patrol-rect           # com stack a correr
bash tools/forest/tests/phase{1..6}_validate.sh
```

Wrappers antigos (`run_trajectory_following.sh`, `kill_forest_stack.sh`, etc.) foram **removidos** — usar só `forest`.

`scripts/lib/_forest_common.sh` continua disponível para scripts internos (reexporta `tools/forest/lib`).

---

Scripts organizados por **assunto**. Os ficheiros na raiz de `scripts/` (ex. `test_lidar_fdpo_ros2.sh`) são **wrappers** que redirecionam para o caminho novo — podes continuar a usar os nomes antigos.

## Estrutura

```
scripts/
  lib/                    # Biblioteca partilhada (não executar diretamente)
    _forest_common.sh     # Sim / cleanup / shutdown (stack + navigation)
    repo_root.sh          # Resolve a raiz do repositório
  nicla/
    advr/                 # Nicla Vision — stack ADVR (Wi‑Fi / TCP)
    legacy/               # Firmware serial + Wi‑Fi (NICLAv1)
    validate/             # Fases 1–4 de validação
  lidar/                  # YDLidar X4 (fdpo + SDK)
  stack/                  # Gazebo / sim / cleanup geral
  navigation/             # Nav2 / trajetórias / bags de debug
  mission/                # mission_manager / smoke tests
```

## Nicla Vision (`nicla/`)

### ADVR (recomendado)

| Novo caminho | Uso |
|--------------|-----|
| `nicla/advr/init_submodules.sh` | Clonar submodules ADVR |
| `nicla/advr/apply_config.sh` | `config/forest_nicla_advr_config.h` → firmware + yaml |
| `nicla/advr/upload_firmware.sh` | Upload Arduino |
| `nicla/advr/build.sh` | colcon nicla packages |
| `nicla/advr/stop.sh` | Libertar porta 8002 / matar receiver |

Wrappers: `nicla_advr_*.sh`

### Legacy (serial / Wi‑Fi)

| Caminho | Uso |
|---------|-----|
| `nicla/legacy/upload_sensor_firmware.sh` | Firmware NICLAv1 |
| `nicla/legacy/install_wifi_firmware.sh` | Firmware Wi‑Fi QSPI |
| `nicla/legacy/wifi_connect.sh` | Ligar Wi‑Fi via serial |
| `nicla/legacy/serial_ping.sh` | Teste serial |
| `nicla/legacy/jpeg_color_check.sh` | Diagnóstico JPEG |

### Validação

| Caminho | Fase |
|---------|------|
| `nicla/validate/phase1_validate.sh` | USB + probe (`--snap`) |
| `nicla/validate/phase2_validate.sh` | Bridge ROS |
| `nicla/validate/phase3_validate.sh` | Wi‑Fi |
| `nicla/validate/phase4_validate.sh` | Integração |

## LiDAR (`lidar/`)

| Caminho | Uso |
|---------|-----|
| `lidar/install_driver.sh` | YDLidar-SDK + ydlidar_ros2_driver (source) |
| `lidar/test_fdpo_ros2.sh` | **Recomendado** — protocolo fdpo em ROS 2 |
| `lidar/test_ydlidar_sdk.sh` | Driver oficial YDLidar SDK |
| `lidar/test_ydlidar_upstream.sh` | Só nó upstream + X4.yaml |
| `lidar/test_fdpo_ros1.sh` | fdpo catkin (ROS 1) |
| `lidar/view_rviz.sh` | RViz com QoS Best Effort |

Wrappers: `test_lidar_*.sh`, `view_lidar_rviz.sh`, `install_ydlidar_ros2_driver.sh`

## Stack / sim (`stack/`)

| Caminho | Uso |
|---------|-----|
| `stack/kill_stack.sh` | Matar processos Forest/Gazebo (interno; preferir `forest down`) |
| `stack/verify_clean.sh` | Verificar se stack ficou limpa |

Sim sensores / MVP / nav: **`forest up <profile>`** (ver `docs/FOREST_CLI.md`).

## Navigation (`navigation/`)

| Caminho | Uso |
|---------|-----|
| `navigation/run_trajectory_following_terrain.sh` | PATROL em terreno (legado; migrar para perfil YAML) |
| `navigation/record_pose_debug_bag.sh` | Gravar bag pose |
| `navigation/analyze_pose_jitter_bag.py` | Analisar jitter |

Testes automáticos: `forest test patrol-rect`, `forest test goto`.

## Mission (`mission/`)

| Caminho | Uso |
|---------|-----|
| `mission/run_layer_checks.sh` | Checks da camada mission |
| `mission/test_manager_smoke.py` | Smoke test mission_manager |

## Exemplos rápidos

```bash
# Sensores reais
bash scripts/nicla/advr/apply_config.sh
bash scripts/lidar/test_fdpo_ros2.sh
bash scripts/lidar/view_rviz.sh

# Sim
forest up sim-sensors-only -d
forest up sim-mvp-nav -d
forest test patrol-rect

# Limpar tudo
forest down
```

## Pré-requisitos

```bash
cd ~/Projetos/Tese/forest-hybrid-ros2-stack
colcon build --symlink-install
export FORESTGEN_PATH=~/Projetos/Gazebo/ForestGen
```
