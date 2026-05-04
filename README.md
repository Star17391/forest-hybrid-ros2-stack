# Forest Hybrid Robot — ROS 2 stack

Workspace colcon para o robô híbrido florestal (lagartas + coaxial quadcopter): drivers, perceção semântica, fusão LiDAR–câmara, localização/mapeamento e *bringup* centralizado.

## Layout (espelha a filosofia do `fdpo-ros-stack`)

| Pasta no repo | Função |
|----------------|--------|
| `src/conf/` | Lançamentos e YAMLs (*single source of truth* para composição do sistema) |
| `src/drivers_stack/` | Câmara, LiDAR 3D, atuadores, ponte com firmware |
| `src/perception_stack/` | Segmentação semântica e enriquecimento de nuvens de pontos |
| `src/localization_mapping_stack/` | Odometria LIO / SLAM (integrações e *launch* wrappers) |
| `src/navigation_stack/` | Navegação (ex.: Nav2, controladores) |
| `src/planner_stack/` | Planeamento de missão e trajetória |
| `src/utilities_stack/` | Supervisor, logging, calibrações, utilitários |
| `src/forest_hybrid_msgs/` | Mensagens e serviços partilhados |
| `deploy/` | Systemd e notas de *deploy* (não é pacote ROS) |

## Build (ROS 2 Humble ou Jazzy)

```bash
cd /home/star17391/Projetos/Tese/forest-hybrid-ros2-stack
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Substitui `$ROS_DISTRO` pela distro instalada no Raspberry Pi / PC de desenvolvimento.

## Estado

Estrutura inicial: pacotes esqueleto sem nós implementados; próximos passos alinhados com o plano da tese (perceção, fusão, LIO).
