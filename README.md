# Forest Hybrid Robot — ROS 2 stack

Workspace **colcon** para o robô híbrido florestal (lagartas + coaxial quadcopter): drivers, perceção semântica, fusão LiDAR–câmara (futuro), localização/mapeamento e *bringup* centralizado.

Treino de redes (PC + GPU): repositório à parte em `../forest-semantic-training/`.

## Layout (espelha a filosofia do `fdpo-ros-stack`)

| Pasta no repo | Função |
|----------------|--------|
| `src/conf/` | Lançamentos e YAMLs (*single source of truth*) |
| `src/drivers_stack/` | Câmara (`usb_cam` baseline + `camera_ros` CSI), LiDAR (deps genéricas), atuadores |
| `src/perception_stack/` | Segmentação semântica, fusão nuvem (futuro) |
| `src/localization_mapping_stack/` | LIO / SLAM (*bringup* futuro) |
| `src/navigation_stack/` | Navegação |
| `src/planner_stack/` | Planeamento |
| `src/utilities_stack/` | Modo de operação (`/system/locomotion_mode`), sim, utilitários |
| `src/forest_hybrid_msgs/` | Mensagens partilhadas |
| `deploy/` | Systemd / *deploy* |

## Dependências sistema (exemplo Jazzy)

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-usb-cam ros-$ROS_DISTRO-camera-ros
```

Na **Raspberry Pi**, se o sensor não for detetado pelo `libcamera` empacotado no ROS, segue a nota do upstream: *fork* Raspberry Pi do `libcamera` — ver documentação de `camera_ros`.

## Build

```bash
cd /home/star17391/Projetos/Tese/forest-hybrid-ros2-stack
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Bringup (camada câmara + modo + segmentação ONNX/placeholder)

```bash
source install/setup.bash
ros2 launch forest_hybrid_conf forest_bringup.launch.py
# modo aéreo (segmentação deixa de processar imagens):
ros2 launch forest_hybrid_conf forest_bringup.launch.py operation_mode:=aerial
# forçar backend CSI (quando disponível):
ros2 launch forest_hybrid_conf forest_bringup.launch.py camera_backend:=csi
# usar modelo ONNX:
ros2 launch forest_hybrid_conf forest_bringup.launch.py onnx_model_path:=/path/model.onnx
```

### Tópicos (raiz)

| Tópico | Tipo | Produtor | Notas |
|--------|------|-----------|--------|
| `/camera/image_raw` | `sensor_msgs/Image` | `usb_cam` ou `camera_ros` | Nó com `name:=camera` |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | `usb_cam` ou `camera_ros` | |
| `/system/locomotion_mode` | `forest_hybrid_msgs/OperationMode` | `operation_mode_node` | `mode_name`: `ground`/`aerial`; QoS *transient local* |
| `/perception/semantic_mask` | `sensor_msgs/Image` | `semantic_segmentation_node` | `mono8`: ID de classe por pixel |

Parâmetros úteis: `operation_mode_node` lê `operation_mode`; segmentação lê `onnx_model_path`, `model_input_width`, `model_input_height`.

## LiDAR (sem código ainda)

Ver `src/drivers_stack/forest_lidar_ros2/DEPENDENCIES.md` e `package.xml` — dependências genéricas para quando integrares o driver.

## Estado

Bringup mínimo com **C++** (`operation_mode_node`, `semantic_segmentation_node`) e câmara por `usb_cam` (default) ou `camera_ros` (CSI). A versão atual já suporta inferência ONNX com fallback seguro para máscara zero.

## Contratos congelados

- Ver [`docs/LAYER_CONTRACTS.md`](docs/LAYER_CONTRACTS.md)
- Versão YAML para integração: `src/conf/forest_hybrid_conf/config/layer_contracts.yaml`
