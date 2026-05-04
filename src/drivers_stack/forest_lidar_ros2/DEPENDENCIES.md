# forest_lidar_ros2 — dependências (pré-LiDAR)

Este pacote ainda **não** contém o driver do teu LiDAR 3D (modelo por escolher). O `package.xml` inclui dependências **genéricas** usadas em quase qualquer *pipeline* de nuvem de pontos em ROS 2:

| Pacote ROS 2 | Uso típico |
|--------------|------------|
| `sensor_msgs` | `PointCloud2`, `LaserScan` |
| `geometry_msgs` | `Transform`, poses |
| `tf2`, `tf2_ros`, `tf2_sensor_msgs` | TF sensor → `base_link` |
| `pcl_conversions`, `pcl_msgs` | Ponte PCL ↔ mensagens ROS |

## Quando tiveres o sensor

1. Adiciona o pacote do **fabricante** (ou driver da comunidade) ao workspace ou via `apt`, consoante o modelo.
2. Regista-o em `rosdep` / `package.xml` deste pacote ou num pacote filho `forest_lidar_<vendor>`.
3. Garante **calibração extrínseca** relativamente à câmara antes da fusão semântica 3D.

## Resolver dependências

```bash
rosdep install --from-paths src/drivers_stack/forest_lidar_ros2 --ignore-src -r -y
```

Se alguma chave falhar na tua distro, ajusta o nome do pacote em `package.xml` conforme `rosdep resolve <chave>`.
