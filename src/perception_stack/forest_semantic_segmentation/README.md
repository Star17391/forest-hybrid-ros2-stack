# `forest_semantic_segmentation`

Nó C++ `semantic_segmentation_node`:

| Tópico | Tipo | Descrição |
|--------|------|-----------|
| `/camera/image_raw` | sub | Entrada RGB (`rgb8` / `bgr8`) |
| `/system/locomotion_mode` | sub | Em modo `aerial` não processa |
| `/perception/semantic_mask` | pub | `mono8`, ID de classe por pixel |
| `/perception/semantic_mask_color` | pub | `rgb8`, paleta para RViz |

## Modelo ONNX

Após treino:

```bash
bash scripts/perception/sync_semantic_onnx.sh
# ou: cp ~/Projetos/Tese/forest-semantic-training/artifacts/model.onnx \
#       src/perception_stack/forest_semantic_segmentation/models/forest_semantic.onnx

colcon build --packages-select forest_semantic_segmentation
source install/setup.bash
```

## Simulador (Gazebo + RViz)

```bash
source install/setup.bash
ros2 launch forest_semantic_segmentation sim_semantic.launch.py
```

No RViz (abre automaticamente):

1. **Camera RGB** → `/camera/image_raw`
2. **Semantic mask (color)** → `/perception/semantic_mask_color`

Cores (v2): cinza=void, azul claro=sky, verde=grass, castanho=dirt_mud, verde escuro=tree, verde claro=vegetation, vermelho=obstacle, cinza claro=pavement.

Pressiona **PLAY** no Gazebo. Verifica:

```bash
ros2 topic hz /perception/semantic_mask_color
```

## Nicla (câmara ADVR já a publicar `/camera/image_raw`)

Terminal 1 — receiver:

```bash
ros2 launch forest_nicla_vision_ros2 nicla_vision_advr.launch.py
```

Terminal 2 — segmentação:

```bash
source install/setup.bash
ros2 launch forest_semantic_segmentation semantic_only.launch.py
```

Opcional: RViz com `config/semantic_perception.rviz` (só painéis de imagem).

## Parâmetros

- `onnx_model_path` — default: `share/forest_semantic_segmentation/models/forest_semantic.onnx`
- `model_input_width` / `model_input_height` — 768×512 (treino v2)
- `min_logit_margin` — pixels incertos → classe 0 (void)
