# `forest_hybrid_conf`

Lançamentos e YAML centralizados.

| Launch | Conteúdo típico |
|--------|------------------|
| `forest_bringup.launch.py` | Câmara (USB/CSI), `operation_mode_node`, `mission_manager_node`, segmentação ONNX |
| `mission_layer_only.launch.py` | Só `mission_manager_node` (testes isolados) |

Contrato de tópicos (YAML): [config/layer_contracts.yaml](config/layer_contracts.yaml) — espelho de [docs/LAYER_CONTRACTS.md](../../../docs/LAYER_CONTRACTS.md).
