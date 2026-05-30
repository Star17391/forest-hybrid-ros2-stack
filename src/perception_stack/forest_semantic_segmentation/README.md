# `forest_semantic_segmentation`

Nó C++ `semantic_segmentation_node`: entrada **`/camera/image_raw`**, modo **`/system/locomotion_mode`** (em modo *aerial* não processa frames), saída **`/perception/semantic_mask`** (ONNX opcional).

Parametrização:

- `onnx_model_path`
- `model_input_width`
- `model_input_height`
- `min_logit_margin` (opcional; margem mínima entre melhor e 2ª melhor classe.
  Quando não passa, publica classe `0`/void para reduzir falsos positivos)

Contrato: [docs/LAYER_CONTRACTS.md](../../../docs/LAYER_CONTRACTS.md).
