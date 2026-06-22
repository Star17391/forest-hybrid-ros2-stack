# Uma árvore/tronco detectado num único frame da perceção LiDAR (geométrico,
# SEM tracking nem fusão — isso é da camada de mapa). Pose-agnóstico: as
# coordenadas estão no frame do header do TreeLandmarkArray (tipicamente base_link).
geometry_msgs/Point base   # base do tronco (x, y, z) [m] em base_link
float32 diameter           # diâmetro estimado do tronco (DBH) [m]
float32 height             # altura observada do tronco [m]
float32 confidence         # confiança desta deteção neste frame [0..1]

# --- Incerteza por deteção (necessária para gate de Mahalanobis no SLAM) ---
# Campos novos no fim para não partir consumidores legados (costmap, viz).
float64[9] base_covariance  # covariância 3x3 row-major sobre (x,y,z) da base [m²]; zeros = não preenchida
float32    diameter_stddev  # desvio-padrão do DBH [m]; 0 = não preenchido

# --- Semântica por deteção (probabilidades; decisão de classe fica no SLAM) ---
float32[3] class_scores     # probabilidades por classe [tronco, rocha, obstáculo]; soma ≈ 1
uint8      semantic_class   # argmax de class_scores (só debug/RViz); o SLAM usa class_scores
