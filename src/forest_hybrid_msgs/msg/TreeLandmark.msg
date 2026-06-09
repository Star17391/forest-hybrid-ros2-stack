# Uma árvore/tronco detectado num único frame da perceção LiDAR (geométrico,
# SEM tracking nem fusão — isso é da camada de mapa). Pose-agnóstico: as
# coordenadas estão no frame do header do TreeLandmarkArray (tipicamente base_link).
geometry_msgs/Point base   # base do tronco (x, y, z) [m]
float32 diameter           # diâmetro estimado do tronco (DBH) [m]
float32 height             # altura observada do tronco [m]
float32 confidence         # confiança desta deteção neste frame [0..1]
