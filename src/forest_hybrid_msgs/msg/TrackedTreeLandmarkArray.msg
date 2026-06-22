# Mapa de landmarks de troncos rastreados pelo Tree-SLAM.
# Publicado em /slam/tree_map; consumido pela camada de mapa de traversabilidade.
# header.frame_id = "map"; header.stamp = instante da última atualização do mapa.
std_msgs/Header header
TrackedTreeLandmark[] trees
