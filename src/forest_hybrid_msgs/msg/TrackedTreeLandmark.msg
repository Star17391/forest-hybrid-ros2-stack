# Landmark de tronco já associado e otimizado pelo Tree-SLAM, em frame map.
# uid é imutável desde a 1.ª deteção: nunca muda por tile, loop closure ou
# re-otimização do grafo. Quando o loop closure move a pose, o uid segue-a.

uint64 uid                           # identidade imutável [atribuída monotonicamente na 1.ª deteção]

# Classe semântica — usar constantes de SemanticClass.msg (decisão do SLAM):
#   CLASS_VEGETATION_RIGID (3) → tronco; CLASS_ROCK (6) → rocha; CLASS_OBSTACLE (4) → obstáculo
uint8  semantic_class

geometry_msgs/Point position         # centróide da base do tronco em frame map [m]
float32 diameter                     # DBH estimado (média ponderada das observações) [m]
float32 height                       # altura observada máxima [m]
float32 confidence                   # confiança acumulada após fusão [0..1]

# Covariância 3x3 row-major sobre o vetor de estado (x, y, DBH) em map [m², m², m²].
# Ordem das variáveis: [pos_x, pos_y, diameter].
float64[9] covariance

uint32 n_observations                # número total de frames em que este landmark foi detetado
builtin_interfaces/Time last_seen    # stamp do último scan que observou este landmark
