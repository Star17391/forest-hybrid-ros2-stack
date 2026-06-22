# Estado interno do Tree-SLAM e indicador de autoridade TF map→odom.
# Publicado em /slam/status; consumido pelo EKF global e pelo mapa de traversabilidade.
#
# Regra de autoridade TF:
#   mode == GROUND  → forest_tree_slam   é o único publisher de map→odom
#   mode == AERIAL  → forest_state_estimation (EKF SE3) é o único publisher de map→odom
#   Nunca dois publishers de map→odom em simultâneo.
#   A comutação é explícita: o nó que perde autoridade para de publicar TF antes
#   de o outro começar; ver /system/locomotion_mode como sinal auxiliar.

std_msgs/Header header

uint8 GROUND=0        # solo: fatores de tronco ativos; Tree-SLAM dono de map→odom
uint8 AERIAL=1        # ar: map→odom congelado; aresta SE2 do voo acumula no grafo via EKF/ArduPilot
uint8 RELOCALIZING=2  # aterragem: relocalizador TreeLoc ativo com prior do autopilot
uint8 LOST=3          # zero associação prolongada; alerta à missão; dead-reckoning IMU/wheel

uint8 mode

uint32  n_landmarks_tracked      # nº de landmarks ativos no mapa neste instante
float32 pose_covariance_trace    # traço da covariância 3x3 da pose corrente [m²]; indicador de qualidade
bool    owns_map_to_odom         # true enquanto este nó é o publisher autorizado de TF map→odom
