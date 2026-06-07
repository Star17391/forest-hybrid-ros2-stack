# Pedido de "salto aéreo" híbrido: o robô transforma-se em drone, descola, voa até
# um ponto de aterragem (livre/seguro) e volta ao modo terrestre.
#
# Disparável manualmente (tópico /forest_gen/hybrid/hop_request) ou pelo mission_manager
# quando o caminho de solo está bloqueado e o modo aéreo foi autorizado.
#
# O ponto de aterragem vem SEMPRE no pedido (quem dispara escolhe um sítio seguro).
# Coordenadas no frame 'map' (ENU). A conversão para NED do ArduPilot é feita no executor.

string command_id        # id para correlacionar com HybridHopStatus
string source            # quem originou (ex. "mission_manager", "operator")

float64 land_x           # X de aterragem (map, metros)
float64 land_y           # Y de aterragem (map, metros)
float64 cruise_alt_m     # altitude de cruzeiro/descolagem (metros acima do solo)
