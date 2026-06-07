# Estado de um salto aéreo híbrido em curso (publicado por hybrid_hop_executor em
# /forest_gen/hybrid/hop_status). O mission_manager espera STATE_DONE antes de
# retomar/re-planear a missão de solo a partir da nova posição.

uint8 STATE_IDLE=0
uint8 STATE_IN_PROGRESS=1
uint8 STATE_DONE=2
uint8 STATE_FAILED=3

uint8 state
string command_id        # ecoa o command_id do HybridHopRequest em curso
string phase             # sub-fase atual (ex. "TAKEOFF", "CRUISE", "LANDING", "TO_GROUND")
string detail            # texto livre de diagnóstico
