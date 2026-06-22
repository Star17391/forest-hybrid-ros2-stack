# Ablação das pernas prismáticas (ground → air)

## Hipótese

A queda observada ao `to_aerial` pode ser:

- **A)** causada pelo deploy das pernas (`leg_extension_deployed_m=0.17`), ou  
- **B)** causada por outro factor (lagartas, motores tardios, TWR, contacto), com as pernas só a amplificar.

## Variáveis (uma de cada vez)

| Caso | Alteração | Resto |
|------|-----------|-------|
| `baseline` | (nenhuma) | deploy 0.17 m, FSM rápido |
| `test1` | `hybrid_leg_deployed_m:=0.0` | FSM igual |
| `test2` | dwell 3s / 2s / 2s entre fases | pernas 0.17 m |
| `test3` | `hybrid_disable_leg_commands:=true` | FSM avança sem mover pernas |

## Execução

**Importante:** Gazebo em **PLAY** antes de Enter. Se o FSM não sair de `GROUND_DRIVE`, ver `sim.log` (stack morto por SIGINT) e confirmar `transition_request subscribers: 1` no probe.

Comando manual equivalente ao `to_aerial` fiável:

```bash
ros2 topic pub --once /forest_gen/hybrid/transition_request std_msgs/msg/String "{data: to_aerial}"
```

```bash
cd ~/Projetos/Tese/forest-hybrid-ros2-stack
source install/setup.bash

forest test hybrid-leg-ablation --case baseline
# PLAY no Gazebo → Enter

forest test hybrid-leg-ablation --case test1
# ...

forest test hybrid-leg-ablation --case all
```

Registo automático: `ros2 run forest_sim_bridge hybrid_leg_ablation_probe`

Métricas:

- `fall_pre_fly`: Δz ≥ 0.08 m em LOCK/LEGS_EXTENDING/TRACKS_ROTATING  
- `runaway`: base_z ou pose_fused ≥ 10 m  
- `aerial_fly`: FSM chegou a voo  

## Critério de decisão

| Resultado | Conclusão |
|-----------|-----------|
| test1 sem queda, baseline com queda | **Pernas = causa forte** |
| test1 ainda com queda | **Pernas ≠ causa raiz** (procurar lagartas/motores/contacto) |
| test2 reduz queda, test1 não | timing + deploy combinados |
| test3 ≈ baseline | movimento das pernas importa; test3 só isola comando |

## Runaway (fase 2)

Depois da ablação:

```bash
forest test hybrid-leg-ablation --case baseline --runaway-probe
# ou
ros2 run forest_sim_bridge hybrid_runaway_probe
```

Verifica: `aerial_cmd_vel.vz`, `gz … motor_speed`, `multicopter_enabled` (Lee stock).

Não correr `hybrid-aerial-lift` attach com robô já a centenas de metros — `forest down/up` antes.
