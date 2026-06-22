# Robô híbrido — física aérea e diagnóstico

**Causa raiz (voo + hélices):** ver [HYBRID_AERIAL_ROOT_CAUSE.md](HYBRID_AERIAL_ROOT_CAUSE.md).

## Porque girava “como um tolo” no takeoff

Causas típicas (várias em simultâneo no modelo antigo):

1. **Massa ~36 kg** (soma dos `<mass>` no SDF), não 7 kg — o `MulticopterVelocityControl` usa ganhos pensados para drones pequenos (ex. X3 ~1.5 kg).
2. **`momentConstant` 0.28** (≈16× o X3) — pequenos erros de empuxo geram **torque de guinada enorme** → rotação violenta.
3. **`fly_up_velocity_z` 0.35** sem rampa — degrau grande na velocidade desejada → saturação dos motores.
4. **Multicopter ligado com pernas/lagartas no chão** — contactos e atrito nas lagartas a 90° aplicam torques que o controlador tenta corrigir agressivamente.
5. **Inércia assimétrica** (lagartas longas) com ganhos de atitude altos → acoplamento roll/pitch/yaw instável.

## Orçamento de massa (alvo ≤ 7 kg)

| Componente | Massa (kg) |
|------------|------------|
| `base_link` | 3.5 |
| 2× `*_track` | 0.9 cada |
| 4× `support_leg_*` | 0.12 cada |
| 4× `lift_rotor_*` | 0.025 cada |
| 4× hélice visual na lagarta | 0.015 cada |
| **Total** | **~5.9 kg** |

## Motores (ordem de grandeza)

Modelo Gazebo: empuxo ∝ `motorConstant × ω²`.

- X3: `motorConstant ≈ 8.5e-6`, massa ~1.5 kg.
- Híbrido 7 kg: `motorConstant ≈ 1.0e-4`, `momentConstant ≈ 0.018` (rácio próximo do X3).
- TWR máximo ~3–4 com `maxRotVelocity=800` — margem de hover sem saturar tudo.

## Testes automatizados

```bash
# Offline (rápido): massa, TWR, rank-4, avisos
forest test hybrid-physics --assert

# Headless Gazebo: enable + vz=0.12, limite de inclinação
forest test hybrid-aerial-probe --assert

# Missão completa (com sim a correr)
forest up sim-hybrid-test -d
forest test hybrid-mission --assert
```

Depois de alterar o SDF ou o stack:

```bash
cd ~/Projetos/Tese/forest-hybrid-ros2-stack
colcon build --packages-select forest_sim_bridge --symlink-install
source install/setup.bash
```
