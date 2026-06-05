# Autopilot SITL — instalação e arranque (M0/M1)

Scripts para construir as toolchains de **ArduPilot** e **PX4** SITL e ligá-las ao Gazebo
(gz Harmonic / Sim 8) neste ambiente (Ubuntu 24.04, ROS 2 Jazzy).

> Plano e milestones: [`../../docs/AUTOPILOT_PX4_VS_ARDUPILOT_PLAN.md`](../../docs/AUTOPILOT_PX4_VS_ARDUPILOT_PLAN.md)
> Fundamentação: [`../../docs/HYBRID_AERIAL_CONTROL_RESEARCH.md`](../../docs/HYBRID_AERIAL_CONTROL_RESEARCH.md)

## ⚠️ Correr na TUA sessão, não em background

Estes builds são **pesados** (vários GB, `sudo apt`, compilação longa) e têm passos
interativos. Corre-os tu (ex.: `! bash scripts/autopilot/install_ardupilot_sitl.sh`).
Os scripts são **idempotentes** — podes voltar a correr se algo falhar a meio.

## Ordem recomendada

| Passo | Comando | O que faz | Tempo aprox. |
|---|---|---|---|
| 1 | `bash scripts/autopilot/install_ardupilot_sitl.sh` | ArduPilot SITL + plugin `ardupilot_gazebo` | 20–40 min |
| 2 | `bash scripts/autopilot/install_px4_sitl.sh` | PX4-Autopilot SITL (gz) + agente uXRCE-DDS | 30–60 min |
| 3 | `source scripts/autopilot/autopilot_env.sh` | Exporta PATH/GZ env das duas toolchains | — |

Começa pelo **ArduPilot** (M1 quad padrão chega a hover mais cedo). Só depois PX4.

## Verificação M1 — via `forest` CLI (recomendado)

Não há comandos soltos: tudo pelo CLI. Depois de instalar (acima), valida o hover com:

```bash
forest test hybrid-aerial-sitl --assert
```

Orquestra gz + ArduPilot SITL + arm/takeoff, **assere o hover** e imprime as métricas
(§3.2 do plano). Opções: `--alt`, `--hover`, `--tol`, `--gui` (mostrar Gazebo), `--keep`.
Headless por omissão; usa partição de transporte isolada (não choca com outra sim aberta).

Se passar, a ponte+EKF estão validados (M1) → avançamos para **M2** (geometria ±90° do
`forest_hybrid_robot`).

PX4 (quando instalado) — smoke test nativo:
```bash
cd ~/PX4-Autopilot && make px4_sitl gz_x500
```

## Notas
- Diretórios configuráveis: `ARDUPILOT_DIR` (def. `~/ardupilot`), `PX4_DIR` (def. `~/PX4-Autopilot`).
- Não removem nada existente; clonam só se faltar.
- Em caso de falha de dependência, o erro do `apt`/`waf`/`make` **também é resultado útil**
  para documentar no plano (§7 riscos).
