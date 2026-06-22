# Voo híbrido — desenho e solução

## O teu desenho (correcto)

| Modo | Lagartas (`track_yaw`) | Hélices |
|------|------------------------|---------|
| Terrestre | 0° — condução por esteiras | inactivas |
| Drone | ±90° (transformação) | empuxo |
| Em voo | **não** conduzem como em solo | controlam voo |

## Medição Gazebo (`forest test hybrid-gz-truth`)

Script `hybrid_gz_truth_probe`: roda lagartas ±90°, lê `gz model -l *_prop_*`, publica motores, mede `z`.

| link roll SDF (errado) | link+Z·world_z @ L+90° |
|------------------------|-------------------------|
| L +90°, R −90° nos **links** (antigo) | **−1.0** (empuxo para baixo) |
| L −90°, R +90° nos **links** (correcto) | **+1.0** nos 4 motores |

Com roll correcto o modelo **sobe** no teste headless (ajustar `motor_omega` para não runaway).

## Pião no RViz com robô parado no Gazebo

`marble_pose_from_gz` usava `dynamic_pose` sem `frame_id` → latch no índice com **mais movimento** (hélice). Correcção: fonte primária `world_tf_full` + latch por **estabilidade** (z≈spawn, baixo spin), não por movimento. `hybrid_gz_truth_probe --attach` evita segundo Gazebo quando `forest up` está activo. Auditar: `forest test hybrid-pose-audit`.

## MulticopterMotorModel — empuxo no +Z do **link**, não no eixo do joint

No `gz-sim` (`MulticopterMotorModel.cc`):

```cpp
link.AddWorldForce(_ecm, worldPose->Rot().RotateVector(Vector3(0, 0, thrust)));
```

O empuxo segue o **+Z do link da hélice**, não o eixo de rotação do joint.

| Pose lagarta | Link hélice sem rotação extra | Empuxo em Z mundo |
|--------------|------------------------------|-------------------|
| 0° | +Z link = +Z track | lateral (Y) |
| L +90° | +Z link = +Z track → mundo **−Y** | horizontal → **pião** no RViz |
| L +90° | link roll **+90°** (+Z link = +Y track) | **+Z** ✓ |
| R −90° | link roll **−90°** (+Z link = −Y track) | **+Z** ✓ |

**Correcção SDF:** `left_prop_*` link `pose` roll `−π/2`; `right_prop_*` roll `+π/2` (relativo ao joint).

## Solução adoptada no stack

1. SDF com frames de link correctos + `motorConstant` ~1.25e-4 (TWR > 1).
2. **`hybrid_aerial_motor_controller`** — matriz Lee após ±90°, `gz.transport` em `/marble_hd2/gazebo/command/motor_speed`.
3. **`hybrid_aerial_lift_diagnostic`** — `forest test hybrid-aerial-lift`.
4. Rotação `rotate_tracks_for_aerial:=true`, lagartas L=+90°, R=−90°; links hélice L=−90°, R=+90°.
5. FSM `base_z` via `/state/pose_fused` (Pose_V sem `child_frame_id` no bridge).

## Teste

```bash
forest test hybrid-aerial-lift --offline-only
forest down && forest up sim-hybrid-test -d   # recarregar SDF
forest test hybrid-aerial-lift --assert
```
