# Diagnóstico — pipeline experimental LiDAR 3D

**Objetivo:** localizar onde a funnel quebra (evidência, não suposições).  
**Data:** análise estática + instrumentação `debug_stage` / `debug_stats`.

---

## 1. Mapa da funnel (código actual)

```
/sensors/lidar/points  (frame: laser)
    → TF → marble_hd2/base_link     [falha silenciosa: return sem publish]
    → crop (range + z band)         [falha: n_crop < 30 → return]
    → voxel 0.08 m
    → CSF do_filtering              [Test 2]
    → non_ground
    → Euclidean cluster             [Test 3]
    → publish
```

**Tópicos publicados (nomes reais):**

| Esperado pelo utilizador | Tópico real no código |
|--------------------------|------------------------|
| `/ground_points` | **não existe** → `/perception/lidar3d/experimental/ground` |
| `/non_ground_points` | **não existe** → `/perception/lidar3d/experimental/non_ground` |
| `/clusters` | **não existe** → `/perception/lidar3d/experimental/clusters` |
| `/tree_candidates` | Sprint 2 stub — **não implementado** |

Se o RViz ou documentação usarem os nomes da coluna esquerda, **parece que “não há dados”** mesmo com pipeline a funcionar.

---

## 2. Falhas silenciosas (antes da instrumentação)

O nó original fazia `return` **sem publicar** `debug_stats` em:

| Condição | `status` JSON (agora) |
|----------|------------------------|
| `enabled:=false` | `disabled` |
| TF `laser` → `base_link` falha | `tf_fail` |
| PCL vazio após TF | `raw_empty` |
| `n_crop < 30` | `crop_too_few` |
| voxel vazio | `voxel_empty` |

**Prova:** com stack a correr, `ros2 topic echo /perception/lidar3d/experimental/debug_stats` — se vazio, o nó não está activo ou não recebe input.

---

## 3. Crash CSF (exit -11 / SIGSEGV) — **CAUSA RAIZ PROVADA E ELIMINADA**

**Sintoma:** `lidar3d_experimental_node` morre no 1.º frame, sempre após `Simulating...` / `post handle` ou em `Rasterizing` (exit code -11).

**Causa raiz (provada, não suposta):**
O LiDAR do Gazebo emite **pontos não-finitos (NaN/Inf)** para raios sem retorno
(contagem fixa `11520` = grelha organizada; sem eco = NaN). O CSF não consegue
processá-los e nenhuma etapa anterior os filtra:

1. `crop_cloud` **não** os remove — toda a comparação `<`/`>` com NaN é falsa,
   logo o ponto passa todos os testes de range/z.
2. No CSF, em runtime:
   - **Inf** → `computeBoundingBox` fica `inf` → `width = floor(inf/res)` →
     overflow para `-2147483644` → cloth inválido → SIGSEGV em `Rasterizing`.
   - **NaN** → `c2cdist`: `int(NaN/step)` = `INT_MIN` (-2147483648) →
     `particles[row*width + col]` lê ~`-1.3e11` fora do array → SIGSEGV.

**Prova reproduzível (offline, sem Gazebo):**

```bash
bash scripts/test_csf_experimental_offline.sh
```

O teste injecta NaN/Inf como o sensor real:
- com CSF **pristino + sem guarda** → `EXIT=139` (SIGSEGV), `bbMax: inf`, `width: -2147483644`.
- com a guarda → saída **idêntica** à da nuvem limpa (lixo rejeitado), `EXIT=0`.

**Cura na raiz (sem pensos):** rejeitar pontos não-finitos à entrada, antes de
qualquer geometria. A biblioteca CSF vendored ficou **pristina** (sem clamps).

| Camada | Ficheiro | Acção |
|--------|----------|-------|
| Ingestão | `src/lidar3d_experimental_node.cpp` `crop_cloud` | descarta NaN/Inf + `is_dense=true` |
| Fronteira do componente | `experimental/csf_ground_segmentation.hpp` | só pontos finitos entram no CSF, índices mapeados de volta |
| Observabilidade | `pipeline_debug.hpp` | `n_non_finite` no `debug_stats` |

---

## 4. Nó experimental não arranca

O perfil `sim-lidar3d-test` **não** inclui `use_experimental_lidar3d: true`.

| Comando | Experimental node |
|---------|-------------------|
| `forest up sim-lidar3d-test -d` | **NÃO** |
| `forest up sim-lidar3d-experimental -d` | **SIM** |
| `forest up sim-lidar3d-test -d --lidar3d-experimental` | **SIM** |

**Prova:** `ros2 node list | grep experimental`

---

## 4. Testes binários (`debug_stage`)

Após `colcon build` e `forest up sim-lidar3d-experimental -d`:

```bash
# Test 1 — TF + crop + voxel (sem CSF)
ros2 param set /lidar3d_experimental_node debug_stage 1
# Esperado: ground topic = voxel cloud; debug_stats status=ok, n_ground≈n_voxel

# Test 2 — CSF only
ros2 param set /lidar3d_experimental_node debug_stage 2
# Esperado: ground + non_ground com pontos; clusters vazio

# Test 3 — clustering only (CSF bypass)
ros2 param set /lidar3d_experimental_node debug_stage 3
# Esperado: clusters > 0 se voxel tem estrutura

# Test 4 — full
ros2 param set /lidar3d_experimental_node debug_stage 0
```

Tópicos de debug por etapa:

- `/perception/lidar3d/experimental/debug/stage_voxel`
- `/perception/lidar3d/experimental/debug/stage_ground`
- `/perception/lidar3d/experimental/debug/stage_non_ground`
- `/perception/lidar3d/experimental/debug/stage_clusters`

---

## 5. Hipóteses CSF (a validar com Test 2)

CSF foi desenhado para LiDAR aéreo. Em mobile forestry:

- `cloth_resolution: 0.5` vs voxel `0.08` — escala muito grosseira
- `class_threshold: 0.5` — pode classificar quase tudo como ground ou tudo como object
- `rigidness: 3` — terreno florestal pode precisar 1–2

**Prova esperada no JSON:** `csf_ground_pct` ≈ 0% ou ≈ 100% com `n_voxel` grande.

---

## 6. Comandos de auditoria

```bash
forest up sim-lidar3d-experimental -d --world forest_rugged_trees_rocks
# Gazebo PLAY

forest diag lidar3d-exp-audit --duration 20

ros2 topic echo /perception/lidar3d/experimental/debug_stats --once
```

---

## 7. Conclusão interina (pré-runtime)

| Prioridade | Causa provável | Como provar |
|------------|----------------|-------------|
| P0 | Tópicos RViz errados (`/ground_points` vs `experimental/ground`) | `ros2 topic list` |
| P0 | Nó não lançado (perfil/flag) | `ros2 node list` |
| P1 | Saída precoce TF/crop sem feedback | `debug_stats.status` |
| P2 | CSF esvazia ou monopoliza uma classe | Test `debug_stage:=2`, `csf_ground_pct` |
| P3 | Clustering com poucos non-ground | Test `debug_stage:=3` |

**Não corrigir CSF até Test 1–3 localizarem a etapa exacta.**
