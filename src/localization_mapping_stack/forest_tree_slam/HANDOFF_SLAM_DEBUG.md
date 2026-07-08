# Handoff — Diagnosticar e corrigir o Tree-SLAM (deriva de landmarks)

> Documento de passagem para um chat novo. Lê isto **todo** antes de tocar em código.
> Objetivo único: perceber e corrigir porque é que o Tree-SLAM **coloca as árvores reais
> no sítio errado** (deriva), e fazê-lo com corridas autónomas mensuráveis.

---

## 0. Como o utilizador quer que trabalhes (LER PRIMEIRO)

Estas regras vêm de correções diretas do utilizador. Não as quebres.

1. **NÃO adivinhes. Faz debug real.** Mede antes de mudar. O utilizador já te apanhou
   a recomendar coisas que "já estavam implementadas" e a chamar "fantasmas" a deteções
   que afinal eram reais mal-colocadas. Antes de propor um fix, prova a causa com dados
   (números de uma corrida, não intuição).

2. **Os "fantasmas" NÃO são fantasmas.** São árvores **reais** que o SLAM coloca no sítio
   errado por **deriva de pose/backend** (RMSE ~0.65–1.24 m, em casos ~1.9 m por landmark).
   O filtro de verticalidade **já existe** — não o reimplementes. O problema está no SLAM
   (pose/backend), não na perceção nem na classificação.

3. **Uma camada de cada vez, com teste.** Ao mudar código, atualiza também o CLI `forest`
   e o RViz e **dá o comando de teste exato**. "Compila" não chega — tem de ser
   demonstrável numa corrida.

4. **Sem atrasos artificiais no arranque** (espera por readiness, não `sleep`).
   **Sem tradutores de tópico.** O **costmap local tem de ser reativo, ponto final.**
   Não mexas nas frequências de publicação do mapa.

5. **Modo auto:** podes correr a simulação e os testes que precisares. Mas se o utilizador
   disser que vai correr OUTRO simulador, **pede antes de correr** e espera. Lê sempre a
   última mensagem dele quanto a isto.

6. **Português, com acentuação correta.** Termos técnicos e identificadores de código ficam
   no original.

7. **Memória.** Há memória persistente em
   `/home/star17391/.claude/projects/-home-star17391-Projetos/memory/`. Lê o `MEMORY.md`
   (índice) e os ficheiros relevantes (ver §8). Atualiza-os quando descobrires algo novo
   e durável; não dupliques.

---

## 1. Onde está o código

Repo: `/home/star17391/Projetos/Tese/forest-hybrid-ros2-stack` (ROS2 Jazzy, colcon).
ForestGen (mundos): `/home/star17391/Projetos/Gazebo/ForestGen`.

Pacote do SLAM: `src/localization_mapping_stack/forest_tree_slam/`
- `src/tree_slam_node.cpp` — o nó. Faz a ponte odom→keyframes→observações, publica
  `map→odom` (SE2), `/slam/tree_map`, `/slam/status`, e a diagnóstica `kf_wobble`.
- `src/backend.cpp` + `include/forest_tree_slam/backend.hpp` — GTSAM iSAM2 (o otimizador).
- `src/tracker.cpp` + `include/forest_tree_slam/tracker.hpp` — LandmarkTracker
  (nascimento/promoção/associação, gate de Mahalanobis, scorer dinâmico S, paralaxe).
- `config/tree_slam.yaml` — params do **tracker** (NÃO os do backend; ver §3).

Perceção (entrada do SLAM): `src/perception_stack/forest_3d_perception/` →
publica troncos detetados (bearing/range + classe + DBH). Pipeline vivo =
`lidar3d_experimental_node`. **Não é aqui o problema.**

---

## 2. Arquitetura do SLAM (modelo mental)

É um **SLAM 2D por landmarks** (árvores) sobre GTSAM/iSAM2, frame `map` (SE2):

- **Pose** do robô vem da odometria/EKF (frame `odom`). O SLAM acumula **keyframes**
  (gate: andou `keyframe_distance_m` OU rodou `keyframe_angle_rad`) ligadas por
  `BetweenFactor<Pose2>` (odometria).
- Cada **observação de tronco** entra como `BearingRangeFactor` ligada à keyframe mais
  recente. Observações entre keyframes são associadas à última keyframe (low-rate
  keyframe, high-rate observation).
- **Loop closure**: quando re-associa um landmark já visto (por geometria/RANSAC no
  relocalizador), os fatores partilham o mesmo nó do landmark → fecha o laço.
- **Saída**: `map→odom` = pose_map_da_keyframe × (odom→base)⁻¹, **projetado a SE(2)**
  (correção já feita — não voltar a achatar roll/pitch; ver memória
  `tree_slam_se2_flattens_attitude`).
- O tracker mantém `score` (S dinâmico) e `parallax_bins`; só promove landmarks a "verde"
  (confirmado) com S e paralaxe suficientes. O **mapping** (outra camada) é que desenha o
  cilindro do inventário — o SLAM **não** desenha cilindro, só fornece os dados.

### Diagnóstica que já existe no nó
`tree_slam_node.cpp` (~linha 600+) calcula **`kf_wobble`**: quanto a MESMA keyframe se
mexe entre dois `optimize()` sem keyframe nova nem loop closure. `kf_wobble` alto =
instabilidade INTERNA do backend. Há também um contador `next_uid`: se não pára de crescer
na 2.ª volta, NÃO há associação/loop closure (cada re-visita nasce landmark novo).
**Usa estes sinais — são a tua sonda primária.**

---

## 3. ⚠️ Achado crítico: os sigmas do backend NÃO são afináveis por YAML

`tree_slam_node.cpp:146` faz `backend_ = std::make_unique<TreeSlamBackend>();` **sem
passar nenhum `BackendParams`**. Logo, os parâmetros de ruído do GTSAM são os **defaults
hardcoded** em `include/forest_tree_slam/backend.hpp`:

```
keyframe_distance_m      = 0.75
keyframe_angle_rad       = 0.35   (~20°)
prior_pose_sigma         = {0.1, 0.1, 0.05}
default_odom_sigma       = {0.05, 0.05, 0.03}   ← ODOM MUITO CONFIANTE
default_bearing_sigma_rad= 0.05
default_range_sigma_m    = 0.15
robust_huber_k           = 1.345
```

O `tree_slam.yaml` só configura o **tracker**; nada disto chega ao backend. Suspeitas de
primeira ordem para a deriva:

- **`default_odom_sigma` demasiado apertado** (5 cm / 1.7°): o GTSAM acredita quase cego
  na odometria entre keyframes → não deixa as observações de troncos corrigir a pose →
  acumula deriva e empurra os landmarks. Em terreno irregular a odom derrapa muito mais
  que 5 cm.
- **`default_range_sigma_m = 0.15`** pode ser otimista para troncos vistos a 8–12 m com
  arco parcial (o DBH/centro são mal-condicionados a essa distância — ver memória
  `dbh_stability_arc_illposed`).
- **iSAM2** com `relinearizeThreshold=0.01`, `relinearizeSkip=1` + extra `isam_.update()`
  sem novos fatores (backend.cpp ~180): confirmar que converge e não fica a "wobble".

**Primeiro passo recomendado:** tornar os `BackendParams` configuráveis por YAML
(declará-los no nó e passá-los ao construtor) para poderes fazer um **varrimento A/B**
sem recompilar de cada vez. NÃO assumas qual o valor certo — mede `kf_wobble` e o RMSE
mapa×GT para cada candidato.

---

## 4. Como construir, correr e testar (o ciclo)

### Build
```bash
cd /home/star17391/Projetos/Tese/forest-hybrid-ros2-stack
colcon build --packages-select forest_tree_slam --symlink-install
source install/setup.bash
```

### Subir a simulação (Gazebo headless + RViz, poupa CPU)
```bash
forest up sim-tree-slam-nav2 -d --world forest_realistic_v2_trees_rocks --rviz_only
```
- `--rviz_only` = Gazebo `gz sim -s` (sem GUI) + só a janela do RViz.
- Mundo de referência: **`forest_realistic_v2_trees_rocks`** (55 árvores + 17 rochas +
  arbustos/troncos caídos inline; terreno irregular).
- **Perf:** se o robô anda a ~10 Hz / encrava, NÃO é o código — é o driver NVIDIA não
  carregado (render por software) + RAM/swap. Fecha Chrome/VSCode, confirma `nvidia-smi`.
  Ver memória `sim_perf_gpu_ram_not_code`.

### Corrida autónoma + avaliação mapa×GT (a tua métrica principal)
```bash
# robô conduz um laço fechado autónomo via nav2 e avalia /slam/tree_map vs GT
python3 tools/diagnostics/slam_race.py --world forest_realistic_v2_trees_rocks
# só planear o laço (sanidade): --plan-only
```
Reporta **recall, precisão, RMSE de posição, duplicados, falsos positivos, histograma de
confianças/classe, n_obs**, separando troncos de rochas. É aqui que se mede a deriva.
**Este é o número que define sucesso: baixar o RMSE de posição dos landmarks e parar os
"UIDs falsos" (births repetidos na 2.ª volta).**

### Trajetória: GT vs EKF-only vs Tree-SLAM
```bash
python3 tools/diagnostics/slam_trajectory_eval.py --duration <s> --out <png/csv>
```
Mostra se o SLAM melhora ou piora a pose face à odom pura. Se o Tree-SLAM ≈ EKF-only,
o backend não está a corrigir nada (sintoma do `default_odom_sigma` apertado).

### Diagnósticas dirigidas (auto-conduzem o robô)
```bash
forest diag tree-slam     # atribui o solavanco map->odom a uma camada (EKF/perceção/backend)
forest diag slam-trajectory
forest diag drive         # cadeia de navegação: diz o ELO exato que falha (S0..S8)
forest diag tree-slam-dbh # estabilidade DBH multi-view por track
```
(Lista completa: `forest diag` ou ver `tools/forest/lib/diag.bash`.)

### Método de trabalho sugerido
1. Corre `slam_race.py` **uma vez** no estado atual → guarda o JSON como **baseline**.
2. Forma UMA hipótese (ex.: "odom_sigma apertado causa deriva"). Prevê o efeito.
3. Faz a alteração mínima (idealmente um param). Recompila.
4. Corre `slam_race.py` de novo → compara com o baseline (RMSE, duplicados, `kf_wobble`).
5. Só aceita a mudança se os números melhorarem. Documenta o A/B.

---

## 5. Hipóteses ordenadas (por onde começar)

1. **`default_odom_sigma` demasiado confiante** → backend ignora correção dos troncos →
   deriva. (Mais provável. Torna afinável e sobe para algo realista, ex. {0.15,0.15,0.08}+.)
2. **Loop closure não dispara na 2.ª volta** → `next_uid` cresce, landmarks duplicam,
   sem fecho a deriva nunca é corrigida. Verifica o relocalizador/associação por geometria.
3. **`kf_wobble` alto** → instabilidade interna do iSAM2 (relinearize/convergência).
4. **`range_sigma`/`bearing_sigma` otimistas para troncos longe** → observações más puxam
   a pose. Considera `range_sigma` dependente do alcance/qualidade da deteção.
5. **Cobertura de observações** (memória `slam_end_to_end_validation`): se o cone frontal
   é estreito e o robô vai LOST a meio, o backend esfomeia. O caminho-B (deteção sem solo)
   já existe — confirma que está ativo na corrida.

> Nota da memória `slam_checkup_realistic`: já se concluiu que os "UIDs falsos" são
> **deriva ~1.9 m por landmark**, não births espúrios; e que a confiança realista feita
> foi insuficiente — **o próximo passo é pose/backend**. É exatamente aqui que entras.

---

## 6. O que NÃO mexer (já feito / fora de âmbito)

- **Filtro de verticalidade** — já existe.
- **Projeção SE(2) do `map→odom`** — corrigida (não voltar a achatar atitude do EKF).
- **EKF SE3 atitude** — funde roll/pitch absolutos do IMU (memória `ekf_se3_attitude_anchor`).
- **Visual dos arbustos** — já são aglomerados de esferas (memória `bush_visual_blob`).
- **nav2/RPP/mission bridge** — conduz end-to-end; não é o foco (memória
  `nav2_mvp_driving_fixes`). Serve só de driver para as corridas.
- **Frequências de publicação do mapa** — não tocar.
- **Scorer dinâmico S / paralaxe** — implementado. A visualização provisória
  castanho-translúcido → verde fixo (Fase 2) ficou por fazer, mas é **secundária**: o
  problema agora é a DERIVA, não o scorer.

---

## 7. Critério de sucesso

Numa corrida `slam_race.py` no `forest_realistic_v2_trees_rocks`:
- **RMSE de posição dos landmarks desce** de forma clara vs baseline (objetivo: bem < 0.5 m).
- **Sem UIDs falsos / duplicados** na 2.ª volta (loop closure funciona; `next_uid` estável).
- **`kf_wobble` baixo e estável.**
- Tree-SLAM **melhor que EKF-only** na trajetória.
- Tudo demonstrado com o comando de teste e o JSON da corrida, não por inspeção visual.

---

## 8. Memória relevante a ler (em memory/)

- `slam_checkup_realistic` — "UIDs falsos = deriva ~1.9 m; próximo = pose/backend".
- `slam_end_to_end_validation` — "backend otimiza mas cov_trace morto; vai LOST a meio".
- `slam_phantom_landmarks_diagnosis` — origem dos "fantasmas" (births sem gate; afinal deriva).
- `tree_slam_implementation`, `map_odom_dual_publisher`, `tree_slam_se2_flattens_attitude`.
- `dbh_stability_arc_illposed` — DBH/centro mal-condicionado a >8–10 m.
- `perception_path_b_no_ground` — deteção sem solo (alimenta o SLAM por trás/lado).
- `sim_perf_gpu_ram_not_code`, `forest_rviz_only_headless`, `forest_diag_drive_tool`.
- `forest_cli_testability_rule` — atualizar CLI + dar comando de teste a cada alteração.
