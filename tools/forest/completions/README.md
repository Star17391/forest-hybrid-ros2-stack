# Forest CLI — Tab completion

**Instalação (uma vez):**

```bash
bash ~/Projetos/Tese/forest-hybrid-ros2-stack/tools/forest/completions/install.sh
source ~/.bashrc
type forest   # deve mostrar: forest is a function
```

**Actualizar opções do Tab** (ex. `--lidar2d`, `--lidar3d`):

```bash
forest completion refresh
# ou, se ainda não tens o wrapper:
bash tools/forest/completions/sync_completions.sh
source tools/forest/completions/forest.bash
```

Não uses `scripts/` — o `install.sh` está aqui em `tools/forest/completions/`.
