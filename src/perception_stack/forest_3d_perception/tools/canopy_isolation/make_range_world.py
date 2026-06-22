"""
Gera mundos Gazebo CONTROLADOS para diagnosticar a perceção de tronco vs distância.

Cada mundo tem terreno PLANO (flat_ground), o robô parado na origem virado para +X,
e as 6 espécies (Tree1..6) num ARCO FRONTAL, TODAS à mesma distância `d`. Como o
LiDAR está inclinado (os feixes da FRENTE apontam para baixo e vêem o fuste; os de
trás apontam para cima), as árvores ficam no setor frontal -> robô não precisa rodar.

GT perfeito: sabemos espécie + distância + posição (x,y) exatas de cada árvore.
Comparar a MESMA espécie entre mundos isola limpa a variável distância.

Saída: $FORESTGEN/worlds/trunk_range_d<d>.sdf  (world name "unified_world" p/ os bridges).
Uso:   python make_range_world.py 4 8 12
"""
import math
import os
import sys

FORESTGEN = os.environ.get("FORESTGEN_PATH", "/home/star17391/Projetos/Gazebo/ForestGen")
WORLDS = os.path.join(FORESTGEN, "worlds")

SPECIES = [1, 2, 3, 4, 5, 6]
# azimutes no setor frontal (graus, 0 = +X = à frente do robô), bem separados
AZIMUTHS_DEG = [-50, -30, -10, 10, 30, 50]

HEADER = """<?xml version="1.0" encoding="utf-8"?>
<!-- GERADO por make_range_world.py: diagnóstico de tronco vs distância ({d} m).
     6 espécies em arco frontal, terreno plano, robô parado na origem. -->
<sdf version="1.9">
  <world name="unified_world">
    <physics name="3ms_dart" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/Sun</uri>
    </include>
    <include>
      <uri>model://flat_ground</uri>
    </include>
    <include>
      <uri>model://forest_tracked_robot</uri>
      <name>marble_hd2</name>
      <pose>0 0 0.35 0 0 0</pose>
    </include>
"""

TREE = """    <include>
      <uri>model://Tree{tid}</uri>
      <name>tree{tid}_d{d}_az{az}</name>
      <pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}</pose>
    </include>
"""

FOOTER = "  </world>\n</sdf>\n"


def make_single(tid, d):
    """UMA árvore isolada à frente do robô (az=0), à distância d. Cluster sempre limpo.

    Distâncias curtas com várias árvores fundem clusters (copas largas fazem ponte);
    uma árvore por mundo elimina isso e dá GT trivial: o único cluster É a árvore.
    """
    parts = [HEADER.format(d=d)]
    yaw = math.pi  # tronco a encarar o robô (irrelevante p/ cilindro vertical)
    parts.append(TREE.format(tid=tid, d=d, az=0, x=float(d), y=0.0, yaw=yaw))
    parts.append(FOOTER)
    path = os.path.join(WORLDS, f"trunk_one_t{tid}_d{d}.sdf")
    with open(path, "w") as f:
        f.write("".join(parts))
    print(f"escrito {path}   (Tree{tid} a ({d:.1f},0) — dist={d}m, az=0)")


def make_surround(dists, n_az=8):
    """Anéis de árvores a RODEAR o robô (todas as direções), nas distâncias `dists`.

    Para testar o caminho B (deteção SEM solo): com o LiDAR inclinado, as árvores
    à FRENTE têm solo (caminho A) e as de TRÁS/LADO não têm (caminho B). Um anel
    completo a 4 e 8 m põe árvores nas duas situações ao mesmo tempo, parado na
    origem. Espécies alternadas por azimute para variar o DBH. GT = posições do anel.
    """
    parts = [HEADER.format(d="+".join(str(int(x)) for x in dists))]
    az_step = 360.0 / n_az
    for d in dists:
        for k in range(n_az):
            az = (k * az_step + (0.0 if int(d) % 2 == 0 else az_step / 2.0))
            tid = SPECIES[k % len(SPECIES)]
            rad = math.radians(az)
            x = d * math.cos(rad)
            y = d * math.sin(rad)
            parts.append(TREE.format(tid=tid, d=f"{int(d)}", az=f"{int(az)}",
                                     x=x, y=y, yaw=math.pi))
    parts.append(FOOTER)
    name = "trunk_surround_d" + "_d".join(str(int(x)) for x in dists)
    path = os.path.join(WORLDS, f"{name}.sdf")
    with open(path, "w") as f:
        f.write("".join(parts))
    print(f"escrito {path}   ({n_az} árvores/anel × {len(dists)} anéis a {dists} m, a rodear)")


if __name__ == "__main__":
    # uso: make_range_world.py <tid> <d1> [d2 ...]            (uma árvore por mundo)
    #      make_range_world.py surround <d1> [d2 ...] [--n=8]  (anéis a rodear)
    if len(sys.argv) < 3:
        print("uso: make_range_world.py <tid 1..6> <dist...>   (ex.: 2 4 8 12)")
        print("     make_range_world.py surround <dist...> [--n=N]  (ex.: surround 4 8)")
        sys.exit(2)
    os.makedirs(WORLDS, exist_ok=True)
    if sys.argv[1] == "surround":
        n_az = 8
        rest = []
        for a in sys.argv[2:]:
            if a.startswith("--n="):
                n_az = int(a.split("=")[1])
            else:
                rest.append(int(a))
        make_surround(rest, n_az=n_az)
    else:
        tid = int(sys.argv[1])
        dists = [int(x) for x in sys.argv[2:]]
        for d in dists:
            make_single(tid, d)
