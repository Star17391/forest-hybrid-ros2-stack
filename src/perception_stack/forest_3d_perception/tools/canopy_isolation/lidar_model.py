"""
Modelo fiel do LiDAR 3D do robô (forest_tracked_robot_lidar3d/model.sdf).

Sensor `front_laser` (gpu_lidar):
  pose no base_link: x=0.40, y=0, z=0.36, pitch=+0.218166156 rad (~12.5° para cima)
  horizontal: 360 amostras, azimute -pi..pi
  vertical:   32 amostras, elevação 0..pi/2 (do horizonte do sensor até zénite)
  range: 0.1 .. 60.0 m

base_link assenta a ~0.11 m do solo (tracks 0.22 m de altura, centro em z=0),
logo o sensor fica a ~0.47 m acima do solo plano.

Estas constantes são a ÚNICA fonte de verdade do padrão de feixes nos testes
offline — têm de espelhar o SDF. Se o SDF mudar, atualizar aqui.
"""
import numpy as np

# --- Constantes do SDF (não alterar sem alterar o model.sdf) ---
SENSOR_XYZ_IN_BASE = np.array([0.40, 0.0, 0.36])
SENSOR_PITCH = 0.218166156  # rad, rotação em torno de Y (nariz para cima)
BASE_LINK_HEIGHT = 0.11     # base_link acima do solo plano [m]

H_SAMPLES = 360
H_MIN, H_MAX = -np.pi, np.pi
V_SAMPLES = 32
V_MIN, V_MAX = 0.0, np.pi / 2.0
RANGE_MIN, RANGE_MAX = 0.1, 60.0


def _rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def beam_directions_sensor():
    """Direções unitárias dos feixes no frame do sensor (antes do pitch de montagem).

    Devolve (N,3) com N = V_SAMPLES * H_SAMPLES.
    """
    # resolution=1 em ambos -> amostras inclusivas nos extremos.
    az = np.linspace(H_MIN, H_MAX, H_SAMPLES, endpoint=False)
    el = np.linspace(V_MIN, V_MAX, V_SAMPLES, endpoint=True)
    AZ, EL = np.meshgrid(az, el)  # (V,H)
    ce = np.cos(EL)
    dirs = np.stack(
        [ce * np.cos(AZ), ce * np.sin(AZ), np.sin(EL)], axis=-1
    ).reshape(-1, 3)
    return dirs


def rays_in_ground_frame(robot_x, robot_y, robot_yaw=0.0, ground_z=0.0):
    """Origens e direções dos feixes no frame do solo (z=0 = solo plano).

    O robô está em (robot_x, robot_y) com guinada robot_yaw; base_link a
    BASE_LINK_HEIGHT acima do solo. Devolve (origin (3,), dirs (N,3)).
    """
    # Orientação do sensor no mundo = yaw(robot) * pitch(montagem).
    cyaw, syaw = np.cos(robot_yaw), np.sin(robot_yaw)
    R_yaw = np.array([[cyaw, -syaw, 0], [syaw, cyaw, 0], [0, 0, 1]])
    R_sensor = R_yaw @ _rot_y(SENSOR_PITCH)

    # Posição do sensor no mundo.
    base_origin = np.array([robot_x, robot_y, ground_z + BASE_LINK_HEIGHT])
    sensor_offset_world = R_yaw @ SENSOR_XYZ_IN_BASE
    sensor_origin = base_origin + sensor_offset_world

    dirs = beam_directions_sensor() @ R_sensor.T
    return sensor_origin, dirs
