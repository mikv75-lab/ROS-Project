# spraycoater_bringup/launch/robot_and_servo.launch.py
#!/usr/bin/env python3
import os
import math
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

CONFIG_REL_PATH = os.path.join('config', 'robot.yaml')
ROBOT_LAUNCH_REL = '/launch/robot.launch.py'  # liegt in beiden Paketen identisch

def _load_cfg_strict() -> dict:
    pkg_share = get_package_share_directory('spraycoater_bringup')
    cfg_path = os.path.join(pkg_share, CONFIG_REL_PATH)
    if not os.path.exists(cfg_path):
        raise RuntimeError(f"[bringup] Konfigurationsdatei fehlt: {cfg_path}")
    try:
        with open(cfg_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except Exception as e:
        raise RuntimeError(f"[bringup] YAML kann nicht geladen werden: {cfg_path} ({e})")
    if not isinstance(data, dict):
        raise RuntimeError(f"[bringup] Ungültige YAML-Struktur in {cfg_path}")
    return data

def _resolve_moveit_pkg(cfg: dict) -> str:
    sel = cfg.get('selected')
    robots = cfg.get('robots')
    if not sel:
        raise RuntimeError("[bringup] 'selected' ist in robot.yaml nicht gesetzt.")
    if not isinstance(robots, dict) or sel not in robots:
        opts = ', '.join(robots.keys()) if isinstance(robots, dict) else '—'
        raise RuntimeError(f"[bringup] Unbekannter Roboter '{sel}'. Erlaubt: {opts}")
    moveit_pkg = robots[sel].get('moveit_pkg')
    if not moveit_pkg:
        raise RuntimeError(f"[bringup] Für '{sel}' fehlt 'moveit_pkg' in robot.yaml.")
    return moveit_pkg

def _read_mount_tf_strict(cfg: dict) -> dict:
    tf_section = cfg.get('tf')
    if not isinstance(tf_section, dict):
        raise RuntimeError("[bringup] Abschnitt 'tf' fehlt oder ist ungültig.")
    tf_cfg = tf_section.get('world_to_robot_mount')
    if not isinstance(tf_cfg, dict):
        raise RuntimeError("[bringup] 'tf.world_to_robot_mount' fehlt oder ist ungültig.")

    parent = tf_cfg.get('parent', 'world')
    child  = tf_cfg.get('child', 'robot_mount')
    xyz    = tf_cfg.get('xyz')
    rpydeg = tf_cfg.get('rpy_deg')

    if not (isinstance(xyz, (list, tuple)) and len(xyz) == 3):
        raise RuntimeError("[bringup] tf.world_to_robot_mount.xyz muss eine 3er-Liste sein.")
    if not (isinstance(rpydeg, (list, tuple)) and len(rpydeg) == 3):
        raise RuntimeError("[bringup] tf.world_to_robot_mount.rpy_deg muss eine 3er-Liste sein.")

    x, y, z = [str(float(v)) for v in xyz]
    # Grad -> Radiant hier umrechnen, damit robot.launch.py nichts rechnen muss
    rr, pr, yr = [str(math.radians(float(v))) for v in rpydeg]

    return {
        'mount_parent': parent,
        'mount_child':  child,
        'mount_x': x, 'mount_y': y, 'mount_z': z,
        'mount_roll_rad': rr, 'mount_pitch_rad': pr, 'mount_yaw_rad': yr,
    }

def _setup(_context):
    cfg = _load_cfg_strict()
    moveit_pkg = _resolve_moveit_pkg(cfg)
    mount_args = _read_mount_tf_strict(cfg)

    # Konsistente Logs / DDS-SHM Off (Docker)
    env = [
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('FASTDDS_SHM_DEFAULT', '0'),
    ]

    # 1) Robot (URDF + Controller + move_group etc.)
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare(moveit_pkg), ROBOT_LAUNCH_REL]),
        launch_arguments=mount_args.items()
    )

    # 2) Servo – vorbereitet; starte später, wenn du willst
    # nodes_pkg = FindPackageShare('spraycoater_nodes_cpp')
    # servo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([nodes_pkg, '/launch/servo.launch.py']),
    #     launch_arguments={
    #         'servo_mode': 'cartesian',
    #         'servo_frame': 'tcp',
    #     }.items()
    # )
    # delayed_servo = TimerAction(period=3.0, actions=[servo])

    return env + [robot]  # + [delayed_servo]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_setup),
    ])
