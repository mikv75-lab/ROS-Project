# spraycoater_bringup/launch/bringup.launch.py
#!/usr/bin/env python3
import os, math, yaml
from launch import LaunchDescription
from launch.actions import OpaqueFunction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

CFG_REL = os.path.join('config', 'robot.yaml')
ROBOT_REL = '/launch/robot.launch.py'

def _load_cfg() -> dict:
    share = get_package_share_directory('spraycoater_bringup')
    path  = os.path.join(share, CFG_REL)
    if not os.path.exists(path):
        raise RuntimeError(f"[bringup] Konfigurationsdatei fehlt: {path}")
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return data

def _resolve_moveit_pkg(cfg: dict) -> str:
    sel = cfg.get('selected')
    robots = cfg.get('robots', {})
    if not sel or sel not in robots:
        raise RuntimeError(f"[bringup] 'selected' fehlt/ungültig. Erlaubt: {', '.join(robots.keys())}")
    pkg = robots[sel].get('moveit_pkg')
    if not pkg:
        raise RuntimeError(f"[bringup] Für '{sel}' fehlt 'moveit_pkg'.")
    return pkg

def _mount_args(cfg: dict) -> dict:
    tf = cfg.get('tf', {}).get('world_to_robot_mount', {})
    parent = tf.get('parent', 'world')
    child  = tf.get('child',  'robot_mount')
    xyz    = tf.get('xyz',    [0,0,0])
    rpydeg = tf.get('rpy_deg',[0,0,0])

    x,y,z = [str(float(v)) for v in xyz]
    rr,pr,yr = [str(math.radians(float(v))) for v in rpydeg]
    return {
        'mount_parent': parent,
        'mount_child':  child,
        'mount_x': x, 'mount_y': y, 'mount_z': z,
        'mount_roll_rad': rr, 'mount_pitch_rad': pr, 'mount_yaw_rad': yr,
    }

def _setup(_ctx):
    cfg = _load_cfg()
    moveit_pkg = _resolve_moveit_pkg(cfg)
    mount = _mount_args(cfg)

    env = [
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('FASTDDS_SHM_DEFAULT', '0'),
    ]

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare(moveit_pkg), ROBOT_REL]),
        launch_arguments=mount.items()
    )

    ld = LaunchDescription()
    for e in env:
        ld.add_action(e)
    ld.add_action(robot)
    return [ld]

def generate_launch_description():
    # OpaqueFunction gibt eine Liste von Actions zurück; NICHT add_action mit *
    return LaunchDescription([OpaqueFunction(function=_setup)])
