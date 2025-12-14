# mecademic_bringup/utils.py
# -*- coding: utf-8 -*-
import os
import math
from typing import Tuple
from ament_index_python.packages import get_package_share_directory

# Standard-Paket, relativ zu dessen Share-Verzeichnis werden relative Pfade aufgelöst
_PKG = "spraycoater_bringup"
_PKG_SHARE = get_package_share_directory(_PKG)


def _get_pkg_share(pkg: str) -> str:
    """Share-Verzeichnis eines Pakets holen."""
    return get_package_share_directory(pkg)


def resolve_mesh_path(path: str) -> str:
    """
    Auflösung von Mesh-Pfaden:
      - ""        -> ""
      - package://<pkg>/<rel> -> <share(pkg)>/<rel>
      - /abs/…    -> /abs/…
      - rel/…     -> <share(spraycoater_bringup)>/rel/…
    """
    if not path:
        return ""
    p = path.strip()
    if not p:
        return ""

    if p.startswith("package://"):
        # package://<pkg>/<rel>
        try:
            rest = p[len("package://") :]
            pkg, rel = rest.split("/", 1)
            return os.path.join(_get_pkg_share(pkg), rel)
        except ValueError:
            # Falls Format unerwartet ist: als Fallback unverändert zurückgeben
            return p

    if os.path.isabs(p):
        return p

    # relativ: gegenüber spraycoater_bringup/share
    return os.path.join(_PKG_SHARE, p)


# ------------------------------
# Quaternion-Helfer (ohne tf_* )
# ------------------------------
def rpy_rad_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    RPY (rad) -> Quaternion (x, y, z, w)

    Konvention:
      - Tait-Bryan ZYX (yaw → pitch → roll), äquivalent zu 'sxyz' in tf_transformations
        mit Aufruf quaternion_from_euler(roll, pitch, yaw, axes="sxyz")
      - Entspricht der üblichen ROS-Nutzung: erst Z (yaw), dann Y (pitch), dann X (roll)
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


def rpy_deg_to_quat(r: float, p: float, y: float) -> Tuple[float, float, float, float]:
    """RPY (deg) -> Quaternion (x, y, z, w)."""
    return rpy_rad_to_quat(math.radians(r), math.radians(p), math.radians(y))
