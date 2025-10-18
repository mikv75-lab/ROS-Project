# mecademic_bringup/utils.py
import os, math
from ament_index_python.packages import get_package_share_directory

_PKG = 'mecademic_bringup'
_PKG_SHARE = get_package_share_directory(_PKG)

def resolve_mesh_path(path: str) -> str:
    if not path:
        return ""
    return path if os.path.isabs(path) else os.path.join(_PKG_SHARE, path)

def rpy_deg_to_quat(r, p, y):
    rx, ry, rz = map(math.radians, (r, p, y))
    cr, sr = math.cos(rx/2), math.sin(rx/2)
    cp, sp = math.cos(ry/2), math.sin(ry/2)
    cy, sy = math.cos(rz/2), math.sin(rz/2)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - cr*sp*cy
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    return (qx/n, qy/n, qz/n, qw/n)
