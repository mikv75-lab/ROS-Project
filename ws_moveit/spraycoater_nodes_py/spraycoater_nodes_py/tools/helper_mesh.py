# helper_mesh.py – kleines Hilfsmodul
import numpy as np
import trimesh
from shape_msgs.msg import Mesh, MeshTriangle
from ament_index_python.packages import get_package_share_path
from pathlib import Path
from geometry_msgs.msg import Point
def load_mesh_from_package(pkg_name: str, rel_path: str) -> Mesh:
    share = get_package_share_path(pkg_name)
    p = Path(share) / rel_path  # z.B. "meshes/spray_nozzle_01.stl"
    tm = trimesh.load_mesh(str(p), process=False)
    if not isinstance(tm, trimesh.Trimesh):
        raise RuntimeError("Mesh is not a Trimesh (got a scene). Export a single STL mesh.")

    # shape_msgs/Mesh
    msg = Mesh()
    # vertices
    for v in np.asarray(tm.vertices):
        g = Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))
        msg.vertices.append(g)
    # triangles
    for f in np.asarray(tm.faces):
        tri = MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])])
        msg.triangles.append(tri)
    return msg
