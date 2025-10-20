#!/usr/bin/env python3
import os
import numpy as np
import trimesh

# ==============================
# ✅ KONFIGURATION
# ==============================
L = 500.0            # Außenmaß (mm)
T = 30.0             # Balkendicke (mm)
WALL_THICKNESS = 5.0 # Wanddicke (mm)

# ✅ Wände aktivieren/deaktivieren
ADD_FLOOR = True
ADD_CEILING = False
ADD_WALL_FRONT = False   # +Y
ADD_WALL_BACK = True    # -Y
ADD_WALL_LEFT = False    # -X
ADD_WALL_RIGHT = False   # +X

# ==============================
# ✅ Hilfsfunktionen
# ==============================
def edge_beam(p0, p1, thickness=T):
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    center = (p0 + p1) / 2.0
    vec = p1 - p0
    length = np.linalg.norm(vec)
    x_axis = np.array([1.0, 0.0, 0.0])
    v = vec / length
    c = np.dot(x_axis, v)
    if c > 0.999999:
        R = np.eye(3)
    elif c < -0.999999:
        R = trimesh.transformations.rotation_matrix(np.pi, [0, 1, 0])[:3, :3]
    else:
        axis = np.cross(x_axis, v)
        angle = np.arccos(c)
        R = trimesh.transformations.rotation_matrix(angle, axis)[:3, :3]
    Tm = np.eye(4)
    Tm[:3, :3] = R
    Tm[:3, 3] = center
    box = trimesh.creation.box(extents=[length, thickness, thickness])
    box.apply_transform(Tm)
    return box

# ==============================
# ✅ Erstellung Cage
# ==============================
h = L * 0.5
Z0 = 0.0
Z1 = L

edges = [
    # Untere Kanten
    ((-h, -h, Z0), ( h, -h, Z0)),
    (( h, -h, Z0), ( h,  h, Z0)),
    (( h,  h, Z0), (-h,  h, Z0)),
    ((-h,  h, Z0), (-h, -h, Z0)),
    # Obere Kanten
    ((-h, -h, Z1), ( h, -h, Z1)),
    (( h, -h, Z1), ( h,  h, Z1)),
    (( h,  h, Z1), (-h,  h, Z1)),
    ((-h,  h, Z1), (-h, -h, Z1)),
    # Vertikal
    ((-h, -h, Z0), (-h, -h, Z1)),
    (( h, -h, Z0), ( h, -h, Z1)),
    (( h,  h, Z0), ( h,  h, Z1)),
    ((-h,  h, Z0), (-h,  h, Z1)),
]

parts = [edge_beam(a, b) for a, b in edges]

# ==============================
# ✅ Wände hinzufügen (optional)
# ==============================
# Boden
if ADD_FLOOR:
    floor = trimesh.creation.box([L, L, WALL_THICKNESS])
    floor.apply_translation([0, 0, Z0 - WALL_THICKNESS / 2])
    parts.append(floor)

# Decke
if ADD_CEILING:
    ceil = trimesh.creation.box([L, L, WALL_THICKNESS])
    ceil.apply_translation([0, 0, Z1 + WALL_THICKNESS / 2])
    parts.append(ceil)

# Rückwand Y-
if ADD_WALL_BACK:
    back = trimesh.creation.box([L, WALL_THICKNESS, L])
    back.apply_translation([0, -h - WALL_THICKNESS / 2, Z0 + L / 2])
    parts.append(back)

# Frontwand Y+
if ADD_WALL_FRONT:
    front = trimesh.creation.box([L, WALL_THICKNESS, L])
    front.apply_translation([0, h + WALL_THICKNESS / 2, Z0 + L / 2])
    parts.append(front)

# Linke Wand X-
if ADD_WALL_LEFT:
    left = trimesh.creation.box([WALL_THICKNESS, L, L])
    left.apply_translation([-h - WALL_THICKNESS / 2, 0, Z0 + L / 2])
    parts.append(left)

# Rechte Wand X+
if ADD_WALL_RIGHT:
    right = trimesh.creation.box([WALL_THICKNESS, L, L])
    right.apply_translation([h + WALL_THICKNESS / 2, 0, Z0 + L / 2])
    parts.append(right)

# ==============================
# ✅ Export
# ==============================
cage = trimesh.util.concatenate(parts)
out_path = "resource/stl/environment/cage_closed_1m.stl"
os.makedirs(os.path.dirname(out_path), exist_ok=True)
cage.export(out_path)
print(f"✅ Fertig: {out_path} geschrieben.")
