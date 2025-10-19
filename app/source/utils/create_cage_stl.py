#!/usr/bin/env python3
# make_cage_closed_1m.py
# erzeugt: resource/stl/environment/cage_closed_1m.stl
import os
import numpy as np
import trimesh

# ---- Parameter (mm) ----
L = 500.0       # Kantenlänge außen
T = 30.0         # Balkenquerschnitt (quadratisch T x T)
Z0 = 0.0         # Boden bei z=0 (Ursprung in der Bodenmitte)
Z1 = Z0 + L

# Hilfen
def edge_beam(p0, p1, thickness=T):
    """
    Erzeugt einen quaderförmigen Balken (box) entlang der Kante p0->p1.
    p0, p1: 3D-Punkte (mm)
    thickness: Balkenquerschnitt (mm)
    """
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    center = (p0 + p1) / 2.0
    vec = p1 - p0
    length = np.linalg.norm(vec)
    if length < 1e-6:
        raise ValueError("Edge length ~ 0")
    # Lokale Box: Achsen-aligned entlang X -> (length, T, T)
    box = trimesh.creation.box(extents=[length, thickness, thickness])
    # Ausrichtung: rotiere X-Achse auf vec
    x_axis = np.array([1.0, 0.0, 0.0])
    v = vec / length
    # Rotationsmatrix (align x_axis -> v)
    c = np.dot(x_axis, v)
    if c > 0.999999:
        R = np.eye(3)
    elif c < -0.999999:
        # 180° um beliebige Achse senkrecht zu x_axis (z.B. y)
        R = trimesh.transformations.rotation_matrix(np.pi, [0,1,0])[:3,:3]
    else:
        axis = np.cross(x_axis, v)
        angle = np.arccos(c)
        R = trimesh.transformations.rotation_matrix(angle, axis)[:3,:3]
    Tm = np.eye(4)
    Tm[:3,:3] = R
    Tm[:3, 3] = center
    box.apply_transform(Tm)
    return box

# ---- Kanten-Endpunkte (Ursprung: Bodenmittelpunkt) ----
# X, Y von -L/2 .. +L/2, Z von 0 .. L
h = L * 0.5
xs = (-h, h)
ys = (-h, h)
z0, z1 = Z0, Z1

edges = []

# 4 Boden-Kanten (Z=z0)
edges += [((-h, -h, z0), ( h, -h, z0))]
edges += [(( h, -h, z0), ( h,  h, z0))]
edges += [(( h,  h, z0), (-h,  h, z0))]
edges += [((-h,  h, z0), (-h, -h, z0))]

# 4 Top-Kanten (Z=z1)
edges += [((-h, -h, z1), ( h, -h, z1))]
edges += [(( h, -h, z1), ( h,  h, z1))]
edges += [(( h,  h, z1), (-h,  h, z1))]
edges += [((-h,  h, z1), (-h, -h, z1))]

# 4 Vertikale Kanten
edges += [((-h, -h, z0), (-h, -h, z1))]
edges += [(( h, -h, z0), ( h, -h, z1))]
edges += [(( h,  h, z0), ( h,  h, z1))]
edges += [((-h,  h, z0), (-h,  h, z1))]

# ---- Beams zusammensetzen ----
beams = [edge_beam(a, b, T) for (a, b) in edges]
cage = trimesh.util.concatenate(beams)

# ---- Export ----
out_path = "resource/stl/environment/cage_closed_1m.stl"
os.makedirs(os.path.dirname(out_path), exist_ok=True)
cage.export(out_path)
print(f"OK: {out_path} geschrieben. Ursprung ist Bodenmitte (0,0,0).")
