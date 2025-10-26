# Spraycoater/src/app/tabs/recipe/surface_projector.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Tuple, Literal
import numpy as np
import pyvista as pv

Side = Literal["top","bottom","front","back","left","right","helix"]

class SurfaceProjector:
    """
    Projiziert 3D-Pfadpunkte (mm) entlang einer Seitenrichtung via Raycast auf ein Mesh
    und liefert (Trefferpunkt_mm, Normal_mm) je Pfadpunkt.
    - Für 'helix' wird pro Punkt radial (in XY) nach innen gestrahlt.
    - Mesh muss in **mm** vorliegen; Ray-Länge groß (1e5 mm).
    """

    def __init__(self, ray_len_mm: float = 1e5):
        self.ray_len = float(ray_len_mm)

    @staticmethod
    def _dir_for_side(side: Side) -> np.ndarray:
        m = {
            "top":    np.array([0, 0, -1.0]),
            "bottom": np.array([0, 0, +1.0]),
            "front":  np.array([0, -1.0, 0]),
            "back":   np.array([0, +1.0, 0]),
            "left":   np.array([-1.0, 0, 0]),
            "right":  np.array([+1.0, 0, 0]),
        }
        if side == "helix":
            raise ValueError("helix handled separately")
        return m[side] / np.linalg.norm(m[side])

    def project(self, mesh: pv.PolyData, points_mm: np.ndarray, side: Side) -> Tuple[np.ndarray, np.ndarray]:
        if side == "helix":
            return self._project_helix(mesh, points_mm)

        d = self._dir_for_side(side)
        # Normals am Mesh sicherstellen (pro Zelle)
        mesh = mesh.copy()
        if "Normals" not in mesh.array_names:
            mesh.compute_normals(cell_normals=True, point_normals=False, inplace=True)

        hits = []
        norms = []

        for p in points_mm:
            p = np.asarray(p, dtype=float).reshape(3)
            start = p + (-d) * self.ray_len
            end   = p + (+d) * self.ray_len
            locs, cells = mesh.ray_trace(start, end, first_point=True)
            if len(locs) == 0 or len(cells) == 0:
                # kein Hit -> Punkt auslassen
                continue
            hits.append(locs[0])
            # Zellnormalen
            n = mesh.cell_data["Normals"][cells[0]]
            n = n / (np.linalg.norm(n) + 1e-12)
            # Orientierung: Normal soll zum Strahl zeigen (Gegenrichtung zu d)
            if np.dot(n, d) > 0:
                n = -n
            norms.append(n)

        if not hits:
            return np.zeros((0, 3)), np.zeros((0, 3))
        return np.asarray(hits, dtype=float), np.asarray(norms, dtype=float)

    def _project_helix(self, mesh: pv.PolyData, points_mm: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        mesh = mesh.copy()
        if "Normals" not in mesh.array_names:
            mesh.compute_normals(cell_normals=True, point_normals=False, inplace=True)

        hits = []
        norms = []
        for p in points_mm:
            p = np.asarray(p, dtype=float).reshape(3)
            # radialer Richtungsvektor (nur XY)
            r = p.copy(); r[2] = 0.0
            nr = np.linalg.norm(r)
            if nr < 1e-9:
                # auf Achse -> überspringen
                continue
            dir_in = -(r / nr)  # nach innen
            start = p + (-dir_in) * self.ray_len
            end   = p + (+dir_in) * self.ray_len
            locs, cells = mesh.ray_trace(start, end, first_point=True)
            if len(locs) == 0 or len(cells) == 0:
                continue
            hits.append(locs[0])
            n = mesh.cell_data["Normals"][cells[0]]
            n = n / (np.linalg.norm(n) + 1e-12)
            # Normal soll grob radial nach außen zeigen:
            if np.dot(n, r) < 0:
                n = -n
            norms.append(n)

        if not hits:
            return np.zeros((0, 3)), np.zeros((0, 3))
        return np.asarray(hits, dtype=float), np.asarray(norms, dtype=float)
