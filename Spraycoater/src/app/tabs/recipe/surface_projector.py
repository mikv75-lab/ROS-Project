# Spraycoater/src/app/tabs/recipe/surface_projector.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Tuple, Optional, Iterable
import numpy as np
import pyvista as pv


@dataclass
class ProjectionResult:
    points_mm: np.ndarray         # Nx3 (mm) – Trefferpunkte auf dem Mesh
    normals:   np.ndarray         # Nx3 – Einheitsnormalen am Treffer
    hit_mask:  np.ndarray         # N bool – True wenn ein Hit gefunden wurde
    meta:      Dict


class SurfaceProjector:
    """
    Projiziert 3D-Punkte (mm) per Raycast auf ein PolyData-Mesh (mm).
    - Für jeden Punkt p wird ein Strahl in einer festen Richtungs-Achse (Seite) geworfen.
    - Start/Ende des Strahls: p +/- dir * L, L groß genug (Standard 10 m = 10000 mm).
    - Normale kommt vom getroffenen Face; falls nötig, Orientierung gegen Ray-Richtung flippen.
    - Fallback bei keinem Hit: nächster Punkt + Punkt-Normale auf dem Mesh (falls vorhanden).
    """

    def __init__(self, *, ray_length_mm: float = 10000.0):
        self.ray_len = float(ray_length_mm)

    # ---------- Public ----------

    def project_on_side(
        self,
        points_mm: np.ndarray,
        mesh_mm: pv.PolyData,
        side: str,
    ) -> ProjectionResult:
        """Projiziert alle Punkte entlang der 'side'-Richtung (scene) auf 'mesh_mm'."""
        points_mm = np.asarray(points_mm, dtype=float).reshape(-1, 3)
        if len(points_mm) == 0:
            return ProjectionResult(
                points_mm=np.zeros((0, 3)),
                normals=np.zeros((0, 3)),
                hit_mask=np.zeros((0,), dtype=bool),
                meta={"empty": True},
            )

        dir_vec = self._dir_for_side(side)
        return self._ray_project(points_mm, mesh_mm, dir_vec)

    # ---------- Core ----------

    def _ray_project(
        self,
        P: np.ndarray,            # Nx3 mm
        mesh: pv.PolyData,
        dir_vec: np.ndarray,      # 3, Einheitsvektor
    ) -> ProjectionResult:
        # Sorg dafür, dass Normals verfügbar sind (für Fallbacks/Orientierung).
        mesh_n = self._with_normals(mesh)

        N = len(P)
        L = self.ray_len
        d = dir_vec / (np.linalg.norm(dir_vec) + 1e-12)

        hits = np.zeros((N, 3), dtype=float)
        nors = np.zeros((N, 3), dtype=float)
        hit_mask = np.zeros((N,), dtype=bool)

        # Face-Normals besorgen (erstellt von _with_normals)
        face_normals = getattr(mesh_n, "face_normals", None)
        # Wenn face_normals fehlen, berechnen wir sie dynamisch aus den Zellen
        use_face_normals = face_normals is not None and len(face_normals) == mesh_n.n_faces

        for i in range(N):
            p = P[i]
            start = p + d * L
            end   = p - d * L

            # PyVista ray_trace gibt alle Schnittpunkte + Face-IDs
            pts, ids = mesh_n.ray_trace(start, end)
            if len(pts) > 0:
                # ersten Treffer nehmen
                hit = np.asarray(pts[0], dtype=float).reshape(3)
                hits[i] = hit
                hit_mask[i] = True

                # Normale am getroffenen Face
                if use_face_normals and ids is not None and len(ids) > 0 and ids[0] >= 0:
                    n = np.asarray(face_normals[ids[0]], dtype=float).reshape(3)
                else:
                    # Fallback: nächste Mesh-Punktnormale
                    pid = mesh_n.find_closest_point(hit)
                    n = np.asarray(mesh_n.point_normals[pid], dtype=float).reshape(3)

                # Normalenrichtung so, dass sie der Strahlrichtung *entgegen* zeigt
                if np.dot(n, d) > 0.0:
                    n = -n
                n /= (np.linalg.norm(n) + 1e-12)
                nors[i] = n
            else:
                # Kein Hit: Fallback = nächster Punkt + Punkt-Normale
                pid = mesh_n.find_closest_point(p)
                hits[i] = np.asarray(mesh_n.points[pid], dtype=float).reshape(3)
                n = np.asarray(mesh_n.point_normals[pid], dtype=float).reshape(3)
                if np.dot(n, d) > 0.0:
                    n = -n
                n /= (np.linalg.norm(n) + 1e-12)
                nors[i] = n

        return ProjectionResult(
            points_mm=hits,
            normals=nors,
            hit_mask=hit_mask,
            meta={"dir": d.tolist(), "ray_length_mm": L},
        )

    # ---------- Helpers ----------

    @staticmethod
    def _dir_for_side(side: str) -> np.ndarray:
        """
        Scene-Achsen:
          x: +right / -left
          y: +back  / -front   (Frontseite wird aus +Y Richtung 'angegossen' -> Ray nach -Y)
          z: +up    / -down
        """
        s = (side or "").strip().lower()
        if s in ("top", "up"):
            return np.array([0.0, 0.0, -1.0])   # aus oben nach unten
        if s in ("bottom", "down"):
            return np.array([0.0, 0.0, 1.0])
        if s == "front":
            return np.array([0.0, -1.0, 0.0])
        if s == "back":
            return np.array([0.0, 1.0, 0.0])
        if s == "left":
            return np.array([-1.0, 0.0, 0.0])
        if s == "right":
            return np.array([1.0, 0.0, 0.0])
        # Default: top
        return np.array([0.0, 0.0, -1.0])

    @staticmethod
    def _with_normals(mesh: pv.PolyData) -> pv.PolyData:
        """
        Stellt sicher, dass Face- und Point-Normals vorhanden sind.
        Gibt ein Mesh zurück (kopiert), an dem beide Felder vorhanden sind.
        """
        m = mesh
        need_point = not hasattr(m, "point_normals") or m.point_normals is None or len(m.point_normals) != m.n_points
        need_face  = not hasattr(m, "face_normals")  or m.face_normals is None  or len(m.face_normals)  != m.n_faces
        if need_point or need_face:
            m = mesh.compute_normals(
                cell_normals=True,
                point_normals=True,
                auto_orient_normals=False,   # wir orientieren selbst zur Ray-Richtung
                splitting=False,
                inplace=False,
            )
        return m
