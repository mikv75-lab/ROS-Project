# -*- coding: utf-8 -*-
from __future__ import annotations
import numpy as np
import pyvista as pv

from .trajectory_builder import Trajectory, TrajPose


class PreviewEngine:
    """
    Zeichnet die **echte TCP-Trajektorie**:
      - Eingabe: Trajectory (Positionen in m, Quaternions x,y,z,w)
      - Anzeige: Punkte/Linie in **mm** (nur visuell skaliert)
      - 'Normale' = Sprühachse (-Z des TCP) visualisiert als Pfeilglyph
      - 'Raycasts' = Linien vom TCP zur Oberfläche über stand_off_mm

    Plus (optional):
      - render_surface_path_mm(): Anzeige eines Oberflächenpfads in mm (z.B. aus PathBuilder),
        inkl. Normalen/Raycasts (TCP -> Oberfläche), ohne TCP-Orientierungen.
    """

    def __init__(self, plotter: pv.Plotter):
        self.plotter = plotter

    # --------- Cameras ---------
    def view_iso(self):   self.plotter.view_isometric(); self.plotter.reset_camera()
    def view_top(self):   self.plotter.view_xy();        self.plotter.reset_camera()
    def view_front(self): self.plotter.view_yz();        self.plotter.reset_camera()
    def view_right(self): self.plotter.view_xz();        self.plotter.reset_camera()

    # --------- Render: TCP-Trajektorie (m -> mm) ---------
    def render_trajectory(self, traj: Trajectory, *, show_normals: bool, show_rays: bool,
                          append: bool = False) -> None:
        """
        Zeichnet eine Trajectory. Positionsdaten sind in **m**; Anzeige in **mm**.
        """
        P_m, Q, _T = traj.as_arrays()
        if len(P_m) == 0:
            if not append:
                self.plotter.clear()
            return

        # umrechnen m->mm für die Anzeige
        P = P_m * 1000.0

        if not append:
            self.plotter.clear()

        # Polyline + Punkte
        n = len(P)
        if n >= 2:
            line = pv.PolyData(P)
            line.lines = np.hstack(([n], np.arange(n, dtype=np.int64))).astype(np.int64)
            self.plotter.add_mesh(line, line_width=2)
        self.plotter.add_points(P, point_size=6, render_points_as_spheres=True)

        # Sprayachsen (-Z) aus Quaternion
        spray_dirs = self._spray_axes_from_quat(Q)  # (N,3) Einheitsvektoren
        if show_normals and len(spray_dirs):
            arrows = pv.PolyData(P)
            arrows["vectors"] = spray_dirs
            glyphs = arrows.glyph(orient="vectors", scale=False, factor=self._axis_len_mm(traj))
            self.plotter.add_mesh(glyphs)

        # Raycasts: TCP -> Oberfläche entlang Sprayrichtung (stand_off)
        if show_rays and len(spray_dirs):
            rays = self._raycasts_polydata(P, spray_dirs, self._stand_off_mm(traj))
            self.plotter.add_mesh(rays, line_width=1)

        self.view_iso()

    # --------- Render: Oberflächenpfad (mm) optional ---------
    def render_surface_path_mm(
        self,
        points_mm: np.ndarray,
        *,
        normals_mm: np.ndarray | None = None,
        stand_off_mm: float | None = None,
        show_normals: bool = True,
        show_rays: bool = True,
        append: bool = False,
    ) -> None:
        """
        Visualisiert einen Pfad in **mm** (z. B. aus PathBuilder).
        - normals_mm: Oberflächennormalen (Nx3, Einheitsvektoren); wenn None, keine Normals/Rays.
        - stand_off_mm: Falls gesetzt und normals_mm vorhanden, werden Raycasts TCP->Surface gezeichnet.
                        TCP = surface - normal * stand_off_mm
        """
        P = np.asarray(points_mm, dtype=float).reshape(-1, 3)
        if P.size == 0:
            if not append:
                self.plotter.clear()
            return
        if not append:
            self.plotter.clear()

        n = len(P)
        if n >= 2:
            line = pv.PolyData(P)
            line.lines = np.hstack(([n], np.arange(n, dtype=np.int64))).astype(np.int64)
            self.plotter.add_mesh(line, line_width=2)
        self.plotter.add_points(P, point_size=6, render_points_as_spheres=True)

        if normals_mm is not None:
            N = np.asarray(normals_mm, dtype=float).reshape(-1, 3)
            if len(N) == len(P):
                # Normals als Pfeile (Skalierung wie bei TCP: ~0.6 * stand_off, fallback 5mm)
                if show_normals:
                    arrows = pv.PolyData(P)
                    arrows["vectors"] = N / (np.linalg.norm(N, axis=1, keepdims=True) + 1e-12)
                    glyphs = arrows.glyph(orient="vectors", scale=False, factor=self._axis_len_from_value_mm(stand_off_mm))
                    self.plotter.add_mesh(glyphs)
                # Raycasts (TCP -> Surface): TCP = P - N * stand_off
                if show_rays and stand_off_mm is not None:
                    dirs = N / (np.linalg.norm(N, axis=1, keepdims=True) + 1e-12)
                    tcp = P - dirs * float(stand_off_mm)
                    rays = self._segments_polydata(tcp, P)
                    self.plotter.add_mesh(rays, line_width=1)

        self.view_iso()

    # --------- Helpers ---------
    @staticmethod
    def _spray_axes_from_quat(Q: np.ndarray) -> np.ndarray:
        """
        Liefert -Z in Weltkoordinaten für jede Orientierung.
        Quaternionen werden vorher normalisiert.
        """
        if len(Q) == 0:
            return np.zeros((0, 3))
        v = np.array([0.0, 0.0, -1.0])
        dirs = []
        for q in Q:
            x, y, z, w = q
            # Normieren (Robustheit)
            n = np.linalg.norm([x, y, z, w])
            if n == 0.0:
                dirs.append(v)
                continue
            x, y, z, w = x / n, y / n, z / n, w / n
            R = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
                [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
            ], dtype=float)
            dirs.append(R @ v)
        D = np.vstack(dirs)
        D /= (np.linalg.norm(D, axis=1, keepdims=True) + 1e-12)
        return D

    @staticmethod
    def _stand_off_mm(traj: Trajectory) -> float:
        try:
            return float((traj.meta or {}).get("stand_off_mm", 10.0))
        except Exception:
            return 10.0

    @staticmethod
    def _axis_len_mm(traj: Trajectory) -> float:
        # Pfeillänge ~ 60% des Standoff, min 5mm
        so = PreviewEngine._stand_off_mm(traj)
        return max(5.0, 0.6 * so)

    @staticmethod
    def _axis_len_from_value_mm(stand_off_mm: float | None) -> float:
        if stand_off_mm is None:
            return 5.0
        return max(5.0, 0.6 * float(stand_off_mm))

    @staticmethod
    def _raycasts_polydata(P: np.ndarray, dirs: np.ndarray, stand_off_mm: float) -> pv.PolyData:
        """
        Raycasts von TCP (P) in Sprayrichtung (dirs) mit Länge stand_off_mm.
        """
        N = len(P)
        if N == 0:
            return pv.PolyData()
        all_pts = np.empty((2 * N, 3), dtype=float)
        all_pts[0::2] = P
        all_pts[1::2] = P + dirs * float(stand_off_mm)
        lines = np.empty((N, 3), dtype=np.int64)
        lines[:, 0] = 2
        lines[:, 1] = np.arange(0, 2 * N, 2, dtype=np.int64)
        lines[:, 2] = np.arange(1, 2 * N, 2, dtype=np.int64)
        poly = pv.PolyData(all_pts)
        poly.lines = lines.reshape(-1)
        return poly

    @staticmethod
    def _segments_polydata(A: np.ndarray, B: np.ndarray) -> pv.PolyData:
        """
        Erzeugt Liniensegmente von A[i] nach B[i].
        """
        assert len(A) == len(B)
        N = len(A)
        pts = np.empty((2 * N, 3), dtype=float)
        pts[0::2] = A
        pts[1::2] = B
        lines = np.empty((N, 3), dtype=np.int64)
        lines[:, 0] = 2
        lines[:, 1] = np.arange(0, 2 * N, 2, dtype=np.int64)
        lines[:, 2] = np.arange(1, 2 * N, 2, dtype=np.int64)
        pd = pv.PolyData(pts)
        pd.lines = lines.reshape(-1)
        return pd
