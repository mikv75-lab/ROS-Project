# -*- coding: utf-8 -*-
from __future__ import annotations
import numpy as np
import pyvista as pv

# keine Imports aus trajectory_builder hier -> vermeidet Zirkularimport
class TrajPose:
    def __init__(self, p, q, t):
        self.p = p
        self.q = q
        self.t = t

class Trajectory:
    def __init__(self, poses, frame_id="scene", meta=None):
        self.poses = poses or []
        self.frame_id = frame_id
        self.meta = dict(meta or {})

    def as_arrays(self):
        if not self.poses:
            return (np.zeros((0,3)), np.zeros((0,4)), np.zeros((0,)))
        P = np.vstack([tp.p for tp in self.poses])
        Q = np.vstack([tp.q for tp in self.poses])
        Tt = np.array([tp.t for tp in self.poses], dtype=float)
        return P, Q, Tt


class PreviewEngine:
    """
    Zeichnet die **echte TCP-Trajektorie**:
      - Eingabe: Trajectory (Positionen in m, Quaternions x,y,z,w)
      - Anzeige: Punkte/Linie in **mm** (nur visuell skaliert)
      - 'Normale' = Sprühachse (-Z des TCP) als Pfeilglyph
      - 'Raycasts' = Linien vom TCP zur Oberfläche über stand_off_mm

    Neu:
      - Hält Actor-Handles fest, damit Normals/Raycasts ohne Recompute toggelbar sind.
    """

    def __init__(self, plotter: pv.Plotter):
        self.plotter = plotter
        # Actor-Sammlungen für Overlay mehrerer Seiten
        self._actors_lines:   list = []
        self._actors_points:  list = []
        self._actors_normals: list = []
        self._actors_rays:    list = []

    # --------- Cameras ---------
    def view_iso(self):   self.plotter.view_isometric(); self.plotter.reset_camera()
    def view_top(self):   self.plotter.view_xy();        self.plotter.reset_camera()
    def view_front(self): self.plotter.view_yz();        self.plotter.reset_camera()
    def view_right(self): self.plotter.view_xz();        self.plotter.reset_camera()
    # Optional: back/left könnten später echte Gegensichten bekommen
    def view_back(self):  self.plotter.view_yz();        self.plotter.reset_camera()
    def view_left(self):  self.plotter.view_xz();        self.plotter.reset_camera()

    # --------- Render ---------
    def clear(self):
        self.plotter.clear()
        self._actors_lines.clear()
        self._actors_points.clear()
        self._actors_normals.clear()
        self._actors_rays.clear()

    def render_trajectory(self, traj: Trajectory, *, show_normals: bool, show_rays: bool,
                          append: bool = False) -> None:
        """
        Zeichnet eine Trajectory. Positionsdaten sind in **m**; Anzeige in **mm**.
        Normals/Rays werden immer erzeugt, Sichtbarkeit wird nach Flags gesetzt,
        damit spätere Toggles ohne Recompute funktionieren.
        """
        P_m, Q, _T = traj.as_arrays()
        if len(P_m) == 0:
            if not append:
                self.clear()
            return

        # umrechnen m->mm für die Anzeige
        P = P_m * 1000.0

        if not append:
            self.clear()

        # Polyline
        line = pv.PolyData(P)
        n = len(P)
        if n >= 2:
            line.lines = np.hstack(([n], np.arange(n))).astype(np.int64)
            act_line = self.plotter.add_mesh(line, line_width=2)
            self._actors_lines.append(act_line)
        act_pts = self.plotter.add_points(P, point_size=6, render_points_as_spheres=True)
        self._actors_points.append(act_pts)

        # Sprayachsen (-Z) aus Quaternion, Länge heuristisch
        spray_dirs = self._spray_axes_from_quat(Q)  # (N,3) Einheitsvektoren

        # Normals (Glyphs)
        arrows = pv.PolyData(P)
        arrows["vectors"] = spray_dirs
        glyphs = arrows.glyph(orient="vectors", scale=False, factor=self._axis_len_mm(traj))
        act_normals = self.plotter.add_mesh(glyphs)
        act_normals.SetVisibility(bool(show_normals))
        self._actors_normals.append(act_normals)

        # Raycasts
        rays_pd = self._raycasts_polydata(P, spray_dirs, self._stand_off_mm(traj))
        act_rays = self.plotter.add_mesh(rays_pd, line_width=1)
        act_rays.SetVisibility(bool(show_rays))
        self._actors_rays.append(act_rays)

        self.view_iso()

    # --------- Visibility Toggles (ohne Recompute) ---------
    def set_normals_visible(self, visible: bool):
        for act in self._actors_normals:
            try: act.SetVisibility(bool(visible))
            except Exception: pass
        self.plotter.render()

    def set_rays_visible(self, visible: bool):
        for act in self._actors_rays:
            try: act.SetVisibility(bool(visible))
            except Exception: pass
        self.plotter.render()

    # --------- Helpers ---------
    @staticmethod
    def _spray_axes_from_quat(Q: np.ndarray) -> np.ndarray:
        """
        Liefert -Z in Weltkoordinaten für jede Orientierung.
        """
        if len(Q) == 0:
            return np.zeros((0,3))
        v = np.array([0.0, 0.0, -1.0])
        dirs = []
        for q in Q:
            x, y, z, w = q
            R = np.array([
                [1-2*(y*y+z*z), 2*(x*y - z*w),   2*(x*z + y*w)],
                [2*(x*y + z*w),   1-2*(x*x+z*z), 2*(y*z - x*w)],
                [2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x*x+y*y)],
            ], dtype=float)
            dirs.append(R @ v)
        D = np.vstack(dirs)
        D /= (np.linalg.norm(D, axis=1, keepdims=True) + 1e-12)
        return D

    @staticmethod
    def _stand_off_mm(traj: Trajectory) -> float:
        try:
            return float(traj.meta.get("stand_off_mm", 10.0))
        except Exception:
            return 10.0

    @staticmethod
    def _axis_len_mm(traj: Trajectory) -> float:
        so = PreviewEngine._stand_off_mm(traj)
        return max(5.0, 0.6 * so)

    @staticmethod
    def _raycasts_polydata(P: np.ndarray, dirs: np.ndarray, stand_off_mm: float) -> pv.PolyData:
        N = len(P)
        if N == 0:
            return pv.PolyData()
        all_pts = np.empty((2*N, 3), dtype=float)
        all_pts[0::2] = P
        all_pts[1::2] = P + dirs * float(stand_off_mm)  # entlang Sprayrichtung zur Oberfläche
        lines = np.empty((N, 3), dtype=np.int64)
        lines[:,0] = 2
        lines[:,1] = np.arange(0, 2*N, 2, dtype=np.int64)
        lines[:,2] = np.arange(1, 2*N, 2, dtype=np.int64)
        poly = pv.PolyData(all_pts)
        poly.lines = lines.reshape(-1)
        return poly
