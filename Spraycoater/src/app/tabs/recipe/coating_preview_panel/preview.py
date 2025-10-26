# -*- coding: utf-8 -*-
# Spraycoater/src/app/tabs/recipe/coating_preview_panel/preview.py
from __future__ import annotations
import numpy as np
import pyvista as pv


class TrajPose:
    def __init__(self, p, q, t):
        self.p = p  # R^3 (m)
        self.q = q  # quat (x,y,z,w)
        self.t = t  # time (s)


class Trajectory:
    def __init__(self, poses, frame_id: str = "scene", meta=None):
        self.poses = poses or []
        self.frame_id = frame_id
        self.meta = dict(meta or {})

    def as_arrays(self):
        if not self.poses:
            return (np.zeros((0, 3)), np.zeros((0, 4)), np.zeros((0,)))
        P = np.vstack([tp.p for tp in self.poses])       # (N,3) in m
        Q = np.vstack([tp.q for tp in self.poses])       # (N,4) x,y,z,w
        Tt = np.array([tp.t for tp in self.poses], float)
        return P, Q, Tt


class PreviewEngine:
    """
    Zeichnet die **TCP-Trajektorie** in PyVista (Anzeige in mm).
    - Scene-Actors (Mount/Substrate) separat von Traj/Overlay-Actors.
    - Normals/Raycasts per Visibility-Toggle ohne Recompute.
    - Vermeidet plotter.clear() → stabil gegen PyVista-Renderer-Fehler.
    """

    def __init__(self, plotter: pv.Plotter):
        self.plotter = plotter
        # Actor-Gruppen
        self._actors_scene: list = []     # Mount/Substrate/etc.
        self._actors_lines: list = []     # Traj-Linien
        self._actors_points: list = []    # Traj-Punkte
        self._actors_normals: list = []   # Pfeil-Glyphs
        self._actors_rays: list = []      # Raycast-Linien

    # ---------- Cameras ----------
    def view_iso(self):   self.plotter.view_isometric(); self.plotter.reset_camera()
    def view_top(self):   self.plotter.view_xy();        self.plotter.reset_camera()
    def view_front(self): self.plotter.view_yz();        self.plotter.reset_camera()
    def view_right(self): self.plotter.view_xz();        self.plotter.reset_camera()
    def view_back(self):  self.plotter.view_yz();        self.plotter.reset_camera()
    def view_left(self):  self.plotter.view_xz();        self.plotter.reset_camera()

    # ---------- Scene-Meshes ----------
    def add_mesh(self, mesh: pv.PolyData | pv.UnstructuredGrid, **kwargs):
        """
        Fügt ein statisches Mesh (Mount/Substrate/Tool) hinzu und tracked den Actor.
        Rückgabe: Actor (von PyVista).
        """
        act = self.plotter.add_mesh(mesh, **kwargs)
        actor = act[0] if isinstance(act, tuple) else act
        self._actors_scene.append(actor)
        return actor

    def clear_scene(self):
        """Entfernt NUR die statischen Scene-Meshes."""
        self._remove_actors(self._actors_scene)
        self._actors_scene.clear()
        self.plotter.render()

    # ---------- Trajectory ----------
    def clear_trajectory(self):
        """Entfernt Trajectory + Overlays."""
        self._remove_actors(self._actors_lines)
        self._remove_actors(self._actors_points)
        self._remove_actors(self._actors_normals)
        self._remove_actors(self._actors_rays)
        self._actors_lines.clear()
        self._actors_points.clear()
        self._actors_normals.clear()
        self._actors_rays.clear()
        self.plotter.render()

    def clear(self):
        """Komplett zurücksetzen (Szene + Traj). Kein plotter.clear()!"""
        self.clear_trajectory()
        self.clear_scene()

    def render_trajectory(self, traj: Trajectory, *, show_normals: bool, show_rays: bool,
                          append: bool = False) -> None:
        """
        Zeichnet eine Trajectory. Positionsdaten sind in **m**; Anzeige in **mm**.
        Normals/Rays werden immer erzeugt; Sichtbarkeit wird per Toggle gesetzt.
        """
        P_m, Q, _T = traj.as_arrays()
        if len(P_m) == 0:
            if not append:
                self.clear_trajectory()
            return

        # Anzeige in mm
        P = P_m * 1000.0
        if not append:
            self.clear_trajectory()

        # Polyline + Punkte
        line_pd = pv.PolyData(P)
        n = len(P)
        if n >= 2:
            line_pd.lines = np.hstack(([n], np.arange(n))).astype(np.int64)
            act_line = self.plotter.add_mesh(line_pd, line_width=2)
            self._actors_lines.append(act_line)
        act_pts = self.plotter.add_points(P, point_size=6, render_points_as_spheres=True)
        self._actors_points.append(act_pts)

        # Sprayachsen (-Z) aus Quaternion
        spray_dirs = self._spray_axes_from_quat(Q)  # (N,3) Unit

        # Normals (Glyphs)
        arrows = pv.PolyData(P)
        arrows["vectors"] = spray_dirs
        glyphs = arrows.glyph(orient="vectors", scale=False, factor=self._axis_len_mm(traj))
        act_normals = self.plotter.add_mesh(glyphs)
        try:
            act_normals.SetVisibility(bool(show_normals))
        except Exception:
            pass
        self._actors_normals.append(act_normals)

        # Raycasts
        rays_pd = self._raycasts_polydata(P, spray_dirs, self._stand_off_mm(traj))
        act_rays = self.plotter.add_mesh(rays_pd, line_width=1)
        try:
            act_rays.SetVisibility(bool(show_rays))
        except Exception:
            pass
        self._actors_rays.append(act_rays)

        self.view_iso()

    # ---------- Visibility-Toggles ----------
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

    # ---------- Intern ----------
    def _remove_actors(self, actors: list):
        for act in actors:
            try:
                self.plotter.remove_actor(act)
            except Exception:
                pass

    @staticmethod
    def _spray_axes_from_quat(Q: np.ndarray) -> np.ndarray:
        """Liefert -Z in Weltkoordinaten für jede Orientierung (x,y,z,w)."""
        if len(Q) == 0:
            return np.zeros((0, 3))
        v = np.array([0.0, 0.0, -1.0])
        dirs = []
        for q in Q:
            x, y, z, w = q
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
