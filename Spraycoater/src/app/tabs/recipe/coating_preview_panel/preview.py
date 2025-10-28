# -*- coding: utf-8 -*-
# Spraycoater/src/app/tabs/recipe/coating_preview_panel/preview.py
from __future__ import annotations
import numpy as np
import pyvista as pv
import vtk


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
    Scene-Setup:
      - Durchgehender Boden als Grid (z=0) über ganze Ansicht
      - Kleine Orientierungstriade unten links
      - Bounding-Box 240×240×240 mm, Z von 0..240
      - Achsen-Grid/Skala mit Z-Start bei 0
    """

    def __init__(self, plotter: pv.Plotter, max_extent_mm: float = 240.0):
        self.plotter = plotter

        self._actors_scene: list = []     # Mount/Substrate/etc.
        self._actors_lines: list = []     # Traj-Linien
        self._actors_points: list = []    # Traj-Punkte
        self._actors_normals: list = []   # Pfeil-Glyphs
        self._actors_rays: list = []      # Raycast-Linien

        self._max_extent_mm = float(max_extent_mm)
        half = 0.5 * self._max_extent_mm
        # feste Bounds: X/Y = ±half, Z = 0..max
        self._bounds = (-half, half, -half, half, 0.0, self._max_extent_mm)

        self._did_reset_after_first_mesh = False
        self._bbox_actor = None
        self._floor_actor = None
        self._triad_on = False

        self._install_default_scene()

    # ---------- Camera ----------
    def view_iso(self):
        self.plotter.view_isometric()
        self.plotter.reset_camera()

    # ---------- Default Scene ----------
    def _install_default_scene(self):
        xmin, xmax, ymin, ymax, zmin, zmax = self._bounds
        try:
            # Achsen-/Grid-Range so setzen, dass Z bei 0 startet
            self.plotter.set_axes_range(xmin, xmax, ymin, ymax, zmin, zmax)
            self.plotter.show_grid(
                location="outer",
                all_edges=True,
                ticks="both",
                color="gray",
            )
        except Exception:
            pass

        self._add_floor_grid()    # durchgehender Boden (Grid) bei Z=0
        self._add_orientation_triad()
        self._add_bounding_box()  # sichtbare Box 240×240×240 mm (Z 0..240)
        self.view_iso()

    def _add_orientation_triad(self):
        if self._triad_on:
            return
        try:
            axes = vtk.vtkAxesActor()
            axes.SetTotalLength(30.0, 30.0, 30.0)  # Pixelgröße
            self.plotter.add_orientation_widget(axes, interactive=False)
            self._triad_on = True
        except Exception:
            self._triad_on = False

    def _add_floor_grid(self):
        """Grid über die ganze Ansicht bei Z=0 (bevorzugt mit add_floor, sonst Fallback)."""
        if self._floor_actor is not None:
            return

        # 1) Versuch: eingebaute Bodenfläche (zeichnet ein View-umspannendes Grid)
        try:
            # PyVista >=0.41: add_floor(face='z', ...)
            act = self.plotter.add_floor(
                face='z',          # Ebene senkrecht zu Z
                offset=0.0,        # bei Z=0
                color=(0.96, 0.96, 0.96),
                line_color=(0.8, 0.8, 0.8),
                lighting=False,
            )
            self._floor_actor = act[0] if isinstance(act, tuple) else act
            return
        except Exception:
            pass

        # 2) Fallback: große fein unterteilte Plane mit Edge-Rendering als Grid
        half = 0.5 * self._max_extent_mm
        # viele Segmente, damit die Kanten ein Grid ergeben
        seg = 20
        plane = pv.Plane(
            center=(0.0, 0.0, 0.0),
            direction=(0.0, 0.0, 1.0),
            i_size=2 * half, j_size=2 * half,
            i_resolution=seg, j_resolution=seg,
        )
        self._floor_actor = self.plotter.add_mesh(
            plane,
            color=(0.98, 0.98, 0.98),
            opacity=1.0,
            smooth_shading=False,
            lighting=False,
            show_edges=True,
            edge_color=(0.82, 0.82, 0.82),
            reset_camera=False,
        )
        self._actors_scene.append(self._floor_actor)

    def _add_bounding_box(self):
        """Drahtgitter-Box mit Z von 0 bis max (nicht zentriert)."""
        if self._bbox_actor is not None:
            return
        half = 0.5 * self._max_extent_mm
        cube = pv.Cube(
            center=(0.0, 0.0, self._max_extent_mm * 0.5),
            x_length=2 * half, y_length=2 * half, z_length=self._max_extent_mm,
        )
        self._bbox_actor = self.plotter.add_mesh(
            cube,
            style="wireframe",
            line_width=1.0,
            opacity=0.0,                # Flächen unsichtbar, nur Kanten
            color="gray",
            lighting=False,
            reset_camera=False,
        )
        self._actors_scene.append(self._bbox_actor)

    # ---------- Scene Meshes ----------
    def add_mesh(self, mesh: pv.PolyData | pv.UnstructuredGrid, **kwargs):
        kwargs.pop("render_mode", None)  # Fremdparam aus älteren Pfaden ignorieren

        m = mesh
        if isinstance(m, pv.UnstructuredGrid):
            m = m.extract_surface()

        pts = np.asarray(m.points, dtype=np.float32, order="C")
        if pts.ndim != 2 or pts.shape[0] == 0 or pts.shape[1] < 3:
            pd = pv.PolyData()
        else:
            faces_arr = getattr(m, "faces", None)
            lines_arr = getattr(m, "lines", None)
            if isinstance(faces_arr, np.ndarray) and faces_arr.size > 0:
                pd = pv.PolyData(pts[:, :3], faces=faces_arr)  # gemischte Faces direkt übergeben
            elif isinstance(lines_arr, np.ndarray) and lines_arr.size > 0:
                pd = pv.PolyData(pts[:, :3])
                pd.lines = lines_arr
            else:
                pd = pv.PolyData(pts[:, :3])

        # sichtbare Defaults
        kwargs.setdefault("color", "lightgray")
        kwargs.setdefault("opacity", 1.0)
        kwargs.setdefault("smooth_shading", True)
        kwargs.setdefault("lighting", False)
        kwargs.setdefault("reset_camera", False)
        kwargs.setdefault("show_edges", False)
        kwargs.setdefault("copy_mesh", True)

        act = self.plotter.add_mesh(pd, **kwargs)
        actor = act[0] if isinstance(act, tuple) else act
        self._actors_scene.append(actor)

        try:
            if not self._did_reset_after_first_mesh and pd.n_points > 0:
                self.view_iso()
                self._did_reset_after_first_mesh = True
            self.plotter.render()
        except Exception:
            pass
        return actor

    # ---------- Clear ----------
    def _remove_list(self, L):
        for a in list(L):
            try:
                self.plotter.remove_actor(a)
            except Exception:
                pass
        L.clear()

    def clear_scene(self):
        self._remove_list(self._actors_scene)
        self._bbox_actor = None
        self._floor_actor = None
        self._did_reset_after_first_mesh = False
        self._install_default_scene()

    def clear_trajectory(self):
        self._remove_list(self._actors_lines)
        self._remove_list(self._actors_points)
        self._remove_list(self._actors_normals)
        self._remove_list(self._actors_rays)
        try:
            self.plotter.render()
        except Exception:
            pass

    def clear(self):
        self.clear_trajectory()
        self.clear_scene()
        self.view_iso()
