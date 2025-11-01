# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Any, Dict, List
import numpy as np
import pyvista as pv

_LOG = logging.getLogger("app.tabs.recipe.preview.scene")


class SceneManager:
    """Clear, add_mesh, Layer-Handling und einfache Path-Renderer."""
    DEFAULT_PATH_COLOR = "#ff7f0e"
    DEFAULT_MARKER_COLOR = "#2ecc71"

    def __init__(self, interactor_getter: callable, grid_build_callable: callable):
        self._get_ia = interactor_getter
        self._build_grid = grid_build_callable
        self._layers: Dict[str, List[Any]] = {}

    def clear(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.clear()
            # Wichtig: KEIN Grid-Rebuild mehr hier!
            self._layers.clear()
        except Exception:
            _LOG.exception("clear() failed")

    def add_mesh(self, mesh, **kwargs):
        ia = self._get_ia()
        if ia is None:
            _LOG.warning("add_mesh(): kein Interactor")
            return
        try:
            fn = getattr(mesh, "is_all_triangles", None)
            if callable(fn) and not fn():
                mesh = mesh.triangulate()
        except Exception:
            pass
        try:
            if "render" not in kwargs:
                kwargs["render"] = False
            if "reset_camera" not in kwargs:
                kwargs["reset_camera"] = False
            actor = ia.add_mesh(mesh, **kwargs)
            # wenn ein Layer via kwargs übergeben wird, tracken
            layer = kwargs.get("layer")
            if layer is not None:
                self._layers.setdefault(layer, []).append(actor)
        except Exception:
            _LOG.exception("add_mesh() failed")

    # — Layer —
    def _layer_add_actor(self, layer: str, actor) -> None:
        self._layers.setdefault(layer, []).append(actor)

    def clear_layer(self, layer: str) -> None:
        ia = self._get_ia()
        if ia is None:
            return
        actors = self._layers.pop(layer, [])
        for a in actors:
            try:
                ia.remove_actor(a, render=False)
            except Exception:
                pass
        try:
            ia.render()
        except Exception:
            pass

    # — Paths —
    def add_path_polyline(self, points_mm: np.ndarray, *,
                          color: str = None, line_width: float = 2.0,
                          as_tube: bool = False, tube_radius_mm: float = 0.6,
                          layer: str = "path", lighting: bool = False):
        ia = self._get_ia()
        if ia is None:
            _LOG.warning("add_path_polyline(): kein Interactor")
            return
        pts = np.asarray(points_mm, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 2:
            _LOG.error("add_path_polyline(): erwartet Nx3, N>=2")
            return
        if color is None:
            color = self.DEFAULT_PATH_COLOR
        n = pts.shape[0]
        lines = np.hstack(([n], np.arange(n, dtype=np.int64)))
        poly = pv.PolyData(pts, lines=lines)
        try:
            if as_tube:
                tube = poly.tube(radius=float(tube_radius_mm), n_sides=12, capping=True)
                actor = ia.add_mesh(tube, color=color, render=False, reset_camera=False, lighting=lighting)
            else:
                actor = ia.add_mesh(poly, color=color, render=False, reset_camera=False,
                                    line_width=float(line_width), lighting=lighting)
            self._layer_add_actor(layer, actor)
        except Exception:
            _LOG.exception("add_path_polyline() failed")

    def add_path_markers(self, points_mm: np.ndarray, *,
                         radius_mm: float = 0.8, step: int = 10,
                         color: str = None, layer: str = "path_markers",
                         lighting: bool = False):
        ia = self._get_ia()
        if ia is None:
            return
        pts = np.asarray(points_mm, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 1:
            return
        if color is None:
            color = self.DEFAULT_MARKER_COLOR
        try:
            pick = pts[::max(1, int(step))]
            if pick.shape[0] == 0:
                pick = pts[:1]
            src = pv.Sphere(radius=float(radius_mm))
            cloud = pv.PolyData(pick)
            glyphs = cloud.glyph(scale=False, geom=src)
            actor = ia.add_mesh(glyphs, color=color, render=False, reset_camera=False, lighting=lighting)
            self._layer_add_actor(layer, actor)
        except Exception:
            _LOG.exception("add_path_markers() failed")

    # — Zusätze für Rays/Poly/Normalen/Frames —
    def show_poly(self, poly: pv.PolyData, *, layer: str, color: str = "#85c1e9",
                  line_width: float = 1.0, lighting: bool = False):
        ia = self._get_ia()
        if ia is None or poly is None or poly.n_points == 0:
            return
        try:
            actor = ia.add_mesh(poly, color=color, line_width=float(line_width),
                                lighting=lighting, render=False, reset_camera=False)
            self._layer_add_actor(layer, actor)
        except Exception:
            _LOG.exception("show_poly() failed")

    def show_normals_from_hits(self, hits: np.ndarray, normals: np.ndarray, *,
                               length_mm: float = 6.0, color: str = "#27ae60",
                               layer: str = "normals", line_width: float = 1.0):
        ia = self._get_ia()
        if ia is None:
            return
        H = np.asarray(hits, dtype=float).reshape(-1, 3)
        N = np.asarray(normals, dtype=float).reshape(-1, 3)
        M = min(len(H), len(N))
        if M == 0:
            return
        A = H[:M]
        B = H[:M] + N[:M] * float(length_mm)

        pts = np.vstack([A, B])
        lines = np.empty((M, 3), dtype=np.int64)
        lines[:, 0] = 2
        lines[:, 1] = np.arange(0, M, dtype=np.int64)
        lines[:, 2] = np.arange(M, 2*M, dtype=np.int64)
        pd = pv.PolyData()
        pd.points = pts
        pd.lines = lines.reshape(-1)

        try:
            actor = ia.add_mesh(pd, color=color, line_width=float(line_width),
                                render=False, reset_camera=False)
            self._layer_add_actor(layer, actor)
        except Exception:
            _LOG.exception("show_normals_from_hits() failed")

    def show_frames_at(self, origins: np.ndarray, z_dirs: np.ndarray, *,
                       scale_mm: float = 10.0, layer: str = "frames",
                       line_width: float = 2.0):
        ia = self._get_ia()
        if ia is None:
            return
        O = np.asarray(origins, dtype=float).reshape(-1, 3)
        Z = np.asarray(z_dirs, dtype=float).reshape(-1, 3)
        K = min(len(O), len(Z))
        if K == 0:
            return

        def _norm(v):
            n = np.linalg.norm(v, axis=-1, keepdims=True) + 1e-12
            return v / n

        Z = _norm(Z)
        X = np.cross(np.tile(np.array([[0,0,1.0]], float), (K,1)), Z)
        bad = (np.linalg.norm(X, axis=1) < 1e-6)
        if np.any(bad):
            X[bad] = np.cross(np.tile(np.array([[0,1.0,0]], float), (np.sum(bad),1)), Z[bad])
        X = _norm(X)
        Y = _norm(np.cross(Z, X))

        def _build_axis_poly(origins, dirs, s):
            A = origins
            B = origins + dirs * s
            pts = np.vstack([A, B])
            M = len(A)
            lines = np.empty((M, 3), dtype=np.int64)
            lines[:, 0] = 2
            lines[:, 1] = np.arange(0, M, dtype=np.int64)
            lines[:, 2] = np.arange(M, 2*M, dtype=np.int64)
            pd = pv.PolyData()
            pd.points = pts
            pd.lines = lines.reshape(-1)
            return pd

        try:
            ax_x = _build_axis_poly(O, X, float(scale_mm))
            ax_y = _build_axis_poly(O, Y, float(scale_mm))
            ax_z = _build_axis_poly(O, Z, float(scale_mm))
            a1 = ia.add_mesh(ax_x, color="#e74c3c", line_width=float(line_width),
                             render=False, reset_camera=False)   # X rot
            a2 = ia.add_mesh(ax_y, color="#2ecc71", line_width=float(line_width),
                             render=False, reset_camera=False)   # Y grün
            a3 = ia.add_mesh(ax_z, color="#3498db", line_width=float(line_width),
                             render=False, reset_camera=False)   # Z blau
            self._layer_add_actor(layer, a1); self._layer_add_actor(layer, a2); self._layer_add_actor(layer, a3)
        except Exception:
            _LOG.exception("show_frames_at() failed")
