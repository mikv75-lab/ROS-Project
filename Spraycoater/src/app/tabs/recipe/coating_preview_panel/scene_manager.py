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
            self._build_grid()  # Grid wieder aufbauen
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
            ia.add_mesh(mesh, reset_camera=False, render=False, **kwargs)
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
            try: ia.remove_actor(a, render=False)
            except Exception: pass
        try: ia.render()
        except Exception: pass

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
            pick = pts[::max(1, int(step))] or pts[:1]
            src = pv.Sphere(radius=float(radius_mm))
            cloud = pv.PolyData(pick)
            glyphs = cloud.glyph(scale=False, geom=src)
            actor = ia.add_mesh(glyphs, color=color, render=False, reset_camera=False, lighting=lighting)
            self._layer_add_actor(layer, actor)
        except Exception:
            _LOG.exception("add_path_markers() failed")
