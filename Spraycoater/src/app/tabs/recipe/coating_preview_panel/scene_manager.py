# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Callable, Dict, List, Optional, Any, Tuple

import numpy as np
import pyvista as pv

_LOG = logging.getLogger("app.tabs.recipe.scene")


class SceneManager:
    """
    Dünne Schicht über dem PyVista-Interactor mit Layer-Verwaltung.
    + Utilitys für Floor-Grid, Achsen + Tick-Labels, Frames und Start/Ende-Marker.
    """

    def __init__(self, interactor_getter: Callable[[], Any], init_scene_builder: Callable[[], None] | None = None):
        self._get_ia: Callable[[], Any] = interactor_getter
        self._init_scene_builder = init_scene_builder
        self._layers: Dict[str, List[Any]] = {}   # layer -> [actors]

    # ---------- intern ----------
    def _ia(self):
        ia = self._get_ia()
        if ia is None:
            _LOG.warning("SceneManager: Interactor not available")
        return ia

    def _ensure_layer(self, layer: str) -> List[Any]:
        if layer not in self._layers:
            self._layers[layer] = []
        return self._layers[layer]

    def _remove_layer_actors(self, layer: str) -> None:
        ia = self._ia()
        if ia is None:
            return
        for a in self._layers.get(layer, []):
            try:
                ia.remove_actor(a)
            except Exception:
                pass
        self._layers[layer] = []

    # ---------- public: primitives ----------
    def add_mesh(self, mesh: pv.PolyData | pv.MultiBlock | Any, *,
                 layer: str = "default",
                 render: bool = False,
                 reset_camera: bool = False,
                 **kwargs) -> Optional[Any]:
        ia = self._ia()
        if ia is None or mesh is None:
            return None
        try:
            actor = ia.add_mesh(mesh, **kwargs)
            self._ensure_layer(layer).append(actor)
            if reset_camera:
                try:
                    b = getattr(mesh, "bounds", None)
                    ia.reset_camera(bounds=b if b is not None else None)
                except Exception:
                    _LOG.exception("reset_camera failed in add_mesh")
            if render:
                try:
                    ia.render()
                except Exception:
                    pass
            return actor
        except Exception:
            _LOG.exception("add_mesh failed")
            return None

    def add_lines(self, points: np.ndarray, lines: np.ndarray, *,
                  layer: str = "lines",
                  color: str = "#5a5a5a",
                  line_width: float = 1.0,
                  render: bool = False,
                  reset_camera: bool = False,
                  lighting: bool = False) -> Optional[Any]:
        ia = self._ia()
        if ia is None:
            return None
        try:
            P = np.asarray(points, float).reshape(-1, 3)
            L = np.asarray(lines)
            if L.ndim == 2 and L.shape[1] == 3:
                L = L.reshape(-1)
            poly = pv.PolyData(P); poly.lines = L
            return self.add_mesh(poly, layer=layer, color=color, line_width=float(line_width),
                                 render=render, reset_camera=reset_camera, lighting=lighting)
        except Exception:
            _LOG.exception("add_lines failed")
            return None

    def add_segments(self, segments: np.ndarray, *,
                     layer: str = "segments",
                     color: str = "#5a5a5a",
                     line_width: float = 1.0,
                     render: bool = False,
                     reset_camera: bool = False,
                     lighting: bool = False) -> Optional[Any]:
        try:
            S = np.asarray(segments, float).reshape(-1, 2, 3)
            if len(S) == 0:
                return None
            P = S.reshape(-1, 3)
            nseg = S.shape[0]
            lines = np.empty((nseg, 3), dtype=np.int64)
            lines[:, 0] = 2
            base = np.arange(nseg, dtype=np.int64) * 2
            lines[:, 1] = base; lines[:, 2] = base + 1
            return self.add_lines(P, lines, layer=layer, color=color, line_width=line_width,
                                  render=render, reset_camera=reset_camera, lighting=lighting)
        except Exception:
            _LOG.exception("add_segments failed")
            return None

    def add_path_polyline(self, points_mm: np.ndarray, *,
                          layer: str = "path",
                          color: str = "#2ecc71",
                          line_width: float = 2.0,
                          render: bool = False,
                          reset_camera: bool = False,
                          lighting: bool = False,
                          as_tube: bool = False,
                          tube_radius: float = 0.8,
                          tube_sides: int = 12,
                          tube_capping: bool = False) -> Optional[Any]:
        try:
            P = np.asarray(points_mm, float).reshape(-1, 3)
            if len(P) < 2:
                return None
            lines = np.hstack([[len(P)], np.arange(len(P), dtype=np.int64)])
            poly = pv.PolyData(P); poly.lines = lines
            if as_tube:
                try:
                    tube = poly.tube(radius=float(tube_radius),
                                     n_sides=int(tube_sides),
                                     capping=bool(tube_capping))
                    return self.add_mesh(tube, layer=layer, color=color,
                                         render=render, reset_camera=reset_camera, lighting=lighting)
                except Exception:
                    _LOG.exception("tube generation from polyline failed; fallback to line")
            return self.add_mesh(poly, layer=layer, color=color, line_width=float(line_width),
                                 render=render, reset_camera=reset_camera, lighting=lighting)
        except Exception:
            _LOG.exception("add_path_polyline failed")
            return None

    def show_poly(self, poly: pv.PolyData, *, layer: str,
                  color: str = "#2ecc71", line_width: float = 1.0,
                  lighting: bool = False, reset_camera: bool = False,
                  render: bool = False) -> Optional[Any]:
        self.clear_layer(layer)
        return self.add_mesh(poly, layer=layer, color=color,
                             line_width=float(line_width), lighting=lighting,
                             reset_camera=reset_camera, render=render)

    # ---------- floor grid + axes + labels ----------
    def add_floor_grid(self, bounds: Tuple[float, float, float, float, float, float], *,
                       step: float = 10.0,
                       layer: str = "floor_grid",
                       color: str = "#cfcfcf") -> Optional[Any]:
        ia = self._ia()
        if ia is None:
            return None
        xmin, xmax, ymin, ymax, zmin, _ = bounds
        width, height = xmax - xmin, ymax - ymin
        i_res = max(1, int(round(width / max(1e-6, step))))
        j_res = max(1, int(round(height / max(1e-6, step))))
        center = ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, zmin)
        plane = pv.Plane(center=center, direction=(0, 0, 1),
                         i_size=width, j_size=height,
                         i_resolution=i_res, j_resolution=j_res)
        return self.add_mesh(plane, style="wireframe", color=color,
                             line_width=1.0, lighting=False, render=False,
                             layer=layer)

    def add_floor_axes(self, bounds: Tuple[float, float, float, float, float, float], *,
                       step: float = 10.0,
                       layer_lines: str = "floor_axes",
                       layer_labels: str = "floor_labels") -> Optional[Any]:
        ia = self._ia()
        if ia is None:
            return None

        xmin, xmax, ymin, ymax, zmin, _ = bounds
        step = max(1.0, float(step))

        def ticks(vmin, vmax):
            start = np.ceil(vmin / step) * step
            vals = np.arange(start, vmax + 0.5 * step, step, dtype=float)
            return vals[(vals >= vmin + 1e-6) & (vals <= vmax - 1e-6)]

        xt, yt = ticks(xmin, xmax), ticks(ymin, ymax)
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)

        pts, lines = [], []
        base = 0

        def add_seg(p0, p1):
            nonlocal base
            pts.extend([p0, p1]); lines.extend([2, base, base + 1]); base += 2

        # axes
        add_seg((xmin, cy, zmin), (xmax, cy, zmin))
        add_seg((cx, ymin, zmin), (cx, ymax, zmin))

        # ticks
        tick_len = max(1.0, min(3.0, step * 0.15))
        for xv in xt:
            add_seg((xv, cy - tick_len, zmin), (xv, cy + tick_len, zmin))
        for yv in yt:
            add_seg((cx - tick_len, yv, zmin), (cx + tick_len, yv, zmin))

        self.clear_layer(layer_lines)
        actor = None
        if pts:
            poly = pv.PolyData(np.asarray(pts, dtype=float))
            poly.lines = np.asarray(lines, dtype=np.int64)
            actor = self.add_mesh(poly, style="wireframe", color="#5a5a5a",
                                  line_width=1.0, lighting=False, render=False,
                                  layer=layer_lines)

        # labels
        self.clear_layer(layer_labels)
        new_labels: list = []
        label_z = zmin + max(0.5, tick_len * 0.5)
        try:
            if len(xt):
                labx = ia.add_point_labels(
                    np.c_[xt, np.full_like(xt, cy), np.full_like(xt, label_z)],
                    [f"{int(v)}" if abs(v - int(v)) < 1e-6 else f"{v:.1f}" for v in xt],
                    point_size=0, font_size=10, text_color="black",
                    shape_opacity=0.0, render=False
                )
                self._ensure_layer(layer_labels).append(labx)
                new_labels.append(labx)
            if len(yt):
                laby = ia.add_point_labels(
                    np.c_[np.full_like(yt, cx), yt, np.full_like(yt, label_z)],
                    [f"{int(v)}" if abs(v - int(v)) < 1e-6 else f"{v:.1f}" for v in yt],
                    point_size=0, font_size=10, text_color="black",
                    shape_opacity=0.0, render=False
                )
                self._ensure_layer(layer_labels).append(laby)
                new_labels.append(laby)
            labxy = ia.add_point_labels(
                [(xmax, cy, label_z), (cx, ymax, label_z)],
                ["X (mm)", "Y (mm)"],
                point_size=0, font_size=12, text_color="black",
                shape_opacity=0.0, render=False
            )
            self._ensure_layer(layer_labels).append(labxy)
            new_labels.append(labxy)
        except Exception:
            _LOG.exception("floor labels failed")

        return actor

    def refresh_floor(self, bounds: Tuple[float, float, float, float, float, float], *,
                      step: float = 10.0) -> None:
        """Alles was Boden/Koordinaten betrifft in einem Rutsch aktualisieren."""
        self.clear_layer("floor_grid")
        self.clear_layer("floor_axes")
        self.clear_layer("floor_labels")
        self.add_floor_grid(bounds, step=step, layer="floor_grid")
        self.add_floor_axes(bounds, step=step, layer_lines="floor_axes", layer_labels="floor_labels")
        ia = self._ia()
        try:
            if ia is not None:
                ia.reset_camera(bounds=bounds)
                ia.render()
        except Exception:
            _LOG.exception("refresh_floor: reset/render failed")

    # ---------- frames ----------
    def add_frames(self, *,
                   origins: np.ndarray,
                   z_dirs: np.ndarray,
                   x_dirs: np.ndarray | None = None,
                   scale: float | None = None,
                   scale_mm: float | None = None,
                   line_width: float | None = 1.0,
                   layer_prefix: str = "frames",
                   add_labels: bool = False,
                   labels: list[str] | None = None) -> None:
        ia = self._ia()
        if ia is None:
            _LOG.warning("add_frames(): no interactor")
            return

        lx, ly, lz, llab = (f"{layer_prefix}_x", f"{layer_prefix}_y", f"{layer_prefix}_z", f"{layer_prefix}_labels")
        for lyr in (lx, ly, lz, llab):
            self.clear_layer(lyr)

        O = np.asarray(origins, dtype=float).reshape(-1, 3)
        Z = np.asarray(z_dirs, dtype=float).reshape(-1, 3)
        if O.shape != Z.shape:
            raise ValueError("origins und z_dirs müssen gleiche Form (N,3) haben")

        def _safe_norm(v, eps=1e-9):
            v = np.asarray(v, dtype=float).reshape(3)
            n = float(np.linalg.norm(v))
            return v / n if n >= eps else np.array([0.0, 0.0, 1.0])

        def _basis_from_z(z):
            z = _safe_norm(z)
            h = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            x = _safe_norm(np.cross(h, z))
            y = _safe_norm(np.cross(z, x))
            return x, y, z

        if scale_mm is not None:
            s = float(scale_mm)
        else:
            s = float(scale or 10.0)

        pts_x, lns_x, ix = [], [], 0
        pts_y, lns_y, iy = [], [], 0
        pts_z, lns_z, iz = [], [], 0

        X = None if x_dirs is None else np.asarray(x_dirs, dtype=float).reshape(-1, 3)
        if X is not None and X.shape != O.shape:
            raise ValueError("x_dirs muss Form (N,3) haben und zu origins passen")

        for i in range(O.shape[0]):
            o = O[i]
            if X is not None:
                z = _safe_norm(Z[i])
                x = _safe_norm(X[i])
                x = _safe_norm(np.cross(np.cross(x, z), z))
                y = _safe_norm(np.cross(z, x))
            else:
                x, y, z = _basis_from_z(Z[i])

            px, py, pz = o + x * s, o + y * s, o + z * s
            pts_x.extend([o, px]); lns_x.extend([2, ix, ix + 1]); ix += 2
            pts_y.extend([o, py]); lns_y.extend([2, iy, iy + 1]); iy += 2
            pts_z.extend([o, pz]); lns_z.extend([2, iz, iz + 1]); iz += 2

        def _poly(pts, lns):
            poly = pv.PolyData(np.asarray(pts))
            poly.lines = np.asarray(lns, dtype=np.int64).reshape(-1)
            return poly

        self.add_mesh(_poly(pts_x, lns_x), color="red",   layer=lx, line_width=float(line_width or 1.0),
                      reset_camera=False, render=False, lighting=False)
        self.add_mesh(_poly(pts_y, lns_y), color="green", layer=ly, line_width=float(line_width or 1.0),
                      reset_camera=False, render=False, lighting=False)
        self.add_mesh(_poly(pts_z, lns_z), color="blue",  layer=lz, line_width=float(line_width or 1.0),
                      reset_camera=False, render=False, lighting=False)

        if add_labels and labels:
            try:
                L = min(len(labels), O.shape[0])
                if L > 0:
                    lab_actor = ia.add_point_labels(
                        O[:L], labels[:L], point_size=0, font_size=12, shape_opacity=0.3, render=False
                    )
                    self._ensure_layer(llab).append(lab_actor)
            except Exception:
                _LOG.exception("add_frames: labels failed")

    # ---------- start/end markers ----------
    def add_start_end_markers(self, points: np.ndarray, *,
                              layer_prefix: str = "path",
                              color: str = "#2ecc71",
                              size: float = 3.0) -> None:
        ia = self._ia()
        if ia is None:
            return
        P = np.asarray(points, dtype=float).reshape(-1, 3)
        layer = f"{layer_prefix}_markers"
        self.clear_layer(layer)

        if len(P) == 0:
            return
        start = P[0]; end = P[-1]
        try:
            s0 = pv.Sphere(radius=float(size) * 0.5, center=start)
            s1 = pv.Sphere(radius=float(size) * 0.5, center=end)
            self.add_mesh(s0, color=color, layer=layer, render=False, lighting=False)
            self.add_mesh(s1, color=color, layer=layer, render=False, lighting=False)
        except Exception:
            _LOG.exception("add_start_end_markers: spheres failed")
        try:
            lab = ia.add_point_labels(
                [start, end], ["Start", "Ende"],
                point_size=0, font_size=12, text_color="black",
                shape_opacity=0.25, render=False
            )
            self._ensure_layer(layer).append(lab)
        except Exception:
            _LOG.exception("add_start_end_markers: labels failed")

    # ---------- layer ops ----------
    def clear_layer(self, layer: str, *, render: bool = False) -> None:
        self._remove_layer_actors(layer)
        if render:
            ia = self._ia()
            try:
                if ia is not None:
                    ia.render()
            except Exception:
                pass

    def clear(self, *, render: bool = False) -> None:
        ia = self._ia()
        if ia is None:
            return
        try:
            for layer in list(self._layers.keys()):
                self._remove_layer_actors(layer)
            if render:
                ia.render()
        except Exception:
            _LOG.exception("clear failed")

    def has_layer(self, layer: str) -> bool:
        return layer in self._layers and len(self._layers[layer]) > 0

    def set_layer_visible(self, layer: str, visible: bool, *, render: bool = True) -> None:
        ia = self._ia()
        if ia is None:
            return
        actors = self._layers.get(layer, [])
        for a in actors:
            try:
                a.SetVisibility(1 if visible else 0)
            except Exception:
                pass
        if render:
            try:
                ia.render()
            except Exception:
                pass

    def get_layer_bounds(self, layer: str) -> Optional[Tuple[float, float, float, float, float, float]]:
        actors = self._layers.get(layer, [])
        if not actors:
            return None
        mins = np.array([ np.inf,  np.inf,  np.inf], float)
        maxs = np.array([-np.inf, -np.inf, -np.inf], float)
        any_ok = False
        for a in actors:
            try:
                b = a.GetBounds()
                if b is None:
                    continue
                b = np.asarray(b, float).reshape(6)
                if not np.all(np.isfinite(b)):
                    continue
                mins = np.minimum(mins, [b[0], b[2], b[4]])
                maxs = np.maximum(maxs, [b[1], b[3], b[5]])
                any_ok = True
            except Exception:
                pass
        if not any_ok:
            return None
        return (float(mins[0]), float(maxs[0]),
                float(mins[1]), float(maxs[1]),
                float(mins[2]), float(maxs[2]))
