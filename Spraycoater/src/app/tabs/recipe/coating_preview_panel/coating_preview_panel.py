# -*- coding: utf-8 -*-
from __future__ import annotations
import os, logging
from typing import Optional, Any

import numpy as np
import pyvista as pv
from PyQt6 import uic
from PyQt6.QtWidgets import QWidget, QPushButton, QCheckBox, QFrame

from .interactor_host import InteractorHost
from .scene_manager import SceneManager
from .views import ViewController

_LOG = logging.getLogger("app.tabs.recipe")

def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)

# ----- math utils -----
def _safe_norm(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n < eps:
        return np.array([0.0, 0.0, 1.0])
    return v / n

def _orthonormal_basis_from_z(z: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    z = _safe_norm(z)
    h = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = _safe_norm(np.cross(h, z))
    y = _safe_norm(np.cross(z, x))
    return x, y, z

class CoatingPreviewPanel(QWidget):
    DEFAULT_MOUNT_COLOR = "lightgray"
    DEFAULT_SUBSTRATE_COLOR = "#3498db"

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        uic.loadUi(_ui_path("coating_preview_panel.ui"), self)

        # UI refs
        self.chkShowMask: QCheckBox = self.findChild(QCheckBox, "chkShowMask")
        self.chkShowRays: QCheckBox = self.findChild(QCheckBox, "chkShowRays")
        self.chkShowHits: QCheckBox = self.findChild(QCheckBox, "chkShowHits")
        self.chkShowMisses: QCheckBox = self.findChild(QCheckBox, "chkShowMisses")
        self.chkShowNormals: QCheckBox = self.findChild(QCheckBox, "chkShowNormals")
        self.chkShowLocalFrames: QCheckBox = self.findChild(QCheckBox, "chkShowLocalFrames")

        self.btnCamIso:   QPushButton = self.findChild(QPushButton, "btnCamIso")
        self.btnCamTop:   QPushButton = self.findChild(QPushButton, "btnCamTop")
        self.btnCamFront: QPushButton = self.findChild(QPushButton, "btnCamFront")
        self.btnCamBack:  QPushButton = self.findChild(QPushButton, "btnCamBack")
        self.btnCamLeft:  QPushButton = self.findChild(QPushButton, "btnCamLeft")
        self.btnCamRight: QPushButton = self.findChild(QPushButton, "btnCamRight")

        self._host: QFrame = self.findChild(QFrame, "previewHost")
        if self._host is None:
            raise RuntimeError("coating_preview_panel.ui braucht QFrame 'previewHost'.")

        # Composition
        self._hoster = InteractorHost(self, self._host)
        self.scene = SceneManager(lambda: self._hoster.ia, None)
        self.views = ViewController(lambda: self._hoster.ia, self.render)

        # Bounds / grid state
        self._bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)
        self._floor_actor  = None
        self._axis_actor   = None
        self._grid_step    = 10.0

        # camera buttons
        self.views.wire_buttons(
            btn_iso=self.btnCamIso, btn_top=self.btnCamTop, btn_front=self.btnCamFront,
            btn_left=self.btnCamLeft, btn_right=self.btnCamRight, btn_back=self.btnCamBack
        )

        # layers
        self._layer_mask   = "mask"
        self._layer_rays_hit  = "rays_hit"
        self._layer_rays_miss = "rays_miss"
        self._layer_normals   = "normals"
        self._frames_layers   = ("frames_x", "frames_y", "frames_z", "frames_labels")
        self._frames_layers_active = self._frames_layers

        # defaults
        for cb in (self.chkShowMask, self.chkShowRays, self.chkShowHits, self.chkShowMisses, self.chkShowNormals, self.chkShowLocalFrames):
            if cb: cb.setChecked(True)

        # wiring — just toggling visibility
        if self.chkShowMask:
            self.chkShowMask.toggled.connect(lambda vis: self.scene.set_layer_visible(self._layer_mask, vis, render=True))
        if self.chkShowRays:
            self.chkShowRays.toggled.connect(self._on_toggle_rays_master)
        if self.chkShowHits:
            self.chkShowHits.toggled.connect(lambda vis: self.scene.set_layer_visible(self._layer_rays_hit, vis, render=True))
        if self.chkShowMisses:
            self.chkShowMisses.toggled.connect(lambda vis: self.scene.set_layer_visible(self._layer_rays_miss, vis, render=True))
        if self.chkShowNormals:
            self.chkShowNormals.toggled.connect(lambda vis: self.scene.set_layer_visible(self._layer_normals, vis, render=True))
        if self.chkShowLocalFrames:
            self.chkShowLocalFrames.toggled.connect(self._on_toggle_frames)

    # hosting
    def preview_host(self) -> QFrame: return self._host
    def attach_interactor(self, interactor: Any) -> None: self._hoster.attach(interactor)

    # master toggle for rays affects both layers
    def _on_toggle_rays_master(self, vis: bool):
        self.scene.set_layer_visible(self._layer_rays_hit, vis and (self.chkShowHits.isChecked() if self.chkShowHits else True), render=False)
        self.scene.set_layer_visible(self._layer_rays_miss, vis and (self.chkShowMisses.isChecked() if self.chkShowMisses else True), render=True)

    # grid / bounds
    def set_grid_step(self, step_mm: float) -> None:
        self._grid_step = max(1.0, float(step_mm))
        self._apply_bounds()

    def _apply_bounds(self) -> None:
        ia = self._hoster.ia or (self._hoster.ensure() and self._hoster.ia)
        if ia is None: return
        for actor in (self._floor_actor, self._axis_actor):
            if actor is not None:
                try: ia.remove_actor(actor, render=False)
                except Exception: pass
        self._floor_actor = None
        self._axis_actor  = None
        self._floor_actor = self._make_floor_wire(ia, step=self._grid_step)
        self._axis_actor  = self._draw_floor_axes(ia, step=self._grid_step)
        try: ia.reset_camera(bounds=self._bounds)
        except Exception: pass
        ia.render()

    def _make_floor_wire(self, ia, step: float):
        xmin, xmax, ymin, ymax, zmin, _ = self._bounds
        width  = xmax - xmin
        height = ymax - ymin
        i_res = max(1, int(round(width  / max(1e-6, step))))
        j_res = max(1, int(round(height / max(1e-6, step))))
        center = ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, zmin)
        plane = pv.Plane(center=center, direction=(0, 0, 1),
                         i_size=width, j_size=height,
                         i_resolution=i_res, j_resolution=j_res)
        return ia.add_mesh(plane, style="wireframe", color="#cfcfcf",
                           line_width=1.0, lighting=False, render=False)

    def _draw_floor_axes(self, ia, step: float = 10.0):
        xmin, xmax, ymin, ymax, zmin, _ = self._bounds
        step = max(1.0, float(step))
        def ticks(vmin, vmax):
            start = np.ceil(vmin / step) * step
            vals = np.arange(start, vmax + 0.5*step, step, dtype=float)
            return vals[(vals >= vmin + 1e-6) & (vals <= vmax - 1e-6)]
        xt, yt = ticks(xmin, xmax), ticks(ymin, ymax)
        cx, cy = 0.5*(xmin+xmax), 0.5*(ymin+ymax)

        lines_pts, lines_idx, base = [], [], 0
        def add_seg(p0, p1):
            nonlocal base
            lines_pts.extend([p0, p1]); lines_idx.extend([2, base, base+1]); base += 2
        add_seg((xmin, cy, zmin), (xmax, cy, zmin))
        add_seg((cx, ymin, zmin), (cx, ymax, zmin))

        tick_len = max(1.0, min(3.0, step*0.15))
        for xv in xt: add_seg((xv, cy - tick_len, zmin), (xv, cy + tick_len, zmin))
        for yv in yt: add_seg((cx - tick_len, yv, zmin), (cx + tick_len, yv, zmin))

        actor = None
        if lines_pts:
            poly = pv.PolyData(np.asarray(lines_pts, dtype=float))
            poly.lines = np.asarray(lines_idx, dtype=np.int64)
            actor = ia.add_mesh(poly, style="wireframe", color="#5a5a5a",
                                line_width=1.0, lighting=False, render=False)

        label_z = zmin + max(0.5, tick_len * 0.5)
        if len(xt):
            ia.add_point_labels(np.c_[xt, np.full_like(xt, cy), np.full_like(xt, label_z)],
                                [f"{int(v)}" if abs(v-int(v))<1e-6 else f"{v:.1f}" for v in xt],
                                point_size=0, font_size=10, text_color="black",
                                shape_opacity=0.0, render=False)
        if len(yt):
            ia.add_point_labels(np.c_[np.full_like(yt, cx), yt, np.full_like(yt, label_z)],
                                [f"{int(v)}" if abs(v-int(v))<1e-6 else f"{v:.1f}" for v in yt],
                                point_size=0, font_size=10, text_color="black",
                                shape_opacity=0.0, render=False)
        ia.add_point_labels([(xmax, cy, label_z), (cx, ymax, label_z)],
                            ["X (mm)", "Y (mm)"],
                            point_size=0, font_size=12, text_color="black",
                            shape_opacity=0.0, render=False)
        return actor

    def set_world_bounds_at(self, *, center_xy=(0.0, 0.0), z0=0.0, span_xy=240.0, span_z=240.0) -> None:
        cx, cy = float(center_xy[0]), float(center_xy[1])
        half = float(span_xy) * 0.5
        z0, span_z = float(z0), float(span_z)
        self._bounds = (cx-half, cx+half, cy-half, cy+half, z0, z0+span_z)
        self._apply_bounds()

    def set_bounds_from_mesh(self, mesh, *, use_contact_plane=True, span_xy=240.0, span_z=240.0) -> None:
        if not hasattr(mesh, "bounds"): return
        xmin, xmax, ymin, ymax, zmin, zmax = mesh.bounds
        cx = 0.5*(xmin+xmax); cy = 0.5*(ymin+ymax)
        z0 = float(zmin if use_contact_plane else 0.5*(zmin+zmax))
        self.set_world_bounds_at(center_xy=(cx, cy), z0=z0, span_xy=span_xy, span_z=span_z)

    # scene passthrough
    def clear(self) -> None:
        if self._hoster.ia is None and not self._hoster.ensure(): return
        self.scene.clear()
    def add_mesh(self, mesh, **kwargs) -> None:
        if self._hoster.ia is None and not self._hoster.ensure():
            _LOG.warning("add_mesh(): kein Interactor"); return
        self.scene.add_mesh(mesh, **kwargs)
    def clear_layer(self, layer: str) -> None: self.scene.clear_layer(layer)
    def add_path_polyline(self, *a, **kw): self.scene.add_path_polyline(*a, **kw)
    def add_path_markers(self, *a, **kw): self.scene.add_path_markers(*a, **kw)

    # normals as green line segments
    def show_normals_from_hits(self, points_mm: np.ndarray, normals: np.ndarray,
                               *, layer: str = "normals", length_mm: float = 8.0,
                               color: str = "#27ae60", line_width: float = 1.0):
        try:
            P = np.asarray(points_mm, dtype=float).reshape(-1, 3)
            N = np.asarray(normals,   dtype=float).reshape(-1, 3)
            M = min(len(P), len(N))
            if M == 0:
                self.clear_layer(layer); return
            A = P[:M]; B = P[:M] + N[:M]*float(length_mm)
            pts = np.vstack([A, B])
            lines = np.empty((M, 3), dtype=np.int64)
            lines[:, 0] = 2; lines[:, 1] = np.arange(M, dtype=np.int64); lines[:, 2] = np.arange(M, dtype=np.int64)+M
            poly = pv.PolyData(pts); poly.lines = lines.reshape(-1)
            self.clear_layer(layer)
            self.add_mesh(poly, color=color, render=False, reset_camera=False,
                          line_width=float(line_width), lighting=False, layer=layer)
        except Exception:
            _LOG.exception("show_normals_from_hits() failed")

    # local frames
    def clear_frames(self) -> None:
        for lyr in self._frames_layers_active:
            try: self.scene.clear_layer(lyr)
            except Exception: pass
    def _on_toggle_frames(self, vis: bool):
        for lyr in self._frames_layers_active[:3]:
            self.scene.set_layer_visible(lyr, vis, render=False)
        self.scene.set_layer_visible(self._frames_layers_active[3], vis, render=True)

    def show_frames_at(self, *, origins: np.ndarray, z_dirs: np.ndarray,
                       x_dirs: np.ndarray | None = None, scale: float = 10.0,
                       scale_mm: float | None = None, tube_radius: float = 0.7,
                       line_width: float | None = None, layer: str | None = None,
                       labels: list[str] | None = None, clear_old: bool = True,
                       add_labels: bool = False) -> None:
        if self._hoster.ia is None and not self._hoster.ensure():
            _LOG.warning("show_frames_at(): kein Interactor"); return
        if scale_mm is not None: scale = float(scale_mm)
        O = np.asarray(origins, dtype=float).reshape(-1, 3)
        Z = np.asarray(z_dirs, dtype=float).reshape(-1, 3)
        if O.shape != Z.shape: raise ValueError("origins und z_dirs müssen gleiche Form (N,3) haben")
        X = None
        if x_dirs is not None:
            X = np.asarray(x_dirs, dtype=float).reshape(-1, 3)
            if X.shape != O.shape: raise ValueError("x_dirs muss Form (N,3) haben und zu origins passen")
        if layer and layer.strip():
            base = layer.strip()
            frames_layers = (f"{base}_x", f"{base}_y", f"{base}_z", f"{base}_labels")
        else:
            frames_layers = self._frames_layers
        self._frames_layers_active = frames_layers
        lx, ly, lz, llab = frames_layers
        if clear_old: self.clear_frames()
        N = O.shape[0]
        if N == 0: return
        def _acc(): return [], []
        pts_x, lns_x = _acc(); pts_y, lns_y = _acc(); pts_z, lns_z = _acc()
        idx_x = idx_y = idx_z = 0
        for i in range(N):
            o = O[i]
            if X is not None:
                z = _safe_norm(Z[i]); x = _safe_norm(X[i]); x = _safe_norm(np.cross(np.cross(x, z), z)); y = _safe_norm(np.cross(z, x))
            else:
                x, y, z = _orthonormal_basis_from_z(Z[i])
            px = o + x*float(scale); py = o + y*float(scale); pz = o + z*float(scale)
            pts_x.extend([o, px]); lns_x.extend([2, idx_x, idx_x+1]); idx_x += 2
            pts_y.extend([o, py]); lns_y.extend([2, idx_y, idx_y+1]); idx_y += 2
            pts_z.extend([o, pz]); lns_z.extend([2, idx_z, idx_z+1]); idx_z += 2
        def _poly_from(pts, lns):
            poly = pv.PolyData(np.asarray(pts)); poly.lines = np.asarray(lns, dtype=np.int64).reshape(-1); return poly
        poly_x = _poly_from(pts_x, lns_x); poly_y = _poly_from(pts_y, lns_y); poly_z = _poly_from(pts_z, lns_z)
        for lyr in (lx, ly, lz): self.scene.clear_layer(lyr)
        if line_width is not None and float(line_width) > 0.0:
            self.scene.add_mesh(poly_x, color="red",   layer=lx, line_width=float(line_width), reset_camera=False, render=False, lighting=False)
            self.scene.add_mesh(poly_y, color="green", layer=ly, line_width=float(line_width), reset_camera=False, render=False, lighting=False)
            self.scene.add_mesh(poly_z, color="blue",  layer=lz, line_width=float(line_width), reset_camera=False, render=False, lighting=False)
        else:
            tx = poly_x.tube(radius=float(tube_radius)); ty = poly_y.tube(radius=float(tube_radius)); tz = poly_z.tube(radius=float(tube_radius))
            self.scene.add_mesh(tx, color="red",   layer=lx, reset_camera=False, render=False, lighting=False)
            self.scene.add_mesh(ty, color="green", layer=ly, reset_camera=False, render=False, lighting=False)
            self.scene.add_mesh(tz, color="blue",  layer=lz, reset_camera=False, render=False, lighting=False)
        if add_labels and labels:
            try:
                L = min(len(labels), O.shape[0])
                if L > 0:
                    ia = self._hoster.ia
                    self.scene.clear_layer(llab)
                    ia.add_point_labels(O[:L], labels[:L], point_size=0, font_size=12, shape_opacity=0.3)
            except Exception:
                _LOG.exception("Labels hinzufügen fehlgeschlagen (optional)")

    # views & render
    def view_isometric(self): self.views.view_isometric()
    def view_top(self):       self.views.view_top()
    def view_front(self):     self.views.view_front()
    def view_left(self):      self.views.view_left()
    def view_right(self):     self.views.view_right()
    def view_back(self):      self.views.view_back()

    def render(self, *, reset_camera: bool = True) -> None:
        ia = self._hoster.ia if self._hoster.ia is not None else (self._hoster.ensure() and self._hoster.ia)
        if ia is None: return
        try:
            if reset_camera: ia.reset_camera(bounds=self._bounds)
            ia.render()
        except Exception:
            _LOG.exception("render() failed")
