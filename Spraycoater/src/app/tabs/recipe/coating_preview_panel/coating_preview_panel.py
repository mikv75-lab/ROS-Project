# -*- coding: utf-8 -*-
from __future__ import annotations
import os, logging
from typing import Optional, Any, Dict

import numpy as np
import pyvista as pv
from PyQt6 import uic
from PyQt6.QtWidgets import QWidget, QPushButton, QCheckBox, QFrame, QStackedWidget, QVBoxLayout
from PyQt6.QtCore import pyqtSignal

from .interactor_host import InteractorHost
from .scene_manager import SceneManager
from .views import ViewController
from .matplot2d import Matplot2DView
from .overlays import OverlayRenderer  # ausgelagerte Overlay-Logik

_LOG = logging.getLogger("app.tabs.recipe")


# --- Pfade/Utils --------------------------------------------------------------
def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    # .../src/app/tabs/recipe/coating_preview_panel -> bis Projekt-Root
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))


def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)


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


# --- Panel --------------------------------------------------------------------
class CoatingPreviewPanel(QWidget):
    # Farben: Ground < Mount < Substrat
    DEFAULT_GROUND_COLOR    = "#3a3a3a"  # dunkelgrau
    DEFAULT_MOUNT_COLOR     = "#5d5d5d"  # etwas heller
    DEFAULT_SUBSTRATE_COLOR = "#d0d6dd"  # hell

    # Signals
    pathReady = pyqtSignal(object)       # np.ndarray | None (nur Punkte)
    previewYamlReady = pyqtSignal(str)   # YAML mit xyz,rx,ry,rz

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        uic.loadUi(_ui_path("coating_preview_panel.ui"), self)

        # --- UI refs
        self.chkShowMask: QCheckBox        = self.findChild(QCheckBox, "chkShowMask")
        self.chkShowPath: QCheckBox        = self.findChild(QCheckBox, "chkShowPath")
        self.chkShowHits: QCheckBox        = self.findChild(QCheckBox, "chkShowHits")
        self.chkShowMisses: QCheckBox      = self.findChild(QCheckBox, "chkShowMisses")
        self.chkShowNormals: QCheckBox     = self.findChild(QCheckBox, "chkShowNormals")
        self.chkShowLocalFrames: QCheckBox = self.findChild(QCheckBox, "chkShowLocalFrames")

        # 3D camera buttons
        self.btnCamIso:   QPushButton = self.findChild(QPushButton, "btnCamIso")
        self.btnCamTop:   QPushButton = self.findChild(QPushButton, "btnCamTop")
        self.btnCamFront: QPushButton = self.findChild(QPushButton, "btnCamFront")
        self.btnCamBack:  QPushButton = self.findChild(QPushButton, "btnCamBack")
        self.btnCamLeft:  QPushButton = self.findChild(QPushButton, "btnCamLeft")
        self.btnCamRight: QPushButton = self.findChild(QPushButton, "btnCamRight")

        # 2D view buttons
        self.btn2DTop:    QPushButton = self.findChild(QPushButton, "btn2DTop")
        self.btn2DFront:  QPushButton = self.findChild(QPushButton, "btn2DFront")
        self.btn2DLeft:   QPushButton = self.findChild(QPushButton, "btn2DLeft")
        self.btn2DRight:  QPushButton = self.findChild(QPushButton, "btn2DRight")
        self.btn2DBack:   QPushButton = self.findChild(QPushButton, "btn2DBack")

        # stacked
        self._stack: QStackedWidget = self.findChild(QStackedWidget, "stackPreview")
        self._host3d: QFrame        = self.findChild(QFrame, "previewHost3D")
        self._host2d                 = self.findChild(QWidget, "matplotHost")
        if not self._stack or not self._host3d or not self._host2d:
            raise RuntimeError("coating_preview_panel.ui muss 'stackPreview', 'previewHost3D', 'matplotHost' enthalten.")

        # --- Composition
        self._hoster = InteractorHost(self, self._host3d)
        self.scene   = SceneManager(lambda: self._hoster.ia, None)
        self._bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

        # ViewController robust mit bounds_getter
        self.views = ViewController(
            interactor_getter=self._get_ia,
            render_callable=self.render,
            bounds_getter=lambda: self._bounds,
            cam_pad=1.8,
        )

        # 2D – robust ohne Parent beim Erzeugen (Reparenting via addWidget)
        self._mat2d = Matplot2DView(parent=None)
        lay2d = self._host2d.layout()
        if lay2d is None:
            lay2d = QVBoxLayout()
            lay2d.setContentsMargins(0, 0, 0, 0)
            self._host2d.setLayout(lay2d)
        else:
            lay2d.setContentsMargins(0, 0, 0, 0)
        if lay2d.indexOf(self._mat2d) == -1:
            lay2d.addWidget(self._mat2d)

        # Grid/Floor
        self._floor_actor = None
        self._axis_actor  = None
        self._grid_step   = 10.0

        # Layer-Namen
        self._layer_ground     = "ground"
        self._layer_mount      = "mount"
        self._layer_substrate  = "substrate"
        self._layer_mask       = "mask"
        self._layer_mask_mrk   = "mask_markers"
        self._layer_path       = "path"
        self._layer_path_mrk   = "path_markers"
        self._layer_rays_hit   = "rays_hit"
        self._layer_rays_miss  = "rays_miss"
        self._layer_normals    = "normals"
        self._frames_layers    = ("frames_x", "frames_y", "frames_z", "frames_labels")
        self._frames_layers_active = self._frames_layers

        # Label-Actors (Start/Ende etc.)
        self._label_actors: Dict[str, object] = {}

        # Defaults (Checkboxen)
        if self.chkShowMask:         self.chkShowMask.setChecked(False)
        if self.chkShowPath:         self.chkShowPath.setChecked(True)
        if self.chkShowHits:         self.chkShowHits.setChecked(False)
        if self.chkShowMisses:       self.chkShowMisses.setChecked(False)
        if self.chkShowNormals:      self.chkShowNormals.setChecked(False)
        if self.chkShowLocalFrames:  self.chkShowLocalFrames.setChecked(False)

        # ----- Overlays-Renderer mit YAML-Hook -----
        self._layers_dict: Dict[str, str] = {
            "ground":    self._layer_ground,
            "mount":     self._layer_mount,
            "substrate": self._layer_substrate,
            "mask":      self._layer_mask,
            "mask_mrk":  self._layer_mask_mrk,
            "path":      self._layer_path,
            "path_mrk":  self._layer_path_mrk,
            "rays_hit":  self._layer_rays_hit,
            "rays_miss": self._layer_rays_miss,
            "normals":   self._layer_normals,
        }

        def _set_vis(layer: str, vis: bool, render: bool = True):
            self.scene.set_layer_visible(layer, vis, render=render)

        def _update2d(mesh, path_xyz, mask_poly):
            self.update_2d_scene(substrate_mesh=mesh, path_xyz=path_xyz, mask_poly=mask_poly)

        def _yaml_out(text: str):
            # direkt an UI weitergeben
            self.previewYamlReady.emit(text)

        self.overlays = OverlayRenderer(
            add_mesh_fn=self.add_mesh,
            clear_layer_fn=self.clear_layer,
            add_path_polyline_fn=self.add_path_polyline,
            show_poly_fn=self.show_poly,
            show_frames_at_fn=self.show_frames_at,
            set_layer_visible_fn=_set_vis,
            update_2d_scene_fn=_update2d,
            layers=self._layers_dict,
            get_bounds=lambda: self._bounds,
            yaml_out_fn=_yaml_out,  # <— YAML an Panel zurück
        )

        # Checkbox-Wiring → Overlays
        if self.chkShowMask:
            self.chkShowMask.toggled.connect(lambda v: self.overlays.set_mask_visible(bool(v)))
        if self.chkShowPath:
            self.chkShowPath.toggled.connect(lambda v: self.overlays.set_path_visible(bool(v)))
        if self.chkShowHits:
            self.chkShowHits.toggled.connect(lambda v: self.overlays.set_hits_visible(bool(v)))
        if self.chkShowMisses:
            self.chkShowMisses.toggled.connect(lambda v: self.overlays.set_misses_visible(bool(v)))
        if self.chkShowNormals:
            self.chkShowNormals.toggled.connect(lambda v: self.overlays.set_normals_visible(bool(v)))
        if self.chkShowLocalFrames:
            self.chkShowLocalFrames.toggled.connect(lambda v: self.overlays.set_frames_visible(bool(v)))

        # 3D/2D Button-Wiring über ViewController
        self.views.wire_buttons_3d(
            btn_iso=self.btnCamIso,
            btn_top=self.btnCamTop,
            btn_front=self.btnCamFront,
            btn_back=self.btnCamBack,
            btn_left=self.btnCamLeft,
            btn_right=self.btnCamRight,
            on_3d=self._switch_to_3d,
        )
        self.views.wire_buttons_2d(
            btn_top=self.btn2DTop,
            btn_front=self.btn2DFront,
            btn_left=self.btn2DLeft,
            btn_right=self.btn2DRight,
            btn_back=self.btn2DBack,
            switcher=self._switch_2d_plane,
        )

        # Startmodus (3D iso)
        self._switch_to_3d()

    # -------- helpers --------
    def _get_ia(self):
        ia = self._hoster.ia
        if ia is None:
            try:
                self._hoster.ensure()
            except Exception:
                _LOG.exception("Interactor ensure() failed")
            ia = self._hoster.ia
        return ia

    # 2D plane switch
    def _switch_2d_plane(self, plane: str) -> None:
        try:
            if hasattr(self._mat2d, "set_plane"):
                self._mat2d.set_plane(plane)
            elif hasattr(self._mat2d, "_set_plane"):
                self._mat2d._set_plane(plane)
            else:
                raise AttributeError("Matplot2DView hat weder set_plane noch _set_plane")
            # Beim Wechsel direkt in 2D-Stack schalten
            if self._stack.currentIndex() != 1:
                self._stack.setCurrentIndex(1)
        except Exception:
            _LOG.exception("2D plane switch failed: %s", plane)

    # -------- Public API for hosts --------
    def preview_host(self) -> QFrame:
        return self._host3d

    def attach_interactor(self, interactor: Any) -> None:
        self._hoster.attach(interactor)

    # -------- 2D Scene Feeder --------
    def update_2d_scene(
        self,
        *,
        substrate_mesh: pv.PolyData | None,
        path_xyz: np.ndarray | None,
        mask_poly: pv.PolyData | None = None,
    ):
        self._mat2d.set_scene(
            substrate_mesh=substrate_mesh,
            path_xyz=path_xyz,
            bounds=self._bounds,
            mask_poly=mask_poly,
        )
        # Rohpfad für evtl. weitere Listener (falls gewünscht)
        try:
            self.pathReady.emit(path_xyz if path_xyz is not None else None)
        except Exception:
            _LOG.exception("pathReady emit failed")

    # -------- View switching --------
    def _switch_to_3d(self):
        if self._stack.currentIndex() != 0:
            self._stack.setCurrentIndex(0)
        self.view_isometric()

    # -------- Grid/Bounds --------
    def set_grid_step(self, step_mm: float) -> None:
        self._grid_step = max(1.0, float(step_mm))
        self._apply_bounds()

    def _apply_bounds(self) -> None:
        ia = self._get_ia()
        if ia is None:
            return
        for actor in (self._floor_actor, self._axis_actor):
            if actor is not None:
                try:
                    ia.remove_actor(actor, render=False)
                except Exception:
                    pass
        self._floor_actor = None
        self._axis_actor  = None
        self._floor_actor = self._make_floor_wire(ia, step=self._grid_step)
        self._axis_actor  = self._draw_floor_axes(ia, step=self._grid_step)

        self._look_at_floor_center()
        try:
            ia.reset_camera(bounds=self._bounds)
            ia.render()
        except Exception:
            _LOG.exception("apply_bounds: reset/render failed")

    def _floor_center(self):
        xmin, xmax, ymin, ymax, zmin, _ = self._bounds
        return np.array([(xmin + xmax) * 0.5, (ymin + ymax) * 0.5, zmin], dtype=float)

    def _look_at_floor_center(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            target = self._floor_center()
            ia.camera.focal_point = tuple(target.tolist())
        except Exception:
            pass

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
            vals = np.arange(start, vmax + 0.5 * step, step, dtype=float)
            return vals[(vals >= vmin + 1e-6) & (vals <= vmax - 1e-6)]

        xt, yt = ticks(xmin, xmax), ticks(ymin, ymax)
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)

        lines_pts, lines_idx, base = [], [], 0

        def add_seg(p0, p1):
            nonlocal base
            lines_pts.extend([p0, p1])
            lines_idx.extend([2, base, base + 1])
            base += 2

        # Achsen
        add_seg((xmin, cy, zmin), (xmax, cy, zmin))
        add_seg((cx, ymin, zmin), (cx, ymax, zmin))

        # Ticks
        tick_len = max(1.0, min(3.0, step * 0.15))
        for xv in xt:
            add_seg((xv, cy - tick_len, zmin), (xv, cy + tick_len, zmin))
        for yv in yt:
            add_seg((cx - tick_len, yv, zmin), (cx + tick_len, yv, zmin))

        actor = None
        if lines_pts:
            poly = pv.PolyData(np.asarray(lines_pts, dtype=float))
            poly.lines = np.asarray(lines_idx, dtype=np.int64)
            actor = ia.add_mesh(poly, style="wireframe", color="#5a5a5a",
                                line_width=1.0, lighting=False, render=False)

        # Achsen-Beschriftung
        label_z = zmin + max(0.5, tick_len * 0.5)
        if len(xt):
            ia.add_point_labels(
                np.c_[xt, np.full_like(xt, cy), np.full_like(xt, label_z)],
                [f"{int(v)}" if abs(v - int(v)) < 1e-6 else f"{v:.1f}" for v in xt],
                point_size=0, font_size=10, text_color="black",
                shape_opacity=0.0, render=False
            )
        if len(yt):
            ia.add_point_labels(
                np.c_[np.full_like(yt, cx), yt, np.full_like(yt, label_z)],
                [f"{int(v)}" if abs(v - int(v)) < 1e-6 else f"{v:.1f}" for v in yt],
                point_size=0, font_size=10, text_color="black",
                shape_opacity=0.0, render=False
            )
        ia.add_point_labels(
            [(xmax, cy, label_z), (cx, ymax, label_z)],
            ["X (mm)", "Y (mm)"],
            point_size=0, font_size=12, text_color="black",
            shape_opacity=0.0, render=False
        )
        return actor

    # ---------- Scene passthrough ----------
    def clear(self) -> None:
        ia = self._get_ia()
        self.scene.clear()
        # Label-Actors wegräumen
        if ia is not None:
            for act in list(self._label_actors.values()):
                try:
                    ia.remove_actor(act, render=False)
                except Exception:
                    pass
        self._label_actors.clear()

    def add_mesh(self, mesh, **kwargs) -> None:
        _ = self._get_ia()
        self.scene.add_mesh(mesh, **kwargs)

    def clear_layer(self, layer: str) -> None:
        self.scene.clear_layer(layer)

    def add_path_polyline(self, *a, **kw):
        self.scene.add_path_polyline(*a, **kw)

    # --- Poly-Helfer (für Overlays) ---
    def show_poly(self, poly: pv.PolyData, *, layer: str,
                  color: str = "royalblue", line_width: float = 2.0, lighting: bool = False):
        try:
            self.clear_layer(layer)
            self.add_mesh(
                poly,
                color=color,
                render=False,
                reset_camera=False,
                line_width=float(line_width),
                lighting=lighting,
                layer=layer,
            )
        except Exception:
            _LOG.exception("show_poly() failed")

    # --- Frames (lokale KS) ---
    def clear_frames(self) -> None:
        for lyr in self._frames_layers_active:
            try:
                self.scene.clear_layer(lyr)
            except Exception:
                pass

    def _on_toggle_frames(self, vis: bool):
        for lyr in self._frames_layers_active[:3]:
            self.scene.set_layer_visible(lyr, vis, render=False)
        self.scene.set_layer_visible(self._frames_layers_active[3], vis, render=True)

    def show_frames_at(
        self,
        *,
        origins: np.ndarray,
        z_dirs: np.ndarray,
        x_dirs: np.ndarray | None = None,
        scale: float = 10.0,
        scale_mm: float | None = None,
        tube_radius: float = 0.15,          # deutlich schlanker
        line_width: float | None = 1.0,     # Standard: Linien statt Tubes
        layer: str | None = None,
        labels: list[str] | None = None,
        clear_old: bool = True,
        add_labels: bool = False,
    ) -> None:
        ia = self._get_ia()
        if ia is None:
            _LOG.warning("show_frames_at(): kein Interactor")
            return

        # Auto-Skalierung relativ zu World-Bounds (ca. 2% des XY-Spans, min 5 mm)
        if scale_mm is None:
            xmin, xmax, ymin, ymax, *_ = self._bounds
            span_xy = max(1e-6, max(xmax - xmin, ymax - ymin))
            scale = max(5.0, 0.02 * span_xy)
        else:
            scale = float(scale_mm)

        O = np.asarray(origins, dtype=float).reshape(-1, 3)
        Z = np.asarray(z_dirs, dtype=float).reshape(-1, 3)
        if O.shape != Z.shape:
            raise ValueError("origins und z_dirs müssen gleiche Form (N,3) haben")

        X = None
        if x_dirs is not None:
            X = np.asarray(x_dirs, dtype=float).reshape(-1, 3)
            if X.shape != O.shape:
                raise ValueError("x_dirs muss Form (N,3) haben und zu origins passen")

        if layer and layer.strip():
            base = layer.strip()
            frames_layers = (f"{base}_x", f"{base}_y", f"{base}_z", f"{base}_labels")
        else:
            frames_layers = self._frames_layers
        self._frames_layers_active = frames_layers
        lx, ly, lz, llab = frames_layers

        if clear_old:
            self.clear_frames()

        if O.shape[0] == 0:
            return

        def _acc():
            return [], []

        pts_x, lns_x = _acc()
        pts_y, lns_y = _acc()
        pts_z, lns_z = _acc()
        idx_x = idx_y = idx_z = 0

        for i in range(O.shape[0]):
            o = O[i]
            if X is not None:
                z = _safe_norm(Z[i])
                x = _safe_norm(X[i])
                x = _safe_norm(np.cross(np.cross(x, z), z))
                y = _safe_norm(np.cross(z, x))
            else:
                x, y, z = _orthonormal_basis_from_z(Z[i])
            px = o + x * float(scale)
            py = o + y * float(scale)
            pz = o + z * float(scale)
            pts_x.extend([o, px]); lns_x.extend([2, idx_x, idx_x + 1]); idx_x += 2
            pts_y.extend([o, py]); lns_y.extend([2, idx_y, idx_y + 1]); idx_y += 2
            pts_z.extend([o, pz]); lns_z.extend([2, idx_z, idx_z + 1]); idx_z += 2

        def _poly_from(pts, lns):
            poly = pv.PolyData(np.asarray(pts))
            poly.lines = np.asarray(lns, dtype=np.int64).reshape(-1)
            return poly

        poly_x = _poly_from(pts_x, lns_x)
        poly_y = _poly_from(pts_y, lns_y)
        poly_z = _poly_from(pts_z, lns_z)

        # Immer dünn: Linienbreite (keine fetten Tubes)
        for lyr in (lx, ly, lz):
            self.scene.clear_layer(lyr)
        self.scene.add_mesh(poly_x, color="red",   layer=lx, line_width=float(line_width or 1.0),
                            reset_camera=False, render=False, lighting=False)
        self.scene.add_mesh(poly_y, color="green", layer=ly, line_width=float(line_width or 1.0),
                            reset_camera=False, render=False, lighting=False)
        self.scene.add_mesh(poly_z, color="blue",  layer=lz, line_width=float(line_width or 1.0),
                            reset_camera=False, render=False, lighting=False)

        if add_labels and labels:
            try:
                L = min(len(labels), O.shape[0])
                if L > 0:
                    self.scene.clear_layer(llab)
                    ia.add_point_labels(O[:L], labels[:L], point_size=0, font_size=12, shape_opacity=0.3, render=False)
            except Exception:
                _LOG.exception("Labels hinzufügen fehlgeschlagen (optional)")

    # -------- Views & render (Delegates zu ViewController) --------
    def view_isometric(self): self.views.view_isometric()
    def view_top(self):       self.views.view_top()
    def view_front(self):     self.views.view_front()
    def view_back(self):      self.views.view_back()
    def view_left(self):      self.views.view_left()
    def view_right(self):     self.views.view_right()

    def render(self, *, reset_camera: bool = True) -> None:
        ia = self._get_ia()
        if ia is None:
            return
        try:
            # Blick auf Floor-Fokus halten
            self._look_at_floor_center()
            if reset_camera:
                ia.reset_camera(bounds=self._bounds)
            ia.render()
        except Exception:
            _LOG.exception("render() failed")

    # --- Bounds setter ---
    def set_world_bounds_at(self, *, center_xy=(0.0, 0.0), z0=0.0, span_xy=240.0, span_z=240.0) -> None:
        cx, cy = float(center_xy[0]), float(center_xy[1])
        half = float(span_xy) * 0.5
        z0, span_z = float(z0), float(span_z)
        self._bounds = (cx - half, cx + half, cy - half, cy + half, z0, z0 + span_z)
        self._apply_bounds()

    def set_bounds_from_mesh(self, mesh, *, use_contact_plane=True, span_xy=240.0, span_z=240.0) -> None:
        if not hasattr(mesh, "bounds"):
            return
        xmin, xmax, ymin, ymax, zmin, zmax = mesh.bounds
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        z0 = float(zmin if use_contact_plane else 0.5 * (zmin + zmax))
        self.set_world_bounds_at(center_xy=(cx, cy), z0=z0, span_xy=span_xy, span_z=span_z)

    # --- Start/Ende-Marker (kleine Kugeln + Labels) ---
    def show_start_end_markers(self, points: np.ndarray, *, layer_prefix: str, color: str, size: float = 3.0):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            P = np.asarray(points, dtype=float).reshape(-1, 3)
            layer = f"{layer_prefix}_markers"
            self.scene.clear_layer(layer)

            # alte Labels entfernen
            old = self._label_actors.pop(layer, None)
            if old is not None:
                try:
                    ia.remove_actor(old, render=False)
                except Exception:
                    pass

            if len(P) == 0:
                return

            start = P[0]; end = P[-1]
            # kleine Kugeln
            try:
                s0 = pv.Sphere(radius=float(size)*0.5, center=start)
                s1 = pv.Sphere(radius=float(size)*0.5, center=end)
                self.scene.add_mesh(s0, color=color, layer=layer, render=False, lighting=False)
                self.scene.add_mesh(s1, color=color, layer=layer, render=False, lighting=False)
            except Exception:
                _LOG.exception("Start/Ende-Kugeln fehlgeschlagen")

            # neue Labels
            try:
                lab = ia.add_point_labels(
                    [start, end],
                    ["Start", "Ende"],
                    point_size=0, font_size=12, text_color="black",
                    shape_opacity=0.25, render=False
                )
                self._label_actors[layer] = lab
            except Exception:
                _LOG.exception("Start/Ende Labels fehlgeschlagen")
        except Exception:
            _LOG.exception("show_start_end_markers() failed")
