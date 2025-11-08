# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Any, Dict

import numpy as np
import pyvista as pv
from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtWidgets import (
    QWidget, QFrame, QStackedWidget, QVBoxLayout, QHBoxLayout, QPushButton, QSizePolicy
)

from .interactor_host import InteractorHost
from .scene_manager import SceneManager
from .views import ViewController
from .matplot2d import Matplot2DView
from .overlays import OverlayRenderer

from .overlays_groupbox import OverlaysGroupBox
from .views_groupbox import ViewsGroupBox
from app.widgets.planner_groupbox import PlannerGroupBox

_LOG = logging.getLogger("app.tabs.recipe")


class CoatingPreviewPanel(QWidget):
    """
    Layout (reine Programmierung):
      [ OverlaysGroupBox (eine Zeile, horizontal) ]
      [ ViewsGroupBox  ("Kamera", 3D & 2D)       ]
      [ QStackedWidget (3D-PyVista / 2D-Matplot) ]  <- bekommt am meisten Platz
      [ PlannerGroupBox (kompakt, vertikal gequetscht) ]
      [  Validate | Optimize  ]  (HBox, Buttons gestreckt)
    """

    # Ausgänge
    pathReady = pyqtSignal(object)       # np.ndarray | None
    previewYamlReady = pyqtSignal(str)   # YAML Text

    # Aktionen
    validateRequested = pyqtSignal()
    optimizeRequested = pyqtSignal()

    # Farben für Boden/Mount/Substrat (bei Bedarf)
    DEFAULT_GROUND_COLOR    = "#3a3a3a"
    DEFAULT_MOUNT_COLOR     = "#5d5d5d"
    DEFAULT_SUBSTRATE_COLOR = "#d0d6dd"

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        # ---------- Root Layout ----------
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ---------- Overlays (eine Zeile, horizontal) ----------
        self.grpOverlays = OverlaysGroupBox(self)
        self.grpOverlays.set_defaults(
            mask=False, path=True, hits=False, misses=False, normals=False, local_frames=False
        )
        root.addWidget(self.grpOverlays, 0)  # kein Stretch

        # ---------- Views ("Kamera") ----------
        self.grpViews = ViewsGroupBox(self)  # Titel & Ausrichtung intern
        root.addWidget(self.grpViews, 0)     # kein Stretch

        # ---------- Stacked (3D/2D) ----------
        self._stack = QStackedWidget(self)
        root.addWidget(self._stack, 1)  # <- größter Stretch (1)

        # ---- 3D-Seite
        self._page3d = QWidget(self)
        v3d = QVBoxLayout(self._page3d)
        v3d.setContentsMargins(0, 0, 0, 0)
        v3d.setSpacing(0)

        self._host3d = QFrame(self._page3d)            # Container für Interactor
        vhost = QVBoxLayout(self._host3d)
        vhost.setContentsMargins(0, 0, 0, 0)
        vhost.setSpacing(0)
        v3d.addWidget(self._host3d)

        self._stack.addWidget(self._page3d)

        # ---- 2D-Seite
        self._page2d = QWidget(self)
        v2d = QVBoxLayout(self._page2d)
        v2d.setContentsMargins(0, 0, 0, 0)
        v2d.setSpacing(0)

        self._host2d = QWidget(self._page2d)
        vmat = QVBoxLayout(self._host2d)
        vmat.setContentsMargins(0, 0, 0, 0)
        vmat.setSpacing(0)
        v2d.addWidget(self._host2d)

        self._stack.addWidget(self._page2d)

        # ---------- PyVista-Interactor + Scene ----------
        self._hoster = InteractorHost(self, self._host3d)
        # SceneManager benötigt callable für interactor und ggf. Poly-/Mesh-Handling
        self.scene = SceneManager(lambda: self._hoster.ia, None)

        # Welt-/View-Bounds (mm): xmin, xmax, ymin, ymax, zmin, zmax
        self._bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

        # View-Controller (steuert Kamera-Buttons)
        self.views = ViewController(
            interactor_getter=self._get_ia,
            render_callable=self.render,
            bounds_getter=lambda: self._bounds,
            cam_pad=1.8,
        )

        # ---------- 2D-View (Matplotlib) ----------
        self._mat2d = Matplot2DView(parent=None)
        self._host2d.layout().addWidget(self._mat2d)

        # ---------- Grid/Floor & Labels ----------
        self._floor_actor = None
        self._axis_actor = None
        self._grid_step = 10.0
        self._axis_label_actors: list = []
        self._label_actors: Dict[str, object] = {}

        # ---------- OverlayRenderer ----------
        layers = {
            "ground": "ground",
            "mount": "mount",
            "substrate": "substrate",
            "mask": "mask",
            "mask_mrk": "mask_markers",
            "path": "path",
            "path_mrk": "path_markers",
            "rays_hit": "rays_hit",
            "rays_miss": "rays_miss",
            "normals": "normals",
        }
        self.overlays = OverlayRenderer(
            add_mesh_fn=self.add_mesh,
            clear_layer_fn=self.clear_layer,
            add_path_polyline_fn=self.add_path_polyline,
            show_poly_fn=self.show_poly,
            show_frames_at_fn=self.show_frames_at,
            set_layer_visible_fn=lambda layer, vis, render=True: self.scene.set_layer_visible(layer, vis, render=render),
            update_2d_scene_fn=lambda mesh, path_xyz, mask_poly: self.update_2d_scene(
                substrate_mesh=mesh, path_xyz=path_xyz, mask_poly=mask_poly
            ),
            layers=layers,
            get_bounds=lambda: self._bounds,
            yaml_out_fn=lambda text: self.previewYamlReady.emit(text),
        )

        # ---------- Wire: Overlay-Toggles ----------
        self.grpOverlays.maskToggled.connect(lambda v: self.overlays.set_mask_visible(bool(v)))
        self.grpOverlays.pathToggled.connect(lambda v: self.overlays.set_path_visible(bool(v)))
        self.grpOverlays.hitsToggled.connect(lambda v: self.overlays.set_hits_visible(bool(v)))
        self.grpOverlays.missesToggled.connect(lambda v: self.overlays.set_misses_visible(bool(v)))
        self.grpOverlays.normalsToggled.connect(lambda v: self.overlays.set_normals_visible(bool(v)))
        self.grpOverlays.localFramesToggled.connect(lambda v: self.overlays.set_frames_visible(bool(v)))

        # ---------- Wire: View-Buttons ----------
        # 3D
        self.views.wire_buttons_3d(
            btn_iso=self.grpViews.btnCamIso,
            btn_top=self.grpViews.btnCamTop,
            btn_front=self.grpViews.btnCamFront,
            btn_back=self.grpViews.btnCamBack,
            btn_left=self.grpViews.btnCamLeft,
            btn_right=self.grpViews.btnCamRight,
            on_3d=self._switch_to_3d,
        )
        # 2D
        self.views.wire_buttons_2d(
            btn_top=self.grpViews.btn2DTop,
            btn_front=self.grpViews.btn2DFront,
            btn_left=self.grpViews.btn2DLeft,
            btn_right=self.grpViews.btn2DRight,
            btn_back=self.grpViews.btn2DBack,
            switcher=self._switch_2d_plane,
        )

        # ---------- Planner (kompakt) ----------
        self.plannerBox = PlannerGroupBox(parent=self)
        self.plannerBox.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Maximum)
        self.plannerBox.setMaximumHeight(220)  # vertikal gequetscht
        # Margins des inneren Layouts verkleinern, falls vorhanden
        if self.plannerBox.layout() is not None:
            self.plannerBox.layout().setContentsMargins(6, 6, 6, 6)
        root.addWidget(self.plannerBox, 0)

        # ---------- Validate/Optimize (HBox, gestreckt) ----------
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)

        self.btnValidate = QPushButton("Validate", self)
        self.btnOptimize = QPushButton("Optimize", self)
        for b in (self.btnValidate, self.btnOptimize):
            b.setAutoDefault(False)
            b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            row.addWidget(b, 1)  # gleichmäßig strecken

        root.addLayout(row)

        # Buttons → externe Signals
        self.btnValidate.clicked.connect(self.validateRequested.emit)
        self.btnOptimize.clicked.connect(self.optimizeRequested.emit)

        # ---------- Initial: 3D Iso ----------
        self._switch_to_3d()

    # =================== Internals / helpers ===================

    def _get_ia(self):
        ia = self._hoster.ia
        if ia is None:
            try:
                self._hoster.ensure()
            except Exception:
                _LOG.exception("Interactor ensure() failed")
            ia = self._hoster.ia
        return ia

    def _switch_to_3d(self):
        if self._stack.currentIndex() != 0:
            self._stack.setCurrentIndex(0)
        self.view_isometric()

    def _switch_2d_plane(self, plane: str) -> None:
        """Wechselt auf die 2D-Seite und setzt die gewünschte Projektion."""
        try:
            if hasattr(self._mat2d, "set_plane"):
                self._mat2d.set_plane(plane)
            elif hasattr(self._mat2d, "_set_plane"):
                # Fallback für Legacy
                self._mat2d._set_plane(plane)  # noqa: SLF001
            else:
                raise AttributeError("Matplot2DView hat weder set_plane noch _set_plane")
            if self._stack.currentIndex() != 1:
                self._stack.setCurrentIndex(1)
        except Exception:
            _LOG.exception("2D plane switch failed: %s", plane)

    # =================== Public API (Hosts) ===================

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
        mask_poly: pv.PolyData | None = None
    ):
        """Füttert Matplot2D mit Mesh/Path/Mask und übernimmt Bounds."""
        self._mat2d.set_scene(
            substrate_mesh=substrate_mesh,
            path_xyz=path_xyz,
            bounds=self._bounds,
            mask_poly=mask_poly,
        )
        try:
            self.pathReady.emit(path_xyz if path_xyz is not None else None)
        except Exception:
            _LOG.exception("pathReady emit failed")

    # -------- Views (Delegates) --------
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
            self._look_at_floor_center()
            if reset_camera:
                ia.reset_camera(bounds=self._bounds)
            ia.render()
        except Exception:
            _LOG.exception("render() failed")

    # -------- Grid/Bounds --------
    def set_grid_step(self, step_mm: float) -> None:
        self._grid_step = max(1.0, float(step_mm))
        self._apply_bounds()

    def set_world_bounds_at(
        self, *, center_xy=(0.0, 0.0), z0=0.0, span_xy=240.0, span_z=240.0
    ) -> None:
        cx, cy = float(center_xy[0]), float(center_xy[1])
        half = float(span_xy) * 0.5
        z0, span_z = float(z0), float(span_z)
        self._bounds = (cx - half, cx + half, cy - half, cy + half, z0, z0 + span_z)
        self._apply_bounds()

    def set_bounds_from_mesh(
        self, mesh, *, use_contact_plane=True, span_xy=240.0, span_z=240.0
    ) -> None:
        if not hasattr(mesh, "bounds"):
            return
        xmin, xmax, ymin, ymax, zmin, zmax = mesh.bounds
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        z0 = float(zmin if use_contact_plane else 0.5 * (zmin + zmax))
        self.set_world_bounds_at(center_xy=(cx, cy), z0=z0, span_xy=span_xy, span_z=span_z)

    def _apply_bounds(self) -> None:
        ia = self._get_ia()
        if ia is None:
            return

        # alte Boden-/Achsen-Actors entfernen
        for actor in (self._floor_actor, self._axis_actor):
            if actor is not None:
                try:
                    ia.remove_actor(actor, render=False)
                except Exception:
                    pass
        self._floor_actor = None
        self._axis_actor = None

        # alte Label-Actors entfernen
        if self._axis_label_actors:
            for lab in self._axis_label_actors:
                try:
                    ia.remove_actor(lab, render=False)
                except Exception:
                    pass
            self._axis_label_actors = []

        # neu aufbauen
        self._floor_actor = self._make_floor_wire(ia, step=self._grid_step)
        self._axis_actor = self._draw_floor_axes(ia, step=self._grid_step)

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
        width, height = xmax - xmin, ymax - ymin
        i_res = max(1, int(round(width / max(1e-6, step))))
        j_res = max(1, int(round(height / max(1e-6, step))))
        center = ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, zmin)
        plane = pv.Plane(
            center=center, direction=(0, 0, 1),
            i_size=width, j_size=height,
            i_resolution=i_res, j_resolution=j_res
        )
        return ia.add_mesh(
            plane, style="wireframe", color="#cfcfcf",
            line_width=1.0, lighting=False, render=False
        )

    def _draw_floor_axes(self, ia, step: float = 10.0):
        xmin, xmax, ymin, ymax, zmin, _ = self._bounds
        step = max(1.0, float(step))

        def ticks(vmin, vmax):
            import numpy as _np
            start = _np.ceil(vmin / step) * step
            vals = _np.arange(start, vmax + 0.5 * step, step, dtype=float)
            return vals[(vals >= vmin + 1e-6) & (vals <= vmax - 1e-6)]

        xt, yt = ticks(xmin, xmax), ticks(ymin, ymax)
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)

        pts, lines = [], []
        base = 0

        def add_seg(p0, p1):
            nonlocal base
            pts.extend([p0, p1])
            lines.extend([2, base, base + 1])
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
        if pts:
            poly = pv.PolyData(np.asarray(pts, dtype=float))
            poly.lines = np.asarray(lines, dtype=np.int64)
            actor = ia.add_mesh(
                poly, style="wireframe", color="#5a5a5a",
                line_width=1.0, lighting=False, render=False
            )

        # Labels
        new_labels: list = []
        label_z = zmin + max(0.5, tick_len * 0.5)
        if len(xt):
            labx = ia.add_point_labels(
                np.c_[xt, np.full_like(xt, cy), np.full_like(xt, label_z)],
                [f"{int(v)}" if abs(v - int(v)) < 1e-6 else f"{v:.1f}" for v in xt],
                point_size=0, font_size=10, text_color="black",
                shape_opacity=0.0, render=False
            )
            new_labels.append(labx)
        if len(yt):
            laby = ia.add_point_labels(
                np.c_[np.full_like(yt, cx), yt, np.full_like(yt, label_z)],
                [f"{int(v)}" if abs(v - int(v)) < 1e-6 else f"{v:.1f}" for v in yt],
                point_size=0, font_size=10, text_color="black",
                shape_opacity=0.0, render=False
            )
            new_labels.append(laby)
        labxy = ia.add_point_labels(
            [(xmax, cy, label_z), (cx, ymax, label_z)],
            ["X (mm)", "Y (mm)"],
            point_size=0, font_size=12, text_color="black",
            shape_opacity=0.0, render=False
        )
        new_labels.append(labxy)

        self._axis_label_actors = new_labels
        return actor

    # -------- Scene passthrough --------
    def clear(self) -> None:
        ia = self._get_ia()
        self.scene.clear()
        if ia is not None:
            for act in list(self._label_actors.values()):
                try:
                    ia.remove_actor(act, render=False)
                except Exception:
                    pass
        self._label_actors.clear()
        if ia is not None and self._axis_label_actors:
            for lab in self._axis_label_actors:
                try:
                    ia.remove_actor(lab, render=False)
                except Exception:
                    pass
            self._axis_label_actors = []

    def add_mesh(self, mesh, **kwargs) -> None:
        _ = self._get_ia()
        self.scene.add_mesh(mesh, **kwargs)

    def clear_layer(self, layer: str) -> None:
        self.scene.clear_layer(layer)

    def add_path_polyline(self, *a, **kw):
        self.scene.add_path_polyline(*a, **kw)

    # --- Poly helper for Overlays ---
    def show_poly(
        self,
        poly: pv.PolyData,
        *,
        layer: str,
        color: str = "royalblue",
        line_width: float = 2.0,
        lighting: bool = False
    ):
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

    # --- Frames ---
    def clear_frames(self) -> None:
        for lyr in ("frames_x", "frames_y", "frames_z", "frames_labels"):
            try:
                self.scene.clear_layer(lyr)
            except Exception:
                pass

    def show_frames_at(
        self,
        *,
        origins: np.ndarray,
        z_dirs: np.ndarray,
        x_dirs: np.ndarray | None = None,
        scale: float = 10.0,
        scale_mm: float | None = None,
        tube_radius: float = 0.15,
        line_width: float | None = 1.0,
        layer: str | None = None,
        labels: list[str] | None = None,
        clear_old: bool = True,
        add_labels: bool = False,
    ) -> None:
        ia = self._get_ia()
        if ia is None:
            _LOG.warning("show_frames_at(): kein Interactor")
            return

        # dynamische Layernamen
        if layer and layer.strip():
            base = layer.strip()
            lx, ly, lz, llab = (f"{base}_x", f"{base}_y", f"{base}_z", f"{base}_labels")
        else:
            lx, ly, lz, llab = ("frames_x", "frames_y", "frames_z", "frames_labels")

        if clear_old:
            for lyr in (lx, ly, lz, llab):
                try:
                    self.scene.clear_layer(lyr)
                except Exception:
                    pass

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
            scale = float(scale_mm)
        else:
            xmin, xmax, ymin, ymax, *_ = self._bounds
            span_xy = max(1e-6, max(xmax - xmin, ymax - ymin))
            scale = max(5.0, 0.02 * span_xy)

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
                x = _safe_norm(np.cross(np.cross(x, z), z))  # orthogonalisieren
                y = _safe_norm(np.cross(z, x))
            else:
                x, y, z = _basis_from_z(Z[i])

            px, py, pz = o + x * scale, o + y * scale, o + z * scale
            pts_x.extend([o, px]); lns_x.extend([2, ix, ix + 1]); ix += 2
            pts_y.extend([o, py]); lns_y.extend([2, iy, iy + 1]); iy += 2
            pts_z.extend([o, pz]); lns_z.extend([2, iz, iz + 1]); iz += 2

        def _poly(pts, lns):
            poly = pv.PolyData(np.asarray(pts))
            poly.lines = np.asarray(lns, dtype=np.int64).reshape(-1)
            return poly

        self.scene.add_mesh(
            _poly(pts_x, lns_x),
            color="red", layer=lx, line_width=float(line_width or 1.0),
            reset_camera=False, render=False, lighting=False
        )
        self.scene.add_mesh(
            _poly(pts_y, lns_y),
            color="green", layer=ly, line_width=float(line_width or 1.0),
            reset_camera=False, render=False, lighting=False
        )
        self.scene.add_mesh(
            _poly(pts_z, lns_z),
            color="blue", layer=lz, line_width=float(line_width or 1.0),
            reset_camera=False, render=False, lighting=False
        )

        if add_labels and labels:
            try:
                L = min(len(labels), O.shape[0])
                if L > 0:
                    self.scene.clear_layer(llab)
                    ia.add_point_labels(
                        O[:L], labels[:L],
                        point_size=0, font_size=12, shape_opacity=0.3, render=False
                    )
            except Exception:
                _LOG.exception("Labels hinzufügen fehlgeschlagen (optional)")

    # --- Start/Ende-Marker ---
    def show_start_end_markers(
        self, points: np.ndarray, *, layer_prefix: str, color: str, size: float = 3.0
    ):
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

            start = P[0]
            end = P[-1]
            try:
                s0 = pv.Sphere(radius=float(size) * 0.5, center=start)
                s1 = pv.Sphere(radius=float(size) * 0.5, center=end)
                self.scene.add_mesh(s0, color=color, layer=layer, render=False, lighting=False)
                self.scene.add_mesh(s1, color=color, layer=layer, render=False, lighting=False)
            except Exception:
                _LOG.exception("Start/Ende-Kugeln fehlgeschlagen")

            try:
                lab = ia.add_point_labels(
                    [start, end], ["Start", "Ende"],
                    point_size=0, font_size=12, text_color="black",
                    shape_opacity=0.25, render=False
                )
                self._label_actors[layer] = lab
            except Exception:
                _LOG.exception("Start/Ende Labels fehlgeschlagen")
        except Exception:
            _LOG.exception("show_start_end_markers() failed")
