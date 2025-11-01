# -*- coding: utf-8 -*-
from __future__ import annotations
import os, logging
from typing import Optional, Any

import numpy as np
import pyvista as pv
from PyQt6 import uic
from PyQt6.QtWidgets import QWidget, QPushButton, QCheckBox, QFrame

from .interactor_host import InteractorHost
from .grid_manager import GridManager
from .scene_manager import SceneManager
from .views import ViewController

_LOG = logging.getLogger("app.tabs.recipe")

# ---------- Pfade ----------
def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)

# ---------- Mathe-Utils ----------
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

# ============================================================================

class CoatingPreviewPanel(QWidget):
    DEFAULT_MOUNT_COLOR = "lightgray"
    DEFAULT_SUBSTRATE_COLOR = "#3498db"

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        uic.loadUi(_ui_path("coating_preview_panel.ui"), self)

        # UI-Refs
        self.chkShowMask: QCheckBox = self.findChild(QCheckBox, "chkShowMask")
        self.chkShowRays: QCheckBox = self.findChild(QCheckBox, "chkShowRays")
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

        # Composition (kein Auto-Grid, keine Auto-Clears)
        self._hoster = InteractorHost(self, self._host)
        self.grid = GridManager(lambda: self._hoster.ia)
        self.scene = SceneManager(lambda: self._hoster.ia, None)  # kein Auto-Aufbau
        self.views = ViewController(lambda: self._hoster.ia, self.render)

        # Kamera-Buttons
        self.views.wire_buttons(
            btn_iso=self.btnCamIso, btn_top=self.btnCamTop, btn_front=self.btnCamFront,
            btn_left=self.btnCamLeft, btn_right=self.btnCamRight, btn_back=self.btnCamBack
        )

        # Layer-Namen
        self._frames_layers = ("frames_x", "frames_y", "frames_z", "frames_labels")
        self._frames_layers_active = self._frames_layers
        self._mask_layer = "mask"
        self._rays_layer = "rays"

        # Checkboxen standardmäßig aktiv
        if self.chkShowMask:         self.chkShowMask.setChecked(True)
        if self.chkShowRays:         self.chkShowRays.setChecked(True)
        if self.chkShowLocalFrames:  self.chkShowLocalFrames.setChecked(True)

        # Checkbox-Wiring → nur Sichtbarkeit togglen
        if self.chkShowLocalFrames:
            self.chkShowLocalFrames.toggled.connect(self._on_toggle_frames)
        if self.chkShowMask:
            self.chkShowMask.toggled.connect(self._on_toggle_mask)
        if self.chkShowRays:
            self.chkShowRays.toggled.connect(self._on_toggle_rays)

    # --- Hosting API ---
    def preview_host(self) -> QFrame:
        return self._host

    def attach_interactor(self, interactor: Any) -> None:
        self._hoster.attach(interactor)

    # --- Scene passthrough / Public API ---
    def clear(self) -> None:
        if self._hoster.ia is None and not self._hoster.ensure():
            return
        self.scene.clear()

    def add_mesh(self, mesh, **kwargs) -> None:
        if self._hoster.ia is None and not self._hoster.ensure():
            _LOG.warning("add_mesh(): kein Interactor")
            return
        self.scene.add_mesh(mesh, **kwargs)

    def clear_layer(self, layer: str) -> None:
        self.scene.clear_layer(layer)

    def add_path_polyline(self, *a, **kw):
        self.scene.add_path_polyline(*a, **kw)

    def add_path_markers(self, *a, **kw):
        self.scene.add_path_markers(*a, **kw)

    # --- generische Poly + Normalen-Linien ---
    def show_poly(self, poly: pv.PolyData, *, layer: str,
                  color: str = "#f39c12", line_width: float = 1.0, lighting: bool = False):
        try:
            self.clear_layer(layer)
            self.add_mesh(poly, color=color, render=False, reset_camera=False,
                          line_width=float(line_width), lighting=lighting)
        except Exception:
            _LOG.exception("show_poly() failed")

    def show_normals_from_hits(self, points_mm: np.ndarray, normals: np.ndarray,
                               *, layer: str = "normals", length_mm: float = 8.0,
                               color: str = "#27ae60", line_width: float = 1.0):
        try:
            P = np.asarray(points_mm, dtype=float).reshape(-1, 3)
            N = np.asarray(normals,   dtype=float).reshape(-1, 3)
            M = min(len(P), len(N))
            if M == 0:
                self.clear_layer(layer)
                return

            A = P[:M]
            B = P[:M] + N[:M] * float(length_mm)

            pts = np.vstack([A, B])
            lines = np.empty((M, 3), dtype=np.int64)
            lines[:, 0] = 2
            lines[:, 1] = np.arange(M, dtype=np.int64)
            lines[:, 2] = np.arange(M, dtype=np.int64) + M
            poly = pv.PolyData(pts)
            poly.lines = lines.reshape(-1)

            self.clear_layer(layer)
            self.add_mesh(poly, color=color, render=False, reset_camera=False,
                          line_width=float(line_width), lighting=False)
        except Exception:
            _LOG.exception("show_normals_from_hits() failed")

    # --- Frames (lokale Koordinatensysteme) ---
    def clear_frames(self) -> None:
        for lyr in self._frames_layers_active:
            try:
                self.scene.clear_layer(lyr)
            except Exception:
                pass

    def _on_toggle_frames(self, vis: bool):
        # nur Sichtbarkeit togglen
        for lyr in self._frames_layers_active[:3]:
            self.scene.set_layer_visible(lyr, vis, render=False)
        self.scene.set_layer_visible(self._frames_layers_active[3], vis, render=False)

    def _on_toggle_mask(self, vis: bool):
        self.scene.set_layer_visible(self._mask_layer, vis, render=False)

    def _on_toggle_rays(self, vis: bool):
        self.scene.set_layer_visible(self._rays_layer, vis, render=False)

    def show_frames_at(
        self,
        *,
        origins: np.ndarray,
        z_dirs: np.ndarray,
        x_dirs: np.ndarray | None = None,
        scale: float = 10.0,
        scale_mm: float | None = None,     # Alias für Kompatibilität
        tube_radius: float = 0.7,
        line_width: float | None = None,   # Linien oder Tubes
        layer: str | None = None,          # optional: eigener Layer-Präfix
        labels: list[str] | None = None,
        clear_old: bool = True,
        add_labels: bool = False,
    ) -> None:
        """
        Zeichnet kleine lokale Koordinatensysteme an N Positionen.
        - Mit `line_width`: als Linien (performant).
        - Ohne `line_width`: als Tubes (rund), gesteuert via `tube_radius`.
        Farben: X=rot, Y=grün, Z=blau.
        """
        if self._hoster.ia is None and not self._hoster.ensure():
            _LOG.warning("show_frames_at(): kein Interactor")
            return

        if scale_mm is not None:
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

        # Ziel-Layer (ggf. Präfix)
        if layer and isinstance(layer, str) and layer.strip():
            base = layer.strip()
            frames_layers = (f"{base}_x", f"{base}_y", f"{base}_z", f"{base}_labels")
        else:
            frames_layers = self._frames_layers
        self._frames_layers_active = frames_layers
        lx, ly, lz, llab = frames_layers

        if clear_old:
            self.clear_frames()

        N = O.shape[0]
        if N == 0:
            return

        # Sammel-PolyDatas pro Achse
        def _acc():
            return [], []

        pts_x, lns_x = _acc()
        pts_y, lns_y = _acc()
        pts_z, lns_z = _acc()
        idx_x = idx_y = idx_z = 0

        for i in range(N):
            o = O[i]
            if X is not None:
                z = _safe_norm(Z[i])
                x = _safe_norm(X[i])
                x = _safe_norm(np.cross(np.cross(x, z), z))  # orth projiziert
                y = _safe_norm(np.cross(z, x))
            else:
                x, y, z = _orthonormal_basis_from_z(Z[i])

            px = o + x * float(scale)
            py = o + y * float(scale)
            pz = o + z * float(scale)

            # X
            pts_x.extend([o, px]); lns_x.extend([2, idx_x, idx_x + 1]); idx_x += 2
            # Y
            pts_y.extend([o, py]); lns_y.extend([2, idx_y, idx_y + 1]); idx_y += 2
            # Z
            pts_z.extend([o, pz]); lns_z.extend([2, idx_z, idx_z + 1]); idx_z += 2

        def _poly_from(pts, lns):
            poly = pv.PolyData(np.asarray(pts))
            poly.lines = np.asarray(lns, dtype=np.int64).reshape(-1)
            return poly

        poly_x = _poly_from(pts_x, lns_x)
        poly_y = _poly_from(pts_y, lns_y)
        poly_z = _poly_from(pts_z, lns_z)

        # Auf Layer legen: Linien oder Tubes
        for lyr in (lx, ly, lz):
            self.scene.clear_layer(lyr)

        if line_width is not None and float(line_width) > 0.0:
            # Linienmodus
            self.scene.add_mesh(poly_x, color="red",   layer=lx,
                                line_width=float(line_width),
                                reset_camera=False, render=False, lighting=False)
            self.scene.add_mesh(poly_y, color="green", layer=ly,
                                line_width=float(line_width),
                                reset_camera=False, render=False, lighting=False)
            self.scene.add_mesh(poly_z, color="blue",  layer=lz,
                                line_width=float(line_width),
                                reset_camera=False, render=False, lighting=False)
        else:
            # Tube-Modus
            tx = poly_x.tube(radius=float(tube_radius))
            ty = poly_y.tube(radius=float(tube_radius))
            tz = poly_z.tube(radius=float(tube_radius))
            self.scene.add_mesh(tx, color="red",   layer=lx,
                                reset_camera=False, render=False, lighting=False)
            self.scene.add_mesh(ty, color="green", layer=ly,
                                reset_camera=False, render=False, lighting=False)
            self.scene.add_mesh(tz, color="blue",  layer=lz,
                                reset_camera=False, render=False, lighting=False)

        # Optional Labels
        if add_labels and labels:
            try:
                L = min(len(labels), N)
                if L > 0:
                    ia = self._hoster.ia
                    self.scene.clear_layer(llab)
                    ia.add_point_labels(
                        O[:L], labels[:L], point_size=0, font_size=12, shape_opacity=0.3
                    )
            except Exception:
                _LOG.exception("Labels hinzufügen fehlgeschlagen (optional)")

    # Views
    def view_isometric(self): self.views.view_isometric()
    def view_top(self):       self.views.view_top()
    def view_front(self):     self.views.view_front()
    def view_left(self):      self.views.view_left()
    def view_right(self):     self.views.view_right()
    def view_back(self):      self.views.view_back()

    # Render (vom Controller gerufen)
    def render(self, *, reset_camera: bool = True) -> None:
        ia = self._hoster.ia if self._hoster.ia is not None else (self._hoster.ensure() and self._hoster.ia)
        if ia is None:
            return
        try:
            if reset_camera:
                b = getattr(self.grid, "bounds", None)
                if b is not None and np.all(np.isfinite(b)):
                    ia.reset_camera(bounds=b)
                else:
                    ia.reset_camera()
            ia.render()
        except Exception:
            _LOG.exception("render() failed")
