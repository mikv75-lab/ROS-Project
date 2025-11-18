# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Any, Dict, Tuple, List

import numpy as np
from PyQt6.QtCore import pyqtSignal, QTimer
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QSizePolicy, QSpacerItem
from PyQt6.sip import isdeleted  # Guard gegen zerstörte Qt-Objekte

from app.model.recipe.recipe import Recipe
from .views_3d.scene_manager import SceneManager
from .views_2d.matplot2d import Matplot2DView
from .overlays_groupbox import OverlaysGroupBox
from app.widgets.info_groupbox import InfoGroupBox
from .views_2d_box import Views2DBox
from .views_3d_box import Views3DBox

_LOG = logging.getLogger("app.tabs.recipe.preview.panel")


def _set_policy(w: QWidget,
                *,
                h: QSizePolicy.Policy = QSizePolicy.Policy.Expanding,
                v: QSizePolicy.Policy = QSizePolicy.Policy.Preferred) -> None:
    sp = w.sizePolicy()
    sp.setHorizontalPolicy(h)
    sp.setVerticalPolicy(v)
    w.setSizePolicy(sp)


class CoatingPreviewPanel(QWidget):
    """
    Layout:
      Info
      HBox:
        - Left VBox:  Views2D + Matplotlib(Expanding) + Spacer
        - Right VBox: Views3D + Overlays + PyVistaHost(Expanding)

    Info-Quelle:
      - Recipe.compile_poses(...) schreibt Basiswerte in recipe.info:
          total_points, total_length_mm, sides{...}
      - Dieses Panel ergänzt:
          eta_s, medium_ml, mesh_tris, mesh_bounds
      - InfoGroupBox bekommt immer recipe.info übergeben.
    """
    sprayPathSetRequested = pyqtSignal(object)
    interactorReady = pyqtSignal()  # signalisiert, dass der Interactor verfügbar ist

    def __init__(self, *, ctx, store=None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.store = store

        # Lifecycle-Guards
        self._dying: bool = False
        self.destroyed.connect(lambda *_: setattr(self, "_dying", True))

        # Fester Interactor: wird von außen gesetzt
        self._interactor: Any = None
        self._retry_left: int = 10
        self._retry_delay_ms: int = 50

        # Sichtbarkeits-State unabhängig von UI-Objekten
        self._vis: Dict[str, bool] = {
            "path": True, "hits": True, "misses": True, "normals": False, "frames": False
        }

        # Root
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # Info
        self.grpInfo = InfoGroupBox(self)
        _set_policy(self.grpInfo, h=QSizePolicy.Expanding, v=QSizePolicy.Preferred)
        root.addWidget(self.grpInfo, 0)

        # Split: left 2D, right 3D
        split = QHBoxLayout()
        split.setContentsMargins(0, 0, 0, 0)
        split.setSpacing(8)
        root.addLayout(split, 1)

        # Left
        vleft = QVBoxLayout()
        vleft.setContentsMargins(0, 0, 0, 0)
        vleft.setSpacing(6)
        split.addLayout(vleft, 1)

        # Matplotlib zuerst erzeugen
        self._mat2d = Matplot2DView(parent=self)
        _set_policy(self._mat2d, h=QSizePolicy.Expanding, v=QSizePolicy.Expanding)
        try:
            tb = self._mat2d.make_toolbar(self)
            if tb is not None:
                self._mat2d_toolbar = tb  # Referenz halten (optional)
                vleft.addWidget(tb, 0)
        except Exception:
            pass
        vleft.addWidget(self._mat2d, 1)

        # Spacer unter dem Plot
        vleft.addSpacerItem(QSpacerItem(
            0, 0,
            QSizePolicy.Policy.Minimum,
            QSizePolicy.Policy.Expanding
        ))

        # Beim Zerstören: Canvas sauber entsorgen
        self.destroyed.connect(lambda *_: getattr(self._mat2d, "dispose", lambda: None)())

        # Danach die 2D-Controls, die _mat2d.refresh nutzen
        self.views2d = Views2DBox(
            switch_2d=self._switch_2d_plane,
            refresh_callable=self._mat2d.refresh,
            get_bounds=self.get_bounds,
            set_bounds=self.set_bounds,
            parent=self
        )
        _set_policy(self.views2d, h=QSizePolicy.Expanding, v=QSizePolicy.Preferred)
        # Controls vor den Plot (oberhalb)
        vleft.insertWidget(0, self.views2d, 0)

        # Right
        vright = QVBoxLayout()
        vright.setContentsMargins(0, 0, 0, 0)
        vright.setSpacing(6)
        split.addLayout(vright, 1)

        self.views3d = Views3DBox(
            interactor_getter=lambda: self._interactor,
            render_callable=self.render,
            bounds_getter=self.get_bounds,
            substrate_bounds_getter=lambda: self.scene.get_layer_bounds("substrate"),
            cam_pad=1.0,
            iso_extra_zoom=3.0,
            parent=self,
        )
        _set_policy(self.views3d, h=QSizePolicy.Expanding, v=QSizePolicy.Policy.Preferred)
        vright.addWidget(self.views3d, 0)

        self.grpOverlays = OverlaysGroupBox(
            self,
            add_mesh_fn=self.add_mesh,
            clear_layer_fn=self.clear_layer,
            add_path_polyline_fn=self.add_path_polyline,
            show_poly_fn=self.show_poly,
            show_frames_at_fn=self.show_frames_at,
            set_layer_visible_fn=lambda layer, vis, render=True: self.scene.set_layer_visible(layer, vis),
            update_2d_scene_fn=lambda mesh, path_xyz, _mask_poly: self.update_2d_scene(
                substrate_mesh=mesh, path_xyz=path_xyz
            ),
            layers={
                "ground": "ground",
                "mount": "mount",
                "substrate": "substrate",
                "path": "path",
                "path_mrk": "path_markers",
                "rays_hit": "rays_hit",
                "rays_miss": "rays_miss",
                "normals": "normals",
                "frames_x": "frames_x",
                "frames_y": "frames_y",
                "frames_z": "frames_z",
            },
            get_bounds=self.get_bounds,
        )
        _set_policy(self.grpOverlays, h=QSizePolicy.Expanding, v=QSizePolicy.Preferred)
        vright.addWidget(self.grpOverlays, 0)

        self._pvHost = QWidget(self)
        self._pvHost.setObjectName("pvHost")
        _set_policy(self._pvHost, h=QSizePolicy.Expanding, v=QSizePolicy.Expanding)
        vright.addWidget(self._pvHost, 1)

        # Scene (nutzt nur den injizierten Interactor)
        self.scene = SceneManager(interactor_getter=lambda: self._interactor)

        # Default-Bounds
        self._bounds: Tuple[float, float, float, float, float, float] = (
            -120.0, 120.0, -120.0, 120.0, 0.0, 240.0
        )

        QTimer.singleShot(0, self._push_initial_visibility)

    # ---------- Public API ----------
    def get_pv_host(self) -> QWidget:
        return self._pvHost

    def set_interactor(self, ia) -> None:
        self._interactor = ia
        try:
            if self._interactor is not None and hasattr(self._interactor, "render"):
                self._interactor.render()  # init Camera/Renderer
        except Exception:
            pass
        self.interactorReady.emit()

    # ---------- Preview Pipeline ----------
    def handle_update_preview(self, model: Recipe) -> None:
        if self._dying or isdeleted(self):
            return

        ia = self._interactor
        if ia is None:
            if self._retry_left > 0:
                self._retry_left -= 1
                _LOG.warning("handle_update_preview: no interactor -> retry (%d left)", self._retry_left)
                QTimer.singleShot(self._retry_delay_ms, lambda m=model: self.handle_update_preview(m))
            else:
                _LOG.error("handle_update_preview: no interactor (give up)")
                self._set_info_defaults()
            return
        self._retry_left = 10

        cam_snap = self.snapshot_camera()

        # 1) Szene
        try:
            scene = self.scene.build_scene(self.ctx, model, grid_step_mm=10.0)
        except Exception:
            _LOG.exception("build_scene failed")
            self._set_info_defaults()
            return

        # 2) Bounds für 2D
        self.set_bounds(scene.bounds)

        # 3) compile_poses → path_xyz (und Basis-Info direkt in model.info)
        stand_off = 10.0
        try:
            p = getattr(model, "parameters", {}) or {}
            stand_off = float(p.get("stand_off_mm", 10.0))
        except Exception:
            pass

        compiled = None
        try:
            if hasattr(model, "compile_poses") and callable(getattr(model, "compile_poses")):
                b = scene.substrate_mesh.bounds if scene.substrate_mesh is not None else scene.bounds
                compiled = model.compile_poses(
                    bounds=b,
                    sides=None,
                    stand_off_mm=stand_off,
                    tool_frame=None
                )
                # Recipe.compile_poses aktualisiert bereits model.info mit Basiswerten
        except Exception:
            _LOG.exception("compile_poses failed")

        path_xyz = None
        if isinstance(compiled, dict):
            try:
                for _side, data in (compiled.get("sides", {}) or {}).items():
                    poses = (data or {}).get("poses_quat", []) or []
                    if poses:
                        path_xyz = np.array(
                            [[p["x"], p["y"], p["z"]] for p in poses],
                            dtype=float
                        )
                        break
            except Exception:
                _LOG.exception("extract path from compiled failed")

        # 4) 2D-View
        try:
            self.update_2d_scene(
                substrate_mesh=scene.substrate_mesh or scene.ground_mesh,
                path_xyz=path_xyz
            )
        except Exception:
            _LOG.exception("update_2d_scene in handle_update_preview failed")

        # 5) Overlays
        try:
            self.scene.build_overlays(
                path_xyz=path_xyz,
                normals_xyz=None,
                frames_at=None,
                rays_hit=None,
                rays_miss=None,
                as_tube=True,
                tube_radius=0.8,
            )
        except Exception:
            _LOG.exception("build_overlays failed")

        # 6) Sichtbarkeit
        try:
            self.scene.toggle_overlays(dict(self._vis))
        except Exception:
            _LOG.exception("toggle_overlays failed")

        # 7) Info in recipe.info ergänzen (ETA, Medium, Mesh) und an InfoBox geben
        try:
            self._update_recipe_info_with_runtime(scene=scene, model=model, compiled=compiled)
            if not self._dying and not isdeleted(self.grpInfo) and self.grpInfo is not None:
                self.grpInfo.set_values(getattr(model, "info", None))
        except Exception:
            _LOG.exception("update recipe.info / set info failed")
            self._set_info_defaults()

        # 8) Kamera & Render
        try:
            if cam_snap:
                self.restore_camera(cam_snap)
        except Exception:
            _LOG.exception("restore_camera (optional) failed")

        self.scene.update_current_views_once(refresh_2d=self._mat2d.refresh)

    # ---------- Info helpers ----------
    def _set_info_defaults(self) -> None:
        try:
            if not self._dying and not isdeleted(self.grpInfo) and self.grpInfo is not None:
                self.grpInfo.set_values(None)
        except Exception:
            pass

    def _update_recipe_info_with_runtime(self, *, scene, model: Recipe, compiled: Any) -> None:
        """
        Ergänzt recipe.info um:
          - points / total_points
          - length_mm / total_length_mm
          - eta_s
          - medium_ml
          - mesh_tris
          - mesh_bounds (L,B,H in mm)

        Basiswerte (total_points, total_length_mm, sides) kommen aus Recipe.compile_poses().
        Falls diese fehlen, werden sie hier aus `compiled` nachgezogen.
        """
        info = getattr(model, "info", None) or {}
        if not isinstance(info, dict):
            info = {}

        # --- Basis: Points / Length aus info oder compiled ---
        total_points = info.get("total_points")
        total_length_mm = info.get("total_length_mm")

        # Falls nötig: aus compiled nachziehen
        if (total_points is None or total_length_mm is None) and isinstance(compiled, dict):
            points_total = 0
            length_mm = 0.0
            try:
                for _side, data in (compiled.get("sides", {}) or {}).items():
                    poses: List[Dict[str, float]] = (data or {}).get("poses_quat", []) or []
                    n = len(poses)
                    points_total += n
                    if n >= 2:
                        P = np.array(
                            [[p["x"], p["y"], p["z"]] for p in poses],
                            dtype=float
                        ).reshape(-1, 3)
                        length_mm += float(np.sum(np.linalg.norm(P[1:] - P[:-1], axis=1)))
            except Exception:
                _LOG.exception("fallback base-info from compiled failed")
            if total_points is None:
                total_points = points_total
            if total_length_mm is None:
                total_length_mm = length_mm

        # In info schreiben (falls noch nicht drin)
        if total_points is not None:
            info.setdefault("total_points", total_points)
            info.setdefault("points", total_points)
        if total_length_mm is not None:
            info.setdefault("total_length_mm", total_length_mm)
            info.setdefault("length_mm", total_length_mm)

        # --- ETA und Medium ---
        params = getattr(model, "parameters", {}) or {}
        speed = float(params.get("speed_mm_s", 0.0)) if isinstance(params, dict) else 0.0
        flow_ml_min = float(params.get("flow_ml_min", 0.0)) if isinstance(params, dict) else 0.0

        length_val = float(info.get("length_mm") or info.get("total_length_mm") or 0.0)

        if length_val > 0.0 and speed > 1e-9:
            eta_s = length_val / speed
        else:
            eta_s = None

        if eta_s is not None and flow_ml_min > 0.0:
            medium_ml = (flow_ml_min / 60.0) * eta_s
        else:
            medium_ml = None

        info["eta_s"] = eta_s
        info["medium_ml"] = medium_ml

        # --- Mesh-Infos ---
        mesh_tris = getattr(scene, "mesh_tris", None)
        b = getattr(scene, "bounds", None)
        if getattr(scene, "substrate_mesh", None) is not None:
            try:
                b = scene.substrate_mesh.bounds
            except Exception:
                pass

        mesh_bounds = None
        if b is not None:
            try:
                xmin, xmax, ymin, ymax, zmin, zmax = b
                mesh_bounds = (
                    float(xmax - xmin),
                    float(ymax - ymin),
                    float(zmax - zmin),
                )
            except Exception:
                mesh_bounds = None

        info["mesh_tris"] = mesh_tris
        if mesh_bounds is not None:
            info["mesh_bounds"] = mesh_bounds

        # Zurück in Recipe
        model.info = info

    # ---------- Helpers ----------
    def _push_initial_visibility(self) -> None:
        try:
            self.scene.toggle_overlays(dict(self._vis))
            self.scene.update_current_views_once(refresh_2d=self._mat2d.refresh)
        except Exception:
            _LOG.exception("Initial visibility push failed")

    def _ia(self):
        return self._interactor

    def render(self) -> None:
        ia = self._ia()
        try:
            if ia is not None and hasattr(ia, "render"):
                ia.render()
        except Exception:
            _LOG.exception("render failed")

    def snapshot_camera(self) -> Dict[str, Any] | None:
        ia = self._ia()
        if ia is None or not hasattr(ia, "camera"):
            return None
        cam = ia.camera
        try:
            return {
                "position": tuple(cam.position),
                "focal_point": tuple(cam.focal_point),
                "view_up": tuple(cam.up),
                "clipping_range": tuple(cam.clipping_range),
            }
        except Exception:
            return None

    def restore_camera(self, snap: Dict[str, Any] | None) -> None:
        if not snap:
            return
        ia = self._ia()
        if ia is None or not hasattr(ia, "camera"):
            return
        try:
            cam = ia.camera
            cam.position = snap.get("position", cam.position)
            cam.focal_point = snap.get("focal_point", cam.focal_point)
            cam.up = snap.get("view_up", cam.up)
            cr = snap.get("clipping_range", None)
            if cr:
                cam.clipping_range = cr
        except Exception:
            _LOG.exception("restore_camera failed")

    def refresh_all_views(self) -> None:
        ia = self._ia()
        try:
            if ia is not None and hasattr(ia, "reset_camera_clipping_range"):
                ia.reset_camera_clipping_range()
            if ia is not None and hasattr(ia, "render"):
                ia.render()
            self._mat2d.refresh()
        except Exception:
            _LOG.exception("refresh_all_views failed")

    # Bounds
    def set_bounds(self, bounds: Tuple[float, float, float, float, float, float]) -> None:
        self._bounds = tuple(map(float, bounds))
        try:
            self._mat2d.set_bounds(self._bounds)
        except Exception:
            _LOG.exception("set_bounds -> 2D failed")

    def get_bounds(self) -> Tuple[float, float, float, float, float, float]:
        return self._bounds

    # 2D
    def _switch_2d_plane(self, plane: str) -> None:
        try:
            self._mat2d.set_plane(plane)
            self._mat2d.refresh()
        except Exception:
            _LOG.exception("2D plane switch failed: %s", plane)

    def update_2d_scene(self, *, substrate_mesh=None, path_xyz: np.ndarray | None):
        """2D zeigt Substrat + optional Pfad (ohne Maske)."""
        try:
            self._mat2d.set_scene(
                substrate_mesh=substrate_mesh,
                path_xyz=None if path_xyz is None else np.asarray(path_xyz, float).reshape(-1, 3),
                bounds=self._bounds,
            )
            self._mat2d.refresh()
        except Exception:
            _LOG.exception("update_2d_scene failed")

    # -------- Passthroughs zum SceneManager ---------------------------------
    def clear(self) -> None:
        self.scene.clear()

    def add_mesh(self, mesh, **kwargs) -> None:
        self.scene.add_mesh(mesh, **kwargs)

    def clear_layer(self, layer: str) -> None:
        self.scene.clear_layer(layer)

    def add_path_polyline(self, *a, **kw):
        self.scene.add_path_polyline(*a, **kw)

    def show_poly(self, poly, *, layer: str,
                  color: str = "royalblue",
                  line_width: float = 2.0,
                  lighting: bool = False):
        try:
            self.clear_layer(layer)
            self.add_mesh(poly, color=color, line_width=float(line_width),
                          lighting=lighting, layer=layer)
        except Exception:
            _LOG.exception("show_poly() failed")

    def show_frames_at(self, **_kwargs) -> None:
        # Noch nicht implementiert
        pass
