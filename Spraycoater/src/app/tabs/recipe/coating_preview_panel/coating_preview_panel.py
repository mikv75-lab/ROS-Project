# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Any, Dict, Tuple, List

import numpy as np
from PyQt6.QtCore import pyqtSignal, QTimer
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QSizePolicy
)

from app.model.recipe.recipe import Recipe
from .scene_manager import SceneManager
from .matplot2d import Matplot2DView
from .overlays_groupbox import OverlaysGroupBox
from .info_groupbox import InfoGroupBox
from .views_groupbox import Views2DBox, Views3DBox

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
    Vertikale Reihenfolge:
      Info (Preferred)
      2D View (Preferred)
      Matplotlib (2D)  [Expanding]
      3D View (Preferred)
      Overlays (Preferred)
      PyVista-Host (blanker QWidget) [Expanding]
    Der echte QtInteractor wird vom MainWindow erzeugt und hier in pvHost eingehängt.
    """
    sprayPathSetRequested = pyqtSignal(object)

    DEFAULT_GROUND_COLOR    = "#3a3a3a"
    DEFAULT_MOUNT_COLOR     = "#5d5d5d"
    DEFAULT_SUBSTRATE_COLOR = "#d0d6dd"

    def __init__(self, *, ctx, store=None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.store = store

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ---- Info ----
        self.grpInfo = InfoGroupBox(self)
        _set_policy(self.grpInfo, h=QSizePolicy.Expanding, v=QSizePolicy.Preferred)
        root.addWidget(self.grpInfo, 0)

        # ---- 2D View Controls ----
        self.views2d = Views2DBox(switch_2d=self._switch_2d_plane, parent=self)
        _set_policy(self.views2d, h=QSizePolicy.Expanding, v=QSizePolicy.Preferred)
        root.addWidget(self.views2d, 0)

        # ---- Matplotlib 2D (groß) ----
        self._mat2d = Matplot2DView(parent=None)
        _set_policy(self._mat2d, h=QSizePolicy.Expanding, v=QSizePolicy.Expanding)
        root.addWidget(self._mat2d, 5)

        # ---- 3D View Controls ----
        # Achtung: interactor_getter sucht das QtInteractor-Child IM pvHost
        self.views3d = Views3DBox(
            interactor_getter=lambda: self._find_interactor_in_host(),
            render_callable=self.render,
            bounds_getter=self.get_bounds,
            cam_pad=1.8,
            parent=self,
        )
        _set_policy(self.views3d, h=QSizePolicy.Expanding, v=QSizePolicy.Preferred)
        root.addWidget(self.views3d, 0)

        # ---- Overlays ----
        self.grpOverlays = OverlaysGroupBox(
            self,
            add_mesh_fn=self.add_mesh,
            clear_layer_fn=self.clear_layer,
            add_path_polyline_fn=self.add_path_polyline,
            show_poly_fn=self.show_poly,
            show_frames_at_fn=self.show_frames_at,
            set_layer_visible_fn=lambda layer, vis, render=True: self.scene.set_layer_visible(layer, vis, render=render),
            update_2d_scene_fn=lambda mesh, path_xyz, mask_poly: self.update_2d_scene(
                substrate_mesh=mesh, path_xyz=path_xyz, mask_poly=mask_poly
            ),
            layers={
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
                "frames_x": "frames_x",
                "frames_y": "frames_y",
                "frames_z": "frames_z",
                "frames_labels": "frames_labels",
            },
            get_bounds=self.get_bounds,
        )
        _set_policy(self.grpOverlays, h=QSizePolicy.Expanding, v=QSizePolicy.Preferred)
        root.addWidget(self.grpOverlays, 0)

        # ---- PyVista Host (blank; Interactor kommt extern aus MainWindow) ----
        self._pvHost = QWidget(self)
        self._pvHost.setObjectName("pvHost")
        _set_policy(self._pvHost, h=QSizePolicy.Expanding, v=QSizePolicy.Expanding)
        root.addWidget(self._pvHost, 7)

        # SceneManager arbeitet mit externem Interactor via Getter
        self.scene = SceneManager(
            interactor_getter=self._find_interactor_in_host
        )

        # Welt-Bounds (mm)
        self._bounds: Tuple[float, float, float, float, float, float] = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

        QTimer.singleShot(0, self._push_initial_visibility)

    # -------- Host-API für MainWindow / RecipeTab --------
    def get_pv_host(self) -> QWidget:
        """Gibt den blanken Host zurück, in den der MainWindow-Interactor eingehängt wird."""
        return self._pvHost

    def _find_interactor_in_host(self):
        """Sucht das QtInteractor-Child im pvHost (wird im MainWindow eingehängt)."""
        try:
            from pyvistaqt import QtInteractor  # type: ignore
        except Exception:
            return None
        for ch in self._pvHost.findChildren(QtInteractor):
            return ch
        for ch in self.findChildren(QtInteractor):
            return ch
        return None

    # =================== API vom RecipeTab ===================
    def handle_update_preview(self, model: Recipe) -> None:
        ia = self._find_interactor_in_host()
        if ia is None:
            _LOG.error("handle_update_preview: no interactor in pvHost")
            self._set_info_defaults()
            return

        cam_snap = self.snapshot_camera()

        # Panels/Layers leeren
        self.clear()

        # Scene bauen & Grenzen setzen
        try:
            scene = self.scene.build_scene(self.ctx, model)
        except Exception:
            _LOG.exception("build_scene failed")
            self._set_info_defaults()
            return
        self.set_bounds(scene.bounds)

        # Grundszene zeichnen (3D)
        try:
            self.scene.draw_scene(scene, visibility=None)
        except Exception:
            _LOG.exception("draw_scene failed")

        # compile_poses -> Overlays + Info
        stand_off = 10.0
        try:
            recipe_params = getattr(model, "parameters", {}) or {}
            stand_off = float(recipe_params.get("stand_off_mm", 10.0))
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
                    tool_frame=None,
                )
        except Exception:
            _LOG.exception("compile_poses failed")

        if compiled:
            try:
                self.grpOverlays.render_compiled(
                    substrate_mesh=scene.substrate_mesh or scene.ground_mesh,
                    compiled=compiled,
                    visibility=self._visibility_from_ui(),
                    mask_lift_mm=50.0,
                    default_stand_off_mm=stand_off,
                    ray_len_mm=1000.0,
                    sides=None,
                    only_selected=True,
                )
            except Exception:
                _LOG.exception("Overlays render failed")

        # Info berechnen
        try:
            info = self._calc_info(scene=scene, model=model, compiled=compiled)
            self.grpInfo.set_values(info)
        except Exception:
            _LOG.exception("set info failed")
            self._set_info_defaults()

        # Kamera zurück + Refresh
        try:
            if cam_snap:
                self.restore_camera(cam_snap)
            self.refresh_all_views()
        except Exception:
            _LOG.exception("finalize view failed")

    # =================== Info helpers ===================
    def _set_info_defaults(self) -> None:
        try:
            self.grpInfo.set_values({
                "points": None,
                "length_mm": None,
                "eta_s": None,
                "medium_ml": None,
                "mesh_tris": None,
            })
        except Exception:
            pass

    def _calc_info(self, *, scene, model, compiled) -> Dict[str, Any]:
        params = getattr(model, "parameters", {}) or {}
        speed = float(params.get("speed_mm_s", 0.0)) if isinstance(params, dict) else 0.0
        flow_ml_min = float(params.get("flow_ml_min", 0.0)) if isinstance(params, dict) else 0.0

        points_total = 0
        length_mm = 0.0

        if isinstance(compiled, dict):
            sides = compiled.get("sides", {}) or {}
            for _side, data in sides.items():
                poses: List[Dict[str, float]] = (data or {}).get("poses_quat", []) or []
                n = len(poses)
                points_total += n
                if n >= 2:
                    P = np.array([[p["x"], p["y"], p["z"]] for p in poses], dtype=float).reshape(-1, 3)
                    seg = np.linalg.norm(P[1:] - P[:-1], axis=1)
                    length_mm += float(np.sum(seg))

        eta_s = (length_mm / speed) if speed > 1e-9 else 0.0
        medium_ml = (flow_ml_min / 60.0) * eta_s if flow_ml_min > 0.0 else 0.0

        mesh_tris = getattr(scene, "mesh_tris", None)

        return {
            "points": points_total if points_total > 0 else None,
            "length_mm": length_mm if length_mm > 0 else None,
            "eta_s": eta_s if eta_s > 0 else None,
            "medium_ml": medium_ml if medium_ml > 0 else None,
            "mesh_tris": mesh_tris,
        }

    # =================== Helpers ===================
    def _visibility_from_ui(self) -> dict:
        gb = self.grpOverlays
        return {
            "mask":   gb.chkShowMask.isChecked(),
            "path":   gb.chkShowPath.isChecked(),
            "hits":   gb.chkShowHits.isChecked(),
            "misses": gb.chkShowMisses.isChecked(),
            "normals":gb.chkShowNormals.isChecked(),
            "frames": gb.chkShowLocalFrames.isChecked(),
        }

    def _push_initial_visibility(self) -> None:
        try:
            self.grpOverlays.overlays.apply_visibility(self._visibility_from_ui())
        except Exception:
            _LOG.exception("Initial visibility push failed")

    def _ia(self):
        return self._find_interactor_in_host()

    def render(self) -> None:
        """Wird von Views3DBox aufgerufen, um nach Kamera-Änderungen zu rendern."""
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

    # -------- Bounds --------
    def set_bounds(self, bounds: Tuple[float, float, float, float, float, float]) -> None:
        self._bounds = tuple(map(float, bounds))
        try:
            self._mat2d.set_bounds(self._bounds)
        except Exception:
            _LOG.exception("set_bounds -> 2D failed")

    def get_bounds(self) -> Tuple[float, float, float, float, float, float]:
        return self._bounds

    # -------- 2D switching --------
    def _switch_2d_plane(self, plane: str) -> None:
        """Wird von Views2DBox getriggert (Top/Front/Back/Left/Right)."""
        try:
            self._mat2d.set_plane(plane)
            self._mat2d.refresh()
        except Exception:
            _LOG.exception("2D plane switch failed: %s", plane)

    # -------- 2D Scene feeder --------
    def update_2d_scene(self, *, substrate_mesh=None, path_xyz: np.ndarray | None, mask_poly=None):
        try:
            self._mat2d.set_scene(
                substrate_mesh=substrate_mesh,
                path_xyz=None if path_xyz is None else np.asarray(path_xyz, float).reshape(-1, 3),
                bounds=self._bounds,
                mask_poly=mask_poly,
            )
            self._mat2d.refresh()
        except Exception:
            _LOG.exception("update_2d_scene failed")

    # -------- Scene passthrough --------
    def clear(self) -> None:
        self.scene.clear()

    def add_mesh(self, mesh, **kwargs) -> None:
        self.scene.add_mesh(mesh, **kwargs)

    def clear_layer(self, layer: str) -> None:
        self.scene.clear_layer(layer)

    def add_path_polyline(self, *a, **kw):
        self.scene.add_path_polyline(*a, **kw)

    def show_poly(self, poly, *, layer: str, color: str = "royalblue", line_width: float = 2.0, lighting: bool = False):
        try:
            self.clear_layer(layer)
            self.add_mesh(poly, color=color, render=False, reset_camera=False,
                          line_width=float(line_width), lighting=lighting, layer=layer)
        except Exception:
            _LOG.exception("show_poly() failed")

    def show_frames_at(self, **kwargs) -> None:
        layer = kwargs.pop("layer", None)
        layer_prefix = layer if layer else "frames"
        for lyr in (f"{layer_prefix}_x", f"{layer_prefix}_y", f"{layer_prefix}_z", f"{layer_prefix}_labels"):
            try:
                self.scene.clear_layer(lyr)
            except Exception:
                pass
        # Falls SceneManager.add_frames nicht existiert, hier kein Crash:
        add_frames = getattr(self.scene, "add_frames", None)
        if callable(add_frames):
            add_frames(layer_prefix=layer_prefix, **kwargs)
