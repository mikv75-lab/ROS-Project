# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
from typing import Optional, Any, Dict, Tuple, List

import numpy as np
from PyQt6.QtCore import pyqtSignal, QTimer
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QSizePolicy, QSpacerItem
from PyQt6.sip import isdeleted

from model.recipe.recipe import Recipe
from model.recipe.recipe_store import RecipeStore
from model.recipe.path_builder import PathBuilder

from widgets.info_groupbox import InfoGroupBox

from .views_3d.scene_manager import SceneManager, PreviewScene
from .views_2d.matplot2d import Matplot2DView
from .overlays_groupbox import OverlaysGroupBox
from .views_2d_box import Views2DBox
from .views_3d_box import Views3DBox

_LOG = logging.getLogger("tabs.recipe.preview.panel")

Bounds = Tuple[float, float, float, float, float, float]


def _set_policy(
    w: QWidget,
    *,
    h: QSizePolicy.Policy = QSizePolicy.Policy.Expanding,
    v: QSizePolicy.Policy = QSizePolicy.Policy.Preferred,
) -> None:
    sp = w.sizePolicy()
    sp.setHorizontalPolicy(h)
    sp.setVerticalPolicy(v)
    w.setSizePolicy(sp)


class CoatingPreviewPanel(QWidget):
    """
    Layout:
      Info
      Split (HBox):
        - Left:  Views2D + Matplotlib
        - Right: Views3D-Buttons + Overlays + pvHost (PyVista interactor)
    """

    interactorReady = pyqtSignal()

    def __init__(self, *, ctx, store: RecipeStore, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        if store is None or not isinstance(store, RecipeStore):
            raise RuntimeError("CoatingPreviewPanel: store fehlt/ungültig (RecipeTab muss store übergeben).")
        self.store: RecipeStore = store

        self._dying: bool = False
        self.destroyed.connect(lambda *_: setattr(self, "_dying", True))

        self._interactor: Any = None
        self._retry_left: int = 10
        self._retry_delay_ms: int = 50

        self._vis: Dict[str, bool] = {
            "path": True,
            "hits": True,
            "misses": True,
            "normals": False,
            "frames": False,
        }

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        self.grpInfo = InfoGroupBox(self)
        _set_policy(self.grpInfo, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        root.addWidget(self.grpInfo, 0)

        split = QHBoxLayout()
        split.setContentsMargins(0, 0, 0, 0)
        split.setSpacing(8)
        root.addLayout(split, 1)

        # ---------------- Left (2D) ----------------
        vleft = QVBoxLayout()
        vleft.setContentsMargins(0, 0, 0, 0)
        vleft.setSpacing(6)
        split.addLayout(vleft, 1)

        self._mat2d = Matplot2DView(parent=self)
        _set_policy(self._mat2d, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)

        try:
            tb = self._mat2d.make_toolbar(self)
            if tb is not None:
                self._mat2d_toolbar = tb
                vleft.addWidget(tb, 0)
        except Exception:
            _LOG.exception("Matplot2D toolbar creation failed")

        vleft.addWidget(self._mat2d, 1)
        vleft.addSpacerItem(QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        self.destroyed.connect(lambda *_: getattr(self._mat2d, "dispose", lambda: None)())

        self.views2d = Views2DBox(
            switch_2d=self._switch_2d_plane,
            refresh_callable=self._mat2d.refresh,
            get_bounds=self.get_bounds,
            set_bounds=self.set_bounds,
            parent=self,
        )
        _set_policy(self.views2d, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        vleft.insertWidget(0, self.views2d, 0)

        # ---------------- Right (3D) ----------------
        vright = QVBoxLayout()
        vright.setContentsMargins(0, 0, 0, 0)
        vright.setSpacing(6)
        split.addLayout(vright, 1)

        self.scene = SceneManager(interactor_getter=lambda: self._interactor)

        self.views3d = Views3DBox(
            interactor_getter=lambda: self._interactor,
            render_callable=self.render,
            bounds_getter=self.get_bounds,
            substrate_bounds_getter=lambda: self.scene.get_layer_bounds("substrate"),
            cam_pad=1.0,
            iso_extra_zoom=3.0,
            parent=self,
        )
        _set_policy(self.views3d, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
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
                substrate_mesh=mesh,
                path_xyz=path_xyz,
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
        _set_policy(self.grpOverlays, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        vright.addWidget(self.grpOverlays, 0)

        self._pvHost = QWidget(self)
        self._pvHost.setObjectName("pvHost")
        _set_policy(self._pvHost, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)

        self._pvHostLayout = QVBoxLayout(self._pvHost)
        self._pvHostLayout.setContentsMargins(0, 0, 0, 0)
        self._pvHostLayout.setSpacing(0)

        vright.addWidget(self._pvHost, 1)

        self._bounds: Bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)
        QTimer.singleShot(0, self._push_initial_visibility)

    # -------- Public API --------

    def get_pv_host(self) -> QWidget:
        return self._pvHost

    def set_interactor(self, ia) -> None:
        self._interactor = ia
        try:
            if self._interactor is not None and hasattr(self._interactor, "render"):
                self._interactor.render()
        except Exception:
            _LOG.exception("set_interactor: initial render failed")
        self.interactorReady.emit()

    def update_preview(self, model: Recipe) -> None:
        self.handle_update_preview(model)

    # -------- Path compilation (Recipe bleibt unangetastet) --------

    def _compile_path_xyz(self, model: Recipe) -> Optional[np.ndarray]:
        """
        Baut den Preview-Path ausschließlich aus:
          - Recipe.paths_by_side
          - Recipe.globals_params
          - Store (defaults + recipe_def/sides)
        """
        rec_def = self.store.get_recipe_def(model.id)
        sides_dict = self.store.sides_for_recipe(rec_def)  # -> Dict[side_id, spec]
        sides: List[str] = list(sides_dict.keys())

        defaults = self.store.collect_global_defaults()
        globals_params = dict(defaults)
        globals_params.update(getattr(model, "globals_params", {}) or {})

        sample_step_mm = float(globals_params.get("sample_step_mm", 1.0))
        max_points = int(globals_params.get("max_points", 500))

        built = PathBuilder.from_recipe_paths(
            recipe=model,
            sides=sides,
            globals_params=globals_params,
            sample_step_mm=sample_step_mm,
            max_points=max_points,
        )

        pts_all: List[np.ndarray] = []
        for _side_id, pdata in built:
            if pdata is None:
                continue

            arr = getattr(pdata, "points_mm", None)
            if arr is None:
                arr = getattr(pdata, "points", None)

            if arr is None:
                continue

            P = np.asarray(arr, dtype=float).reshape(-1, 3)
            if P.size == 0:
                continue

            pts_all.append(P)

    # -------- Preview Pipeline --------

    def handle_update_preview(self, model: Recipe) -> None:
        if self._dying or isdeleted(self):
            return

        ia = self._interactor
        if ia is None:
            if self._retry_left > 0:
                self._retry_left -= 1
                QTimer.singleShot(self._retry_delay_ms, lambda m=model: self.handle_update_preview(m))
            else:
                self._set_info_defaults()
            return
        self._retry_left = 10

        cam_snap = self.snapshot_camera()

        # 1) Szene (Substrat/Mount/Ground)
        try:
            scene: PreviewScene = self.scene.build_scene(self.ctx, model, grid_step_mm=10.0)
        except Exception:
            _LOG.exception("build_scene failed")
            self._set_info_defaults()
            return

        self.set_bounds(scene.bounds)

        # 2) Pfad kompilieren (aus Recipe + Store) und Overlays bauen
        path_xyz: Optional[np.ndarray] = None
        try:
            path_xyz = self._compile_path_xyz(model)
        except Exception:
            _LOG.exception("path compile failed -> no overlays")
            path_xyz = None

        try:
            self.scene.build_overlays(
                path_xyz=path_xyz,
                normals_xyz=None,
                frames_at=None,
                rays_hit=None,
                rays_miss=None,
                as_tube=False,
                tube_radius=0.8,
            )
        except Exception:
            _LOG.exception("build_overlays failed")

        # 3) Sichtbarkeit (Checkboxes -> SceneManager)
        try:
            vis = self.grpOverlays.current_visibility()
            self.scene.toggle_overlays(vis)
        except Exception:
            _LOG.exception("apply visibility failed")

        # 4) 2D View updaten (Substrat + Pfad)
        try:
            self.update_2d_scene(substrate_mesh=scene.substrate_mesh, path_xyz=path_xyz)
        except Exception:
            _LOG.exception("update_2d_scene failed")

        # 5) InfoGroupBox (als dict, nicht PreviewScene!)
        try:
            info = {
                "points": int(0 if path_xyz is None else len(path_xyz)),
                "tris": scene.mesh_tris,
                "bounds": scene.bounds,
            }
            self.grpInfo.set_values(info)
        except Exception:
            _LOG.exception("InfoGroupBox update failed")

        # 6) Kamera wiederherstellen + render
        try:
            if cam_snap:
                self.restore_camera(cam_snap)
        except Exception:
            _LOG.exception("restore_camera failed")

        self.scene.update_current_views_once(refresh_2d=self._mat2d.refresh)

    # -------- Helpers --------

    def _set_info_defaults(self) -> None:
        try:
            if not self._dying and self.grpInfo is not None and not isdeleted(self.grpInfo):
                self.grpInfo.set_values(None)
        except Exception:
            pass

    def _push_initial_visibility(self) -> None:
        try:
            self.scene.toggle_overlays(dict(self._vis))
            self.scene.update_current_views_once(refresh_2d=self._mat2d.refresh)
        except Exception:
            _LOG.exception("initial visibility push failed")

    def render(self) -> None:
        ia = self._interactor
        try:
            if ia is not None and hasattr(ia, "render"):
                ia.render()
        except Exception:
            _LOG.exception("render failed")

    def snapshot_camera(self) -> Optional[Dict[str, Any]]:
        ia = self._interactor
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

    def restore_camera(self, snap: Optional[Dict[str, Any]]) -> None:
        if not snap:
            return
        ia = self._interactor
        if ia is None or not hasattr(ia, "camera"):
            return
        try:
            cam = ia.camera
            cam.position = snap.get("position", cam.position)
            cam.focal_point = snap.get("focal_point", cam.focal_point)
            cam.up = snap.get("view_up", cam.up)
            cr = snap.get("clipping_range")
            if cr:
                cam.clipping_range = cr
        except Exception:
            _LOG.exception("restore_camera failed")

    def set_bounds(self, bounds: Bounds) -> None:
        self._bounds = tuple(map(float, bounds))  # type: ignore[assignment]
        try:
            self._mat2d.set_bounds(self._bounds)
        except Exception:
            _LOG.exception("set_bounds -> 2D failed")

    def get_bounds(self) -> Bounds:
        return self._bounds

    def _switch_2d_plane(self, plane: str) -> None:
        try:
            self._mat2d.set_plane(plane)
            self._mat2d.refresh()
        except Exception:
            _LOG.exception("2D plane switch failed: %s", plane)

    def update_2d_scene(self, *, substrate_mesh=None, path_xyz: Optional[np.ndarray] = None) -> None:
        try:
            self._mat2d.set_scene(
                substrate_mesh=substrate_mesh,
                path_xyz=None if path_xyz is None else np.asarray(path_xyz, float).reshape(-1, 3),
                bounds=self._bounds,
            )
            self._mat2d.refresh()
        except Exception:
            _LOG.exception("update_2d_scene failed")

    # SceneManager passthroughs
    def clear(self) -> None:
        self.scene.clear()

    def add_mesh(self, mesh, **kwargs) -> None:
        self.scene.add_mesh(mesh, **kwargs)

    def clear_layer(self, layer: str) -> None:
        self.scene.clear_layer(layer)

    def add_path_polyline(self, *a, **kw):
        return self.scene.add_path_polyline(*a, **kw)

    def show_poly(self, poly, *, layer: str, color: str = "royalblue", line_width: float = 2.0, lighting: bool = False) -> None:
        try:
            self.clear_layer(layer)
            self.add_mesh(poly, color=color, line_width=float(line_width), lighting=lighting, layer=layer)
        except Exception:
            _LOG.exception("show_poly() failed")

    def show_frames_at(self, **_kwargs) -> None:
        return
