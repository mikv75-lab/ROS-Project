# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab3d/tab3d.py
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, Any, Dict, Tuple, List

import numpy as np

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from pyvistaqt import QtInteractor

from model.recipe.recipe import Recipe

# FIX: tab3d has its own scene_manager.py in the same folder
from .scene_manager import SceneManager, PreviewScene, PreviewResult, Renderable
from .overlays_groupbox import OverlaysGroupBox
from .views_3d_box import Views3DBox

_LOG = logging.getLogger("tabs.recipe.preview.tab3d")

Bounds = Tuple[float, float, float, float, float, float]
_DEFAULT_BOUNDS: Bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)


@dataclass
class _LayerActors:
    actors: List[Any]


class Tab3D(QWidget):
    """
    UI-only 3D view.

    STRICT:
      - SceneManager owns preview logic + styling decisions.
      - Tab3D only renders renderables and manages actor visibility.

    IMPORTANT (your request):
      - All overlay layers are ALWAYS generated once (mask/path/tcp/hits/misses/normals).
      - Checkboxes only hide/show existing actors (no rebuild, no re-load).
      - PyVista QtInteractor is injected from MainWindow (shared interactor).
    """

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        render_callable: Any = None,
        get_bounds_callable: Any = None,
        get_substrate_bounds_callable: Any = None,
        on_overlay_changed: Any = None,  # kept for compat but NOT used for rebuild
    ) -> None:
        super().__init__(parent)

        self._render_callable = render_callable

        self._scene_mgr = SceneManager()
        self._scene: Optional[PreviewScene] = None

        self._preview_valid: bool = False
        self._preview_invalid_reason: Optional[str] = "no_preview"
        self._final_tcp_world_mm: Optional[np.ndarray] = None
        self._path_xyz_mm: Optional[np.ndarray] = None

        # injected QtInteractor (shared from MainWindow)
        self._pv: Optional[QtInteractor] = None

        # actors grouped by layer
        self._layers: Dict[str, _LayerActors] = {}

        # if update_preview happens before interactor injection
        self._pending_renderables: List[Renderable] = []

        # -------- layout: overlays, views, pv host (stretch) --------
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(6)

        self.overlays_box = OverlaysGroupBox(parent=self)
        self._set_policy(self.overlays_box, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Fixed)
        self._layout.addWidget(self.overlays_box, 0)

        # overlay changes MUST NOT request rebuild; only toggle actor visibility
        self.overlays_box.sig_changed.connect(self._apply_overlay_visibility)

        self.views_box = Views3DBox(
            interactor_getter=lambda: self._pv,
            render_callable=self.render,
            bounds_getter=self.get_bounds,
            substrate_bounds_getter=self.get_substrate_bounds,
            cam_pad=1.1,
            iso_extra_zoom=1.30,
            parent=self,
        )
        self._set_policy(self.views_box, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Fixed)
        self._layout.addWidget(self.views_box, 0)

        # Host container for injected shared QtInteractor
        self.pv_host = QWidget(self)
        self.pv_host.setLayout(QVBoxLayout(self.pv_host))
        self.pv_host.layout().setContentsMargins(0, 0, 0, 0)
        self.pv_host.layout().setSpacing(0)
        self._set_policy(self.pv_host, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)
        self._layout.addWidget(self.pv_host, 1)

        # make sure Tab3D itself expands
        self._set_policy(self, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)

    # ------------------------------------------------------------------
    # Injection API (called by MainWindow.attach_preview_widget)
    # ------------------------------------------------------------------

    def get_pv_host(self) -> QWidget:
        return self.pv_host

    def set_interactor(self, interactor: QtInteractor) -> None:
        """
        Called by MainWindow.attach_preview_widget(). After this, Tab3D renders into it.
        """
        if interactor is None:
            return
        self._pv = interactor

        # apply default styling
        try:
            self._pv.set_background("white")
        except Exception:
            pass
        try:
            self._pv.hide_axes()
        except Exception:
            pass

        # if we already computed renderables before injection, render them now
        if self._pending_renderables:
            self.clear_layers()
            for r in list(self._pending_renderables):
                self.add_polydata(r.layer, r.poly, name=r.name, render_kwargs=getattr(r, "render_kwargs", None))
            self._pending_renderables.clear()

        # apply overlay visibility state on current actors
        self._apply_overlay_visibility()
        self.render()

    # ------------------------------------------------------------------
    # Overlay visibility (NO rebuild)
    # ------------------------------------------------------------------

    def get_overlay_config(self) -> Dict[str, Any]:
        return self.overlays_box.get_config()

    def _set_layer_visible(self, layer: str, visible: bool) -> None:
        layer_obj = self._layers.get(layer)
        if not layer_obj:
            return
        for a in list(layer_obj.actors):
            try:
                a.SetVisibility(1 if visible else 0)
            except Exception:
                try:
                    a.SetVisibility(bool(visible))
                except Exception:
                    pass

    def _apply_overlay_visibility(self) -> None:
        cfg = self.get_overlay_config()

        # base meshes always ON
        for base in ("cage", "ground", "mount", "substrate"):
            self._set_layer_visible(base, True)

        # overlay -> layer mapping
        self._set_layer_visible("mask", bool(cfg.get("mask", True)))
        self._set_layer_visible("path", bool(cfg.get("path", True)))
        self._set_layer_visible("hits", bool(cfg.get("hits", True)))
        self._set_layer_visible("misses", bool(cfg.get("misses", True)))
        self._set_layer_visible("normals", bool(cfg.get("normals", True)))

        tcp_vis = bool(cfg.get("tcp", True))
        for l in ("tcp_line", "tcp_x", "tcp_y", "tcp_z"):
            self._set_layer_visible(l, tcp_vis)

        self.render()

    # ------------------------------------------------------------------
    # Rendering helpers
    # ------------------------------------------------------------------

    def render(self) -> None:
        if callable(self._render_callable):
            try:
                self._render_callable()
                return
            except Exception:
                pass
        if self._pv is not None:
            try:
                self._pv.render()
            except Exception:
                pass

    def preview_is_valid(self) -> bool:
        return bool(self._preview_valid)

    def preview_invalid_reason(self) -> Optional[str]:
        return self._preview_invalid_reason

    def final_tcp_world_mm(self) -> Optional[np.ndarray]:
        if self._final_tcp_world_mm is None:
            return None
        return np.asarray(self._final_tcp_world_mm, dtype=float).reshape(-1, 3)

    def path_xyz_mm(self) -> Optional[np.ndarray]:
        if self._path_xyz_mm is None:
            return None
        return np.asarray(self._path_xyz_mm, dtype=float).reshape(-1, 3)

    def get_bounds(self) -> Bounds:
        if self._scene and getattr(self._scene, "bounds", None):
            try:
                b = tuple(self._scene.bounds)  # type: ignore[arg-type]
                if len(b) == 6:
                    return b  # type: ignore[return-value]
            except Exception:
                pass
        return _DEFAULT_BOUNDS

    def get_substrate_bounds(self) -> Optional[Bounds]:
        if self._scene and getattr(self._scene, "substrate_mesh", None) is not None:
            try:
                b = self._scene.substrate_mesh.bounds  # type: ignore[attr-defined]
                return (float(b[0]), float(b[1]), float(b[2]), float(b[3]), float(b[4]), float(b[5]))
            except Exception:
                return None
        return None

    # ------------------------------------------------------------------
    # Actor management
    # ------------------------------------------------------------------

    def clear_layers(self) -> None:
        if self._pv is None:
            self._layers.clear()
            return

        for _name, layer_obj in list(self._layers.items()):
            for a in list(layer_obj.actors):
                try:
                    self._pv.remove_actor(a)
                except Exception:
                    pass
            layer_obj.actors.clear()
        self._layers.clear()

    def add_polydata(self, layer: str, poly: Any, *, name: str = "", render_kwargs: Optional[Dict[str, Any]] = None) -> None:
        if poly is None:
            return
        if self._pv is None:
            return

        try:
            if hasattr(poly, "n_points") and int(getattr(poly, "n_points") or 0) == 0:
                return
        except Exception:
            pass

        if layer not in self._layers:
            self._layers[layer] = _LayerActors(actors=[])

        kwargs: Dict[str, Any] = {
            "name": str(name or layer),
            "pickable": False,
        }
        if isinstance(render_kwargs, dict) and render_kwargs:
            kwargs.update(render_kwargs)

        try:
            a = self._pv.add_mesh(poly, **kwargs)
            self._layers[layer].actors.append(a)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Preview pipeline
    # ------------------------------------------------------------------

    def update_preview(self, *, recipe: Optional[Recipe], ctx: Any) -> PreviewResult:
        self._preview_valid = False
        self._preview_invalid_reason = "no_preview"
        self._final_tcp_world_mm = None
        self._path_xyz_mm = None
        self._scene = None
        self._pending_renderables = []

        if self._pv is not None:
            self.clear_layers()

        overlay_all_true = {
            "mask": True,
            "path": True,
            "tcp": True,
            "hits": True,
            "misses": True,
            "normals": True,
        }

        res = self._scene_mgr.build_preview(recipe=recipe, ctx=ctx, overlay_cfg=overlay_all_true)

        self._scene = res.scene
        self._preview_valid = bool(res.valid)
        self._preview_invalid_reason = res.invalid_reason
        self._final_tcp_world_mm = res.final_tcp_world_mm
        self._path_xyz_mm = res.path_xyz_mm

        if self._pv is None:
            self._pending_renderables = list(res.renderables or [])
            return res

        for r in list(res.renderables or []):
            try:
                self.add_polydata(r.layer, r.poly, name=r.name, render_kwargs=getattr(r, "render_kwargs", None))
            except Exception:
                pass

        self._apply_overlay_visibility()
        return res

    @staticmethod
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
