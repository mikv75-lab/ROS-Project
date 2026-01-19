# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab3d/tab3d.py
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, Any, Dict, Tuple, List, Protocol, runtime_checkable

import numpy as np
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from pyvistaqt import QtInteractor

from model.recipe.recipe import Recipe

from .scene_manager import SceneManager, PreviewScene, Renderable

_LOG = logging.getLogger("tabs.recipe.preview.tab3d")

Bounds = Tuple[float, float, float, float, float, float]
_DEFAULT_BOUNDS: Bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)


def _err(msg: str) -> None:
    raise ValueError(msg)


@runtime_checkable
class PreviewResultLike(Protocol):
    # STRICT: Tab3D requires exactly these attributes
    recipe: Optional[Recipe]
    valid: bool
    invalid_reason: Optional[str]
    scene: Optional[PreviewScene]
    substrate_mesh: Any
    path_xyz_mm: Optional[np.ndarray]
    final_tcp_world_mm: Optional[np.ndarray]
    renderables: List[Renderable]
    bounds: Bounds
    substrate_bounds: Optional[Bounds]
    meta: Dict[str, Any]


@dataclass
class _LayerActors:
    """Container für PyVista-Akteure einer Ebene."""
    actors: List[Any]


class Tab3D(QWidget):
    """
    Striktes 3D-Anzeige-Widget für Beschichtungs-Vorschauen.

    Regeln:
      - SceneManager ist SSoT für Preview-Build.
      - Overlays werden erzeugt, UI toggelt nur Visibility (SetVisibility).
      - STRICT contract: build_preview() muss PreviewResultLike liefern.
    """

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        render_callable: Any = None,
        get_bounds_callable: Any = None,
        get_substrate_bounds_callable: Any = None,
    ) -> None:
        super().__init__(parent)

        self._render_callable = render_callable
        self._scene_mgr = SceneManager()
        self._scene: Optional[PreviewScene] = None

        self._preview_valid: bool = False
        self._preview_invalid_reason: Optional[str] = "no_preview"

        # Geometry cache
        self._final_tcp_world_mm: Optional[np.ndarray] = None
        self._path_xyz_mm: Optional[np.ndarray] = None

        # PyVista Interactor (shared)
        self._pv: Optional[QtInteractor] = None

        # Actors grouped by layer name
        self._layers: Dict[str, _LayerActors] = {}

        # Renderables cache if called before interactor injection
        self._pending_renderables: List[Renderable] = []

        self._setup_ui()

    # ------------------- Public API -------------------

    def get_pv_host(self) -> QWidget:
        return self.pv_host

    # ------------------- UI -------------------

    def _setup_ui(self) -> None:
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(6)

        from .overlays_groupbox import OverlaysGroupBox
        from .views_3d_box import Views3DBox

        # Overlay controls
        self.overlays_box = OverlaysGroupBox(parent=self)
        self._set_policy(self.overlays_box, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Fixed)
        self._layout.addWidget(self.overlays_box, 0)
        self.overlays_box.sig_changed.connect(self._apply_overlay_visibility)

        # Camera controls
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

        # Host for externally injected QtInteractor
        self.pv_host = QWidget(self)
        self.pv_host.setLayout(QVBoxLayout(self.pv_host))
        self.pv_host.layout().setContentsMargins(0, 0, 0, 0)
        self.pv_host.layout().setSpacing(0)
        self._set_policy(self.pv_host, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)
        self._layout.addWidget(self.pv_host, 1)

        self._set_policy(self, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)

    # ------------------- Interactor Injection -------------------

    def set_interactor(self, interactor: QtInteractor) -> None:
        if interactor is None:
            return
        self._pv = interactor
        try:
            self._pv.set_background("white")
            self._pv.hide_axes()
        except Exception:
            pass

        # Render pending renderables if any
        if self._pending_renderables:
            self.clear_layers()
            for r in list(self._pending_renderables):
                self.add_polydata(r.layer, r.poly, name=r.name, render_kwargs=r.render_kwargs)
            self._pending_renderables.clear()

        self._apply_overlay_visibility()
        self.render()

    # ------------------- Visibility Management -------------------

    def _apply_overlay_visibility(self) -> None:
        cfg: Dict[str, Any] = {}
        try:
            cfg = self.overlays_box.get_config() or {}
        except Exception:
            cfg = {}

        # Base layers always visible
        for base in ("cage", "ground", "mount", "substrate"):
            self._set_layer_visible(base, True)

        # Overlays (main)
        self._set_layer_visible("mask", bool(cfg.get("mask", True)))
        self._set_layer_visible("path", bool(cfg.get("path", True)))
        self._set_layer_visible("hits", bool(cfg.get("hits", True)))
        self._set_layer_visible("misses", bool(cfg.get("misses", True)))
        self._set_layer_visible("normals", bool(cfg.get("normals", True)))

        # TCP overlay controls (grouped) -> ONLY axes (no tcp_line)
        tcp_vis = bool(cfg.get("tcp", True))
        for l in ("tcp_x", "tcp_y", "tcp_z", "planned_tcp", "executed_tcp"):
            self._set_layer_visible(l, tcp_vis)

        self.render()

    def _set_layer_visible(self, layer: str, visible: bool) -> None:
        layer_obj = self._layers.get(layer)
        if not layer_obj:
            return
        for a in layer_obj.actors:
            try:
                a.SetVisibility(1 if visible else 0)
            except Exception:
                pass

    # ------------------- Rendering & Bounds -------------------

    def render(self) -> None:
        if self._pv is not None:
            try:
                self._pv.render()
            except Exception:
                pass

    def get_bounds(self) -> Bounds:
        if self._scene and self._scene.bounds:
            return tuple(self._scene.bounds)  # type: ignore
        return _DEFAULT_BOUNDS

    def get_substrate_bounds(self) -> Optional[Bounds]:
        if self._scene and self._scene.substrate_mesh is not None:
            b = self._scene.substrate_mesh.bounds
            return (float(b[0]), float(b[1]), float(b[2]), float(b[3]), float(b[4]), float(b[5]))
        return None

    # ------------------- Actor Management -------------------

    def clear_layers(self) -> None:
        if self._pv is None:
            self._layers.clear()
            return

        for layer_obj in self._layers.values():
            for a in layer_obj.actors:
                try:
                    self._pv.remove_actor(a)
                except Exception:
                    pass
        self._layers.clear()

    def add_polydata(self, layer: str, poly: Any, *, name: str = "", render_kwargs: Optional[Dict] = None) -> None:
        if poly is None or self._pv is None:
            return

        if layer not in self._layers:
            self._layers[layer] = _LayerActors(actors=[])

        kwargs = {"name": str(name or layer), "pickable": False}
        if render_kwargs:
            kwargs.update(render_kwargs)

        try:
            actor = self._pv.add_mesh(poly, **kwargs)
            self._layers[layer].actors.append(actor)
        except Exception as e:
            _LOG.error("Fehler beim Hinzufügen von Mesh zu Layer %s: %s", layer, e)

    # ------------------- Preview Pipeline -------------------

    def update_preview(self, *, recipe: Optional[Recipe], ctx: Any) -> Any:
        """
        STRICT:
          - calls SceneManager.build_preview()
          - requires the returned object to have PreviewResultLike attributes
        """
        self._preview_valid = False
        self._preview_invalid_reason = "no_preview"
        self._scene = None
        self._pending_renderables = []

        if self._pv is not None:
            self.clear_layers()

        # build preview (overlay_cfg configures overlay geometry generation parameters only)
        res = self._scene_mgr.build_preview(
            recipe=recipe,
            ctx=ctx,
            overlay_cfg={
                "normals_len_mm": 10.0,
                "normals_max": 250,
                "tcp_axes_len_mm": 12.0,
                "tcp_axes_max": 80,
            },
        )

        # STRICT contract check (no legacy fallback)
        if not isinstance(res, PreviewResultLike):
            missing = []
            for k in (
                "scene",
                "valid",
                "invalid_reason",
                "renderables",
                "path_xyz_mm",
                "final_tcp_world_mm",
                "substrate_mesh",
                "bounds",
            ):
                if not hasattr(res, k):
                    missing.append(k)
            _err(f"SceneManager.build_preview returned incompatible result (missing={missing}).")

        self._scene = res.scene
        self._preview_valid = bool(res.valid)
        self._preview_invalid_reason = res.invalid_reason
        self._final_tcp_world_mm = res.final_tcp_world_mm
        self._path_xyz_mm = res.path_xyz_mm

        if self._pv is None:
            self._pending_renderables = list(res.renderables)
            return res

        for r in res.renderables:
            self.add_polydata(r.layer, r.poly, name=r.name, render_kwargs=r.render_kwargs)

        self._apply_overlay_visibility()
        return res

    @staticmethod
    def _set_policy(w: QWidget, h: QSizePolicy.Policy, v: QSizePolicy.Policy) -> None:
        sp = w.sizePolicy()
        sp.setHorizontalPolicy(h)
        sp.setVerticalPolicy(v)
        w.setSizePolicy(sp)
