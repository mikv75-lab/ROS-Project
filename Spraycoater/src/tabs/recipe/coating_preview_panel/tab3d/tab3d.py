# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab3d/tab3d.py
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, Any, Dict, Tuple, List

import numpy as np
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from pyvistaqt import QtInteractor

# STRICT V2 Models
from model.recipe.recipe import Recipe

# Lokale Komponenten
from .scene_manager import SceneManager, PreviewScene, PreviewResult, Renderable
from .overlays_groupbox import OverlaysGroupBox
from .views_3d_box import Views3DBox

_LOG = logging.getLogger("tabs.recipe.preview.tab3d")

Bounds = Tuple[float, float, float, float, float, float]
_DEFAULT_BOUNDS: Bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)


@dataclass
class _LayerActors:
    """Container für PyVista-Akteure einer Ebene."""
    actors: List[Any]


class Tab3D(QWidget):
    """
    Striktes 3D-Anzeige-Widget für Beschichtungs-Vorschauen.

    Eigenschaften:
      - Nutzt Recipe V2 als Single Source of Truth.
      - SceneManager entscheidet über Geometrie und visuellen Stil.
      - Overlays werden einmalig generiert; Checkboxen schalten nur die Sichtbarkeit (SetVisibility),
        ohne eine Neuberechnung auszulösen.
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

        # Geometrie-Cache
        self._final_tcp_world_mm: Optional[np.ndarray] = None
        self._path_xyz_mm: Optional[np.ndarray] = None

        # PyVista Interactor (Shared von MainWindow)
        self._pv: Optional[QtInteractor] = None

        # Akteure gruppiert nach Layer-Namen
        self._layers: Dict[str, _LayerActors] = {}

        # Cache für Renderables, falls update_preview vor dem Interactor-Injection gerufen wird
        self._pending_renderables: List[Renderable] = []

        self._setup_ui()

    # ------------------- Small public API -------------------

    def get_pv_host(self) -> QWidget:
        """Host widget where MainWindow injects the shared QtInteractor."""
        return self.pv_host

    # ------------------- UI -------------------

    def _setup_ui(self) -> None:
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(6)

        # Overlay-Steuerung (Maske, Pfad, TCP, Hits, etc.)
        self.overlays_box = OverlaysGroupBox(parent=self)
        self._set_policy(self.overlays_box, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Fixed)
        self._layout.addWidget(self.overlays_box, 0)

        # Signal: Sichtbarkeit umschalten (kein Rebuild!)
        self.overlays_box.sig_changed.connect(self._apply_overlay_visibility)

        # Kamera-Steuerung
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

        # Host für den extern injizierten PyVista Interactor
        self.pv_host = QWidget(self)
        self.pv_host.setLayout(QVBoxLayout(self.pv_host))
        self.pv_host.layout().setContentsMargins(0, 0, 0, 0)
        self.pv_host.layout().setSpacing(0)
        self._set_policy(self.pv_host, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)
        self._layout.addWidget(self.pv_host, 1)

        self._set_policy(self, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)

    # ------------------- Interactor Injection -------------------

    def set_interactor(self, interactor: QtInteractor) -> None:
        """Wird von MainWindow gerufen, um den Interactor zu teilen."""
        if interactor is None:
            return
        self._pv = interactor
        self._pv.set_background("white")
        self._pv.hide_axes()

        # Nachzügler rendern
        if self._pending_renderables:
            self.clear_layers()
            for r in list(self._pending_renderables):
                self.add_polydata(r.layer, r.poly, name=r.name, render_kwargs=r.render_kwargs)
            self._pending_renderables.clear()

        self._apply_overlay_visibility()
        self.render()

    # ------------------- Sichtbarkeits-Management -------------------

    def _apply_overlay_visibility(self) -> None:
        """Steuert die Sichtbarkeit der Akteure basierend auf den UI-Toggles."""
        cfg = self.overlays_box.get_config()

        # Basis-Meshes (immer an)
        for base in ("cage", "ground", "mount", "substrate"):
            self._set_layer_visible(base, True)

        # Overlays
        self._set_layer_visible("mask", bool(cfg.get("mask", True)))
        self._set_layer_visible("path", bool(cfg.get("path", True)))
        self._set_layer_visible("hits", bool(cfg.get("hits", True)))
        self._set_layer_visible("misses", bool(cfg.get("misses", True)))
        self._set_layer_visible("normals", bool(cfg.get("normals", True)))

        # TCP Layer (beinhaltet Linie und Frames/Achsen)
        tcp_vis = bool(cfg.get("tcp", True))
        for l in ("tcp_line", "tcp_x", "tcp_y", "tcp_z", "planned_tcp", "executed_tcp"):
            self._set_layer_visible(l, tcp_vis)

        self.render()

    def _set_layer_visible(self, layer: str, visible: bool) -> None:
        layer_obj = self._layers.get(layer)
        if layer_obj:
            for a in layer_obj.actors:
                try:
                    a.SetVisibility(1 if visible else 0)
                except Exception:
                    pass

    # ------------------- Rendering & Bounds -------------------

    def render(self) -> None:
        # STRICT: niemals zurück in ein externes Render-Callable springen,
        # sonst kann eine Rekursion entstehen (Panel.render -> Tab3D.render -> Panel.render ...)
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

    # ------------------- Akteur-Management -------------------

    def clear_layers(self) -> None:
        """Entfernt alle Akteure aus dem Interactor."""
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
        """Fügt Geometrie zu einem spezifischen Layer hinzu."""
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

    # ------------------- Pipeline -------------------

    def update_preview(self, *, recipe: Optional[Recipe], ctx: Any) -> PreviewResult:
        """
        Zentraler Einstiegspunkt für die Aktualisierung der 3D-Szene.

        Verarbeitet das Recipe V2 Objekt und fordert Renderables vom SceneManager an.
        """
        self._preview_valid = False
        self._preview_invalid_reason = "no_preview"
        self._scene = None
        self._pending_renderables = []

        if self._pv is not None:
            self.clear_layers()

        # SceneManager baut die Preview (Raycast, Post-Processing, Mesh-Loading)
        res = self._scene_mgr.build_preview(
            recipe=recipe,
            ctx=ctx,
            overlay_cfg={"all_true": True},  # Immer alles generieren für schnelles Toggling
        )

        self._scene = res.scene
        self._preview_valid = res.valid
        self._preview_invalid_reason = res.invalid_reason
        self._final_tcp_world_mm = res.final_tcp_world_mm
        self._path_xyz_mm = res.path_xyz_mm

        if self._pv is None:
            self._pending_renderables = list(res.renderables)
            return res

        # Akteure in die Szene einfügen
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
