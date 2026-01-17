# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab3d/tab3d.py
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, Any, Dict

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from pyvistaqt import QtInteractor

# Lokale Imports aus dem tab3d Ordner
from .overlays_groupbox import OverlaysGroupBox
from .views_3d_box import Views3DBox

@dataclass
class _LayerActors:
    actors: list[Any]


_LOG = logging.getLogger("tabs.recipe.preview.tab3d")


class Tab3D(QWidget):
    """
    Kapselt die 3D-Ansicht:
    - Oben: Overlays
    - Mitte: View Controls
    - Unten: PyVista Plotter
    """

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        render_callable: Any = None,
        get_bounds_callable: Any = None,
        get_substrate_bounds_callable: Any = None,
        on_overlay_changed: Any = None,
    ) -> None:
        super().__init__(parent)
        
        self._render_callable = render_callable
        self._get_bounds_callable = get_bounds_callable
        self._get_substrate_bounds_callable = get_substrate_bounds_callable
        self._on_overlay_changed = on_overlay_changed

        self._layers: Dict[str, _LayerActors] = {}

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(4, 4, 4, 4)
        self._layout.setSpacing(6)

        # 1. Overlays (Top)
        self.overlays_box = OverlaysGroupBox(parent=self)
        self._set_policy(self.overlays_box, v=QSizePolicy.Policy.Preferred)
        self._layout.addWidget(self.overlays_box, 0)
        
        if callable(self._on_overlay_changed):
            self.overlays_box.sig_changed.connect(self._on_overlay_changed)

        # 2. PV Host Setup (Created before Views3DBox because of dependency)
        self.pv_host = QWidget(self)
        self._host_layout = QVBoxLayout(self.pv_host)
        self._host_layout.setContentsMargins(0, 0, 0, 0)
        self._host_layout.setSpacing(0)
        
        self.pv_plot = QtInteractor(self.pv_host)
        self._host_layout.addWidget(self.pv_plot, 1)

        # 3. View Controls (Mid)
        self.views_box = Views3DBox(
            interactor_getter=lambda: self.pv_plot,
            render_callable=self._render,
            bounds_getter=self._get_bounds_callable,
            substrate_bounds_getter=self._get_substrate_bounds_callable,
            cam_pad=1.1,
            iso_extra_zoom=1.30,
            parent=self,
        )
        self._set_policy(self.views_box, v=QSizePolicy.Policy.Preferred)
        self._layout.addWidget(self.views_box, 0)

        # 4. Add Plotter (Bottom)
        self._set_policy(self.pv_host, v=QSizePolicy.Policy.Expanding)
        self._layout.addWidget(self.pv_host, 1)

    def get_pv_host(self) -> QWidget:
        return self.pv_host

    def get_overlay_config(self) -> Dict[str, Any]:
        return self.overlays_box.get_config()

    def render(self) -> None:
        self._render()

    def _render(self) -> None:
        if self.pv_plot:
            try:
                self.pv_plot.render()
            except Exception:
                pass

    def clear_layers(self) -> None:
        """Clears all managed actor layers."""
        if self.pv_plot is None: return
        
        for name, layer_Obj in self._layers.items():
            for a in list(layer_Obj.actors):
                try:
                    self.pv_plot.remove_actor(a)
                except Exception:
                    pass
            layer_Obj.actors.clear()

    def add_polydata(self, layer: str, poly: Any, *, name: str = "", opacity: float = 1.0) -> None:
        """Adds a mesh/polydata to a specific layer."""
        if self.pv_plot is None or poly is None:
            return
        
        # Check for empty mesh crash fix
        if hasattr(poly, "n_points") and poly.n_points == 0:
            return

        if layer not in self._layers:
            self._layers[layer] = _LayerActors(actors=[])

        try:
            a = self.pv_plot.add_mesh(
                poly, 
                name=str(name or layer), 
                opacity=float(opacity), 
                pickable=False
            )
            self._layers[layer].actors.append(a)
        except Exception:
            pass

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