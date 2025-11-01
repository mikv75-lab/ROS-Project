# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Callable, Dict, List, Optional, Any

import numpy as np
import pyvista as pv

_LOG = logging.getLogger("app.tabs.recipe.scene")


class SceneManager:
    """
    Dünne Schicht über dem PyVista-Interactor mit Layer-Verwaltung.
    - Jeder Layer hält eine Liste von Actor-Handles.
    - add_mesh() gibt den Actor zurück und merkt ihn im Layer.
    - clear_layer() entfernt Actors eines Layers.
    - set_layer_visible() toggelt nur Sichtbarkeit.
    """

    def __init__(self, interactor_getter: Callable[[], Any], init_scene_builder: Callable[[], None] | None = None):
        self._get_ia: Callable[[], Any] = interactor_getter
        self._init_scene_builder = init_scene_builder
        self._layers: Dict[str, List[Any]] = {}   # layer -> [actors]

    # ---------- intern ----------
    def _ia(self):
        ia = self._get_ia()
        if ia is None:
            _LOG.warning("SceneManager: Interactor not available")
        return ia

    def _ensure_layer(self, layer: str) -> List[Any]:
        if layer not in self._layers:
            self._layers[layer] = []
        return self._layers[layer]

    # ---------- public ----------
    def add_mesh(self, mesh: pv.PolyData | pv.MultiBlock | Any, *,
                 layer: str = "default",
                 render: bool = False,
                 reset_camera: bool = False,
                 **kwargs) -> Optional[Any]:
        """
        Fügt eine Geometrie hinzu und merkt den Actor im Layer.
        Alle kwargs gehen an ia.add_mesh(...).
        """
        ia = self._ia()
        if ia is None or mesh is None:
            return None

        try:
            actor = ia.add_mesh(mesh, **kwargs)
            self._ensure_layer(layer).append(actor)

            if reset_camera:
                try:
                    b = getattr(mesh, "bounds", None)
                    if b is not None and np.all(np.isfinite(b)):
                        ia.reset_camera(bounds=b)
                    else:
                        ia.reset_camera()
                except Exception:
                    _LOG.exception("reset_camera failed in add_mesh")

            if render:
                ia.render()
            return actor
        except Exception:
            _LOG.exception("add_mesh failed")
            return None

    def add_path_polyline(self, points_mm: np.ndarray, *,
                          layer: str = "path",
                          color: str = "#e74c3c",
                          line_width: float = 2.0,
                          render: bool = False,
                          reset_camera: bool = False,
                          lighting: bool = False,
                          # Optionen:
                          as_tube: bool = False,
                          tube_radius: float = 0.8,
                          tube_sides: int = 12,
                          tube_capping: bool = False) -> Optional[Any]:
        """
        Fügt eine Polyline (Pfad) hinzu.
        - Normal: Linie mit `line_width`.
        - as_tube=True: 3D-Tube entlang der Polyline.
        """
        try:
            P = np.asarray(points_mm, float).reshape(-1, 3)
            if len(P) < 2:
                return None

            # Polyline-Konstruktion
            lines = np.hstack([[len(P)], np.arange(len(P), dtype=np.int64)])
            poly = pv.PolyData(P)
            poly.lines = lines

            if as_tube:
                try:
                    tube = poly.tube(radius=float(tube_radius),
                                     n_sides=int(tube_sides),
                                     capping=bool(tube_capping))
                    return self.add_mesh(
                        tube, layer=layer, color=color,
                        render=render, reset_camera=reset_camera, lighting=lighting
                    )
                except Exception:
                    _LOG.exception("tube generation from polyline failed; fallback to line")

            return self.add_mesh(
                poly, layer=layer, color=color, line_width=float(line_width),
                render=render, reset_camera=reset_camera, lighting=lighting
            )
        except Exception:
            _LOG.exception("add_path_polyline failed")
            return None

    def add_path_markers(self, points_mm: np.ndarray, *,
                         layer: str = "path_markers",
                         color: str = "#e74c3c",
                         point_size: float = 8.0,
                         render: bool = False,
                         reset_camera: bool = False,
                         lighting: bool = False,
                         step: int = 1) -> Optional[Any]:
        """
        Fügt Punktmarker entlang eines Pfads hinzu.
        step: Nur jeden 'step'-ten Punkt anzeigen (>=1).
        """
        try:
            P = np.asarray(points_mm, float).reshape(-1, 3)
            if len(P) == 0:
                return None

            s = int(step) if step and int(step) > 1 else 1
            if s > 1:
                P = P[::s]
            if len(P) == 0:
                return None

            poly = pv.PolyData(P)
            return self.add_mesh(poly, layer=layer, color=color,
                                 point_size=float(point_size),
                                 render=render, reset_camera=reset_camera,
                                 lighting=lighting)
        except Exception:
            _LOG.exception("add_path_markers failed")
            return None

    def clear_layer(self, layer: str, *, render: bool = False) -> None:
        ia = self._ia()
        if ia is None:
            return
        actors = self._layers.get(layer, [])
        if not actors:
            return
        try:
            for a in actors:
                try:
                    ia.remove_actor(a)
                except Exception:
                    pass
            self._layers[layer] = []
            if render:
                try:
                    ia.render()
                except Exception:
                    pass
        except Exception:
            _LOG.exception("clear_layer failed")

    def clear(self, *, render: bool = False) -> None:
        ia = self._ia()
        if ia is None:
            return
        try:
            for layer, actors in list(self._layers.items()):
                for a in actors:
                    try:
                        ia.remove_actor(a)
                    except Exception:
                        pass
                self._layers[layer] = []
            if render:
                try:
                    ia.render()
                except Exception:
                    pass
        except Exception:
            _LOG.exception("clear failed")

    # ---------- Layer-Helfer ----------
    def has_layer(self, layer: str) -> bool:
        return layer in self._layers and len(self._layers[layer]) > 0

    def set_layer_visible(self, layer: str, visible: bool, *, render: bool = True) -> None:
        ia = self._ia()
        if ia is None:
            return
        actors = self._layers.get(layer, [])
        for a in actors:
            try:
                a.SetVisibility(1 if visible else 0)
            except Exception:
                pass
        if render:
            try:
                ia.render()
            except Exception:
                pass
