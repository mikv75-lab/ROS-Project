# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Callable, Dict, List, Optional, Any, Tuple

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
                try:
                    ia.render()
                except Exception:
                    pass
            return actor
        except Exception:
            _LOG.exception("add_mesh failed")
            return None

    def add_points(self, points_mm: np.ndarray | None = None, *,
                   # tolerant für ältere Aufrufe: points=...
                   points: np.ndarray | None = None,
                   layer: str = "points",
                   color: str = "#111111",
                   point_size: float = 6.0,
                   render: bool = False,
                   reset_camera: bool = False,
                   lighting: bool = False,
                   labels: list[str] | None = None) -> Optional[Any]:
        """
        Punkte (Glyphs) als eigener Layer. Optional mit Labels.
        """
        try:
            src = points_mm if points_mm is not None else points
            if src is None:
                return None

            P = np.asarray(src, float).reshape(-1, 3)
            if len(P) == 0:
                return None

            poly = pv.PolyData(P)
            actor = self.add_mesh(poly, layer=layer, color=color,
                                  point_size=float(point_size),
                                  render=False, reset_camera=reset_camera,
                                  lighting=lighting)

            if labels:
                try:
                    ia = self._ia()
                    if ia is not None:
                        L = min(len(labels), len(P))
                        if L > 0:
                            lab_actor = ia.add_point_labels(
                                P[:L],
                                list(labels)[:L],
                                point_size=0,
                                font_size=12,
                                text_color="black",
                                shape_opacity=0.25,
                                render=False,
                            )
                            self._ensure_layer(layer).append(lab_actor)
                except Exception:
                    _LOG.exception("add_points: labels failed")

            if render:
                try:
                    ia = self._ia()
                    if ia is not None:
                        ia.render()
                except Exception:
                    pass

            return actor
        except Exception:
            _LOG.exception("add_points failed")
            return None

    def add_lines(self, points: np.ndarray, lines: np.ndarray, *,
                  layer: str = "lines",
                  color: str = "#5a5a5a",
                  line_width: float = 1.0,
                  render: bool = False,
                  reset_camera: bool = False,
                  lighting: bool = False) -> Optional[Any]:
        """
        Generische Linien: `points` (N,3) und `lines` im VTK-Schema:
        [2, i0, i1, 2, i2, i3, ...] oder als (M,3)-Array mit erster Spalte=2.
        """
        ia = self._ia()
        if ia is None:
            return None
        try:
            P = np.asarray(points, float).reshape(-1, 3)
            L = np.asarray(lines)
            if L.ndim == 2 and L.shape[1] == 3:
                L = L.reshape(-1)
            poly = pv.PolyData(P)
            poly.lines = L
            return self.add_mesh(poly, layer=layer, color=color, line_width=float(line_width),
                                 render=render, reset_camera=reset_camera, lighting=lighting)
        except Exception:
            _LOG.exception("add_lines failed")
            return None

    def add_segments(self, segments: np.ndarray, *,
                     layer: str = "segments",
                     color: str = "#5a5a5a",
                     line_width: float = 1.0,
                     render: bool = False,
                     reset_camera: bool = False,
                     lighting: bool = False) -> Optional[Any]:
        """
        Bequemlichkeit: Segmente als Array (N,2,3) entgegennehmen.
        """
        try:
            S = np.asarray(segments, float).reshape(-1, 2, 3)
            if len(S) == 0:
                return None
            # Punkte stapeln
            P = S.reshape(-1, 3)
            # Linien-Indexliste erzeugen
            nseg = S.shape[0]
            lines = np.empty((nseg, 3), dtype=np.int64)
            lines[:, 0] = 2
            base = np.arange(nseg, dtype=np.int64) * 2
            lines[:, 1] = base
            lines[:, 2] = base + 1
            return self.add_lines(P, lines, layer=layer, color=color, line_width=line_width,
                                  render=render, reset_camera=reset_camera, lighting=lighting)
        except Exception:
            _LOG.exception("add_segments failed")
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

            # Polyline-Konstruktion (eine zusammenhängende Kette)
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

    def add_path_markers(self, points_mm: np.ndarray | None = None, *,
                         # tolerant für ältere Aufrufe: points=...
                         points: np.ndarray | None = None,
                         layer: str = "path_markers",
                         color: str = "#e74c3c",
                         point_size: float = 8.0,
                         render: bool = False,
                         reset_camera: bool = False,
                         lighting: bool = False,
                         step: int = 1,
                         labels: list[str] | None = None) -> Optional[Any]:
        """
        Fügt Punktmarker (und optional Labels) entlang eines Pfads hinzu.
        step: Nur jeden 'step'-ten Punkt anzeigen (>=1).
        """
        try:
            src = points_mm if points_mm is not None else points
            if src is None:
                return None

            P = np.asarray(src, float).reshape(-1, 3)
            if len(P) == 0:
                return None

            s = int(step) if step and int(step) > 1 else 1
            if s > 1:
                P = P[::s]
            if len(P) == 0:
                return None

            ia = self._ia()
            if ia is None:
                return None

            poly = pv.PolyData(P)
            actor = self.add_mesh(poly, layer=layer, color=color,
                                  point_size=float(point_size),
                                  render=False, reset_camera=reset_camera,
                                  lighting=lighting)

            if labels:
                try:
                    L = min(len(labels), len(P))
                    if L > 0:
                        lab_actor = ia.add_point_labels(
                            P[:L],
                            list(labels)[:L],
                            point_size=0,
                            font_size=12,
                            text_color="black",
                            shape_opacity=0.25,
                            render=False,
                        )
                        self._ensure_layer(layer).append(lab_actor)
                except Exception:
                    _LOG.exception("add_path_markers: labels failed")

            if render:
                try:
                    ia.render()
                except Exception:
                    pass

            return actor
        except Exception:
            _LOG.exception("add_path_markers failed")
            return None

    def show_poly(self, poly: pv.PolyData, *, layer: str,
                  color: str = "#2ecc71", line_width: float = 1.0,
                  lighting: bool = False, reset_camera: bool = False,
                  render: bool = False) -> Optional[Any]:
        """
        Einfache Poly-Helferfunktion (räumt Layer vorher auf).
        """
        try:
            self.clear_layer(layer)
        except Exception:
            pass
        return self.add_mesh(
            poly,
            color=color,
            line_width=float(line_width),
            lighting=lighting,
            layer=layer,
            reset_camera=reset_camera,
            render=render,
        )

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

    def get_layer_bounds(self, layer: str) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        Kombinierte Bounds aller Actors eines Layers (via a.GetBounds()).
        """
        actors = self._layers.get(layer, [])
        if not actors:
            return None
        mins = np.array([ np.inf,  np.inf,  np.inf], float)
        maxs = np.array([-np.inf, -np.inf, -np.inf], float)
        any_ok = False
        for a in actors:
            try:
                b = a.GetBounds()  # (xmin, xmax, ymin, ymax, zmin, zmax)
                if b is None:
                    continue
                b = np.asarray(b, float).reshape(6)
                if not np.all(np.isfinite(b)):
                    continue
                mins = np.minimum(mins, [b[0], b[2], b[4]])
                maxs = np.maximum(maxs, [b[1], b[3], b[5]])
                any_ok = True
            except Exception:
                pass
        if not any_ok:
            return None
        return (float(mins[0]), float(maxs[0]),
                float(mins[1]), float(maxs[1]),
                float(mins[2]), float(maxs[2]))
