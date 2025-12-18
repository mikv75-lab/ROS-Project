# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Callable, Optional, Tuple
import logging

_LOG = logging.getLogger("app.tabs.recipe.preview.views.controller2d")

# Bounds-Format: (xmin, xmax, ymin, ymax, zmin, zmax) – Einheit hängt vom restlichen System ab
Bounds = Tuple[float, float, float, float, float, float]


class ViewController2D:
    """Schlanker Controller, der die Matplotlib-2D-Ansicht bedient (Plane wechseln, optional Bounds)."""

    def __init__(
        self,
        set_plane: Callable[[str], None],
        refresh: Callable[[], None],
        *,
        get_bounds: Optional[Callable[[], Bounds]] = None,
        set_bounds: Optional[Callable[[Bounds], None]] = None,
    ):
        # Funktions-Callbacks aus der View (oder einem Adapter), damit der Controller keine GUI-Klassen kennen muss
        self._set_plane = set_plane
        self._refresh = refresh

        # Optional: Zugriff auf Bounds (derzeit im Switch nicht genutzt)
        self._get_bounds = get_bounds
        self._set_bounds = set_bounds

    def plane_top(self):
        """Wechselt zur Top-Ansicht (XY)."""
        self._switch("top")

    def plane_front(self):
        """Wechselt zur Front-Ansicht (XZ)."""
        self._switch("front")

    def plane_back(self):
        """Wechselt zur Back-Ansicht (XZ, nur andere Blickrichtung)."""
        self._switch("back")

    def plane_left(self):
        """Wechselt zur Left-Ansicht (YZ)."""
        self._switch("left")

    def plane_right(self):
        """Wechselt zur Right-Ansicht (YZ, nur andere Blickrichtung)."""
        self._switch("right")

    def _switch(self, plane: str):
        """Interner Helfer: setzt die Ebene und führt best-effort Fehlerlogging aus."""
        # 1) Ebene umschalten (eigentliche View-Logik steckt in set_plane)
        try:
            self._set_plane(plane)
        except Exception:
            _LOG.exception("2D plane switch failed (set_plane): %s", plane)

        # 2) Bounds sind hier absichtlich nicht verändert.
        #    (Hooks über get_bounds/set_bounds sind vorhanden, falls später benötigt.)

        # 3) Refresh ist aktuell auskommentiert, weil set_plane typischerweise selbst redrawt.
        #    Wenn man später gezielt refreshen will, kann man das hier wieder aktivieren.
        #try:
        #    self._refresh()
        #except Exception:
        #    _LOG.exception("2D refresh failed after switch: %s", plane)
