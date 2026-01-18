# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab2d/view_controller_2d.py
from __future__ import annotations
from typing import Callable, Optional, Tuple
import logging

_LOG = logging.getLogger("tabs.recipe.preview.views.controller2d")

# Bounds-Format: (xmin, xmax, ymin, ymax, zmin, zmax)
Bounds = Tuple[float, float, float, float, float, float]


class ViewController2D:
    """
    Schlanker Controller, der die Interaktionen der 2D-Ansicht steuert 
    (z. B. Ebenenwechsel über die Buttons).
    """

    def __init__(
        self,
        set_plane: Callable[[str], None],
        refresh: Callable[[], None],
        *,
        get_bounds: Optional[Callable[[], Bounds]] = None,
        set_bounds: Optional[Callable[[Bounds], None]] = None,
    ):
        # Callbacks zur View (Matplot2DView)
        self._set_plane = set_plane
        self._refresh = refresh

        # Optional: Zugriff auf Bounds
        self._get_bounds = get_bounds
        self._set_bounds = set_bounds

    def plane_top(self) -> None:
        """Wechselt zur Top-Ansicht (XY)."""
        self.switch_plane("top")

    def plane_front(self) -> None:
        """Wechselt zur Front-Ansicht (XZ)."""
        self.switch_plane("front")

    def plane_back(self) -> None:
        """Wechselt zur Back-Ansicht (XZ, nur andere Blickrichtung)."""
        self.switch_plane("back")

    def plane_left(self) -> None:
        """Wechselt zur Left-Ansicht (YZ)."""
        self.switch_plane("left")

    def plane_right(self) -> None:
        """Wechselt zur Right-Ansicht (YZ, nur andere Blickrichtung)."""
        self.switch_plane("right")

    def switch_plane(self, plane: str) -> None:
        """
        Setzt die Ebene und führt ein Refresh durch.
        """
        try:
            self._set_plane(plane)
            # Ein expliziter Refresh stellt sicher, dass die Änderung sofort sichtbar ist.
            self._refresh()
        except Exception:
            _LOG.exception("2D plane switch failed (plane=%s)", plane)