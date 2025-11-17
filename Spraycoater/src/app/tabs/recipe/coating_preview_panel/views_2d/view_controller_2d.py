# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Callable, Optional, Tuple
import logging

_LOG = logging.getLogger("app.tabs.recipe.preview.views.controller2d")

Bounds = Tuple[float, float, float, float, float, float]


class ViewController2D:
    """Schlanker 2D-Controller für die Matplotlib-Ansicht."""
    def __init__(
        self,
        set_plane: Callable[[str], None],
        refresh: Callable[[], None],
        *,
        get_bounds: Optional[Callable[[], Bounds]] = None,
        set_bounds: Optional[Callable[[Bounds], None]] = None,
    ):
        self._set_plane = set_plane
        self._refresh = refresh
        self._get_bounds = get_bounds
        self._set_bounds = set_bounds

    def plane_top(self):   self._switch("top")
    def plane_front(self): self._switch("front")
    def plane_back(self):  self._switch("back")
    def plane_left(self):  self._switch("left")
    def plane_right(self): self._switch("right")

    def _switch(self, plane: str):
        # 1) Ebene umschalten
        try:
            self._set_plane(plane)
        except Exception:
            _LOG.exception("2D plane switch failed (set_plane): %s", plane)

        # 2) Optional: Bounds einfach so lassen wie sie sind.
        #    Falls du hier später wieder was mit get/set_bounds machen willst,
        #    können wir das gezielt und vorsichtig einbauen.

        # 3) Refresh
        #try:
        #    self._refresh()
        #except Exception:
        #    _LOG.exception("2D refresh failed after switch: %s", plane)
