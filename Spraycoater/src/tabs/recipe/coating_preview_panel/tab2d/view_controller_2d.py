# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab2d/view_controller_2d.py
from __future__ import annotations

from typing import Callable
import logging

_LOG = logging.getLogger("tabs.recipe.preview.views.controller2d")


class ViewController2D:
    """
    Strict 2D controller: plane switching only.
    """

    def __init__(
        self,
        *,
        set_plane: Callable[[str], None],
        refresh: Callable[[], None],
    ) -> None:
        self._set_plane = set_plane
        self._refresh = refresh

    def switch_plane(self, plane: str) -> None:
        """
        Set plane and refresh.
        """
        try:
            self._set_plane(plane)
            self._refresh()
        except Exception:
            _LOG.exception("2D plane switch failed (plane=%s)", plane)

    def plane_top(self) -> None:
        self.switch_plane("top")

    def plane_front(self) -> None:
        self.switch_plane("front")

    def plane_back(self) -> None:
        self.switch_plane("back")

    def plane_left(self) -> None:
        self.switch_plane("left")

    def plane_right(self) -> None:
        self.switch_plane("right")
