# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab2d/tab2d.py
from __future__ import annotations

import logging
from typing import Optional, Any

import numpy as np
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

from .matplot2d import Matplot2DView
from .views_2d_box import Views2DBox
from .view_controller_2d import ViewController2D

_LOG = logging.getLogger("tabs.recipe.preview.tab2d")


class Tab2D(QWidget):
    """
    Strict 2D tab.

    Contract:
      - substrate_mesh: PyVista mesh in substrate-frame
      - path_xyz: FINAL path (postprocessed) in the same substrate-frame
    """

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        refresh_callable: Any = None,
    ) -> None:
        super().__init__(parent)

        self._refresh_callable = refresh_callable

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(4, 4, 4, 4)
        self._layout.setSpacing(6)

        # 1) View
        self.matplot_view = Matplot2DView(parent=self)

        # 2) Controller
        self.controller = ViewController2D(
            set_plane=self.matplot_view.set_plane,
            refresh=self._on_local_refresh_needed,
        )

        # 3) Controls
        self.views_box = Views2DBox(
            switch_2d=self.controller.switch_plane,
            refresh_callable=self._on_local_refresh_needed,
            parent=self,
        )

        # Layout
        self._set_policy(self.views_box, v=QSizePolicy.Policy.Preferred)
        self._layout.addWidget(self.views_box, 0)

        self._set_policy(self.matplot_view, v=QSizePolicy.Policy.Expanding)
        self._layout.addWidget(self.matplot_view, 1)

    def update_scene(
        self,
        *,
        substrate_mesh: Any,
        path_xyz: Optional[np.ndarray],
    ) -> None:
        """
        Push scene into 2D view (STRICT).
        """
        try:
            if path_xyz is None:
                path_xyz = np.zeros((0, 3), dtype=float)

            self.matplot_view.set_scene(
                substrate_mesh=substrate_mesh,
                path_xyz=path_xyz,
            )
        except Exception:
            _LOG.exception("Failed to update 2D scene")

    def _on_local_refresh_needed(self) -> None:
        """
        If a global refresh is available (main panel rebuild), use it; otherwise redraw locally.
        """
        if callable(self._refresh_callable):
            self._refresh_callable()
        else:
            self.matplot_view.refresh()

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
