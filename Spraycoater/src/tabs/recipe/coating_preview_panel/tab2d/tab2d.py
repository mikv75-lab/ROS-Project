# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab2d/tab2d.py
from __future__ import annotations

import logging
from typing import Any, Callable, Optional, Tuple

import numpy as np
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

from .matplot2d import Matplot2DView
from .views_2d_box import Views2DBox
from .view_controller_2d import ViewController2D

_LOG = logging.getLogger("tabs.recipe.preview.tab2d")

Bounds = Tuple[float, float, float, float, float, float]


def _default_bounds() -> Bounds:
    # Conservative fallback (mm) – should be overridden by Tab3D bounds.
    return (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)


class Tab2D(QWidget):
    """2D preview tab.

    New filesystem rule (mirrors 3D refactor):
      - This tab owns ALL 2D preview logic and state.
      - The parent panel should only build the GUI and forward updates.

    Composition:
      - View: Matplot2DView (matplotlib canvas)
      - UI:   Views2DBox (plane buttons)
      - Logic: ViewController2D (connects UI -> view)
    """

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        refresh_callable: Optional[Callable[[], None]] = None,
        get_bounds_callable: Optional[Callable[[], Bounds]] = None,
    ) -> None:
        super().__init__(parent)

        # Optional global refresh (e.g. PV host render). If not provided, we refresh locally.
        self._global_refresh_callback = refresh_callable
        # Optional bounds provider (typically delegates to Tab3D). If not provided, we fallback.
        self._get_bounds_callback = get_bounds_callable

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(4, 4, 4, 4)
        self._layout.setSpacing(6)

        # 1) View
        self._view = Matplot2DView(parent=self)

        # 2) Controller (logic)
        self._controller = ViewController2D(
            set_plane=self._view.set_plane,
            refresh=self._on_local_refresh_needed,
            get_bounds=self._get_bounds_safe,
        )

        # 3) Controls (UI only)
        self._views_box = Views2DBox(
            switch_2d=self._controller.switch_plane,
            refresh_callable=self._on_local_refresh_needed,
            get_bounds=self._get_bounds_safe,
            parent=self,
        )

        self._set_policy(self._views_box, v=QSizePolicy.Policy.Preferred)
        self._layout.addWidget(self._views_box, 0)

        self._set_policy(self._view, v=QSizePolicy.Policy.Expanding)
        self._layout.addWidget(self._view, 1)

        # Initial empty scene
        self.clear_scene()

    # ---------------- Public API ----------------

    def clear_scene(self) -> None:
        """Reset the 2D view to an empty scene.

        The panel should call this when no recipe/preview is available.
        """
        try:
            self.update_scene(substrate_mesh=None, path_xyz=None, bounds=None)
        except Exception:
            _LOG.exception("Tab2D.clear_scene failed")

    def update_scene(
        self,
        *,
        substrate_mesh: Any,
        path_xyz: Optional[np.ndarray],
        bounds: Optional[Bounds],
    ) -> None:
        """Push new scene data into the 2D view.

        Contract (strict-ish):
          - path_xyz may be None -> treated as empty.
          - bounds may be None -> resolved via get_bounds_callable or fallback.
        """
        try:
            if path_xyz is None:
                path_xyz = np.zeros((0, 3), dtype=float)
            else:
                path_xyz = np.asarray(path_xyz, dtype=float).reshape(-1, 3)

            b = bounds if bounds is not None else self._get_bounds_safe()

            # Matplot2DView handles Z-offset relative to bounds[4] (zmin)
            self._view.set_scene(
                substrate_mesh=substrate_mesh,
                path_xyz=path_xyz,
                bounds=b,
            )
        except Exception:
            _LOG.exception("Tab2D.update_scene failed")

    # ---------------- Internals ----------------

    def _get_bounds_safe(self) -> Bounds:
        try:
            if callable(self._get_bounds_callback):
                b = self._get_bounds_callback()
                if isinstance(b, (tuple, list)) and len(b) == 6:
                    return tuple(float(x) for x in b)  # type: ignore[return-value]
        except Exception:
            # Bounds must never break 2D redraw.
            pass
        return _default_bounds()

    def _on_local_refresh_needed(self) -> None:
        """Called by controller/UI when user changes plane or asks for refresh."""
        try:
            if callable(self._global_refresh_callback):
                self._global_refresh_callback()
            else:
                self._view.refresh()
        except Exception:
            _LOG.exception("Tab2D refresh failed")

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
