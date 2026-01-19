# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/coating_preview_panel.py
from __future__ import annotations

import logging
from typing import Any, Optional, Tuple

import numpy as np

from PyQt6.QtCore import QTimer, pyqtSignal
from PyQt6.QtWidgets import QTabWidget, QVBoxLayout, QWidget, QSizePolicy
from PyQt6.sip import isdeleted

# --- Models & Widgets ---
from model.recipe.recipe import Recipe
from model.recipe.recipe_store import RecipeStore
from widgets.info_groupbox import InfoGroupBox

# --- UI Sub-Components (Tabs) ---
from .tab2d.tab2d import Tab2D
from .tab3d.tab3d import Tab3D

_LOG = logging.getLogger("tabs.recipe.preview.panel")

Bounds = Tuple[float, float, float, float, float, float]


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


class CoatingPreviewPanel(QWidget):
    """UI wrapper for the coating preview.

    IMPORTANT:
      - Tab3D is the single source of truth for preview build.
      - Tab2D is fed STRICTLY with:
          substrate_mesh (substrate-frame mesh) + final path_xyz (substrate-frame)
        i.e. NO bounds, NO mask, NO tcp.

    Layout:
      - InfoGroupBox (header)
      - QTabWidget (2D / 3D)
    """

    sig_request_update = pyqtSignal()
    sig_preview_updated = pyqtSignal()

    def __init__(
        self,
        *,
        ctx: Any,
        store: RecipeStore,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)

        self.ctx = ctx
        if store is None or not isinstance(store, RecipeStore):
            raise TypeError(f"CoatingPreviewPanel: store invalid: {type(store)}")
        self.store: RecipeStore = store

        self._recipe: Optional[Recipe] = None
        self._busy = False

        # Status (mirrors Tab3D latest result)
        self._preview_valid: bool = False
        self._preview_invalid_reason: Optional[str] = "no_preview"
        self._final_tcp_world_mm: Optional[np.ndarray] = None

        # Update Timer (debounce)
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._on_update_timer)

        # UI
        self._build_ui()
        self._wire_signals()

        # Initial reset
        self._update_info(None, None)

    # ---------------- Public API ----------------

    def get_pv_host(self) -> QWidget:
        """
        RecipeTab expects this.
        We delegate to Tab3D; Tab3D must expose pv_host (and ideally get_pv_host()).
        """
        try:
            m = getattr(self._tab3d, "get_pv_host", None)
            if callable(m):
                return m()
        except Exception:
            pass

        pv_host = getattr(self._tab3d, "pv_host", None)
        if isinstance(pv_host, QWidget):
            return pv_host

        raise RuntimeError("CoatingPreviewPanel: Tab3D has no pv_host/get_pv_host()")

    def update_preview(self, model: Optional[Recipe]) -> None:
        self._recipe = model
        self.request_update()

    def request_update(self) -> None:
        if isdeleted(self):
            return
        self._timer.start(30)

    def render(self) -> None:
        self._tab3d.render()

    def preview_is_valid(self) -> bool:
        return bool(self._preview_valid)

    def preview_invalid_reason(self) -> Optional[str]:
        return self._preview_invalid_reason

    def final_tcp_world_mm(self) -> Optional[np.ndarray]:
        if self._final_tcp_world_mm is None:
            return None
        return np.asarray(self._final_tcp_world_mm, dtype=float).reshape(-1, 3)

    # ---------------- Bounds Getters (Tab3D SSoT) ----------------

    def get_bounds(self) -> Bounds:
        try:
            return self._tab3d.get_bounds()
        except Exception:
            return (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

    def get_substrate_bounds(self) -> Optional[Bounds]:
        try:
            return self._tab3d.get_substrate_bounds()
        except Exception:
            return None

    # ---------------- Info box ----------------

    def _update_info(self, recipe: Optional[Recipe], points_mm: Optional[np.ndarray]) -> None:
        try:
            self._info_box.update_from_recipe(recipe, points_mm)
        except Exception:
            _LOG.exception("InfoGroupBox.update_from_recipe failed")

    # ---------------- UI Construction ----------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # 1) Info Header
        self._info_box = InfoGroupBox(parent=self, title="Preview")
        _set_policy(self._info_box, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        root.addWidget(self._info_box, 0)

        # 2) Tabs
        self._tabs = QTabWidget(self)
        root.addWidget(self._tabs, 1)

        # Always define members BEFORE use
        self._tab3d = QWidget(self)
        self._tab2d = QWidget(self)

        # Tab3D (authoritative preview build)
        try:
            self._tab3d = Tab3D(
                parent=self,
                render_callable=None,  # avoid recursion
            )
        except Exception:
            _LOG.exception("CoatingPreviewPanel: Tab3D construction failed")
            self._tab3d = QWidget(self)

        # Tab2D (STRICT: only substrate + final path)
        try:
            self._tab2d = Tab2D(
                parent=self,
                refresh_callable=self.render,
            )
        except Exception:
            _LOG.exception("CoatingPreviewPanel: Tab2D construction failed")
            self._tab2d = QWidget(self)

        self._tabs.addTab(self._tab2d, "2D View")
        self._tabs.addTab(self._tab3d, "3D View")
        self._tabs.setCurrentIndex(1)

    def _wire_signals(self) -> None:
        self.sig_request_update.connect(self.request_update)

    # ---------------- ValidSave flag (STRICT SSoT) ----------------

    @staticmethod
    def _ensure_info(recipe: Recipe) -> None:
        info = getattr(recipe, "info", None)
        if not isinstance(info, dict):
            recipe.info = {}
        if "validSave" not in recipe.info:
            recipe.info["validSave"] = False
        if "validSaveReason" not in recipe.info:
            recipe.info["validSaveReason"] = "no_preview"

    def _set_recipe_valid_save(self, recipe: Recipe, ok: bool, reason: Optional[str]) -> None:
        try:
            self._ensure_info(recipe)
            recipe.info["validSave"] = bool(ok)
            recipe.info["validSaveReason"] = (None if ok else str(reason or "invalid"))
        except Exception:
            pass

    # ---------------- Update pipeline ----------------

    def _on_update_timer(self) -> None:
        if self._busy:
            return
        self._busy = True
        try:
            self._update_preview()
        except Exception:
            _LOG.exception("update_preview failed")
        finally:
            self._busy = False

    def _update_preview(self) -> None:
        recipe = self._recipe

        # Reset status
        self._preview_valid = False
        self._preview_invalid_reason = "no_preview"
        self._final_tcp_world_mm = None

        # Reset tabs (STRICT 2D API)
        try:
            self._tab2d.update_scene(substrate_mesh=None, path_xyz=None)
        except Exception:
            pass
        try:
            self._tab3d.clear_layers()
        except Exception:
            pass

        if recipe is None:
            self._update_info(None, None)
            self.render()
            return

        self._ensure_info(recipe)

        # 1) Build preview in Tab3D
        res = self._tab3d.update_preview(recipe=recipe, ctx=self.ctx)

        # 2) Mirror status
        self._preview_valid = bool(getattr(res, "valid", False))
        self._preview_invalid_reason = getattr(res, "invalid_reason", None)
        self._final_tcp_world_mm = getattr(res, "final_tcp_world_mm", None)

        # 3) Update 2D view + info header
        # STRICT: 2D gets ONLY substrate + FINAL path (postprocess)
        path_xyz = getattr(res, "final_tcp_world_mm", None)  # final path (substrate-frame per your SceneManager)
        substrate_mesh = getattr(res, "substrate_mesh", None)  # must also be substrate-frame for 2D correctness

        try:
            self._tab2d.update_scene(substrate_mesh=substrate_mesh, path_xyz=path_xyz)
        except Exception:
            _LOG.exception("Tab2D.update_scene failed")

        self._update_info(recipe, path_xyz)
        self._set_recipe_valid_save(recipe, self._preview_valid, self._preview_invalid_reason)

        # 4) Render and notify
        self.render()
        self.sig_preview_updated.emit()
