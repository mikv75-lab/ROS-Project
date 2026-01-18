# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/coating_preview_panel.py
from __future__ import annotations

import logging
from typing import Any, Optional, Tuple

import numpy as np
from PyQt6.QtCore import QTimer, pyqtSignal
from PyQt6.QtWidgets import QTabWidget, QVBoxLayout, QWidget, QSizePolicy
from PyQt6.sip import isdeleted

from model.recipe.recipe import Recipe
from model.recipe.recipe_store import RecipeStore
from widgets.info_groupbox import InfoGroupBox

from .tab2d.tab2d import Tab2D
from .tab3d.tab3d import Tab3D, PreviewResult


_LOG = logging.getLogger("tabs.recipe.preview.panel")

Bounds = Tuple[float, float, float, float, float, float]
_DEFAULT_BOUNDS: Bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)


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
    """GUI container for the coating preview.

    NEW FILESYSTEM RULE (strict):
      - Panel builds UI and debounces updates.
      - All preview logic lives in the tabs (Tab3D owns the pipeline; Tab2D is a pure consumer).
      - Panel only forwards (recipe, ctx) to Tab3D and propagates the resulting scene/path into Tab2D + Info.
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

        # SSoT for status API
        self._preview_valid: bool = False
        self._preview_invalid_reason: Optional[str] = "no_preview"
        self._final_tcp_world_mm: Optional[np.ndarray] = None
        self._last_preview: Optional[PreviewResult] = None

        # Debounced update timer
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._on_update_timer)

        self._build_ui()
        self._wire_signals()

        # Initial header state
        self._update_info(None, None)

    # ------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------

    def get_pv_host(self) -> QWidget:
        return self._tab3d.get_pv_host()

    def update_preview(self, model: Optional[Recipe]) -> None:
        self._recipe = model
        self.request_update()

    def request_update(self) -> None:
        if isdeleted(self):
            return
        # small debounce to coalesce many small edits (slider typing etc.)
        self._timer.start(30)

    def render(self) -> None:
        # Delegate rendering to Tab3D (PyVista plotter lives there)
        self._tab3d.render()

    def preview_is_valid(self) -> bool:
        return bool(self._preview_valid)

    def preview_invalid_reason(self) -> Optional[str]:
        return self._preview_invalid_reason

    def final_tcp_world_mm(self) -> Optional[np.ndarray]:
        if self._final_tcp_world_mm is None:
            return None
        return np.asarray(self._final_tcp_world_mm, dtype=float).reshape(-1, 3)

    # ------------------------------------------------------------
    # Bounds getters (consumed by 2D + view controllers)
    # ------------------------------------------------------------

    def get_bounds(self) -> Bounds:
        if self._last_preview is not None and isinstance(self._last_preview.bounds, (tuple, list)):
            try:
                b = tuple(float(x) for x in self._last_preview.bounds)
                if len(b) == 6:
                    return b  # type: ignore[return-value]
            except Exception:
                pass
        return _DEFAULT_BOUNDS

    def get_substrate_bounds(self) -> Optional[Bounds]:
        if self._last_preview is not None:
            return self._last_preview.substrate_bounds
        return None

    # ------------------------------------------------------------
    # UI helpers
    # ------------------------------------------------------------

    def _update_info(self, recipe: Optional[Recipe], points_mm: Optional[np.ndarray]) -> None:
        try:
            self._info_box.update_from_recipe(recipe, points_mm)
        except Exception:
            _LOG.exception("InfoGroupBox.update_from_recipe failed")

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

        # 2D tab consumes (mesh, path, bounds) and renders with Matplotlib.
        self._tab2d = Tab2D(
            parent=self,
            refresh_callable=self.render,
            get_bounds_callable=self.get_bounds,
        )
        self._tabs.addTab(self._tab2d, "2D View")

        # 3D tab owns the preview pipeline and rendering.
        self._tab3d = Tab3D(
            parent=self,
            render_callable=self.render,
            get_bounds_callable=self.get_bounds,
            get_substrate_bounds_callable=self.get_substrate_bounds,
            on_overlay_changed=self.request_update,
        )
        self._tabs.addTab(self._tab3d, "3D View")

        # Default to 3D.
        self._tabs.setCurrentIndex(1)

    def _wire_signals(self) -> None:
        self.sig_request_update.connect(self.request_update)

    # ------------------------------------------------------------
    # Persisted validity flag (recipe metadata)
    # ------------------------------------------------------------

    def _set_recipe_valid_save(self, recipe: Recipe, ok: bool, reason: Optional[str]) -> None:
        try:
            setattr(recipe, "validSave", bool(ok))
            setattr(recipe, "validSaveReason", (None if ok else str(reason or "invalid")))
            meta = getattr(recipe, "meta", None)
            if isinstance(meta, dict):
                meta["valid_save"] = bool(ok)
                meta["valid_save_reason"] = (None if ok else str(reason or "invalid"))
        except Exception:
            pass

    # ------------------------------------------------------------
    # Update pipeline (debounced)
    # ------------------------------------------------------------

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

        # Reset state
        self._preview_valid = False
        self._preview_invalid_reason = "no_preview"
        self._final_tcp_world_mm = None
        self._last_preview = None

        if recipe is None:
            try:
                self._tab2d.clear_scene()
            except Exception:
                pass
            try:
                self._tab3d.clear_layers()
            except Exception:
                pass
            self._update_info(None, None)
            self.render()
            return

        # 1) Let Tab3D compute+render preview (SSoT)
        res = self._tab3d.update_preview(recipe=recipe, ctx=self.ctx)
        self._last_preview = res

        # 2) Propagate status + path to consumers
        self._preview_valid = bool(res.valid)
        self._preview_invalid_reason = res.invalid_reason
        self._final_tcp_world_mm = res.final_tcp_world_mm

        # 3) Push to 2D (substrate + consumer path)
        try:
            self._tab2d.update_scene(
                substrate_mesh=res.substrate_mesh,
                path_xyz=res.path_xyz_mm,
                bounds=res.bounds,
            )
        except Exception:
            _LOG.exception("Tab2D.update_scene failed")

        # 4) Update header + recipe metadata
        try:
            self._update_info(recipe, res.path_xyz_mm)
        except Exception:
            pass
        self._set_recipe_valid_save(recipe, bool(res.valid), res.invalid_reason)

        # 5) Render
        self.render()
        self.sig_preview_updated.emit()
