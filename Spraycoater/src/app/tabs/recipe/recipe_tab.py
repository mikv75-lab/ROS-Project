# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Callable, Any, Dict

import numpy as np
from PyQt6.QtWidgets import QWidget, QHBoxLayout
from PyQt6.QtCore import Qt

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel

_LOG = logging.getLogger("app.tabs.recipe")


class RecipeTab(QWidget):
    def __init__(self, *, ctx, store, bridge, attach_preview_widget: Callable[[QWidget], None],
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.store = store
        self.bridge = bridge
        self._attach_preview_widget = attach_preview_widget

        # Interner Traj-/Meta-Puffer (ohne PlanningPanel)
        self._last_traj: Optional[Dict[str, Any]] = None
        self._last_yaml: str = ""
        self._info_base: Dict[str, float] = {}   # speed/flow/pre/post cache

        # Layout
        hroot = QHBoxLayout(self)
        hroot.setContentsMargins(6, 6, 6, 6)
        hroot.setSpacing(8)

        # Links: Recipe-Editor
        self.recipePanel = RecipeEditorPanel(ctx=self.ctx, store=self.store, parent=self)
        hroot.addWidget(self.recipePanel, 0)

        # Mitte/Rechts: Preview (bekommt Store)
        self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, store=self.store, parent=self)
        hroot.addWidget(self.previewPanel, 1)

        # Interactor aus dem MainWindow übernehmen
        self._attach_preview_widget(self.previewPanel.preview_host())
        try:
            win = self.window() or self.parent()
            if win is not None and hasattr(win, "previewPlot") and win.previewPlot is not None:
                self.previewPanel.attach_interactor(win.previewPlot)
        except Exception:
            _LOG.exception("RecipeTab: adopt previewPlot into previewPanel failed")

        # --- Wiring: Recipe → Preview (Render anstoßen) --------------------
        self.recipePanel.updatePreviewRequested.connect(
            lambda payload: self._on_update_preview(payload.get("model"), payload.get("sides")),
            Qt.ConnectionType.QueuedConnection
        )

        # --- Preview → interne Puffer -------------------------------------
        try:
            self.previewPanel.previewYamlReady.connect(self._on_preview_yaml_ready)
            self.previewPanel.previewYamlReady.connect(lambda *_: self._on_preview_event())
        except Exception:
            _LOG.exception("connect previewYamlReady failed")

        try:
            self.previewPanel.pathReady.connect(self._on_path_ready)
            self.previewPanel.pathReady.connect(lambda *_: self._on_preview_event())
        except Exception:
            _LOG.exception("connect pathReady failed")

    # ---------- Slots ----------
    def _on_update_preview(self, model_or_payload: object, sides: Optional[list] = None) -> None:
        """Nur Übergabe: Param-Cache setzen (für Info-Berechnung) + SceneManager rendern lassen."""
        try:
            model = model_or_payload
            if isinstance(model_or_payload, dict):
                model = model_or_payload.get("model", model_or_payload)
                sides = model_or_payload.get("sides", sides)

            # Param-Cache für Info-Berechnung (ETA/Medium) – bleibt hier
            try:
                params = dict(getattr(model, "parameters", {}) or {})
            except Exception:
                params = {}
            import math
            def _f(x, default=np.nan):
                try:
                    return float(x)
                except Exception:
                    return float(default)
            self._info_base = {
                "speed_mm_s":      _f(params.get("speed_mm_s")),
                "flow_ml_min":     _f(params.get("flow_ml_min")),
                "pre_dispense_s":  _f(params.get("pre_dispense_s"), 0.0),
                "post_dispense_s": _f(params.get("post_dispense_s"), 0.0),
            }

            # Orchestrierung vollständig im SceneManager
            self.previewPanel.scene.render_recipe_preview(panel=self.previewPanel, model=model, sides=sides)
        except Exception:
            _LOG.exception("_on_update_preview failed")

    def _on_preview_yaml_ready(self, text: str) -> None:
        try:
            self._last_yaml = text or ""
        except Exception:
            _LOG.exception("_on_preview_yaml_ready failed")

    def _on_path_ready(self, path_xyz: Optional[np.ndarray]) -> None:
        """
        Aktualisiert Traj-Puffer und berechnet InfoPanel-Kennzahlen:
          - Points
          - Path length (mm)
          - ETA (s) = length / speed_mm_s
          - Medium (ml) = flow_ml_min * (ETA + pre + post) / 60
        """
        try:
            if path_xyz is None:
                self._last_traj = {"points_mm": [], "count": 0}
                self.previewPanel.set_runtime_info({
                    "points": 0,
                    "length_mm": 0.0,
                    "eta_s": None,
                    "medium_ml": None,
                })
                return

            P = np.asarray(path_xyz, dtype=float).reshape(-1, 3)
            npts = int(P.shape[0])
            length = 0.0
            if npts >= 2:
                d = np.linalg.norm(P[1:] - P[:-1], axis=1)
                length = float(d.sum())

            self._last_traj = {"points_mm": P.tolist(), "count": npts}

            speed = self._info_base.get("speed_mm_s")
            eta_s = (length / float(speed)) if (isinstance(speed, (int, float)) and speed and speed > 1e-12) else None

            flow = self._info_base.get("flow_ml_min")
            pre  = float(self._info_base.get("pre_dispense_s", 0.0) or 0.0)
            post = float(self._info_base.get("post_dispense_s", 0.0) or 0.0)
            total_s = (eta_s or 0.0) + pre + post
            medium_ml = (float(flow) * (total_s / 60.0)) if (isinstance(flow, (int, float)) and flow and flow > 0.0) else None

            self.previewPanel.set_runtime_info({
                "points": npts,
                "length_mm": length,
                "eta_s": eta_s,
                "medium_ml": medium_ml,
            })
        except Exception:
            _LOG.exception("_on_path_ready failed")

    def _on_preview_event(self) -> None:
        """Aktuelle Ansicht neu zeichnen (kein View-Wechsel, kein harter Reset)."""
        try:
            self.previewPanel.refresh_current_view(hard_reset=False)
        except Exception:
            _LOG.exception("preview refresh failed")
