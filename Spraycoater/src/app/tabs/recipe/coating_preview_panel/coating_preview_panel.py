# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Any, Dict, Iterable, List, Optional

from PyQt5.QtCore import pyqtSignal, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout
import sip

# Matplotlib erst importieren, wenn wir die Canvas wirklich bauen
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import numpy as np

from .path_builder import PathBuilder, PathData  # deine vorhandenen Klassen

_LOG = logging.getLogger("app.tabs.recipe.coating_preview_panel")


class CoatingPreviewPanel(QWidget):
    """
    2D-Preview (Top-Down) – robust gegen Tab-Wechsel:
      - MPL-Figure/Canvas wird erst bei der ersten Sichtbarkeit gebaut
      - Render-Requests werden gepuffert, wenn unsichtbar
      - Kein draw(), wenn Canvas/Widget disposed ist
    """
    readyChanged = pyqtSignal(bool, str)

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        self._mpl_ready = False
        self._fig: Optional[Figure] = None
        self._ax = None
        self._canvas: Optional[FigureCanvas] = None

        self._visible = False
        self._pending_render: Optional[Dict[str, Any]] = None
        self._render_timer = QTimer(self)
        self._render_timer.setSingleShot(True)
        self._render_timer.timeout.connect(self._render_now)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        self._last_traj: Optional[Dict[str, Any]] = None

    # ---------- Lifecycle ----------
    def showEvent(self, e):
        super().showEvent(e)
        self._visible = True
        self._ensure_mpl()
        if self._pending_render:
            # leicht verzögert, damit Layout fertig ist
            QTimer.singleShot(0, self._render_now)

    def hideEvent(self, e):
        super().hideEvent(e)
        self._visible = False
        self._render_timer.stop()

    def _ensure_mpl(self):
        if self._mpl_ready:
            return
        try:
            self._fig = Figure()
            self._canvas = FigureCanvas(self._fig)
            self.layout().addWidget(self._canvas)
            self._ax = self._fig.add_subplot(111)
            self._ax.set_aspect("equal")
            self._ax.grid(True)
            self._ax.set_xlabel("X [mm]")
            self._ax.set_ylabel("Y [mm]")
            self._mpl_ready = True
        except Exception as e:
            _LOG.critical("MPL init failed: %s", e, exc_info=True)
            self.readyChanged.emit(False, f"mpl init failed: {e}")

    # ---------- Public API ----------
    def render_from_model(self, model: Any, sides: Iterable[str]) -> None:
        """
        Kann jederzeit aufgerufen werden. Wenn das Panel (Canvas) noch nicht sichtbar/ready ist,
        wird der Render gespeichert und beim showEvent ausgeführt.
        """
        self._pending_render = {
            "model": model,
            "sides": list(sides),
        }
        if self._visible and self._mpl_ready:
            # Debounce: in 30 ms zeichnen (bündelt mehrere Updates)
            self._render_timer.start(30)

    def last_trajectory_dict(self) -> Optional[Dict[str, Any]]:
        return self._last_traj

    # ---------- Intern ----------
    def _render_now(self):
        if not self._pending_render:
            return
        if not self._visible or not self._mpl_ready:
            return
        if sip.isdeleted(self) or (self._canvas and sip.isdeleted(self._canvas)):
            return

        payload = self._pending_render
        self._pending_render = None

        model = payload["model"]
        sides = payload["sides"]

        try:
            # leere Achse
            self._ax.clear()
            self._ax.set_aspect("equal")
            self._ax.grid(True)
            self._ax.set_xlabel("X [mm]")
            self._ax.set_ylabel("Y [mm]")

            # jedes Side rendern
            any_ok = False
            for side in sides:
                try:
                    pd: PathData = PathBuilder.from_side(
                        recipe=model,
                        side=side,
                        sample_step_mm=self._get_param(model, "sample_step_mm", 1.0),
                        max_points=int(self._get_param(model, "max_points", 200_000)),
                    )
                    P2 = pd.points_mm[:, :2]
                    if len(P2) > 1:
                        self._ax.plot(P2[:, 0], P2[:, 1], linewidth=1.0)
                        any_ok = True
                except Exception as se:
                    _LOG.error("Side '%s' render failed: %s", side, se, exc_info=True)

            if any_ok:
                self._ax.relim()
                self._ax.autoscale()
                self._safe_draw()
                self.readyChanged.emit(True, "ok")
            else:
                # nichts gezeichnet, aber nicht crashen
                self._safe_draw()
                self.readyChanged.emit(False, "no path drawn")

        except Exception as e:
            _LOG.error("render_now failed: %s", e, exc_info=True)
            self.readyChanged.emit(False, str(e))

    def _safe_draw(self):
        try:
            if self._canvas and not sip.isdeleted(self._canvas):
                # draw() statt draw_idle() -> synchron & sicher nach Tabwechsel
                self._canvas.draw()
        except Exception as e:
            _LOG.error("canvas draw failed: %s", e, exc_info=True)

    @staticmethod
    def _get_param(model: Any, key: str, default: Any) -> Any:
        params = getattr(model, "parameters", None)
        if isinstance(model, dict):
            params = model.get("parameters", params)
        if isinstance(params, dict) and key in params:
            return params[key]
        return default
