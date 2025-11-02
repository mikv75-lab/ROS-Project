# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
from typing import Optional, Callable, Any, Dict

from PyQt6 import uic
from PyQt6.QtWidgets import QWidget, QMessageBox, QTextEdit

_LOG = logging.getLogger("app.tabs.recipe.planning")


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))


def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)


class PlanningPanel(QWidget):
    """
    Minimaler Platzhalter:
      - Validate / Optimize: nur Log + MessageBox (keine ROS-Calls).
      - Save: noch kein Speichern, nur Info.
      - txtPreviewYaml wird via set_preview_yaml(text) befüllt.
    """
    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge  # aktuell ungenutzt (Platzhalter)

        uic.loadUi(_ui_path("planning_panel.ui"), self)

        # UI-Feld-Ref robust holen (falls der Objektname im UI anders wäre)
        self.txtPreviewYaml: Optional[QTextEdit] = getattr(self, "txtPreviewYaml", None)

        # Anbieter (werden vom RecipeTab gesetzt)
        self._model_provider: Optional[Callable[[], Any]] = None
        self._traj_provider: Optional[Callable[[], Dict[str, Any] | None]] = None

        # Buttons → Platzhalter
        self.btnValidate.clicked.connect(self._on_validate_clicked)
        self.btnOptimize.clicked.connect(self._on_optimize_clicked)
        self.btnSave.clicked.connect(self._on_save_clicked)

    # ---- Wiring vom RecipeTab ------------------------------------------------
    def set_model_provider(self, fn: Callable[[], Any]) -> None:
        self._model_provider = fn

    def set_traj_provider(self, fn: Callable[[], Dict[str, Any] | None]) -> None:
        self._traj_provider = fn

    def set_bridge(self, bridge) -> None:
        # Platzhalter: aktuell nicht genutzt
        self.bridge = bridge

    # ---- Vom PreviewPanel befüllt -------------------------------------------
    def set_preview_yaml(self, text: str) -> None:
        if self.txtPreviewYaml is not None:
            self.txtPreviewYaml.setPlainText(text or "")
        else:
            _LOG.warning("PlanningPanel: txtPreviewYaml not found in UI")

    # ---- Button-Handler (nur Platzhalter) -----------------------------------
    def _on_validate_clicked(self) -> None:
        _LOG.info("Validate (placeholder) triggered")
        QMessageBox.information(self, "Validate", "Validate ausgelöst (Platzhalter).")

    def _on_optimize_clicked(self) -> None:
        _LOG.info("Optimize (placeholder) triggered")
        QMessageBox.information(self, "Optimize", "Optimize ausgelöst (Platzhalter).")

    def _on_save_clicked(self) -> None:
        _LOG.info("Save (placeholder) triggered")
        # Beispiel: aktuellen Traj-Snapshot kurz in den Logs zeigen
        try:
            traj = self._traj_provider() if self._traj_provider else None
            _LOG.debug("Current traj snapshot: %s", "None" if traj is None else f"{traj.get('count', 0)} points")
        except Exception as e:
            _LOG.warning("Traj-Provider Fehler (placeholder): %s", e)

        # Model-Provider optional inspizieren
        if self._model_provider:
            try:
                model = self._model_provider()
                _LOG.debug("Model snapshot (placeholder): %s", getattr(model, "id", "<no id>"))
            except Exception as e:
                _LOG.warning("Model-Provider Fehler (placeholder): %s", e)

        QMessageBox.information(self, "Save", "Speichern folgt später (Platzhalter).")
