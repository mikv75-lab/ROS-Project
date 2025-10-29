# Spraycoater/src/app/tabs/service/service_tab.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QLabel

_LOG = logging.getLogger("app.tabs.service")


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/app/tabs/service
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))


def _ui_path(filename: str) -> str:
    # Struktur: resource/ui/tabs/service/service_tab.ui
    return os.path.join(_project_root(), "resource", "ui", "tabs", "service", filename)


def _rviz_cfg_path() -> str:
    # live.rviz liegt in resource/rviz/
    return os.path.join(_project_root(), "resource", "rviz", "live.rviz")


class ServiceTab(QWidget):
    """
    Minimaler Service-Tab:
      - Keine Start/Stop-Buttons für RViz.
      - Zeigt den Pfad zur RViz-Config im RViz-Container-Platzhalter (lblRvizHint).
      - Läd ein schlichtes UI ohne Spezial-Properties.
    """

    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        ui_file = _ui_path("service_tab.ui")
        if not os.path.exists(ui_file):
            _LOG.error("ServiceTab UI nicht gefunden: %s", ui_file)
        uic.loadUi(ui_file, self)

        self._cfg = _rviz_cfg_path()
        self._show_config_in_container()

    # ---------- intern ----------

    def _show_config_in_container(self) -> None:
        """
        Schreibt die RViz-Config-Info in den Container-Platzhalter.
        Erwartet im UI ein QLabel mit objectName 'lblRvizHint' innerhalb 'rvizContainer'.
        """
        label: QLabel = self.findChild(QLabel, "lblRvizHint")
        if label is None:
            _LOG.warning("lblRvizHint nicht gefunden – nichts anzuzeigen.")
            return

        if os.path.exists(self._cfg):
            label.setText(f"RViz config:\n{self._cfg}")
            # dezente Anzeige
            label.setStyleSheet("")
        else:
            label.setText(f"RViz config fehlt:\n{self._cfg}")
            # rot markieren, wenn fehlt
            label.setStyleSheet("color: #b00020;")
