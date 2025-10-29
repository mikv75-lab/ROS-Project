# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import json
from typing import Any, Callable, Dict, Optional

from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QMessageBox, QLabel

# Pfad-Helfer
def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)

def _rviz_cfg_path() -> str:
    return os.path.join(_project_root(), "resource", "rviz", "live.rviz")


class PlanningPanel(QWidget):
    """
    Rechtes Panel, minimal:
      - Buttons: Optimize, Save
      - Ausgabe in txtResults
      - Zeigt RViz-Config-Pfad im lblRvizPlaceholder
    Erwartete Provider (optional):
      - model_provider() -> RecipeModel (mit .to_dict())
      - traj_provider()  -> Optional[Dict]  (Traj-Daten aus Preview)
    """
    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        uic.loadUi(_ui_path("planning_panel.ui"), self)

        self._model_provider: Optional[Callable[[], Any]] = None
        self._traj_provider: Optional[Callable[[], Optional[Dict[str, Any]]]] = None

        # Buttons verdrahten (nur die, die im UI existieren)
        if hasattr(self, "btnOptimize"):
            self.btnOptimize.clicked.connect(self._handle_optimize)
        if hasattr(self, "btnSave"):
            self.btnSave.clicked.connect(self._handle_save)

        # RViz-Hinweis im Placeholder anzeigen
        self._show_rviz_config_hint()

        # Initialer Hinweis
        self._write_results({"status": "ready", "hint": "No results yet."})

    # --- Injection/Provider ---
    def set_bridge(self, bridge):
        self.bridge = bridge

    def set_model_provider(self, fn: Callable[[], Any]):
        self._model_provider = fn

    def set_traj_provider(self, fn: Callable[[], Optional[Dict[str, Any]]]):
        self._traj_provider = fn

    # --- UI helpers ---
    def _write_results(self, obj: Dict[str, Any]):
        if hasattr(self, "txtResults") and self.txtResults:
            self.txtResults.setPlainText(json.dumps(obj, indent=2, ensure_ascii=False))

    def _show_rviz_config_hint(self):
        label: QLabel = getattr(self, "lblRvizPlaceholder", None)
        cfg = _rviz_cfg_path()
        if not isinstance(label, QLabel):
            return
        if os.path.exists(cfg):
            label.setText(f"RViz config:\n{cfg}")
            label.setStyleSheet("color: #0a0;")
        else:
            label.setText(f"RViz config fehlt:\n{cfg}")
            label.setStyleSheet("color: #b00020;")

    # --- Button-Handler ---
    def _handle_optimize(self):
        try:
            if not self._model_provider:
                raise RuntimeError("Kein model_provider gesetzt.")
            model = self._model_provider()
            payload = model.to_dict()
            if self._traj_provider:
                traj = self._traj_provider()
                if traj:
                    payload["trajectory"] = traj
            resp = self.bridge.optimize(payload, timeout=10.0)
            ok = bool(getattr(resp, "ok", True))
            self._write_results({"action": "optimize", "ok": ok, "response": getattr(resp, "__dict__", {}) or str(resp)})
        except Exception as e:
            self._write_results({"action": "optimize", "ok": False, "exception": str(e)})

    def _handle_save(self):
        try:
            if not self._model_provider:
                raise RuntimeError("Kein model_provider gesetzt.")
            model = self._model_provider()
            rec = model.to_dict()
            rid = rec.get("id") or "recipe"
            default_name = os.path.join(self.ctx.paths.recipe_dir, f"{rid}.yaml")
            from PyQt5.QtWidgets import QFileDialog
            fname, _ = QFileDialog.getSaveFileName(self, "Rezept speichern", default_name, "YAML (*.yaml *.yml)")
            if not fname:
                return
            import yaml
            with open(fname, "w", encoding="utf-8") as f:
                yaml.safe_dump(rec, f, allow_unicode=True, sort_keys=False)
            self._write_results({"action": "save", "ok": True, "file": fname})
            QMessageBox.information(self, "Gespeichert", f"Rezept gespeichert:\n{fname}")
        except Exception as e:
            self._write_results({"action": "save", "ok": False, "exception": str(e)})
            QMessageBox.critical(self, "Speicherfehler", str(e))
