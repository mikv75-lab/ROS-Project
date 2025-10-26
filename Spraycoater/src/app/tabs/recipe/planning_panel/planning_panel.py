# -*- coding: utf-8 -*-
from __future__ import annotations
import os
from typing import Any, Callable, Dict, Optional

from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QMessageBox

# Pfad-Helfer
def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)


class PlanningPanel(QWidget):
    """
    RECHTES PANEL – Planner-Auswahl, Validate/Optimize/Save.
    Erwartet Provider-Callables:
      - model_provider() -> RecipeModel
      - traj_provider()  -> Optional[Dict]  (aus Preview)
    """
    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        uic.loadUi(_ui_path("planning_panel.ui"), self)

        self._model_provider: Optional[Callable[[], Any]] = None
        self._traj_provider: Optional[Callable[[], Optional[Dict[str, Any]]]] = None

        # Buttons
        if hasattr(self, "btnValidate"):
            self.btnValidate.clicked.connect(self._handle_validate)
        if hasattr(self, "btnOptimize"):
            self.btnOptimize.clicked.connect(self._handle_optimize)
        if hasattr(self, "btnSave"):
            self.btnSave.clicked.connect(self._handle_save)

        # Planner-Wahl optional
        if hasattr(self, "comboPlanner"):
            # falls du Plannerliste dynamisch füllen willst, tu das hier
            pass

        # Status
        self.set_preview_ready(False, "no preview yet")

    # --- Injection/Provider ---
    def set_bridge(self, bridge):
        self.bridge = bridge

    def set_model_provider(self, fn: Callable[[], Any]):
        self._model_provider = fn

    def set_traj_provider(self, fn: Callable[[], Optional[Dict[str, Any]]]):
        self._traj_provider = fn

    # --- Status/Ergebnisse ---
    def set_preview_ready(self, ok: bool, msg: str):
        if hasattr(self, "lblStatus"):
            self.lblStatus.setText("Preview OK" if ok else f"Preview Fehler: {msg}")
            self.lblStatus.setStyleSheet("color:#0a0;" if ok else "color:#a00;")
        if hasattr(self, "btnValidate"):
            self.btnValidate.setEnabled(ok)

    def show_results(self, obj: Dict[str, Any], ok: bool):
        if hasattr(self, "lblResult"):
            self.lblResult.setText("OK" if ok else "Fehler")
            self.lblResult.setStyleSheet("color:#0a0;" if ok else "color:#a00;")
        if hasattr(self, "txtDetails"):
            import json
            self.txtDetails.setPlainText(json.dumps(obj, indent=2, ensure_ascii=False))

    # --- Button-Handler ---
    def _handle_validate(self):
        planner = None
        if hasattr(self, "comboPlanner"):
            planner = self.comboPlanner.currentText() or None
        self._on_validate_clicked(planner or "default")

    def _handle_optimize(self):
        self._on_optimize_clicked()

    def _handle_save(self):
        self._on_save_clicked()

    # --- Aktionen (wie von dir gewünscht) ---
    def _on_validate_clicked(self, planner: str):
        try:
            if not self._model_provider:
                raise RuntimeError("Kein model_provider gesetzt.")
            model = self._model_provider()
            req = model.to_dict()
            req["planner"] = planner
            if self._traj_provider:
                traj = self._traj_provider()
                if traj:
                    req["trajectory"] = traj
            resp = self.bridge.validate(req, syntactic_only=False, timeout=10.0)
            ok = bool(getattr(resp, "ok", True))
            self.show_results({"validate": getattr(resp, "__dict__", {}) or str(resp)}, ok=ok)
        except Exception as e:
            self.show_results({"exception": str(e)}, ok=False)

    def _on_optimize_clicked(self):
        try:
            if not self._model_provider:
                raise RuntimeError("Kein model_provider gesetzt.")
            model = self._model_provider()
            resp = self.bridge.optimize(model.to_dict(), timeout=10.0)
            ok = bool(getattr(resp, "ok", True))
            self.show_results({"optimize": getattr(resp, "__dict__", {}) or str(resp)}, ok=ok)
        except Exception as e:
            self.show_results({"exception": str(e)}, ok=False)

    def _on_save_clicked(self):
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
            QMessageBox.information(self, "Gespeichert", f"Rezept gespeichert:\n{fname}")
        except Exception as e:
            QMessageBox.critical(self, "Speicherfehler", str(e))
