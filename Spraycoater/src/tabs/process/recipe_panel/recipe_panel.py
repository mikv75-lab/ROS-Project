# -*- coding: utf-8 -*-
# File: src/tabs/process/recipe_panel/recipe_panel.py
from __future__ import annotations

import logging
from typing import Optional, Any

import numpy as np
from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QPushButton,
    QTextEdit,
    QLabel,
    QSplitter,
    QMessageBox,
    QInputDialog,
    QSizePolicy,
)

from model.recipe.recipe import Recipe
from model.recipe.recipe_run_result import RunResult

from .spray_path_box import SprayPathBox
from widgets.info_groupbox import InfoGroupBox

_LOG = logging.getLogger("tabs.process.recipe_panel")


class RecipePanel(QWidget):
    """Anzeige und Management von Rezepten und deren Ausführungsergebnissen (Strict V2)."""

    sig_recipe_selected = QtCore.pyqtSignal(str, object)
    sig_recipe_cleared = QtCore.pyqtSignal()

    def __init__(self, *, ctx: Any, repo: Any, ros: Any, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.ctx, self.repo, self.ros = ctx, repo, ros
        self._recipe: Optional[Recipe] = None
        self._recipe_key: Optional[str] = None

        self._build_ui()
        self.btnLoad.clicked.connect(self._on_load_clicked)

    # ---------------- UI ----------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        # ============================================================
        # Active Recipe
        # ============================================================
        self.RecipeGRP = QGroupBox("Active Recipe", self)
        vrec = QVBoxLayout(self.RecipeGRP)
        vrec.setContentsMargins(8, 8, 8, 8)
        vrec.setSpacing(8)

        top_row = QWidget(self.RecipeGRP)
        htop = QHBoxLayout(top_row)
        htop.setContentsMargins(0, 0, 0, 0)
        htop.setSpacing(10)

        self.btnLoad = QPushButton("Load Recipe", top_row)
        self.lblRecipeName = QLabel("Recipe: –", top_row)
        self.lblRecipeName.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

        htop.addWidget(self.btnLoad, 0)
        htop.addWidget(self.lblRecipeName, 1)
        vrec.addWidget(top_row, 0)

        self.grpRecipeParams = QGroupBox("Recipe Parameters", self.RecipeGRP)
        vparams = QVBoxLayout(self.grpRecipeParams)
        vparams.setContentsMargins(8, 8, 8, 8)
        vparams.setSpacing(6)

        self.txtRecipeParams = QTextEdit(self.grpRecipeParams)
        self.txtRecipeParams.setReadOnly(True)
        self.txtRecipeParams.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        vparams.addWidget(self.txtRecipeParams, 1)

        vrec.addWidget(self.grpRecipeParams, 1)
        root.addWidget(self.RecipeGRP, 2)

        # ============================================================
        # Middle: Info (left) + SprayPaths (right)
        # ============================================================
        mid = QWidget(self)
        hmid = QHBoxLayout(mid)
        hmid.setContentsMargins(0, 0, 0, 0)
        hmid.setSpacing(8)

        self.infoBox = InfoGroupBox(mid, title="Info")
        sp = self.infoBox.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Preferred)
        self.infoBox.setSizePolicy(sp)
        self.infoBox.setMinimumWidth(320)
        self.infoBox.setMaximumWidth(520)

        self.sprayPathBox = SprayPathBox(ros=self.ros, parent=mid)
        sp2 = self.sprayPathBox.sizePolicy()
        sp2.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        self.sprayPathBox.setSizePolicy(sp2)

        hmid.addWidget(self.infoBox, 0)
        hmid.addWidget(self.sprayPathBox, 1)
        root.addWidget(mid, 0)

        # ============================================================
        # Bottom: STORED vs CURRENT (each in a named GroupBox)
        # ============================================================
        bottom = QWidget(self)
        hbot = QHBoxLayout(bottom)
        hbot.setContentsMargins(0, 0, 0, 0)
        hbot.setSpacing(8)

        self.grpStored = QGroupBox("Stored Eval (Disk)", bottom)
        vstored = QVBoxLayout(self.grpStored)
        vstored.setContentsMargins(8, 8, 8, 8)
        self.txtStored = QTextEdit(self.grpStored)
        self.txtStored.setReadOnly(True)
        vstored.addWidget(self.txtStored, 1)

        self.grpCurrent = QGroupBox("Current Run (Live)", bottom)
        vcur = QVBoxLayout(self.grpCurrent)
        vcur.setContentsMargins(8, 8, 8, 8)
        self.txtNewRun = QTextEdit(self.grpCurrent)
        self.txtNewRun.setReadOnly(True)
        vcur.addWidget(self.txtNewRun, 1)

        hbot.addWidget(self.grpStored, 1)
        hbot.addWidget(self.grpCurrent, 1)
        root.addWidget(bottom, 2)

    # --------------- Recipe binding ----------------

    def set_recipe(self, key: str, model: Recipe) -> None:
        self._recipe_key = key
        self._recipe = model

        recipe_name = getattr(model, "id", None) or getattr(model, "key", None) or key
        self.lblRecipeName.setText(f"Recipe: {recipe_name}")

        self.txtRecipeParams.setPlainText(self._format_recipe_params(model))

        # STORED ist Disk-SSoT: beim Load immer refresh von Disk
        self._refresh_stored_from_disk(key)

        self._update_infobox_from_recipe(model)

        self.txtNewRun.clear()
        self.sig_recipe_selected.emit(key, model)

    def _format_recipe_params(self, model: Recipe) -> str:
        try:
            d = model.to_params_dict()
        except Exception:
            d = {"repr": repr(model)}

        def fmt(obj: Any, indent: int = 0) -> str:
            pad = " " * indent
            if isinstance(obj, dict):
                if not obj:
                    return f"{pad}{{}}"
                lines = []
                for k in sorted(obj.keys(), key=lambda x: str(x)):
                    v = obj[k]
                    if isinstance(v, (dict, list, tuple)):
                        lines.append(f"{pad}{k}:")
                        lines.append(fmt(v, indent + 2))
                    else:
                        lines.append(f"{pad}{k}: {v}")
                return "\n".join(lines)
            if isinstance(obj, (list, tuple)):
                if not obj:
                    return f"{pad}[]"
                lines = []
                for v in obj:
                    if isinstance(v, (dict, list, tuple)):
                        lines.append(f"{pad}-")
                        lines.append(fmt(v, indent + 2))
                    else:
                        lines.append(f"{pad}- {v}")
                return "\n".join(lines)
            return f"{pad}{obj}"

        return fmt(d, 0)

    # --- Slots für den Prozess-Lifecycle (Strict V2) ---

    @QtCore.pyqtSlot(str, str)
    def on_run_started(self, mode: str, key: str) -> None:
        self.txtNewRun.clear()
        self.txtNewRun.setPlaceholderText(f"Running {mode} for {key}...")
        _LOG.info("RecipePanel: Run started (mode=%s, key=%s)", mode, key)

    @QtCore.pyqtSlot(str, object)
    def on_run_finished(self, key: str, rr: RunResult) -> None:
        if not self._recipe:
            return

        try:
            self.txtNewRun.setPlaceholderText("")

            rr.postprocess(
                recipe=self._recipe,
                segment_order=("MOVE_PREDISPENSE", "MOVE_RECIPE", "MOVE_RETREAT"),
                tcp_target_frame="substrate",
            )

            success = self.repo.save_run_result_if_valid(key, run_result=rr)

            self.txtNewRun.setPlainText(rr.format_eval_text())

            if success:
                _LOG.info("RunResult persistiert (Disk ggf. überschrieben). Refresh STORED aus Disk.")
                self._refresh_stored_from_disk(key)

        except Exception as e:
            _LOG.exception("Fehler im Post-Processing")
            self.txtNewRun.setPlainText(f"Post-Processing Error: {e}")
            QMessageBox.warning(self, "Postprocess Error", str(e))

    @QtCore.pyqtSlot(str, str)
    def on_run_error(self, key: str, message: str) -> None:
        self.txtNewRun.setPlaceholderText("")
        self.txtNewRun.setPlainText(f"ERROR for {key}:\n{message}")
        _LOG.error("RecipePanel: Run error for %s: %s", key, message)

    # --- Interne Helfer ---

    def _refresh_stored_from_disk(self, key: str) -> None:
        try:
            fresh = self.repo.load_for_process(key)
            self._recipe = fresh
            self._recipe_key = key
            self._update_stored_view(fresh)
        except Exception as e:
            _LOG.exception("Failed to refresh STORED from disk (key=%s)", key)
            self.txtStored.setPlainText(f"=== STORED EVAL (Disk) ===\nERROR reloading: {e}")

    def _update_infobox_from_recipe(self, recipe: Recipe) -> None:
        pts = None
        for attr in ("compiled_draft", "draft", "compiled"):
            d = getattr(recipe, attr, None)
            if d is None:
                continue
            p = getattr(d, "points_mm", None)
            if p is None:
                continue
            try:
                arr = np.asarray(p, dtype=float).reshape(-1, 3)
                if arr.size > 0:
                    pts = arr
                    break
            except Exception:
                continue
        self.infoBox.update_from_recipe(recipe, points=pts)

    def _update_stored_view(self, recipe: Recipe) -> None:
        lines = ["=== STORED EVAL (Disk) ==="]
        found = False

        for mode, draft in [
            ("Planned", getattr(recipe, "planned_tcp", None)),
            ("Executed", getattr(recipe, "executed_tcp", None)),
        ]:
            if draft:
                eval_data = getattr(draft, "eval", None)
                if eval_data:
                    lines.append(f"{mode}: {str(eval_data)}")
                else:
                    lines.append(f"{mode}: (no eval)")
                found = True

        if not found:
            lines.append("(No stored evaluations found)")

        self.txtStored.setPlainText("\n".join(lines))

    def _on_load_clicked(self) -> None:
        keys = self.repo.list_recipes()
        choice, ok = QInputDialog.getItem(self, "Load", "Select Recipe:", keys, 0, False)
        if ok and choice:
            model = self.repo.load_for_process(choice)
            self.set_recipe(choice, model)
