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
    QMessageBox,
    QInputDialog,
    QSizePolicy,
)

from model.recipe.recipe import Recipe
from model.recipe.recipe_run_result import RunResult

# NOTE: recipe_markers moved under spray_paths (spraypaths package)
from model.spray_paths import recipe_markers

from .spray_path_box import SprayPathBox
from widgets.info_groupbox import InfoGroupBox

_LOG = logging.getLogger("tabs.process.recipe_panel")


class RecipePanel(QWidget):
    """Anzeige und Management von Rezepten und deren Ausführungsergebnissen (Strict V2)."""

    sig_recipe_selected = QtCore.pyqtSignal(str, object)
    sig_recipe_cleared = QtCore.pyqtSignal()

    def __init__(self, *, ctx: Any, repo: Any, ros: Any, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        if ctx is None:
            raise RuntimeError("RecipePanel: ctx is None (strict)")
        if repo is None:
            raise RuntimeError("RecipePanel: repo is None (strict)")
        if ros is None:
            raise RuntimeError("RecipePanel: ros is None (strict)")

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
        key = str(key or "").strip()
        if not key:
            raise ValueError("RecipePanel.set_recipe: empty key (strict)")
        if model is None:
            raise ValueError("RecipePanel.set_recipe: model is None (strict)")

        self._recipe_key = key
        self._recipe = model

        recipe_name = getattr(model, "id", None) or getattr(model, "key", None) or key
        self.lblRecipeName.setText(f"Recipe: {recipe_name}")

        self.txtRecipeParams.setPlainText(self._format_recipe_params(model))

        # STORED ist Disk-SSoT: beim Load immer refresh von Disk
        self._refresh_stored_from_disk(key)

        self._update_infobox_from_recipe(model)
        self.txtNewRun.clear()

        # spraypaths on recipe load (compiled + stored ghosts)
        try:
            self._republish_spraypaths_for_key(key, model)
        except Exception as e:
            _LOG.warning("RecipePanel: spraypath republish failed: %s", e)

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

    @QtCore.pyqtSlot(str, object, object)
    def on_run_finished(self, key: str, payload: object, rr_obj: object) -> None:
        """
        STRICT: must match ProcessPanel.sig_run_finished(str, object, object).

        ProcessPanel already:
          - rr = RunResult.from_process_payload(payload)
          - rr.postprocess(...)
          - persisted YAML (repo.bundle.paths)
        RecipePanel responsibilities:
          - display rr
          - refresh STORED from disk
          - republish overlays
        """
        key = str(key or "").strip()
        self.txtNewRun.setPlaceholderText("")

        if not key:
            self.txtNewRun.setPlainText("Run finished, but key is empty.")
            return

        if not isinstance(rr_obj, RunResult):
            self.txtNewRun.setPlainText(f"Run finished for {key}, but rr is not RunResult.")
            _LOG.warning("RecipePanel.on_run_finished: rr_obj invalid type=%s", type(rr_obj).__name__)
            return

        rr: RunResult = rr_obj

        # Show current run summary
        try:
            self.txtNewRun.setPlainText(rr.format_eval_text())
        except Exception as e:
            self.txtNewRun.setPlainText(f"Run finished for {key}, but format_eval_text failed: {e}")

        # Refresh disk SSoT + republish stored overlays
        try:
            self._refresh_stored_from_disk(key)
        except Exception as e:
            _LOG.warning("RecipePanel: refresh stored after run failed: %s", e)

        # Optional: republish NEW overlays immediately from rr TCP docs
        try:
            self._republish_newrun_overlays_from_rr(key, rr)
        except Exception as e:
            _LOG.warning("RecipePanel: republish newrun overlays failed: %s", e)

    @QtCore.pyqtSlot(str, str)
    def on_run_error(self, key: str, message: str) -> None:
        self.txtNewRun.setPlaceholderText("")
        self.txtNewRun.setPlainText(f"ERROR for {key}:\n{message}")
        _LOG.error("RecipePanel: Run error for %s: %s", key, message)

    # --- Interne Helfer ---

    def _refresh_stored_from_disk(self, key: str) -> None:
        key = str(key or "").strip()
        if not key:
            raise ValueError("_refresh_stored_from_disk: empty key")

        fresh = self.repo.load_for_process(key)
        self._recipe = fresh
        self._recipe_key = key
        self._update_stored_view(fresh)

        try:
            self._republish_spraypaths_for_key(key, fresh)
        except Exception as e2:
            _LOG.warning("RecipePanel: spraypath republish after disk refresh failed: %s", e2)

    def _ros_has(self, name: str) -> bool:
        return bool(self.ros is not None and hasattr(self.ros, name) and callable(getattr(self.ros, name)))

    def _republish_spraypaths_for_key(self, key: str, recipe_model: Recipe) -> None:
        """
        Republishes spray path layers to RViz:

        - Compiled: from current recipe model (draft)
        - Planned/Executed STORED: from disk (repo.load_for_process) as ghost markers

        Bridge remains unchanged; we capability-detect:
          - prefer ros.spray_set_planned(...stored_markers=...)
          - else fallback to legacy ros.spray_set_traj(markers=...)
        """
        key = str(key or "").strip()
        if not key or self.ros is None:
            return

        if self._ros_has("spray_clear"):
            try:
                self.ros.spray_clear()
            except Exception:
                pass

        # 1) Compiled (Draft)
        try:
            pa, ma = recipe_markers.build_draft_pose_and_markers(
                recipe_model,
                frame_id="substrate",
                ns_prefix="draft",
                clear_legacy=True,
                rgba_line=(0.0, 0.0, 1.0, 1.0),
                line_width_m=0.0008,
                round_style="none",
            )
            has_poses = bool(getattr(pa, "poses", None)) and len(pa.poses) > 0
            has_markers = bool(getattr(ma, "markers", None)) and len(ma.markers) > 0
            if has_poses or has_markers:
                if self._ros_has("spray_set_compiled"):
                    self.ros.spray_set_compiled(poses=pa if has_poses else None, markers=ma if has_markers else None)
        except Exception as e:
            _LOG.warning("RecipePanel: publish compiled failed: %s", e)

        # 2) STORED (Disk) ghosts
        recipe_disk = self.repo.load_for_process(key)

        # Planned stored
        try:
            planned_obj = getattr(recipe_disk, "planned_tcp", None)
            if planned_obj:
                _, ma = recipe_markers.build_tcp_pose_and_markers(
                    planned_obj,
                    ns_prefix="planned_tcp/stored",
                    mid_start=33000,
                    default_frame="substrate",
                    clear_legacy=True,
                    line_width_m=0.0005,
                    rgba_line=(0.7, 0.7, 0.7, 0.4),
                    round_style="none",
                )
                if getattr(ma, "markers", None) and len(ma.markers) > 0:
                    if self._ros_has("spray_set_planned"):
                        self.ros.spray_set_planned(stored_markers=ma)
                    elif self._ros_has("spray_set_traj"):
                        self.ros.spray_set_traj(markers=ma)
        except Exception as e:
            _LOG.warning("RecipePanel: publish planned stored failed: %s", e)

        # Executed stored
        try:
            executed_obj = getattr(recipe_disk, "executed_tcp", None)
            if executed_obj:
                _, ma = recipe_markers.build_tcp_pose_and_markers(
                    executed_obj,
                    ns_prefix="executed_tcp/stored",
                    mid_start=43000,
                    default_frame="substrate",
                    clear_legacy=True,
                    line_width_m=0.0005,
                    rgba_line=(0.7, 0.7, 0.7, 0.4),
                    round_style="none",
                )
                if getattr(ma, "markers", None) and len(ma.markers) > 0:
                    if self._ros_has("spray_set_executed"):
                        self.ros.spray_set_executed(stored_markers=ma)
        except Exception as e:
            _LOG.warning("RecipePanel: publish executed stored failed: %s", e)

    def _republish_newrun_overlays_from_rr(self, key: str, rr: RunResult) -> None:
        """
        Optional NEW overlays from rr TCP docs (already postprocessed in ProcessPanel).
        """
        if self.ros is None:
            return

        planned_tcp_doc = rr.planned_run.get("tcp") if isinstance(rr.planned_run, dict) else None
        executed_tcp_doc = rr.executed_run.get("tcp") if isinstance(rr.executed_run, dict) else None

        # Planned NEW
        try:
            if isinstance(planned_tcp_doc, dict) and planned_tcp_doc:
                _, ma = recipe_markers.build_tcp_pose_and_markers(
                    planned_tcp_doc,
                    ns_prefix="planned_tcp/new",
                    mid_start=23000,
                    default_frame="substrate",
                    clear_legacy=True,
                    line_width_m=0.0007,
                    rgba_line=(0.0, 1.0, 0.0, 0.8),
                    round_style="none",
                )
                if getattr(ma, "markers", None) and len(ma.markers) > 0:
                    if self._ros_has("spray_set_planned"):
                        self.ros.spray_set_planned(new_markers=ma)
                    elif self._ros_has("spray_set_traj"):
                        self.ros.spray_set_traj(markers=ma)
        except Exception as e:
            _LOG.warning("RecipePanel: publish planned newrun failed: %s", e)

        # Executed NEW
        try:
            if isinstance(executed_tcp_doc, dict) and executed_tcp_doc:
                _, ma = recipe_markers.build_tcp_pose_and_markers(
                    executed_tcp_doc,
                    ns_prefix="executed_tcp/new",
                    mid_start=53000,
                    default_frame="substrate",
                    clear_legacy=True,
                    line_width_m=0.0007,
                    rgba_line=(1.0, 0.0, 0.0, 0.8),
                    round_style="none",
                )
                if getattr(ma, "markers", None) and len(ma.markers) > 0:
                    if self._ros_has("spray_set_executed"):
                        self.ros.spray_set_executed(new_markers=ma)
        except Exception as e:
            _LOG.warning("RecipePanel: publish executed newrun failed: %s", e)

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
