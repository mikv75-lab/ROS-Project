# -*- coding: utf-8 -*-
# File: src/tabs/process/recipe_panel/recipe_panel.py
from __future__ import annotations

import os
import logging
from typing import Optional, Any, Dict, List, Tuple

import yaml
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
    QSizePolicy,
    QSplitter,
    QMessageBox,
    QInputDialog,
)
from PyQt6.QtCore import Qt

from widgets.info_groupbox import InfoGroupBox
from .spray_path_box import SprayPathBox

from model.recipe.recipe import Recipe
from model.recipe.recipe_run_result import RunResult
from config.startup import resolve_path

_LOG = logging.getLogger("tabs.process.recipe_panel")


class RecipePanel(QWidget):
    """
    RecipePanel (Logic + UI).
    
    Verantwortlichkeiten:
      - Laden von Rezepten.
      - Anzeige Draft/Meta.
      - Split-View unten: "Stored Result" (vom Datenträger) vs. "New Run Result" (aktueller Lauf).
      - Beide Views zeigen kombiniert Planned + Executed Eval an.
    """

    sig_recipe_selected = QtCore.pyqtSignal(str, object)
    sig_recipe_cleared = QtCore.pyqtSignal()

    def __init__(self, *, ctx: Any, repo: Any, ros: Any, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        if ctx is None or repo is None or ros is None:
            raise RuntimeError("RecipePanel: ctx, repo, ros must not be None (strict)")

        self.ctx = ctx
        self.repo = repo
        self.ros = ros

        self._recipe_key: Optional[str] = None
        self._recipe: Optional[Recipe] = None

        self._build_ui()
        
        self.btnLoad.clicked.connect(self._on_load_clicked)
        
        # Initial Reset
        self._update_recipe_ui()
        self._clear_result_ui()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        # ------------------------------------------------------------
        # 1. Top: RecipeGRP (Load, Key, Meta, Draft)
        # ------------------------------------------------------------
        self.RecipeGRP = QGroupBox("Recipe", self)
        vrec = QVBoxLayout(self.RecipeGRP)
        vrec.setSpacing(6)

        row = QHBoxLayout()
        row.setSpacing(8)
        self.btnLoad = QPushButton("Load", self.RecipeGRP)
        self.btnLoad.setMinimumHeight(28)
        self.lblRecipeKey = QLabel("Key: –", self.RecipeGRP)
        self.lblRecipeKey.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        
        row.addWidget(self.btnLoad, 0)
        row.addWidget(self.lblRecipeKey, 1)
        vrec.addLayout(row)

        self.lblRecipeMeta = QLabel("Info: –", self.RecipeGRP)
        self.lblRecipeMeta.setWordWrap(True)
        self.lblRecipeMeta.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        vrec.addWidget(self.lblRecipeMeta)

        self.txtRecipeDraft = QTextEdit(self.RecipeGRP)
        self.txtRecipeDraft.setReadOnly(True)
        self.txtRecipeDraft.setPlaceholderText("recipe draft view")
        vrec.addWidget(self.txtRecipeDraft, 1)

        root.addWidget(self.RecipeGRP, 2)

        # ------------------------------------------------------------
        # 2. Middle: Info + SprayPaths (Side-by-Side Splitter)
        # ------------------------------------------------------------
        mid_split = QSplitter(Qt.Orientation.Horizontal, self)
        mid_split.setChildrenCollapsible(False)

        # Left: Info
        self.infoBox = InfoGroupBox(parent=mid_split, title="Info")
        mid_split.addWidget(self.infoBox)

        # Right: Spray Paths
        self.sprayPathBox = SprayPathBox(ros=self.ros, parent=mid_split, title="Spray Paths")
        mid_split.addWidget(self.sprayPathBox)

        mid_split.setStretchFactor(0, 3)
        mid_split.setStretchFactor(1, 2)
        root.addWidget(mid_split, 0)

        # ------------------------------------------------------------
        # 3. Bottom: Stored Result vs. New Run Result
        # ------------------------------------------------------------
        res_split = QSplitter(Qt.Orientation.Horizontal, self)
        res_split.setChildrenCollapsible(False)

        # Left: Stored (Disk)
        self.StoredGRP = QGroupBox("Stored Result (Disk)", res_split)
        vsto = QVBoxLayout(self.StoredGRP)
        vsto.setContentsMargins(8, 8, 8, 8)
        vsto.setSpacing(6)
        
        self.txtStored = QTextEdit(self.StoredGRP)
        self.txtStored.setReadOnly(True)
        self.txtStored.setPlaceholderText("Stored results (Planned + Executed)...")
        vsto.addWidget(self.txtStored)
        
        res_split.addWidget(self.StoredGRP)

        # Right: New Run (Memory/Current)
        self.NewRunGRP = QGroupBox("New Run Result", res_split)
        vnew = QVBoxLayout(self.NewRunGRP)
        vnew.setContentsMargins(8, 8, 8, 8)
        vnew.setSpacing(6)

        self.txtNewRun = QTextEdit(self.NewRunGRP)
        self.txtNewRun.setReadOnly(True)
        self.txtNewRun.setPlaceholderText("Result of current run...")
        vnew.addWidget(self.txtNewRun)

        res_split.addWidget(self.NewRunGRP)

        # 50/50 Split
        res_split.setStretchFactor(0, 1)
        res_split.setStretchFactor(1, 1)
        root.addWidget(res_split, 2)

        # Policies
        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.setSizePolicy(sp)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_load_enabled(self, enabled: bool) -> None:
        self.btnLoad.setEnabled(enabled)

    def set_recipe(self, key: str, recipe_model: Recipe) -> None:
        if not key or recipe_model is None:
            self.clear_recipe()
            return
            
        self._recipe_key = key
        self._recipe = recipe_model

        # Reset UI
        self._update_recipe_ui()
        self._update_info_box()
        
        # Load stored data into LEFT box, clear RIGHT box
        self._load_stored_results_ui()
        self.txtNewRun.clear()
        
        # Spray paths defaults
        self.sprayPathBox.set_defaults(compiled=True, planned=True, executed=True)

        self.sig_recipe_selected.emit(key, recipe_model)

    def clear_recipe(self) -> None:
        self._recipe_key = None
        self._recipe = None
        self._update_recipe_ui()
        self._update_info_box()
        self._clear_result_ui()
        self.sig_recipe_cleared.emit()

    # ------------------------------------------------------------------
    # Run Lifecycle
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot(str, str)
    def on_run_started(self, mode: str, key: str) -> None:
        """Called when process starts. Lock UI, clear New Run box."""
        self.btnLoad.setEnabled(False)
        self.txtNewRun.setPlainText("Running...")

    @QtCore.pyqtSlot(str, object)
    def on_run_finished(self, key: str, rr_obj: object) -> None:
        """
        Wird aufgerufen, wenn der Roboterprozess fertig ist.
        Führt Post-Processing, Evaluation und Speichern durch.
        """
        self.btnLoad.setEnabled(True)

        if not isinstance(rr_obj, RunResult):
            _LOG.error("on_run_finished: Payload is not RunResult.")
            return

        rr: RunResult = rr_obj
        
        # 1. Post-Process (FK, TCP, Eval)
        try:
            self._postprocess_and_eval(rr)
        except Exception as e:
            _LOG.error(f"Postprocess failed: {e}")
            QMessageBox.warning(self, "Postprocess Error", str(e))
        
        # 2. Persist (Save to Disk)
        # Note: This overwrites the files on disk. 
        # The "Stored" box currently shows what was loaded BEFORE this run.
        # This allows comparison: Left=Old, Right=New.
        try:
            self._persist_runresult(key=key, rr=rr)
        except Exception as e:
            _LOG.error(f"Persist failed: {e}")
            QMessageBox.warning(self, "Persist Error", str(e))

        # 3. Update RIGHT box (New Run)
        self._display_run_result(rr, target_widget=self.txtNewRun)
        
        # 4. Update Info Box (e.g. duration)
        self._update_info_box()
        
        # 5. Trigger SprayPath update
        self.sprayPathBox.publish_current()

    @QtCore.pyqtSlot(str, str)
    def on_run_error(self, key: str, message: str) -> None:
        self.btnLoad.setEnabled(True)
        self.txtNewRun.setPlainText(f"ERROR: {message}")
        QMessageBox.critical(self, "Process Error", str(message))

    # ------------------------------------------------------------------
    # Internal Logic
    # ------------------------------------------------------------------

    def _on_load_clicked(self) -> None:
        keys = self._list_repo_keys()
        if not keys:
            QMessageBox.information(self, "Load", "Keine Rezepte gefunden.")
            return

        current = self._recipe_key if (self._recipe_key in keys) else keys[0]
        idx = keys.index(current) if current in keys else 0

        choice, ok = QInputDialog.getItem(self, "Recipe laden", "Auswahl:", keys, idx, False)
        if not ok or not choice.strip(): return
        
        key = choice.strip()
        try:
            model = self.repo.load_for_process(key)
            self.set_recipe(key, model)
        except Exception as e:
            QMessageBox.critical(self, "Load Error", f"Fehler beim Laden:\n{e}")

    def _list_repo_keys(self) -> List[str]:
        try:
            keys = self.repo.list_recipes() or []
            return sorted([str(k) for k in keys if str(k).strip()])
        except Exception:
            return []

    def _postprocess_and_eval(self, rr: RunResult) -> None:
        if self._recipe is None:
            raise RuntimeError("Kein Rezept geladen für Postprocessing.")

        scene_yaml, robot_yaml = self._ctx_scene_robot_yaml_paths()
        mounts_yaml = self._ctx_mounts_yaml_path()

        rr.postprocess(
            recipe=self._recipe,
            segment_order=("MOVE_PREDISPENSE", "MOVE_RECIPE", "MOVE_RETREAT", "MOVE_HOME"),
            ee_link="tcp",
            step_mm=1.0,
            max_points=0,
            gate_valid_on_eval=False,
            require_tcp=True,
            tcp_target_frame="substrate",
            scene_yaml_path=scene_yaml,
            robot_yaml_path=robot_yaml,
            mounts_yaml_path=mounts_yaml,
        )

    def _persist_runresult(self, *, key: str, rr: RunResult) -> None:
        p = self.repo.bundle.paths(key)
        
        # Save raw trajectory & TCP YAMLs
        planned_traj = rr.planned_run.get("traj")
        executed_traj = rr.executed_run.get("traj")
        planned_tcp = rr.planned_run.get("tcp")
        executed_tcp = rr.executed_run.get("tcp")

        if p.planned_traj_yaml and planned_traj:
            self._yaml_write_file(p.planned_traj_yaml, planned_traj)
        if p.executed_traj_yaml and executed_traj:
            self._yaml_write_file(p.executed_traj_yaml, executed_traj)
        if p.planned_tcp_yaml and planned_tcp:
            self._yaml_write_file(p.planned_tcp_yaml, planned_tcp)
        if p.executed_tcp_yaml and executed_tcp:
            self._yaml_write_file(p.executed_tcp_yaml, executed_tcp)

    def _load_stored_results_ui(self) -> None:
        """Loads data from disk into the 'Stored' box."""
        self.txtStored.clear()
        if not self._recipe_key: return
        
        try:
            p = self.repo.bundle.paths(self._recipe_key)
            # Load stored TCP results (containing eval)
            pt_doc = self._yaml_load_file(p.planned_tcp_yaml) if p.planned_tcp_yaml else {}
            et_doc = self._yaml_load_file(p.executed_tcp_yaml) if p.executed_tcp_yaml else {}
            
            # Create a temporary/dummy RR to reuse formatting logic
            rr_stored = RunResult(
                planned_run={"tcp": pt_doc},
                executed_run={"tcp": et_doc},
                eval={
                    "planned": pt_doc.get("eval", {}),
                    "executed": et_doc.get("eval", {})
                },
                valid=True, invalid_reason=""
            )
            self._display_run_result(rr_stored, target_widget=self.txtStored)
        except Exception as e:
            self.txtStored.setPlainText(f"Failed to load stored results: {e}")

    # ------------------------------------------------------------------
    # UI Helper & Formatting
    # ------------------------------------------------------------------

    def _display_run_result(self, rr: RunResult, target_widget: QTextEdit) -> None:
        """Formats combined Planned + Executed stats into one text box."""
        ev = rr.eval or {}
        # Try finding eval in explicit .eval, else fallback to .run.tcp.eval
        p_eval = ev.get("planned") or rr.planned_run.get("tcp", {}).get("eval", {})
        e_eval = ev.get("executed") or rr.executed_run.get("tcp", {}).get("eval", {})

        lines = []
        
        # --- Planned Section ---
        lines.append("=== PLANNED ===")
        if p_eval:
            lines.append(f"Summary: {self._summary_line(p_eval)}")
            lines.append(self._yaml_dump(p_eval))
        else:
            lines.append("No data.")
        
        lines.append("") # Spacer

        # --- Executed Section ---
        lines.append("=== EXECUTED ===")
        if e_eval:
            lines.append(f"Summary: {self._summary_line(e_eval)}")
            lines.append(self._yaml_dump(e_eval))
        else:
            lines.append("No data.")

        target_widget.setPlainText("\n".join(lines))

    def _update_recipe_ui(self) -> None:
        r = self._recipe
        if r is None:
            self.lblRecipeKey.setText("Key: –")
            self.lblRecipeMeta.setText("Info: –")
            self.txtRecipeDraft.clear()
            return

        self.lblRecipeKey.setText(f"Key: {self._recipe_key}")
        info = f"id={r.id} | tool={r.tool} | sub={r.substrate}"
        self.lblRecipeMeta.setText(f"Info: {info}")
        
        dump_data = {
            "id": r.id, "tool": r.tool, "substrate": r.substrate,
            "parameters": r.parameters, "planner": r.planner, "meta": r.meta,
        }
        self.txtRecipeDraft.setPlainText(self._yaml_dump(dump_data))

    def _update_info_box(self) -> None:
        points = None
        if self._recipe:
            v = getattr(self._recipe, "compiled_points_mm", None)
            if v is not None:
                try: points = np.asarray(v).reshape(-1, 3)
                except: pass
        self.infoBox.update_from_recipe(self._recipe, points)

    def _clear_result_ui(self) -> None:
        self.txtStored.clear()
        self.txtNewRun.clear()

    @staticmethod
    def _summary_line(ev: Dict) -> str:
        if not ev: return "–"
        try:
            score = ev.get("total", {}).get("score")
            valid = ev.get("valid")
            ret = []
            if score is not None: ret.append(f"score={score:.3f}")
            if valid is not None: ret.append(f"valid={valid}")
            return " | ".join(ret) if ret else "–"
        except: return "–"

    # ------------------------------------------------------------------
    # File / Path Utils
    # ------------------------------------------------------------------

    def _yaml_dump(self, obj: Any) -> str:
        try: return yaml.safe_dump(obj or {}, allow_unicode=True, sort_keys=False)
        except: return str(obj)

    def _yaml_write_file(self, path: str, obj: Any) -> None:
        if not path: return
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(obj or {}, f, allow_unicode=True, sort_keys=False)

    def _yaml_load_file(self, path: str) -> Dict:
        if not path or not os.path.isfile(path): return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        except: return {}

    def _ctx_scene_robot_yaml_paths(self) -> Tuple[Optional[str], Optional[str]]:
        try:
            base = os.environ.get("SC_PROJECT_ROOT", "")
            cfgs = self.ctx.ros.configs
            return resolve_path(base, cfgs.scene_file), resolve_path(base, cfgs.robot_file)
        except: return None, None

    def _ctx_mounts_yaml_path(self) -> Optional[str]:
        try:
            return resolve_path(os.environ.get("SC_PROJECT_ROOT", ""), self.ctx.mounts_yaml_path)
        except: return None