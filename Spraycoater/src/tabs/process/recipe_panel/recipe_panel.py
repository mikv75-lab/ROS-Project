# -*- coding: utf-8 -*-
# File: src/tabs/process/recipe_panel/recipe_panel.py
from __future__ import annotations

import os
from typing import Optional, Any, Dict, List, Tuple

import yaml

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


class RecipePanel(QWidget):
    """
    RecipePanel (STRICT):

    - Loads/selects Recipe model (repo.load_for_process(key))
    - Emits recipe selection to ProcessPanel
    - Receives RunResult rr from ProcessPanel and THEN does:
        * rr.postprocess(...)  (FK/TCP/etc)
        * evaluation (as part of postprocess, per RunResult implementation)
        * persistence of YAML artifacts via repo.bundle.paths(key)
        * UI updates (eval text + summary + info)

    Signals:
      - sig_recipe_selected(str key, object recipe_model)
      - sig_recipe_cleared()

    Slots:
      - on_run_started(str mode, str key)
      - on_run_finished(str key, object rr)
      - on_run_error(str key, str message)
    """

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

        if not callable(getattr(repo, "list_recipes", None)):
            raise RuntimeError("RecipePanel: repo.list_recipes() missing (strict)")
        if not callable(getattr(repo, "load_for_process", None)):
            raise RuntimeError("RecipePanel: repo.load_for_process(key) missing (strict)")
        bundle = getattr(repo, "bundle", None)
        if bundle is None or not callable(getattr(bundle, "paths", None)):
            raise RuntimeError("RecipePanel: repo.bundle.paths(key) missing (strict)")

        self.ctx = ctx
        self.repo = repo
        self.ros = ros

        self._recipe_key: Optional[str] = None
        self._recipe: Optional[Recipe] = None

        self._build_ui()
        self._wire_ui()

        self._update_recipe_ui()
        self._clear_eval_ui()

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        # RecipeGRP
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

        # Middle: Info + SprayPaths
        mid_split = QSplitter(Qt.Orientation.Horizontal, self)
        mid_split.setChildrenCollapsible(False)

        self.infoBox = InfoGroupBox(parent=mid_split, title="Info")
        if not callable(getattr(self.infoBox, "update_from_recipe", None)):
            raise RuntimeError("RecipePanel: InfoGroupBox.update_from_recipe(...) missing (strict)")
        mid_split.addWidget(self.infoBox)

        self.SprayPathsGRP = QGroupBox("Spray Paths", mid_split)
        vsp = QVBoxLayout(self.SprayPathsGRP)
        vsp.setContentsMargins(8, 8, 8, 8)
        vsp.setSpacing(6)

        self.sprayPathBox = SprayPathBox(ros=self.ros, parent=self.SprayPathsGRP, title=None)
        if not callable(getattr(self.sprayPathBox, "set_defaults", None)):
            raise RuntimeError("RecipePanel: SprayPathBox.set_defaults(...) missing (strict)")

        vsp.addWidget(self.sprayPathBox, 1)
        mid_split.addWidget(self.SprayPathsGRP)

        mid_split.setStretchFactor(0, 3)
        mid_split.setStretchFactor(1, 2)
        root.addWidget(mid_split, 0)

        # Bottom: Planned + Executed Eval
        eval_split = QSplitter(Qt.Orientation.Horizontal, self)
        eval_split.setChildrenCollapsible(False)

        self.PlannedGRP = QGroupBox("Planned (Eval)", eval_split)
        vpl = QVBoxLayout(self.PlannedGRP)
        vpl.setContentsMargins(8, 8, 8, 8)
        vpl.setSpacing(6)

        self.txtPlannedEval = QTextEdit(self.PlannedGRP)
        self.txtPlannedEval.setReadOnly(True)
        self.txtPlannedEval.setPlaceholderText("planned tcp eval (yaml)")
        vpl.addWidget(self.txtPlannedEval, 1)

        self.lblPlannedSummary = QLabel("planned: –", self.PlannedGRP)
        self.lblPlannedSummary.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        vpl.addWidget(self.lblPlannedSummary, 0)
        eval_split.addWidget(self.PlannedGRP)

        self.ExecutedGRP = QGroupBox("Executed (Eval)", eval_split)
        vex = QVBoxLayout(self.ExecutedGRP)
        vex.setContentsMargins(8, 8, 8, 8)
        vex.setSpacing(6)

        self.txtExecutedEval = QTextEdit(self.ExecutedGRP)
        self.txtExecutedEval.setReadOnly(True)
        self.txtExecutedEval.setPlaceholderText("executed tcp eval (yaml)")
        vex.addWidget(self.txtExecutedEval, 1)

        self.lblExecutedSummary = QLabel("executed: –", self.ExecutedGRP)
        self.lblExecutedSummary.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        vex.addWidget(self.lblExecutedSummary, 0)
        eval_split.addWidget(self.ExecutedGRP)

        eval_split.setStretchFactor(0, 1)
        eval_split.setStretchFactor(1, 1)
        root.addWidget(eval_split, 2)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.setSizePolicy(sp)

    def _wire_ui(self) -> None:
        self.btnLoad.clicked.connect(self._on_load_clicked)

        # SprayPathBox publishes to ROS itself; RecipePanel stays UI-only here.
        self.sprayPathBox.showCompiledToggled.connect(lambda _: None)
        self.sprayPathBox.showPlannedToggled.connect(lambda _: None)
        self.sprayPathBox.showExecutedToggled.connect(lambda _: None)

    # ------------------------------------------------------------------
    # Repo / selection
    # ------------------------------------------------------------------

    def _list_repo_keys(self) -> List[str]:
        keys = self.repo.list_recipes()
        if not isinstance(keys, list):
            raise RuntimeError("RecipePanel: repo.list_recipes() must return list[str] (strict)")
        out = [str(k).strip() for k in keys if isinstance(k, str) and str(k).strip()]
        out.sort()
        return out

    def _on_load_clicked(self) -> None:
        keys = self._list_repo_keys()
        if not keys:
            QMessageBox.information(self, "Load", "Keine gespeicherten Rezepte gefunden.")
            return

        current = self._recipe_key if (self._recipe_key in keys) else keys[0]
        idx = keys.index(current) if current in keys else 0

        choice, ok = QInputDialog.getItem(self, "Recipe laden", "Rezept auswählen:", keys, idx, editable=False)
        if not ok:
            return

        key = str(choice).strip()
        if not key:
            return

        try:
            recipe_model = self.repo.load_for_process(key)
        except Exception as e:
            QMessageBox.critical(self, "Load", f"Load fehlgeschlagen: {e}")
            return

        self.set_recipe(key, recipe_model)

    def set_recipe(self, key: str, recipe_model: Recipe) -> None:
        key = str(key or "").strip()
        if not key:
            raise ValueError("RecipePanel.set_recipe: empty key (strict)")
        if recipe_model is None:
            raise ValueError("RecipePanel.set_recipe: recipe_model is None (strict)")

        self._recipe_key = key
        self._recipe = recipe_model

        # Default toggles; SprayPathBox publishes once.
        self.sprayPathBox.set_defaults(compiled=True, planned=True, executed=True)

        self._update_recipe_ui()
        self._update_info_box()
        self._clear_eval_ui()

        self.sig_recipe_selected.emit(key, recipe_model)

    def clear_recipe(self) -> None:
        self._recipe_key = None
        self._recipe = None
        self._update_recipe_ui()
        self._update_info_box()
        self._clear_eval_ui()
        self.sig_recipe_cleared.emit()

    # ------------------------------------------------------------------
    # Run lifecycle slots
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot(str, str)
    def on_run_started(self, mode: str, key: str) -> None:
        self.btnLoad.setEnabled(False)
        self._clear_eval_ui()

    @QtCore.pyqtSlot(str, object)
    def on_run_finished(self, key: str, rr_obj: object) -> None:
        self.btnLoad.setEnabled(True)

        if not isinstance(rr_obj, RunResult):
            raise RuntimeError("RecipePanel.on_run_finished: rr_obj is not RunResult (strict)")
        rr: RunResult = rr_obj

        # STRICT: all postprocess/eval/persist happens HERE (not in ProcessPanel)
        try:
            self._postprocess_and_eval(rr)
        except Exception as e:
            QMessageBox.critical(self, "Postprocess/Eval", f"{e}")
            # Even if postprocess fails, still show raw rr.eval if present
        try:
            self._persist_runresult(key=str(key), rr=rr)
        except Exception as e:
            QMessageBox.critical(self, "Persist", f"{e}")

        self._set_eval_from_runresult(rr)
        self._update_info_box()

    @QtCore.pyqtSlot(str, str)
    def on_run_error(self, key: str, message: str) -> None:
        self.btnLoad.setEnabled(True)
        QMessageBox.critical(self, "Process", str(message or "Unbekannter Fehler"))

    # ------------------------------------------------------------------
    # Postprocess (FK/TCP) + Eval (strictly here)
    # ------------------------------------------------------------------

    def _ctx_robot_xml(self) -> Tuple[str, str]:
        rd = getattr(self.ctx, "robot_description", None)
        urdf = str(getattr(rd, "urdf_xml", "") or "")
        srdf = str(getattr(rd, "srdf_xml", "") or "")
        return urdf, srdf

    def _ctx_scene_robot_yaml_paths(self) -> Tuple[Optional[str], Optional[str]]:
        base_dir = os.environ.get("SC_PROJECT_ROOT", "").strip()
        if not base_dir:
            return None, None

        ros_cfg = getattr(self.ctx, "ros", None)
        cfgs = getattr(ros_cfg, "configs", None) if ros_cfg is not None else None
        if cfgs is None:
            return None, None

        scene_uri = str(getattr(cfgs, "scene_file", "") or "").strip()
        robot_uri = str(getattr(cfgs, "robot_file", "") or "").strip()
        if not scene_uri or not robot_uri:
            return None, None

        scene_abs = resolve_path(base_dir, scene_uri)
        robot_abs = resolve_path(base_dir, robot_uri)

        if not os.path.isfile(scene_abs) or not os.path.isfile(robot_abs):
            return None, None
        return scene_abs, robot_abs

    def _ctx_mounts_yaml_path(self) -> Optional[str]:
        base_dir = os.environ.get("SC_PROJECT_ROOT", "").strip()
        if not base_dir:
            return None

        uri = str(getattr(self.ctx, "mounts_yaml_path", "") or "").strip()
        if not uri:
            return None

        p = resolve_path(base_dir, uri)
        return p if os.path.isfile(p) else None

    def _postprocess_and_eval(self, rr: RunResult) -> None:
        if self._recipe is None:
            raise RuntimeError("RecipePanel: cannot postprocess without loaded recipe model (strict)")

        scene_yaml, robot_yaml = self._ctx_scene_robot_yaml_paths()
        mounts_yaml = self._ctx_mounts_yaml_path()

        # IMPORTANT:
        # We rely on RunResult.postprocess to do FK/TCP and evaluation (as per your project design).
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

    # ------------------------------------------------------------------
    # Persistence (strictly here)
    # ------------------------------------------------------------------

    @staticmethod
    def _yaml_write_file(path: str, obj: Any) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(obj or {}, f, allow_unicode=True, sort_keys=False)

    def _persist_runresult(self, *, key: str, rr: RunResult) -> None:
        p = self.repo.bundle.paths(key)

        planned_traj_path = str(getattr(p, "planned_traj_yaml", "") or "")
        executed_traj_path = str(getattr(p, "executed_traj_yaml", "") or "")
        planned_tcp_path = str(getattr(p, "planned_tcp_yaml", "") or "")
        executed_tcp_path = str(getattr(p, "executed_tcp_yaml", "") or "")

        planned_traj_doc = rr.planned_run.get("traj") if isinstance(rr.planned_run, dict) else {}
        executed_traj_doc = rr.executed_run.get("traj") if isinstance(rr.executed_run, dict) else {}
        planned_tcp_doc = rr.planned_run.get("tcp") if isinstance(rr.planned_run, dict) else {}
        executed_tcp_doc = rr.executed_run.get("tcp") if isinstance(rr.executed_run, dict) else {}

        if planned_traj_path:
            self._yaml_write_file(planned_traj_path, planned_traj_doc)
        if executed_traj_path:
            self._yaml_write_file(executed_traj_path, executed_traj_doc)

        if planned_tcp_path and isinstance(planned_tcp_doc, dict) and planned_tcp_doc:
            self._yaml_write_file(planned_tcp_path, planned_tcp_doc)
        if executed_tcp_path and isinstance(executed_tcp_doc, dict) and executed_tcp_doc:
            self._yaml_write_file(executed_tcp_path, executed_tcp_doc)

    # ------------------------------------------------------------------
    # UI helpers
    # ------------------------------------------------------------------

    def _update_info_box(self) -> None:
        if self._recipe is None:
            self.infoBox.update_from_recipe(None, None)
            return
        self.infoBox.update_from_recipe(self._recipe, None)

    def _update_recipe_ui(self) -> None:
        if self._recipe is None or not self._recipe_key:
            self.lblRecipeKey.setText("Key: –")
            self.lblRecipeMeta.setText("Info: –")
            self.txtRecipeDraft.clear()
            return

        r = self._recipe
        self.lblRecipeKey.setText(f"Key: {self._recipe_key}")

        parts: List[str] = []
        rid = getattr(r, "id", None)
        if rid:
            parts.append(f"id={rid}")
        tool = getattr(r, "tool", None)
        if tool:
            parts.append(f"tool={tool}")
        substrate = getattr(r, "substrate", None)
        if substrate:
            parts.append(f"substrate={substrate}")
        mount = getattr(r, "substrate_mount", None)
        if mount:
            parts.append(f"mount={mount}")
        desc = getattr(r, "description", None)
        if desc:
            parts.append(f"desc={desc}")

        self.lblRecipeMeta.setText("Info: " + (" | ".join(parts) if parts else "–"))

        draft_view: Dict[str, Any] = {
            "id": getattr(r, "id", None),
            "description": getattr(r, "description", None),
            "tool": getattr(r, "tool", None),
            "substrate": getattr(r, "substrate", None),
            "substrate_mount": getattr(r, "substrate_mount", None),
            "parameters": getattr(r, "parameters", None) or {},
            "planner": getattr(r, "planner", None) or {},
            "paths_by_side": getattr(r, "paths_by_side", None) or {},
            "meta": getattr(r, "meta", None) or {},
            "draft_present": bool(getattr(r, "draft", None) is not None),
            "planned_traj_present": bool(getattr(r, "planned_traj", None) is not None),
            "executed_traj_present": bool(getattr(r, "executed_traj", None) is not None),
        }
        self.txtRecipeDraft.setPlainText(yaml.safe_dump(draft_view, allow_unicode=True, sort_keys=False))

    # ------------------------------------------------------------------
    # Eval UI
    # ------------------------------------------------------------------

    def _clear_eval_ui(self) -> None:
        self.txtPlannedEval.setPlainText("–")
        self.txtExecutedEval.setPlainText("–")
        self.lblPlannedSummary.setText("planned: –")
        self.lblExecutedSummary.setText("executed: –")

    @staticmethod
    def _extract_eval_pair(rr: RunResult) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        ev = rr.eval if isinstance(rr.eval, dict) else {}
        planned = ev.get("planned") if isinstance(ev.get("planned"), dict) else {}
        executed = ev.get("executed") if isinstance(ev.get("executed"), dict) else {}
        return planned, executed

    @staticmethod
    def _summary_line(ev: Dict[str, Any]) -> str:
        if not isinstance(ev, dict) or not ev:
            return "–"
        valid = ev.get("valid")
        thr = ev.get("threshold")
        total = ev.get("total") if isinstance(ev.get("total"), dict) else {}
        score = total.get("score") if isinstance(total, dict) else None

        parts: List[str] = []
        if score is not None:
            parts.append(f"score={float(score):.3f}")
        if thr is not None:
            parts.append(f"thr={float(thr):.3f}")
        if valid is not None:
            parts.append(f"valid={bool(valid)}")
        return " | ".join(parts) if parts else "–"

    def _set_eval_from_runresult(self, rr: RunResult) -> None:
        planned, executed = self._extract_eval_pair(rr)

        self.txtPlannedEval.setPlainText(
            yaml.safe_dump(planned or {}, allow_unicode=True, sort_keys=False) if planned else "–"
        )
        self.txtExecutedEval.setPlainText(
            yaml.safe_dump(executed or {}, allow_unicode=True, sort_keys=False) if executed else "–"
        )
        self.lblPlannedSummary.setText("planned: " + self._summary_line(planned))
        self.lblExecutedSummary.setText("executed: " + self._summary_line(executed))
