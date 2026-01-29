# -*- coding: utf-8 -*-
# File: src/tabs/process/recipe_panel/recipe_panel.py

from __future__ import annotations

import logging
import os
from typing import Optional, Any, Dict, List, Tuple

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
    QInputDialog,
    QSizePolicy,
    QMessageBox,
)

from model.recipe.recipe import Recipe
from model.recipe.recipe_run_result import RunResult

# NEW location (no legacy)
from model.spray_paths import recipe_markers
from model.spray_paths.draft import Draft  # strict Draft API for points extraction

from config.startup import require_env_dir, resolve_path

from .spray_path_box import SprayPathBox
from widgets.info_groupbox import InfoGroupBox

_LOG = logging.getLogger("tabs.process.recipe_panel")


class RecipePanel(QWidget):
    """
    Anzeige und Management von Rezepten und deren Ausführungsergebnissen (Strict V2).

    STRICT principles:
      - No legacy attribute scanning / alias fallbacks.
      - ctx is expected to be AppContext from config.startup.load_startup().
      - Offline TF paths are derived deterministically from ctx.content and ctx.ros.
      - ROS bridge methods required by this panel are enforced.

    UI:
      - Top row: Active Recipe | Info (CENTER, BIGGEST) | Spray Paths
      - Bottom row: Recipe Params | Stored Eval | Current Run

    Persist policy (as you requested):
      - executed_*: overwrite always when present in RunResult (execute only)
      - planned_*: overwrite only if validate/optimize produced it;
                  if it would overwrite an existing planned_* on disk -> ask user.
      - missing parts never delete previous artifacts (handled by repo merge policy)
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

        self.ctx, self.repo, self.ros = ctx, repo, ros
        self._recipe: Optional[Recipe] = None
        self._recipe_key: Optional[str] = None

        self._build_ui()
        self.btnLoad.clicked.connect(self._on_load_clicked)

    # ---------------- UI ----------------

    def _build_ui(self) -> None:
        """
        Layout:
        vbox(
          hbox( ActiveRecipe, Info (biggest), SprayPaths ),
          hbox( RecipeParams, StoredEval, CurrentEval ) -> Alle 3 exakt gleich breit
        )
        """
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ============================================================
        # Top row: Active Recipe + Info + Spray Paths
        # ============================================================
        top = QWidget(self)
        htop = QHBoxLayout(top)
        htop.setContentsMargins(0, 0, 0, 0)
        htop.setSpacing(8)

        # --- Active Recipe (left, bounded width) ---
        self.grpActive = QGroupBox("Active Recipe", top)
        vactive = QVBoxLayout(self.grpActive)
        vactive.setContentsMargins(8, 8, 8, 8)
        vactive.setSpacing(6)

        row = QWidget(self.grpActive)
        hrow = QHBoxLayout(row)
        hrow.setContentsMargins(0, 0, 0, 0)
        hrow.setSpacing(10)

        self.btnLoad = QPushButton("Load Recipe", row)
        self.lblRecipeName = QLabel("Recipe: –", row)
        self.lblRecipeName.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

        hrow.addWidget(self.btnLoad, 0)
        hrow.addWidget(self.lblRecipeName, 1)
        vactive.addWidget(row, 0)

        self.txtActiveInfo = QTextEdit(self.grpActive)
        self.txtActiveInfo.setReadOnly(True)
        self.txtActiveInfo.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        vactive.addWidget(self.txtActiveInfo, 1)

        # Obere Reihe: Feste Breiten für die äußeren Boxen
        self.grpActive.setMinimumWidth(320)
        self.grpActive.setMaximumWidth(520)

        # --- Info (center, BIGGEST) ---
        self.infoBox = InfoGroupBox(top, title="Info")
        self.infoBox.setMinimumWidth(520)
        self.infoBox.setMaximumWidth(99999)

        # --- Spray Paths (right, bounded width) ---
        self.sprayPathBox = SprayPathBox(ros=self.ros, parent=top)
        self.sprayPathBox.setMinimumWidth(260)
        self.sprayPathBox.setMaximumWidth(420)

        # Hinzufügen der oberen Widgets: Info (Mitte) bekommt Stretch 2
        htop.addWidget(self.grpActive, 0)
        htop.addWidget(self.infoBox, 2) 
        htop.addWidget(self.sprayPathBox, 0)

        root.addWidget(top, 0)

        # ============================================================
        # Bottom row: Recipe Params + Stored Eval + Current Eval
        # ============================================================
        bottom = QWidget(self)
        hbot = QHBoxLayout(bottom)
        hbot.setContentsMargins(0, 0, 0, 0)
        hbot.setSpacing(8)

        # 1. Recipe Parameters
        self.grpRecipeParams = QGroupBox("Recipe Parameters", bottom)
        vparams = QVBoxLayout(self.grpRecipeParams)
        vparams.setContentsMargins(8, 8, 8, 8)
        self.txtRecipeParams = QTextEdit(self.grpRecipeParams)
        self.txtRecipeParams.setReadOnly(True)
        self.txtRecipeParams.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        vparams.addWidget(self.txtRecipeParams, 1)

        # 2. Stored Eval (Disk)
        self.grpStored = QGroupBox("Stored Eval (Disk)", bottom)
        vstored = QVBoxLayout(self.grpStored)
        vstored.setContentsMargins(8, 8, 8, 8)
        self.txtStored = QTextEdit(self.grpStored)
        self.txtStored.setReadOnly(True)
        self.txtStored.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        vstored.addWidget(self.txtStored, 1)

        # 3. Current Run (Live)
        self.grpCurrent = QGroupBox("Current Run (Live)", bottom)
        vcur = QVBoxLayout(self.grpCurrent)
        vcur.setContentsMargins(8, 8, 8, 8)
        self.txtNewRun = QTextEdit(self.grpCurrent)
        self.txtNewRun.setReadOnly(True)
        self.txtNewRun.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        vcur.addWidget(self.txtNewRun, 1)

        # WICHTIG: Hier alle drei mit Stretch-Faktor 1 hinzufügen für gleiche Breite
        hbot.addWidget(self.grpRecipeParams, 1)
        hbot.addWidget(self.grpStored, 1)
        hbot.addWidget(self.grpCurrent, 1)

        root.addWidget(bottom, 1)

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

        self.txtActiveInfo.setPlainText(self._format_active_recipe_info(model))
        self.txtRecipeParams.setPlainText(self._format_recipe_params(model))

        # Disk SSoT refresh on load
        self._refresh_stored_from_disk(key)

        # NOTE: _refresh_stored_from_disk() updates self._recipe -> use that for infobox
        if self._recipe is not None:
            self._update_infobox_from_recipe(self._recipe)

        self.txtNewRun.clear()

        # Publish current recipe draft + stored ghosts
        if self._recipe is not None:
            self._republish_spraypaths_for_key(key, self._recipe)

        self.sig_recipe_selected.emit(key, self._recipe if self._recipe is not None else model)

    def _format_active_recipe_info(self, model: Recipe) -> str:
        src: Dict[str, Any] = {}
        try:
            d = model.to_params_dict()
            if isinstance(d, dict):
                src = d
        except Exception:
            src = {}

        def pick_attr(name: str) -> Any:
            v = getattr(model, name, None)
            if v is not None and v != "":
                return v
            return src.get(name)

        description = pick_attr("description")
        rid = pick_attr("id") or pick_attr("key")
        info = pick_attr("info")
        meta = pick_attr("meta")
        substrate = pick_attr("substrate")
        substrate_mount = pick_attr("substrate_mount")
        tool = pick_attr("tool")

        info_d: Dict[str, Any] = info if isinstance(info, dict) else {}
        meta_d: Dict[str, Any] = meta if isinstance(meta, dict) else {}

        lines: List[str] = []
        lines.append(f"description: {description if description not in (None, '') else '–'}")
        lines.append(f"id: {rid if rid not in (None, '') else '–'}")
        lines.append("info:")
        lines.append(f"  validSave: {info_d.get('validSave', '–')}")
        lines.append(f"  validSaveReason: {info_d.get('validSaveReason', '–')}")
        lines.append("meta:")
        lines.append(f"  template_id: {meta_d.get('template_id', '–')}")
        lines.append(f"substrate: {substrate if substrate not in (None, '') else '–'}")
        lines.append(f"substrate_mount: {substrate_mount if substrate_mount not in (None, '') else '–'}")
        lines.append(f"tool: {tool if tool not in (None, '') else '–'}")
        return "\n".join(lines).strip()

    def _format_recipe_params(self, model: Recipe) -> str:
        try:
            d = model.to_params_dict()
        except Exception:
            d = {"repr": repr(model)}

        if not isinstance(d, dict):
            d = {"repr": repr(model)}
        else:
            d = dict(d)
            for k in ("description", "id", "key", "info", "meta", "substrate", "substrate_mount", "tool"):
                d.pop(k, None)

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
        Strict V2 (final persist policy):

        1) Load recipe (disk) and attach to RunResult BEFORE eval (required for speed + selection).
        2) Postprocess + Eval first
        3) SHOW the new eval/report immediately
        4) THEN ask whether planned_* should overwrite (only if needed)
        5) Persist (ONLY if rr.valid True):
           - executed_* saved when present (execute)
           - planned_* saved when present (validate/optimize) but only after confirmation if overwrite
           - missing parts never delete previous artifacts (repo merge policy)
        """
        key = str(key or "").strip()
        self.txtNewRun.setPlaceholderText("")

        if not key:
            self.txtNewRun.setPlainText("Run finished, but key is empty.")
            return

        # ----------------------------
        # Strict RunResult resolve
        # ----------------------------
        rr: Optional[RunResult] = None
        if isinstance(rr_obj, RunResult):
            rr = rr_obj
        elif isinstance(payload, dict):
            try:
                rr = RunResult.from_process_payload(payload)
            except Exception as e:
                rr = None
                _LOG.error("RecipePanel.on_run_finished: from_process_payload failed (key=%s): %s", key, e)

        if rr is None:
            self.txtNewRun.setPlainText(
                f"Run finished for {key}, but no RunResult was provided and payload is invalid.\n"
                f"payload type: {type(payload).__name__}\n"
                f"rr_obj type: {type(rr_obj).__name__}"
            )
            _LOG.error(
                "RecipePanel.on_run_finished: invalid rr (key=%s) payload=%s rr_obj=%s",
                key,
                type(payload).__name__,
                type(rr_obj).__name__,
            )
            return

        # ----------------------------
        # Load recipe from disk (SSoT) and ATTACH CONTEXT BEFORE EVAL
        # ----------------------------
        recipe_disk: Optional[Recipe]
        try:
            recipe_disk = self.repo.load_for_process(key)
        except Exception as e:
            recipe_disk = None
            _LOG.error("RecipePanel: load_for_process failed (key=%s): %s", key, e)

        try:
            if recipe_disk is not None:
                # STRICT: attach_recipe_context exists and is the ONLY API we use here.
                rr.attach_recipe_context(recipe_disk)
        except Exception as e:
            _LOG.error("RecipePanel: rr.attach_recipe_context failed (key=%s): %s", key, e)

        seg_order = list(self._default_segment_order())

        # ------------------------------------------------------------
        # Postprocess + Eval FIRST (now with attached recipe context)
        # ------------------------------------------------------------
        post_err: Optional[str] = None
        try:
            scene_yaml_path, robot_yaml_path, mounts_yaml_path = self._required_offline_tf_paths()
            rr.postprocess(
                recipe=recipe_disk,
                segment_order=seg_order,
                segment_to_side={
                    "MOVE_PREDISPENSE": "top",
                    "MOVE_RECIPE": "top",
                    "MOVE_RETREAT": "top",
                    "MOVE_HOME": "top",
                },
                step_mm=2.0,
                max_points=20000,
                require_tcp=False,
                scene_yaml_path=scene_yaml_path,
                robot_yaml_path=robot_yaml_path,
                mounts_yaml_path=mounts_yaml_path,
                evaluate=False,
            )
        except Exception as e:
            post_err = str(e)
            _LOG.error("RecipePanel: rr.postprocess failed (key=%s): %s", key, e)

        eval_err: Optional[str] = None
        try:
            rr.evaluate_tcp_against_draft(
                recipe=recipe_disk,  # explicit (even though attached)
                segment_order=seg_order,
                domain="tcp",
                gate_valid_on_eval=False,
            )
        except Exception as e:
            eval_err = str(e)
            _LOG.error("RecipePanel: rr.evaluate_tcp_against_draft failed (key=%s): %s", key, e)

        # ------------------------------------------------------------
        # SHOW report immediately (before any save prompt)
        # ------------------------------------------------------------
        try:
            txt = self._current_run_report_text(rr)
            extra: List[str] = []
            if post_err:
                extra.append(f"postprocess_error: {post_err}")
            if eval_err:
                extra.append(f"eval_error: {eval_err}")
            if extra:
                txt = f"{txt}\n\n" + "\n".join(extra)
            self.txtNewRun.setPlainText(txt)
        except Exception as e:
            self.txtNewRun.setPlainText(f"Run finished for {key}, but report render failed: {e}")

        # Overlays: show what was just evaluated
        try:
            self._republish_newrun_overlays_from_rr(rr)
        except Exception as e:
            _LOG.error("RecipePanel: republish newrun overlays failed: %s", e)

        # ------------------------------------------------------------
        # Persist ONLY if run is valid ("durchgelaufen")
        # ------------------------------------------------------------
        persist_err: Optional[str] = None
        try:
            if bool(getattr(rr, "valid", False)):
                if recipe_disk is not None:
                    # ask ONLY for planned overwrite (executed has no prompt)
                    if not self._confirm_planned_overwrite_if_needed(key, recipe_disk, rr):
                        # user chose keep existing planned_* -> ensure repo KEEP policy
                        self._drop_planned_from_run_result(rr)

                # single call does everything (repo merge keep-on-missing)
                self.repo.save_run_result_if_valid(key, run_result=rr)
        except Exception as e:
            persist_err = str(e)
            _LOG.error("RecipePanel: persist failed (key=%s): %s", key, e)

        if persist_err:
            try:
                cur = self.txtNewRun.toPlainText().rstrip()
                self.txtNewRun.setPlainText(f"{cur}\n\npersist_error: {persist_err}")
            except Exception:
                pass

        # ------------------------------------------------------------
        # Refresh disk view last
        # ------------------------------------------------------------
        try:
            self._refresh_stored_from_disk(key)
        except Exception as e:
            _LOG.error("RecipePanel: refresh stored after run failed: %s", e)


    @QtCore.pyqtSlot(str, str)
    def on_run_error(self, key: str, message: str) -> None:
        self.txtNewRun.setPlaceholderText("")
        self.txtNewRun.setPlainText(f"ERROR for {key}:\n{message}")
        _LOG.error("RecipePanel: Run error for %s: %s", key, message)

    # ============================================================
    # Persist overwrite confirm (planned only)
    # ============================================================

    @staticmethod
    def _rr_has_new_planned(rr: RunResult) -> bool:
        pr = getattr(rr, "planned_run", None)
        if not isinstance(pr, dict):
            return False
        t = pr.get("traj")
        c = pr.get("tcp")
        has_traj = isinstance(t, dict) and bool(t)
        has_tcp = isinstance(c, dict) and bool(c)
        return bool(has_traj or has_tcp)

    @staticmethod
    def _recipe_has_existing_planned(recipe: Recipe) -> bool:
        # disk attachments are objects (JTBySegment/Draft) or None
        return bool(getattr(recipe, "planned_traj", None) is not None or getattr(recipe, "planned_tcp", None) is not None)

    def _confirm_planned_overwrite_if_needed(self, key: str, recipe_disk: Recipe, rr: RunResult) -> bool:
        """
        Returns True if we should allow overwriting planned_* on disk.
        Returns False if user chooses to keep existing planned_*.
        """
        if not self._rr_has_new_planned(rr):
            return True  # nothing new -> no overwrite
        if not self._recipe_has_existing_planned(recipe_disk):
            return True  # nothing to overwrite

        # Ask user once per run.
        mb = QMessageBox(self)
        mb.setIcon(QMessageBox.Icon.Question)
        mb.setWindowTitle("Overwrite planned artifacts?")
        mb.setText(f"Recipe '{key}' already has planned_* on disk.")
        mb.setInformativeText(
            "This run produced new planned_* (Validate/Optimize).\n\n"
            "Do you want to overwrite the stored planned_traj/planned_tcp?\n\n"
            "Executed_* will still be saved as usual when present."
        )
        mb.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        mb.setDefaultButton(QMessageBox.StandardButton.No)
        res = mb.exec()
        return res == int(QMessageBox.StandardButton.Yes)

    @staticmethod
    def _drop_planned_from_run_result(rr: RunResult) -> None:
        """
        Make planned_run effectively 'missing' so RecipeRepo keeps old planned_*.
        IMPORTANT: repo merge treats empty dict as missing (=> None) and keeps on disk.
        """
        if getattr(rr, "planned_run", None) is None or not isinstance(rr.planned_run, dict):
            rr.planned_run = {}
        rr.planned_run["traj"] = {}
        rr.planned_run["tcp"] = {}

    # ============================================================
    # STRICT context access
    # ============================================================

    def _required_offline_tf_paths(self) -> Tuple[str, str, str]:
        if not hasattr(self.ctx, "content") or self.ctx.content is None:
            raise RuntimeError("ctx.content is missing (strict)")

        scene_fn = getattr(self.ctx.content, "scene_yaml_path", None)
        robot_fn = getattr(self.ctx.content, "robot_yaml_path", None)
        if not callable(scene_fn) or not callable(robot_fn):
            raise RuntimeError("ctx.content must provide scene_yaml_path() and robot_yaml_path() (strict)")

        scene_yaml_path = str(scene_fn()).strip()
        robot_yaml_path = str(robot_fn()).strip()
        if not scene_yaml_path:
            raise RuntimeError("ctx.content.scene_yaml_path() returned empty (strict)")
        if not robot_yaml_path:
            raise RuntimeError("ctx.content.robot_yaml_path() returned empty (strict)")

        if not hasattr(self.ctx, "ros") or self.ctx.ros is None:
            raise RuntimeError("ctx.ros is missing (strict)")
        mounts_uri = str(getattr(self.ctx.ros, "substrate_mounts_file", "") or "").strip()
        if not mounts_uri:
            raise RuntimeError("ctx.ros.substrate_mounts_file is missing/empty (strict)")

        base = require_env_dir("SC_PROJECT_ROOT")
        mounts_yaml_path = resolve_path(base, mounts_uri)

        if not os.path.isfile(scene_yaml_path):
            raise RuntimeError(f"scene_yaml_path does not exist: {scene_yaml_path}")
        if not os.path.isfile(robot_yaml_path):
            raise RuntimeError(f"robot_yaml_path does not exist: {robot_yaml_path}")
        if not os.path.isfile(mounts_yaml_path):
            raise RuntimeError(f"mounts_yaml_path does not exist: {mounts_yaml_path}")

        return scene_yaml_path, robot_yaml_path, mounts_yaml_path

    def _default_segment_order(self) -> Tuple[str, str, str, str]:
        from tabs.process.process_panel.base_statemachine import (
            STATE_MOVE_PREDISPENSE,
            STATE_MOVE_RECIPE,
            STATE_MOVE_RETREAT,
            STATE_MOVE_HOME,
        )

        return (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME)

    def _refresh_stored_from_disk(self, key: str) -> None:
        key = str(key or "").strip()
        if not key:
            raise ValueError("_refresh_stored_from_disk: empty key")

        fresh = self.repo.load_for_process(key)
        self._recipe = fresh
        self._recipe_key = key

        recipe_name = getattr(fresh, "id", None) or getattr(fresh, "key", None) or key
        self.lblRecipeName.setText(f"Recipe: {recipe_name}")
        self.txtActiveInfo.setPlainText(self._format_active_recipe_info(fresh))
        self.txtRecipeParams.setPlainText(self._format_recipe_params(fresh))

        self._update_stored_view(fresh)

        # InfoBox must update whenever we rebind the recipe from disk
        self._update_infobox_from_recipe(fresh)

        self._republish_spraypaths_for_key(key, fresh)

    # ============================================================
    # Report strings (ONLY display, no local table logic)
    # ============================================================

    @staticmethod
    def _get_eval_report_text_from_tcp_obj(tcp_obj: Any) -> Optional[str]:
        if tcp_obj is None:
            return None
        ev = getattr(tcp_obj, "eval", None)
        if isinstance(ev, dict):
            for k in ("report_text_disk", "report_text_live", "report_text"):
                s = ev.get(k)
                if isinstance(s, str) and s.strip():
                    return s.strip()
        return None

    def _stored_report_text(self, recipe: Recipe) -> str:
        planned_obj = getattr(recipe, "planned_tcp", None)
        executed_obj = getattr(recipe, "executed_tcp", None)

        s = self._get_eval_report_text_from_tcp_obj(planned_obj)
        if s:
            return s
        s = self._get_eval_report_text_from_tcp_obj(executed_obj)
        if s:
            return s

        return "=== STORED EVAL (Disk) ===\nmissing: planned_tcp.eval['report_text*'] / executed_tcp.eval['report_text*']"

    def _current_run_report_text(self, rr: RunResult) -> str:
        ev = rr.eval if isinstance(getattr(rr, "eval", None), dict) else {}
        s = ev.get("report_text_live") if isinstance(ev, dict) else None
        if isinstance(s, str) and s.strip():
            return s.strip()

        if hasattr(rr, "report_text") and callable(getattr(rr, "report_text")):
            return rr.report_text(title="=== CURRENT RUN (Live) ===")

        return "=== CURRENT RUN (Live) ===\nmissing: rr.eval['report_text_live'] and rr.report_text()"

    # ============================================================
    # Publishing / overlays (STRICT: required ROS methods)
    # ============================================================

    def _ros_req(self, name: str):
        fn = getattr(self.ros, name, None)
        if not callable(fn):
            raise RuntimeError(f"ROS bridge missing required method: {name} (strict)")
        return fn

    def _republish_spraypaths_for_key(self, key: str, recipe_model: Recipe) -> None:
        key = str(key or "").strip()
        if not key:
            raise ValueError("_republish_spraypaths_for_key: empty key")

        self._ros_req("spray_clear")()

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
            self._ros_req("spray_set_compiled")(poses=pa if has_poses else None, markers=ma if has_markers else None)

        recipe_disk = self.repo.load_for_process(key)

        planned_obj = getattr(recipe_disk, "planned_tcp", None)
        if planned_obj:
            _, ma2 = recipe_markers.build_tcp_pose_and_markers(
                planned_obj,
                ns_prefix="planned_tcp/stored",
                mid_start=33000,
                default_frame="substrate",
                clear_legacy=True,
                line_width_m=0.0005,
                rgba_line=(0.7, 0.7, 0.7, 0.4),
                round_style="none",
            )
            if getattr(ma2, "markers", None) and len(ma2.markers) > 0:
                self._ros_req("spray_set_planned")(stored_markers=ma2)

        executed_obj = getattr(recipe_disk, "executed_tcp", None)
        if executed_obj:
            _, ma3 = recipe_markers.build_tcp_pose_and_markers(
                executed_obj,
                ns_prefix="executed_tcp/stored",
                mid_start=43000,
                default_frame="substrate",
                clear_legacy=True,
                line_width_m=0.0005,
                rgba_line=(0.7, 0.7, 0.7, 0.4),
                round_style="none",
            )
            if getattr(ma3, "markers", None) and len(ma3.markers) > 0:
                self._ros_req("spray_set_executed")(stored_markers=ma3)

    def _republish_newrun_overlays_from_rr(self, rr: RunResult) -> None:
        planned_tcp_doc = rr.planned_run.get("tcp") if isinstance(rr.planned_run, dict) else None
        executed_tcp_doc = rr.executed_run.get("tcp") if isinstance(rr.executed_run, dict) else None

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
                self._ros_req("spray_set_planned")(new_markers=ma)

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
                self._ros_req("spray_set_executed")(new_markers=ma)

    # ============================================================
    # InfoBox
    # ============================================================

    @staticmethod
    def _draft_points_mm_from_recipe(recipe: Recipe) -> Optional[np.ndarray]:
        """
        Extract Nx3 points for InfoGroupBox from recipe.draft.

        STRICT:
          - Use Draft API (recipe_poses_quat preferred).
          - Prefer side 'top' if present else first available side.
        """
        d = getattr(recipe, "draft", None)
        if d is None:
            return None

        if isinstance(d, Draft):
            draft = d
        elif isinstance(d, dict):
            draft = Draft.from_yaml_dict(d, name="recipe.draft")
        else:
            return None

        sides = list((draft.sides or {}).keys())
        if not sides:
            return None

        side = "top" if "top" in sides else sides[0]

        poses = []
        fn = getattr(draft, "recipe_poses_quat", None)
        if callable(fn):
            poses = list(fn(side) or [])
        if not poses:
            poses = list(draft.poses_quat(side) or [])

        if not poses:
            return None

        pts = np.array([[p.x, p.y, p.z] for p in poses], dtype=float).reshape(-1, 3)
        return pts if pts.size > 0 else None

    def _update_infobox_from_recipe(self, recipe: Recipe) -> None:
        pts = self._draft_points_mm_from_recipe(recipe)
        self.infoBox.update_from_recipe(recipe, points=pts)

    def _update_stored_view(self, recipe: Recipe) -> None:
        try:
            self.txtStored.setPlainText(self._stored_report_text(recipe))
        except Exception as e:
            self.txtStored.setPlainText(f"=== STORED EVAL (Disk) ===\n(render failed: {e})")

    # ============================================================
    # UI actions
    # ============================================================

    def _on_load_clicked(self) -> None:
        keys = self.repo.list_recipes()
        choice, ok = QInputDialog.getItem(self, "Load", "Select Recipe:", keys, 0, False)
        if ok and choice:
            model = self.repo.load_for_process(choice)
            self.set_recipe(choice, model)
