# -*- coding: utf-8 -*-
# File: tabs/process/process_tab.py
from __future__ import annotations

import logging
from typing import Optional, Any, Dict

import yaml

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QPushButton,
    QTextEdit,
    QLabel,
    QMessageBox,
    QCheckBox,
)

from visualization_msgs.msg import MarkerArray

from plc.plc_client import PlcClientBase
from ros.bridge.ros_bridge import RosBridge

from model.recipe.recipe import Recipe
from model.recipe.recipe_markers import build_marker_array_from_recipe

from .process_thread import ProcessThread
from .robot_init_thread import RobotInitThread

_LOG = logging.getLogger("tabs.process")


class ProcessTab(QWidget):
    """
    ProcessTab

    - Init / Validate / Optimize / Execute / Stop
    - Layer-Checkboxes: compiled / traj / executed (Marker Sichtbarkeit)
    - Persistenz optional per Checkboxen

    Contract:
      - Thread bekommt IMMER ein "process loaded" Recipe (draft+compiled+last runs)
      - Ergebnis (planned/executed) wird im Tab übernommen, EVAL läuft IMMER vor:
          - Anzeige
          - Speichern
    """

    def __init__(
        self,
        *,
        ctx,
        repo,
        ros: Optional[RosBridge],
        plc: PlcClientBase | None = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.repo = repo  # ✅ RecipeRepo (SSoT: ctx.repo)
        self.ros = ros
        self.plc = plc

        if self.repo is None:
            # ProcessTab braucht Repo zwingend (load/save)
            raise RuntimeError("ProcessTab: repo ist None (ctx.repo fehlt?)")

        # currently displayed recipe in tab (used for markers/score)
        self._recipe: Optional[Recipe] = None
        self._recipe_id: Optional[str] = None

        self._process_thread: Optional[ProcessThread] = None

        # Robot init gating
        self._init_thread: Optional[RobotInitThread] = None
        self._robot_ready: bool = False

        self._process_active: bool = False
        self._active_mode: str = ""

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ---------------- Process buttons ----------------
        grp_proc = QGroupBox("Process", self)
        vproc = QVBoxLayout(grp_proc)

        row_load = QHBoxLayout()
        self.btnLoad = QPushButton("Load", grp_proc)
        self.btnLoad.setMinimumHeight(28)
        self.lblRecipe = QLabel("Recipe: –", grp_proc)
        row_load.addWidget(self.btnLoad)
        row_load.addWidget(self.lblRecipe, 1)
        vproc.addLayout(row_load)

        self.btnInit = QPushButton("Init", grp_proc)
        self.btnValidate = QPushButton("Validate", grp_proc)
        self.btnOptimize = QPushButton("Optimize", grp_proc)
        self.btnExecute = QPushButton("Execute", grp_proc)
        self.btnStop = QPushButton("Stop", grp_proc)

        for b in (self.btnInit, self.btnValidate, self.btnOptimize, self.btnExecute, self.btnStop):
            b.setMinimumHeight(28)
            vproc.addWidget(b)

        self.lblInit = QLabel("Init: –", grp_proc)
        vproc.addWidget(self.lblInit)

        # optional Persistenz
        self.chkPersistTraj = QCheckBox("Persist planned traj", grp_proc)
        self.chkPersistExecuted = QCheckBox("Persist executed traj", grp_proc)
        self.chkPersistTraj.setChecked(False)
        self.chkPersistExecuted.setChecked(False)
        vproc.addSpacing(6)
        vproc.addWidget(self.chkPersistTraj)
        vproc.addWidget(self.chkPersistExecuted)

        root.addWidget(grp_proc)

        # ---------------- SprayPath view (checkboxes + score) ----------------
        grp_view = QGroupBox("SprayPath View", self)
        vview = QVBoxLayout(grp_view)

        row = QHBoxLayout()
        self.chkCompiled = QCheckBox("Compiled", grp_view)
        self.chkTraj = QCheckBox("Traj", grp_view)
        self.chkExecuted = QCheckBox("Executed", grp_view)

        self.chkCompiled.setChecked(True)
        self.chkTraj.setChecked(True)
        self.chkExecuted.setChecked(True)

        row.addWidget(self.chkCompiled)
        row.addWidget(self.chkTraj)
        row.addWidget(self.chkExecuted)
        vview.addLayout(row)

        self.lblScore = QLabel("Score: –", grp_view)
        vview.addWidget(self.lblScore)

        root.addWidget(grp_view)

        # ---------------- Results ----------------
        grp_res = QGroupBox("Run Data (scored)", self)
        vres = QVBoxLayout(grp_res)
        row_res = QHBoxLayout()

        self.txtPlanned = QTextEdit(grp_res)
        self.txtPlanned.setReadOnly(True)
        self.txtPlanned.setPlaceholderText("planned traj (traj.yaml) – inkl. eval/score")

        self.txtExecuted = QTextEdit(grp_res)
        self.txtExecuted.setReadOnly(True)
        self.txtExecuted.setPlaceholderText("executed traj (executed_traj.yaml) – inkl. eval/score")

        row_res.addWidget(self.txtPlanned, 1)
        row_res.addWidget(self.txtExecuted, 1)
        vres.addLayout(row_res)

        root.addWidget(grp_res)

        # ---------------- Log ----------------
        grp_log = QGroupBox("Log", self)
        vlog = QVBoxLayout(grp_log)
        self.txtLog = QTextEdit(grp_log)
        self.txtLog.setReadOnly(True)
        vlog.addWidget(self.txtLog)
        root.addWidget(grp_log, 1)

        # ---------------- signals ----------------
        self.btnLoad.clicked.connect(self._on_load_clicked)

        self.btnInit.clicked.connect(self._on_init_clicked)
        self.btnValidate.clicked.connect(self._on_validate_clicked)
        self.btnOptimize.clicked.connect(self._on_optimize_clicked)
        self.btnExecute.clicked.connect(self._on_execute_clicked)
        self.btnStop.clicked.connect(self._on_stop_clicked)

        self.chkCompiled.toggled.connect(self._publish_layers)
        self.chkTraj.toggled.connect(self._publish_layers)
        self.chkExecuted.toggled.connect(self._publish_layers)

        # init thread
        self._setup_init_thread()

        self._update_buttons()
        self._update_layer_enablement()

    # ---------------- Utils ----------------

    def _append_log(self, msg: str) -> None:
        if not msg:
            return
        self.txtLog.append(msg)

    def _yaml_dump(self, obj: Any) -> str:
        try:
            return yaml.safe_dump(obj or {}, allow_unicode=True, sort_keys=False)
        except Exception:
            return str(obj)

    def _compiled_sides(self) -> list[str]:
        try:
            pc = getattr(self._recipe, "paths_compiled", {}) or {}
            sides = pc.get("sides") or {}
            if isinstance(sides, dict) and sides:
                return [str(k) for k in sides.keys()]
        except Exception:
            pass
        # fallback: active_side
        try:
            side = str((getattr(self._recipe, "parameters", {}) or {}).get("active_side", "top"))
            return [side]
        except Exception:
            return ["top"]

    def _active_side(self) -> str:
        try:
            return str((getattr(self._recipe, "parameters", {}) or {}).get("active_side", "top"))
        except Exception:
            return "top"

    # ---------------- Public API ----------------

    def set_recipe(self, recipe: Optional[Recipe]) -> None:
        if self._process_active:
            QMessageBox.warning(self, "Process", "Während eines laufenden Prozesses kann kein Rezept gewechselt werden.")
            return
        if self._init_thread is not None and self._init_thread.isRunning():
            QMessageBox.warning(self, "Process", "Während RobotInit läuft kann kein Rezept gewechselt werden.")
            return

        self._recipe = recipe
        self._recipe_id = getattr(recipe, "id", None) if recipe else None

        self.lblRecipe.setText(f"Recipe: {self._recipe_id}" if self._recipe_id else "Recipe: –")
        self._append_log(f"Recipe gesetzt: {self._recipe_id}")

        self._update_buttons()
        self._update_layer_enablement()

        self.txtPlanned.clear()
        self.txtExecuted.clear()

        if recipe:
            self._publish_layers()
            self._update_spray_score_label(prefer_executed=True)
            self._render_run_yaml_views()
        else:
            self._clear_layers()
            self._update_spray_score_label(prefer_executed=True)

    # ---------------- Load ----------------

    def _on_load_clicked(self) -> None:
        if self._process_active:
            QMessageBox.warning(self, "Load", "Während eines laufenden Prozesses nicht möglich.")
            return
        if self._init_thread is not None and self._init_thread.isRunning():
            QMessageBox.warning(self, "Load", "Während RobotInit läuft nicht möglich.")
            return

        if not self._recipe_id:
            QMessageBox.information(self, "Load", "Kein Rezept ausgewählt.")
            return

        try:
            r = self.repo.load_for_process(self._recipe_id)
            self.set_recipe(r)
            self._append_log("Load: OK")
        except Exception as e:
            QMessageBox.critical(self, "Load", f"Load fehlgeschlagen: {e}")

    # ---------------- Robot Init ----------------

    def _setup_init_thread(self) -> None:
        if self.ros is None:
            return
        try:
            self._init_thread = RobotInitThread(ros=self.ros)
            self._init_thread.notifyFinished.connect(self._on_init_finished_ok)
            self._init_thread.notifyError.connect(self._on_init_finished_err)
            self._init_thread.logMessage.connect(self._append_log)
            self._init_thread.stateChanged.connect(lambda s: self.lblInit.setText(f"Init: {s}"))
        except Exception:
            _LOG.exception("RobotInitThread setup failed")

    def _on_init_clicked(self) -> None:
        if self._init_thread is None:
            QMessageBox.warning(self, "Init", "ROS nicht verfügbar.")
            return
        if self._process_active:
            QMessageBox.warning(self, "Init", "Während eines laufenden Prozesses nicht möglich.")
            return
        self._robot_ready = False
        self.lblInit.setText("Init: START")
        self._append_log("=== Robot-Init gestartet ===")
        self._init_thread.startSignal.emit()
        self._update_buttons()

    def _on_init_finished_ok(self) -> None:
        self._robot_ready = True
        self.lblInit.setText("Init: OK")
        self._append_log("=== Robot-Init erfolgreich abgeschlossen ===")
        self._update_buttons()

    def _on_init_finished_err(self, msg: str) -> None:
        self._robot_ready = False
        self.lblInit.setText("Init: ERROR")
        self._append_log(f"=== Robot-Init Fehler: {msg} ===")
        self._update_buttons()

    # ---------------- Process actions ----------------

    def _on_validate_clicked(self) -> None:
        self._start_process(ProcessThread.MODE_VALIDATE)

    def _on_optimize_clicked(self) -> None:
        self._start_process(ProcessThread.MODE_OPTIMIZE)

    def _on_execute_clicked(self) -> None:
        self._start_process(ProcessThread.MODE_EXECUTE)

    def _on_stop_clicked(self) -> None:
        if self._process_thread is not None:
            self._append_log("Stop: request_stop()")
            self._process_thread.request_stop()
        if self._init_thread is not None and self._init_thread.isRunning():
            self._append_log("Stop: RobotInit request_stop()")
            self._init_thread.request_stop()

    def _start_process(self, mode: str) -> None:
        if self._process_active:
            QMessageBox.warning(self, "Process", "Es läuft bereits ein Prozess.")
            return
        if not self._recipe_id:
            QMessageBox.warning(self, "Process", "Kein Rezept geladen.")
            return
        if self.ros is None:
            QMessageBox.warning(self, "Process", "ROS nicht verfügbar.")
            return

        # gating
        if mode in (ProcessThread.MODE_VALIDATE, ProcessThread.MODE_OPTIMIZE, ProcessThread.MODE_EXECUTE):
            if not self._robot_ready:
                QMessageBox.warning(self, "Process", "Robot nicht initialisiert. Bitte zuerst Init.")
                return

        if mode == ProcessThread.MODE_EXECUTE and self.plc is None:
            QMessageBox.warning(self, "Execute", "PLC ist nicht verfügbar (Execute benötigt PLC).")
            return

        # IMPORTANT: load fresh process recipe snapshot for this run
        try:
            recipe_run = self.repo.load_for_process(self._recipe_id)
        except Exception as e:
            QMessageBox.critical(self, "Process", f"Rezept laden fehlgeschlagen: {e}")
            return

        self._process_active = True
        self._active_mode = mode
        self.txtLog.clear()
        self.txtPlanned.clear()
        self.txtExecuted.clear()
        self._append_log(f"=== {mode.upper()} gestartet ===")

        self._update_buttons()
        self._update_layer_enablement()

        self._process_thread = ProcessThread(
            recipe=recipe_run,
            ros=self.ros,
            plc=self.plc,
            mode=mode,
        )
        self._process_thread.stateChanged.connect(lambda s: self._append_log(f"STATE: {s}"))
        self._process_thread.logMessage.connect(self._append_log)
        self._process_thread.notifyFinished.connect(self._on_process_finished_success)
        self._process_thread.notifyError.connect(self._on_process_finished_error)
        self._process_thread.start()

    # ---------------- Process result handling ----------------

    def _apply_payload_to_recipe_and_eval(self, payload: Dict[str, Any]) -> None:
        if self._recipe is None:
            return

        planned = payload.get("planned_traj")
        executed = payload.get("executed_traj")

        if isinstance(planned, dict):
            self._recipe.trajectories[Recipe.TRAJ_TRAJ] = planned
        if isinstance(executed, dict):
            self._recipe.trajectories[Recipe.TRAJ_EXECUTED] = executed

        sides = self._compiled_sides()
        for side in sides:
            try:
                if isinstance(self._recipe.trajectories.get(Recipe.TRAJ_TRAJ), dict):
                    self._recipe.evaluate_trajectory_against_compiled(traj_key=Recipe.TRAJ_TRAJ, side=side)
            except Exception:
                pass
            try:
                if isinstance(self._recipe.trajectories.get(Recipe.TRAJ_EXECUTED), dict):
                    self._recipe.evaluate_trajectory_against_compiled(traj_key=Recipe.TRAJ_EXECUTED, side=side)
            except Exception:
                pass

    def _persist_runs_if_enabled(self) -> None:
        if self._recipe is None or not self._recipe_id:
            return

        # save planned
        if self.chkPersistTraj.isChecked():
            traj = self._recipe.trajectories.get(Recipe.TRAJ_TRAJ)
            if isinstance(traj, dict):
                try:
                    self.repo.save_traj_run(self._recipe_id, traj=traj)
                    self._append_log("I persist: planned traj gespeichert (scored).")
                except Exception as e:
                    self._append_log(f"W persist: planned traj konnte nicht gespeichert werden: {e}")

        # save executed
        if self.chkPersistExecuted.isChecked():
            ex = self._recipe.trajectories.get(Recipe.TRAJ_EXECUTED)
            if isinstance(ex, dict):
                try:
                    self.repo.save_executed_run(self._recipe_id, executed=ex)
                    self._append_log("I persist: executed traj gespeichert (scored).")
                except Exception as e:
                    self._append_log(f"W persist: executed traj konnte nicht gespeichert werden: {e}")

    def _render_run_yaml_views(self) -> None:
        if self._recipe is None:
            self.txtPlanned.clear()
            self.txtExecuted.clear()
            return

        t = self._recipe.trajectories.get(Recipe.TRAJ_TRAJ)
        e = self._recipe.trajectories.get(Recipe.TRAJ_EXECUTED)

        self.txtPlanned.setPlainText(self._yaml_dump(t if isinstance(t, dict) else {}))
        self.txtExecuted.setPlainText(self._yaml_dump(e if isinstance(e, dict) else {}))

    def _on_process_finished_success(self, result_obj: object) -> None:
        payload: Dict[str, Any] = result_obj if isinstance(result_obj, dict) else {}
        self._append_log("=== Process finished ===")

        # reload base process recipe (draft+compiled) fresh, then apply runs+eval on top
        if self._recipe_id:
            try:
                self._recipe = self.repo.load_for_process(self._recipe_id)
            except Exception:
                pass

        self._apply_payload_to_recipe_and_eval(payload)

        self._update_spray_score_label(prefer_executed=True)
        self._publish_layers()
        self._render_run_yaml_views()

        self._persist_runs_if_enabled()

        self._process_active = False
        self._active_mode = ""
        self._process_thread = None
        self._update_buttons()
        self._update_layer_enablement()

    def _on_process_finished_error(self, msg: str) -> None:
        self._append_log(f"=== Process ERROR: {msg} ===")
        self._process_active = False
        self._active_mode = ""
        self._process_thread = None
        self._update_buttons()
        self._update_layer_enablement()

    # ---------------- Marker layers / score ----------------

    def _clear_layers(self) -> None:
        if self.ros is None:
            return
        try:
            self.ros.publish_process_markers(MarkerArray())
        except Exception:
            pass

    def _publish_layers(self) -> None:
        if self.ros is None or self._recipe is None:
            return

        try:
            ma = MarkerArray()

            if self.chkCompiled.isChecked():
                ma_c = build_marker_array_from_recipe(self._recipe, source=Recipe.TRAJ_COMPILED, frame_id="scene")
                ma.markers.extend(ma_c.markers)

            if self.chkTraj.isChecked():
                if isinstance(self._recipe.trajectories.get(Recipe.TRAJ_TRAJ), dict):
                    ma_t = build_marker_array_from_recipe(self._recipe, source=Recipe.TRAJ_TRAJ, frame_id="scene")
                    ma.markers.extend(ma_t.markers)

            if self.chkExecuted.isChecked():
                if isinstance(self._recipe.trajectories.get(Recipe.TRAJ_EXECUTED), dict):
                    ma_e = build_marker_array_from_recipe(self._recipe, source=Recipe.TRAJ_EXECUTED, frame_id="scene")
                    ma.markers.extend(ma_e.markers)

            self.ros.publish_process_markers(ma)
        except Exception as e:
            self._append_log(f"W publish_layers failed: {e}")

    def _update_spray_score_label(self, *, prefer_executed: bool = True) -> None:
        if self._recipe is None:
            self.lblScore.setText("Score: –")
            return

        side = self._active_side()

        def _score_for(traj_key: str) -> Optional[float]:
            try:
                t = self._recipe.trajectories.get(traj_key)
                if not isinstance(t, dict):
                    return None
                s = (t.get("sides") or {}).get(side) or {}
                ev = s.get("eval") or {}
                if isinstance(ev, dict) and "score" in ev:
                    return float(ev.get("score"))
            except Exception:
                return None
            return None

        score = None
        src = ""

        if prefer_executed:
            score = _score_for(Recipe.TRAJ_EXECUTED)
            src = "executed"
            if score is None:
                score = _score_for(Recipe.TRAJ_TRAJ)
                src = "traj"
        else:
            score = _score_for(Recipe.TRAJ_TRAJ)
            src = "traj"
            if score is None:
                score = _score_for(Recipe.TRAJ_EXECUTED)
                src = "executed"

        if score is None:
            self.lblScore.setText("Score: –")
        else:
            self.lblScore.setText(f"Score ({src}/{side}): {score:.1f}")

    # ---------------- UI gating ----------------

    def _update_layer_enablement(self) -> None:
        enabled = bool(self._recipe is not None)
        for w in (self.chkCompiled, self.chkTraj, self.chkExecuted):
            w.setEnabled(enabled)

    def _update_buttons(self) -> None:
        has_recipe = bool(self._recipe_id)
        ros_ok = self.ros is not None

        self.btnLoad.setEnabled(not self._process_active and ros_ok and has_recipe)

        self.btnInit.setEnabled(not self._process_active and ros_ok and has_recipe and (self._init_thread is not None))
        self.btnValidate.setEnabled(not self._process_active and ros_ok and has_recipe and self._robot_ready)
        self.btnOptimize.setEnabled(not self._process_active and ros_ok and has_recipe and self._robot_ready)
        self.btnExecute.setEnabled(
            (not self._process_active) and ros_ok and has_recipe and self._robot_ready and (self.plc is not None)
        )
        self.btnStop.setEnabled(self._process_active or (self._init_thread is not None and self._init_thread.isRunning()))
