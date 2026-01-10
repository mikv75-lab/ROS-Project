# -*- coding: utf-8 -*-
# File: tabs/process/process_tab.py
from __future__ import annotations

import logging
import os
from typing import Optional, Any, Dict, List, Tuple

import yaml

from PyQt6 import QtCore
from PyQt6.QtCore import QTimer
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

from visualization_msgs.msg import MarkerArray

from plc.plc_client import PlcClientBase
from ros.bridge.ros_bridge import RosBridge

from model.recipe.recipe import Recipe, Draft, JTBySegment
from model.recipe.recipe_markers import build_marker_array_from_recipe

from model.recipe.traj_fk_builder import TrajFkBuilder, TrajFkConfig

# STRICT result schema (single source of truth)
from model.recipe.recipe_run_result import RunResult

from widgets.robot_status_box import RobotStatusInfoBox
from widgets.info_groupbox import InfoGroupBox

from tabs.process.spray_path_box import SprayPathBox  # REQUIRED (no fallbacks)

from .process_thread import ProcessThread
from .robot_init_thread import RobotInitThread

from .base_statemachine import (
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
)

_LOG = logging.getLogger("tabs.process")


class ProcessTab(QWidget):
    """
    ProcessTab (strict, no fallbacks, single result schema)

    STRICT CONTRACT (aligned to app/model/recipe/recipe_run_result.py RunResult.to_process_payload):

      result_obj = {
        "planned_run":  {"traj": <JTBySegment YAML v1 dict>, "tcp": <Draft YAML dict oder {}>},
        "executed_run": {"traj": <JTBySegment YAML v1 dict>, "tcp": <Draft YAML dict oder {}>},
        "fk_meta": {...},
        "eval": {...},
        "valid": true/false,
        "invalid_reason": "..."
      }

    Persistence:
      - planned_traj.yaml and executed_traj.yaml are written as JTBySegment YAML v1
        with optional top-level key 'eval' (dict)
      - Save happens only if result_obj["valid"] is True.
    """

    _SEG_ORDER = (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME)

    def __init__(
        self,
        *,
        ctx,
        repo,
        ros: RosBridge,
        plc: PlcClientBase | None = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)

        # ---- strict dependencies ----
        if repo is None:
            raise RuntimeError("ProcessTab: repo ist None (ctx.repo fehlt?)")
        if ros is None:
            raise RuntimeError("ProcessTab: ros ist None (strict)")
        self.ctx = ctx
        self.repo = repo
        self.ros: RosBridge = ros
        self.plc = plc

        # ---- strict repo API (must exist) ----
        for name in ("list_recipes", "load_for_process"):
            fn = getattr(self.repo, name, None)
            if not callable(fn):
                raise RuntimeError(f"ProcessTab: repo missing required API: {name}()")
        if getattr(getattr(self.repo, "bundle", None), "paths", None) is None:
            raise RuntimeError("ProcessTab: repo.bundle.paths fehlt (strict)")

        self._recipe: Optional[Recipe] = None
        self._recipe_key: Optional[str] = None

        self._process_thread: Optional[ProcessThread] = None

        # Robot init gating
        self._init_thread: Optional[RobotInitThread] = None
        self._robot_ready: bool = False

        self._process_active: bool = False
        self._active_mode: str = ""

        # ---- SprayPath UI state (default all TRUE) ----
        self._show_compiled: bool = True
        self._show_traj: bool = True
        self._show_executed: bool = True

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # =========================================================
        # TOP ROW
        # =========================================================
        top_row = QHBoxLayout()
        top_row.setSpacing(8)

        # ---------------- Recipe GroupBox (links) ----------------
        grp_recipe = QGroupBox("Recipe", self)
        vrec = QVBoxLayout(grp_recipe)
        vrec.setSpacing(6)

        row_load = QHBoxLayout()
        self.btnLoad = QPushButton("Load", grp_recipe)
        self.btnLoad.setMinimumHeight(28)
        self.lblRecipeKey = QLabel("Key: –", grp_recipe)
        row_load.addWidget(self.btnLoad)
        row_load.addWidget(self.lblRecipeKey, 1)
        vrec.addLayout(row_load)

        self.lblRecipeMeta = QLabel("Info: –", grp_recipe)
        self.lblRecipeMeta.setWordWrap(True)
        vrec.addWidget(self.lblRecipeMeta)

        self.txtRecipeDraft = QTextEdit(grp_recipe)
        self.txtRecipeDraft.setReadOnly(True)
        self.txtRecipeDraft.setPlaceholderText("Draft info (parameters/planner/paths_by_side/meta)")
        sp = self.txtRecipeDraft.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.txtRecipeDraft.setSizePolicy(sp)
        vrec.addWidget(self.txtRecipeDraft, 1)

        top_row.addWidget(grp_recipe, 2)

        # ---------------- Process GroupBox (mitte) ----------------
        grp_proc = QGroupBox("Process", self)
        vproc = QVBoxLayout(grp_proc)
        vproc.setSpacing(6)

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

        top_row.addWidget(grp_proc, 1)

        # ---------------- Robot Status (rechts) ----------------
        self.robotStatusBox = RobotStatusInfoBox(self, title="Robot Status")
        top_row.addWidget(self.robotStatusBox, 2)

        root.addLayout(top_row)

        # =========================================================
        # INFO + SPRAY PATHS ROW
        # =========================================================
        row_info = QHBoxLayout()
        row_info.setSpacing(8)

        self.infoBox = InfoGroupBox(self)
        row_info.addWidget(self.infoBox, 2)

        self.sprayPathBox = SprayPathBox(ros=self.ros, parent=self, title="Spray Paths")
        row_info.addWidget(self.sprayPathBox, 1)

        root.addLayout(row_info)

        # =========================================================
        # EVAL (planned/executed) + LOG ROW
        # =========================================================
        row_eval_log = QHBoxLayout()
        row_eval_log.setSpacing(8)

        left_eval = QVBoxLayout()
        left_eval.setSpacing(8)

        grp_pl = QGroupBox("Planned (Eval)", self)
        vpl = QVBoxLayout(grp_pl)
        self.txtPlannedEval = QTextEdit(grp_pl)
        self.txtPlannedEval.setReadOnly(True)
        self.txtPlannedEval.setPlaceholderText("Loaded from planned_traj.yaml: eval")
        vpl.addWidget(self.txtPlannedEval, 1)
        self.lblPlannedSummary = QLabel("planned: –", grp_pl)
        vpl.addWidget(self.lblPlannedSummary)
        left_eval.addWidget(grp_pl, 1)

        grp_ex = QGroupBox("Executed (Eval)", self)
        vex = QVBoxLayout(grp_ex)
        self.txtExecutedEval = QTextEdit(grp_ex)
        self.txtExecutedEval.setReadOnly(True)
        self.txtExecutedEval.setPlaceholderText("Loaded from executed_traj.yaml: eval")
        vex.addWidget(self.txtExecutedEval, 1)
        self.lblExecutedSummary = QLabel("executed: –", grp_ex)
        vex.addWidget(self.lblExecutedSummary)
        left_eval.addWidget(grp_ex, 1)

        row_eval_log.addLayout(left_eval, 2)

        grp_log = QGroupBox("Log", self)
        vlog = QVBoxLayout(grp_log)
        self.txtLog = QTextEdit(grp_log)
        self.txtLog.setReadOnly(True)
        vlog.addWidget(self.txtLog, 1)
        row_eval_log.addWidget(grp_log, 3)

        root.addLayout(row_eval_log, 2)

        # ---------------- signals ----------------
        self.btnLoad.clicked.connect(self._on_load_clicked)

        self.btnInit.clicked.connect(self._on_init_clicked)
        self.btnValidate.clicked.connect(self._on_validate_clicked)
        self.btnOptimize.clicked.connect(self._on_optimize_clicked)
        self.btnExecute.clicked.connect(self._on_execute_clicked)
        self.btnStop.clicked.connect(self._on_stop_clicked)

        self.sprayPathBox.showCompiledToggled.connect(self._on_show_compiled_toggled)
        self.sprayPathBox.showTrajToggled.connect(self._on_show_traj_toggled)
        self.sprayPathBox.showExecutedToggled.connect(self._on_show_executed_toggled)

        self._setup_init_thread()
        self._wire_robot_status_inbound()

        self._spray_defaults_publish(compiled=True, planned=True, executed=True)

        self._update_buttons()
        self._update_recipe_box()
        self._update_info_box()
        self._clear_eval_views()

    # ---------------------------------------------------------------------
    # PLC availability (Execute) - supports sim mode even if plc is None
    # ---------------------------------------------------------------------

    def _plc_execute_available(self) -> bool:
        """
        Execute requires PLC in live mode, BUT:
          - if ctx.plc.sim == True -> Execute is allowed even if self.plc is None
        """
        if self.plc is not None:
            return True
        try:
            plc_cfg = getattr(self.ctx, "plc", None)
            if plc_cfg is not None and bool(getattr(plc_cfg, "sim", False)):
                return True
        except Exception:
            pass
        return False

    # ---------------------------------------------------------------------
    # SprayPathBox compat helpers (traj vs planned) - UI only
    # ---------------------------------------------------------------------

    def _spray_defaults_publish(self, *, compiled: bool, planned: bool, executed: bool) -> None:
        sb = getattr(self, "sprayPathBox", None)
        if sb is None:
            return
        fn = getattr(sb, "set_defaults", None)
        if not callable(fn):
            return

        try:
            fn(compiled=bool(compiled), planned=bool(planned), executed=bool(executed))
            return
        except TypeError:
            pass

        try:
            fn(compiled=bool(compiled), traj=bool(planned), executed=bool(executed))
            return
        except TypeError:
            pass

        try:
            fn(bool(compiled), bool(planned), bool(executed))
        except Exception:
            _LOG.exception("ProcessTab: SprayPathBox.set_defaults compat call failed")

    # ---------------- Utils ----------------

    def _append_log(self, msg: str) -> None:
        if msg:
            self.txtLog.append(msg)

    def _yaml_dump(self, obj: Any) -> str:
        try:
            return yaml.safe_dump(obj or {}, allow_unicode=True, sort_keys=False)
        except Exception:
            return str(obj)

    def _yaml_load_file(self, path: str) -> Dict[str, Any]:
        if not path or not os.path.isfile(path):
            return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            return data if isinstance(data, dict) else {}
        except Exception:
            return {}

    def _yaml_write_file(self, path: str, obj: Any) -> None:
        if not path:
            raise ValueError("yaml_write_file: empty path")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(obj or {}, f, allow_unicode=True, sort_keys=False)

    def _list_repo_keys(self) -> List[str]:
        keys = self.repo.list_recipes() or []
        keys = [str(k) for k in keys if isinstance(k, str) and str(k).strip()]
        keys.sort()
        return keys

    # ------------------------------------------------------------
    # Result extraction (STRICT: RunResult.to_process_payload schema)
    # ------------------------------------------------------------

    @staticmethod
    def _is_jtbysegment_yaml_v1(d: Any) -> bool:
        if not isinstance(d, dict):
            return False
        try:
            ver = int(d.get("version", 0))
        except Exception:
            return False
        if ver != 1:
            return False
        segs = d.get("segments", None)
        return isinstance(segs, dict) and bool(segs)

    def _extract_from_result_strict(
        self, result_obj: object
    ) -> Tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any], bool, str]:
        """
        Returns:
          planned_traj_yaml, executed_traj_yaml, eval_dict, valid, invalid_reason
        """
        if not isinstance(result_obj, dict):
            raise ValueError("RunResult muss dict sein (strict).")

        planned_run = result_obj.get("planned_run")
        executed_run = result_obj.get("executed_run")
        if not isinstance(planned_run, dict) or not isinstance(executed_run, dict):
            raise ValueError("planned_run/executed_run fehlen oder sind ungültig (strict).")

        planned_traj = planned_run.get("traj", None)
        executed_traj = executed_run.get("traj", None)
        if not self._is_jtbysegment_yaml_v1(planned_traj):
            raise ValueError("planned_run['traj'] fehlt/ungültig (JTBySegment v1 strict).")
        if not self._is_jtbysegment_yaml_v1(executed_traj):
            raise ValueError("executed_run['traj'] fehlt/ungültig (JTBySegment v1 strict).")

        eval_dict = result_obj.get("eval", {})
        if not isinstance(eval_dict, dict):
            raise ValueError("eval ist kein dict (strict).")

        valid = result_obj.get("valid", False)
        if not isinstance(valid, bool):
            raise ValueError("valid ist kein bool (strict).")

        invalid_reason = result_obj.get("invalid_reason", "")
        if not isinstance(invalid_reason, str):
            raise ValueError("invalid_reason ist kein str (strict).")

        return dict(planned_traj), dict(executed_traj), dict(eval_dict), bool(valid), str(invalid_reason or "")

    # ------------------------------------------------------------
    # FK (JTBySegment -> Draft TCP) - best effort
    # ------------------------------------------------------------

    def _get_robot_model_best_effort(self) -> Any:
        ros = self.ros
        candidates = [
            ("moveitpy", "robot_model"),
            ("moveitpy", "_robot_model"),
            ("moveit_py", "robot_model"),
            ("moveit_py", "_robot_model"),
        ]
        for a, b in candidates:
            try:
                obj = getattr(ros, a, None)
                if obj is None:
                    continue
                rm = getattr(obj, b, None)
                if rm is not None:
                    return rm
            except Exception:
                continue

        for attr in ("robot_model", "_robot_model"):
            try:
                rm = getattr(ros, attr, None)
                if rm is not None:
                    return rm
            except Exception:
                pass
        return None

    def _fk_tcp_best_effort(self, *, traj: JTBySegment, recipe: Recipe) -> Optional[Draft]:
        robot_model = self._get_robot_model_best_effort()
        if robot_model is None:
            return None

        ee_link = "tcp"
        step_mm = 1.0
        max_points = 0
        try:
            params = getattr(recipe, "parameters", {}) or {}
            ee_link = str(params.get("fk_ee_link", ee_link) or ee_link)
            step_mm = float(params.get("fk_step_mm", step_mm))
            max_points = int(params.get("fk_max_points", max_points))
        except Exception:
            pass

        cfg = TrajFkConfig(ee_link=ee_link, step_mm=step_mm, max_points=max_points)
        try:
            return TrajFkBuilder.build_tcp_draft(traj, robot_model=robot_model, cfg=cfg, default_side="top")
        except Exception as e:
            self._append_log(f"W FK skipped: {e}")
            return None

    # ---------------- Spray toggles (cache only; publish is done by SprayPathBox) ----------------

    def _on_show_compiled_toggled(self, v: bool) -> None:
        self._show_compiled = bool(v)

    def _on_show_traj_toggled(self, v: bool) -> None:
        self._show_traj = bool(v)

    def _on_show_executed_toggled(self, v: bool) -> None:
        self._show_executed = bool(v)

    # ---------------- RobotStatus: INBOUND wiring ----------------

    @QtCore.pyqtSlot(object)
    def _on_joints(self, js) -> None:
        if js is None or not hasattr(js, "position"):
            self.robotStatusBox.set_joints(None)
            return
        try:
            self.robotStatusBox.set_joints(list(js.position or []))
        except Exception:
            self.robotStatusBox.set_joints(None)

    def _wire_robot_status_inbound(self) -> None:
        try:
            rb = self.ros.robot
            sig = rb.signals

            sig.connectionChanged.connect(self.robotStatusBox.set_connection)
            sig.modeChanged.connect(self.robotStatusBox.set_mode)
            sig.initializedChanged.connect(self.robotStatusBox.set_initialized)
            sig.movingChanged.connect(self.robotStatusBox.set_moving)
            sig.powerChanged.connect(self.robotStatusBox.set_power)
            sig.servoEnabledChanged.connect(self.robotStatusBox.set_servo_enabled)
            sig.estopChanged.connect(self.robotStatusBox.set_estop)
            sig.errorsChanged.connect(self.robotStatusBox.set_errors)
            sig.tcpPoseChanged.connect(self.robotStatusBox.set_tcp_from_ps)
            sig.jointsChanged.connect(self._on_joints)
        except Exception as e:
            raise RuntimeError(f"ProcessTab: robot status wiring failed: {e}")

    # ---------------- Recipe UI ----------------

    def _update_recipe_box(self) -> None:
        if self._recipe is None:
            self.lblRecipeKey.setText("Key: –")
            self.lblRecipeMeta.setText("Info: –")
            self.txtRecipeDraft.clear()
            return

        r = self._recipe
        key = self._recipe_key or "-"
        self.lblRecipeKey.setText(f"Key: {key}")

        parts = [f"id={r.id}"]
        if r.tool:
            parts.append(f"tool={r.tool}")
        if r.substrate:
            parts.append(f"substrate={r.substrate}")
        if r.substrate_mount:
            parts.append(f"mount={r.substrate_mount}")
        if r.description:
            parts.append(f"desc={r.description}")

        self.lblRecipeMeta.setText("Info: " + (" | ".join(parts) if parts else "-"))

        draft_view = {
            "id": r.id,
            "description": r.description,
            "tool": r.tool,
            "substrate": r.substrate,
            "substrate_mount": r.substrate_mount,
            "parameters": r.parameters or {},
            "planner": r.planner or {},
            "paths_by_side": r.paths_by_side or {},
            "meta": r.meta or {},
            "draft_present": bool(isinstance(getattr(r, "draft", None), (Draft, dict))),
            "planned_traj_present": bool(getattr(r, "planned_traj", None) is not None),
            "executed_traj_present": bool(getattr(r, "executed_traj", None) is not None),
        }
        self.txtRecipeDraft.setPlainText(self._yaml_dump(draft_view))

    def _update_info_box(self) -> None:
        if self._recipe is None:
            self.infoBox.set_values({})
            return
        self.infoBox.set_values(getattr(self._recipe, "info", {}) or {})

    # ---------------- Eval UI ----------------

    def _clear_eval_views(self) -> None:
        self.txtPlannedEval.setPlainText(self._yaml_dump({}))
        self.txtExecutedEval.setPlainText(self._yaml_dump({}))
        self.lblPlannedSummary.setText("planned: –")
        self.lblExecutedSummary.setText("executed: –")

    def _set_eval_views(
        self,
        *,
        eval_dict: Optional[Dict[str, Any]] = None,
        planned_eval: Optional[Dict[str, Any]] = None,
        executed_eval: Optional[Dict[str, Any]] = None,
    ) -> None:
        if isinstance(eval_dict, dict):
            pl = eval_dict
            ex = eval_dict
        else:
            pl = planned_eval if isinstance(planned_eval, dict) else {}
            ex = executed_eval if isinstance(executed_eval, dict) else {}

        self.txtPlannedEval.setPlainText(self._yaml_dump(pl))
        self.txtExecutedEval.setPlainText(self._yaml_dump(ex))

        def _summary(ev: Dict[str, Any]) -> str:
            if not isinstance(ev, dict) or not ev:
                return "–"
            if isinstance(ev.get("total"), dict):
                score = ev["total"].get("score")
                thr = ev.get("threshold")
                valid = ev.get("valid")
                return f"score={score} | thr={thr} | valid={valid}"
            if isinstance(ev.get("result"), dict):
                score = ev["result"].get("score")
                ms = ev.get("min_score")
                dom = ev.get("domain")
                return f"domain={dom} | score={score} | min_score={ms}"
            return "–"

        self.lblPlannedSummary.setText("planned: " + _summary(pl))
        self.lblExecutedSummary.setText("executed: " + _summary(ex))

    def _load_eval_from_bundle(self) -> None:
        self._clear_eval_views()
        if not self._recipe_key:
            return

        try:
            p = self.repo.bundle.paths(self._recipe_key)
            planned_path = str(getattr(p, "planned_traj_yaml", ""))
            executed_path = str(getattr(p, "executed_traj_yaml", ""))
        except Exception:
            return

        planned_doc = self._yaml_load_file(planned_path)
        executed_doc = self._yaml_load_file(executed_path)

        pl_eval = planned_doc.get("eval") if isinstance(planned_doc, dict) else None
        ex_eval = executed_doc.get("eval") if isinstance(executed_doc, dict) else None
        ev = pl_eval if isinstance(pl_eval, dict) else (ex_eval if isinstance(ex_eval, dict) else {})
        self._set_eval_views(eval_dict=ev)

    # ---------------- Public API ----------------

    def set_recipe(self, recipe: Optional[Recipe], *, key: Optional[str] = None) -> None:
        if self._process_active:
            QMessageBox.warning(self, "Process", "Während eines laufenden Prozesses kann kein Rezept gewechselt werden.")
            return
        if self._init_thread is not None and self._init_thread.isRunning():
            QMessageBox.warning(self, "Process", "Während RobotInit läuft kann kein Rezept gewechselt werden.")
            return

        self._recipe = recipe
        self._recipe_key = str(key).strip() if key else self._recipe_key

        self._update_recipe_box()
        self._update_info_box()
        self._update_buttons()

        self._clear_eval_views()

        self._show_compiled = True
        self._show_traj = True
        self._show_executed = True
        self._spray_defaults_publish(compiled=True, planned=True, executed=True)

        if recipe is None:
            self._clear_layers()
            return

        self._load_eval_from_bundle()

    # ---------------- Load ----------------

    def _on_load_clicked(self) -> None:
        if self._process_active:
            QMessageBox.warning(self, "Load", "Während eines laufenden Prozesses nicht möglich.")
            return
        if self._init_thread is not None and self._init_thread.isRunning():
            QMessageBox.warning(self, "Load", "Während RobotInit läuft nicht möglich.")
            return

        keys = self._list_repo_keys()
        if not keys:
            QMessageBox.information(self, "Load", "Keine gespeicherten Rezepte gefunden.")
            return

        current = self._recipe_key if (self._recipe_key in keys) else keys[0]
        idx = keys.index(current) if current in keys else 0

        choice, ok = QInputDialog.getItem(
            self,
            "Recipe laden",
            "Rezept auswählen:",
            keys,
            idx,
            editable=False,
        )
        if not ok:
            return

        key = str(choice).strip()
        if not key:
            return

        try:
            r = self.repo.load_for_process(key)
            self.set_recipe(r, key=key)

            self._clear_layers()
            QTimer.singleShot(0, self._publish_all_available_layers)

            self._append_log("Load: OK")
        except Exception as e:
            QMessageBox.critical(self, "Load", f"Load fehlgeschlagen: {e}")

    # ---------------- Robot Init ----------------

    def _setup_init_thread(self) -> None:
        if self._init_thread is not None:
            return
        self._init_thread = RobotInitThread(ros=self.ros)
        self._init_thread.notifyFinished.connect(self._on_init_finished_ok)
        self._init_thread.notifyError.connect(self._on_init_finished_err)
        self._init_thread.logMessage.connect(self._append_log)
        self._init_thread.stateChanged.connect(lambda s: self.lblInit.setText(f"Init: {s}"))

    def _on_init_clicked(self) -> None:
        if self._process_active:
            QMessageBox.warning(self, "Init", "Während eines laufenden Prozesses nicht möglich.")
            return
        if self._init_thread is None:
            raise RuntimeError("RobotInitThread fehlt (strict)")
        if self._init_thread.isRunning():
            self._append_log("RobotInit: bereits aktiv (ignoriere Start).")
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
        if not self._recipe_key:
            QMessageBox.warning(self, "Process", "Kein Rezept geladen.")
            return

        if mode in (ProcessThread.MODE_VALIDATE, ProcessThread.MODE_OPTIMIZE, ProcessThread.MODE_EXECUTE):
            if not self._robot_ready:
                QMessageBox.warning(self, "Process", "Robot nicht initialisiert. Bitte zuerst Init.")
                return

        # FIX: Execute allowed in PLC sim mode even if plc is None
        if mode == ProcessThread.MODE_EXECUTE and not self._plc_execute_available():
            QMessageBox.warning(
                self,
                "Execute",
                "PLC ist nicht verfügbar (Execute benötigt PLC).\n\n"
                "Hinweis: Im PLC-Simulationsmodus (ctx.plc.sim=true) ist Execute erlaubt.",
            )
            return

        try:
            recipe_run = self.repo.load_for_process(self._recipe_key)
        except Exception as e:
            QMessageBox.critical(self, "Process", f"Rezept laden fehlgeschlagen: {e}")
            return

        # IMPORTANT for Execute baseline:
        # ExecuteSM (new) uses recipe.planned_traj as baseline planned_run.
        if mode == ProcessThread.MODE_EXECUTE:
            if getattr(recipe_run, "planned_traj", None) is None:
                QMessageBox.warning(
                    self,
                    "Execute",
                    "Execute benötigt eine geplante Baseline (planned_traj.yaml).\n\n"
                    "Bitte zuerst Validate/Optimize ausführen oder planned_traj.yaml bereitstellen.",
                )
                return

        self._process_active = True
        self._active_mode = mode
        self.txtLog.clear()
        self._clear_eval_views()

        self._append_log(f"=== {mode.upper()} gestartet ===")
        self._update_buttons()

        self._process_thread = ProcessThread(
            recipe=recipe_run,
            ros=self.ros,
            plc=self.plc,   # may be None in sim mode; ExecuteSM must tolerate that
            mode=mode,
        )
        self._process_thread.stateChanged.connect(lambda s: self._append_log(f"STATE: {s}"))
        self._process_thread.logMessage.connect(self._append_log)
        self._process_thread.notifyFinished.connect(self._on_process_finished_success)
        self._process_thread.notifyError.connect(self._on_process_finished_error)
        self._process_thread.start()

    # ---------------- Save prompt ----------------

    def _maybe_overwrite_prompt(self, *, title: str, filename: str) -> bool:
        if not filename:
            return True
        if not os.path.isfile(filename):
            return True
        yn = QMessageBox.question(
            self,
            title,
            f"Es existiert bereits ein Ergebnis ({os.path.basename(filename)}).\n\nÜberschreiben?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        return yn == QMessageBox.StandardButton.Yes

    # ---------------- Process result handling ----------------

    def _finish_process_ui(self) -> None:
        self._process_active = False
        self._active_mode = ""
        self._process_thread = None
        self._update_buttons()

    def _on_process_finished_success(self, result_obj: object) -> None:
        self._append_log("=== Process finished ===")

        # STRICT: RunResult.to_process_payload schema
        try:
            planned_yaml, executed_yaml, eval_dict, valid, invalid_reason = self._extract_from_result_strict(result_obj)
        except Exception as e:
            self._append_log(f"ERROR: result invalid (strict): {e}")
            self._finish_process_ui()
            return

        # Parse/validate JTBySegment (strict parser)
        try:
            planned_jt = JTBySegment.from_yaml_dict(planned_yaml)
        except Exception as e:
            self._append_log(f"ERROR: planned JTBySegment build failed: {e}")
            self._finish_process_ui()
            return

        try:
            executed_jt = JTBySegment.from_yaml_dict(executed_yaml)
        except Exception as e:
            self._append_log(f"ERROR: executed JTBySegment build failed: {e}")
            self._finish_process_ui()
            return

        # Show eval from RunResult (authoritative)
        self._set_eval_views(eval_dict=eval_dict or {})

        # FK TCP (best-effort, only for persistence artifacts)
        recipe_run = None
        try:
            recipe_run = self.repo.load_for_process(self._recipe_key) if self._recipe_key else None
        except Exception:
            recipe_run = None

        recipe_for_fk = (
            recipe_run
            if isinstance(recipe_run, Recipe)
            else (self._recipe if isinstance(self._recipe, Recipe) else None)
        )

        planned_tcp: Optional[Draft] = None
        executed_tcp: Optional[Draft] = None
        if recipe_for_fk is not None:
            planned_tcp = self._fk_tcp_best_effort(traj=planned_jt, recipe=recipe_for_fk)
            executed_tcp = self._fk_tcp_best_effort(traj=executed_jt, recipe=recipe_for_fk)

        # Resolve paths
        try:
            p = self.repo.bundle.paths(self._recipe_key)
            planned_path = str(getattr(p, "planned_traj_yaml"))
            executed_path = str(getattr(p, "executed_traj_yaml"))
            planned_tcp_path = str(getattr(p, "planned_tcp_yaml", ""))
            executed_tcp_path = str(getattr(p, "executed_tcp_yaml", ""))
        except Exception as e:
            self._append_log(f"ERROR: bundle.paths() failed: {e}")
            planned_path = ""
            executed_path = ""
            planned_tcp_path = ""
            executed_tcp_path = ""

        # FIX (critical): Save happens only if valid == True
        if not valid:
            reason = invalid_reason or "invalid"
            self._append_log(f"Save: übersprungen (RunResult invalid: {reason}).")
        else:
            ok_pl = self._maybe_overwrite_prompt(title="Run speichern", filename=planned_path)
            ok_ex = self._maybe_overwrite_prompt(title="Run speichern", filename=executed_path)

            ok_tcp = True
            if planned_tcp_path:
                ok_tcp = ok_tcp and self._maybe_overwrite_prompt(title="Run speichern", filename=planned_tcp_path)
            if executed_tcp_path:
                ok_tcp = ok_tcp and self._maybe_overwrite_prompt(title="Run speichern", filename=executed_tcp_path)

            if ok_pl and ok_ex and ok_tcp:
                try:
                    planned_doc = planned_jt.to_yaml_dict()
                    executed_doc = executed_jt.to_yaml_dict()

                    # persist eval into traj yaml (optional)
                    if isinstance(eval_dict, dict) and eval_dict:
                        planned_doc = dict(planned_doc or {})
                        executed_doc = dict(executed_doc or {})
                        planned_doc["eval"] = dict(eval_dict)
                        executed_doc["eval"] = dict(eval_dict)

                    self._yaml_write_file(planned_path, planned_doc)
                    self._yaml_write_file(executed_path, executed_doc)

                    if planned_tcp_path and planned_tcp is not None:
                        self._yaml_write_file(planned_tcp_path, planned_tcp.to_yaml_dict())
                    if executed_tcp_path and executed_tcp is not None:
                        self._yaml_write_file(executed_tcp_path, executed_tcp.to_yaml_dict())

                    self._append_log("Save: OK -> planned/executed traj (+eval) (+tcp best-effort)")
                except Exception as e:
                    self._append_log(f"Save: ERROR: {e}")
            else:
                self._append_log("Save: übersprungen (nicht überschrieben).")

        # Reload + republish
        try:
            if self._recipe_key:
                self._recipe = self.repo.load_for_process(self._recipe_key)
        except Exception:
            pass

        self._clear_layers()
        self._publish_all_available_layers()

        self._update_recipe_box()
        self._update_info_box()
        self._load_eval_from_bundle()

        self._finish_process_ui()

    def _on_process_finished_error(self, msg: str) -> None:
        self._append_log(f"=== Process ERROR: {msg} ===")
        self._finish_process_ui()

    # ---------------- SprayPath publish ----------------

    def _ros_call_first(self, candidates: Tuple[str, ...], *args, **kwargs) -> None:
        for name in candidates:
            fn = getattr(self.ros, name, None)
            if callable(fn):
                fn(*args, **kwargs)
                return

    def _clear_layers(self) -> None:
        try:
            from visualization_msgs.msg import MarkerArray as _MA
            from geometry_msgs.msg import PoseArray as _PA

            empty_ma = _MA()
            empty_pa = _PA()
            self._ros_call_first(("spray_set_compiled",), poses=empty_pa, markers=empty_ma)
            self._ros_call_first(("spray_set_planned", "spray_set_traj"), poses=empty_pa, markers=empty_ma)
            self._ros_call_first(("spray_set_executed",), poses=empty_pa, markers=empty_ma)
        except Exception:
            pass

    def _build_all_markers(self, *, recipe: Recipe, frame_id: str = "scene") -> Optional[MarkerArray]:
        try:
            return build_marker_array_from_recipe(
                recipe,
                frame_id=frame_id,
                show_draft=True,
                show_planned=True,
                show_executed=True,
            )
        except Exception as e:
            self._append_log(f"W markers build skipped: {e}")
            return None

    @staticmethod
    def _filter_markers(
        arr: MarkerArray, *, ns_prefix: Optional[str] = None, ns_exact: Optional[str] = None
    ) -> MarkerArray:
        out = MarkerArray()
        if arr is None or not getattr(arr, "markers", None):
            return out

        for m in arr.markers:
            ns = getattr(m, "ns", "") or ""
            if ns_exact is not None:
                if ns == ns_exact:
                    out.markers.append(m)
            elif ns_prefix is not None:
                if ns.startswith(ns_prefix):
                    out.markers.append(m)

        return out

    def _publish_all_available_layers(self) -> None:
        if self._recipe is None:
            return

        r = self._recipe
        all_ma = self._build_all_markers(recipe=r, frame_id="scene")
        if all_ma is None:
            try:
                self.sprayPathBox.publish_current()
            except Exception:
                pass
            return

        ma_compiled = self._filter_markers(all_ma, ns_prefix="draft/")
        ma_planned = self._filter_markers(all_ma, ns_exact="planned_traj")
        ma_exec = self._filter_markers(all_ma, ns_exact="executed_traj")

        try:
            if ma_compiled.markers:
                self._ros_call_first(("spray_set_compiled",), markers=ma_compiled)
        except Exception as e:
            self._append_log(f"W spray_set_compiled failed: {e}")

        try:
            if ma_planned.markers:
                self._ros_call_first(("spray_set_planned", "spray_set_traj"), markers=ma_planned)
        except Exception as e:
            self._append_log(f"W spray_set_planned/traj failed: {e}")

        try:
            if ma_exec.markers:
                self._ros_call_first(("spray_set_executed",), markers=ma_exec)
        except Exception as e:
            self._append_log(f"W spray_set_executed failed: {e}")

        try:
            self.sprayPathBox.publish_current()
        except Exception:
            pass

    # ---------------- UI gating ----------------

    def _update_buttons(self) -> None:
        ros_ok = True
        has_recipe = bool(self._recipe_key)

        self.btnLoad.setEnabled((not self._process_active) and ros_ok)
        self.btnInit.setEnabled((not self._process_active) and ros_ok and (self._init_thread is not None))

        self.btnValidate.setEnabled((not self._process_active) and ros_ok and has_recipe and self._robot_ready)
        self.btnOptimize.setEnabled((not self._process_active) and ros_ok and has_recipe and self._robot_ready)

        # FIX: Execute also allowed if PLC is simulated (ctx.plc.sim=true)
        self.btnExecute.setEnabled(
            (not self._process_active) and ros_ok and has_recipe and self._robot_ready and self._plc_execute_available()
        )

        self.btnStop.setEnabled(True)
