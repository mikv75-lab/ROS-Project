# -*- coding: utf-8 -*-
# File: tabs/process/process_tab.py
from __future__ import annotations

import logging
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
    QMessageBox,
    QInputDialog,
    QSizePolicy,
)

from visualization_msgs.msg import MarkerArray

from plc.plc_client import PlcClientBase
from ros.bridge.ros_bridge import RosBridge

from model.recipe.recipe import Recipe, Draft, JTBySegment
from model.recipe.recipe_markers import build_marker_array_from_recipe

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
    ProcessTab (strict, no fallbacks)

    Worker result contract (strict):
      result_obj is dict with:
        {
          "planned_run":  RunPayload v1  (segments -> LIST of JT-event dicts),
          "executed_run": RunPayload v1  (segments -> LIST of JT-event dicts),
        }

    Persistence contract (SSoT, Option A):
      repo.save_run_artifacts(recipe_id, planned_traj=JTBySegment, executed_traj=JTBySegment, ...)
      -> Bundle writes:
          planned_traj.yaml
          executed_traj.yaml
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

        # ---- strict repo API (Option A) ----
        for name in ("list_recipes", "load_for_process", "save_run_artifacts"):
            fn = getattr(self.repo, name, None)
            if not callable(fn):
                raise RuntimeError(f"ProcessTab: repo missing required API: {name}()")

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

        # ---- last run payloads (raw run payloads for UI) ----
        self._last_run_payload_planned: Dict[str, Any] = {}
        self._last_run_payload_executed: Dict[str, Any] = {}

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

        # ---------------- InfoGroupBox (links) ----------------
        self.infoBox = InfoGroupBox(self)
        row_info.addWidget(self.infoBox, 2)

        # ---------------- SprayPath GroupBox (rechts) ----------------
        self.sprayPathBox = SprayPathBox(ros=self.ros, parent=self, title="Spray Paths")
        row_info.addWidget(self.sprayPathBox, 1)
        root.addLayout(row_info)

        # ---------------- Results ----------------
        grp_res = QGroupBox("Run Data", self)
        vres = QVBoxLayout(grp_res)
        row_res = QHBoxLayout()

        self.txtPlanned = QTextEdit(grp_res)
        self.txtPlanned.setReadOnly(True)
        self.txtPlanned.setPlaceholderText("planned_run payload (raw)")

        self.txtExecuted = QTextEdit(grp_res)
        self.txtExecuted.setReadOnly(True)
        self.txtExecuted.setPlaceholderText("executed_run payload (raw)")

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

        # SprayPathBox -> ProcessTab cache
        self.sprayPathBox.showCompiledToggled.connect(self._on_show_compiled_toggled)
        self.sprayPathBox.showTrajToggled.connect(self._on_show_traj_toggled)
        self.sprayPathBox.showExecutedToggled.connect(self._on_show_executed_toggled)

        # init thread (persistent)
        self._setup_init_thread()

        # RobotStatus wiring
        self._wire_robot_status_inbound()

        # Ensure defaults are published once
        self.sprayPathBox.set_defaults(compiled=True, traj=True, executed=True)

        self._update_buttons()
        self._update_recipe_box()
        self._update_info_box()
        self._render_run_yaml_views()

    # ---------------- Utils ----------------

    def _append_log(self, msg: str) -> None:
        if msg:
            self.txtLog.append(msg)

    def _yaml_dump(self, obj: Any) -> str:
        try:
            return yaml.safe_dump(obj or {}, allow_unicode=True, sort_keys=False)
        except Exception:
            return str(obj)

    def _list_repo_keys(self) -> List[str]:
        keys = self.repo.list_recipes() or []
        keys = [str(k) for k in keys if isinstance(k, str) and str(k).strip()]
        keys.sort()
        return keys

    # ---------------- Draft/Compiled helpers ----------------

    def _draft_sides_dict(self) -> Dict[str, Any]:
        r = self._recipe
        if r is None:
            return {}
        d = getattr(r, "draft", None)

        if isinstance(d, Draft):
            yd = d.to_yaml_dict() or {}
            sides = yd.get("sides") or {}
            return sides if isinstance(sides, dict) else {}

        if isinstance(d, dict):
            sides = (d.get("sides") or {})
            return sides if isinstance(sides, dict) else {}

        return {}

    def _has_compiled_in_draft(self) -> bool:
        sides = self._draft_sides_dict()
        if not sides:
            return False
        for _side, sd in sides.items():
            if isinstance(sd, dict):
                pq = sd.get("poses_quat") or []
                if isinstance(pq, list) and len(pq) > 0:
                    return True
        return False

    # ---------------- Run payload helpers ----------------

    def _is_run_payload(self, payload: Any) -> bool:
        return isinstance(payload, dict) and isinstance(payload.get("segments"), dict)

    def _extract_planned_and_executed_from_result(self, result_obj: object) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        planned: Dict[str, Any] = {}
        executed: Dict[str, Any] = {}

        if isinstance(result_obj, dict):
            a = result_obj.get("planned_run")
            b = result_obj.get("executed_run")
            planned = a if isinstance(a, dict) else {}
            executed = b if isinstance(b, dict) else {}
            return planned, executed

        return planned, executed

    def _require_both_runs(self, planned: Dict[str, Any], executed: Dict[str, Any], *, context: str) -> None:
        if not self._is_run_payload(planned) or not self._is_run_payload(executed):
            raise RuntimeError(
                f"{context}: planned/executed payload fehlt oder invalid "
                f"(planned_ok={self._is_run_payload(planned)}, executed_ok={self._is_run_payload(executed)})"
            )

    # ------------------------------------------------------------
    # JTBySegment conversion (RunPayload v1 -> JTBySegment YAML dict)
    # ------------------------------------------------------------

    @staticmethod
    def _dur_to_ns(d: Any) -> int:
        if d is None:
            return 0
        if isinstance(d, dict):
            try:
                return int(d.get("sec", 0)) * 1_000_000_000 + int(d.get("nanosec", 0))
            except Exception:
                return 0
        if isinstance(d, (int, float)):
            return int(float(d) * 1_000_000_000)
        return 0

    @staticmethod
    def _ns_to_dur(ns: int) -> Dict[str, int]:
        ns = int(max(ns, 0))
        return {"sec": ns // 1_000_000_000, "nanosec": ns % 1_000_000_000}

    def _concat_joint_traj_dicts(self, dicts: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        """
        Merge multiple JT dicts (same joint_names) into one JT dict with monotonic time_from_start.
        """
        items = [d for d in (dicts or []) if isinstance(d, dict) and d.get("joint_names") and d.get("points")]
        if not items:
            return None

        base_names = list(items[0].get("joint_names") or [])
        if not base_names:
            return None

        merged_pts: List[Dict[str, Any]] = []
        last_global_ns = 0
        t_offset_ns = 0

        for jt in items:
            names = list(jt.get("joint_names") or [])
            pts = list(jt.get("points") or [])
            if not pts:
                continue
            if names != base_names:
                continue

            local_times = [self._dur_to_ns(p.get("time_from_start")) if isinstance(p, dict) else 0 for p in pts]
            for i, p in enumerate(pts):
                if not isinstance(p, dict):
                    continue
                t_local_ns = local_times[i] if i < len(local_times) else 0
                t_global_ns = t_offset_ns + t_local_ns

                if merged_pts and t_global_ns <= last_global_ns:
                    t_global_ns = last_global_ns + 1_000_000  # +1ms

                q = dict(p)
                q["time_from_start"] = self._ns_to_dur(t_global_ns)

                if merged_pts and i == 0:
                    try:
                        if q.get("positions") == merged_pts[-1].get("positions"):
                            last_global_ns = t_global_ns
                            continue
                    except Exception:
                        pass

                merged_pts.append(q)
                last_global_ns = t_global_ns

            t_offset_ns = last_global_ns

        if not merged_pts:
            return None

        return {"joint_names": base_names, "points": merged_pts}

    def _runpayload_to_jtbysegment_yaml(self, run_payload: Dict[str, Any]) -> Dict[str, Any]:
        """
        RunPayload v1:
          segments[SEG] = [jt_event_dict, jt_event_dict, ...]

        JTBySegment YAML expects:
          segments[SEG] = jt_dict (single)
        """
        if not self._is_run_payload(run_payload):
            return {}

        seg_out: Dict[str, Any] = {}
        segs = run_payload.get("segments") or {}

        for seg in self._SEG_ORDER:
            evs = segs.get(seg) or []
            if not isinstance(evs, list) or not evs:
                continue
            evs2 = [e for e in evs if isinstance(e, dict) and isinstance(e.get("joint_names"), list) and isinstance(e.get("points"), list)]
            if not evs2:
                continue
            merged = self._concat_joint_traj_dicts(evs2)
            if merged is not None:
                seg_out[seg] = merged

        return {
            "version": 1,
            "meta": dict(run_payload.get("meta") or {}),
            "segments": seg_out,
        }

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
        }
        self.txtRecipeDraft.setPlainText(self._yaml_dump(draft_view))

    def _update_info_box(self) -> None:
        if self._recipe is None:
            self.infoBox.set_values({})
            return
        self.infoBox.set_values(getattr(self._recipe, "info", {}) or {})

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

        self._last_run_payload_planned = {}
        self._last_run_payload_executed = {}
        self._render_run_yaml_views()

        # defaults always true + publish to ROS (latched)
        self._show_compiled = True
        self._show_traj = True
        self._show_executed = True
        self.sprayPathBox.set_defaults(compiled=True, traj=True, executed=True)

        if recipe is None:
            self._clear_layers()

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

            # Clear and publish current truth (compiled only; traj markers require TCP trajectories)
            self._clear_layers()
            self._publish_all_available_layers()

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

        if mode == ProcessThread.MODE_EXECUTE and self.plc is None:
            QMessageBox.warning(self, "Execute", "PLC ist nicht verfügbar (Execute benötigt PLC).")
            return

        try:
            recipe_run = self.repo.load_for_process(self._recipe_key)
        except Exception as e:
            QMessageBox.critical(self, "Process", f"Rezept laden fehlgeschlagen: {e}")
            return

        self._process_active = True
        self._active_mode = mode
        self.txtLog.clear()

        self._last_run_payload_planned = {}
        self._last_run_payload_executed = {}
        self._render_run_yaml_views()

        self._append_log(f"=== {mode.upper()} gestartet ===")
        self._update_buttons()

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

    def _render_run_yaml_views(self) -> None:
        self.txtPlanned.setPlainText(self._yaml_dump(self._last_run_payload_planned))
        self.txtExecuted.setPlainText(self._yaml_dump(self._last_run_payload_executed))

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

    def _on_process_finished_success(self, result_obj: object) -> None:
        self._append_log("=== Process finished ===")

        planned_run, executed_run = self._extract_planned_and_executed_from_result(result_obj)
        try:
            self._require_both_runs(planned_run, executed_run, context=self._active_mode or "process")
        except Exception as e:
            self._append_log(f"ERROR: {e}")
            self._process_active = False
            self._active_mode = ""
            self._process_thread = None
            self._update_buttons()
            return

        # keep raw run payloads visible
        self._last_run_payload_planned = dict(planned_run)
        self._last_run_payload_executed = dict(executed_run)
        self._render_run_yaml_views()

        # Convert to JTBySegment (schema expected by RecipeBundle persistence)
        planned_jt_yaml = self._runpayload_to_jtbysegment_yaml(planned_run)
        executed_jt_yaml = self._runpayload_to_jtbysegment_yaml(executed_run)

        try:
            planned_jt = JTBySegment.from_yaml_dict(planned_jt_yaml)
        except Exception as e:
            self._append_log(f"ERROR: planned JTBySegment build failed: {e}")
            planned_jt = None

        try:
            executed_jt = JTBySegment.from_yaml_dict(executed_jt_yaml)
        except Exception as e:
            self._append_log(f"ERROR: executed JTBySegment build failed: {e}")
            executed_jt = None

        if planned_jt is None or executed_jt is None:
            # strict: both required for storage
            self._append_log("ERROR: Persist skipped (planned/executed JTBySegment missing).")
        else:
            # Determine target file paths for overwrite prompt (strict, no repo fallbacks)
            p = self.repo.bundle.paths(self._recipe_key)  # rid-only key
            planned_path = str(getattr(p, "planned_traj_yaml"))
            executed_path = str(getattr(p, "executed_traj_yaml"))

            ok_pl = self._maybe_overwrite_prompt(title="Run speichern", filename=planned_path)
            ok_ex = self._maybe_overwrite_prompt(title="Run speichern", filename=executed_path)

            if ok_pl and ok_ex:
                try:
                    self.repo.save_run_artifacts(
                        self._recipe_key,
                        planned_traj=planned_jt,
                        executed_traj=executed_jt,
                    )
                    self._append_log("Save: OK -> planned_traj.yaml + executed_traj.yaml")
                except Exception as e:
                    self._append_log(f"Save: ERROR: {e}")
            else:
                self._append_log("Save: übersprungen (nicht überschrieben).")

        # Reload recipe fresh after persistence
        try:
            if self._recipe_key:
                self._recipe = self.repo.load_for_process(self._recipe_key)
        except Exception:
            pass

        # republish markers (compiled only unless tcp trajectories exist elsewhere)
        self._clear_layers()
        self._publish_all_available_layers()

        self._update_recipe_box()
        self._update_info_box()

        self._process_active = False
        self._active_mode = ""
        self._process_thread = None
        self._update_buttons()

    def _on_process_finished_error(self, msg: str) -> None:
        self._append_log(f"=== Process ERROR: {msg} ===")
        self._process_active = False
        self._active_mode = ""
        self._process_thread = None
        self._update_buttons()

    # ---------------- SprayPath publish ----------------

    def _ros_call_first(self, candidates: Tuple[str, ...], *args, **kwargs) -> None:
        for name in candidates:
            fn = getattr(self.ros, name, None)
            if callable(fn):
                fn(*args, **kwargs)
                return

    def _clear_layers(self) -> None:
        try:
            empty = MarkerArray()
            self._ros_call_first(("spray_set_compiled",), markers=empty)
            self._ros_call_first(("spray_set_planned", "spray_set_traj"), markers=empty)
            self._ros_call_first(("spray_set_executed",), markers=empty)
        except Exception:
            pass

    def _build_markers_compat(self, *, recipe: Recipe, source: str, frame_id: str = "scene") -> Optional[MarkerArray]:
        fn = build_marker_array_from_recipe
        try:
            import inspect

            sig = inspect.signature(fn)
            params = sig.parameters

            kw: Dict[str, Any] = {}

            if "source" in params:
                kw["source"] = source
            elif "kind" in params:
                kw["kind"] = source
            elif "layer" in params:
                kw["layer"] = source
            elif "which" in params:
                kw["which"] = source

            if "frame_id" in params:
                kw["frame_id"] = frame_id
            elif "frame" in params:
                kw["frame"] = frame_id

            if kw:
                part = fn(recipe, **kw)
            else:
                part = fn(recipe, source)

            if part is None or getattr(part, "markers", None) is None:
                return None
            return part
        except Exception as e:
            self._append_log(f"W markers ({source}) skipped: {e}")
            return None

    def _publish_all_available_layers(self) -> None:
        """
        Marker builder consumes Recipe + layer id in {"compiled_path","traj","executed_traj"}.

        Here:
          - compiled: available if Draft has poses_quat
          - traj/executed: only if recipe.trajectories contains tcp totals (not generated here yet)
        """
        if self._recipe is None:
            return

        r = self._recipe
        has_compiled = bool(self._has_compiled_in_draft())

        trajectories = getattr(r, "trajectories", {}) or {}
        has_traj_tcp = bool(isinstance(trajectories.get("planned"), dict) and trajectories.get("planned"))
        has_exec_tcp = bool(isinstance(trajectories.get("executed"), dict) and trajectories.get("executed"))

        ma_compiled = self._build_markers_compat(recipe=r, source="compiled_path", frame_id="scene") if has_compiled else None
        ma_traj = self._build_markers_compat(recipe=r, source="traj", frame_id="scene") if has_traj_tcp else None
        ma_exec = self._build_markers_compat(recipe=r, source="executed_traj", frame_id="scene") if has_exec_tcp else None

        try:
            if ma_compiled is not None:
                self._ros_call_first(("spray_set_compiled",), markers=ma_compiled)
        except Exception as e:
            self._append_log(f"W spray_set_compiled failed: {e}")

        try:
            if ma_traj is not None:
                self._ros_call_first(("spray_set_planned", "spray_set_traj"), markers=ma_traj)
        except Exception as e:
            self._append_log(f"W spray_set_planned/traj failed: {e}")

        try:
            if ma_exec is not None:
                self._ros_call_first(("spray_set_executed",), markers=ma_exec)
        except Exception as e:
            self._append_log(f"W spray_set_executed failed: {e}")

        # Always republish current toggles (idempotent, latched)
        self.sprayPathBox.publish_current()

    # ---------------- UI gating ----------------

    def _update_buttons(self) -> None:
        ros_ok = True  # strict: ros always present
        has_recipe = bool(self._recipe_key)

        self.btnLoad.setEnabled((not self._process_active) and ros_ok)
        self.btnInit.setEnabled((not self._process_active) and ros_ok and (self._init_thread is not None))

        self.btnValidate.setEnabled((not self._process_active) and ros_ok and has_recipe and self._robot_ready)
        self.btnOptimize.setEnabled((not self._process_active) and ros_ok and has_recipe and self._robot_ready)
        self.btnExecute.setEnabled(
            (not self._process_active) and ros_ok and has_recipe and self._robot_ready and (self.plc is not None)
        )

        self.btnStop.setEnabled(True)
