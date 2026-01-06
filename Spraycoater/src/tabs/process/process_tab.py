# -*- coding: utf-8 -*-
# File: tabs/process/process_tab.py
from __future__ import annotations

import logging
import os
from typing import Optional, Any, Dict, List

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
    QCheckBox,
)

from visualization_msgs.msg import MarkerArray

from plc.plc_client import PlcClientBase
from ros.bridge.ros_bridge import RosBridge

from model.recipe.recipe import Recipe
from model.recipe.recipe_markers import build_marker_array_from_recipe

from widgets.robot_status_box import RobotStatusInfoBox
from widgets.info_groupbox import InfoGroupBox
from .spray_path_box import SprayPathBox

from .process_thread import ProcessThread
from .robot_init_thread import RobotInitThread

from .base_statemachine import (
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
)

_LOG = logging.getLogger("tabs.process")


class _SprayPathBoxDisabled(QGroupBox):
    """
    Fallback-Widget, wenn ROS/SprayPathBridge nicht verfügbar ist.

    Contract:
      - bietet dieselben Qt-Signale wie SprayPathBox, damit ProcessTab nicht crasht
      - Checkboxen sind deaktiviert (UI-only), publish_current() ist no-op
      - set_defaults/reset_defaults existieren (no-op UI state), damit Call-Sites stabil bleiben
    """

    showCompiledToggled = QtCore.pyqtSignal(bool)
    showTrajToggled = QtCore.pyqtSignal(bool)
    showExecutedToggled = QtCore.pyqtSignal(bool)

    # optional/legacy aliases (falls andere Stellen diese erwarten)
    showCompiledRequested = QtCore.pyqtSignal(bool)
    showTrajRequested = QtCore.pyqtSignal(bool)
    showExecutedRequested = QtCore.pyqtSignal(bool)

    def __init__(self, *, parent: Optional[QWidget] = None, title: str = "Spray Paths", reason: str = "") -> None:
        super().__init__(title, parent)

        self._block = False
        self._reason = (reason or "").strip()

        g = QVBoxLayout(self)
        g.setContentsMargins(8, 8, 8, 8)
        g.setSpacing(6)

        msg = "ROS/SprayPathBridge nicht verfügbar."
        if self._reason:
            msg += f" ({self._reason})"
        lbl = QLabel(msg, self)
        lbl.setWordWrap(True)
        g.addWidget(lbl)

        row = QHBoxLayout()
        row.setSpacing(12)

        def _mk_group(name: str) -> tuple[QGroupBox, QCheckBox]:
            box = QGroupBox(name, self)
            vb = QVBoxLayout(box)
            vb.setContentsMargins(8, 8, 8, 8)
            vb.setSpacing(6)

            cb = QCheckBox("Show", box)
            cb.setChecked(True)
            cb.setEnabled(False)
            vb.addWidget(cb)

            info = QLabel("Available: –", box)
            info.setWordWrap(True)
            vb.addWidget(info)

            return box, cb

        b1, self.chkCompiled = _mk_group("Compiled")
        b2, self.chkTraj = _mk_group("Traj")
        b3, self.chkExecuted = _mk_group("Executed")

        row.addWidget(b1)
        row.addWidget(b2)
        row.addWidget(b3)

        g.addLayout(row)

    def set_defaults(self, *, compiled: bool = True, traj: bool = True, executed: bool = True) -> None:
        self._block = True
        try:
            self.chkCompiled.setChecked(bool(compiled))
            self.chkTraj.setChecked(bool(traj))
            self.chkExecuted.setChecked(bool(executed))
        finally:
            self._block = False

    def reset_defaults(self) -> None:
        self.set_defaults(compiled=True, traj=True, executed=True)

    def publish_current(self) -> None:
        # no ROS, intentionally no-op
        return


class ProcessTab(QWidget):
    """
    ProcessTab

    Layout:
      - Top Row: Recipe (links) | Process (mitte) | Robot Status (rechts)
      - Row: InfoGroupBox (links) | Spray Paths (rechts)
      - Run Data (scored)
      - Log

    NEU (Contract):
      - Worker liefert SegmentRunPayload (dict):
          {
            "version": 1,
            "meta": {...},
            "segments": {
              "<STATE>": { "tcp": <traj-segment>, "meta": {...} },
              ...
            }
          }

      - Persistenz speichert genau dieses Payload (segments-only).
        (traj.yaml vs executed_traj.yaml: gleicher Inhalt, nur anderer Name)

      - Für Visualisierung/Eval kann daraus ein "total" Traj on-the-fly gebaut werden
        (Concatenate der Segment-TCP poses in State-Reihenfolge).
    """

    _SEG_ORDER = (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME)

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
        self.repo = repo
        self.ros = ros
        self.plc = plc

        if self.repo is None:
            raise RuntimeError("ProcessTab: repo ist None (ctx.repo fehlt?)")

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

        # ---- last run payloads (segments-only) for UI/save ----
        self._last_run_payload_traj: Dict[str, Any] = {}
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
        # IMPORTANT: darf NICHT crashen, wenn ros=None oder ros.spray fehlt.
        try:
            self.sprayPathBox = SprayPathBox(ros=self.ros, parent=self, title="Spray Paths")
        except Exception as e:
            self.sprayPathBox = _SprayPathBoxDisabled(parent=self, title="Spray Paths", reason=str(e))

        row_info.addWidget(self.sprayPathBox, 1)
        root.addLayout(row_info)

        # ---------------- Results ----------------
        grp_res = QGroupBox("Run Data (scored)", self)
        vres = QVBoxLayout(grp_res)
        row_res = QHBoxLayout()

        self.txtPlanned = QTextEdit(grp_res)
        self.txtPlanned.setReadOnly(True)
        self.txtPlanned.setPlaceholderText("traj.yaml payload (segments-only) – inkl. eval/score")

        self.txtExecuted = QTextEdit(grp_res)
        self.txtExecuted.setReadOnly(True)
        self.txtExecuted.setPlaceholderText("executed_traj.yaml payload (segments-only) – inkl. eval/score")

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
        # (Fallback Widget bietet dieselben Signale)
        self.sprayPathBox.showCompiledToggled.connect(self._on_show_compiled_toggled)
        self.sprayPathBox.showTrajToggled.connect(self._on_show_traj_toggled)
        self.sprayPathBox.showExecutedToggled.connect(self._on_show_executed_toggled)

        # init thread (persistent)
        self._setup_init_thread()

        # RobotStatus wiring (wie ServiceRobotTab)
        self._wire_robot_status_inbound()

        # Ensure defaults are published once (SprayPathBox does it already, but keep idempotent)
        try:
            self.sprayPathBox.set_defaults(compiled=True, traj=True, executed=True)
        except Exception:
            pass

        self._update_buttons()
        self._update_recipe_box()
        self._update_info_box()

        # initial run yaml views (empty)
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
        try:
            keys = self.repo.list_recipes() or []
            keys = [str(k) for k in keys if isinstance(k, str) and str(k).strip()]
            keys.sort()
            return keys
        except Exception:
            return []

    def _compiled_sides(self) -> list[str]:
        try:
            pc = getattr(self._recipe, "paths_compiled", {}) or {}
            sides = pc.get("sides") or {}
            if isinstance(sides, dict) and sides:
                return [str(k) for k in sides.keys()]
        except Exception:
            pass
        try:
            side = str((getattr(self._recipe, "parameters", {}) or {}).get("active_side", "top"))
            return [side]
        except Exception:
            return ["top"]

    def _trigger_ros_on_loaded_recipe(self) -> None:
        if self.ros is None or self._recipe is None:
            return
        for fn_name in (
            "on_recipe_loaded",
            "notify_recipe_loaded",
            "publish_recipe_info",
            "publish_loaded_recipe",
            "set_active_recipe",
        ):
            fn = getattr(self.ros, fn_name, None)
            if callable(fn):
                try:
                    fn(self._recipe)  # type: ignore[misc]
                except Exception:
                    pass

    # ---------------- Segment payload helpers ----------------

    def _is_run_payload(self, payload: Any) -> bool:
        return isinstance(payload, dict) and isinstance(payload.get("segments"), dict)

    def _segment_tcp_traj(self, payload: Dict[str, Any], seg: str) -> Dict[str, Any]:
        try:
            segs = payload.get("segments") or {}
            s = segs.get(seg) or {}
            tcp = s.get("tcp")
            return tcp if isinstance(tcp, dict) else {}
        except Exception:
            return {}

    def _concat_total_traj_from_run(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """
        Build a 'total' trajectory dict compatible with Recipe.trajectory_points_mm_for_side()
        by concatenating segment TCP poses in _SEG_ORDER.

        Output:
          {"frame":..., "tool_frame":..., "sides":{side:{"poses_quat":[...]}}}
        """
        if not self._is_run_payload(payload):
            return {}

        meta = payload.get("meta") or {}
        frame = str(meta.get("frame") or "scene")
        tool_frame = str(meta.get("tool_frame") or "tool_mount")

        # decide side from recipe parameters (or fallback)
        side = "top"
        try:
            if self._recipe is not None:
                side = str((getattr(self._recipe, "parameters", {}) or {}).get("active_side", "top"))
        except Exception:
            side = "top"

        poses: List[Dict[str, float]] = []

        for seg in self._SEG_ORDER:
            tcp = self._segment_tcp_traj(payload, seg)
            if not isinstance(tcp, dict) or not tcp:
                continue
            try:
                sdict = tcp.get("sides") or {}
                s0 = sdict.get(side) or {}
                pq = s0.get("poses_quat") or []
                if isinstance(pq, list):
                    for item in pq:
                        if isinstance(item, dict) and ("x" in item and "y" in item and "z" in item):
                            poses.append(item)
            except Exception:
                continue

        if not poses:
            return {}

        return {
            "frame": frame,
            "tool_frame": tool_frame,
            "sides": {side: {"poses_quat": poses}},
            "meta": {"source": "concat_segments", "n_points": int(len(poses))},
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
        if self.ros is None:
            self.robotStatusBox.set_connection("disabled")
            return
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

        except Exception:
            _LOG.exception("ProcessTab: robot status wiring failed")
            self.robotStatusBox.set_connection("connected?")

    # ---------------- Run persistence helpers ----------------

    def _run_file_path(self, kind: str) -> str:
        key = self._recipe_key or ""
        if not key:
            return ""
        bundle = getattr(self.repo, "bundle", None)
        if bundle is None:
            return ""
        try:
            p = bundle.paths(key)
            if kind == "traj":
                return str(getattr(p, "traj_yaml", "") or "")
            if kind == "executed":
                return str(getattr(p, "executed_yaml", "") or "")
        except Exception:
            return ""
        return ""

    def _maybe_save_run(self, *, kind: str, data: dict) -> None:
        """
        Persist segments-only payload. Contents for traj/executed are identical;
        only the filename differs (repo policy).
        """
        if not self._recipe_key:
            return
        if not isinstance(data, dict) or not data:
            return

        path = self._run_file_path(kind)
        exists = bool(path and os.path.isfile(path))

        if exists:
            yn = QMessageBox.question(
                self,
                "Run speichern",
                f"Es existiert bereits ein Ergebnis ({os.path.basename(path)}).\n\nÜberschreiben?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if yn != QMessageBox.StandardButton.Yes:
                self._append_log(f"Save: übersprungen (nicht überschrieben): {os.path.basename(path) or kind}")
                return

        try:
            if kind == "traj":
                # same payload, different API name
                self.repo.save_traj_run(self._recipe_key, traj=data)
                self._append_log(f"Save: OK -> {os.path.basename(path) or 'traj.yaml'}")
            elif kind == "executed":
                # same payload, different API name
                self.repo.save_executed_run(self._recipe_key, executed=data)
                self._append_log(f"Save: OK -> {os.path.basename(path) or 'executed_traj.yaml'}")
        except Exception as e:
            self._append_log(f"Save: ERROR ({kind}): {e}")

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

        parts = []
        parts.append(f"id={r.id}")
        if r.tool:
            parts.append(f"tool={r.tool}")
        if r.substrate:
            parts.append(f"substrate={r.substrate}")
        if r.substrate_mount:
            parts.append(f"mount={r.substrate_mount}")
        if r.description:
            parts.append(f"desc={r.description}")

        try:
            info = getattr(r, "info", {}) or {}
            if isinstance(info, dict) and info:
                tp = info.get("total_points")
                tl = info.get("total_length_mm")
                if tp is not None or tl is not None:
                    parts.append(f"compiled: points={tp} len_mm={tl}")
        except Exception:
            pass

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
        }
        self.txtRecipeDraft.setPlainText(self._yaml_dump(draft_view))

    def _update_info_box(self) -> None:
        if self._recipe is None:
            self.infoBox.set_values({})
            return
        try:
            self.infoBox.set_values(getattr(self._recipe, "info", {}) or {})
        except Exception:
            self.infoBox.set_values({})

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

        self._last_run_payload_traj = {}
        self._last_run_payload_executed = {}
        self._render_run_yaml_views()

        # defaults always true + publish to ROS (latched)
        self._show_compiled = True
        self._show_traj = True
        self._show_executed = True
        try:
            self.sprayPathBox.set_defaults(compiled=True, traj=True, executed=True)
        except Exception:
            pass

        if recipe:
            self._publish_all_available_layers()
            self._trigger_ros_on_loaded_recipe()
        else:
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
            self._append_log("Load: OK")
        except Exception as e:
            QMessageBox.critical(self, "Load", f"Load fehlgeschlagen: {e}")

    # ---------------- Robot Init ----------------

    def _setup_init_thread(self) -> None:
        if self._init_thread is not None:
            return
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
            self._init_thread = None

    def _on_init_clicked(self) -> None:
        if self._init_thread is None:
            QMessageBox.warning(self, "Init", "ROS nicht verfügbar.")
            return
        if self._process_active:
            QMessageBox.warning(self, "Init", "Während eines laufenden Prozesses nicht möglich.")
            return
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
        if self.ros is None:
            QMessageBox.warning(self, "Process", "ROS nicht verfügbar.")
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
        self._last_run_payload_traj = {}
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

    def _apply_run_payload_to_recipe_and_eval(self, run_payload: Dict[str, Any], *, traj_slot: str) -> None:
        """
        - Speichert segments-only payload für Save/UI.
        - Baut zusätzlich total_traj (concat segments) für Visualization/Eval-Kompatibilität.
        - Eval:
            1) bevorzugt: Recipe.evaluate_run_against_compiled(...) falls vorhanden
            2) fallback: Recipe.evaluate_trajectory_against_compiled(...) auf total_traj
        """
        if self._recipe is None:
            return
        if not self._is_run_payload(run_payload):
            return

        # 1) stash payload for UI/save
        if traj_slot == "traj":
            self._last_run_payload_traj = dict(run_payload)
        elif traj_slot == "executed":
            self._last_run_payload_executed = dict(run_payload)

        # 2) create total trajectory for existing marker/eval pipeline
        total_traj = self._concat_total_traj_from_run(run_payload)
        if isinstance(total_traj, dict) and total_traj:
            try:
                # Keep existing keys for marker builder and legacy eval code.
                if traj_slot == "traj":
                    self._recipe.trajectories[Recipe.TRAJ_TRAJ] = total_traj
                elif traj_slot == "executed":
                    self._recipe.trajectories[Recipe.TRAJ_EXECUTED] = total_traj
            except Exception:
                pass

        # 3) evaluate (segmentwise + total) if available, else fallback total-only
        for side in self._compiled_sides():
            # preferred new API (your new eval module can hang here)
            try:
                fn = getattr(self._recipe, "evaluate_run_against_compiled", None)
                if callable(fn):
                    fn(run=run_payload, side=side)  # expected to write eval into run/meta or recipe.info
                    continue
            except Exception:
                pass

            # fallback legacy API on total traj
            try:
                if isinstance(total_traj, dict) and total_traj:
                    if traj_slot == "traj":
                        self._recipe.evaluate_trajectory_against_compiled(traj_key=Recipe.TRAJ_TRAJ, side=side)
                    elif traj_slot == "executed":
                        self._recipe.evaluate_trajectory_against_compiled(traj_key=Recipe.TRAJ_EXECUTED, side=side)
            except Exception:
                pass

    def _render_run_yaml_views(self) -> None:
        """
        UI shows the persisted payloads (segments-only), not the derived total traj.
        """
        self.txtPlanned.setPlainText(self._yaml_dump(self._last_run_payload_traj))
        self.txtExecuted.setPlainText(self._yaml_dump(self._last_run_payload_executed))

    def _on_process_finished_success(self, result_obj: object) -> None:
        run_payload: Dict[str, Any] = result_obj if isinstance(result_obj, dict) else {}
        self._append_log("=== Process finished ===")

        # reload recipe fresh (compiled etc.)
        if self._recipe_key:
            try:
                self._recipe = self.repo.load_for_process(self._recipe_key)
            except Exception:
                pass

        # decide slot + save-kind by mode
        if self._active_mode in (ProcessThread.MODE_VALIDATE, ProcessThread.MODE_OPTIMIZE):
            if self._is_run_payload(run_payload):
                self._apply_run_payload_to_recipe_and_eval(run_payload, traj_slot="traj")
                self._maybe_save_run(kind="traj", data=run_payload)

        if self._active_mode == ProcessThread.MODE_EXECUTE:
            if self._is_run_payload(run_payload):
                self._apply_run_payload_to_recipe_and_eval(run_payload, traj_slot="executed")
                self._maybe_save_run(kind="executed", data=run_payload)

            # ensure executed is ON at end of execute
            self._show_executed = True
            try:
                self.sprayPathBox.set_defaults(compiled=self._show_compiled, traj=self._show_traj, executed=True)
            except Exception:
                pass

        # publish layers based on recipe.trajectories (total/legacy) + compiled
        self._publish_all_available_layers()

        # show payload yaml in UI panes
        self._render_run_yaml_views()
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

    def _clear_layers(self) -> None:
        if self.ros is None:
            return
        try:
            empty = MarkerArray()
            self.ros.spray_set_compiled(markers=empty)
            self.ros.spray_set_traj(markers=empty)
            self.ros.spray_set_executed(markers=empty)
        except Exception:
            pass

    def _publish_all_available_layers(self) -> None:
        """
        Marker builder currently consumes Recipe + source in {"compiled_path","traj","executed_traj"}.
        Therefore we publish from recipe.trajectories[TRAJ_TRAJ/TRAJ_EXECUTED] (derived total).
        Persisted YAML remains segments-only in _last_run_payload_* and repo files.
        """
        if self.ros is None or self._recipe is None:
            return

        r = self._recipe

        def _build(source: str) -> Optional[MarkerArray]:
            try:
                part = build_marker_array_from_recipe(r, source=source, frame_id="scene")
                if part is None:
                    return None
                if getattr(part, "markers", None) is None:
                    return None
                return part
            except Exception as e:
                self._append_log(f"W markers ({source}) skipped: {e}")
                return None

        has_compiled = bool(isinstance(getattr(r, "paths_compiled", None), dict) and (r.paths_compiled or {}))
        has_traj = bool(isinstance((r.trajectories or {}).get(Recipe.TRAJ_TRAJ), dict) and (r.trajectories[Recipe.TRAJ_TRAJ] or {}))
        has_exec = bool(isinstance((r.trajectories or {}).get(Recipe.TRAJ_EXECUTED), dict) and (r.trajectories[Recipe.TRAJ_EXECUTED] or {}))

        ma_compiled = _build("compiled_path") if has_compiled else None
        ma_traj = _build("traj") if has_traj else None
        ma_exec = _build("executed_traj") if has_exec else None

        try:
            if ma_compiled is not None:
                self.ros.spray_set_compiled(markers=ma_compiled)
        except Exception as e:
            self._append_log(f"W spray_set_compiled failed: {e}")

        try:
            if ma_traj is not None:
                self.ros.spray_set_traj(markers=ma_traj)
        except Exception as e:
            self._append_log(f"W spray_set_traj failed: {e}")

        try:
            if ma_exec is not None:
                self.ros.spray_set_executed(markers=ma_exec)
        except Exception as e:
            self._append_log(f"W spray_set_executed failed: {e}")

        # Always republish current toggles (idempotent, latched)
        try:
            self.sprayPathBox.publish_current()
        except Exception:
            pass

    # ---------------- UI gating ----------------

    def _update_buttons(self) -> None:
        ros_ok = self.ros is not None
        has_recipe = bool(self._recipe_key)

        self.btnLoad.setEnabled((not self._process_active) and ros_ok)

        self.btnInit.setEnabled((not self._process_active) and ros_ok and (self._init_thread is not None))

        self.btnValidate.setEnabled((not self._process_active) and ros_ok and has_recipe and self._robot_ready)
        self.btnOptimize.setEnabled((not self._process_active) and ros_ok and has_recipe and self._robot_ready)
        self.btnExecute.setEnabled(
            (not self._process_active) and ros_ok and has_recipe and self._robot_ready and (self.plc is not None)
        )

        self.btnStop.setEnabled(ros_ok or (self._init_thread is not None))
