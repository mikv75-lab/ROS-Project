# -*- coding: utf-8 -*-
# File: tabs/process/process_tab.py
from __future__ import annotations

import logging
from typing import Optional, Any, Dict, List

import yaml

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

from model.recipe.recipe import Recipe
from model.recipe.recipe_markers import build_marker_array_from_recipe

from widgets.robot_status_box import RobotStatusInfoBox
from widgets.info_groupbox import InfoGroupBox

from .process_thread import ProcessThread
from .robot_init_thread import RobotInitThread

_LOG = logging.getLogger("tabs.process")


class ProcessTab(QWidget):
    """
    ProcessTab (aktuell)

    Layout:
      - Top Row: Recipe (links) | Process (mitte) | Robot Status (rechts)
      - InfoGroupBox (wie im RecipeTab, ETA bleibt)
      - Run Data (scored)
      - Log

    Behavior:
      - Load: lädt process-mode Rezept (draft + compiled + last runs)
      - Beim Load: publisht alle 3 Layer (compiled/traj/executed) sofern vorhanden
      - Process buttons: Init/Validate/Optimize/Execute/Stop
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

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # =========================================================
        # TOP ROW: Recipe (links) + Process (mitte) + Robot Status (rechts)
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

        # ---------------- InfoGroupBox ----------------
        self.infoBox = InfoGroupBox(self)
        root.addWidget(self.infoBox)

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

        # init thread
        self._setup_init_thread()

        # status timer
        self._status_timer = QTimer(self)
        self._status_timer.setInterval(300)
        self._status_timer.timeout.connect(self._update_robot_status_widget)
        self._status_timer.start()

        self._update_buttons()
        self._update_robot_status_widget()
        self._update_recipe_box()
        self._update_info_box()

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

        # compiled info (falls vorhanden)
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

    # ---------------- Robot Status Box update ----------------

    def _update_robot_status_widget(self) -> None:
        box = self.robotStatusBox

        if self.ros is None:
            box.set_connection("disabled")
            box.set_mode(self._active_mode or "idle")
            box.set_initialized(bool(self._robot_ready))
            box.set_moving(False)
            box.set_servo_enabled(False)
            box.set_power(False)
            box.set_estop(False)
            box.set_errors("-")
            box.set_tcp_pose6((-1, -1, -1, 0, 0, 0))
            box.set_joints(None)
            return

        status_dict = None
        for fn_name in ("get_status", "status_dict", "robot_status_dict", "get_robot_status"):
            fn = getattr(self.ros, fn_name, None)
            if callable(fn):
                try:
                    v = fn()
                    if isinstance(v, dict):
                        status_dict = v
                        break
                except Exception:
                    pass
            elif isinstance(fn, dict):
                status_dict = fn
                break

        if isinstance(status_dict, dict):
            try:
                box.set_status_dict(status_dict)
            except Exception:
                pass
        else:
            ros_ok = None
            for attr in ("is_connected", "connected", "ok", "is_ok"):
                v = getattr(self.ros, attr, None)
                if callable(v):
                    try:
                        ros_ok = bool(v())
                        break
                    except Exception:
                        pass
                elif isinstance(v, bool):
                    ros_ok = v
                    break
            box.set_connection(ros_ok if ros_ok is not None else "connected?")
            box.set_mode(self._active_mode or "idle")
            box.set_initialized(bool(self._robot_ready))

        # optional best-effort: tcp + joints
        ps = None
        for attr in ("tcp_pose_stamped", "tcp_pose", "last_tcp_pose", "tcp_ps"):
            v = getattr(self.ros, attr, None)
            if v is not None:
                ps = v() if callable(v) else v
                if ps is not None:
                    break
        if ps is not None:
            try:
                box.set_tcp_from_ps(ps)
            except Exception:
                pass

        joints = None
        for attr in ("joint_positions", "joints", "last_joints", "q"):
            v = getattr(self.ros, attr, None)
            if v is not None:
                joints = v() if callable(v) else v
                if joints is not None:
                    break
        if joints is not None:
            try:
                box.set_joints(joints)
            except Exception:
                pass

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

        self.txtPlanned.clear()
        self.txtExecuted.clear()

        if recipe:
            # ✅ beim Load: alle 3 publizieren sofern verfügbar
            self._publish_all_available_layers()
            self._render_run_yaml_views()
            self._trigger_ros_on_loaded_recipe()
        else:
            self._clear_layers()

        self._update_robot_status_widget()

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
        """Create the persistent RobotInitThread exactly once.

        The RobotInitThread itself owns a persistent QThread + a single worker instance.
        ProcessTab must therefore NOT recreate it on repeated UI actions.
        """
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

        self._robot_ready = False
        self.lblInit.setText("Init: START")
        self._append_log("=== Robot-Init gestartet ===")
        self._init_thread.startSignal.emit()
        self._update_buttons()
        self._update_robot_status_widget()

    def _on_init_finished_ok(self) -> None:
        self._robot_ready = True
        self.lblInit.setText("Init: OK")
        self._append_log("=== Robot-Init erfolgreich abgeschlossen ===")
        self._update_buttons()
        self._update_robot_status_widget()

    def _on_init_finished_err(self, msg: str) -> None:
        self._robot_ready = False
        self.lblInit.setText("Init: ERROR")
        self._append_log(f"=== Robot-Init Fehler: {msg} ===")
        self._update_buttons()
        self._update_robot_status_widget()

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

        # IMPORTANT: load fresh process recipe snapshot for this run
        try:
            recipe_run = self.repo.load_for_process(self._recipe_key)
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
        self._update_robot_status_widget()

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

        for side in self._compiled_sides():
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
        if self._recipe_key:
            try:
                self._recipe = self.repo.load_for_process(self._recipe_key)
            except Exception:
                pass

        self._apply_payload_to_recipe_and_eval(payload)

        # publish all available after run
        self._publish_all_available_layers()

        self._render_run_yaml_views()
        self._update_recipe_box()
        self._update_info_box()

        self._process_active = False
        self._active_mode = ""
        self._process_thread = None
        self._update_buttons()
        self._update_robot_status_widget()

    def _on_process_finished_error(self, msg: str) -> None:
        self._append_log(f"=== Process ERROR: {msg} ===")
        self._process_active = False
        self._active_mode = ""
        self._process_thread = None
        self._update_buttons()
        self._update_robot_status_widget()

    # ---------------- SprayPath publish ----------------

    def _clear_layers(self) -> None:
        """
        Clears all SprayPath caches/views best-effort.
        Note: SprayPath node ignores empty messages for selection,
        but publishing empty MarkerArrays is still useful to "visually clear"
        in RViz if your display is set to replace.
        """
        if self.ros is None:
            return
        try:
            empty = MarkerArray()
            self.ros.spray_set_compiled(markers=empty)
            self.ros.spray_set_traj(markers=empty)
            self.ros.spray_set_executed(markers=empty)
            # keep current view as-is (or clear if you add a "clear" command topic later)
        except Exception:
            pass

    def _publish_all_available_layers(self) -> None:
        """
        Beim Load/Run-Ende: publisht compiled/traj/executed sofern vorhanden.
        Nutzt SprayPathBridge API (ros.spray_set_*).
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
                # erlaub auch leere arrays (für clear)
                return part
            except Exception as e:
                self._append_log(f"W markers ({source}) skipped: {e}")
                return None

        has_compiled = bool(isinstance(getattr(r, "paths_compiled", None), dict) and (r.paths_compiled or {}))
        has_traj = bool(isinstance((r.trajectories or {}).get(Recipe.TRAJ_TRAJ), dict))
        has_exec = bool(isinstance((r.trajectories or {}).get(Recipe.TRAJ_EXECUTED), dict))

        ma_compiled = _build("compiled_path") if has_compiled else None
        ma_traj = _build("traj") if has_traj else None
        ma_exec = _build("executed_traj") if has_exec else None

        # publish into the three input slots
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

        # choose a reasonable default view if nothing selected yet or after load:
        # prefer compiled > traj > executed
        try:
            if has_compiled:
                self.ros.spray_set_view("compiled_path")
            elif has_traj:
                self.ros.spray_set_view("traj")
            elif has_exec:
                self.ros.spray_set_view("executed_traj")
        except Exception:
            pass

    # ---------------- UI gating ----------------

    def _update_buttons(self) -> None:
        ros_ok = self.ros is not None
        has_recipe = bool(self._recipe_key)

        self.btnLoad.setEnabled((not self._process_active) and ros_ok)

        # Init immer möglich (wenn ROS + init_thread)
        self.btnInit.setEnabled((not self._process_active) and ros_ok and (self._init_thread is not None))

        self.btnValidate.setEnabled((not self._process_active) and ros_ok and has_recipe and self._robot_ready)
        self.btnOptimize.setEnabled((not self._process_active) and ros_ok and has_recipe and self._robot_ready)
        self.btnExecute.setEnabled(
            (not self._process_active) and ros_ok and has_recipe and self._robot_ready and (self.plc is not None)
        )

        # Stop immer möglich (best-effort)
        self.btnStop.setEnabled(ros_ok or (self._init_thread is not None))
