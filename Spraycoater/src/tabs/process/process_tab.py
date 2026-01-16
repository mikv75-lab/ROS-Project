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

from geometry_msgs.msg import PoseArray  # type: ignore
from visualization_msgs.msg import MarkerArray  # type: ignore

from plc.plc_client import PlcClientBase
from ros.bridge.ros_bridge import RosBridge

from model.recipe.recipe import Recipe
from model.recipe.recipe_markers import (
    build_marker_array_from_recipe,
    build_tcp_pose_array_from_tcp_yaml,
    build_tcp_marker_array_from_tcp_yaml,
)

from model.recipe.recipe_run_result import RunResult

from widgets.robot_status_box import RobotStatusInfoBox
from widgets.info_groupbox import InfoGroupBox

from tabs.process.spray_path_box import SprayPathBox

from .process_thread import ProcessThread
from .robot_init_thread import RobotInitThread

from .base_statemachine import (
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
)

from config.startup import resolve_path

_LOG = logging.getLogger("tabs.process")


class ProcessTab(QWidget):
    """
    STRICT ProcessTab.

    Key fixes:
      - NEVER publish PoseArray with empty header.frame_id (RViz drops it; latched topics keep it).
      - Canonical publish frame is 'substrate' (as requested).
      - On clear, publish empty PoseArray with frame_id='substrate' (not 'world').
      - When loading/publishing layers, enforce frame_id on PoseArray and on every Marker.header.

    FK/TCP strictness:
      - FK->TCP must be produced from planned_traj / executed_traj (JTBySegment YAML), not from draft.
      - TCP output frame is 'substrate' (publish frame).
      - Mount offsets come from substrate_mounts.yaml (SSoT). For offline transform, RunResult requires:
          scene_yaml_path + robot_yaml_path + mounts_yaml_path
        (mounts YAML path, not only a loaded dict).
    """

    _SEG_ORDER = (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME)
    _DEFAULT_PUBLISH_FRAME = "substrate"

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

        if repo is None:
            raise RuntimeError("ProcessTab: repo ist None (ctx.repo fehlt?)")
        if ros is None:
            raise RuntimeError("ProcessTab: ros ist None (strict)")
        self.ctx = ctx
        self.repo = repo
        self.ros: RosBridge = ros
        self.plc = plc

        for name in ("list_recipes", "load_for_process"):
            fn = getattr(self.repo, name, None)
            if not callable(fn):
                raise RuntimeError(f"ProcessTab: repo missing required API: {name}()")
        if getattr(getattr(self.repo, "bundle", None), "paths", None) is None:
            raise RuntimeError("ProcessTab: repo.bundle.paths fehlt (strict)")

        self._recipe: Optional[Recipe] = None
        self._recipe_key: Optional[str] = None

        self._process_thread: Optional[ProcessThread] = None

        self._init_thread: Optional[RobotInitThread] = None
        self._robot_ready: bool = False

        self._process_active: bool = False
        self._active_mode: str = ""

        self._show_compiled: bool = True
        self._show_traj: bool = True
        self._show_executed: bool = True

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        top_row = QHBoxLayout()
        top_row.setSpacing(8)

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

        self.robotStatusBox = RobotStatusInfoBox(self, title="Robot Status")
        top_row.addWidget(self.robotStatusBox, 2)

        root.addLayout(top_row)

        row_info = QHBoxLayout()
        row_info.setSpacing(8)

        self.infoBox = InfoGroupBox(self)
        row_info.addWidget(self.infoBox, 2)

        self.sprayPathBox = SprayPathBox(ros=self.ros, parent=self, title="Spray Paths")
        row_info.addWidget(self.sprayPathBox, 1)

        root.addLayout(row_info)

        row_eval_log = QHBoxLayout()
        row_eval_log.setSpacing(8)

        left_eval = QVBoxLayout()
        left_eval.setSpacing(8)

        grp_pl = QGroupBox("Planned (Eval)", self)
        vpl = QVBoxLayout(grp_pl)
        self.txtPlannedEval = QTextEdit(grp_pl)
        self.txtPlannedEval.setReadOnly(True)
        self.txtPlannedEval.setPlaceholderText("Loaded from planned_tcp.yaml: eval")
        vpl.addWidget(self.txtPlannedEval, 1)
        self.lblPlannedSummary = QLabel("planned: –", grp_pl)
        vpl.addWidget(self.lblPlannedSummary)
        left_eval.addWidget(grp_pl, 1)

        grp_ex = QGroupBox("Executed (Eval)", self)
        vex = QVBoxLayout(grp_ex)
        self.txtExecutedEval = QTextEdit(grp_ex)
        self.txtExecutedEval.setReadOnly(True)
        self.txtExecutedEval.setPlaceholderText("Loaded from executed_tcp.yaml: eval")
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

        # wiring
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

        # Defaults: show all layers in GUI + publish current toggles
        self._spray_defaults_publish(compiled=True, planned=True, executed=True)

        self._update_buttons()
        self._update_recipe_box()
        self._update_info_box()
        self._clear_eval_views()

    # ---------------------------------------------------------------------
    # Buttons
    # ---------------------------------------------------------------------

    def _update_buttons(self) -> None:
        has_recipe = bool(self._recipe_key)
        running = bool(self._process_active)
        init_running = bool(self._init_thread is not None and self._init_thread.isRunning())
        can_click = (not running) and (not init_running)

        # Load is only allowed when nothing is running
        self.btnLoad.setEnabled(can_click)

        # Init only when nothing running
        self.btnInit.setEnabled(can_click)

        # Stop enabled if any worker active
        self.btnStop.setEnabled(bool(running or init_running))

        # Process buttons
        self.btnValidate.setEnabled(can_click and has_recipe and self._robot_ready)
        self.btnOptimize.setEnabled(can_click and has_recipe and self._robot_ready)
        self.btnExecute.setEnabled(can_click and has_recipe and self._robot_ready and self._plc_execute_available())

        # If no recipe -> disable all process
        if not has_recipe:
            self.btnValidate.setEnabled(False)
            self.btnOptimize.setEnabled(False)
            self.btnExecute.setEnabled(False)

    # ---------------------------------------------------------------------
    # PLC availability (Execute) - supports sim mode even if plc is None
    # ---------------------------------------------------------------------

    def _plc_execute_available(self) -> bool:
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
    # URDF/SRDF from ctx (deterministic)
    # ---------------------------------------------------------------------

    def _ctx_robot_xml(self) -> Tuple[str, str]:
        try:
            rd = getattr(self.ctx, "robot_description", None)
            if rd is None:
                return "", ""
            urdf = str(getattr(rd, "urdf_xml", "") or "")
            srdf = str(getattr(rd, "srdf_xml", "") or "")
            return urdf, srdf
        except Exception:
            return "", ""

    # ---------------------------------------------------------------------
    # Deterministic config path resolution (scene.yaml + robot.yaml + mounts.yaml)
    # ---------------------------------------------------------------------

    def _ctx_config_yaml_paths(self) -> Tuple[str, str]:
        base_dir = os.environ.get("SC_PROJECT_ROOT", "").strip()
        if not base_dir:
            return "", ""

        try:
            ros_cfg = getattr(self.ctx, "ros", None)
            cfgs = getattr(ros_cfg, "configs", None) if ros_cfg is not None else None

            scene_uri = str(getattr(cfgs, "scene_file", "") or "").strip()
            robot_uri = str(getattr(cfgs, "robot_file", "") or "").strip()

            if not scene_uri or not robot_uri:
                return "", ""

            scene_abs = resolve_path(base_dir, scene_uri)
            robot_abs = resolve_path(base_dir, robot_uri)

            if not os.path.isfile(scene_abs) or not os.path.isfile(robot_abs):
                return "", ""
            return scene_abs, robot_abs
        except Exception:
            return "", ""

    def _ctx_mounts_yaml_path(self) -> str:
        """
        STRICT: Resolve substrate_mounts.yaml path (SSoT), required for offline TCP transform into 'substrate'.

        We DO NOT guess defaults here. We only accept explicit configuration in ctx:
          - ctx.ros.configs.mounts_file / substrate_mounts_file / mounts_yaml_file
          - or ctx.mounts_yaml_path (if your startup stores it)

        Returns "" if not resolvable.
        """
        base_dir = os.environ.get("SC_PROJECT_ROOT", "").strip()
        if not base_dir:
            return ""

        # 1) direct attribute if your startup stores it
        try:
            p = str(getattr(self.ctx, "mounts_yaml_path", "") or "").strip()
            if p:
                p_abs = resolve_path(base_dir, p)
                if os.path.isfile(p_abs):
                    return p_abs
        except Exception:
            pass

        # 2) from ctx.ros.configs
        try:
            ros_cfg = getattr(self.ctx, "ros", None)
            cfgs = getattr(ros_cfg, "configs", None) if ros_cfg is not None else None
            if cfgs is None:
                return ""

            for attr in ("mounts_file", "substrate_mounts_file", "mounts_yaml_file", "substrate_mounts_yaml_file"):
                uri = str(getattr(cfgs, attr, "") or "").strip()
                if not uri:
                    continue
                p_abs = resolve_path(base_dir, uri)
                if os.path.isfile(p_abs):
                    return p_abs
        except Exception:
            return ""

        return ""

    # ---------------------------------------------------------------------
    # SprayPathBox compat helpers
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

    # ---------------- Spray toggles (cache only) ----------------

    def _on_show_compiled_toggled(self, v: bool) -> None:
        self._show_compiled = bool(v)
        self._publish_layers_from_bundle(reason="toggle_compiled")

    def _on_show_traj_toggled(self, v: bool) -> None:
        self._show_traj = bool(v)
        self._publish_layers_from_bundle(reason="toggle_planned")

    def _on_show_executed_toggled(self, v: bool) -> None:
        self._show_executed = bool(v)
        self._publish_layers_from_bundle(reason="toggle_executed")

    # ---------------- RobotStatus wiring ----------------

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
            "draft_present": bool(getattr(r, "draft", None) is not None),
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
        self.txtPlannedEval.setPlainText("–")
        self.txtExecutedEval.setPlainText("–")
        self.lblPlannedSummary.setText("planned: –")
        self.lblExecutedSummary.setText("executed: –")

    @staticmethod
    def _extract_eval_pair(rr: RunResult) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        planned: Dict[str, Any] = {}
        executed: Dict[str, Any] = {}

        try:
            ev = getattr(rr, "eval", None)
            if isinstance(ev, dict):
                p = ev.get("planned")
                e = ev.get("executed")
                planned = p if isinstance(p, dict) else {}
                executed = e if isinstance(e, dict) else {}
        except Exception:
            pass

        if not planned:
            try:
                pr = getattr(rr, "planned_run", None)
                pr = pr if isinstance(pr, dict) else {}
                tcp = pr.get("tcp")
                tcp = tcp if isinstance(tcp, dict) else {}
                planned = tcp.get("eval") if isinstance(tcp.get("eval"), dict) else {}
            except Exception:
                planned = {}

        if not executed:
            try:
                er = getattr(rr, "executed_run", None)
                er = er if isinstance(er, dict) else {}
                tcp = er.get("tcp")
                tcp = tcp if isinstance(tcp, dict) else {}
                executed = tcp.get("eval") if isinstance(tcp.get("eval"), dict) else {}
            except Exception:
                executed = {}

        return planned, executed

    @staticmethod
    def _summary_line_from_eval(ev: Dict[str, Any]) -> str:
        if not isinstance(ev, dict) or not ev:
            return ""
        try:
            valid = ev.get("valid")
            thr = ev.get("threshold")
            total = ev.get("total") if isinstance(ev.get("total"), dict) else {}
            score = total.get("score") if isinstance(total, dict) else None
            domain = ev.get("domain")
            if not domain:
                m = total.get("metrics")
                if isinstance(m, dict):
                    domain = m.get("label")
            parts = []
            if domain:
                parts.append(str(domain))
            if score is not None:
                parts.append(f"score={float(score):.3f}")
            if thr is not None:
                parts.append(f"thr={float(thr):.3f}")
            if valid is not None:
                parts.append(f"valid={bool(valid)}")
            return " | ".join(parts)
        except Exception:
            return ""

    def _set_eval_from_runresult(self, rr: RunResult) -> None:
        planned_ev, executed_ev = self._extract_eval_pair(rr)

        if planned_ev:
            self.txtPlannedEval.setPlainText(self._yaml_dump(planned_ev))
            self.lblPlannedSummary.setText("planned: " + (self._summary_line_from_eval(planned_ev) or "–"))
        else:
            self.txtPlannedEval.setPlainText("–")
            self.lblPlannedSummary.setText("planned: –")

        if executed_ev:
            self.txtExecutedEval.setPlainText(self._yaml_dump(executed_ev))
            self.lblExecutedSummary.setText("executed: " + (self._summary_line_from_eval(executed_ev) or "–"))
        else:
            self.txtExecutedEval.setPlainText("–")
            self.lblExecutedSummary.setText("executed: –")

    def _set_eval_missing_fk(self, extra: str = "") -> None:
        msg = (
            "FK->TCP / Eval wurde übersprungen.\n"
            "Ursache: fehlendes/ungültiges URDF/SRDF oder fehlende Config-Dateien.\n\n"
            "Trajectories wurden trotzdem erzeugt/persistiert (falls Speichern erlaubt).\n"
        )
        if extra:
            msg += f"\nDetails:\n{extra}\n"
        self.txtPlannedEval.setPlainText(msg)
        self.txtExecutedEval.setPlainText(msg)
        self.lblPlannedSummary.setText("planned: – (no TCP)")
        self.lblExecutedSummary.setText("executed: – (no TCP)")

    # ---------------------------------------------------------------------
    # Publishing helpers (STRICT frame_id)
    # ---------------------------------------------------------------------

    def _ensure_markerarray_frame(self, ma: MarkerArray, frame_id: str) -> MarkerArray:
        try:
            for m in list(ma.markers or []):
                try:
                    m.header.frame_id = str(frame_id)
                except Exception:
                    pass
        except Exception:
            pass
        return ma

    def _publish_compiled_from_recipe(self, recipe: Recipe, *, frame_id: str) -> None:
        if not self._show_compiled:
            # Wenn compiled aus ist, lieber nichts setzen (oder optional löschen).
            return
        try:
            ma = build_marker_array_from_recipe(recipe)
            ma = self._ensure_markerarray_frame(ma, frame_id)
            self.ros.spray_set_compiled(markers=ma)
        except Exception as e:
            self._append_log(f"W: publish compiled failed: {e}")
            
    def _publish_tcp_layer(self, tcp_doc: Dict[str, Any], *, which: str, frame_id: str) -> None:
        if which == "planned" and (not self._show_traj):
            return
        if which == "executed" and (not self._show_executed):
            return
        try:
            # PoseArray
            pa = build_tcp_pose_array_from_tcp_yaml(tcp_doc, frame_id=str(frame_id))
            if isinstance(pa, PoseArray):
                pa.header.frame_id = str(frame_id)

            # MarkerArray
            ma = build_tcp_marker_array_from_tcp_yaml(tcp_doc, frame_id=str(frame_id))
            if isinstance(ma, MarkerArray):
                ma = self._ensure_markerarray_frame(ma, frame_id)

            if which == "planned":
                self.ros.spray_set_planned(poses=pa, markers=ma)
            else:
                self.ros.spray_set_executed(poses=pa, markers=ma)

        except Exception as e:
            self._append_log(f"W: publish tcp({which}) failed: {e}")


    def _clear_layers(self, *, frame_id: str) -> None:
        frame_id = str(frame_id or self._DEFAULT_PUBLISH_FRAME)

        try:
            # SprayPathBridge hat eine Clear-Funktion die alle Layer leert
            self.ros.spray_clear()
        except Exception:
            pass

        # Optional: “leere” PoseArrays mit gültigem frame_id setzen,
        # damit RViz das Topic nicht „verliert“ (falls SprayPathBridge das nicht selbst macht).
        try:
            pa = PoseArray()
            pa.header.frame_id = frame_id
            self.ros.spray_set_planned(poses=pa, markers=None)
        except Exception:
            pass
        try:
            pa = PoseArray()
            pa.header.frame_id = frame_id
            self.ros.spray_set_executed(poses=pa, markers=None)
        except Exception:
            pass


    def _publish_layers_from_bundle(self, *, reason: str = "") -> None:
        if not self._recipe_key:
            return

        frame_id = self._DEFAULT_PUBLISH_FRAME

        # compiled from loaded recipe (if available)
        if self._recipe is not None:
            self._publish_compiled_from_recipe(self._recipe, frame_id=frame_id)

        # planned/executed tcp from persisted yaml (if exists)
        try:
            p = self.repo.bundle.paths(self._recipe_key)
            planned_tcp_path = str(getattr(p, "planned_tcp_yaml", "") or "")
            executed_tcp_path = str(getattr(p, "executed_tcp_yaml", "") or "")
        except Exception:
            planned_tcp_path, executed_tcp_path = "", ""

        planned_tcp_doc = self._yaml_load_file(planned_tcp_path) if planned_tcp_path else {}
        executed_tcp_doc = self._yaml_load_file(executed_tcp_path) if executed_tcp_path else {}

        if isinstance(planned_tcp_doc, dict) and planned_tcp_doc:
            self._publish_tcp_layer(planned_tcp_doc, which="planned", frame_id=frame_id)
        if isinstance(executed_tcp_doc, dict) and executed_tcp_doc:
            self._publish_tcp_layer(executed_tcp_doc, which="executed", frame_id=frame_id)

    # ---------------- Bundle Load (Eval) ----------------

    def _load_runresult_from_bundle(self, *, publish_layers: bool = False) -> None:
        self._clear_eval_views()
        if not self._recipe_key:
            return

        try:
            p = self.repo.bundle.paths(self._recipe_key)
            planned_tcp_path = str(getattr(p, "planned_tcp_yaml", "") or "")
            executed_tcp_path = str(getattr(p, "executed_tcp_yaml", "") or "")
        except Exception:
            return

        planned_tcp_doc = self._yaml_load_file(planned_tcp_path) if planned_tcp_path else {}
        executed_tcp_doc = self._yaml_load_file(executed_tcp_path) if executed_tcp_path else {}

        # For display, we only need eval dicts; no need to reconstruct RunResult from disk.
        rr = RunResult(
            planned_run={"traj": {}, "tcp": planned_tcp_doc if isinstance(planned_tcp_doc, dict) else {}},
            executed_run={"traj": {}, "tcp": executed_tcp_doc if isinstance(executed_tcp_doc, dict) else {}},
            eval={
                "planned": (planned_tcp_doc.get("eval") if isinstance(planned_tcp_doc, dict) else {}) or {},
                "executed": (executed_tcp_doc.get("eval") if isinstance(executed_tcp_doc, dict) else {}) or {},
            },
            valid=True,
            invalid_reason="",
        )
        self._set_eval_from_runresult(rr)

        if publish_layers:
            self._publish_layers_from_bundle(reason="eval_loaded")

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
            self._clear_layers(frame_id=self._DEFAULT_PUBLISH_FRAME)
            return

        self._load_runresult_from_bundle(publish_layers=True)

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

        if mode == ProcessThread.MODE_EXECUTE:
            if getattr(recipe_run, "planned_traj", None) is None:
                QMessageBox.warning(
                    self,
                    "Execute",
                    "Execute benötigt eine geplante Baseline (planned_traj.yaml).\n\n"
                    "Bitte zuerst Validate/Optimize ausführen oder planned_traj.yaml bereitstellen.",
                )
                return

        urdf_xml, srdf_xml = self._ctx_robot_xml()
        if not urdf_xml or not srdf_xml:
            self._append_log("W: ctx.robot_description fehlt/leer → FK/TCP/Eval wird später übersprungen.")

        self._process_active = True
        self._active_mode = mode
        self.txtLog.clear()
        self._clear_eval_views()

        self._append_log(f"=== {mode.upper()} gestartet ===")
        self._update_buttons()

        # IMPORTANT: ProcessThread gets ctx (RosBridge unchanged)
        self._process_thread = ProcessThread(
            ctx=self.ctx,
            recipe=recipe_run,
            ros=self.ros,
            plc=self.plc,
            mode=mode,
            urdf_xml=urdf_xml,
            srdf_xml=srdf_xml,
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

    def _should_overwrite(self, *, which: str, filename: str) -> bool:
        if not filename:
            return True
        if not os.path.isfile(filename):
            return True
        which = (which or "").strip().lower()
        if which == "executed":
            return True
        return self._maybe_overwrite_prompt(title="Run speichern", filename=filename)

    # ---------------- Process result handling ----------------

    def _finish_process_ui(self) -> None:
        self._process_active = False
        self._active_mode = ""

        pt = self._process_thread
        self._process_thread = None

        if pt is not None:
            try:
                pt.notifyFinished.disconnect()
            except Exception:
                pass
            try:
                pt.notifyError.disconnect()
            except Exception:
                pass
            try:
                pt.stateChanged.disconnect()
            except Exception:
                pass
            try:
                pt.logMessage.disconnect()
            except Exception:
                pass
            try:
                pt.deleteLater()
            except Exception:
                pass

        self._update_buttons()

    def _persist_runresult(self, *, rr: RunResult) -> None:
        if not self._recipe_key:
            return

        try:
            p = self.repo.bundle.paths(self._recipe_key)
            planned_traj_path = str(getattr(p, "planned_traj_yaml", "") or "")
            executed_traj_path = str(getattr(p, "executed_traj_yaml", "") or "")
            planned_tcp_path = str(getattr(p, "planned_tcp_yaml", "") or "")
            executed_tcp_path = str(getattr(p, "executed_tcp_yaml", "") or "")
        except Exception as e:
            self._append_log(f"E: bundle.paths failed: {e}")
            return

        planned_traj_doc = rr.planned_run.get("traj") if isinstance(rr.planned_run, dict) else {}
        executed_traj_doc = rr.executed_run.get("traj") if isinstance(rr.executed_run, dict) else {}
        planned_tcp_doc = rr.planned_run.get("tcp") if isinstance(rr.planned_run, dict) else {}
        executed_tcp_doc = rr.executed_run.get("tcp") if isinstance(rr.executed_run, dict) else {}

        # planned overwrite prompt (executed always allowed)
        if planned_traj_path and not self._should_overwrite(which="planned", filename=planned_traj_path):
            self._append_log("Persist: planned_traj skipped (user declined overwrite).")
        else:
            if planned_traj_path:
                self._yaml_write_file(planned_traj_path, planned_traj_doc)

        if executed_traj_path:
            self._yaml_write_file(executed_traj_path, executed_traj_doc)

        # TCP docs only if present
        if planned_tcp_path and isinstance(planned_tcp_doc, dict) and planned_tcp_doc:
            if planned_tcp_path and not self._should_overwrite(which="planned", filename=planned_tcp_path):
                self._append_log("Persist: planned_tcp skipped (user declined overwrite).")
            else:
                self._yaml_write_file(planned_tcp_path, planned_tcp_doc)

        if executed_tcp_path and isinstance(executed_tcp_doc, dict) and executed_tcp_doc:
            self._yaml_write_file(executed_tcp_path, executed_tcp_doc)

        self._append_log("Persist: OK")

    @QtCore.pyqtSlot(object)
    def _on_process_finished_success(self, payload: object) -> None:
        """
        payload is RunResult.to_process_payload() dict emitted by statemachines/process_thread.
        """
        try:
            rr = RunResult.from_process_payload(payload)
        except Exception as e:
            self._append_log(f"E: RunResult.from_process_payload failed: {e}")
            self._finish_process_ui()
            return

        # Postprocess: FK->TCP->Eval (strict, but skip gracefully if missing prerequisites)
        extra = ""
        try:
            scene_yaml, robot_yaml = self._ctx_config_yaml_paths()
            mounts_yaml = self._ctx_mounts_yaml_path()

            # If urdf/srdf empty -> RunResult.postprocess will raise
            rr.postprocess(
                recipe=self._recipe if self._recipe is not None else getattr(self._process_thread, "recipe", None),
                segment_order=self._SEG_ORDER,
                ee_link="tcp",
                step_mm=1.0,
                max_points=0,
                gate_valid_on_eval=False,
                require_tcp=True,
                tcp_target_frame=self._DEFAULT_PUBLISH_FRAME,
                scene_yaml_path=scene_yaml or None,
                robot_yaml_path=robot_yaml or None,
                mounts_yaml_path=mounts_yaml or None,
            )
        except Exception as e:
            extra = str(e)
            self._append_log(f"W: postprocess skipped/failed: {e}")
            self._set_eval_missing_fk(extra=extra)

        # Update eval UI if we have it
        try:
            self._set_eval_from_runresult(rr)
        except Exception:
            pass

        # Persist
        try:
            self._persist_runresult(rr=rr)
        except Exception as e:
            self._append_log(f"E: Persist failed: {e}")

        # Publish layers from persisted bundle (single source of truth)
        try:
            self._publish_layers_from_bundle(reason="process_finished_success")
        except Exception:
            pass

        self._finish_process_ui()

    @QtCore.pyqtSlot(str)
    def _on_process_finished_error(self, msg: str) -> None:
        self._append_log(f"=== Process ERROR: {msg} ===")
        QMessageBox.critical(self, "Process", str(msg or "Unbekannter Fehler"))
        self._finish_process_ui()
