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

from geometry_msgs.msg import PoseArray  # type: ignore
from visualization_msgs.msg import MarkerArray  # type: ignore

from plc.plc_client import PlcClientBase
from ros.bridge.ros_bridge import RosBridge

from model.recipe.recipe import Recipe, Draft
from model.recipe.recipe_markers import (
    build_marker_array_from_recipe,            # draft markers (legacy helper, but still fine)
    build_tcp_pose_array_from_tcp_yaml,        # planned/executed tcp -> PoseArray
    build_tcp_marker_array_from_tcp_yaml,      # planned/executed tcp -> MarkerArray
)

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

# IMPORTANT: use the SAME resolver logic as startup.py (package://, relative, abs)
from config.startup import resolve_path

_LOG = logging.getLogger("tabs.process")


class ProcessTab(QWidget):
    """
    ProcessTab (strict, no fallbacks, single result schema)

    Key behavior (RViz publishing):
      - Always publish ALL 6 topics after Load and after Process finish:
          compiled: PoseArray + MarkerArray
          planned:  PoseArray + MarkerArray
          executed: PoseArray + MarkerArray
      - compiled is built from recipe.draft (mm->m) and draft markers.
      - planned/executed are built from persisted planned_tcp.yaml / executed_tcp.yaml.

    IMPORTANT (fix for your current issue):
      - DO NOT force frame_id="world" if your TCP docs say "scene".
        Otherwise markers (scene) look correct, but PoseArray (world) is offset.
        
      - This implementation uses the TCP doc's frame if present; otherwise falls back to "scene".
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
    # URDF/SRDF from ctx (deterministic)
    # ---------------------------------------------------------------------

    def _ctx_robot_xml(self) -> Tuple[str, str]:
        """
        Deterministic URDF/SRDF: comes from ctx.robot_description (startup loader).
        Returns ("","") if missing.
        """
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
    # Deterministic config path resolution (scene.yaml + robot.yaml)
    # ---------------------------------------------------------------------

    def _ctx_config_yaml_paths(self) -> Tuple[str, str]:
        """
        Resolve absolute filesystem paths for:
          - scene.yaml
          - robot.yaml

        Source of truth is ctx.ros.configs.scene_file / robot_file (startup.yaml).
        These can be "package://..." or relative to SC_PROJECT_ROOT.

        Returns ("","") if not resolvable.
        """
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

    # ---------------- Eval UI (planned/executed; from TCP YAML) ----------------

    def _clear_eval_views(self) -> None:
        self.txtPlannedEval.setPlainText("–")
        self.txtExecutedEval.setPlainText("–")
        self.lblPlannedSummary.setText("planned: –")
        self.lblExecutedSummary.setText("executed: –")

    @staticmethod
    def _dict(v: Any) -> Dict[str, Any]:
        return v if isinstance(v, dict) else {}

    @staticmethod
    def _extract_eval_from_tcp_doc(tcp_doc: Dict[str, Any]) -> Dict[str, Any]:
        """
        tcp.yaml expected:
          frame: ...
          poses: [...]
          eval: { ... }   # per-run eval result (planned or executed)
        """
        if not isinstance(tcp_doc, dict):
            return {}
        ev = tcp_doc.get("eval")
        return ev if isinstance(ev, dict) else {}

    @staticmethod
    def _extract_eval_pair(rr: RunResult) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """
        Prefer RunResult.eval = {"planned":..., "executed":...}
        fallback to rr.planned_run["tcp"]["eval"] / rr.executed_run["tcp"]["eval"]
        """
        planned: Dict[str, Any] = {}
        executed: Dict[str, Any] = {}

        # 1) primary: rr.eval dict
        try:
            ev = getattr(rr, "eval", None)
            if isinstance(ev, dict):
                p = ev.get("planned")
                e = ev.get("executed")
                planned = p if isinstance(p, dict) else {}
                executed = e if isinstance(e, dict) else {}
        except Exception:
            pass

        # 2) fallback: inside planned_run/executed_run tcp blocks
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
        """
        Compact one-line summary from eval dict.
        """
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

        # planned
        if planned_ev:
            self.txtPlannedEval.setPlainText(self._yaml_dump(planned_ev))
            ps = self._summary_line_from_eval(planned_ev) or "–"
            self.lblPlannedSummary.setText("planned: " + ps)
        else:
            self.txtPlannedEval.setPlainText("–")
            self.lblPlannedSummary.setText("planned: –")

        # executed
        if executed_ev:
            self.txtExecutedEval.setPlainText(self._yaml_dump(executed_ev))
            es = self._summary_line_from_eval(executed_ev) or "–"
            self.lblExecutedSummary.setText("executed: " + es)
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

    def _load_runresult_from_bundle(self) -> None:
        self._clear_eval_views()
        if not self._recipe_key:
            return

        try:
            p = self.repo.bundle.paths(self._recipe_key)
            planned_path = str(getattr(p, "planned_traj_yaml", ""))
            executed_path = str(getattr(p, "executed_traj_yaml", ""))
            planned_tcp_path = str(getattr(p, "planned_tcp_yaml", ""))
            executed_tcp_path = str(getattr(p, "executed_tcp_yaml", ""))
        except Exception:
            return

        planned_doc = self._yaml_load_file(planned_path)
        executed_doc = self._yaml_load_file(executed_path)
        planned_tcp_doc = self._yaml_load_file(planned_tcp_path) if planned_tcp_path else {}
        executed_tcp_doc = self._yaml_load_file(executed_tcp_path) if executed_tcp_path else {}

        if not isinstance(planned_doc, dict) or not isinstance(executed_doc, dict):
            return
        if not planned_doc or not executed_doc:
            return

        try:
            rr = RunResult.from_persist_docs(
                planned_traj_doc=planned_doc,
                executed_traj_doc=executed_doc,
                planned_tcp_doc=planned_tcp_doc if isinstance(planned_tcp_doc, dict) else {},
                executed_tcp_doc=executed_tcp_doc if isinstance(executed_tcp_doc, dict) else {},
            )
        except Exception:
            # Fallback: build minimal RunResult using eval from TCP docs (new source of truth)
            planned_ev = self._extract_eval_from_tcp_doc(planned_tcp_doc) if isinstance(planned_tcp_doc, dict) else {}
            executed_ev = self._extract_eval_from_tcp_doc(executed_tcp_doc) if isinstance(executed_tcp_doc, dict) else {}
            rr = RunResult(
                eval={"planned": planned_ev, "executed": executed_ev},
                valid=True,
                invalid_reason="",
            )

        self._set_eval_from_runresult(rr)

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

        self._load_runresult_from_bundle()

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

            # publish ALL 6 topics immediately after load
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

        self._process_thread = ProcessThread(
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

    # ---------------- Process result handling ----------------

    def _finish_process_ui(self) -> None:
        self._process_active = False
        self._active_mode = ""

        pt = self._process_thread
        self._process_thread = None

        # IMPORTANT: explicit cleanup of the last ProcessThread instance
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

    def _on_process_finished_success(self, result_obj: object) -> None:
        self._append_log("=== Process finished ===")

        # 1) decode payload -> RunResult (strict)
        try:
            rr = RunResult.from_process_payload(result_obj)  # type: ignore[arg-type]
        except Exception as e:
            self._append_log(f"ERROR: result invalid (strict): {e}")
            self._finish_process_ui()
            return

        # 2) pick recipe (parameters for FK/Eval)
        recipe_for_eval: Optional[Recipe] = None
        try:
            recipe_run = self.repo.load_for_process(self._recipe_key) if self._recipe_key else None
            recipe_for_eval = (
                recipe_run
                if isinstance(recipe_run, Recipe)
                else (self._recipe if isinstance(self._recipe, Recipe) else None)
            )
        except Exception:
            recipe_for_eval = self._recipe if isinstance(self._recipe, Recipe) else None

        if recipe_for_eval is None:
            self._append_log("ERROR: recipe_for_eval ist None (cannot postprocess).")
            self._finish_process_ui()
            return

        # 3) postprocess (deterministic)
        did_postprocess = False
        try:
            urdf = str(getattr(rr, "urdf_xml", "") or "")
            srdf = str(getattr(rr, "srdf_xml", "") or "")
        except Exception:
            urdf, srdf = "", ""

        if not urdf or not srdf:
            self._append_log("W: Kein URDF/SRDF im RunResult → FK->TCP/Eval übersprungen.")
            self._set_eval_missing_fk("RunResult.urdf_xml/srdf_xml leer.")
        else:
            scene_yaml_path, robot_yaml_path = self._ctx_config_yaml_paths()
            if not scene_yaml_path or not robot_yaml_path:
                self._append_log("W: scene.yaml/robot.yaml nicht resolvable → FK->TCP transform/eval übersprungen.")
                self._set_eval_missing_fk(
                    "Konnte scene.yaml/robot.yaml nicht aus ctx.ros.configs (package://) auflösen.\n"
                    "Prüfe SC_PROJECT_ROOT und ob ROS overlay (ament_index) verfügbar ist."
                )
            else:
                try:
                    rr.postprocess_from_urdf_srdf(
                        urdf_xml=urdf,
                        srdf_xml=srdf,
                        recipe=recipe_for_eval,
                        segment_order=self._SEG_ORDER,
                        gate_valid_on_eval=False,
                        require_tcp=True,
                        tcp_target_frame="substrate",
                        scene_yaml_path=scene_yaml_path,
                        robot_yaml_path=robot_yaml_path,
                    )
                    did_postprocess = True
                    try:
                        pf = (rr.planned_run.get("tcp") or {}).get("frame") if isinstance(rr.planned_run, dict) else None
                        ef = (rr.executed_run.get("tcp") or {}).get("frame") if isinstance(rr.executed_run, dict) else None
                        if pf or ef:
                            self._append_log(f"FK/TCP frames: planned={pf} executed={ef}")
                    except Exception:
                        pass
                except Exception as e:
                    self._append_log(f"ERROR: postprocess failed (FK/Eval): {e}")
                    self._set_eval_missing_fk(str(e))

        # 4) show eval if available
        if did_postprocess:
            self._set_eval_from_runresult(rr)

        # 5) resolve paths
        try:
            p = self.repo.bundle.paths(self._recipe_key)
            planned_path = str(getattr(p, "planned_traj_yaml"))
            executed_path = str(getattr(p, "executed_traj_yaml"))
            planned_tcp_path = str(getattr(p, "planned_tcp_yaml", ""))
            executed_tcp_path = str(getattr(p, "executed_tcp_yaml", ""))
        except Exception as e:
            self._append_log(f"ERROR: bundle.paths() failed: {e}")
            self._finish_process_ui()
            return

        # 6) Save
        ok_pl = self._maybe_overwrite_prompt(title="Run speichern", filename=planned_path)
        ok_ex = self._maybe_overwrite_prompt(title="Run speichern", filename=executed_path)

        ok_tcp = True
        if planned_tcp_path and did_postprocess:
            ok_tcp = ok_tcp and self._maybe_overwrite_prompt(title="Run speichern", filename=planned_tcp_path)
        if executed_tcp_path and did_postprocess:
            ok_tcp = ok_tcp and self._maybe_overwrite_prompt(title="Run speichern", filename=executed_tcp_path)

        if ok_pl and ok_ex and ok_tcp:
            try:
                # IMPORTANT: eval must be stored in TCP YAML, NOT in traj YAML
                docs = rr.build_persist_docs(embed_eval_into_traj=False)

                self._yaml_write_file(planned_path, docs.get("planned_traj") or {})
                self._yaml_write_file(executed_path, docs.get("executed_traj") or {})

                if did_postprocess:
                    if planned_tcp_path:
                        self._yaml_write_file(planned_tcp_path, docs.get("planned_tcp") or {})
                    if executed_tcp_path:
                        self._yaml_write_file(executed_tcp_path, docs.get("executed_tcp") or {})
                    self._append_log("Save: OK -> traj + tcp (+eval/fk_meta)")
                else:
                    self._append_log("Save: OK -> traj (ohne tcp/eval, FK fehlt)")
            except Exception as e:
                self._append_log(f"Save: ERROR: {e}")
        else:
            self._append_log("Save: übersprungen (nicht überschrieben).")

        # 7) Reload + republish
        try:
            if self._recipe_key:
                self._recipe = self.repo.load_for_process(self._recipe_key)
        except Exception:
            pass

        self._clear_layers()
        self._publish_all_available_layers()  # publishes ALL 6 topics

        self._update_recipe_box()
        self._update_info_box()
        self._load_runresult_from_bundle()

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
            empty_ma = MarkerArray()
            empty_pa = PoseArray()
            self._ros_call_first(("spray_set_compiled",), poses=empty_pa, markers=empty_ma)
            self._ros_call_first(("spray_set_planned", "spray_set_traj"), poses=empty_pa, markers=empty_ma)
            self._ros_call_first(("spray_set_executed",), poses=empty_pa, markers=empty_ma)
        except Exception:
            pass

    def _build_all_markers(self, *, recipe: Recipe, frame_id: str) -> Optional[MarkerArray]:
        try:
            return build_marker_array_from_recipe(
                recipe,
                frame_id=frame_id,
                show_draft=True,
                show_planned=False,   # we do TCP markers separately
                show_executed=False,  # we do TCP markers separately
            )
        except Exception as e:
            self._append_log(f"W markers build skipped: {e}")
            return None

    @staticmethod
    def _filter_markers(arr: MarkerArray, *, ns_prefix: str) -> MarkerArray:
        out = MarkerArray()
        if arr is None or not getattr(arr, "markers", None):
            return out
        for m in arr.markers:
            ns = getattr(m, "ns", "") or ""
            if ns.startswith(ns_prefix):
                out.markers.append(m)
        return out

    @staticmethod
    def _pick_frame_from_tcp_docs(planned_doc: Dict[str, Any], executed_doc: Dict[str, Any]) -> str:
        # Prefer explicit tcp frame (this fixes your “poses offset but markers fine” issue).
        try:
            f = str(planned_doc.get("frame") or "").strip() if isinstance(planned_doc, dict) else ""
            if f:
                return f
        except Exception:
            pass
        try:
            f = str(executed_doc.get("frame") or "").strip() if isinstance(executed_doc, dict) else ""
            if f:
                return f
        except Exception:
            pass
        return "substrate"

    def _publish_all_available_layers(self) -> None:
        """
        MUST publish ALL 6 topics (3 layers * poses+markers) after Load and after Process.
        """
        if self._recipe is None or not self._recipe_key:
            return

        r = self._recipe

        # ---- load tcp yaml docs from bundle (canonical) ----
        planned_tcp_doc: Dict[str, Any] = {}
        executed_tcp_doc: Dict[str, Any] = {}
        try:
            p = self.repo.bundle.paths(self._recipe_key)
            planned_tcp_path = str(getattr(p, "planned_tcp_yaml", "") or "")
            executed_tcp_path = str(getattr(p, "executed_tcp_yaml", "") or "")
            if planned_tcp_path:
                planned_tcp_doc = self._yaml_load_file(planned_tcp_path)
            if executed_tcp_path:
                executed_tcp_doc = self._yaml_load_file(executed_tcp_path)
        except Exception as e:
            self._append_log(f"W: tcp yaml load failed: {e}")
            planned_tcp_doc = {}
            executed_tcp_doc = {}

        frame_id = self._pick_frame_from_tcp_docs(planned_tcp_doc, executed_tcp_doc)

        # ---- COMPILED (draft) PoseArray ----
        compiled_pa = PoseArray()
        compiled_pa.header.frame_id = frame_id
        try:
            draft_obj = getattr(r, "draft", None)
            if draft_obj is None:
                draft_obj = getattr(r, "draft_data", None)

            if draft_obj is not None and hasattr(draft_obj, "sides"):
                sides = getattr(draft_obj, "sides", None)
                if isinstance(sides, dict):
                    for _, side in sides.items():
                        pq_list = getattr(side, "poses_quat", None)
                        if isinstance(pq_list, list):
                            for pq in pq_list:
                                try:
                                    # IMPORTANT: draft is already in scene/workspace coords (mm), so only mm->m here.
                                    from model.recipe.recipe_markers import _make_pose_quat_mm  # local import, helper exists

                                    compiled_pa.poses.append(
                                        _make_pose_quat_mm(pq.x, pq.y, pq.z, pq.qx, pq.qy, pq.qz, pq.qw)
                                    )
                                except Exception:
                                    continue
        except Exception as e:
            self._append_log(f"W: compiled PoseArray build failed: {e}")

        # ---- COMPILED (draft) MarkerArray ----
        compiled_ma = MarkerArray()
        try:
            all_ma = self._build_all_markers(recipe=r, frame_id=frame_id)
            compiled_ma = self._filter_markers(all_ma, ns_prefix="draft/") if all_ma else MarkerArray()
            # also force marker frames to match (safety, if legacy builder used other frame)
            for m in compiled_ma.markers:
                m.header.frame_id = frame_id
        except Exception as e:
            self._append_log(f"W: compiled MarkerArray build failed: {e}")

        # ---- PLANNED (TCP) PoseArray + MarkerArray ----
        planned_pa = PoseArray()
        planned_pa.header.frame_id = frame_id
        planned_ma = MarkerArray()
        try:
            if isinstance(planned_tcp_doc, dict) and planned_tcp_doc:
                planned_pa = build_tcp_pose_array_from_tcp_yaml(planned_tcp_doc, default_frame=frame_id)
                planned_ma = build_tcp_marker_array_from_tcp_yaml(
                    planned_tcp_doc,
                    ns_prefix="planned_tcp",
                    default_frame=frame_id,
                    include_text=True,
                )
                # force headers consistent
                planned_pa.header.frame_id = frame_id
                for m in planned_ma.markers:
                    m.header.frame_id = frame_id
        except Exception as e:
            self._append_log(f"W: planned tcp build failed: {e}")

        # ---- EXECUTED (TCP) PoseArray + MarkerArray ----
        executed_pa = PoseArray()
        executed_pa.header.frame_id = frame_id
        executed_ma = MarkerArray()
        try:
            if isinstance(executed_tcp_doc, dict) and executed_tcp_doc:
                executed_pa = build_tcp_pose_array_from_tcp_yaml(executed_tcp_doc, default_frame=frame_id)
                executed_ma = build_tcp_marker_array_from_tcp_yaml(
                    executed_tcp_doc,
                    ns_prefix="executed_tcp",
                    default_frame=frame_id,
                    include_text=True,
                )
                executed_pa.header.frame_id = frame_id
                for m in executed_ma.markers:
                    m.header.frame_id = frame_id
        except Exception as e:
            self._append_log(f"W: executed tcp build failed: {e}")

        # ---- publish ALL 6 topics (even if empty) ----
        try:
            self._ros_call_first(("spray_set_compiled",), poses=compiled_pa, markers=compiled_ma)
        except Exception as e:
            self._append_log(f"W spray_set_compiled failed: {e}")

        try:
            self._ros_call_first(("spray_set_planned", "spray_set_traj"), poses=planned_pa, markers=planned_ma)
        except Exception as e:
            self._append_log(f"W spray_set_planned failed: {e}")

        try:
            self._ros_call_first(("spray_set_executed",), poses=executed_pa, markers=executed_ma)
        except Exception as e:
            self._append_log(f"W spray_set_executed failed: {e}")

        # apply current show/hide toggles (latched Bool topics)
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

        # Start gating is ONLY robot_ready now (no FK gate).
        self.btnValidate.setEnabled((not self._process_active) and ros_ok and has_recipe and self._robot_ready)
        self.btnOptimize.setEnabled((not self._process_active) and ros_ok and has_recipe and self._robot_ready)

        self.btnExecute.setEnabled(
            (not self._process_active)
            and ros_ok
            and has_recipe
            and self._robot_ready
            and self._plc_execute_available()
        )

        self.btnStop.setEnabled(True)