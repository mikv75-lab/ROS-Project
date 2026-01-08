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

from geometry_msgs.msg import PoseArray  # NEW: for clearing layers fully
from visualization_msgs.msg import MarkerArray

from plc.plc_client import PlcClientBase
from ros.bridge.ros_bridge import RosBridge

from model.recipe.recipe import Recipe, Draft, JTBySegment
from model.recipe.recipe_markers import build_marker_array_from_recipe

# NEW: FK + Eval
from model.recipe.traj_fk_builder import TrajFkBuilder, TrajFkConfig
from model.recipe.recipe_eval import RecipeEvaluator

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

    Worker result contract (NEW, strict, traj-only):
      result_obj is dict with exactly:
        {
          "planned_traj":  <JTBySegment v1 YAML-dict> | {},
          "executed_traj": <JTBySegment v1 YAML-dict> | {},
        }

    Persistence contract (SSoT, Option A):
      repo.save_run_artifacts(recipe_id,
                              planned_traj=JTBySegment,
                              executed_traj=JTBySegment,
                              planned_tcp=Draft,
                              executed_tcp=Draft,
                              eval=<dict optional>)
      -> Bundle writes:
          planned_traj.yaml   (with optional top-level eval:)
          executed_traj.yaml  (with optional top-level eval:)
          planned_tcp.yaml
          executed_tcp.yaml
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

        # ---- last run views (raw yaml dicts for UI) ----
        self._last_planned_traj_yaml: Dict[str, Any] = {}
        self._last_executed_traj_yaml: Dict[str, Any] = {}
        self._last_eval: Dict[str, Any] = {}
        self._last_valid: Optional[bool] = None

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
        self.txtPlanned.setPlaceholderText("planned_traj.yaml (JTBySegment yaml dict)")

        self.txtExecuted = QTextEdit(grp_res)
        self.txtExecuted.setReadOnly(True)
        self.txtExecuted.setPlaceholderText("executed_traj.yaml (JTBySegment yaml dict)")

        row_res.addWidget(self.txtPlanned, 1)
        row_res.addWidget(self.txtExecuted, 1)
        vres.addLayout(row_res)

        # eval / valid
        self.lblEval = QLabel("Eval: –", grp_res)
        vres.addWidget(self.lblEval)

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

        # Ensure defaults are published once (compat: traj vs planned)
        self._spray_defaults_publish(compiled=True, planned=True, executed=True)

        self._update_buttons()
        self._update_recipe_box()
        self._update_info_box()
        self._render_run_yaml_views()

    # ---------------------------------------------------------------------
    # SprayPathBox compat helpers (traj vs planned)
    # ---------------------------------------------------------------------

    def _spray_defaults_publish(self, *, compiled: bool, planned: bool, executed: bool) -> None:
        """
        SprayPathBox API changed historically. Support:
          - set_defaults(compiled=..., traj=..., executed=...)
          - set_defaults(compiled=..., planned=..., executed=...)
          - set_defaults(compiled, planned, executed)  (positional legacy)
        """
        sb = getattr(self, "sprayPathBox", None)
        if sb is None:
            return
        fn = getattr(sb, "set_defaults", None)
        if not callable(fn):
            return

        # 1) prefer "planned"
        try:
            fn(compiled=bool(compiled), planned=bool(planned), executed=bool(executed))
            return
        except TypeError:
            pass

        # 2) fallback to "traj"
        try:
            fn(compiled=bool(compiled), traj=bool(planned), executed=bool(executed))
            return
        except TypeError:
            pass

        # 3) last resort: positional
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

    # ------------------------------------------------------------
    # NEW: result extraction (traj-only dict)
    # ------------------------------------------------------------

    @staticmethod
    def _is_jtbysegment_yaml(d: Any) -> bool:
        return isinstance(d, dict) and isinstance(d.get("segments"), dict)

    def _extract_traj_yaml_from_result(self, result_obj: object) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """
        Preferred (NEW): {"planned_traj": {...}, "executed_traj": {...}}

        Backward-compat (OLD): {"planned_run": runpayload, "executed_run": runpayload}
          -> converted later via _runpayload_to_jtbysegment_yaml()
        """
        planned: Dict[str, Any] = {}
        executed: Dict[str, Any] = {}

        if not isinstance(result_obj, dict):
            return planned, executed

        if "planned_traj" in result_obj or "executed_traj" in result_obj:
            a = result_obj.get("planned_traj")
            b = result_obj.get("executed_traj")
            planned = a if isinstance(a, dict) else {}
            executed = b if isinstance(b, dict) else {}
            return planned, executed

        # old payload mode
        a = result_obj.get("planned_run")
        b = result_obj.get("executed_run")
        if isinstance(a, dict) and isinstance(b, dict):
            # will convert downstream
            return a, b

        return planned, executed

    # ------------------------------------------------------------
    # Backward-compat: RunPayload v1 -> JTBySegment YAML dict
    # ------------------------------------------------------------

    def _is_run_payload(self, payload: Any) -> bool:
        return isinstance(payload, dict) and isinstance(payload.get("segments"), dict)

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
        if not self._is_run_payload(run_payload):
            return {}

        seg_out: Dict[str, Any] = {}
        segs = run_payload.get("segments") or {}

        for seg in self._SEG_ORDER:
            evs = segs.get(seg) or []
            if not isinstance(evs, list) or not evs:
                continue
            evs2 = [
                e
                for e in evs
                if isinstance(e, dict) and isinstance(e.get("joint_names"), list) and isinstance(e.get("points"), list)
            ]
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

    # ------------------------------------------------------------
    # NEW: Eval (joint domain) + valid threshold
    # ------------------------------------------------------------

    def _get_validate_threshold(self, recipe: Recipe) -> float:
        # default 80; allow override in params
        try:
            params = getattr(recipe, "parameters", {}) or {}
            v = params.get("validate_min_score", 80.0)
            return float(v)
        except Exception:
            return 80.0

    def _eval_joint_planned_vs_executed(
        self,
        *,
        planned_yaml: Dict[str, Any],
        executed_yaml: Dict[str, Any],
    ) -> Tuple[Dict[str, Any], float]:
        """
        Returns (eval_dict, total_score).
        eval_dict stored in traj yamls under top-level 'eval'.
        """
        ev = RecipeEvaluator()

        # total: concat all segment points in the order
        def _concat_total(jt_by_seg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
            if not isinstance(jt_by_seg, dict):
                return None
            segs = jt_by_seg.get("segments") or {}
            if not isinstance(segs, dict):
                return None

            joint_names: Optional[List[str]] = None
            points: List[Dict[str, Any]] = []

            for seg in self._SEG_ORDER:
                jt = segs.get(seg)
                if not isinstance(jt, dict):
                    continue
                jn = jt.get("joint_names")
                pts = jt.get("points")
                if not isinstance(jn, list) or not isinstance(pts, list) or not jn or not pts:
                    continue
                if joint_names is None:
                    joint_names = [str(x) for x in jn]
                if joint_names != [str(x) for x in jn]:
                    return None
                for p in pts:
                    if isinstance(p, dict):
                        points.append(p)

            if joint_names is None or not points:
                return None
            return {"joint_names": joint_names, "points": points}

        # segmentwise
        by_segment: Dict[str, Any] = {}
        p_segs = (planned_yaml.get("segments") or {}) if isinstance(planned_yaml, dict) else {}
        e_segs = (executed_yaml.get("segments") or {}) if isinstance(executed_yaml, dict) else {}

        if isinstance(p_segs, dict) and isinstance(e_segs, dict):
            for seg in self._SEG_ORDER:
                pj = p_segs.get(seg)
                ej = e_segs.get(seg)
                if not (isinstance(pj, dict) and isinstance(ej, dict)):
                    continue
                res = ev.evaluate_joint_trajectory_dict(ref_joint=pj, test_joint=ej, label=f"joint/{seg}")
                by_segment[seg] = res.to_dict()

        # total
        p_total = _concat_total(planned_yaml)
        e_total = _concat_total(executed_yaml)
        total_res = ev.evaluate_joint_trajectory_dict(ref_joint=p_total, test_joint=e_total, label="joint/total")

        eval_dict = {
            "domain": "joint",
            "total": total_res.to_dict(),
            "by_segment": dict(by_segment),
        }
        return eval_dict, float(total_res.score)

    # ------------------------------------------------------------
    # NEW: FK (JTBySegment -> Draft TCP)
    # ------------------------------------------------------------

    def _get_robot_model_best_effort(self) -> Any:
        """
        Try common places where MoveIt RobotModel is reachable in your stack.
        Returns None if not available.
        """
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

        # last resort: ros.robot / ros.scene etc
        for attr in ("robot_model", "_robot_model"):
            try:
                rm = getattr(ros, attr, None)
                if rm is not None:
                    return rm
            except Exception:
                pass
        return None

    def _fk_tcp_best_effort(
        self,
        *,
        traj: JTBySegment,
        recipe: Recipe,
    ) -> Optional[Draft]:
        robot_model = self._get_robot_model_best_effort()
        if robot_model is None:
            return None

        # FK config: allow overriding ee_link/step in recipe.parameters
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
            draft = TrajFkBuilder.build_tcp_draft(traj, robot_model=robot_model, cfg=cfg, default_side="top")
            return draft
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

        self._last_planned_traj_yaml = {}
        self._last_executed_traj_yaml = {}
        self._last_eval = {}
        self._last_valid = None
        self._render_run_yaml_views()

        # defaults always true + publish to ROS (latched)
        self._show_compiled = True
        self._show_traj = True
        self._show_executed = True
        self._spray_defaults_publish(compiled=True, planned=True, executed=True)

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

            # IMPORTANT: clear + republish via UI-loop (Qt/ROS thread ordering)
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

        self._last_planned_traj_yaml = {}
        self._last_executed_traj_yaml = {}
        self._last_eval = {}
        self._last_valid = None
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
        self.txtPlanned.setPlainText(self._yaml_dump(self._last_planned_traj_yaml))
        self.txtExecuted.setPlainText(self._yaml_dump(self._last_executed_traj_yaml))

        if isinstance(self._last_eval, dict) and self._last_eval:
            try:
                total = (
                    ((self._last_eval.get("total") or {}).get("score"))
                    if isinstance(self._last_eval.get("total"), dict)
                    else None
                )
            except Exception:
                total = None
            self.lblEval.setText(f"Eval: total_score={total} | valid={self._last_valid}")
        else:
            self.lblEval.setText(f"Eval: – | valid={self._last_valid}")

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

        planned_obj, executed_obj = self._extract_traj_yaml_from_result(result_obj)

        # If old runpayload: convert
        if self._is_run_payload(planned_obj) and self._is_run_payload(executed_obj):
            planned_yaml = self._runpayload_to_jtbysegment_yaml(planned_obj)
            executed_yaml = self._runpayload_to_jtbysegment_yaml(executed_obj)
        else:
            planned_yaml = planned_obj if self._is_jtbysegment_yaml(planned_obj) else {}
            executed_yaml = executed_obj if self._is_jtbysegment_yaml(executed_obj) else {}

        self._last_planned_traj_yaml = dict(planned_yaml or {})
        self._last_executed_traj_yaml = dict(executed_yaml or {})
        self._render_run_yaml_views()

        if not planned_yaml or not executed_yaml:
            self._append_log("ERROR: result missing planned_traj/executed_traj (JTBySegment yaml). Persist skipped.")
            self._process_active = False
            self._active_mode = ""
            self._process_thread = None
            self._update_buttons()
            return

        # Build JTBySegment objects
        try:
            planned_jt = JTBySegment.from_yaml_dict(planned_yaml)
        except Exception as e:
            self._append_log(f"ERROR: planned JTBySegment build failed: {e}")
            planned_jt = None

        try:
            executed_jt = JTBySegment.from_yaml_dict(executed_yaml)
        except Exception as e:
            self._append_log(f"ERROR: executed JTBySegment build failed: {e}")
            executed_jt = None

        if planned_jt is None or executed_jt is None:
            self._append_log("ERROR: Persist skipped (planned/executed JTBySegment missing).")
            self._process_active = False
            self._active_mode = ""
            self._process_thread = None
            self._update_buttons()
            return

        # FK TCP (best-effort)
        recipe_run = None
        try:
            recipe_run = self.repo.load_for_process(self._recipe_key) if self._recipe_key else None
        except Exception:
            recipe_run = None

        recipe_for_eval = (
            recipe_run if isinstance(recipe_run, Recipe) else (self._recipe if isinstance(self._recipe, Recipe) else None)
        )

        planned_tcp: Optional[Draft] = None
        executed_tcp: Optional[Draft] = None
        if recipe_for_eval is not None:
            planned_tcp = self._fk_tcp_best_effort(traj=planned_jt, recipe=recipe_for_eval)
            executed_tcp = self._fk_tcp_best_effort(traj=executed_jt, recipe=recipe_for_eval)

        # Eval + valid
        eval_dict: Dict[str, Any] = {}
        valid = False
        if recipe_for_eval is not None:
            eval_dict, total_score = self._eval_joint_planned_vs_executed(
                planned_yaml=planned_yaml,
                executed_yaml=executed_yaml,
            )
            thr = self._get_validate_threshold(recipe_for_eval)
            valid = bool(total_score >= thr)
            eval_dict["threshold"] = float(thr)
            eval_dict["valid"] = bool(valid)

        self._last_eval = dict(eval_dict or {})
        self._last_valid = bool(valid)
        self._render_run_yaml_views()

        # Prompt overwrite paths
        try:
            p = self.repo.bundle.paths(self._recipe_key)  # rid-only key
            planned_path = str(getattr(p, "planned_traj_yaml"))
            executed_path = str(getattr(p, "executed_traj_yaml"))
            planned_tcp_path = str(getattr(p, "planned_tcp_yaml", ""))
            executed_tcp_path = str(getattr(p, "executed_tcp_yaml", ""))
        except Exception:
            planned_path = ""
            executed_path = ""
            planned_tcp_path = ""
            executed_tcp_path = ""

        # Only save if valid
        if not valid:
            self._append_log("Save: übersprungen (RunResult invalid).")
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
                    # Preferred: repo supports eval injection
                    try:
                        self.repo.save_run_artifacts(
                            self._recipe_key,
                            planned_traj=planned_jt,
                            executed_traj=executed_jt,
                            planned_tcp=planned_tcp,
                            executed_tcp=executed_tcp,
                            eval=eval_dict if eval_dict else None,
                        )
                    except TypeError:
                        # fallback if repo hasn't been updated yet
                        self.repo.save_run_artifacts(
                            self._recipe_key,
                            planned_traj=planned_jt,
                            executed_traj=executed_jt,
                            planned_tcp=planned_tcp,
                            executed_tcp=executed_tcp,
                        )

                    self._append_log("Save: OK -> planned/executed traj (+tcp best-effort) (+eval if supported)")
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

        # republish markers (draft lines + planned/executed text markers)
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
        """
        Clear BOTH markers and poses so SprayPathBridge cache node definitely resets.
        """
        try:
            empty_ma = MarkerArray()
            empty_pa = PoseArray()
            self._ros_call_first(("spray_set_compiled",), poses=empty_pa, markers=empty_ma)
            self._ros_call_first(("spray_set_planned", "spray_set_traj"), poses=empty_pa, markers=empty_ma)
            self._ros_call_first(("spray_set_executed",), poses=empty_pa, markers=empty_ma)
        except Exception:
            pass

    def _build_all_markers(self, *, recipe: Recipe, frame_id: str = "scene") -> Optional[MarkerArray]:
        """
        NEW API (2026-01):
          build_marker_array_from_recipe(recipe, frame_id, show_draft/show_planned/show_executed)
        Returns a single MarkerArray with namespaces:
          - draft/<side>
          - planned_traj
          - executed_traj
        """
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
    def _filter_markers(arr: MarkerArray, *, ns_prefix: Optional[str] = None, ns_exact: Optional[str] = None) -> MarkerArray:
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
        """
        Publish layers to SprayPathBridge cache node.

        With current recipe_markers.py:
          - compiled == draft paths -> ns "draft/<side>" (LINE_STRIP)
          - planned/executed == summary TEXT markers -> ns "planned_traj"/"executed_traj"
        """
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
