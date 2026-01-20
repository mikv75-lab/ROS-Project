# -*- coding: utf-8 -*-
# File: src/tabs/process/recipe_panel/recipe_panel.py
from __future__ import annotations

import logging
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
)

from model.recipe.recipe import Recipe
from model.recipe.recipe_run_result import RunResult

# NOTE: recipe_markers moved under spray_paths (spraypaths package)
from model.spray_paths import recipe_markers

from .spray_path_box import SprayPathBox
from widgets.info_groupbox import InfoGroupBox

_LOG = logging.getLogger("tabs.process.recipe_panel")


class RecipePanel(QWidget):
    """Anzeige und Management von Rezepten und deren Ausführungsergebnissen (Strict V2)."""

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
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        # ============================================================
        # Active Recipe
        # ============================================================
        self.RecipeGRP = QGroupBox("Active Recipe", self)
        vrec = QVBoxLayout(self.RecipeGRP)
        vrec.setContentsMargins(8, 8, 8, 8)
        vrec.setSpacing(8)

        top_row = QWidget(self.RecipeGRP)
        htop = QHBoxLayout(top_row)
        htop.setContentsMargins(0, 0, 0, 0)
        htop.setSpacing(10)

        self.btnLoad = QPushButton("Load Recipe", top_row)
        self.lblRecipeName = QLabel("Recipe: –", top_row)
        self.lblRecipeName.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

        htop.addWidget(self.btnLoad, 0)
        htop.addWidget(self.lblRecipeName, 1)
        vrec.addWidget(top_row, 0)

        self.grpRecipeParams = QGroupBox("Recipe Parameters", self.RecipeGRP)
        vparams = QVBoxLayout(self.grpRecipeParams)
        vparams.setContentsMargins(8, 8, 8, 8)
        vparams.setSpacing(6)

        self.txtRecipeParams = QTextEdit(self.grpRecipeParams)
        self.txtRecipeParams.setReadOnly(True)
        self.txtRecipeParams.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        vparams.addWidget(self.txtRecipeParams, 1)

        vrec.addWidget(self.grpRecipeParams, 1)
        root.addWidget(self.RecipeGRP, 2)

        # ============================================================
        # Middle: Info (left) + SprayPaths (right)
        # ============================================================
        mid = QWidget(self)
        hmid = QHBoxLayout(mid)
        hmid.setContentsMargins(0, 0, 0, 0)
        hmid.setSpacing(8)

        self.infoBox = InfoGroupBox(mid, title="Info")
        sp = self.infoBox.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Preferred)
        self.infoBox.setSizePolicy(sp)
        self.infoBox.setMinimumWidth(320)
        self.infoBox.setMaximumWidth(520)

        self.sprayPathBox = SprayPathBox(ros=self.ros, parent=mid)
        sp2 = self.sprayPathBox.sizePolicy()
        sp2.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        self.sprayPathBox.setSizePolicy(sp2)

        hmid.addWidget(self.infoBox, 0)
        hmid.addWidget(self.sprayPathBox, 1)
        root.addWidget(mid, 0)

        # ============================================================
        # Bottom: STORED vs CURRENT (each in a named GroupBox)
        # ============================================================
        bottom = QWidget(self)
        hbot = QHBoxLayout(bottom)
        hbot.setContentsMargins(0, 0, 0, 0)
        hbot.setSpacing(8)

        self.grpStored = QGroupBox("Stored Eval (Disk)", bottom)
        vstored = QVBoxLayout(self.grpStored)
        vstored.setContentsMargins(8, 8, 8, 8)
        self.txtStored = QTextEdit(self.grpStored)
        self.txtStored.setReadOnly(True)
        vstored.addWidget(self.txtStored, 1)

        self.grpCurrent = QGroupBox("Current Run (Live)", bottom)
        vcur = QVBoxLayout(self.grpCurrent)
        vcur.setContentsMargins(8, 8, 8, 8)
        self.txtNewRun = QTextEdit(self.grpCurrent)
        self.txtNewRun.setReadOnly(True)
        vcur.addWidget(self.txtNewRun, 1)

        hbot.addWidget(self.grpStored, 1)
        hbot.addWidget(self.grpCurrent, 1)
        root.addWidget(bottom, 2)

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

        self.txtRecipeParams.setPlainText(self._format_recipe_params(model))

        # STORED ist Disk-SSoT: beim Load immer refresh von Disk
        self._refresh_stored_from_disk(key)

        self._update_infobox_from_recipe(model)
        self.txtNewRun.clear()

        # spraypaths on recipe load (compiled + stored ghosts)
        try:
            self._republish_spraypaths_for_key(key, model)
        except Exception as e:
            _LOG.warning("RecipePanel: spraypath republish failed: %s", e)

        self.sig_recipe_selected.emit(key, model)

    def _format_recipe_params(self, model: Recipe) -> str:
        try:
            d = model.to_params_dict()
        except Exception:
            d = {"repr": repr(model)}

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
        STRICT: must match ProcessPanel.sig_run_finished(str, object, object).

        ProcessPanel already:
          - rr = RunResult.from_process_payload(payload)
          - rr.postprocess(...)
          - persisted YAML (repo.bundle.paths)
        RecipePanel responsibilities:
          - display rr
          - refresh STORED from disk
          - republish overlays
        """
        key = str(key or "").strip()
        self.txtNewRun.setPlaceholderText("")

        if not key:
            self.txtNewRun.setPlainText("Run finished, but key is empty.")
            return

        if not isinstance(rr_obj, RunResult):
            self.txtNewRun.setPlainText(f"Run finished for {key}, but rr is not RunResult.")
            _LOG.warning("RecipePanel.on_run_finished: rr_obj invalid type=%s", type(rr_obj).__name__)
            return

        rr: RunResult = rr_obj

        # Show current run in TABLE format (topic | planned | executed)
        try:
            self.txtNewRun.setPlainText(self._format_eval_table_for_run(rr, title="=== CURRENT RUN (Live) ==="))
        except Exception as e:
            self.txtNewRun.setPlainText(f"Run finished for {key}, but table render failed: {e}")

        # Refresh disk SSoT + republish stored overlays
        try:
            self._refresh_stored_from_disk(key)
        except Exception as e:
            _LOG.warning("RecipePanel: refresh stored after run failed: %s", e)

        # Optional: republish NEW overlays immediately from rr TCP docs
        try:
            self._republish_newrun_overlays_from_rr(key, rr)
        except Exception as e:
            _LOG.warning("RecipePanel: republish newrun overlays failed: %s", e)

    @QtCore.pyqtSlot(str, str)
    def on_run_error(self, key: str, message: str) -> None:
        self.txtNewRun.setPlaceholderText("")
        self.txtNewRun.setPlainText(f"ERROR for {key}:\n{message}")
        _LOG.error("RecipePanel: Run error for %s: %s", key, message)

    # --- Interne Helfer ---

    def _refresh_stored_from_disk(self, key: str) -> None:
        key = str(key or "").strip()
        if not key:
            raise ValueError("_refresh_stored_from_disk: empty key")

        fresh = self.repo.load_for_process(key)
        self._recipe = fresh
        self._recipe_key = key
        self._update_stored_view(fresh)

        try:
            self._republish_spraypaths_for_key(key, fresh)
        except Exception as e2:
            _LOG.warning("RecipePanel: spraypath republish after disk refresh failed: %s", e2)

    def _ros_has(self, name: str) -> bool:
        return bool(self.ros is not None and hasattr(self.ros, name) and callable(getattr(self.ros, name)))

    # ============================================================
    # Table rendering (topic | planned | executed)
    # ============================================================

    @staticmethod
    def _count_tcp_poses(doc: Any) -> int:
        if not isinstance(doc, dict):
            return 0
        sides = doc.get("sides")
        if not isinstance(sides, dict):
            return 0
        n = 0
        for _, s in sides.items():
            if isinstance(s, dict):
                pq = s.get("poses_quat")
                if isinstance(pq, list):
                    n += len(pq)
        return int(n)

    @staticmethod
    def _count_traj_points(doc: Any) -> int:
        """
        Best-effort count for JTBySegment YAML.
        Expected: {"segments": {seg: {"points": [...]}}} or {"segments": {seg: {"joint_trajectory": {"points": [...]}}}}
        """
        if not isinstance(doc, dict):
            return 0
        segs = doc.get("segments")
        if not isinstance(segs, dict):
            return 0
        total = 0
        for _, s in segs.items():
            if not isinstance(s, dict):
                continue
            pts = s.get("points")
            if isinstance(pts, list):
                total += len(pts)
                continue
            jt = s.get("joint_trajectory")
            if isinstance(jt, dict):
                pts2 = jt.get("points")
                if isinstance(pts2, list):
                    total += len(pts2)
        return int(total)

    @staticmethod
    def _fmt_num(x: Any) -> str:
        if x is None:
            return "–"
        if isinstance(x, bool):
            return "true" if x else "false"
        if isinstance(x, (int, float)):
            try:
                return f"{float(x):.3f}"
            except Exception:
                return str(x)
        return str(x)

    @staticmethod
    def _as_text(x: Any) -> str:
        s = str(x or "").strip()
        return s if s else "–"

    def _pad_table(self, rows: List[Tuple[str, str, str]]) -> str:
        """
        Render as monospaced aligned text. QTextEdit uses default font; alignment still helps.
        """
        if not rows:
            return ""

        c0 = max(len(r[0]) for r in rows)
        c1 = max(len(r[1]) for r in rows)
        c2 = max(len(r[2]) for r in rows)

        def line(a: str, b: str, c: str) -> str:
            return f"{a.ljust(c0)} | {b.ljust(c1)} | {c.ljust(c2)}"

        out: List[str] = []
        out.append(line("topic", "planned", "executed"))
        out.append(line("-" * 5, "-" * 6, "-" * 8))
        for a, b, c in rows:
            out.append(line(a, b, c))
        return "\n".join(out)

    def _extract_eval_pair(self, recipe: Recipe) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """
        STRICT-ish:
          - Primary: recipe.planned_tcp.eval and recipe.executed_tcp.eval (stored on disk)
          - Fallback: empty dicts
        """
        pl: Dict[str, Any] = {}
        ex: Dict[str, Any] = {}
        planned_obj = getattr(recipe, "planned_tcp", None)
        executed_obj = getattr(recipe, "executed_tcp", None)

        if planned_obj is not None:
            v = getattr(planned_obj, "eval", None)
            if isinstance(v, dict):
                pl = v
        if executed_obj is not None:
            v = getattr(executed_obj, "eval", None)
            if isinstance(v, dict):
                ex = v
        return pl, ex

    def _format_eval_table_for_run(self, rr: RunResult, *, title: str) -> str:
        """
        Build a compact overview for a RunResult:
          - tcp counts + frames
          - fk meta
          - eval summary + score/threshold/valid
          - comparison rows from eval (if present): Score / mean/max etc.
        """
        pl_tcp = rr.planned_run.get("tcp") if isinstance(rr.planned_run, dict) else {}
        ex_tcp = rr.executed_run.get("tcp") if isinstance(rr.executed_run, dict) else {}
        pl_traj = rr.planned_run.get("traj") if isinstance(rr.planned_run, dict) else {}
        ex_traj = rr.executed_run.get("traj") if isinstance(rr.executed_run, dict) else {}

        rows: List[Tuple[str, str, str]] = []

        # Trajectory point counts (best-effort)
        rows.append(("traj_points", str(self._count_traj_points(pl_traj)), str(self._count_traj_points(ex_traj))))

        # TCP pose counts + frame
        rows.append(("tcp_poses", str(self._count_tcp_poses(pl_tcp)), str(self._count_tcp_poses(ex_tcp))))
        rows.append(
            ("tcp_frame", self._as_text(pl_tcp.get("frame") if isinstance(pl_tcp, dict) else ""), self._as_text(ex_tcp.get("frame") if isinstance(ex_tcp, dict) else ""))
        )

        # Eval summary
        ev = rr.eval if isinstance(rr.eval, dict) else {}
        rows.append(("eval_valid", self._as_text(ev.get("valid")), self._as_text(ev.get("valid"))))
        rows.append(("eval_thr", self._fmt_num(ev.get("threshold")), self._fmt_num(ev.get("threshold"))))

        # Prefer ev["score"] then ev["total"]["score"]
        total_score = ev.get("score")
        if total_score is None and isinstance(ev.get("total"), dict):
            total_score = ev["total"].get("score")
        rows.append(("eval_score", self._fmt_num(total_score), self._fmt_num(total_score)))

        # Per-mode scores (if v4 evaluator returned planned/executed)
        p = ev.get("planned") if isinstance(ev.get("planned"), dict) else {}
        e = ev.get("executed") if isinstance(ev.get("executed"), dict) else {}
        if p or e:
            rows.append(("planned_score", self._fmt_num(p.get("score")), "–"))
            rows.append(("executed_score", "–", self._fmt_num(e.get("score"))))

        # Comparison table rows (Score / mean / max / rot) if provided
        comp = ev.get("comparison")
        if isinstance(comp, list) and comp:
            for r in comp:
                if not isinstance(r, dict):
                    continue
                metric = self._as_text(r.get("metric"))
                rows.append((f"metric:{metric}", self._as_text(r.get("planned")), self._as_text(r.get("executed"))))

        # FK meta (same for both columns; show once mirrored)
        fk = rr.fk_meta if isinstance(rr.fk_meta, dict) else {}
        if fk:
            rows.append(("fk.base_link", self._as_text(fk.get("base_link")), self._as_text(fk.get("base_link"))))
            rows.append(("fk.ee_link", self._as_text(fk.get("ee_link")), self._as_text(fk.get("ee_link"))))
            rows.append(("fk.step_mm", self._fmt_num(fk.get("step_mm")), self._fmt_num(fk.get("step_mm"))))
            segs = fk.get("segments_included") or fk.get("segment_order") or []
            rows.append(("fk.segments", str(len(segs)) if isinstance(segs, list) else self._as_text(segs), str(len(segs)) if isinstance(segs, list) else self._as_text(segs)))

        txt = []
        txt.append(str(title or "").strip())
        txt.append(self._pad_table(rows))
        inv = self._as_text(ev.get("invalid_reason"))
        if inv and inv != "–":
            txt.append("")
            txt.append(f"reason: {inv}")
        return "\n".join(txt).strip()

    def _format_eval_table_for_recipe_disk(self, recipe: Recipe, *, title: str) -> str:
        """
        Disk view: render eval using recipe.planned_tcp.eval and recipe.executed_tcp.eval (per-mode).
        Additionally show stored TCP counts + frame if available as dict-like or Draft-like.
        """
        rows: List[Tuple[str, str, str]] = []

        # Try to extract stored tcp docs if they exist (Draft or dict)
        planned_obj = getattr(recipe, "planned_tcp", None)
        executed_obj = getattr(recipe, "executed_tcp", None)

        # TCP counts
        def _count_from_obj(o: Any) -> int:
            if o is None:
                return 0
            if isinstance(o, dict):
                return self._count_tcp_poses(o)
            # Draft-like: o.sides[side].poses_quat
            sides = getattr(o, "sides", None)
            if isinstance(sides, dict):
                n = 0
                for _, s in sides.items():
                    if isinstance(s, dict):
                        pq = s.get("poses_quat")
                        if isinstance(pq, list):
                            n += len(pq)
                return int(n)
            return 0

        def _frame_from_obj(o: Any) -> str:
            if o is None:
                return "–"
            if isinstance(o, dict):
                return self._as_text(o.get("frame"))
            fr = getattr(o, "frame", None)
            return self._as_text(fr)

        rows.append(("tcp_poses", str(_count_from_obj(planned_obj)), str(_count_from_obj(executed_obj))))
        rows.append(("tcp_frame", _frame_from_obj(planned_obj), _frame_from_obj(executed_obj)))

        # Eval dicts (stored separately per TCP)
        pl_eval, ex_eval = self._extract_eval_pair(recipe)

        rows.append(("eval_valid", self._as_text(pl_eval.get("valid")), self._as_text(ex_eval.get("valid"))))
        rows.append(("eval_thr", self._fmt_num(pl_eval.get("threshold")), self._fmt_num(ex_eval.get("threshold"))))

        # Prefer per-mode score fields
        def _score_of(ev: Dict[str, Any]) -> Any:
            s = ev.get("score")
            if s is None and isinstance(ev.get("total"), dict):
                s = ev["total"].get("score")
            return s

        rows.append(("eval_score", self._fmt_num(_score_of(pl_eval)), self._fmt_num(_score_of(ex_eval))))

        # If each eval stored a comparison table, try to show it; else nothing
        # (Typically you store full v4 dict into both, so comparison exists.)
        comp = pl_eval.get("comparison")
        if isinstance(comp, list) and comp:
            for r in comp:
                if not isinstance(r, dict):
                    continue
                metric = self._as_text(r.get("metric"))
                rows.append((f"metric:{metric}", self._as_text(r.get("planned")), self._as_text(r.get("executed"))))

        txt = []
        txt.append(str(title or "").strip())
        txt.append(self._pad_table(rows))

        # show reasons if present
        pl_reason = self._as_text(pl_eval.get("invalid_reason"))
        ex_reason = self._as_text(ex_eval.get("invalid_reason"))
        if (pl_reason and pl_reason != "–") or (ex_reason and ex_reason != "–"):
            txt.append("")
            txt.append(f"planned_reason: {pl_reason}")
            txt.append(f"executed_reason: {ex_reason}")

        return "\n".join(txt).strip()

    # ============================================================
    # Publishing / overlays
    # ============================================================

    def _republish_spraypaths_for_key(self, key: str, recipe_model: Recipe) -> None:
        """
        Republishes spray path layers to RViz:

        - Compiled: from current recipe model (draft)
        - Planned/Executed STORED: from disk (repo.load_for_process) as ghost markers

        Bridge remains unchanged; we capability-detect:
          - prefer ros.spray_set_planned(...stored_markers=...)
          - else fallback to legacy ros.spray_set_traj(markers=...)
        """
        key = str(key or "").strip()
        if not key or self.ros is None:
            return

        if self._ros_has("spray_clear"):
            try:
                self.ros.spray_clear()
            except Exception:
                pass

        # 1) Compiled (Draft)
        try:
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
                if self._ros_has("spray_set_compiled"):
                    self.ros.spray_set_compiled(poses=pa if has_poses else None, markers=ma if has_markers else None)
        except Exception as e:
            _LOG.warning("RecipePanel: publish compiled failed: %s", e)

        # 2) STORED (Disk) ghosts
        recipe_disk = self.repo.load_for_process(key)

        # Planned stored
        try:
            planned_obj = getattr(recipe_disk, "planned_tcp", None)
            if planned_obj:
                _, ma = recipe_markers.build_tcp_pose_and_markers(
                    planned_obj,
                    ns_prefix="planned_tcp/stored",
                    mid_start=33000,
                    default_frame="substrate",
                    clear_legacy=True,
                    line_width_m=0.0005,
                    rgba_line=(0.7, 0.7, 0.7, 0.4),
                    round_style="none",
                )
                if getattr(ma, "markers", None) and len(ma.markers) > 0:
                    if self._ros_has("spray_set_planned"):
                        self.ros.spray_set_planned(stored_markers=ma)
                    elif self._ros_has("spray_set_traj"):
                        self.ros.spray_set_traj(markers=ma)
        except Exception as e:
            _LOG.warning("RecipePanel: publish planned stored failed: %s", e)

        # Executed stored
        try:
            executed_obj = getattr(recipe_disk, "executed_tcp", None)
            if executed_obj:
                _, ma = recipe_markers.build_tcp_pose_and_markers(
                    executed_obj,
                    ns_prefix="executed_tcp/stored",
                    mid_start=43000,
                    default_frame="substrate",
                    clear_legacy=True,
                    line_width_m=0.0005,
                    rgba_line=(0.7, 0.7, 0.7, 0.4),
                    round_style="none",
                )
                if getattr(ma, "markers", None) and len(ma.markers) > 0:
                    if self._ros_has("spray_set_executed"):
                        self.ros.spray_set_executed(stored_markers=ma)
        except Exception as e:
            _LOG.warning("RecipePanel: publish executed stored failed: %s", e)

    def _republish_newrun_overlays_from_rr(self, key: str, rr: RunResult) -> None:
        """
        Optional NEW overlays from rr TCP docs (already postprocessed in ProcessPanel).
        """
        if self.ros is None:
            return

        planned_tcp_doc = rr.planned_run.get("tcp") if isinstance(rr.planned_run, dict) else None
        executed_tcp_doc = rr.executed_run.get("tcp") if isinstance(rr.executed_run, dict) else None

        # Planned NEW
        try:
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
                    if self._ros_has("spray_set_planned"):
                        self.ros.spray_set_planned(new_markers=ma)
                    elif self._ros_has("spray_set_traj"):
                        self.ros.spray_set_traj(markers=ma)
        except Exception as e:
            _LOG.warning("RecipePanel: publish planned newrun failed: %s", e)

        # Executed NEW
        try:
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
                    if self._ros_has("spray_set_executed"):
                        self.ros.spray_set_executed(new_markers=ma)
        except Exception as e:
            _LOG.warning("RecipePanel: publish executed newrun failed: %s", e)

    def _update_infobox_from_recipe(self, recipe: Recipe) -> None:
        pts = None
        for attr in ("compiled_draft", "draft", "compiled"):
            d = getattr(recipe, attr, None)
            if d is None:
                continue
            p = getattr(d, "points_mm", None)
            if p is None:
                continue
            try:
                arr = np.asarray(p, dtype=float).reshape(-1, 3)
                if arr.size > 0:
                    pts = arr
                    break
            except Exception:
                continue
        self.infoBox.update_from_recipe(recipe, points=pts)

    def _update_stored_view(self, recipe: Recipe) -> None:
        """
        STORED view shows a table: topic | planned | executed.
        Uses per-mode eval stored on disk (recipe.planned_tcp.eval + recipe.executed_tcp.eval).
        """
        try:
            self.txtStored.setPlainText(self._format_eval_table_for_recipe_disk(recipe, title="=== STORED EVAL (Disk) ==="))
        except Exception as e:
            self.txtStored.setPlainText(f"=== STORED EVAL (Disk) ===\n(render failed: {e})")

    def _on_load_clicked(self) -> None:
        keys = self.repo.list_recipes()
        choice, ok = QInputDialog.getItem(self, "Load", "Select Recipe:", keys, 0, False)
        if ok and choice:
            model = self.repo.load_for_process(choice)
            self.set_recipe(choice, model)
