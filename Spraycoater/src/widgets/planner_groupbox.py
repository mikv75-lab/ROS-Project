# -*- coding: utf-8 -*-
# File: app/widgets/planner_groupbox.py
from __future__ import annotations

from typing import Any, Dict, Optional, Tuple, List

from PyQt6 import QtWidgets

try:
    from model.recipe.recipe_store import RecipeStore  # type: ignore
except Exception:  # pragma: no cover
    RecipeStore = Any  # type: ignore


def _intish(*vals: Any) -> bool:
    """True, wenn alle Werte integer-kompatibel sind."""
    try:
        return all(float(v).is_integer() for v in vals)
    except Exception:
        return False


def _safe_dict(x: Any) -> Dict[str, Any]:
    return dict(x or {}) if isinstance(x, dict) else {}


class PlannerGroupBox(QtWidgets.QGroupBox):
    """
    Planner-Box (SSoT) basierend auf planner_catalog.yaml.

    Model-Schema (was in recipe.planner[...] gespeichert wird):
      {
        "role": "validate_move" | "validate_path" | "refine" | "optimize" | "service" | ...,
        "pipeline": "ompl" | "pilz" | "chomp" | "stomp" | "post",
        "planner_id": "...",
        "params": { "<param_key>": <value>, ... }
      }

    STRICT:
      - Keine Hardcode-Maps (z.B. _ROLE_PIPELINES) – alles wird aus dem YAML abgeleitet.
      - Verfügbare Pipelines = pipelines, die mindestens einen für die Rolle erlaubten planner_id enthalten.
      - Verfügbare Planner je Pipeline = Schnittmenge aus (pipelines[pipeline].keys) und roles[role].
      - Params-Schema = params_ref Profile + params_override (nur aus YAML).
    """

    _ROLE_MAP = {
        "move": "validate_move",
        "path": "validate_path",
        "validate_move": "validate_move",
        "validate_path": "validate_path",
        "refine": "refine",
        "optimize": "optimize",
        "service": "service",
    }

    def __init__(
        self,
        parent: Optional[QtWidgets.QWidget] = None,
        *,
        title: str = "Planner",
        role: str = "move",
        store: Optional["RecipeStore"] = None,
        catalog: Optional[Dict[str, Any]] = None,
    ) -> None:
        super().__init__(title, parent)

        self.store = store
        self.catalog = _safe_dict(catalog) if catalog is not None else _safe_dict(getattr(store, "planner_catalog", None))
        if not self.catalog:
            raise ValueError("PlannerGroupBox: planner_catalog ist leer (store.planner_catalog fehlt?).")

        self.role_raw = str(role or "").strip()
        self.role = self._ROLE_MAP.get(self.role_raw, self.role_raw)

        # UI state
        self._param_widgets: Dict[str, Tuple[QtWidgets.QWidget, str, Dict[str, Any]]] = {}

        self._build_ui()
        self._apply_default_selection()

    # ------------------------------------------------------------------ Catalog helpers

    def _root(self) -> Dict[str, Any]:
        # akzeptiert entweder root==planner_catalog oder root=={planner_catalog:{...}}
        return _safe_dict(self.catalog.get("planner_catalog", self.catalog))

    def _roles(self) -> Dict[str, Any]:
        return _safe_dict(self._root().get("roles"))

    def _param_profiles(self) -> Dict[str, Any]:
        return _safe_dict(self._root().get("param_profiles"))

    def _pipelines(self) -> Dict[str, Any]:
        return _safe_dict(self._root().get("pipelines"))

    def _allowed_planners_for_role(self) -> List[str]:
        roles = self._roles()
        allowed = roles.get(self.role)
        if isinstance(allowed, list):
            return [str(x) for x in allowed if str(x).strip()]
        # fallback: wenn Rolle nicht existiert -> alles erlauben
        return []

    def _planners_for_pipeline(self, pipeline: str) -> Dict[str, Any]:
        pipes = self._pipelines()
        return _safe_dict(pipes.get(str(pipeline)))

    def _planner_spec(self, pipeline: str, planner_id: str) -> Dict[str, Any]:
        return _safe_dict(self._planners_for_pipeline(pipeline).get(str(planner_id)))

    def _available_pipelines_for_role(self) -> List[str]:
        """
        Datengetrieben: welche Pipelines bieten mindestens einen Planner,
        der für die Rolle erlaubt ist?
        """
        pipes = self._pipelines()
        if not pipes:
            return []

        allowed = set(self._allowed_planners_for_role())
        out: List[str] = []

        for pname, pmap in pipes.items():
            if not isinstance(pmap, dict) or not pmap:
                continue
            if not allowed:
                # role unbekannt -> alles anzeigen
                out.append(str(pname))
                continue
            if any(str(pid) in allowed for pid in pmap.keys()):
                out.append(str(pname))

        # stabile Sortierung (alphabetisch)
        out = sorted(out)
        return out

    # ------------------------------------------------------------------ UI

    def _build_ui(self) -> None:
        self.setFlat(False)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QtWidgets.QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QtWidgets.QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

        self.form = QtWidgets.QFormLayout(self)
        self.form.setContentsMargins(8, 6, 8, 6)
        self.form.setHorizontalSpacing(8)
        self.form.setVerticalSpacing(4)
        self.form.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow)

        self.pipelineCombo = QtWidgets.QComboBox(self)
        self.pipelineCombo.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToContents)
        self.form.addRow("Pipeline", self.pipelineCombo)

        self.plannerCombo = QtWidgets.QComboBox(self)
        self.plannerCombo.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToContents)
        self.form.addRow("Planner", self.plannerCombo)

        self.paramsHost = QtWidgets.QWidget(self)
        self.paramsForm = QtWidgets.QFormLayout(self.paramsHost)
        self.paramsForm.setContentsMargins(0, 0, 0, 0)
        self.paramsForm.setHorizontalSpacing(8)
        self.paramsForm.setVerticalSpacing(3)
        self.paramsForm.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow)
        self.form.addRow("Params", self.paramsHost)

        self.btnDefaults = QtWidgets.QPushButton("Reset defaults", self)
        self.form.addRow("", self.btnDefaults)

        self.pipelineCombo.currentTextChanged.connect(self._on_pipeline_changed)
        self.plannerCombo.currentTextChanged.connect(self._on_planner_changed)
        self.btnDefaults.clicked.connect(self._apply_defaults_for_current)

    def _apply_default_selection(self) -> None:
        available_pipes = self._available_pipelines_for_role()

        # Falls Rolle unbekannt oder leer -> alle Pipelines anzeigen
        if not available_pipes:
            pipes = self._pipelines()
            available_pipes = sorted(list(pipes.keys())) if pipes else []

        self.pipelineCombo.blockSignals(True)
        self.pipelineCombo.clear()
        for p in available_pipes:
            self.pipelineCombo.addItem(p)
        self.pipelineCombo.blockSignals(False)

        # Datengetrieben "lock", wenn nur eine Pipeline sinnvoll ist (z.B. optimize->post)
        self.pipelineCombo.setEnabled(self.pipelineCombo.count() > 1)

        # initial fill planner list + params
        self._on_pipeline_changed(self.pipelineCombo.currentText())

    # ------------------------------------------------------------------ Dynamic content

    def _on_pipeline_changed(self, pipeline: str) -> None:
        pipeline = str(pipeline or "").strip()
        pmap = self._planners_for_pipeline(pipeline)

        allowed = set(self._allowed_planners_for_role())
        items = sorted(list(pmap.keys()))
        if allowed:
            items = [x for x in items if x in allowed]

        self.plannerCombo.blockSignals(True)
        self.plannerCombo.clear()
        for pid in items:
            self.plannerCombo.addItem(pid)
        self.plannerCombo.blockSignals(False)

        if self.plannerCombo.count() > 0:
            self.plannerCombo.setCurrentIndex(0)
        self._on_planner_changed(self.plannerCombo.currentText())

    def _on_planner_changed(self, planner_id: str) -> None:
        self._rebuild_params_ui()
        self._apply_defaults_for_current()

    def _merged_param_schema(self, pipeline: str, planner_id: str) -> Dict[str, Any]:
        """
        Merge aus:
          - param_profiles aus params_ref[]
          - params_override (nur default override + zusätzliche keys)
        """
        spec = self._planner_spec(pipeline, planner_id)
        profiles = self._param_profiles()

        merged: Dict[str, Any] = {}

        refs = spec.get("params_ref") or []
        if isinstance(refs, (list, tuple)):
            for ref in refs:
                rname = str(ref)
                prof = profiles.get(rname)
                if isinstance(prof, dict):
                    for k, v in prof.items():
                        if isinstance(v, dict):
                            merged[str(k)] = dict(v)

        overrides = spec.get("params_override")
        if isinstance(overrides, dict):
            for k, v in overrides.items():
                if isinstance(v, dict):
                    if k in merged and isinstance(merged.get(k), dict):
                        merged[k] = {**merged[k], **dict(v)}
                    else:
                        merged[k] = dict(v)

        return merged

    def _clear_params_ui(self) -> None:
        self._param_widgets.clear()
        while self.paramsForm.rowCount() > 0:
            self.paramsForm.removeRow(0)

    def _rebuild_params_ui(self) -> None:
        self._clear_params_ui()

        pipeline = self.pipelineCombo.currentText().strip()
        planner_id = self.plannerCombo.currentText().strip()
        schema = self._merged_param_schema(pipeline, planner_id)

        # stabile Reihenfolge: erst ohne '.' dann mit '.'
        keys = sorted(schema.keys(), key=lambda s: (1 if "." in str(s) else 0, str(s)))

        for key in keys:
            pspec = dict(schema.get(key) or {})
            t = str(pspec.get("type", "number")).strip().lower()

            w: Optional[QtWidgets.QWidget] = None
            kind: str = ""

            if t in ("int", "integer"):
                sb = QtWidgets.QSpinBox(self.paramsHost)
                sb.setRange(int(pspec.get("min", 0)), int(pspec.get("max", 999999)))
                sb.setSingleStep(int(pspec.get("step", 1)))
                w = sb
                kind = "int"

            elif t == "boolean":
                cb = QtWidgets.QCheckBox(self.paramsHost)
                w = cb
                kind = "bool"

            elif t == "enum":
                cbx = QtWidgets.QComboBox(self.paramsHost)
                vals = pspec.get("values") or []
                if isinstance(vals, list):
                    cbx.addItems([str(v) for v in vals])
                w = cbx
                kind = "enum"

            else:
                # number (default)
                minv = float(pspec.get("min", 0.0))
                maxv = float(pspec.get("max", 0.0))
                step = float(pspec.get("step", 0.1))
                if _intish(minv, maxv, step):
                    sb = QtWidgets.QSpinBox(self.paramsHost)
                    sb.setRange(int(minv), int(maxv))
                    sb.setSingleStep(int(step))
                    w = sb
                    kind = "int"
                else:
                    dsb = QtWidgets.QDoubleSpinBox(self.paramsHost)
                    dsb.setRange(minv, maxv)
                    dsb.setSingleStep(step)

                    s = str(pspec.get("step", "0.1"))
                    decimals = 0 if "." not in s else min(6, max(1, len(s.split(".")[1])))
                    dsb.setDecimals(decimals)

                    w = dsb
                    kind = "float"

            assert w is not None

            unit = str(pspec.get("unit", "") or "").strip()
            if unit and hasattr(w, "setSuffix"):
                suf = (" " + unit) if not unit.startswith(" ") else unit
                try:
                    w.setSuffix(suf)  # type: ignore[attr-defined]
                except Exception:
                    pass

            if pspec.get("description") and hasattr(w, "setToolTip"):
                w.setToolTip(str(pspec["description"]))

            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QtWidgets.QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QtWidgets.QSizePolicy.Policy.Minimum)
            w.setSizePolicy(sp)

            self._param_widgets[str(key)] = (w, kind, pspec)
            self.paramsForm.addRow(str(key), w)

    def _apply_defaults_for_current(self) -> None:
        pipeline = self.pipelineCombo.currentText().strip()
        planner_id = self.plannerCombo.currentText().strip()
        schema = self._merged_param_schema(pipeline, planner_id)

        for key, (w, kind, pspec) in self._param_widgets.items():
            default = schema.get(key, pspec).get("default", None)
            if default is None:
                continue
            try:
                if kind == "int" and isinstance(w, QtWidgets.QSpinBox):
                    w.setValue(int(default))
                elif kind == "float" and isinstance(w, QtWidgets.QDoubleSpinBox):
                    w.setValue(float(default))
                elif kind == "bool" and isinstance(w, QtWidgets.QCheckBox):
                    w.setChecked(bool(default))
                elif kind == "enum" and isinstance(w, QtWidgets.QComboBox):
                    ix = w.findText(str(default))
                    if ix >= 0:
                        w.setCurrentIndex(ix)
            except Exception:
                pass

    # ------------------------------------------------------------------ Public API

    @property
    def role_name(self) -> str:
        return self.role

    def apply_planner_model(self, cfg: Dict[str, Any] | None) -> None:
        cfg = cfg if isinstance(cfg, dict) else {}
        pipeline = str(cfg.get("pipeline") or "").strip()
        planner_id = str(cfg.get("planner_id") or "").strip()
        params = cfg.get("params") or {}

        # pipeline list might be role-filtered; ensure selection happens only if present
        if pipeline:
            ix = self.pipelineCombo.findText(pipeline)
            if ix >= 0:
                self.pipelineCombo.setCurrentIndex(ix)

        if planner_id:
            ix = self.plannerCombo.findText(planner_id)
            if ix >= 0:
                self.plannerCombo.setCurrentIndex(ix)

        # ensure params ui is ready
        self._rebuild_params_ui()

        if isinstance(params, dict):
            for k, v in params.items():
                if k not in self._param_widgets:
                    continue
                w, kind, _pspec = self._param_widgets[k]
                try:
                    if kind == "int" and isinstance(w, QtWidgets.QSpinBox):
                        w.setValue(int(v))
                    elif kind == "float" and isinstance(w, QtWidgets.QDoubleSpinBox):
                        w.setValue(float(v))
                    elif kind == "bool" and isinstance(w, QtWidgets.QCheckBox):
                        w.setChecked(bool(v))
                    elif kind == "enum" and isinstance(w, QtWidgets.QComboBox):
                        ix = w.findText(str(v))
                        if ix >= 0:
                            w.setCurrentIndex(ix)
                except Exception:
                    pass

    def collect_planner(self) -> Dict[str, Any]:
        pipeline = self.pipelineCombo.currentText().strip()
        planner_id = self.plannerCombo.currentText().strip()

        out_params: Dict[str, Any] = {}
        for k, (w, kind, _pspec) in self._param_widgets.items():
            if kind == "int" and isinstance(w, QtWidgets.QSpinBox):
                out_params[k] = int(w.value())
            elif kind == "float" and isinstance(w, QtWidgets.QDoubleSpinBox):
                out_params[k] = float(w.value())
            elif kind == "bool" and isinstance(w, QtWidgets.QCheckBox):
                out_params[k] = bool(w.isChecked())
            elif kind == "enum" and isinstance(w, QtWidgets.QComboBox):
                out_params[k] = str(w.currentText())

        return {
            "role": self.role,
            "pipeline": pipeline,
            "planner_id": planner_id,
            "params": out_params,
        }
