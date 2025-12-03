# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Dict, Any
from PyQt6 import QtWidgets, QtCore
from app.model.recipe.recipe_store import RecipeStore


def _get(cfg: Dict[str, Any], dotted: str, default=None):
    """Einfacher Helper für verschachtelte Keys (kann auch 'a.b.c')."""
    if dotted in cfg:
        return cfg[dotted]
    cur = cfg
    for p in dotted.split("."):
        if not isinstance(cur, dict) or p not in cur:
            return default
        cur = cur[p]
    return cur


def _hexp_vmin(widget: QtWidgets.QWidget):
    """Horizontal Expanding (breit wie es will), Vertikal Minimum (so flach wie möglich)."""
    sp = widget.sizePolicy()
    sp.setHorizontalPolicy(QtWidgets.QSizePolicy.Policy.Expanding)
    sp.setVerticalPolicy(QtWidgets.QSizePolicy.Policy.Minimum)
    widget.setSizePolicy(sp)


def _hmin_vmin(widget: QtWidgets.QWidget):
    """Für einzelne Feld-Widgets: Horizontal minimal, Vertikal minimal (drückt Höhe)."""
    sp = widget.sizePolicy()
    sp.setHorizontalPolicy(QtWidgets.QSizePolicy.Policy.Minimum)
    sp.setVerticalPolicy(QtWidgets.QSizePolicy.Policy.Minimum)
    widget.setSizePolicy(sp)


class PlannerGroupBox(QtWidgets.QGroupBox):
    """
    Kompakte Planner-Box für GENAU EINE Rolle, z.B.:
        - role="move"      → validate_move
        - role="path"      → validate_path
        - role="optimize"  → optimize
        - role="service"   → service

    Die Box kennt nur ihre Rolle und verwaltet:
        - pipeline (ompl / chomp / stomp / pilz)
        - planner_id        (z.B. RRTConnectkConfigDefault)
        - generische Parameter:
            planning_time_s, attempts, velocity/acceleration_scaling,
            cartesian.eef_step_mm, allow_replanning,
        - pipeline-spezifische Unter-Parameter (ompl/chomp/stomp/pilz).

    Sie weiß NICHTS über Rezepte, Sides, path_overrides etc.
    Das kommt alles von außen über apply_planner_model()/collect_planner().
    """

    _ROLE_MAP = {
        "move": "validate_move",
        "path": "validate_path",
        "validate_move": "validate_move",
        "validate_path": "validate_path",
        "optimize": "optimize",
        "service": "service",
    }

    def __init__(
        self,
        parent: QtWidgets.QWidget | None = None,
        *,
        title: str = "Planner",
        role: str = "move",
        store: RecipeStore | None = None,
        **kwargs,
    ):
        """
        role: logische Planner-Rolle, z.B. "move", "path", "optimize", "service".
              Für Kompatibilität kann auch type="move" über kwargs übergeben werden.
        """
        # type="..." als Alias akzeptieren (wie im Aufrufer vorgeschlagen)
        if "type" in kwargs and not role:
            role = str(kwargs.pop("type"))

        super().__init__(parent)

        if store is None:
            raise ValueError("PlannerGroupBox: RecipeStore ist Pflicht (store=None).")

        self.store: RecipeStore = store
        self.role_raw: str = str(role)
        self.role: str = self._ROLE_MAP.get(self.role_raw, self.role_raw)

        self.setTitle(title)
        self.setFlat(False)

        _hexp_vmin(self)

        # Pipelines → Seiten im Stack
        self._stack_map = {
            "ompl": 0,
            "chomp": 1,
            "stomp": 2,
            "pilz": 3,
        }

        self._build_ui()
        # Default-Werte aus Store (falls definiert, sonst bleiben Qt-Defaults)
        self.apply_store_defaults()

    # ---------- UI ----------
    def _build_ui(self):
        form = QtWidgets.QFormLayout(self)
        form.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow)
        form.setLabelAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        form.setFormAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        form.setContentsMargins(6, 4, 6, 4)
        form.setHorizontalSpacing(8)
        form.setVerticalSpacing(3)

        # --- COMMON (rollenunabhängig) ---
        self.spinPlanningTime = QtWidgets.QDoubleSpinBox(self)
        self.spinPlanningTime.setRange(0.10, 120.0)
        self.spinPlanningTime.setSingleStep(0.10)

        self.spinAttempts = QtWidgets.QSpinBox(self)
        self.spinAttempts.setRange(1, 100)

        self.spinVel = QtWidgets.QDoubleSpinBox(self)
        self.spinVel.setRange(0.0, 1.0)
        self.spinVel.setSingleStep(0.01)

        self.spinAcc = QtWidgets.QDoubleSpinBox(self)
        self.spinAcc.setRange(0.0, 1.0)
        self.spinAcc.setSingleStep(0.01)

        self.checkCartesian = QtWidgets.QCheckBox(self)
        self.checkReplan = QtWidgets.QCheckBox(self)

        for w in (
            self.spinPlanningTime,
            self.spinAttempts,
            self.spinVel,
            self.spinAcc,
            self.checkCartesian,
            self.checkReplan,
        ):
            _hmin_vmin(w)

        form.addRow("Planning time (s)", self.spinPlanningTime)
        form.addRow("Attempts", self.spinAttempts)
        form.addRow("Velocity scaling", self.spinVel)
        form.addRow("Acceleration scaling", self.spinAcc)
        form.addRow("Use Cartesian waypoints", self.checkCartesian)
        form.addRow("Allow replanning", self.checkReplan)

        # --- Pipeline ---
        self.pipelineCombo = QtWidgets.QComboBox(self)
        self.pipelineCombo.addItems(["ompl", "chomp", "stomp", "pilz"])
        self.pipelineCombo.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToContents)
        _hmin_vmin(self.pipelineCombo)
        self.pipelineCombo.currentTextChanged.connect(self._on_pipeline_changed)
        form.addRow("Pipeline", self.pipelineCombo)

        # --- Planner-ID (OMPL-Planner, Pilz-Name etc.) ---
        self._lblPlannerId = QtWidgets.QLabel("Planner ID", self)
        self._lblPlannerId.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        _hmin_vmin(self._lblPlannerId)

        self.plannerIdCombo = QtWidgets.QComboBox(self)
        # Default-Liste – später kannst du sie dynamisch setzen (allowed_planners_for_side)
        self.plannerIdCombo.addItems(
            [
                "RRTConnectkConfigDefault",
                "RRTstarkConfigDefault",
                "PRMstarkConfigDefault",
                "KPIECEkConfigDefault",
                "BKPIECEkConfigDefault",
            ]
        )
        self.plannerIdCombo.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToContents)
        _hmin_vmin(self.plannerIdCombo)
        form.addRow(self._lblPlannerId, self.plannerIdCombo)

        # --- paramsStack: pro Pipeline eigene Zusatz-Parameter ---
        self.paramsStack = QtWidgets.QStackedWidget(self)
        _hexp_vmin(self.paramsStack)
        form.addRow("Parameters", self.paramsStack)

        # OMPL-Seite
        self.pageOmpl = QtWidgets.QWidget(self)
        self.formOmpl = QtWidgets.QFormLayout(self.pageOmpl)
        self._leftify(self.formOmpl)
        self.spinOmplRange = QtWidgets.QDoubleSpinBox(self.pageOmpl)
        self.spinOmplRange.setRange(0.0, 1.0)
        self.spinOmplRange.setSingleStep(0.01)
        self.spinOmplGoalBias = QtWidgets.QDoubleSpinBox(self.pageOmpl)
        self.spinOmplGoalBias.setRange(0.0, 1.0)
        self.spinOmplGoalBias.setSingleStep(0.01)
        for w in (self.spinOmplRange, self.spinOmplGoalBias):
            _hmin_vmin(w)
        self.formOmpl.addRow("Range (m)", self.spinOmplRange)
        self.formOmpl.addRow("Goal bias", self.spinOmplGoalBias)
        self.paramsStack.addWidget(self.pageOmpl)

        # CHOMP-Seite
        self.pageChomp = QtWidgets.QWidget(self)
        self.formChomp = QtWidgets.QFormLayout(self.pageChomp)
        self._leftify(self.formChomp)
        self.spinChompTime = QtWidgets.QDoubleSpinBox(self.pageChomp)
        self.spinChompTime.setRange(0.10, 120.0)
        self.spinChompTime.setSingleStep(0.10)
        self.spinChompLR = QtWidgets.QDoubleSpinBox(self.pageChomp)
        self.spinChompLR.setDecimals(4)
        self.spinChompLR.setRange(0.0001, 1.0)
        self.spinChompLR.setSingleStep(0.0010)
        self.spinChompSmooth = QtWidgets.QDoubleSpinBox(self.pageChomp)
        self.spinChompSmooth.setRange(0.0, 100.0)
        self.spinChompObs = QtWidgets.QDoubleSpinBox(self.pageChomp)
        self.spinChompObs.setRange(0.0, 100.0)
        for w in (self.spinChompTime, self.spinChompLR, self.spinChompSmooth, self.spinChompObs):
            _hmin_vmin(w)
        self.formChomp.addRow("Time limit (s)", self.spinChompTime)
        self.formChomp.addRow("Learning rate", self.spinChompLR)
        self.formChomp.addRow("Smoothness weight", self.spinChompSmooth)
        self.formChomp.addRow("Obstacle weight", self.spinChompObs)
        self.paramsStack.addWidget(self.pageChomp)

        # STOMP-Seite
        self.pageStomp = QtWidgets.QWidget(self)
        self.formStomp = QtWidgets.QFormLayout(self.pageStomp)
        self._leftify(self.formStomp)
        self.spinStompIter = QtWidgets.QSpinBox(self.pageStomp)
        self.spinStompIter.setRange(1, 10000)
        self.spinStompRoll = QtWidgets.QSpinBox(self.pageStomp)
        self.spinStompRoll.setRange(1, 1000)
        self.spinStompStd = QtWidgets.QDoubleSpinBox(self.pageStomp)
        self.spinStompStd.setRange(0.000, 1.000)
        self.spinStompStd.setSingleStep(0.001)
        for w in (self.spinStompIter, self.spinStompRoll, self.spinStompStd):
            _hmin_vmin(w)
        self.formStomp.addRow("Iterations", self.spinStompIter)
        self.formStomp.addRow("Rollouts", self.spinStompRoll)
        self.formStomp.addRow("Noise stddev", self.spinStompStd)
        self.paramsStack.addWidget(self.pageStomp)

        # Pilz-Seite
        self.pagePilz = QtWidgets.QWidget(self)
        self.formPilz = QtWidgets.QFormLayout(self.pagePilz)
        self._leftify(self.formPilz)
        self.spinPilzVel = QtWidgets.QDoubleSpinBox(self.pagePilz)
        self.spinPilzVel.setRange(0.0, 1.0)
        self.spinPilzAcc = QtWidgets.QDoubleSpinBox(self.pagePilz)
        self.spinPilzAcc.setRange(0.0, 1.0)
        for w in (self.spinPilzVel, self.spinPilzAcc):
            _hmin_vmin(w)
        self.formPilz.addRow("Max vel. scaling", self.spinPilzVel)
        self.formPilz.addRow("Max acc. scaling", self.spinPilzAcc)
        self.paramsStack.addWidget(self.pagePilz)

        # Reset-Button (lädt nur die YAML-Defaults aus dem Store)
        self.btnDefaults = QtWidgets.QPushButton("Reset from YAML", self)
        _hmin_vmin(self.btnDefaults)
        self.btnDefaults.clicked.connect(self.apply_store_defaults)
        form.addRow("", self.btnDefaults)

        # initiale Pipeline-Einstellung anwenden
        self._on_pipeline_changed(self.pipelineCombo.currentText())

    @staticmethod
    def _leftify(form: QtWidgets.QFormLayout):
        form.setLabelAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        form.setFormAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        form.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow)
        form.setContentsMargins(0, 0, 0, 0)
        form.setHorizontalSpacing(8)
        form.setVerticalSpacing(3)

    # ---------- Intern ----------
    def _on_pipeline_changed(self, name: str):
        idx = self._stack_map.get(name, 0)
        self.paramsStack.setCurrentIndex(idx)
        # Planner-ID ist aktuell nur für OMPL wirklich sinnvoll sichtbar
        vis = (name == "ompl")
        self._lblPlannerId.setVisible(vis)
        self.plannerIdCombo.setVisible(vis)

    # ---------- Store-Integration ----------
    def set_store(self, store: RecipeStore):
        if store is None:
            raise ValueError("PlannerGroupBox.set_store: store=None ist nicht erlaubt.")
        self.store = store

    def _store_defaults(self) -> Dict[str, Any]:
        """
        Holt ggf. globale Planner-Defaults aus dem Store.
        Im neuen Schema kann das leer sein – dann bleiben die Qt-Defaults.
        """
        try:
            # collect_planner_defaults kann im neuen Schema leer sein, ist ok
            d = self.store.collect_planner_defaults()
            return d if isinstance(d, dict) else {}
        except Exception:
            return {}

    def apply_store_defaults(self):
        self.set_params(self._store_defaults())

    # ---------- Public API ----------
    @property
    def role_name(self) -> str:
        """Kanonsiche Rollenbezeichnung (validate_move / validate_path / optimize / service)."""
        return self.role

    def apply_planner_model(self, planner_cfg: Dict[str, Any] | None):
        """
        Setzt alle Felder anhand eines Planner-Config-Dicts.
        Das Dict kommt typischerweise aus recipe.planner[role].
        """
        base = self._store_defaults()
        merged = dict(base)
        if isinstance(planner_cfg, dict):
            merged.update(planner_cfg)
        self.set_params(merged)

    def collect_planner(self) -> Dict[str, Any]:
        """
        Liefert das Config-Dict zurück, das direkt in recipe.planner[role] geschrieben werden kann.
        """
        return self.get_params()

    # ---------- Getter/Setter ----------
    def get_planner(self) -> str:
        """Liefert die aktuelle Pipeline (ompl/chomp/stomp/pilz)."""
        return self.pipelineCombo.currentText()

    def set_planner(self, pipeline: str):
        ix = self.pipelineCombo.findText(str(pipeline))
        if ix >= 0:
            self.pipelineCombo.setCurrentIndex(ix)

    def get_params(self) -> Dict[str, Any]:
        pipeline = self.pipelineCombo.currentText()
        return {
            "role": self.role,
            "pipeline": pipeline,
            "planner_id": self.plannerIdCombo.currentText(),
            "planning_time_s": float(self.spinPlanningTime.value()),
            "attempts": int(self.spinAttempts.value()),
            "velocity_scaling": float(self.spinVel.value()),
            "acceleration_scaling": float(self.spinAcc.value()),
            "cartesian.eef_step_mm": 1.0 if self.checkCartesian.isChecked() else 0.0,
            "allow_replanning": bool(self.checkReplan.isChecked()),
            "ompl": {
                "range": float(self.spinOmplRange.value()),
                "goal_bias": float(self.spinOmplGoalBias.value()),
            },
            "chomp": {
                "planning_time_limit": float(self.spinChompTime.value()),
                "learning_rate": float(self.spinChompLR.value()),
                "smoothness_weight": float(self.spinChompSmooth.value()),
                "obstacle_cost_weight": float(self.spinChompObs.value()),
            },
            "stomp": {
                "num_iterations": int(self.spinStompIter.value()),
                "num_rollouts": int(self.spinStompRoll.value()),
                "noise_stddev": float(self.spinStompStd.value()),
            },
            "pilz": {
                "max_velocity_scaling": float(self.spinPilzVel.value()),
                "max_acceleration_scaling": float(self.spinPilzAcc.value()),
            },
        }

    def set_params(self, cfg: Dict[str, Any]):
        if not isinstance(cfg, dict):
            return

        # Rolle ignorieren – ist im Konstruktor festgelegt

        v = _get(cfg, "pipeline")
        if v is not None:
            self.set_planner(str(v))

        v = _get(cfg, "planner_id")
        if v is not None:
            ix = self.plannerIdCombo.findText(str(v))
            if ix >= 0:
                self.plannerIdCombo.setCurrentIndex(ix)

        v = _get(cfg, "planning_time_s")
        if v is not None:
            self.spinPlanningTime.setValue(float(v))
        v = _get(cfg, "attempts")
        if v is not None:
            self.spinAttempts.setValue(int(v))
        v = _get(cfg, "velocity_scaling")
        if v is not None:
            self.spinVel.setValue(float(v))
        v = _get(cfg, "acceleration_scaling")
        if v is not None:
            self.spinAcc.setValue(float(v))

        v = _get(cfg, "allow_replanning")
        if v is not None:
            self.checkReplan.setChecked(bool(v))
        v = _get(cfg, "cartesian.eef_step_mm")
        if v is not None:
            self.checkCartesian.setChecked(float(v) > 0.0)

        ompl = _get(cfg, "ompl", {})
        if isinstance(ompl, dict):
            if "range" in ompl:
                self.spinOmplRange.setValue(float(ompl["range"]))
            if "goal_bias" in ompl:
                self.spinOmplGoalBias.setValue(float(ompl["goal_bias"]))

        chomp = _get(cfg, "chomp", {})
        if isinstance(chomp, dict):
            if "planning_time_limit" in chomp:
                self.spinChompTime.setValue(float(chomp["planning_time_limit"]))
            if "learning_rate" in chomp:
                self.spinChompLR.setValue(float(chomp["learning_rate"]))
            if "smoothness_weight" in chomp:
                self.spinChompSmooth.setValue(float(chomp["smoothness_weight"]))
            if "obstacle_cost_weight" in chomp:
                self.spinChompObs.setValue(float(chomp["obstacle_cost_weight"]))

        stomp = _get(cfg, "stomp", {})
        if isinstance(stomp, dict):
            if "num_iterations" in stomp:
                self.spinStompIter.setValue(int(stomp["num_iterations"]))
            if "num_rollouts" in stomp:
                self.spinStompRoll.setValue(int(stomp["num_rollouts"]))
            if "noise_stddev" in stomp:
                self.spinStompStd.setValue(float(stomp["noise_stddev"]))

        pilz = _get(cfg, "pilz", {})
        if isinstance(pilz, dict):
            if "max_velocity_scaling" in pilz:
                self.spinPilzVel.setValue(float(pilz["max_velocity_scaling"]))
            if "max_acceleration_scaling" in pilz:
                self.spinPilzAcc.setValue(float(pilz["max_acceleration_scaling"]))

        self._on_pipeline_changed(self.pipelineCombo.currentText())
