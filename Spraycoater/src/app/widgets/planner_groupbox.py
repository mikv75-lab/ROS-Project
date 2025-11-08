# -*- coding: utf-8 -*-
# File: widgets/planner_groupbox.py
from __future__ import annotations
from typing import Dict, Any, Optional
from copy import deepcopy
from PyQt6 import QtWidgets, QtCore

# ===== Default-Konfiguration (später von dir an MoveIt anpassen) =====
PLANNER_CONFIG: Dict[str, Any] = {
    "pipeline": "ompl",
    "planner_id": "RRTConnectkConfigDefault",
    "common": {
        "planning_time": 5.0,
        "attempts": 1,
        "velocity_scaling": 0.5,
        "acceleration_scaling": 0.5,
        "use_cartesian": False,
        "allow_replanning": False,
    },
    "ompl": {"range": 0.0, "goal_bias": 0.05},
    "chomp": {
        "planning_time_limit": 3.0,
        "learning_rate": 0.05,
        "smoothness_weight": 0.1,
        "obstacle_cost_weight": 1.0,
    },
    "stomp": {"num_iterations": 60, "num_rollouts": 30, "noise_stddev": 0.01},
    "pilz": {"max_velocity_scaling": 0.5, "max_acceleration_scaling": 0.5},
    "servo": {"publish_period": 0.010, "low_pass_filter_coeff": 2.0},
}


class PlannerGroupBox(QtWidgets.QGroupBox):
    """
    Minimaler Planner-Selector ohne Signals.
    Kern-API:
        get_planner() -> str
        set_planner(pipeline: str) -> None
        get_params() -> Dict[str, Any]
        set_params(cfg: Dict[str, Any]) -> None
        reset_defaults() -> None

    Rezept-Helper:
        set_from_recipe(recipe: Dict, key: str = "planner") -> None
        to_recipe_block(key: str = "planner") -> Dict[str, Any]
    """

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None, title: str = "Planner"):
        super().__init__(parent)
        self.setTitle(title)
        self._stack_map = {"ompl": 0, "chomp": 1, "stomp": 2, "pilz": 3, "servo": 4}
        self._build_ui()
        self._apply_config(PLANNER_CONFIG)

    # ---------- UI ----------
    def _build_ui(self):
        root = QtWidgets.QVBoxLayout(self)

        # Header (Pipeline + PlannerID)
        header = QtWidgets.QFormLayout()
        header.setLabelAlignment(QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)

        self.pipelineCombo = QtWidgets.QComboBox()
        self.pipelineCombo.addItems(["ompl", "chomp", "stomp", "pilz", "servo"])
        self.pipelineCombo.currentTextChanged.connect(self._on_pipeline_changed)
        header.addRow("Pipeline", self.pipelineCombo)

        self._lblPlannerId = QtWidgets.QLabel("Planner ID")
        self.plannerIdCombo = QtWidgets.QComboBox()
        self.plannerIdCombo.addItems([
            "RRTConnectkConfigDefault",
            "RRTstarkConfigDefault",
            "PRMstarkConfigDefault",
            "KPIECEkConfigDefault",
            "BKPIECEkConfigDefault",
        ])
        header.addRow(self._lblPlannerId, self.plannerIdCombo)

        headerBox = QtWidgets.QWidget()
        headerBox.setLayout(header)
        root.addWidget(headerBox)

        # Common
        self.groupCommon = QtWidgets.QGroupBox("Common")
        commonForm = QtWidgets.QFormLayout(self.groupCommon)

        self.spinPlanningTime = QtWidgets.QDoubleSpinBox()
        self.spinPlanningTime.setRange(0.10, 120.0); self.spinPlanningTime.setSingleStep(0.10)

        self.spinAttempts = QtWidgets.QSpinBox()
        self.spinAttempts.setRange(1, 100)

        self.spinVel = QtWidgets.QDoubleSpinBox()
        self.spinVel.setRange(0.0, 1.0); self.spinVel.setSingleStep(0.01)

        self.spinAcc = QtWidgets.QDoubleSpinBox()
        self.spinAcc.setRange(0.0, 1.0); self.spinAcc.setSingleStep(0.01)

        self.checkCartesian = QtWidgets.QCheckBox()
        self.checkReplan = QtWidgets.QCheckBox()

        commonForm.addRow("Planning time (s)", self.spinPlanningTime)
        commonForm.addRow("Attempts", self.spinAttempts)
        commonForm.addRow("Velocity scaling", self.spinVel)
        commonForm.addRow("Acceleration scaling", self.spinAcc)
        commonForm.addRow("Use Cartesian waypoints", self.checkCartesian)
        commonForm.addRow("Allow replanning", self.checkReplan)
        root.addWidget(self.groupCommon)

        # Pipeline-spezifisch (Stack)
        self.paramsStack = QtWidgets.QStackedWidget()

        # OMPL
        pageOmpl = QtWidgets.QWidget()
        formOmpl = QtWidgets.QFormLayout(pageOmpl)
        self.spinOmplRange = QtWidgets.QDoubleSpinBox()
        self.spinOmplRange.setRange(0.0, 1.0); self.spinOmplRange.setSingleStep(0.01)
        self.spinOmplGoalBias = QtWidgets.QDoubleSpinBox()
        self.spinOmplGoalBias.setRange(0.0, 1.0); self.spinOmplGoalBias.setSingleStep(0.01)
        formOmpl.addRow("Range (m)", self.spinOmplRange)
        formOmpl.addRow("Goal bias", self.spinOmplGoalBias)

        # CHOMP
        pageChomp = QtWidgets.QWidget()
        formChomp = QtWidgets.QFormLayout(pageChomp)
        self.spinChompTime = QtWidgets.QDoubleSpinBox()
        self.spinChompTime.setRange(0.10, 120.0); self.spinChompTime.setSingleStep(0.10)
        self.spinChompLR = QtWidgets.QDoubleSpinBox()
        self.spinChompLR.setDecimals(4); self.spinChompLR.setRange(0.0001, 1.0); self.spinChompLR.setSingleStep(0.0010)
        self.spinChompSmooth = QtWidgets.QDoubleSpinBox()
        self.spinChompSmooth.setRange(0.0, 100.0)
        self.spinChompObs = QtWidgets.QDoubleSpinBox()
        self.spinChompObs.setRange(0.0, 100.0)
        formChomp.addRow("Time limit (s)", self.spinChompTime)
        formChomp.addRow("Learning rate", self.spinChompLR)
        formChomp.addRow("Smoothness weight", self.spinChompSmooth)
        formChomp.addRow("Obstacle weight", self.spinChompObs)

        # STOMP
        pageStomp = QtWidgets.QWidget()
        formStomp = QtWidgets.QFormLayout(pageStomp)
        self.spinStompIter = QtWidgets.QSpinBox()
        self.spinStompIter.setRange(1, 10000)
        self.spinStompRoll = QtWidgets.QSpinBox()
        self.spinStompRoll.setRange(1, 1000)
        self.spinStompStd = QtWidgets.QDoubleSpinBox()
        self.spinStompStd.setRange(0.000, 1.000); self.spinStompStd.setSingleStep(0.001)
        formStomp.addRow("Iterations", self.spinStompIter)
        formStomp.addRow("Rollouts", self.spinStompRoll)
        formStomp.addRow("Noise stddev", self.spinStompStd)

        # Pilz
        pagePilz = QtWidgets.QWidget()
        formPilz = QtWidgets.QFormLayout(pagePilz)
        self.spinPilzVel = QtWidgets.QDoubleSpinBox()
        self.spinPilzVel.setRange(0.0, 1.0)
        self.spinPilzAcc = QtWidgets.QDoubleSpinBox()
        self.spinPilzAcc.setRange(0.0, 1.0)
        formPilz.addRow("Max vel. scaling", self.spinPilzVel)
        formPilz.addRow("Max acc. scaling", self.spinPilzAcc)

        # Servo
        pageServo = QtWidgets.QWidget()
        formServo = QtWidgets.QFormLayout(pageServo)
        self.spinServoPeriod = QtWidgets.QDoubleSpinBox()
        self.spinServoPeriod.setDecimals(4); self.spinServoPeriod.setRange(0.0010, 0.1000); self.spinServoPeriod.setSingleStep(0.0010)
        self.spinServoLP = QtWidgets.QDoubleSpinBox()
        self.spinServoLP.setRange(0.1, 10.0)
        formServo.addRow("Publish period (s)", self.spinServoPeriod)
        formServo.addRow("Low-pass coeff", self.spinServoLP)

        # Stack befüllen
        self.paramsStack.addWidget(pageOmpl)   # 0
        self.paramsStack.addWidget(pageChomp)  # 1
        self.paramsStack.addWidget(pageStomp)  # 2
        self.paramsStack.addWidget(pagePilz)   # 3
        self.paramsStack.addWidget(pageServo)  # 4
        root.addWidget(self.paramsStack)

        # Reset (nur intern)
        btnRow = QtWidgets.QHBoxLayout()
        btnRow.addStretch(1)
        self.btnDefaults = QtWidgets.QPushButton("Reset defaults")
        self.btnDefaults.clicked.connect(self.reset_defaults)
        btnWrap = QtWidgets.QWidget()
        btnWrap.setLayout(btnRow)
        btnRow.addWidget(self.btnDefaults)
        root.addWidget(btnWrap)

    # ---------- Intern ----------
    def _on_pipeline_changed(self, name: str):
        idx = self._stack_map.get(name, 0)
        self.paramsStack.setCurrentIndex(idx)
        vis = (name == "ompl")
        self._lblPlannerId.setVisible(vis)
        self.plannerIdCombo.setVisible(vis)

    def _apply_config(self, cfg: Dict[str, Any]):
        self.set_params(cfg)

    # ---------- Public API ----------
    def get_planner(self) -> str:
        return self.pipelineCombo.currentText()

    def set_planner(self, pipeline: str):
        ix = self.pipelineCombo.findText(pipeline)
        if ix >= 0:
            self.pipelineCombo.setCurrentIndex(ix)

    def get_params(self) -> Dict[str, Any]:
        pipeline = self.pipelineCombo.currentText()
        return {
            "pipeline": pipeline,
            "planner_id": self.plannerIdCombo.currentText(),
            "common": {
                "planning_time": float(self.spinPlanningTime.value()),
                "attempts": int(self.spinAttempts.value()),
                "velocity_scaling": float(self.spinVel.value()),
                "acceleration_scaling": float(self.spinAcc.value()),
                "use_cartesian": bool(self.checkCartesian.isChecked()),
                "allow_replanning": bool(self.checkReplan.isChecked()),
            },
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
            "servo": {
                "publish_period": float(self.spinServoPeriod.value()),
                "low_pass_filter_coeff": float(self.spinServoLP.value()),
            },
        }

    def set_params(self, cfg: Dict[str, Any]):
        # Pipeline zuerst (schaltet den Stack)
        if (p := cfg.get("pipeline")) is not None:
            self.set_planner(p)

        if (pid := cfg.get("planner_id")) is not None:
            ix = self.plannerIdCombo.findText(pid)
            if ix >= 0:
                self.plannerIdCombo.setCurrentIndex(ix)

        common = cfg.get("common", {})
        if "planning_time" in common: self.spinPlanningTime.setValue(float(common["planning_time"]))
        if "attempts" in common: self.spinAttempts.setValue(int(common["attempts"]))
        if "velocity_scaling" in common: self.spinVel.setValue(float(common["velocity_scaling"]))
        if "acceleration_scaling" in common: self.spinAcc.setValue(float(common["acceleration_scaling"]))
        if "use_cartesian" in common: self.checkCartesian.setChecked(bool(common["use_cartesian"]))
        if "allow_replanning" in common: self.checkReplan.setChecked(bool(common["allow_replanning"]))

        ompl = cfg.get("ompl", {})
        if "range" in ompl: self.spinOmplRange.setValue(float(ompl["range"]))
        if "goal_bias" in ompl: self.spinOmplGoalBias.setValue(float(ompl["goal_bias"]))

        chomp = cfg.get("chomp", {})
        if "planning_time_limit" in chomp: self.spinChompTime.setValue(float(chomp["planning_time_limit"]))
        if "learning_rate" in chomp: self.spinChompLR.setValue(float(chomp["learning_rate"]))
        if "smoothness_weight" in chomp: self.spinChompSmooth.setValue(float(chomp["smoothness_weight"]))
        if "obstacle_cost_weight" in chomp: self.spinChompObs.setValue(float(chomp["obstacle_cost_weight"]))

        stomp = cfg.get("stomp", {})
        if "num_iterations" in stomp: self.spinStompIter.setValue(int(stomp["num_iterations"]))
        if "num_rollouts" in stomp: self.spinStompRoll.setValue(int(stomp["num_rollouts"]))
        if "noise_stddev" in stomp: self.spinStompStd.setValue(float(stomp["noise_stddev"]))

        pilz = cfg.get("pilz", {})
        if "max_velocity_scaling" in pilz: self.spinPilzVel.setValue(float(pilz["max_velocity_scaling"]))
        if "max_acceleration_scaling" in pilz: self.spinPilzAcc.setValue(float(pilz["max_acceleration_scaling"]))

        servo = cfg.get("servo", {})
        if "publish_period" in servo: self.spinServoPeriod.setValue(float(servo["publish_period"]))
        if "low_pass_filter_coeff" in servo: self.spinServoLP.setValue(float(servo["low_pass_filter_coeff"]))

    def reset_defaults(self):
        """Setzt alles auf PLANNER_CONFIG zurück."""
        self.set_params(deepcopy(PLANNER_CONFIG))

    # ---------- Rezept-Helper ----------
    def set_from_recipe(self, recipe: Dict[str, Any], key: str = "planner"):
        """
        Lädt Planner-Block aus dem Rezept:
            recipe[key] -> set_params(...)
        Tut nichts, wenn key fehlt oder kein Dict ist.
        """
        block = recipe.get(key)
        if isinstance(block, dict):
            self.set_params(block)

    def to_recipe_block(self, key: str = "planner") -> Dict[str, Any]:
        """
        Erzeugt einen Dict-Block für das Rezept:
            { key: <params-dict> }
        """
        return {key: self.get_params()}
