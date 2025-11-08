# -*- coding: utf-8 -*-
# File: widgets/planner_groupbox.py
from __future__ import annotations
from typing import Dict, Any, Optional
from copy import deepcopy
from PyQt6 import QtWidgets, QtCore

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
    "chomp": {"planning_time_limit": 3.0, "learning_rate": 0.05, "smoothness_weight": 0.1, "obstacle_cost_weight": 1.0},
    "stomp": {"num_iterations": 60, "num_rollouts": 30, "noise_stddev": 0.01},
    "pilz": {"max_velocity_scaling": 0.5, "max_acceleration_scaling": 0.5},
    "servo": {"publish_period": 0.010, "low_pass_filter_coeff": 2.0},
}


class PlannerGroupBox(QtWidgets.QGroupBox):
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None, title: str = "Planner"):
        super().__init__(parent)
        self.setTitle(title)
        # kompakte Rahmenoptik erlaubt noch etwas weniger vertical chrome
        self.setFlat(False)

        self._stack_map = {"ompl": 0, "chomp": 1, "stomp": 2, "pilz": 3, "servo": 4}
        self._build_ui()
        self._apply_config(PLANNER_CONFIG)

    # ---------- UI ----------
    def _build_ui(self):
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(8)

        # --- COMMON (always on top) ---
        commonForm = QtWidgets.QFormLayout()
        commonForm.setLabelAlignment(QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)

        self.spinPlanningTime = QtWidgets.QDoubleSpinBox(self)
        self.spinPlanningTime.setRange(0.10, 120.0); self.spinPlanningTime.setSingleStep(0.10)
        self.spinAttempts = QtWidgets.QSpinBox(self); self.spinAttempts.setRange(1, 100)
        self.spinVel = QtWidgets.QDoubleSpinBox(self); self.spinVel.setRange(0.0, 1.0); self.spinVel.setSingleStep(0.01)
        self.spinAcc = QtWidgets.QDoubleSpinBox(self); self.spinAcc.setRange(0.0, 1.0); self.spinAcc.setSingleStep(0.01)
        self.checkCartesian = QtWidgets.QCheckBox(self)
        self.checkReplan = QtWidgets.QCheckBox(self)

        commonForm.addRow("Planning time (s)", self.spinPlanningTime)
        commonForm.addRow("Attempts", self.spinAttempts)
        commonForm.addRow("Velocity scaling", self.spinVel)
        commonForm.addRow("Acceleration scaling", self.spinAcc)
        commonForm.addRow("Use Cartesian waypoints", self.checkCartesian)
        commonForm.addRow("Allow replanning", self.checkReplan)

        commonWrap = QtWidgets.QWidget(self)
        commonWrap.setLayout(commonForm)
        root.addWidget(commonWrap)

        # --- Pipeline + Planner ID ---
        header = QtWidgets.QFormLayout()
        header.setLabelAlignment(QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)

        self.pipelineCombo = QtWidgets.QComboBox(self)
        self.pipelineCombo.addItems(["ompl", "chomp", "stomp", "pilz", "servo"])
        self.pipelineCombo.currentTextChanged.connect(self._on_pipeline_changed)
        header.addRow("Pipeline", self.pipelineCombo)

        self._lblPlannerId = QtWidgets.QLabel("Planner ID", self)
        self.plannerIdCombo = QtWidgets.QComboBox(self)
        self.plannerIdCombo.addItems([
            "RRTConnectkConfigDefault",
            "RRTstarkConfigDefault",
            "PRMstarkConfigDefault",
            "KPIECEkConfigDefault",
            "BKPIECEkConfigDefault",
        ])
        header.addRow(self._lblPlannerId, self.plannerIdCombo)

        headerWrap = QtWidgets.QWidget(self)
        headerWrap.setLayout(header)
        root.addWidget(headerWrap)

        # --- Per-pipeline params (Stack) ---
        self.paramsStack = QtWidgets.QStackedWidget(self)
        root.addWidget(self.paramsStack)

        # Keep strong refs to pages AND their forms to avoid GC
        # OMPL
        self.pageOmpl = QtWidgets.QWidget(self)
        self.formOmpl = QtWidgets.QFormLayout(self.pageOmpl)
        self.spinOmplRange = QtWidgets.QDoubleSpinBox(self.pageOmpl)
        self.spinOmplRange.setRange(0.0, 1.0); self.spinOmplRange.setSingleStep(0.01)
        self.spinOmplGoalBias = QtWidgets.QDoubleSpinBox(self.pageOmpl)
        self.spinOmplGoalBias.setRange(0.0, 1.0); self.spinOmplGoalBias.setSingleStep(0.01)
        self.formOmpl.addRow("Range (m)", self.spinOmplRange)
        self.formOmpl.addRow("Goal bias", self.spinOmplGoalBias)
        self.paramsStack.addWidget(self.pageOmpl)   # index 0

        # CHOMP
        self.pageChomp = QtWidgets.QWidget(self)
        self.formChomp = QtWidgets.QFormLayout(self.pageChomp)
        self.spinChompTime = QtWidgets.QDoubleSpinBox(self.pageChomp)
        self.spinChompTime.setRange(0.10, 120.0); self.spinChompTime.setSingleStep(0.10)
        self.spinChompLR = QtWidgets.QDoubleSpinBox(self.pageChomp)
        self.spinChompLR.setDecimals(4); self.spinChompLR.setRange(0.0001, 1.0); self.spinChompLR.setSingleStep(0.0010)
        self.spinChompSmooth = QtWidgets.QDoubleSpinBox(self.pageChomp); self.spinChompSmooth.setRange(0.0, 100.0)
        self.spinChompObs = QtWidgets.QDoubleSpinBox(self.pageChomp); self.spinChompObs.setRange(0.0, 100.0)
        self.formChomp.addRow("Time limit (s)", self.spinChompTime)
        self.formChomp.addRow("Learning rate", self.spinChompLR)
        self.formChomp.addRow("Smoothness weight", self.spinChompSmooth)
        self.formChomp.addRow("Obstacle weight", self.spinChompObs)
        self.paramsStack.addWidget(self.pageChomp)  # index 1

        # STOMP
        self.pageStomp = QtWidgets.QWidget(self)
        self.formStomp = QtWidgets.QFormLayout(self.pageStomp)
        self.spinStompIter = QtWidgets.QSpinBox(self.pageStomp); self.spinStompIter.setRange(1, 10000)
        self.spinStompRoll = QtWidgets.QSpinBox(self.pageStomp); self.spinStompRoll.setRange(1, 1000)
        self.spinStompStd  = QtWidgets.QDoubleSpinBox(self.pageStomp); self.spinStompStd.setRange(0.000, 1.000); self.spinStompStd.setSingleStep(0.001)
        self.formStomp.addRow("Iterations", self.spinStompIter)
        self.formStomp.addRow("Rollouts", self.spinStompRoll)
        self.formStomp.addRow("Noise stddev", self.spinStompStd)
        self.paramsStack.addWidget(self.pageStomp)  # index 2

        # Pilz
        self.pagePilz = QtWidgets.QWidget(self)
        self.formPilz = QtWidgets.QFormLayout(self.pagePilz)
        self.spinPilzVel = QtWidgets.QDoubleSpinBox(self.pagePilz); self.spinPilzVel.setRange(0.0, 1.0)
        self.spinPilzAcc = QtWidgets.QDoubleSpinBox(self.pagePilz); self.spinPilzAcc.setRange(0.0, 1.0)
        self.formPilz.addRow("Max vel. scaling", self.spinPilzVel)
        self.formPilz.addRow("Max acc. scaling", self.spinPilzAcc)
        self.paramsStack.addWidget(self.pagePilz)   # index 3

        # Servo
        self.pageServo = QtWidgets.QWidget(self)
        self.formServo = QtWidgets.QFormLayout(self.pageServo)
        self.spinServoPeriod = QtWidgets.QDoubleSpinBox(self.pageServo)
        self.spinServoPeriod.setDecimals(4); self.spinServoPeriod.setRange(0.0010, 0.1000); self.spinServoPeriod.setSingleStep(0.0010)
        self.spinServoLP = QtWidgets.QDoubleSpinBox(self.pageServo); self.spinServoLP.setRange(0.1, 10.0)
        self.formServo.addRow("Publish period (s)", self.spinServoPeriod)
        self.formServo.addRow("Low-pass coeff", self.spinServoLP)
        self.paramsStack.addWidget(self.pageServo)  # index 4

        # Bottom row (Reset) – do NOT let it hog vertical space
        btnRow = QtWidgets.QHBoxLayout()
        btnRow.addStretch(1)
        self.btnDefaults = QtWidgets.QPushButton("Reset defaults", self)
        btnRow.addWidget(self.btnDefaults)
        self.btnDefaults.clicked.connect(self.reset_defaults)
        btnWrap = QtWidgets.QWidget(self)
        btnWrap.setLayout(btnRow)
        btnWrap.setSizePolicy(QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Minimum)
        root.addWidget(btnWrap, 0)  # stretch 0 so it stays compact

        # start on OMPL page
        self._on_pipeline_changed(self.pipelineCombo.currentText())

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
            "ompl": {"range": float(self.spinOmplRange.value()), "goal_bias": float(self.spinOmplGoalBias.value())},
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
            "servo": {"publish_period": float(self.spinServoPeriod.value()), "low_pass_filter_coeff": float(self.spinServoLP.value())},
        }

    def set_params(self, cfg: Dict[str, Any]):
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
        self.set_params(deepcopy(PLANNER_CONFIG))
