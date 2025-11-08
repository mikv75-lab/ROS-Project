# -*- coding: utf-8 -*-
# File: widgets/servo_boxes.py
from __future__ import annotations
from typing import Optional, List, Tuple, Dict, Any, Sequence

from PyQt6 import QtCore
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QFormLayout,
    QLabel, QPushButton, QSlider, QDoubleSpinBox, QRadioButton,
    QSizePolicy
)

# =============================================================================
# JointJogWidget (QWidget)
# =============================================================================
class JointJogWidget(QWidget):
    """
    Joint-Jogging:
      - Form oben: Step(°), Speed(%)
      - Grid je Joint: [Label] [−] [Slider] [+] [Value(°)]

    Signals:
      - paramsChanged(dict)                     -> {"joint_step_deg","joint_speed_pct"}
      - jointJogRequested(str, float, float)    -> (joint_name, delta_deg, speed_pct)
    """
    paramsChanged = QtCore.pyqtSignal(dict)
    jointJogRequested = QtCore.pyqtSignal(str, float, float)

    def __init__(self, bridge=None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.bridge = bridge

        self._joint_names: List[str] = []
        self._joint_lims_deg: List[Tuple[float, float]] = []
        # pro Joint-Reihe: (btn-, slider, btn+, lblValue)
        self._rows: List[Tuple[QPushButton, QSlider, QPushButton, QLabel]] = []

        self._build_ui()
        self._wire_signals()
        self._try_bind_to_bridge()

        # Defaults
        self.set_params({"joint_step_deg": 2.0, "joint_speed_pct": 40.0})

        # Sofort nutzbar: 6 Joints ±180° & Startwerte 0°
        self.set_joint_model([f"J{i}" for i in range(1, 7)], [(-180.0, 180.0)] * 6)
        self.set_joint_positions_deg([0.0] * 6)

        # Policies: vertikal überall Maximum
        self._apply_vertical_max()

    # ---------- UI ----------
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # Form: Step + Speed
        form = QFormLayout()
        self.spinStepDeg = QDoubleSpinBox(self)
        self.spinStepDeg.setRange(0.1, 45.0)
        self.spinStepDeg.setDecimals(1)
        self.spinStepDeg.setSingleStep(0.1)

        self.spinSpeedPct = QDoubleSpinBox(self)
        self.spinSpeedPct.setRange(1.0, 100.0)
        self.spinSpeedPct.setSingleStep(1.0)
        self.spinSpeedPct.setSuffix(" %")

        form.addRow("Step (°)", self.spinStepDeg)
        form.addRow("Speed (%)", self.spinSpeedPct)
        root.addLayout(form)

        # Grid: [Label] [−] [Slider] [+] [Value]
        self.grid = QGridLayout()
        self.grid.setHorizontalSpacing(10)
        self.grid.setVerticalSpacing(6)
        self.grid.setContentsMargins(4, 0, 4, 0)
        # Header
        self.grid.addWidget(self._bold("Joint"),     0, 0)
        self.grid.addWidget(self._bold("−"),         0, 1)
        self.grid.addWidget(self._bold("Position"),  0, 2)
        self.grid.addWidget(self._bold("+"),         0, 3)
        self.grid.addWidget(self._bold("Value (°)"), 0, 4)
        root.addLayout(self.grid)

    def _bold(self, txt: str) -> QLabel:
        l = QLabel(txt, self)
        l.setStyleSheet("font-weight:600;")
        return l

    # ---------- Signals ----------
    def _wire_signals(self):
        self.spinStepDeg.valueChanged.connect(lambda _: self._emit_params())
        self.spinSpeedPct.valueChanged.connect(lambda _: self._emit_params())

    def _try_bind_to_bridge(self):
        # optionales Forward der Outbound-Signale an Bridge-Signale
        target = None
        for attr in ("_servo", "_motion"):
            obj = getattr(self.bridge, attr, None) if self.bridge else None
            if obj and getattr(obj, "signals", None):
                target = obj.signals
                break
        if not target:
            return
        if hasattr(target, "paramsChanged"):
            self.paramsChanged.connect(target.paramsChanged.emit)
        if hasattr(target, "jointJogRequested"):
            self.jointJogRequested.connect(target.jointJogRequested.emit)

    # ---------- Public API ----------
    def set_params(self, cfg: Dict[str, Any]):
        if "joint_step_deg" in cfg:
            self.spinStepDeg.setValue(float(cfg["joint_step_deg"]))
        if "joint_speed_pct" in cfg:
            self.spinSpeedPct.setValue(float(cfg["joint_speed_pct"]))
        self._emit_params()

    def get_params(self) -> Dict[str, Any]:
        return {
            "joint_step_deg": float(self.spinStepDeg.value()),
            "joint_speed_pct": float(self.spinSpeedPct.value()),
        }

    def set_joint_model(self, joint_names: Sequence[str], limits_deg: Sequence[Tuple[float, float]]):
        # alte Reihen entfernen
        for (btnm, sld, btnp, lbl) in self._rows:
            btnm.deleteLater(); sld.deleteLater(); btnp.deleteLater(); lbl.deleteLater()
        self._rows.clear()

        # Grid leeren & Header neu setzen
        while self.grid.count():
            item = self.grid.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()
        self.grid.addWidget(self._bold("Joint"),     0, 0)
        self.grid.addWidget(self._bold("−"),         0, 1)
        self.grid.addWidget(self._bold("Position"),  0, 2)
        self.grid.addWidget(self._bold("+"),         0, 3)
        self.grid.addWidget(self._bold("Value (°)"), 0, 4)

        self._joint_names = list(joint_names)
        self._joint_lims_deg = list(limits_deg)

        for row_idx, name in enumerate(self._joint_names, start=1):
            mn, mx = self._joint_lims_deg[row_idx - 1]

            # Label
            self.grid.addWidget(QLabel(name, self), row_idx, 0)

            # Minus
            btnm = QPushButton("−", self)
            btnm.setMinimumHeight(24)
            self.grid.addWidget(btnm, row_idx, 1)

            # Slider
            sld = QSlider(Qt.Orientation.Horizontal, self)
            sld.setMinimum(int(mn * 10))
            sld.setMaximum(int(mx * 10))
            sld.setSingleStep(1)
            sld.setPageStep(5)
            sld.setTickPosition(QSlider.TickPosition.TicksBelow)
            tick = max(1, int((mx - mn) // 10)) * 10
            sld.setTickInterval(tick)
            self.grid.addWidget(sld, row_idx, 2)

            # Plus
            btnp = QPushButton("+", self)
            btnp.setMinimumHeight(24)
            self.grid.addWidget(btnp, row_idx, 3)

            # Value
            lblVal = QLabel("-", self)
            lblVal.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lblVal.setMinimumWidth(70)
            self.grid.addWidget(lblVal, row_idx, 4)

            # Wiring
            idx = row_idx - 1
            btnm.clicked.connect(lambda _, i=idx: self._emit_joint_step(i, negative=True))
            btnp.clicked.connect(lambda _, i=idx: self._emit_joint_step(i, negative=False))
            sld.valueChanged.connect(lambda _, i=idx: self._on_slider_changed(i))

            self._rows.append((btnm, sld, btnp, lblVal))

        self.grid.setColumnStretch(2, 1)

    def set_joint_positions_deg(self, positions_deg: Sequence[float]):
        for i, v in enumerate(positions_deg):
            if i >= len(self._rows):
                break
            _, sld, _, lbl = self._rows[i]
            val10 = int(float(v) * 10.0)
            sld.blockSignals(True)
            sld.setValue(val10)
            sld.blockSignals(False)
            lbl.setText(f"{float(v):.2f}")

    # ---------- intern ----------
    def _emit_params(self):
        self.paramsChanged.emit(self.get_params())

    def _emit_joint_step(self, idx: int, *, negative: bool):
        if idx < 0 or idx >= len(self._rows):
            return
        name = self._joint_names[idx]
        step = float(self.spinStepDeg.value())
        delta = -step if negative else step
        speed = float(self.spinSpeedPct.value())
        self.jointJogRequested.emit(name, delta, speed)

    def _on_slider_changed(self, idx: int):
        _, sld, _, lbl = self._rows[idx]
        lbl.setText(f"{float(sld.value())/10.0:.2f}")

    # ---------- Sizing Helper ----------
    def _apply_vertical_max(self):
        widgets = [self] + self.findChildren(QWidget)
        for w in widgets:
            sp = w.sizePolicy()
            sp.setVerticalPolicy(QSizePolicy.Policy.Maximum)
            if sp.horizontalPolicy() < QSizePolicy.Policy.Expanding:
                sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            w.setSizePolicy(sp)


# =============================================================================
# CartesianJogWidget (QWidget) – linear + rotational
# =============================================================================
class CartesianJogWidget(QWidget):
    """
    Cartesian Jogging (linear + rotational):

      Form:
        - Linear step (mm)
        - Angular step (°)
        - Speed (mm/s)   -> für x/y/z (lineare Achsen)

      Frame:
        - WRF/TRF

      Buttons:
        - Linear:  ±X, ±Y, ±Z  (delta in mm)
        - Rot:     ±RX, ±RY, ±RZ (delta in Grad)

    Signals:
      - frameChanged(str) -> "wrf" | "trf"
      - paramsChanged(dict) -> {"cart_lin_step_mm","cart_ang_step_deg","cart_speed_mm_s","frame"}
      - cartesianJogRequested(str, float, float, str)
          axis∈{x,y,z}  -> delta=mm,  speed=mm/s
          axis∈{rx,ry,rz}-> delta=deg, speed=mm/s (Backend kann speed ignorieren/abbilden)
    """
    frameChanged = QtCore.pyqtSignal(str)
    paramsChanged = QtCore.pyqtSignal(dict)
    cartesianJogRequested = QtCore.pyqtSignal(str, float, float, str)

    def __init__(self, bridge=None, parent: Optional[WIDGET] = None):  # type: ignore[name-defined]
        super().__init__(parent)
        self.bridge = bridge
        self._build_ui()
        self._wire_signals()
        self._try_bind_to_bridge()
        self.set_params({
            "cart_lin_step_mm": 1.0,
            "cart_ang_step_deg": 2.0,
            "cart_speed_mm_s": 50.0,
            "frame": "wrf",
        })

        # Policies: vertikal überall Maximum
        self._apply_vertical_max()

    # ---------- UI ----------
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # --- Form ---
        form = QFormLayout()
        self.spinLinStep = QDoubleSpinBox(self)
        self.spinLinStep.setRange(0.1, 200.0)
        self.spinLinStep.setDecimals(1)
        self.spinLinStep.setSingleStep(0.1)
        self.spinLinStep.setSuffix(" mm")

        self.spinAngStep = QDoubleSpinBox(self)
        self.spinAngStep.setRange(0.1, 45.0)
        self.spinAngStep.setDecimals(1)
        self.spinAngStep.setSingleStep(0.1)
        self.spinAngStep.setSuffix(" °")

        self.spinSpeedMmS = QDoubleSpinBox(self)
        self.spinSpeedMmS.setRange(1.0, 2000.0)
        self.spinSpeedMmS.setDecimals(1)
        self.spinSpeedMmS.setSingleStep(1.0)
        self.spinSpeedMmS.setSuffix(" mm/s")

        form.addRow("Linear step (mm)", self.spinLinStep)
        form.addRow("Angular step (°)", self.spinAngStep)
        form.addRow("Speed (mm/s)", self.spinSpeedMmS)
        root.addLayout(form)

        # --- Frame ---
        frameRow = QHBoxLayout()
        self.lblFrame = self._bold("Frame:")
        self.rbWRF = QRadioButton("WRF (World)", self)
        self.rbTRF = QRadioButton("TRF (Tool)", self)
        self.rbWRF.setChecked(True)
        frameRow.addWidget(self.lblFrame)
        frameRow.addWidget(self.rbWRF)
        frameRow.addWidget(self.rbTRF)
        frameRow.addStretch(1)
        root.addLayout(frameRow)

        # --- Linear Buttons ---
        grid_lin = QGridLayout()
        grid_lin.addWidget(self._bold("Linear"), 0, 0)
        self.btnXm = QPushButton("−X")
        self.btnXp = QPushButton("+X")
        self.btnYm = QPushButton("−Y")
        self.btnYp = QPushButton("+Y")
        self.btnZm = QPushButton("−Z")
        self.btnZp = QPushButton("+Z")
        grid_lin.addWidget(self.btnXm, 1, 0)
        grid_lin.addWidget(self.btnXp, 1, 1)
        grid_lin.addWidget(self.btnYm, 2, 0)
        grid_lin.addWidget(self.btnYp, 2, 1)
        grid_lin.addWidget(self.btnZm, 3, 0)
        grid_lin.addWidget(self.btnZp, 3, 1)
        root.addLayout(grid_lin)

        # --- Rotational Buttons ---
        grid_rot = QGridLayout()
        grid_rot.addWidget(self._bold("Rotational"), 0, 0)
        self.btnRxm = QPushButton("−RX")
        self.btnRxp = QPushButton("+RX")
        self.btnRym = QPushButton("−RY")
        self.btnRyp = QPushButton("+RY")
        self.btnRzm = QPushButton("−RZ")
        self.btnRzp = QPushButton("+RZ")
        grid_rot.addWidget(self.btnRxm, 1, 0)
        grid_rot.addWidget(self.btnRxp, 1, 1)
        grid_rot.addWidget(self.btnRym, 2, 0)
        grid_rot.addWidget(self.btnRyp, 2, 1)
        grid_rot.addWidget(self.btnRzm, 3, 0)
        grid_rot.addWidget(self.btnRzp, 3, 1)
        root.addLayout(grid_rot)

    def _bold(self, txt: str) -> QLabel:
        l = QLabel(txt, self)
        l.setStyleSheet("font-weight:600;")
        return l

    # ---------- Signals ----------
    def _wire_signals(self):
        self.spinLinStep.valueChanged.connect(lambda _: self._emit_params())
        self.spinAngStep.valueChanged.connect(lambda _: self._emit_params())
        self.spinSpeedMmS.valueChanged.connect(lambda _: self._emit_params())
        self.rbWRF.toggled.connect(lambda _: self._on_frame_changed())
        self.rbTRF.toggled.connect(lambda _: self._on_frame_changed())

        # Linear
        self.btnXm.clicked.connect(lambda: self._emit_cart("x", -self.spinLinStep.value()))
        self.btnXp.clicked.connect(lambda: self._emit_cart("x",  self.spinLinStep.value()))
        self.btnYm.clicked.connect(lambda: self._emit_cart("y", -self.spinLinStep.value()))
        self.btnYp.clicked.connect(lambda: self._emit_cart("y",  self.spinLinStep.value()))
        self.btnZm.clicked.connect(lambda: self._emit_cart("z", -self.spinLinStep.value()))
        self.btnZp.clicked.connect(lambda: self._emit_cart("z",  self.spinLinStep.value()))

        # Rotational (delta in Grad; speed_mm_s wird mitgegeben)
        self.btnRxm.clicked.connect(lambda: self._emit_rot("rx", -self.spinAngStep.value()))
        self.btnRxp.clicked.connect(lambda: self._emit_rot("rx",  self.spinAngStep.value()))
        self.btnRym.clicked.connect(lambda: self._emit_rot("ry", -self.spinAngStep.value()))
        self.btnRyp.clicked.connect(lambda: self._emit_rot("ry",  self.spinAngStep.value()))
        self.btnRzm.clicked.connect(lambda: self._emit_rot("rz", -self.spinAngStep.value()))
        self.btnRzp.clicked.connect(lambda: self._emit_rot("rz",  self.spinAngStep.value()))

    def _try_bind_to_bridge(self):
        target = None
        for attr in ("_servo", "_motion"):
            obj = getattr(self.bridge, attr, None) if self.bridge else None
            if obj and getattr(obj, "signals", None):
                target = obj.signals
                break
        if not target:
            return
        if hasattr(target, "frameChanged"):
            self.frameChanged.connect(target.frameChanged.emit)
        elif hasattr(target, "jogFrameChanged"):
            self.frameChanged.connect(target.jogFrameChanged.emit)
        if hasattr(target, "paramsChanged"):
            self.paramsChanged.connect(target.paramsChanged.emit)
        if hasattr(target, "cartesianJogRequested"):
            self.cartesianJogRequested.connect(target.cartesianJogRequested.emit)

    # ---------- Public API ----------
    def set_params(self, cfg: Dict[str, Any]):
        if "cart_lin_step_mm" in cfg:
            self.spinLinStep.setValue(float(cfg["cart_lin_step_mm"]))
        if "cart_ang_step_deg" in cfg:
            self.spinAngStep.setValue(float(cfg["cart_ang_step_deg"]))
        if "cart_speed_mm_s" in cfg:
            self.spinSpeedMmS.setValue(float(cfg["cart_speed_mm_s"]))
        if "frame" in cfg:
            self.set_frame(str(cfg["frame"]))
        self._emit_params()

    def get_params(self) -> Dict[str, Any]:
        return {
            "cart_lin_step_mm": float(self.spinLinStep.value()),
            "cart_ang_step_deg": float(self.spinAngStep.value()),
            "cart_speed_mm_s": float(self.spinSpeedMmS.value()),
            "frame": self.get_frame(),
        }

    def set_frame(self, frame: str):
        f = (frame or "").strip().lower()
        if f == "trf":
            self.rbTRF.setChecked(True)
        else:
            self.rbWRF.setChecked(True)

    def get_frame(self) -> str:
        return "trf" if self.rbTRF.isChecked() else "wrf"

    # ---------- intern / Emit ----------
    def _emit_params(self):
        self.paramsChanged.emit(self.get_params())

    def _on_frame_changed(self):
        self.frameChanged.emit(self.get_frame())
        self._emit_params()

    def _emit_cart(self, axis: str, delta_mm: float):
        speed = float(self.spinSpeedMmS.value())
        frame = self.get_frame()
        self.cartesianJogRequested.emit(axis, float(delta_mm), speed, frame)

    def _emit_rot(self, axis: str, delta_deg: float):
        speed = float(self.spinSpeedMmS.value())   # linear speed; bei Rot kann Backend entscheiden
        frame = self.get_frame()
        self.cartesianJogRequested.emit(axis, float(delta_deg), speed, frame)

    # ---------- Sizing Helper ----------
    def _apply_vertical_max(self):
        widgets = [self] + self.findChildren(QWidget)
        for w in widgets:
            sp = w.sizePolicy()
            sp.setVerticalPolicy(QSizePolicy.Policy.Maximum)
            if sp.horizontalPolicy() < QSizePolicy.Policy.Expanding:
                sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            w.setSizePolicy(sp)
