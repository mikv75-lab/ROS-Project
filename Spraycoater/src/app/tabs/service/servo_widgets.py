# -*- coding: utf-8 -*-
# File: tabs/service/servo_widgets.py
from __future__ import annotations
from typing import Optional, List, Tuple, Dict, Any
import math

from PyQt6 import QtCore
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QGridLayout,
    QSizePolicy, QFormLayout, QDoubleSpinBox, QSlider, QSpacerItem, QRadioButton
)


# =============================================================================
# JointJogWidget – Joint-Jogging
# =============================================================================
class JointJogWidget(QWidget):
    paramsChanged = QtCore.pyqtSignal(dict)
    jointJogRequested = QtCore.pyqtSignal(str, float, float)

    def __init__(self, ctx, bridge=None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.content = ctx.content          # Frames/QoS/Topics
        self.bridge = bridge

        self._joint_names: List[str] = []
        self._joint_lims_deg: List[Tuple[float, float]] = []
        self._rows: List[Tuple[QPushButton, QSlider, QPushButton, QLabel]] = []

        self._build_ui()
        self._wire_signals()
        self._try_bind_to_bridge()

        # Default parameters
        self.set_params({"joint_step_deg": 2.0, "joint_speed_pct": 40.0})

        # Default 6-DOF joint model
        joint_names = [f"joint_{i}" for i in range(1, 7)]
        limits_deg = [
            (-170.0, 170.0),
            (-190.0, 45.0),
            (-29.0, 256.0),
            (-190.0, 190.0),
            (-120.0, 120.0),
            (-180.0, 180.0),
        ]
        self.set_joint_model(joint_names, limits_deg)
        self.set_joint_positions_deg([0.0] * 6)

        self._apply_vertical_max()

    # ---------------- UI -----------------
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        form = QFormLayout()
        self.spinStepDeg = QDoubleSpinBox(self)
        self.spinStepDeg.setRange(0.1, 45.0)
        self.spinStepDeg.setDecimals(1)

        self.spinSpeedPct = QDoubleSpinBox(self)
        self.spinSpeedPct.setRange(1.0, 100.0)
        self.spinSpeedPct.setSuffix(" %")

        form.addRow("Step (°)", self.spinStepDeg)
        form.addRow("Speed (%)", self.spinSpeedPct)
        root.addLayout(form)

        # Grid of joints
        self.grid = QGridLayout()
        self.grid.setHorizontalSpacing(10)
        self.grid.setVerticalSpacing(6)

        self.grid.addWidget(self._bold("Joint"),     0, 0)
        self.grid.addWidget(self._bold("Decrease"),  0, 1)
        self.grid.addWidget(self._bold("Position"),  0, 2)
        self.grid.addWidget(self._bold("Increase"),  0, 3)
        self.grid.addWidget(self._bold("Value (°)"), 0, 4)

        root.addLayout(self.grid)
        root.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

    def _bold(self, txt: str) -> QLabel:
        l = QLabel(txt, self)
        l.setStyleSheet("font-weight:600;")
        return l

    # ---------------- Signals -----------------
    def _wire_signals(self):
        self.spinStepDeg.valueChanged.connect(lambda _: self._emit_params())
        self.spinSpeedPct.valueChanged.connect(lambda _: self._emit_params())

    def _try_bind_to_bridge(self):
        if not self.bridge:
            return
        target = None
        for attr in ("_servo", "_motion"):
            obj = getattr(self.bridge, attr, None)
            if obj and getattr(obj, "signals", None):
                target = obj.signals
                break
        if not target:
            return

        if hasattr(target, "paramsChanged"):
            self.paramsChanged.connect(target.paramsChanged.emit)
        if hasattr(target, "jointJogRequested"):
            self.jointJogRequested.connect(target.jointJogRequested.emit)

    # ---------------- Public API -----------------
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

    def set_joint_model(self, names, limits_deg):
        # Delete old rows
        for (btnm, sld, btnp, lbl) in self._rows:
            btnm.deleteLater()
            sld.deleteLater()
            btnp.deleteLater()
            lbl.deleteLater()
        self._rows.clear()

        while self.grid.count():
            item = self.grid.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()

        # Header
        self.grid.addWidget(self._bold("Joint"),     0, 0)
        self.grid.addWidget(self._bold("Decrease"),  0, 1)
        self.grid.addWidget(self._bold("Position"),  0, 2)
        self.grid.addWidget(self._bold("Increase"),  0, 3)
        self.grid.addWidget(self._bold("Value (°)"), 0, 4)

        # Save model
        self._joint_names = list(names)
        self._joint_lims_deg = list(limits_deg)

        # Build rows
        for row_idx, name in enumerate(self._joint_names, start=1):
            mn, mx = self._joint_lims_deg[row_idx - 1]

            self.grid.addWidget(QLabel(name, self), row_idx, 0)

            btnm = QPushButton("−", self)
            btnp = QPushButton("+", self)

            sld = QSlider(Qt.Orientation.Horizontal, self)
            sld.setMinimum(int(mn * 10))
            sld.setMaximum(int(mx * 10))

            lblVal = QLabel("-", self)
            lblVal.setAlignment(Qt.AlignmentFlag.AlignCenter)

            self.grid.addWidget(btnm, row_idx, 1)
            self.grid.addWidget(sld,  row_idx, 2)
            self.grid.addWidget(btnp, row_idx, 3)
            self.grid.addWidget(lblVal, row_idx, 4)

            for b in (btnm, btnp):
                b.setAutoRepeat(True)
                b.setAutoRepeatDelay(300)
                b.setAutoRepeatInterval(80)

            idx = row_idx - 1
            btnm.clicked.connect(lambda _, i=idx: self._emit_joint_step(i, True))
            btnp.clicked.connect(lambda _, i=idx: self._emit_joint_step(i, False))
            sld.valueChanged.connect(lambda _, i=idx: self._on_slider_changed(i))

            self._rows.append((btnm, sld, btnp, lblVal))

        self.grid.setColumnStretch(2, 1)

    # ---------------- Joint Feedback -----------------
    def set_joint_positions_deg(self, positions_deg):
        for i, v in enumerate(positions_deg):
            if i >= len(self._rows):
                break
            _, sld, _, lbl = self._rows[i]
            sld.blockSignals(True)
            sld.setValue(int(v * 10))
            sld.blockSignals(False)
            lbl.setText(f"{v:.2f}")

    @QtCore.pyqtSlot(object)
    def update_from_joint_state(self, msg):
        if not msg:
            return
        names = getattr(msg, "name", None)
        pos   = getattr(msg, "position", None)
        if not names or not pos:
            return

        name_to_pos = {n: float(p) for n, p in zip(names, pos)}
        deg_list = [math.degrees(name_to_pos.get(j, 0.0)) for j in self._joint_names]
        self.set_joint_positions_deg(deg_list)

    # ---------------- Emitters -----------------
    def _emit_params(self):
        self.paramsChanged.emit(self.get_params())

    def _emit_joint_step(self, idx, negative):
        if idx < 0 or idx >= len(self._rows):
            return
        name = self._joint_names[idx]
        step = float(self.spinStepDeg.value())
        delta = -step if negative else step
        speed = float(self.spinSpeedPct.value())
        self.jointJogRequested.emit(name, delta, speed)

    def _on_slider_changed(self, idx):
        _, sld, _, lbl = self._rows[idx]
        lbl.setText(f"{sld.value()/10.0:.2f}")

    def _apply_vertical_max(self):
        for w in [self] + self.findChildren(QWidget):
            sp = w.sizePolicy()
            sp.setVerticalPolicy(QSizePolicy.Policy.Maximum)
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            w.setSizePolicy(sp)



# =============================================================================
# CartesianJogWidget – MIT ctx.content
# =============================================================================
class CartesianJogWidget(QWidget):
    frameChanged = QtCore.pyqtSignal(str)
    paramsChanged = QtCore.pyqtSignal(dict)
    cartesianJogRequested = QtCore.pyqtSignal(str, float, float, str)

    def __init__(self, ctx, bridge=None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.content = ctx.content        # Frames/QoS/Topics
        self.bridge = bridge

        self._build_ui()
        self._wire_signals()
        self._try_bind_to_bridge()

        # Default params
        self.set_params({
            "cart_lin_step_mm": 1.0,
            "cart_ang_step_deg": 2.0,
            "cart_speed_mm_s": 50.0,
            "frame": "wrf",
        })

        self._apply_vertical_max()

    # ----- Frame mapping UI → real frame in frames.yaml -----
    def _map_ui_frame(self, ui: str) -> str:
        if ui == "trf":
            return self.content.frame("tcp")
        return self.content.frame("world")

    # ---------------- UI Aufbau ----------------
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)

        form = QFormLayout()
        self.spinLinStep = QDoubleSpinBox(self)
        self.spinLinStep.setRange(0.1, 200.0)
        self.spinLinStep.setDecimals(1)
        self.spinLinStep.setSuffix(" mm")

        self.spinAngStep = QDoubleSpinBox(self)
        self.spinAngStep.setRange(0.1, 45.0)
        self.spinAngStep.setDecimals(1)
        self.spinAngStep.setSuffix(" °")

        self.spinSpeed = QDoubleSpinBox(self)
        self.spinSpeed.setRange(1.0, 2000.0)
        self.spinSpeed.setSuffix(" mm/s")

        form.addRow("Linear step (mm)", self.spinLinStep)
        form.addRow("Angular step (°)", self.spinAngStep)
        form.addRow("Speed (mm/s)", self.spinSpeed)
        root.addLayout(form)

        # Frame selector
        fr = QHBoxLayout()
        fr.addWidget(QLabel("Frame:", self))
        self.rbWRF = QRadioButton("WRF (World)", self)
        self.rbTRF = QRadioButton("TRF (Tool)", self)
        self.rbWRF.setChecked(True)
        fr.addWidget(self.rbWRF)
        fr.addWidget(self.rbTRF)
        fr.addStretch(1)
        root.addLayout(fr)

        # Linear buttons
        grid1 = QGridLayout()
        grid1.addWidget(self._bold("Linear"), 0, 0)
        self.btnXm = QPushButton("−X")
        self.btnXp = QPushButton("+X")
        self.btnYm = QPushButton("−Y")
        self.btnYp = QPushButton("+Y")
        self.btnZm = QPushButton("−Z")
        self.btnZp = QPushButton("+Z")
        grid1.addWidget(self.btnXm, 1, 0)
        grid1.addWidget(self.btnXp, 1, 1)
        grid1.addWidget(self.btnYm, 2, 0)
        grid1.addWidget(self.btnYp, 2, 1)
        grid1.addWidget(self.btnZm, 3, 0)
        grid1.addWidget(self.btnZp, 3, 1)
        root.addLayout(grid1)

        # Rotational
        grid2 = QGridLayout()
        grid2.addWidget(self._bold("Rotational"), 0, 0)
        self.btnRxm = QPushButton("−RX")
        self.btnRxp = QPushButton("+RX")
        self.btnRym = QPushButton("−RY")
        self.btnRyp = QPushButton("+RY")
        self.btnRzm = QPushButton("−RZ")
        self.btnRzp = QPushButton("+RZ")
        grid2.addWidget(self.btnRxm, 1, 0)
        grid2.addWidget(self.btnRxp, 1, 1)
        grid2.addWidget(self.btnRym, 2, 0)
        grid2.addWidget(self.btnRyp, 2, 1)
        grid2.addWidget(self.btnRzm, 3, 0)
        grid2.addWidget(self.btnRzp, 3, 1)
        root.addLayout(grid2)

        for b in (
            self.btnXm, self.btnXp, self.btnYm, self.btnYp,
            self.btnZm, self.btnZp, self.btnRxm, self.btnRxp,
            self.btnRym, self.btnRyp, self.btnRzm, self.btnRzp
        ):
            b.setAutoRepeat(True)
            b.setAutoRepeatDelay(300)
            b.setAutoRepeatInterval(80)

        root.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

    def _bold(self, txt):
        lbl = QLabel(txt, self)
        lbl.setStyleSheet("font-weight:600;")
        return lbl

    # ---------------- Signals ----------------
    def _wire_signals(self):
        self.spinLinStep.valueChanged.connect(lambda _: self._emit_params())
        self.spinAngStep.valueChanged.connect(lambda _: self._emit_params())
        self.spinSpeed.valueChanged.connect(lambda _: self._emit_params())

        self.rbWRF.toggled.connect(lambda _: self._on_frame_changed())
        self.rbTRF.toggled.connect(lambda _: self._on_frame_changed())

        # Linear jog
        self.btnXm.clicked.connect(lambda: self._emit_cart("x", -self.spinLinStep.value()))
        self.btnXp.clicked.connect(lambda: self._emit_cart("x",  self.spinLinStep.value()))
        self.btnYm.clicked.connect(lambda: self._emit_cart("y", -self.spinLinStep.value()))
        self.btnYp.clicked.connect(lambda: self._emit_cart("y",  self.spinLinStep.value()))
        self.btnZm.clicked.connect(lambda: self._emit_cart("z", -self.spinLinStep.value()))
        self.btnZp.clicked.connect(lambda: self._emit_cart("z",  self.spinLinStep.value()))

        # Rotational jog
        self.btnRxm.clicked.connect(lambda: self._emit_rot("rx", -self.spinAngStep.value()))
        self.btnRxp.clicked.connect(lambda: self._emit_rot("rx",  self.spinAngStep.value()))
        self.btnRym.clicked.connect(lambda: self._emit_rot("ry", -self.spinAngStep.value()))
        self.btnRyp.clicked.connect(lambda: self._emit_rot("ry",  self.spinAngStep.value()))
        self.btnRzm.clicked.connect(lambda: self._emit_rot("rz", -self.spinAngStep.value()))
        self.btnRzp.clicked.connect(lambda: self._emit_rot("rz",  self.spinAngStep.value()))

    def _try_bind_to_bridge(self):
        if not self.bridge:
            return
        target = None
        for attr in ("_servo", "_motion"):
            obj = getattr(self.bridge, attr, None)
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

    # ---------------- Public API ----------------
    def set_params(self, cfg):
        if "cart_lin_step_mm" in cfg:
            self.spinLinStep.setValue(cfg["cart_lin_step_mm"])
        if "cart_ang_step_deg" in cfg:
            self.spinAngStep.setValue(cfg["cart_ang_step_deg"])
        if "cart_speed_mm_s" in cfg:
            self.spinSpeed.setValue(cfg["cart_speed_mm_s"])
        if "frame" in cfg:
            self.set_frame(cfg["frame"])
        self._emit_params()

    def get_params(self) -> Dict[str, Any]:
        return {
            "cart_lin_step_mm": float(self.spinLinStep.value()),
            "cart_ang_step_deg": float(self.spinAngStep.value()),
            "cart_speed_mm_s": float(self.spinSpeed.value()),
            "frame": self.get_frame(),
        }

    def set_frame(self, f: str):
        if f.strip().lower() == "trf":
            self.rbTRF.setChecked(True)
        else:
            self.rbWRF.setChecked(True)

    def get_frame(self) -> str:
        return "trf" if self.rbTRF.isChecked() else "wrf"

    # ---------------- intern ----------------
    def _emit_params(self):
        self.paramsChanged.emit(self.get_params())

    def _on_frame_changed(self):
        self.frameChanged.emit(self.get_frame())
        self._emit_params()

    def _emit_cart(self, axis: str, delta_mm: float):
        frame = self._map_ui_frame(self.get_frame())
        speed = float(self.spinSpeed.value())
        self.cartesianJogRequested.emit(axis, float(delta_mm), speed, frame)

    def _emit_rot(self, axis: str, delta_deg: float):
        frame = self._map_ui_frame(self.get_frame())
        speed = float(self.spinSpeed.value())
        self.cartesianJogRequested.emit(axis, float(delta_deg), speed, frame)

    def _apply_vertical_max(self):
        for w in [self] + self.findChildren(QWidget):
            sp = w.sizePolicy()
            sp.setVerticalPolicy(QSizePolicy.Maximum)
            sp.setHorizontalPolicy(QSizePolicy.Expanding)
            w.setSizePolicy(sp)
