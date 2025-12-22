# -*- coding: utf-8 -*-
# File: tabs/service/servo_widgets.py
from __future__ import annotations

from typing import Optional, List, Tuple, Dict, Any
import math
import logging

from PyQt6 import QtCore
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QGridLayout,
    QSizePolicy, QFormLayout, QDoubleSpinBox, QSlider, QSpacerItem, QRadioButton,
    QLayout
)

_LOG = logging.getLogger("app.tabs.service.servo_widgets")


# =============================================================================
# Helpers
# =============================================================================

def _clear_layout(layout: QLayout) -> None:
    """Safely clears any Qt layout (also nested layouts) without double-delete."""
    while layout.count() > 0:
        item = layout.takeAt(0)
        if item is None:
            continue

        w = item.widget()
        if w is not None:
            w.deleteLater()
            continue

        l = item.layout()
        if l is not None:
            _clear_layout(l)
            continue

        sp = item.spacerItem()
        if sp is not None:
            # SpacerItems are not widgets/layouts; dropping reference is enough.
            del sp


def _find_servo_bridge(bridge: Any, *, log_prefix: str):
    """
    Robust: findet ServoBridge unabhängig davon ob neue Property-API oder _servo verwendet wird.
    Erwartet auf Bridge-Seite: .servo_bridge (Property) oder ._servo (fallback), jeweils mit .signals.
    """
    if bridge is None:
        _LOG.warning("[%s] bridge=None -> ServoBridge wiring skipped.", log_prefix)
        return None

    ensure = getattr(bridge, "ensure_connected", None)
    if callable(ensure):
        try:
            ensure()
        except Exception as e:
            _LOG.warning("[%s] bridge.ensure_connected() failed: %s", log_prefix, e)

    if hasattr(bridge, "servo_bridge"):
        try:
            b = bridge.servo_bridge
            sig = getattr(b, "signals", None)
            if sig is None:
                _LOG.error("[%s] bridge.servo_bridge exists but has no .signals", log_prefix)
                return None
            return b
        except AssertionError as e:
            _LOG.warning("[%s] bridge.servo_bridge not available (not connected?): %s", log_prefix, e)
        except Exception as e:
            _LOG.exception("[%s] error accessing bridge.servo_bridge: %s", log_prefix, e)

    b = getattr(bridge, "_servo", None)
    if b is not None and getattr(b, "signals", None) is not None:
        _LOG.info("[%s] ServoBridge found via bridge._servo (fallback).", log_prefix)
        return b

    _LOG.error(
        "[%s] No ServoBridge found. has servo_bridge=%s, _servo=%r",
        log_prefix,
        hasattr(bridge, "servo_bridge"),
        getattr(bridge, "_servo", None),
    )
    return None


# =============================================================================
# JointJogWidget – Joint-Jogging
# =============================================================================
class JointJogWidget(QWidget):
    paramsChanged = QtCore.pyqtSignal(dict)
    jointJogRequested = QtCore.pyqtSignal(str, float, float)  # joint_name, delta_deg, speed_pct

    def __init__(self, ctx, bridge=None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.content = getattr(ctx, "content", None)
        self.bridge = bridge  # UIBridge (optional)

        self._servo_wired: bool = False
        self._wired_warned: bool = False

        # debounced wiring attempts
        self._wire_tries: int = 0
        self._wire_timer: Optional[QtCore.QTimer] = None

        self._joint_names: List[str] = []
        self._joint_lims_deg: List[Tuple[float, float]] = []
        self._rows: List[Tuple[QPushButton, QSlider, QPushButton, QLabel]] = []

        self._build_ui()
        self._wire_signals()

        self._try_wire_debounced()

        self.set_params({"joint_step_deg": 2.0, "joint_speed_pct": 40.0})

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

    # ------------------------------------------------------------------ Bridge wiring
    def _try_wire_debounced(self) -> None:
        """Avoid spamming logs by retrying wiring with a timer (max N tries)."""
        if self._servo_wired:
            return

        if self._wire_timer is None:
            self._wire_timer = QtCore.QTimer(self)
            self._wire_timer.setSingleShot(True)
            self._wire_timer.timeout.connect(self._wire_outbound_to_bridge_if_present)

        if not self._wire_timer.isActive():
            self._wire_timer.start(150)

    def _wire_outbound_to_bridge_if_present(self) -> None:
        if self._servo_wired:
            return

        self._wire_tries += 1
        servo_bridge = _find_servo_bridge(self.bridge, log_prefix="JointJogWidget")
        if servo_bridge is None:
            if self._wire_tries < 40:
                self._schedule_retry()
            return

        sig = getattr(servo_bridge, "signals", None)
        if sig is None:
            _LOG.error("[JointJogWidget] ServoBridge found, but signals=None -> wiring aborted.")
            if self._wire_tries < 40:
                self._schedule_retry()
            return

        ok = True

        if hasattr(sig, "paramsChanged"):
            self.paramsChanged.connect(sig.paramsChanged)
        else:
            _LOG.error("[JointJogWidget] ServoSignals missing: paramsChanged")
            ok = False

        if hasattr(sig, "jointJogRequested"):
            self.jointJogRequested.connect(sig.jointJogRequested)
        else:
            _LOG.error("[JointJogWidget] ServoSignals missing: jointJogRequested")
            ok = False

        self._servo_wired = bool(ok)
        if self._servo_wired:
            _LOG.info("[JointJogWidget] ✅ wired -> ServoBridge.signals (%r)", servo_bridge)
            self._emit_params()
        else:
            _LOG.error("[JointJogWidget] wiring failed (missing signals).")
            if self._wire_tries < 40:
                self._schedule_retry()

    def _schedule_retry(self) -> None:
        if self._wire_timer is None:
            return
        # backoff: up to 1s
        delay = min(1000, 150 + self._wire_tries * 25)
        if not self._wire_timer.isActive():
            self._wire_timer.start(delay)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        form = QFormLayout()
        self.spinStepDeg = QDoubleSpinBox(self)
        self.spinStepDeg.setRange(0.1, 45.0)
        self.spinStepDeg.setDecimals(1)
        self.spinStepDeg.setSingleStep(0.1)

        self.spinSpeedPct = QDoubleSpinBox(self)
        self.spinSpeedPct.setRange(1.0, 100.0)
        self.spinSpeedPct.setDecimals(1)
        self.spinSpeedPct.setSingleStep(1.0)
        self.spinSpeedPct.setSuffix(" %")

        form.addRow("Step (°)", self.spinStepDeg)
        form.addRow("Speed (%)", self.spinSpeedPct)
        root.addLayout(form)

        self.grid = QGridLayout()
        self.grid.setHorizontalSpacing(10)
        self.grid.setVerticalSpacing(6)

        self.grid.addWidget(self._bold("Joint"),     0, 0)
        self.grid.addWidget(self._bold("Decrease"),  0, 1)
        self.grid.addWidget(self._bold("Position"),  0, 2)
        self.grid.addWidget(self._bold("Increase"),  0, 3)
        self.grid.addWidget(self._bold("Value (°)"), 0, 4)

        root.addLayout(self.grid)
        root.addItem(QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))

    def _bold(self, txt: str) -> QLabel:
        l = QLabel(txt, self)
        l.setStyleSheet("font-weight:600;")
        return l

    def _wire_signals(self):
        self.spinStepDeg.valueChanged.connect(lambda _v: self._emit_params())
        self.spinSpeedPct.valueChanged.connect(lambda _v: self._emit_params())

    # ------------------------------------------------------------------ Params API
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

    # ------------------------------------------------------------------ Joint table
    def set_joint_model(self, names, limits_deg):
        _clear_layout(self.grid)
        self._rows.clear()

        self.grid.addWidget(self._bold("Joint"),     0, 0)
        self.grid.addWidget(self._bold("Decrease"),  0, 1)
        self.grid.addWidget(self._bold("Position"),  0, 2)
        self.grid.addWidget(self._bold("Increase"),  0, 3)
        self.grid.addWidget(self._bold("Value (°)"), 0, 4)

        self._joint_names = list(names or [])
        self._joint_lims_deg = list(limits_deg or [])

        for row_idx, name in enumerate(self._joint_names, start=1):
            mn, mx = (
                self._joint_lims_deg[row_idx - 1]
                if (row_idx - 1) < len(self._joint_lims_deg)
                else (-180.0, 180.0)
            )

            self.grid.addWidget(QLabel(str(name), self), row_idx, 0)

            btnm = QPushButton("−", self)
            btnp = QPushButton("+", self)

            sld = QSlider(Qt.Orientation.Horizontal, self)
            sld.setMinimum(int(float(mn) * 10))
            sld.setMaximum(int(float(mx) * 10))

            sld.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            sld.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
            sld.setTracking(False)
            sld.setSingleStep(0)
            sld.setPageStep(0)

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
            btnm.clicked.connect(lambda _=False, i=idx: self._emit_joint_step(i, True))
            btnp.clicked.connect(lambda _=False, i=idx: self._emit_joint_step(i, False))

            self._rows.append((btnm, sld, btnp, lblVal))

        self.grid.setColumnStretch(2, 1)

    def set_joint_positions_deg(self, positions_deg):
        """Setzt UI-Anzeige (Slider + Label). Slider bleibt read-only."""
        for i, v in enumerate(positions_deg or []):
            if i >= len(self._rows):
                break
            _, sld, _, lbl = self._rows[i]
            mn = sld.minimum()
            mx = sld.maximum()
            vv = int(float(v) * 10)
            vv = max(mn, min(mx, vv))
            sld.blockSignals(True)
            sld.setValue(vv)
            sld.blockSignals(False)
            lbl.setText(f"{vv / 10.0:.2f}")

    @QtCore.pyqtSlot(object)
    def update_from_joint_state(self, msg):
        """
        ROS JointState -> Anzeige.
        Erwartet: msg.position in rad (typisch ROS) -> wird in deg umgerechnet.
        """
        if not msg:
            return
        names = getattr(msg, "name", None)
        pos = getattr(msg, "position", None)
        if not names or not pos:
            return
        name_to_pos = {str(n): float(p) for n, p in zip(names, pos)}
        deg_list = [math.degrees(name_to_pos.get(j, 0.0)) for j in self._joint_names]
        self.set_joint_positions_deg(deg_list)

    # ------------------------------------------------------------------ Internals
    def _emit_params(self):
        if not self._servo_wired:
            self._try_wire_debounced()
        self.paramsChanged.emit(self.get_params())

    def _emit_joint_step(self, idx, negative):
        if idx < 0 or idx >= len(self._rows):
            return

        if not self._servo_wired:
            self._try_wire_debounced()
            if not self._servo_wired and not self._wired_warned:
                _LOG.warning("[JointJogWidget] Jog pressed but ServoBridge not wired.")
                self._wired_warned = True

        name = self._joint_names[idx]
        step = float(self.spinStepDeg.value())
        delta = -step if negative else step
        speed = float(self.spinSpeedPct.value())
        self.jointJogRequested.emit(str(name), float(delta), float(speed))

    def _apply_vertical_max(self):
        for w in [self] + self.findChildren(QWidget):
            sp = w.sizePolicy()
            sp.setVerticalPolicy(QSizePolicy.Policy.Maximum)
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            w.setSizePolicy(sp)


# =============================================================================
# CartesianJogWidget – sendet frame als "world"/"tcp" (kein wrf/trf mehr)
# =============================================================================
class CartesianJogWidget(QWidget):
    frameChanged = QtCore.pyqtSignal(str)  # "world" / "tcp"
    paramsChanged = QtCore.pyqtSignal(dict)
    cartesianJogRequested = QtCore.pyqtSignal(str, float, float, str)  # axis, delta, speed, frame

    def __init__(self, ctx, bridge=None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.content = getattr(ctx, "content", None)
        self.bridge = bridge  # UIBridge (optional)

        self._servo_wired: bool = False
        self._wired_warned: bool = False

        # debounced wiring attempts
        self._wire_tries: int = 0
        self._wire_timer: Optional[QtCore.QTimer] = None

        self._build_ui()
        self._wire_signals()

        self._try_wire_debounced()

        self.set_params({
            "cart_lin_step_mm": 1.0,
            "cart_ang_step_deg": 2.0,
            "cart_speed_mm_s": 50.0,
            "cart_speed_deg_s": 30.0,
            "frame": "world",
        })

        self._apply_vertical_max()

    # ------------------------------------------------------------------ Bridge wiring
    def _try_wire_debounced(self) -> None:
        if self._servo_wired:
            return

        if self._wire_timer is None:
            self._wire_timer = QtCore.QTimer(self)
            self._wire_timer.setSingleShot(True)
            self._wire_timer.timeout.connect(self._wire_outbound_to_bridge_if_present)

        if not self._wire_timer.isActive():
            self._wire_timer.start(150)

    def _wire_outbound_to_bridge_if_present(self) -> None:
        if self._servo_wired:
            return

        self._wire_tries += 1
        servo_bridge = _find_servo_bridge(self.bridge, log_prefix="CartesianJogWidget")
        if servo_bridge is None:
            if self._wire_tries < 40:
                self._schedule_retry()
            return

        sig = getattr(servo_bridge, "signals", None)
        if sig is None:
            _LOG.error("[CartesianJogWidget] ServoBridge found, but signals=None -> wiring aborted.")
            if self._wire_tries < 40:
                self._schedule_retry()
            return

        ok = True

        if hasattr(sig, "frameChanged"):
            self.frameChanged.connect(sig.frameChanged)
        else:
            _LOG.error("[CartesianJogWidget] ServoSignals missing: frameChanged")
            ok = False

        if hasattr(sig, "paramsChanged"):
            self.paramsChanged.connect(sig.paramsChanged)
        else:
            _LOG.error("[CartesianJogWidget] ServoSignals missing: paramsChanged")
            ok = False

        if hasattr(sig, "cartesianJogRequested"):
            self.cartesianJogRequested.connect(sig.cartesianJogRequested)
        else:
            _LOG.error("[CartesianJogWidget] ServoSignals missing: cartesianJogRequested")
            ok = False

        self._servo_wired = bool(ok)
        if self._servo_wired:
            _LOG.info("[CartesianJogWidget] ✅ wired -> ServoBridge.signals (%r)", servo_bridge)
            self._on_frame_changed()
        else:
            _LOG.error("[CartesianJogWidget] wiring failed (missing signals).")
            if self._wire_tries < 40:
                self._schedule_retry()

    def _schedule_retry(self) -> None:
        if self._wire_timer is None:
            return
        delay = min(1000, 150 + self._wire_tries * 25)
        if not self._wire_timer.isActive():
            self._wire_timer.start(delay)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)

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

        # ✅ separate speeds (fix semantics)
        self.spinSpeedLin = QDoubleSpinBox(self)
        self.spinSpeedLin.setRange(1.0, 5000.0)
        self.spinSpeedLin.setDecimals(1)
        self.spinSpeedLin.setSingleStep(5.0)
        self.spinSpeedLin.setSuffix(" mm/s")

        self.spinSpeedRot = QDoubleSpinBox(self)
        self.spinSpeedRot.setRange(1.0, 360.0)
        self.spinSpeedRot.setDecimals(1)
        self.spinSpeedRot.setSingleStep(1.0)
        self.spinSpeedRot.setSuffix(" °/s")

        form.addRow("Linear step (mm)", self.spinLinStep)
        form.addRow("Angular step (°)", self.spinAngStep)
        form.addRow("Speed lin (mm/s)", self.spinSpeedLin)
        form.addRow("Speed rot (°/s)", self.spinSpeedRot)
        root.addLayout(form)

        fr = QHBoxLayout()
        fr.addWidget(QLabel("Frame:", self))
        # UI-Text darf bleiben, aber intern senden wir world/tcp
        self.rbWRF = QRadioButton("World", self)
        self.rbTRF = QRadioButton("TCP", self)
        self.rbWRF.setChecked(True)
        fr.addWidget(self.rbWRF)
        fr.addWidget(self.rbTRF)
        fr.addStretch(1)
        root.addLayout(fr)

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

        root.addItem(QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))

    def _bold(self, txt):
        lbl = QLabel(txt, self)
        lbl.setStyleSheet("font-weight:600;")
        return lbl

    def _wire_signals(self):
        self.spinLinStep.valueChanged.connect(lambda _v: self._emit_params())
        self.spinAngStep.valueChanged.connect(lambda _v: self._emit_params())
        self.spinSpeedLin.valueChanged.connect(lambda _v: self._emit_params())
        self.spinSpeedRot.valueChanged.connect(lambda _v: self._emit_params())

        # ✅ FIX: toggled feuert doppelt (TRF True + WRF False). Wir reagieren nur auf checked=True.
        self.rbWRF.toggled.connect(self._on_frame_toggled)
        self.rbTRF.toggled.connect(self._on_frame_toggled)

        self.btnXm.clicked.connect(lambda _=False: self._emit_cart("x", -self.spinLinStep.value()))
        self.btnXp.clicked.connect(lambda _=False: self._emit_cart("x",  self.spinLinStep.value()))
        self.btnYm.clicked.connect(lambda _=False: self._emit_cart("y", -self.spinLinStep.value()))
        self.btnYp.clicked.connect(lambda _=False: self._emit_cart("y",  self.spinLinStep.value()))
        self.btnZm.clicked.connect(lambda _=False: self._emit_cart("z", -self.spinLinStep.value()))
        self.btnZp.clicked.connect(lambda _=False: self._emit_cart("z",  self.spinLinStep.value()))

        self.btnRxm.clicked.connect(lambda _=False: self._emit_rot("rx", -self.spinAngStep.value()))
        self.btnRxp.clicked.connect(lambda _=False: self._emit_rot("rx",  self.spinAngStep.value()))
        self.btnRym.clicked.connect(lambda _=False: self._emit_rot("ry", -self.spinAngStep.value()))
        self.btnRyp.clicked.connect(lambda _=False: self._emit_rot("ry",  self.spinAngStep.value()))
        self.btnRzm.clicked.connect(lambda _=False: self._emit_rot("rz", -self.spinAngStep.value()))
        self.btnRzp.clicked.connect(lambda _=False: self._emit_rot("rz",  self.spinAngStep.value()))

    @QtCore.pyqtSlot(bool)
    def _on_frame_toggled(self, checked: bool) -> None:
        # ✅ verhindert Doppel-Call / Signal-Sturm
        if not checked:
            return
        self._on_frame_changed()

    # ------------------------------------------------------------------ Params API
    def set_params(self, cfg):
        if "cart_lin_step_mm" in cfg:
            self.spinLinStep.setValue(float(cfg["cart_lin_step_mm"]))
        if "cart_ang_step_deg" in cfg:
            self.spinAngStep.setValue(float(cfg["cart_ang_step_deg"]))
        if "cart_speed_mm_s" in cfg:
            self.spinSpeedLin.setValue(float(cfg["cart_speed_mm_s"]))
        if "cart_speed_deg_s" in cfg:
            self.spinSpeedRot.setValue(float(cfg["cart_speed_deg_s"]))
        if "frame" in cfg:
            self.set_frame(str(cfg["frame"]))
        self._emit_params()

    def get_params(self) -> Dict[str, Any]:
        return {
            "cart_lin_step_mm": float(self.spinLinStep.value()),
            "cart_ang_step_deg": float(self.spinAngStep.value()),
            "cart_speed_mm_s": float(self.spinSpeedLin.value()),
            "cart_speed_deg_s": float(self.spinSpeedRot.value()),
            "frame": self.get_frame(),  # world/tcp
        }

    def set_frame(self, f: str):
        """Accepts 'world' or 'tcp' (legacy 'wrf'/'trf' tolerated)."""
        raw = (f or "").strip().lower()

        # tolerate legacy
        if raw == "trf":
            raw = "tcp"
        elif raw == "wrf":
            raw = "world"

        target_tcp = (raw == "tcp")

        # ✅ FIX: avoid feedback loop when setting checked programmatically
        b1 = QtCore.QSignalBlocker(self.rbWRF)
        b2 = QtCore.QSignalBlocker(self.rbTRF)
        try:
            self.rbTRF.setChecked(target_tcp)
            self.rbWRF.setChecked(not target_tcp)
        finally:
            del b1, b2

    def get_frame(self) -> str:
        return "tcp" if self.rbTRF.isChecked() else "world"

    # ------------------------------------------------------------------ Emit
    def _emit_params(self):
        if not self._servo_wired:
            self._try_wire_debounced()
        self.paramsChanged.emit(self.get_params())

    def _on_frame_changed(self):
        if not self._servo_wired:
            self._try_wire_debounced()
        self.frameChanged.emit(self.get_frame())  # world/tcp
        self._emit_params()

    def _emit_cart(self, axis: str, delta_mm: float):
        if not self._servo_wired:
            self._try_wire_debounced()
            if not self._servo_wired and not self._wired_warned:
                _LOG.warning("[CartesianJogWidget] Jog pressed but ServoBridge not wired.")
                self._wired_warned = True

        frame = self.get_frame()  # world/tcp
        speed_mm_s = float(self.spinSpeedLin.value())
        self.cartesianJogRequested.emit(str(axis), float(delta_mm), speed_mm_s, frame)

    def _emit_rot(self, axis: str, delta_deg: float):
        if not self._servo_wired:
            self._try_wire_debounced()
            if not self._servo_wired and not self._wired_warned:
                _LOG.warning("[CartesianJogWidget] Jog pressed but ServoBridge not wired.")
                self._wired_warned = True

        frame = self.get_frame()  # world/tcp
        speed_deg_s = float(self.spinSpeedRot.value())
        self.cartesianJogRequested.emit(str(axis), float(delta_deg), speed_deg_s, frame)

    def _apply_vertical_max(self):
        for w in [self] + self.findChildren(QWidget):
            sp = w.sizePolicy()
            sp.setVerticalPolicy(QSizePolicy.Policy.Maximum)
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            w.setSizePolicy(sp)
