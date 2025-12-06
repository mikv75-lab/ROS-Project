# src/app/widgets/omron_tcp_widget.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional
import html

from PyQt6 import QtCore, QtWidgets

from ros.bridge.ui_bridge import UIBridge


class OmronTcpWidget(QtWidgets.QWidget):
    """
    Einfaches TCP-Client-Widget für den Omron ACE-Server.

    Erwartet:
        ui_bridge._omron -> OmronBridge mit:
            - signals.rawTxChanged(str)
            - signals.rawRxChanged(str)
            - signals.connectionChanged(bool)
            - Methode send_command(str)
    """

    def __init__(self, ui_bridge: UIBridge, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self._bridge = ui_bridge
        self._omron = getattr(ui_bridge, "_omron", None)

        # letzter bekannter Connection-Status (für Change-Detection)
        self._last_connected: Optional[bool] = None

        self._build_ui()
        self._wire_signals()

        # Initialer Status (nur UI setzen, kein Log-Spam)
        self._update_connection_indicator(False)

    # ------------------------------------------------------------------
    # UI-Aufbau
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        self.setObjectName("OmronTcpWidget")

        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(6)

        # --- Header: Titel + Status ---
        header_layout = QtWidgets.QHBoxLayout()
        header_label = QtWidgets.QLabel("Omron ACE TCP Client", self)
        header_label.setStyleSheet("font-weight: bold;")

        self._status_label = QtWidgets.QLabel("DISCONNECTED", self)
        self._status_label.setAlignment(
            QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter
        )

        header_layout.addWidget(header_label)
        header_layout.addStretch(1)
        header_layout.addWidget(self._status_label)

        # --- Eingabezeile: Command + Buttons ---
        cmd_layout = QtWidgets.QHBoxLayout()
        self._cmd_edit = QtWidgets.QLineEdit(self)
        self._cmd_edit.setPlaceholderText("Command eingeben, z.B. 'PING' ...")
        self._send_btn = QtWidgets.QPushButton("Send", self)
        self._clear_btn = QtWidgets.QPushButton("Clear", self)

        cmd_layout.addWidget(self._cmd_edit, 1)
        cmd_layout.addWidget(self._send_btn)
        cmd_layout.addWidget(self._clear_btn)

        # --- Gemeinsames Log-Fenster für TX/RX ---
        self._log_view = QtWidgets.QTextEdit(self)
        self._log_view.setReadOnly(True)
        self._log_view.setLineWrapMode(QtWidgets.QTextEdit.LineWrapMode.NoWrap)

        main_layout.addLayout(header_layout)
        main_layout.addLayout(cmd_layout)
        main_layout.addWidget(self._log_view, 1)

    # ------------------------------------------------------------------
    # Wiring
    # ------------------------------------------------------------------
    def _wire_signals(self) -> None:
        # UI-Interaktion
        self._send_btn.clicked.connect(self._on_send_clicked)
        self._clear_btn.clicked.connect(self._on_clear_clicked)
        self._cmd_edit.returnPressed.connect(self._on_send_clicked)

        # OmronBridge-Signale (falls vorhanden)
        if self._omron is None or not hasattr(self._omron, "signals"):
            self._append_log("SYS", "[WARN] OmronBridge nicht verfügbar.")
            return

        sig = self._omron.signals

        # raw_tx -> Log
        if hasattr(sig, "rawTxChanged"):
            sig.rawTxChanged.connect(lambda txt: self._append_log("TX", txt))

        # raw_rx -> Log
        if hasattr(sig, "rawRxChanged"):
            sig.rawRxChanged.connect(lambda txt: self._append_log("RX", txt))

        # connectionChanged -> Status (falls vorhanden)
        if hasattr(sig, "connectionChanged"):
            sig.connectionChanged.connect(self._update_connection_indicator)

    # ------------------------------------------------------------------
    # Helper: Log + Status
    # ------------------------------------------------------------------
    def _current_time_str(self) -> str:
        return QtCore.QTime.currentTime().toString("HH:mm:ss")

    def _append_log(self, direction: str, text: str) -> None:
        """
        direction: "TX", "RX" oder "SYS"
        """
        text = (text or "").rstrip()
        if not text:
            return

        ts = self._current_time_str()

        # Farben
        if direction == "TX":
            color = "#0077cc"   # blau
        elif direction == "RX":
            color = "#00aa00"   # grün
        else:
            color = "#888888"   # grau für System

        escaped = html.escape(text)
        line = (
            f'<span style="color:#666666;">[{ts}]</span> '
            f'<span style="color:{color};">[{direction}] {escaped}</span>'
        )

        self._log_view.append(line)

    @QtCore.pyqtSlot(bool)
    def _update_connection_indicator(self, connected: bool) -> None:
        # Nur reagieren, wenn sich der Status geändert hat
        if self._last_connected is not None and connected == self._last_connected:
            # Zustand ist derselbe wie vorher -> keine Log-Zeile schreiben
            return

        self._last_connected = connected

        if connected:
            self._status_label.setText("CONNECTED")
            self._status_label.setStyleSheet("color: #00aa00; font-weight: bold;")
            self._append_log("SYS", "Verbunden mit ACE-Server.")
        else:
            self._status_label.setText("DISCONNECTED")
            self._status_label.setStyleSheet("color: #aa0000; font-weight: bold;")
            self._append_log("SYS", "Verbindung getrennt.")

    # ------------------------------------------------------------------
    # Slots: Buttons
    # ------------------------------------------------------------------
    @QtCore.pyqtSlot()
    def _on_send_clicked(self) -> None:
        cmd = self._cmd_edit.text().strip()
        if not cmd:
            return

        if self._omron is None:
            self._append_log("TX", f"(kein OmronBridge) {cmd}")
            self._cmd_edit.clear()
            return

        # bevorzugt: Methode send_command()
        if hasattr(self._omron, "send_command"):
            try:
                self._omron.send_command(cmd)
            except Exception as e:
                self._append_log("SYS", f"[ERROR] send_command failed: {e}")
        # Fallback: Signal commandRequested, falls vorhanden
        elif hasattr(self._omron, "signals") and hasattr(self._omron.signals, "commandRequested"):
            try:
                self._omron.signals.commandRequested.emit(cmd)
            except Exception as e:
                self._append_log("SYS", f"[ERROR] commandRequested.emit failed: {e}")
        else:
            self._append_log(
                "SYS",
                "[ERROR] OmronBridge hat weder send_command() noch signals.commandRequested",
            )

        # lokales Echo, falls raw_tx noch nicht kommt
        self._append_log("TX", cmd)
        self._cmd_edit.clear()

    @QtCore.pyqtSlot()
    def _on_clear_clicked(self) -> None:
        self._log_view.clear()
