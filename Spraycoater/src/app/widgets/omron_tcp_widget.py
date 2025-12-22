# src/app/widgets/omron_tcp_widget.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional
import html

from PyQt6 import QtCore, QtWidgets

from ros.bridge.ros_bridge import RosBridge


class OmronTcpWidget(QtWidgets.QWidget):
    """
    Einfaches TCP-Client-Widget für den Omron ACE-Server.

    Erwartet:
        ros.omron -> OmronBridge mit:
            - signals.rawRxChanged(str)
            - signals.connectionChanged(bool)
            - Methode send_command(str)
    """

    def __init__(self, ros: Optional[RosBridge], parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self._ros = ros
        self._omron = getattr(ros, "omron", None) if ros is not None else None

        # letzter bekannter Connection-Status (für Change-Detection)
        self._last_connected: Optional[bool] = None

        self._build_ui()
        self._wire_signals()

        # Initialer Status: NUR UI setzen, KEIN Log-Spam
        self._set_connection_indicator_silent(False)

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

        # --- Log-Fenster (Konsole) ---
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
            self._append_sys("[WARN] OmronBridge nicht verfügbar.")
            return

        sig = self._omron.signals

        # raw_rx -> Konsole (als RX)
        if hasattr(sig, "rawRxChanged"):
            sig.rawRxChanged.connect(self._on_raw_rx)

        # connectionChanged -> Status
        if hasattr(sig, "connectionChanged"):
            sig.connectionChanged.connect(self._update_connection_indicator)

    # ------------------------------------------------------------------
    # Helper: Zeit + Konsolen-Ausgabe
    # ------------------------------------------------------------------
    def _current_time_str(self) -> str:
        return QtCore.QTime.currentTime().toString("HH:mm:ss")

    def _append_console_line(self, text: str) -> None:
        """
        Reine Konsolenzeile (für User-Eingaben):
        [HH:MM:SS] <text>
        """
        text = (text or "").rstrip()
        if not text:
            return

        ts = self._current_time_str()
        escaped = html.escape(text)
        line = f'<span style="color:#666666;">[{ts}]</span> {escaped}'
        self._log_view.append(line)

    def _append_rx(self, text: str) -> None:
        """
        RX-Zeilen vom Omron-Server:
        [HH:MM:SS] [RX] <text>
        """
        text = (text or "").rstrip()
        if not text:
            return

        ts = self._current_time_str()
        escaped = html.escape(text)
        line = (
            f'<span style="color:#666666;">[{ts}]</span> '
            f'<span style="color:#00aa00;">[RX] {escaped}</span>'
        )
        self._log_view.append(line)

    def _append_sys(self, text: str) -> None:
        """
        System-Meldungen:
        [HH:MM:SS] [SYS] <text>
        """
        text = (text or "").rstrip()
        if not text:
            return

        ts = self._current_time_str()
        escaped = html.escape(text)
        line = (
            f'<span style="color:#666666;">[{ts}]</span> '
            f'<span style="color:#888888;">[SYS] {escaped}</span>'
        )
        self._log_view.append(line)

    # ------------------------------------------------------------------
    # Slots für eingehende ROS-Signale
    # ------------------------------------------------------------------
    @QtCore.pyqtSlot(str)
    def _on_raw_rx(self, txt: str) -> None:
        self._append_rx(txt)

    def _set_connection_indicator_silent(self, connected: bool) -> None:
        """
        Setzt nur Label+Style und aktualisiert _last_connected,
        ohne _append_sys() zu triggern (Startup/Restore).
        """
        self._last_connected = connected

        if connected:
            self._status_label.setText("CONNECTED")
            self._status_label.setStyleSheet("color: #00aa00; font-weight: bold;")
        else:
            self._status_label.setText("DISCONNECTED")
            self._status_label.setStyleSheet("color: #aa0000; font-weight: bold;")

    @QtCore.pyqtSlot(bool)
    def _update_connection_indicator(self, connected: bool) -> None:
        # Nur reagieren, wenn sich der Status geändert hat
        if self._last_connected is not None and connected == self._last_connected:
            return

        self._last_connected = connected

        if connected:
            self._status_label.setText("CONNECTED")
            self._status_label.setStyleSheet("color: #00aa00; font-weight: bold;")
            self._append_sys("Verbunden mit ACE-Server.")
        else:
            self._status_label.setText("DISCONNECTED")
            self._status_label.setStyleSheet("color: #aa0000; font-weight: bold;")
            self._append_sys("Verbindung getrennt.")

    # ------------------------------------------------------------------
    # Slots: Buttons
    # ------------------------------------------------------------------
    @QtCore.pyqtSlot()
    def _on_send_clicked(self) -> None:
        cmd = self._cmd_edit.text().strip()
        if not cmd:
            return

        # Direkt in die Konsole schreiben (ohne [TX], einfach der Text)
        self._append_console_line(cmd)

        if self._omron is None:
            self._append_sys("(kein OmronBridge – Command nicht gesendet)")
            self._cmd_edit.clear()
            return

        # bevorzugt: Methode send_command()
        if hasattr(self._omron, "send_command"):
            try:
                self._omron.send_command(cmd)
            except Exception as e:
                self._append_sys(f"[ERROR] send_command failed: {e}")
        # Fallback: Signal commandRequested, falls vorhanden
        elif hasattr(self._omron, "signals") and hasattr(self._omron.signals, "commandRequested"):
            try:
                self._omron.signals.commandRequested.emit(cmd)
            except Exception as e:
                self._append_sys(f"[ERROR] commandRequested.emit failed: {e}")
        else:
            self._append_sys(
                "[ERROR] OmronBridge hat weder send_command() noch signals.commandRequested"
            )

        self._cmd_edit.clear()

    @QtCore.pyqtSlot()
    def _on_clear_clicked(self) -> None:
        self._log_view.clear()
