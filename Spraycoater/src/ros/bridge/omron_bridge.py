# src/ros/bridge/omron_bridge.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional

from PyQt6 import QtCore
from std_msgs.msg import String as MsgString, Bool as MsgBool

from config.startup import AppContent
from .base_bridge import BaseBridge, sub_handler


class OmronSignals(QtCore.QObject):
    """
    Qt-Signalträger für die Omron-Bridge.

    Inbound (ROS -> UI):
      - rawTxChanged(str)      : alles, was zum ACE-Server rausgeht
      - rawRxChanged(str)      : alles, was vom ACE-Server reinkommt
      - connectionChanged(bool): Verbindungstatus aus OmronTcpBridge

    Outbound (UI -> ROS):
      - commandRequested(str): eine Textzeile, die an den ACE-Server gesendet
                               werden soll (fallback, wenn send_command()
                               nicht direkt verwendet wird)
    """

    # Inbound
    rawTxChanged = QtCore.pyqtSignal(str)
    rawRxChanged = QtCore.pyqtSignal(str)
    connectionChanged = QtCore.pyqtSignal(bool)

    # Outbound
    commandRequested = QtCore.pyqtSignal(str)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)

        # Letzte Werte (optional für Initialisierung in Widgets)
        self.last_raw_tx: str = ""
        self.last_raw_rx: str = ""
        self.last_connection: bool = False

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        """Re-emittiert letzte Werte (wird von UIBridge._try_reemit_cached() genutzt)."""
        if self.last_raw_tx:
            self.rawTxChanged.emit(self.last_raw_tx)
        if self.last_raw_rx:
            self.rawRxChanged.emit(self.last_raw_rx)
        self.connectionChanged.emit(self.last_connection)


class OmronBridge(BaseBridge):
    """
    UI-Bridge für 'omron'.

    Erwartete topics.yaml-Struktur:

      omron:
        subscribe:
          - id: command
            name: /spraycoater/omron/command
            type: std_msgs/msg/String

        publish:
          - id: raw_tx
            name: /spraycoater/omron/raw_tx
            type: std_msgs/msg/String
          - id: raw_rx
            name: /spraycoater/omron/raw_rx
            type: std_msgs/msg/String
          - id: connectionStatus
            name: /spraycoater/omron/connection_status
            type: std_msgs/msg/Bool

    → passt zu OmronTcpBridge.
    """

    GROUP = "omron"

    def __init__(self, content: AppContent, namespace: str = ""):
        # Qt-Signale anlegen, bevor BaseBridge wiring macht
        self.signals = OmronSignals()

        # Cache für UI
        self.raw_tx: str = ""
        self.raw_rx: str = ""

        # Node ggf. im Namespace (z.B. /live) anlegen
        super().__init__("omron_bridge", content, namespace=namespace)

        # Outbound: UI → ROS
        self.signals.commandRequested.connect(self.send_command)

    # ---------------- eingehend (ROS -> UI) ----------------

    @sub_handler("omron", "raw_tx")
    def _on_raw_tx(self, msg: MsgString):
        text = (getattr(msg, "data", "") or "").rstrip("\n")
        self.raw_tx = text
        self.signals.last_raw_tx = text
        self.signals.rawTxChanged.emit(text)
        self.get_logger().debug(f"[omron] raw_tx: {text}")

    @sub_handler("omron", "raw_rx")
    def _on_raw_rx(self, msg: MsgString):
        text = (getattr(msg, "data", "") or "").rstrip("\n")
        self.raw_rx = text
        self.signals.last_raw_rx = text
        self.signals.rawRxChanged.emit(text)
        self.get_logger().debug(f"[omron] raw_rx: {text}")

    @sub_handler("omron", "connectionStatus")
    def _on_connection_status(self, msg: MsgBool):
        """
        Verbindungstatus vom OmronTcpBridge-Node:
        True  -> CONNECTED
        False -> DISCONNECTED
        """
        val = bool(getattr(msg, "data", False))
        self.signals.last_connection = val
        self.signals.connectionChanged.emit(val)
        self.get_logger().info(f"[omron] connectionStatus: {val}")

    # ---------------- ausgehend (UI -> ROS) ----------------

    def send_command(self, line: str) -> None:
        """
        Sendet eine Zeile an den OmronTcpBridge-Node,
        der sie dann über TCP an ACE weiterreicht.
        """
        text = (line or "").rstrip("\n")
        if not text:
            self.get_logger().warning("[omron] send_command: Leerstring ignoriert")
            return

        try:
            # Spezifikation aus topics.yaml ziehen, damit Name + Typ
            spec = self.spec("subscribe", "command")
            Msg = spec.resolve_type()
            pub = self.pub("command")

            msg = Msg()
            msg.data = text

            self.get_logger().info(f"[omron] -> command: {text}")
            pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[omron] publish command failed: {e}")

    # ---------------- Hilfs-API für Widgets ----------------

    def get_last_raw_tx(self) -> str:
        return self.raw_tx

    def get_last_raw_rx(self) -> str:
        return self.raw_rx
