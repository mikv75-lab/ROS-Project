# -*- coding: utf-8 -*-
# spraycoater_nodes_py/robot_bridge.py
from __future__ import annotations
from typing import Optional

from PyQt6 import QtCore
from std_msgs.msg import Bool, String, Empty
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from config.startup import AppContent
from .base_bridge import BaseBridge, sub_handler


class RobotSignals(QtCore.QObject):
    """
    Qt-Signalträger für die Robot-Bridge.

    Inbound (ROS -> UI):
      - connectionChanged(bool)
      - modeChanged(str)
      - initializedChanged(bool)
      - movingChanged(bool)
      - servoEnabledChanged(bool)
      - powerChanged(bool)
      - estopChanged(bool)
      - errorsChanged(str)
      - tcpPoseChanged(PoseStamped|None)
      - jointsChanged(JointState|None)

    Outbound (UI -> Bridge):
      - initRequested()
      - stopRequested()
      - clearErrorRequested()
      - powerOnRequested()
      - powerOffRequested()
      - servoEnableRequested()
      - servoDisableRequested()

    Zusätzlich werden die aktuellen Werte als Attribute auf diesem QObject
    gespiegelt (connection, mode, initialized, moving, servo_enabled, power,
    estop, errors, tcp_pose, joints), damit eine UI initial synchronisieren
    kann, ohne auf neue ROS-Nachrichten zu warten.
    """

    # Inbound (ROS -> UI)
    connectionChanged = QtCore.pyqtSignal(bool)
    modeChanged = QtCore.pyqtSignal(str)
    initializedChanged = QtCore.pyqtSignal(bool)
    movingChanged = QtCore.pyqtSignal(bool)
    servoEnabledChanged = QtCore.pyqtSignal(bool)
    powerChanged = QtCore.pyqtSignal(bool)
    estopChanged = QtCore.pyqtSignal(bool)
    errorsChanged = QtCore.pyqtSignal(str)
    tcpPoseChanged = QtCore.pyqtSignal(object)   # PoseStamped | None
    jointsChanged = QtCore.pyqtSignal(object)    # JointState | None

    # Outbound (UI -> Bridge)
    initRequested         = QtCore.pyqtSignal()
    stopRequested         = QtCore.pyqtSignal()
    clearErrorRequested   = QtCore.pyqtSignal()
    powerOnRequested      = QtCore.pyqtSignal()
    powerOffRequested     = QtCore.pyqtSignal()
    servoEnableRequested  = QtCore.pyqtSignal()
    servoDisableRequested = QtCore.pyqtSignal()

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)

        # Spiegel-Attribute (werden von RobotBridge gepflegt)
        self.connection: bool = False
        self.mode: str = "DISCONNECTED"
        self.initialized: bool = False
        self.moving: bool = False
        self.servo_enabled: bool = False
        self.power: bool = False
        self.estop: bool = False
        self.errors: str = ""
        self.tcp_pose: Optional[PoseStamped] = None
        self.joints: Optional[JointState] = None

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        """
        Emittiert alle zuletzt bekannten Werte erneut.
        Wird von der UIBridge nach dem Connect verwendet, falls vorhanden.
        """
        self.connectionChanged.emit(self.connection)
        self.modeChanged.emit(self.mode)
        self.initializedChanged.emit(self.initialized)
        self.movingChanged.emit(self.moving)
        self.servoEnabledChanged.emit(self.servo_enabled)
        self.powerChanged.emit(self.power)
        self.estopChanged.emit(self.estop)
        self.errorsChanged.emit(self.errors)
        self.tcpPoseChanged.emit(self.tcp_pose)
        self.jointsChanged.emit(self.joints)


class RobotBridge(BaseBridge):
    """
    UI-Bridge für 'robot' mit eingebauten Qt-Signalen.

    - Abonniert (vom ROS-Robot-Node publiziert):
        connection, mode, initialized, moving, servo_enabled, power, estop,
        errors, tcp_pose, joints

      -> aktualisiert internen State + emittiert Qt-Signale

    - Publiziert (vom ROS-Robot-Node abonniert):
        init, stop, clear_error, power_on, power_off, servo_on, servo_off

      -> via Outbound-Signale von RobotSignals:
         initRequested/stopRequested/... -> do_init/do_stop/...
    """

    GROUP = "robot"

    def __init__(self, content: AppContent):
        # Signale VOR super().__init__ anlegen (wie bei SceneBridge/PosesBridge)
        self.signals = RobotSignals()

        # Interner Cache (spiegeln 1:1 die Signals-Attribute)
        self.connection: bool = False
        self.mode: str = "DISCONNECTED"
        self.initialized: bool = False
        self.moving: bool = False
        self.servo_enabled: bool = False
        self.power: bool = False
        self.estop: bool = False
        self.errors: str = ""
        self.tcp_pose: Optional[PoseStamped] = None
        self.joints: Optional[JointState] = None

        super().__init__("robot_bridge", content)

        # Outbound: Widget-Signale -> Bridge-Methoden
        self.signals.initRequested.connect(self.do_init)
        self.signals.stopRequested.connect(self.do_stop)
        self.signals.clearErrorRequested.connect(self.do_clear_error)
        self.signals.powerOnRequested.connect(self.do_power_on)
        self.signals.powerOffRequested.connect(self.do_power_off)
        self.signals.servoEnableRequested.connect(self.do_servo_on)
        self.signals.servoDisableRequested.connect(self.do_servo_off)

    # -------- eingehende Nachrichten (vom ROS-Node 'publish') --------

    @sub_handler("robot", "connection")
    def _on_connection(self, msg: Bool):
        val = bool(getattr(msg, "data", False))
        self.connection = val
        self.signals.connection = val
        self.signals.connectionChanged.emit(val)
        self.get_logger().debug(f"[robot] connection: {val}")

    @sub_handler("robot", "mode")
    def _on_mode(self, msg: String):
        val = (getattr(msg, "data", "") or "").strip()
        self.mode = val
        self.signals.mode = val
        self.signals.modeChanged.emit(val)
        self.get_logger().debug(f"[robot] mode: {val or '-'}")

    @sub_handler("robot", "initialized")
    def _on_initialized(self, msg: Bool):
        val = bool(getattr(msg, "data", False))
        self.initialized = val
        self.signals.initialized = val
        self.signals.initializedChanged.emit(val)
        self.get_logger().debug(f"[robot] initialized: {val}")

    @sub_handler("robot", "moving")
    def _on_moving(self, msg: Bool):
        val = bool(getattr(msg, "data", False))
        self.moving = val
        self.signals.moving = val
        self.signals.movingChanged.emit(val)
        self.get_logger().debug(f"[robot] moving: {val}")

    @sub_handler("robot", "servo_enabled")
    def _on_servo_enabled(self, msg: Bool):
        val = bool(getattr(msg, "data", False))
        self.servo_enabled = val
        self.signals.servo_enabled = val
        self.signals.servoEnabledChanged.emit(val)
        self.get_logger().debug(f"[robot] servo_enabled: {val}")

    @sub_handler("robot", "power")
    def _on_power(self, msg: Bool):
        val = bool(getattr(msg, "data", False))
        self.power = val
        self.signals.power = val
        self.signals.powerChanged.emit(val)
        self.get_logger().debug(f"[robot] power: {val}")

    @sub_handler("robot", "estop")
    def _on_estop(self, msg: Bool):
        val = bool(getattr(msg, "data", False))
        self.estop = val
        self.signals.estop = val
        self.signals.estopChanged.emit(val)
        self.get_logger().debug(f"[robot] estop: {val}")

    @sub_handler("robot", "errors")
    def _on_errors(self, msg: String):
        val = (getattr(msg, "data", "") or "").strip()
        self.errors = val
        self.signals.errors = val
        self.signals.errorsChanged.emit(val)
        if val:
            self.get_logger().error(f"[robot] errors: {val}")
        else:
            self.get_logger().info("[robot] errors cleared")

    @sub_handler("robot", "tcp_pose")
    def _on_tcp_pose(self, msg: PoseStamped):
        self.tcp_pose = msg
        self.signals.tcp_pose = msg
        self.signals.tcpPoseChanged.emit(msg)
        try:
            self.get_logger().debug(
                f"[robot] tcp_pose: frame={msg.header.frame_id}, "
                f"pos=({msg.pose.position.x:.3f}, "
                f"{msg.pose.position.y:.3f}, "
                f"{msg.pose.position.z:.3f})"
            )
        except Exception:
            pass

    @sub_handler("robot", "joints")
    def _on_joints(self, msg: JointState):
        self.joints = msg
        self.signals.joints = msg
        self.signals.jointsChanged.emit(msg)
        self.get_logger().debug(f"[robot] joints: {len(msg.name)} joints")

    # -------- Publish-API (Bridge -> ROS-Node 'subscribe') --------

    def _publish_empty(self, topic_id: str, log_label: str) -> None:
        """
        Hilfsfunktion: publisht eine std_msgs/Empty Nachricht auf das Topic,
        das im Node (Robot) als 'subscribe' definiert ist (init/stop/…).
        """
        try:
            Msg = self.spec("subscribe", topic_id).resolve_type()
            pub = self.pub(topic_id)
            msg = Msg()
            self.get_logger().info(f"[robot] -> {topic_id} ({log_label})")
            pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[robot] publish {topic_id} failed: {e}")

    def do_init(self) -> None:
        self._publish_empty("init", "INIT")

    def do_stop(self) -> None:
        self._publish_empty("stop", "STOP")

    def do_clear_error(self) -> None:
        self._publish_empty("clear_error", "CLEAR_ERROR")

    def do_power_on(self) -> None:
        self._publish_empty("power_on", "POWER_ON")

    def do_power_off(self) -> None:
        self._publish_empty("power_off", "POWER_OFF")

    def do_servo_on(self) -> None:
        self._publish_empty("servo_on", "SERVO_ON")

    def do_servo_off(self) -> None:
        self._publish_empty("servo_off", "SERVO_OFF")
