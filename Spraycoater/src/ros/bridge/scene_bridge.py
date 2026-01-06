# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional, List, Any

from PyQt6 import QtCore
from std_msgs.msg import String as MsgString

from config.startup import AppContent
from .base_bridge import BaseBridge, sub_handler

try:
    from moveit_msgs.msg import CollisionObject
except Exception:  # pragma: no cover
    CollisionObject = Any  # type: ignore


class SceneSignals(QtCore.QObject):
    # ---------------- Node -> UI ----------------
    cageListChanged = QtCore.pyqtSignal(object)       # List[str]
    mountListChanged = QtCore.pyqtSignal(object)      # List[str]
    substrateListChanged = QtCore.pyqtSignal(object)  # List[str]

    cageCurrentChanged = QtCore.pyqtSignal(str)
    mountCurrentChanged = QtCore.pyqtSignal(str)
    substrateCurrentChanged = QtCore.pyqtSignal(str)

    # ---------------- UI -> Node (NEW contract used by SceneGroupBox) ----------------
    setCageRequested = QtCore.pyqtSignal(str)
    setMountRequested = QtCore.pyqtSignal(str)
    setSubstrateRequested = QtCore.pyqtSignal(str)

    # ---------------- UI -> Node (LEGACY aliases) ----------------
    cageSetRequested = QtCore.pyqtSignal(str)
    mountSetRequested = QtCore.pyqtSignal(str)
    substrateSetRequested = QtCore.pyqtSignal(str)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)

        # cached values (for RosBridge._reemit_cached())
        self.last_cage_list: List[str] = []
        self.last_mount_list: List[str] = []
        self.last_substrate_list: List[str] = []
        self.last_cage_current: str = ""
        self.last_mount_current: str = ""
        self.last_substrate_current: str = ""

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        self.cageListChanged.emit(list(self.last_cage_list))
        self.mountListChanged.emit(list(self.last_mount_list))
        self.substrateListChanged.emit(list(self.last_substrate_list))
        self.cageCurrentChanged.emit(self.last_cage_current or "")
        self.mountCurrentChanged.emit(self.last_mount_current or "")
        self.substrateCurrentChanged.emit(self.last_substrate_current or "")


class SceneBridge(BaseBridge):
    """
    UI-Bridge für 'scene' (SSoT topics/qos via AppContent).

    Contract (wie SceneGroupBox verdrahtet):
      - signals.setCageRequested(str)
      - signals.setMountRequested(str)
      - signals.setSubstrateRequested(str)

    Inbound (ROS -> UI), topics.yaml -> scene.publish:
      - cage_list, mount_list, substrate_list (std_msgs/String; CSV oder "[a,b]")
      - cage_current, mount_current, substrate_current (std_msgs/String)

    Outbound (UI -> ROS), topics.yaml -> scene.subscribe:
      - cage_set, mount_set, substrate_set (std_msgs/String)
      - Clear wird als "" publiziert (wichtig!)
    """

    GROUP = "scene"

    def __init__(self, content: AppContent, namespace: str = "") -> None:
        self.signals = SceneSignals()

        # state
        self.cage_list: List[str] = []
        self.mount_list: List[str] = []
        self.substrate_list: List[str] = []
        self.cage_current: str = ""
        self.mount_current: str = ""
        self.substrate_current: str = ""

        # Node inkl. Namespace anlegen
        super().__init__("scene_bridge", content, namespace=namespace)

        # --- publishers (UI -> ROS) aus subscribe-spec ableiten ---
        self._pub_cage_set = None
        self._pub_mount_set = None
        self._pub_substrate_set = None

        try:
            spec = self.spec("subscribe", "cage_set")
            Msg = spec.resolve_type()
            qos = getattr(spec, "qos_profile", None)
            self._pub_cage_set = self.create_publisher(Msg, spec.name, qos if qos is not None else 10)
        except Exception as e:
            self.get_logger().error(f"[scene] cage_set publisher init failed: {e}")

        try:
            spec = self.spec("subscribe", "mount_set")
            Msg = spec.resolve_type()
            qos = getattr(spec, "qos_profile", None)
            self._pub_mount_set = self.create_publisher(Msg, spec.name, qos if qos is not None else 10)
        except Exception as e:
            self.get_logger().error(f"[scene] mount_set publisher init failed: {e}")

        try:
            spec = self.spec("subscribe", "substrate_set")
            Msg = spec.resolve_type()
            qos = getattr(spec, "qos_profile", None)
            self._pub_substrate_set = self.create_publisher(Msg, spec.name, qos if qos is not None else 10)
        except Exception as e:
            self.get_logger().error(f"[scene] substrate_set publisher init failed: {e}")

        # --- legacy aliases -> new contract ---
        self.signals.cageSetRequested.connect(self.signals.setCageRequested.emit)
        self.signals.mountSetRequested.connect(self.signals.setMountRequested.emit)
        self.signals.substrateSetRequested.connect(self.signals.setSubstrateRequested.emit)

        # --- new contract -> handlers (NO @pyqtSlot on Node methods!) ---
        # Use lambda to avoid Qt metaobject issues with non-QObject receivers.
        self.signals.setCageRequested.connect(lambda s: self.set_cage(str(s)))
        self.signals.setMountRequested.connect(lambda s: self.set_mount(str(s)))
        self.signals.setSubstrateRequested.connect(lambda s: self.set_substrate(str(s)))

    # ---------------- inbound (ROS -> UI) ----------------

    @sub_handler("scene", "cage_list")
    def _on_cage_list(self, msg: MsgString) -> None:
        items = self._split_csv(msg)
        self.cage_list = items
        self.signals.last_cage_list = list(items)
        self.signals.cageListChanged.emit(list(items))

    @sub_handler("scene", "mount_list")
    def _on_mount_list(self, msg: MsgString) -> None:
        items = self._split_csv(msg)
        self.mount_list = items
        self.signals.last_mount_list = list(items)
        self.signals.mountListChanged.emit(list(items))

    @sub_handler("scene", "substrate_list")
    def _on_substrate_list(self, msg: MsgString) -> None:
        items = self._split_csv(msg)
        self.substrate_list = items
        self.signals.last_substrate_list = list(items)
        self.signals.substrateListChanged.emit(list(items))

    @sub_handler("scene", "cage_current")
    def _on_cage_current(self, msg: MsgString) -> None:
        v = (msg.data or "").strip()
        self.cage_current = v
        self.signals.last_cage_current = v
        self.signals.cageCurrentChanged.emit(v)

    @sub_handler("scene", "mount_current")
    def _on_mount_current(self, msg: MsgString) -> None:
        v = (msg.data or "").strip()
        self.mount_current = v
        self.signals.last_mount_current = v
        self.signals.mountCurrentChanged.emit(v)

    @sub_handler("scene", "substrate_current")
    def _on_substrate_current(self, msg: MsgString) -> None:
        v = (msg.data or "").strip()
        self.substrate_current = v
        self.signals.last_substrate_current = v
        self.signals.substrateCurrentChanged.emit(v)

    # Optional inbound
    @sub_handler("scene", "collision_object")
    def _on_collision_object(self, msg: Any) -> None:
        _ = msg

    # ---------------- outbound (UI -> ROS) ----------------

    def set_cage(self, name: str) -> None:
        # allow "" to clear
        self._publish_set(self._pub_cage_set, "cage_set", name)

    def set_mount(self, name: str) -> None:
        self._publish_set(self._pub_mount_set, "mount_set", name)

    def set_substrate(self, name: str) -> None:
        self._publish_set(self._pub_substrate_set, "substrate_set", name)

    def _publish_set(self, pub, label: str, value: str) -> None:
        v = (value or "").strip()  # may be "" (clear)
        if pub is None:
            self.get_logger().error(f"[scene] publish {label} failed: publisher not initialized")
            return
        try:
            pub.publish(MsgString(data=v))  # std_msgs/String
        except Exception as e:
            self.get_logger().error(f"[scene] publish {label} failed: {e}")

    # ---------------- helpers ----------------

    @staticmethod
    def _split_csv(msg: MsgString) -> List[str]:
        raw = (getattr(msg, "data", "") or "").strip()
        if not raw:
            return []

        if raw.startswith("[") and raw.endswith("]"):
            inner = raw[1:-1].strip()
            if not inner:
                return []
            parts = [p.strip().strip("'\"") for p in inner.split(",")]
            return [p for p in parts if p]

        parts = [p.strip() for p in raw.split(",")]
        return [p for p in parts if p]
