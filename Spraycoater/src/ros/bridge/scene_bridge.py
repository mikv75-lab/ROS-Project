# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import List

import rclpy
from std_msgs.msg import String

from config.startup import AppContent
from .base_bridge import BaseBridge, sub_handler


class SceneBridge(BaseBridge):
    """
    UI-Bridge für 'scene':
      - Abonniert:  *_current, *_list   (vom ROS-Scene-Node publiziert)
      - Publiziert: *_set               (vom ROS-Scene-Node abonniert)

    Hält den UI-State (Listen + Currents) in Attributen:
      - self.cage_list / self.mount_list / self.substrate_list : List[str]
      - self.cage_current / self.mount_current / self.substrate_current : str

    Bietet einfache Setter-Methoden (publish auf *_set):
      - set_cage(name: str), set_mount(name: str), set_substrate(name: str)
    """
    GROUP = "scene"

    def __init__(self, content: AppContent):
        super().__init__("scene_bridge", content)

        # UI-State
        self.cage_list: List[str] = []
        self.mount_list: List[str] = []
        self.substrate_list: List[str] = []

        self.cage_current: str = ""
        self.mount_current: str = ""
        self.substrate_current: str = ""

        # Hinweis: Es gibt absichtlich KEINE Verzeichnis-/YAML-Scans hier.
        # Die Listen und Currents kommen vollständig über Topics aus dem ROS-Node.

    # -------- eingehende Nachrichten (vom ROS-Node 'publish') --------

    @sub_handler("scene", "cage_list")
    def _on_cage_list(self, msg: String):
        self.cage_list = self._parse_csv(getattr(msg, "data", ""))
        self.get_logger().info(f"[scene] cage_list: {len(self.cage_list)} items")

    @sub_handler("scene", "mount_list")
    def _on_mount_list(self, msg: String):
        self.mount_list = self._parse_csv(getattr(msg, "data", ""))
        self.get_logger().info(f"[scene] mount_list: {len(self.mount_list)} items")

    @sub_handler("scene", "substrate_list")
    def _on_substrate_list(self, msg: String):
        self.substrate_list = self._parse_csv(getattr(msg, "data", ""))
        self.get_logger().info(f"[scene] substrate_list: {len(self.substrate_list)} items")

    @sub_handler("scene", "cage_current")
    def _on_cage_current(self, msg: String):
        self.cage_current = (getattr(msg, "data", "") or "").strip()
        self.get_logger().info(f"[scene] cage_current: {self.cage_current or '-'}")

    @sub_handler("scene", "mount_current")
    def _on_mount_current(self, msg: String):
        self.mount_current = (getattr(msg, "data", "") or "").strip()
        self.get_logger().info(f"[scene] mount_current: {self.mount_current or '-'}")

    @sub_handler("scene", "substrate_current")
    def _on_substrate_current(self, msg: String):
        self.substrate_current = (getattr(msg, "data", "") or "").strip()
        self.get_logger().info(f"[scene] substrate_current: {self.substrate_current or '-'}")

    # -------- Publish-API (UI -> ROS-Node 'subscribe') --------

    def set_cage(self, name: str) -> None:
        self._publish_set("cage_set", name)

    def set_mount(self, name: str) -> None:
        self._publish_set("mount_set", name)

    def set_substrate(self, name: str) -> None:
        self._publish_set("substrate_set", name)

    # -------- intern --------

    def _publish_set(self, topic_id: str, value: str) -> None:
        try:
            Msg = self.spec("subscribe", topic_id).resolve_type()  # Node 'subscribe' -> UI publisht
            pub = self.pub(topic_id)
            msg = Msg()
            # Für std_msgs/msg/String: .data
            if hasattr(msg, "data"):
                msg.data = str(value)
            self.get_logger().info(f"[scene] -> {topic_id}: {value}")
            pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[scene] publish {topic_id} failed: {e}")

    @staticmethod
    def _parse_csv(s: str) -> List[str]:
        if not s:
            return []
        return [x.strip() for x in s.split(",") if x.strip()]
