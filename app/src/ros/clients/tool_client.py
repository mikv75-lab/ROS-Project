# ros/clients/tool_client.py
from __future__ import annotations

import time
from typing import Callable, Optional, List

from rclpy.node import Node
from std_msgs.msg import String, Empty

from ..common.topics import Topics
from ..common.qos import qos_default, qos_latched


class ToolClient:
    """
    Leichter Client zum Setzen/Lesen des aktiven Tools über Topics (ohne Services).

    Topics (relativ, Remaps/Namespaces via Node/Launch):
      - publish:   Topics.tool_set        (std_msgs/String)      -> Tool setzen
      - publish:   Topics.tool_republish  (std_msgs/Empty)       -> Status erneut senden
      - subscribe: Topics.tool_current    (std_msgs/String, latched) -> Aktueller Status

    Beispiel:
        topics = Topics(controller="meca_arm_group_controller")
        tool = ToolClient(self, topics)
        tool.wait_for_state(timeout=2.0)  # optional
        tool.set_tool("spray_nozzle_01", wait_for_confirm=True)
        current = tool.get_current()
    """

    def __init__(self, node: Node, topics: Optional[Topics] = None) -> None:
        self._node = node
        self._topics = topics or Topics()
        self._logger = node.get_logger()

        self._current_tool: Optional[str] = None
        self._callbacks: List[Callable[[str], None]] = []

        # Publisher
        self._pub_set = node.create_publisher(String, self._topics.tool_set, qos_default())
        self._pub_repub = node.create_publisher(Empty, self._topics.tool_republish, qos_default())

        # Subscriber (latched QoS, damit sofort ein Wert ankommt)
        self._sub_current = node.create_subscription(
            String, self._topics.tool_current, self._on_current, qos_latched()
        )

    # ------------------------- Public API -------------------------

    def set_tool(self, name: str, *, wait_for_confirm: bool = False, timeout: float = 2.0) -> bool:
        """
        Setzt das aktive Tool. Optional auf Bestätigung warten
        (bis /meca/tool/current == name empfangen wurde).
        """
        name = (name or "").strip()
        if not name:
            self._logger.error("ToolClient: Leerer Tool-Name – Abbruch.")
            return False

        self._pub_set.publish(String(data=name))
        self._logger.info(f"🔧 Tool-Set gesendet: {name}")

        if not wait_for_confirm:
            return True

        ok = self._wait_until(lambda: self._current_tool == name, timeout=timeout)
        if not ok:
            self._logger.warning(
                f"ToolClient: Keine Bestätigung für Tool '{name}' innerhalb {timeout:.1f}s."
            )
        return ok

    def ensure_tool(self, name: str, *, wait_for_confirm: bool = True, timeout: float = 2.0) -> bool:
        """Setzt das Tool nur, wenn der aktuelle Status davon abweicht (oder unbekannt)."""
        if (name or "").strip() == (self._current_tool or ""):
            self._logger.debug(f"ToolClient: Tool bereits aktiv: {name}")
            return True
        return self.set_tool(name, wait_for_confirm=wait_for_confirm, timeout=timeout)

    def get_current(self) -> Optional[str]:
        """Gibt das zuletzt empfangene aktive Tool zurück (oder None, falls noch unbekannt)."""
        return self._current_tool

    def wait_for_state(self, timeout: float = 2.0) -> bool:
        """
        Wartet, bis überhaupt ein /meca/tool/current empfangen wurde.
        Triggert vorher ein Republish beim tool_manager.
        """
        try:
            self._pub_repub.publish(Empty())
        except Exception:
            # Falls Publisher noch nicht bereit sein sollte, einfach weiter warten.
            pass
        ok = self._wait_until(lambda: self._current_tool is not None, timeout=timeout)
        if not ok:
            self._logger.warning("ToolClient: Kein initialer Tool-Status empfangen.")
        return ok

    def on_change(self, cb: Callable[[str], None]) -> None:
        """Registriert einen Callback, der bei Tool-Änderung aufgerufen wird."""
        if callable(cb):
            self._callbacks.append(cb)

    def destroy(self) -> None:
        """Publisher/Subscriber sauber freigeben (optional)."""
        try:
            self._node.destroy_publisher(self._pub_set)
            self._node.destroy_publisher(self._pub_repub)
            self._node.destroy_subscription(self._sub_current)
        except Exception:
            pass

    # ------------------------- Internals -------------------------

    def _on_current(self, msg: String) -> None:
        name = (msg.data or "").strip()
        prev = self._current_tool
        self._current_tool = name
        if name != prev:
            self._logger.info(f"🔎 Aktuelles Tool: {name or '—'}")
            for cb in list(self._callbacks):
                try:
                    cb(name)
                except Exception as e:
                    self._logger.warning(f"ToolClient: Callback-Fehler: {e}")

    def _wait_until(self, predicate, *, timeout: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout)
        # kleine Spin-Schleife: nutzt rclpy.spin_once über den Node
        while time.monotonic() < deadline:
            try:
                import rclpy
                rclpy.spin_once(self._node, timeout_sec=0.05)
            except Exception:
                time.sleep(0.05)
            if predicate():
                return True
        return predicate()
