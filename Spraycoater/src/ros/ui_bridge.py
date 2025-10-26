# Spraycoater/src/ros/ui_bridge.py
# -*- coding: utf-8 -*-
"""
UIBridge: dünne Brücke zwischen Qt-UI und ROS2.

- Kein stiller Fallback: Wenn ROS-Funktionen genutzt werden und keine Verbindung
  existiert, wird ein klarer Fehler geworfen (RuntimeError).
- rclpy wird lazy importiert; die UI kann das Modul laden, auch wenn ROS nicht
  installiert ist – ROS-Funktionen funktionieren aber erst nach `connect()`.

Aktuell implementierte Features:
- connect()/shutdown() -> ROS2 Node + Executor-Thread
- publish_ui_cmd() -> JSON-Kommandos an /spray/ui_cmd
- optionales "Request/Response" via /spray/ui_events (String JSON)
- validate()/optimize() -> nur ROS-Command (UI führt syntaktische Checks selbst aus)
- save_recipe() -> Datei speichern

Service-/Topic-Namen an Ihr System anpassen.
"""

from __future__ import annotations
import json
import logging
import os
import threading
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

_LOG = logging.getLogger("ros.ui_bridge")


@dataclass
class BridgeResult:
    ok: bool
    message: str = ""
    data: Any = None


class UIBridge:
    """
    Minimaler ROS2-Bridge-Layer für die UI.
    """
    # Default-Topics (bei Bedarf anpassen)
    TOPIC_UI_CMD: str = "/spray/ui_cmd"       # std_msgs/String (JSON: {"cmd": "...", "payload": {...}, "corr_id": "..."} )
    TOPIC_UI_EVENTS: str = "/spray/ui_events" # std_msgs/String (JSON: {"event": "...", "payload": {...}, "corr_id": "..."} )

    def __init__(self, ctx, node_name: str = "spray_ui", namespace: Optional[str] = None):
        self.ctx = ctx
        self.node_name = node_name
        self.namespace = namespace or ""
        self._rclpy = None
        self._node = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._pub_cmd = None
        self._sub_events = None

        # Simple in-memory "response wait" map
        self._wait_map: Dict[str, Tuple[threading.Event, Optional[Dict[str, Any]]]] = {}

    # ------------- Lifecycle -------------

    @property
    def is_connected(self) -> bool:
        return self._node is not None

    def connect(self) -> None:
        """
        Initialisiert rclpy, erstellt Node, Publisher/Subscriber und startet Executor-Thread.
        """
        if self.is_connected:
            _LOG.info("UIBridge already connected.")
            return

        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.executors import MultiThreadedExecutor
            from std_msgs.msg import String
        except Exception as e:
            _LOG.exception("rclpy/std_msgs nicht verfügbar.")
            raise RuntimeError("ROS2 (rclpy) ist nicht installiert/verfügbar.") from e

        self._rclpy = rclpy

        if not rclpy.ok():
            rclpy.init(args=None)

        self._node = Node(self.node_name, namespace=self.namespace or "")

        # Publisher/Subscriber
        self._pub_cmd = self._node.create_publisher(String, self.TOPIC_UI_CMD, 10)
        self._sub_events = self._node.create_subscription(String, self.TOPIC_UI_EVENTS, self._on_event, 10)

        # Executor + Spin-Thread
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)

        def _spin():
            try:
                _LOG.info("UIBridge spin thread started.")
                self._executor.spin()
            except Exception:
                _LOG.exception("Executor spin crashed.")
            finally:
                _LOG.info("UIBridge spin thread stopped.")

        self._spin_thread = threading.Thread(target=_spin, name="ui_bridge_spin", daemon=True)
        self._spin_thread.start()

        _LOG.info("UIBridge connected (node=%s, ns=%s).", self.node_name, self.namespace)

    def shutdown(self) -> None:
        """
        Stoppt Executor, zerstört Node, ruft rclpy.shutdown() (falls nötig).
        """
        try:
            if self._executor:
                try:
                    self._executor.shutdown(timeout_sec=0.5)
                except Exception:
                    pass
        finally:
            self._executor = None

        try:
            if self._node:
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
        finally:
            self._node = None
            self._pub_cmd = None
            self._sub_events = None

        try:
            if self._rclpy and self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass
        finally:
            self._rclpy = None

        try:
            if self._spin_thread and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=0.5)
        except Exception:
            pass
        finally:
            self._spin_thread = None

        _LOG.info("UIBridge shutdown complete.")

    # ------------- Pub/Sub JSON Helpers -------------

    def _on_event(self, msg) -> None:
        """
        Callback für /spray/ui_events (std_msgs/String).
        Erwartetes JSON: {"event": "...", "payload": {...}, "corr_id": "..."}
        """
        try:
            data = json.loads(msg.data)
        except Exception:
            _LOG.warning("Invalid JSON on %s: %r", self.TOPIC_UI_EVENTS, msg.data[:200])
            return

        corr_id = data.get("corr_id")
        if corr_id and corr_id in self._wait_map:
            ev, _ = self._wait_map[corr_id]
            self._wait_map[corr_id] = (ev, data)
            ev.set()

        # Zusätzlich ins Log spiegeln
        _LOG.debug("ui_event: %s", data)

    def publish_ui_cmd(self, cmd: str, payload: Optional[Dict[str, Any]] = None, corr_id: Optional[str] = None) -> None:
        """
        Fire-and-forget UI-Command als JSON an /spray/ui_cmd.
        """
        if not self.is_connected:
            raise RuntimeError("UIBridge nicht verbunden (connect() aufrufen).")

        from std_msgs.msg import String
        data = {"cmd": cmd, "payload": payload or {}}
        if corr_id:
            data["corr_id"] = corr_id
        try:
            msg = String()
            msg.data = json.dumps(data, ensure_ascii=False)
            self._pub_cmd.publish(msg)
            _LOG.info("ui_cmd -> %s: %s", self.TOPIC_UI_CMD, data)
        except Exception as e:
            _LOG.exception("publish_ui_cmd failed: %s", e)
            raise

    def request_response(self, cmd: str, payload: Optional[Dict[str, Any]] = None, timeout: float = 5.0) -> BridgeResult:
        """
        Einfaches synchrones Request/Response über die Event-Subscription.
        Erwartet eine Antwort mit passender corr_id auf /spray/ui_events.
        """
        import uuid
        corr_id = str(uuid.uuid4())
        ev = threading.Event()
        self._wait_map[corr_id] = (ev, None)
        try:
            self.publish_ui_cmd(cmd, payload=payload, corr_id=corr_id)
            ok = ev.wait(timeout=timeout)
            if not ok:
                return BridgeResult(ok=False, message=f"Timeout bei Antwort auf '{cmd}'")
            _, data = self._wait_map.get(corr_id, (None, None))
            return BridgeResult(ok=True, message="ok", data=data)
        finally:
            self._wait_map.pop(corr_id, None)

    # ------------- High-Level API -------------

    def validate(self, recipe: Dict[str, Any], *, syntactic_only: bool = False, timeout: float = 0.0) -> BridgeResult:
        """
        Validierung an ROS weiterreichen.
        Die syntaktische Validierung passiert vorab in der UI (RecipeTab).
        """
        if syntactic_only:
            # Nur Status zurückgeben: syntaktische Prüfung wurde upstream erledigt.
            return BridgeResult(ok=True, message="Syntaktische Prüfung (UI) OK.")

        if not self.is_connected:
            return BridgeResult(ok=False, message="Keine ROS-Verbindung für validate().")

        if timeout and timeout > 0:
            # Request/Response erwartet eine Antwort (Adapter in Ihren Nodes nötig)
            return self.request_response("validate", {"recipe": recipe}, timeout=timeout)

        # Fire-and-forget (Nodes loggen/prüfen asynchron)
        self.publish_ui_cmd("validate", {"recipe": recipe})
        return BridgeResult(ok=True, message="Validate an ROS gesendet (async).")

    def optimize(self, recipe: Dict[str, Any], *, timeout: float = 0.0) -> BridgeResult:
        """
        Rezeptoptimierung auf ROS-Seite anstoßen (z. B. Pfad glätten, Umsortierung).
        Ohne ROS-Verbindung gibt es KEINE „Fake“-Optimierung.
        """
        if not self.is_connected:
            return BridgeResult(ok=False, message="Keine ROS-Verbindung für optimize().")

        if timeout and timeout > 0:
            return self.request_response("optimize", {"recipe": recipe}, timeout=timeout)

        self.publish_ui_cmd("optimize", {"recipe": recipe})
        return BridgeResult(ok=True, message="Optimize an ROS gesendet (async).")

    def set_tool(self, tool_id: str) -> BridgeResult:
        if not self.is_connected:
            return BridgeResult(ok=False, message="Keine ROS-Verbindung für set_tool().")
        self.publish_ui_cmd("set_tool", {"tool": tool_id})
        return BridgeResult(ok=True, message=f"Tool '{tool_id}' gesetzt (Command gesendet).")

    def set_substrate(self, substrate_id: str) -> BridgeResult:
        if not self.is_connected:
            return BridgeResult(ok=False, message="Keine ROS-Verbindung für set_substrate().")
        self.publish_ui_cmd("set_substrate", {"substrate": substrate_id})
        return BridgeResult(ok=True, message=f"Substrate '{substrate_id}' gesetzt (Command gesendet).")

    def set_mount(self, mount_id: str) -> BridgeResult:
        if not self.is_connected:
            return BridgeResult(ok=False, message="Keine ROS-Verbindung für set_mount().")
        self.publish_ui_cmd("set_mount", {"mount": mount_id})
        return BridgeResult(ok=True, message=f"Mount '{mount_id}' gesetzt (Command gesendet).")

    def preview_plan(self, points_xyz: list) -> BridgeResult:
        """
        Punkte für eine UI-Vorschau an ROS senden (z. B. Marker/Path).
        """
        if not self.is_connected:
            return BridgeResult(ok=False, message="Keine ROS-Verbindung für preview_plan().")
        self.publish_ui_cmd("preview_plan", {"points": points_xyz})
        return BridgeResult(ok=True, message=f"{len(points_xyz)} Punkte an Preview gesendet.")

    # ------------- Datei-Helper -------------

    def save_recipe(self, recipe: Dict[str, Any], *, filename: Optional[str] = None) -> BridgeResult:
        """
        Lokales Speichern eines einzelnen Rezepts (YAML-Teil).
        Die UI validiert vor dem Speichern.
        """
        fname = filename or f"{recipe.get('id','recipe')}.yaml"
        target = os.path.join(self.ctx.paths.recipe_dir, fname)
        try:
            import yaml
            with open(target, "w", encoding="utf-8") as f:
                yaml.safe_dump(recipe, f, allow_unicode=True, sort_keys=False)
            return BridgeResult(ok=True, message=f"Gespeichert: {target}", data={"path": target})
        except Exception as e:
            _LOG.exception("save_recipe failed: %s", e)
            return BridgeResult(ok=False, message=str(e))


# -------- Singleton-Helfer --------

_SINGLETON: Optional[UIBridge] = None

def get_ui_bridge(ctx) -> UIBridge:
    """
    Globalen Bridge-Singleton holen/erzeugen.
    """
    global _SINGLETON
    if _SINGLETON is None:
        _SINGLETON = UIBridge(ctx)
    return _SINGLETON
