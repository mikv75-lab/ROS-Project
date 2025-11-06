# Spraycoater/src/ros/bridge/runner.py
from __future__ import annotations
import threading
from typing import Optional, List, Any, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor

from config.startup import AppContent
from .scene_bridge import SceneBridge


class RosBridge:
    """
    Betreibt Bridge-Nodes (aktuell: nur SceneBridge) in einem eigenen Executor-Thread.
    - AppContent wird intern genutzt (keine Exposition nach außen).
    - Thread-sicheres Start/Stop.
    - Nodes können in Zukunft erweitert werden (add_node).
    """

    def __init__(self, startup_yaml_path: str):
        self._startup_yaml_path = startup_yaml_path
        self._content = AppContent(startup_yaml_path)  # intern genutzt

        self._exec = SingleThreadedExecutor()
        self._nodes: List[Any] = []
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.RLock()

    # --- öffentliche API (ohne AppContent-Leak) ---

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._running

    @property
    def primary_node(self):
        """Erster Bridge-Node (z. B. für generisches Publish in UIBridge)."""
        with self._lock:
            return self._nodes[0] if self._nodes else None

    @property
    def nodes(self) -> Tuple[Any, ...]:
        with self._lock:
            return tuple(self._nodes)

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            if not rclpy.ok():
                rclpy.init(args=None)

            # Nodes anlegen (nur Scene – weitere später ergänzen)
            scene = SceneBridge(self._content)
            self._nodes.append(scene)
            for n in self._nodes:
                self._exec.add_node(n)

            self._running = True
            self._thread = threading.Thread(target=self._spin, name="ros-bridge", daemon=True)
            self._thread.start()

    def _spin(self) -> None:
        try:
            self._exec.spin()
        except Exception:
            # Executor-Fehler nicht hart crashen lassen; Aufräumen folgt in stop()
            pass
        finally:
            with self._lock:
                self._running = False

    def stop(self) -> None:
        with self._lock:
            if not self._running:
                # Falls Thread noch hängt, trotzdem versuchen aufzuräumen
                if self._thread and self._thread.is_alive():
                    try:
                        self._exec.shutdown()
                    except Exception:
                        pass
                    self._thread.join(timeout=1.0)
                    self._thread = None
                return

            # Executor sauber stoppen
            try:
                self._exec.shutdown()
            except Exception:
                pass

            # Nodes aus Executor entfernen und zerstören
            for n in list(self._nodes):
                try:
                    self._exec.remove_node(n)
                except Exception:
                    pass
                try:
                    n.destroy_node()
                except Exception:
                    pass
            self._nodes.clear()

            # rclpy herunterfahren
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

            self._running = False

            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.0)
            self._thread = None

    # --- (optional) spätere Erweiterung: weitere Nodes anhängen ---
    def add_node(self, node: Any) -> None:
        """
        Fügt einen bereits instanziierten rclpy-Node hinzu.
        - Vor start(): der Node wird beim start() dem Executor hinzugefügt.
        - Nach start(): der Node wird sofort dem Executor hinzugefügt.
        """
        with self._lock:
            self._nodes.append(node)
            if self._running:
                try:
                    self._exec.add_node(node)
                except Exception:
                    # Fail fast: wieder entfernen, wenn add fehlschlägt
                    try:
                        self._nodes.remove(node)
                    except ValueError:
                        pass
                    raise
