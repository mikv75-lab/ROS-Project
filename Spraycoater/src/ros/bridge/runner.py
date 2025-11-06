# Spraycoater/src/ros/bridge/runner.py
from __future__ import annotations
import threading
from typing import Optional, List, Any, Tuple, Type, TypeVar

import rclpy
from rclpy.executors import SingleThreadedExecutor

from config.startup import AppContent
from .scene_bridge import SceneBridge

T = TypeVar("T")


class RosBridge:
    """
    Betreibt Bridge-Nodes (aktuell: nur SceneBridge) in einem eigenen Executor-Thread.

    WICHTIG:
    - Executor wird LAZY NACH rclpy.init() erzeugt (Crash-Vermeidung auf manchen RMWs).
    - Alle Zugriffe auf _exec sind gegen None abgesichert.
    - stop() räumt deterministisch auf; danach ist start() erneut möglich.
    """
    def __init__(self, startup_yaml_path: str):
        self._startup_yaml_path = startup_yaml_path
        self._content = AppContent(startup_yaml_path)

        self._exec: Optional[SingleThreadedExecutor] = None  # erst in start() erzeugen
        self._nodes: List[Any] = []
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.RLock()

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._running

    @property
    def primary_node(self):
        with self._lock:
            return self._nodes[0] if self._nodes else None

    @property
    def nodes(self) -> Tuple[Any, ...]:
        with self._lock:
            return tuple(self._nodes)

    def get_node(self, cls: Type[T]) -> Optional[T]:
        with self._lock:
            for n in self._nodes:
                if isinstance(n, cls):
                    return n
        return None

    def start(self) -> None:
        with self._lock:
            if self._running:
                return

            if not rclpy.ok():
                rclpy.init(args=None)

            # Executor ERST JETZT (nach init) erzeugen
            self._exec = SingleThreadedExecutor(context=rclpy.get_default_context())

            scene = SceneBridge(self._content)
            self._nodes.append(scene)
            for n in self._nodes:
                self._exec.add_node(n)

            self._running = True
            self._thread = threading.Thread(target=self._spin, name="ros-bridge", daemon=True)
            self._thread.start()

    def _spin(self) -> None:
        try:
            if self._exec is not None:
                self._exec.spin()
        except Exception:
            pass
        finally:
            with self._lock:
                self._running = False

    def stop(self) -> None:
        with self._lock:
            # Falls bereits gestoppt, aber Thread existiert noch: sauber beenden
            if not self._running:
                if self._thread and self._thread.is_alive():
                    try:
                        if self._exec is not None:
                            self._exec.shutdown()
                    except Exception:
                        pass
                    self._thread.join(timeout=1.0)
                    self._thread = None
                # Zustand bereinigen
                self._exec = None
                return

            # Normaler Shutdown-Pfad
            try:
                if self._exec is not None:
                    self._exec.shutdown()
            except Exception:
                pass

            for n in list(self._nodes):
                try:
                    if self._exec is not None:
                        self._exec.remove_node(n)
                except Exception:
                    pass
                try:
                    n.destroy_node()
                except Exception:
                    pass
            self._nodes.clear()

            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

            self._running = False

            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.0)
            self._thread = None
            self._exec = None  # wichtig: Referenz kappen

    def add_node(self, node: Any) -> None:
        with self._lock:
            self._nodes.append(node)
            if self._running and self._exec is not None:
                try:
                    self._exec.add_node(node)
                except Exception:
                    try:
                        self._nodes.remove(node)
                    except ValueError:
                        pass
                    raise
