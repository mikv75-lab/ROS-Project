# src/ros/bridge/runner.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import threading
from typing import Optional, List, Any, Tuple, Type, TypeVar

import rclpy
from rclpy.executors import SingleThreadedExecutor

from config.startup import AppContent
from .scene_bridge import SceneBridge
from .poses_bridge import PosesBridge
from .spray_path_bridge import SprayPathBridge
from .servo_bridge import ServoBridge
from .robot_bridge import RobotBridge
from .moveitpy_bridge import MoveItPyBridge
from .omron_bridge import OmronBridge

T = TypeVar("T")


class RosBridge:
    """
    Betreibt alle Bridge-Nodes (SceneBridge, PosesBridge, SprayPathBridge,
    ServoBridge, RobotBridge, MoveItPyBridge und optional OmronBridge) in einem
    eigenen Executor-Thread.

    WICHTIG:
      - namespace wird an alle Bridge-Nodes weitergereicht, damit diese auf
        /<namespace>/... Topics arbeiten (z.B. /shadow/..., /live/...).
    """

    def __init__(
        self,
        startup_yaml_path: str,
        *,
        enable_omron: bool = False,
        namespace: str | None = None,
    ):
        self._startup_yaml_path = startup_yaml_path
        self._content = AppContent(startup_yaml_path)

        self._enable_omron = bool(enable_omron)

        # Namespace normalisieren: "shadow" statt "/shadow/"
        self._namespace = (namespace or "").strip().strip("/")

        self._exec: Optional[SingleThreadedExecutor] = None
        self._nodes: List[Any] = []
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.RLock()

    @property
    def namespace(self) -> str:
        return self._namespace

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

            self._exec = SingleThreadedExecutor(context=rclpy.get_default_context())

            ns = self._namespace  # "shadow" | "live" | ""

            # --- Bridges erzeugen (immer) ---
            scene = SceneBridge(self._content, namespace=ns)
            poses = PosesBridge(self._content, namespace=ns)
            spray = SprayPathBridge(self._content, namespace=ns)
            servo = ServoBridge(self._content, namespace=ns)
            robot = RobotBridge(self._content, namespace=ns)
            moveitpy = MoveItPyBridge(self._content, namespace=ns)

            self._nodes.extend([scene, poses, spray, servo, robot, moveitpy])

            # --- Omron nur, wenn explizit aktiviert (typisch: live) ---
            if self._enable_omron:
                omron = OmronBridge(self._content, namespace=ns)
                self._nodes.append(omron)

            # dem Executor hinzufügen
            for n in self._nodes:
                self._exec.add_node(n)

            self._running = True
            self._thread = threading.Thread(
                target=self._spin,
                name=f"ros-bridge-{ns or 'root'}",
                daemon=True,
            )
            self._thread.start()

    def _spin(self) -> None:
        try:
            if self._exec is not None:
                self._exec.spin()
        except Exception:
            # optional: logging
            pass
        finally:
            with self._lock:
                self._running = False

    def stop(self) -> None:
        with self._lock:
            if not self._running:
                if self._thread and self._thread.is_alive():
                    try:
                        if self._exec is not None:
                            self._exec.shutdown()
                    except Exception:
                        pass
                    self._thread.join(timeout=1.0)
                    self._thread = None
                self._exec = None
                return

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
            self._exec = None

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
