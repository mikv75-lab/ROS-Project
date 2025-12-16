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
    Betreibt alle Bridge-Nodes in einem eigenen Executor-Thread.
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
        self._namespace = (namespace or "").strip().strip("/")

        self._exec: Optional[SingleThreadedExecutor] = None
        self._nodes: List[Any] = []
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.RLock()

        # wichtig: nur shutdown() wenn WIR init gemacht haben
        self._did_init_rclpy = False

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
                self._did_init_rclpy = True

            self._exec = SingleThreadedExecutor(context=rclpy.get_default_context())
            ns = self._namespace

            # Bridges erzeugen (immer)
            scene = SceneBridge(self._content, namespace=ns)
            poses = PosesBridge(self._content, namespace=ns)
            spray = SprayPathBridge(self._content, namespace=ns)
            servo = ServoBridge(self._content, namespace=ns)
            robot = RobotBridge(self._content, namespace=ns)
            moveitpy = MoveItPyBridge(self._content, namespace=ns)

            self._nodes = [scene, poses, spray, servo, robot, moveitpy]

            if self._enable_omron:
                self._nodes.append(OmronBridge(self._content, namespace=ns))

            # Nodes in Executor
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
        # absichtlich keine "silent" errors: wenn das crasht, willst du es sehen
        try:
            assert self._exec is not None
            self._exec.spin()
        finally:
            with self._lock:
                self._running = False

    def stop(self) -> None:
        with self._lock:
            if not self._running and self._exec is None and not self._nodes:
                # bereits sauber gestoppt
                return

            exec_ = self._exec
            thread = self._thread
            nodes = list(self._nodes)
            did_init = self._did_init_rclpy

            # lokale Felder sofort leeren (damit re-entrancy safe ist)
            self._exec = None
            self._thread = None
            self._nodes = []
            self._running = False
            self._did_init_rclpy = False

        # 1) Executor stoppen
        if exec_ is not None:
            exec_.shutdown()

        # 2) Nodes entfernen + destroy
        if exec_ is not None:
            for n in nodes:
                exec_.remove_node(n)

        for n in nodes:
            n.destroy_node()

        # 3) Thread join
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

        # 4) rclpy shutdown nur wenn wir init gemacht haben
        if did_init and rclpy.ok():
            rclpy.shutdown()

    def add_node(self, node: Any) -> None:
        with self._lock:
            if self._exec is None:
                raise RuntimeError("RosBridge läuft nicht (Executor ist None).")

            self._nodes.append(node)
            self._exec.add_node(node)
