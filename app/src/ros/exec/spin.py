# ros/exec/spin.py
from __future__ import annotations

import threading
import time
from typing import Iterable, Optional, Sequence, Union

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor


# -------------------------------
# Variante A: Qt-Spinner (QTimer)
# -------------------------------

def start_qt_spin(node: Node, interval_ms: int = 20):
    """
    Integriert rclpy in die Qt-Eventloop:
    Erzeugt einen QTimer, der periodisch rclpy.spin_once(node, timeout=0) aufruft.
    Rückgabe: QTimer-Objekt (muss referenziert bleiben).
    """
    # Import hier drin lassen, damit PyQt optional bleibt.
    from PyQt5.QtCore import QTimer

    timer = QTimer()
    timer.setInterval(int(max(1, interval_ms)))

    def _tick():
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            # nicht crashen, minimal robust bleiben
            pass

    timer.timeout.connect(_tick)
    timer.start()
    return timer


# --------------------------------------
# Variante B: Thread-Spinner (Executor)
# --------------------------------------

class ThreadSpinner:
    """
    Spinnt einen oder mehrere Nodes in einem Hintergrundthread mit SingleThreadedExecutor.
    Benutzung:
        spinner = ThreadSpinner([bridge, optional_other_nodes], rate_hz=50)
        spinner.start()
        ...
        spinner.stop()
    """

    def __init__(self, nodes: Union[Node, Sequence[Node]], *, rate_hz: float = 50.0):
        self._nodes: Sequence[Node] = nodes if isinstance(nodes, (list, tuple)) else [nodes]
        self._rate = float(max(1.0, rate_hz))
        self._exec = SingleThreadedExecutor()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self) -> None:
        # Nodes dem Executor hinzufügen (nur einmal)
        for n in self._nodes:
            try:
                self._exec.add_node(n)
            except Exception:
                pass

        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ros-thread-spinner", daemon=True)
        self._thread.start()

    def stop(self, join_timeout: float = 1.5) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=join_timeout)
        # sauber aufräumen
        try:
            for n in self._nodes:
                self._exec.remove_node(n)
        except Exception:
            pass
        try:
            self._exec.shutdown(timeout=0.1)
        except Exception:
            pass

    def _run(self):
        period = 1.0 / self._rate
        # Wir nutzen spin_once mit kleinem Timeout, damit der Thread responsiv stoppt.
        while not self._stop.is_set():
            try:
                self._exec.spin_once(timeout_sec=period)
            except Exception:
                # Executor darf nicht sterben, kurze Pause
                time.sleep(0.01)


# -------------------------------
# Bequeme Factory-Funktionen
# -------------------------------

def spin_with_qt(node: Node, interval_ms: int = 20):
    """Alias für start_qt_spin – besserer Name für Aufrufer."""
    return start_qt_spin(node, interval_ms=interval_ms)

def spin_in_thread(nodes: Union[Node, Iterable[Node]], rate_hz: float = 50.0) -> ThreadSpinner:
    """Erzeugt und startet einen ThreadSpinner; gibt ihn zurück (zum späteren Stoppen)."""
    sp = ThreadSpinner(nodes, rate_hz=rate_hz)
    sp.start()
    return sp
